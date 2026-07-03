#include "toshiba_ab.h"
#include "esphome.h"
#include "esphome/components/network/util.h"
#include <inttypes.h>
#include <cstring>
#include <cstdlib>
#ifdef USE_ESP8266
#include <HardwareSerial.h>
#endif


namespace esphome {
namespace toshiba_ab {



static const char *const TAG = "tcc_link.climate";

#ifdef USE_ESP8266
// Optional hardware UART0 instance, used for RX only when GPIO13 RX is
// auto-detected on ESP8266 (or hardware_uart_rx_pin is configured). ESPHome
// disables the Arduino global `Serial`, so we own this instance. Only begun when
// enabled.
static HardwareSerial s_bus_serial(UART0);
#endif


const LogString *opcode_to_string(uint8_t opcode) {
  switch (opcode) {
    case OPCODE_PING:
      return LOG_STR("OPCODE_PING");
    case OPCODE_PARAMETER:
      return LOG_STR("OPCODE_PARAMETER");
    case OPCODE_ERROR_HISTORY:
      return LOG_STR("OPCODE_ERROR_HISTORY");
    case OPCODE_SENSOR_QUERY:
      return LOG_STR("OPCODE_SENSOR_QUERY");
    case OPCODE_ACK:
      return LOG_STR("OPCODE_ACK");
    case OPCODE_SENSOR_VALUE:
      return LOG_STR("OPCODE_SENSOR_VALUE");
    case OPCODE_STATUS:
      return LOG_STR("OPCODE_STATUS");
    case OPCODE_TEMPERATURE:
      return LOG_STR("OPCODE_TEMPERATURE");
    case OPCODE_EXTENDED_STATUS:
      return LOG_STR("OPCODE_EXTENDED_STATUS");
    default:
      // return LOG_STR(str_sprintf("UNKNOWN OPCODE 1: 0x%02x", opcode));
      return LOG_STR("UNKNOWN");
  }
}

uint8_t temp_celcius_to_payload(float t) {
  // raw = round( (t + offset) * ratio )
  // Guard NaN: lround(NaN) is impl-defined and cast-to-uint8 was producing
  // 0xFF, which the AC decodes as 92.5°C and the wall remote displays as 92.
  // Substitute 22°C as a safe fallback before any encoding.
  if (std::isnan(t)) {
    t = 22.0f;
  }
  const float scaled = (t + TEMPERATURE_CONVERSION_OFFSET) * TEMPERATURE_CONVERSION_RATIO;
  int v = static_cast<int>(std::lround(scaled));
  if (v < 0) v = 0;
  if (v > 255) v = 255;
  return static_cast<uint8_t>(v);
}

uint8_t get_heat_cool_bits(uint8_t mode) {
  // TODO: figure the bits out:
  // https://github.com/issalig/toshiba_air_cond/blob/master/air/toshiba_serial.hpp#L285
  switch (mode) {
    case MODE_HEAT:
    case MODE_AUTO:
      return 0x55;  // 0b01010101  // = 0x01 + 0x04 * air->heat + 0x02 * air->cold;
    case MODE_COOL:
    case MODE_DRY:
    case MODE_FAN_ONLY:
      return 0x33;  // 0b00110011  // = 0x01 + 0x04 * air->heat + 0x02 * air->cold;
  }

  return 0;
}

uint8_t get_fan_bit_mask_for_mode(uint8_t mode) {
  switch (mode) {
    case MODE_HEAT:
    case MODE_AUTO:
      return 0b00100000 | 0b00001000;
    case MODE_COOL:
    case MODE_DRY:
    case MODE_FAN_ONLY:
      return 0b00010000 | 0b00001000;
  }

  return 0;
}

uint8_t calculate_tu2c_set_parameter_flags_crc(const uint8_t *data, size_t size) {
  constexpr uint8_t crc_seed = 0xB8;
  uint16_t sum = crc_seed;
  for (size_t i = 0; i < size; i++) {
    sum += data[i];
  }
  return static_cast<uint8_t>(sum & 0xFF);
}

uint8_t calculate_tu2c_power_off_crc(const uint8_t *data, size_t size) {
  // Based on observed TU2C power-off frame:
  // 0B:50:90:03:01:21:02:CF
  constexpr uint8_t crc_seed = 0xBD;
  uint16_t sum = crc_seed;
  for (size_t i = 0; i < size; i++) {
    sum += data[i];
  }
  return static_cast<uint8_t>(sum & 0xFF);
}

uint8_t calculate_tu2c_registration_query_crc(const uint8_t *data, size_t size) {
  // Observed for short query frames like 0A:50:90:02:49:0D:00
  // CRC = (sum(bytes) - 0x42) & 0xFF
  constexpr uint8_t crc_seed = 0xBE;
  uint16_t sum = crc_seed;
  for (size_t i = 0; i < size; i++) {
    sum += data[i];
  }
  return static_cast<uint8_t>(sum & 0xFF);
}

uint8_t calculate_tu2c_keepalive_crc(const uint8_t *data, size_t size) {
  // Observed for keepalive frames like 0C:50:90:04:41:5C:90:F3:CC
  constexpr uint8_t crc_seed = 0xBC;
  uint16_t sum = crc_seed;
  for (size_t i = 0; i < size; i++) {
    sum += data[i];
  }
  return static_cast<uint8_t>(sum & 0xFF);
}

bool ToshibaAbClimate::is_own_tx_echo_(const DataFrame *f) const { // used to filter out echo from our last command sent
  if (!this->last_tx_frame_for_echo_.has_value() || f == nullptr) return false;
  if ((millis() - this->last_tx_frame_millis_) > ECHO_MATCH_WINDOW_MS) return false;
  const auto &tx = this->last_tx_frame_for_echo_.value();  // last frame we wrote
  if (f->size() == tx.size() && std::memcmp(f->raw, tx.raw, f->size()) == 0) return true;

  // First-generation Estia uses the TU2C reader path. On some adapters the
  // local TX can be echoed back with slightly different wrapper/timing state,
  // so fall back to dropping recent frames that still have our Estia source,
  // current master destination and known local command/query markers.
  if (this->data_reader.frame_format() == FrameFormat::ESTIA && f->is_tu2c() && f->size() >= 6 &&
      f->raw[1] == this->remote_address_ && f->raw[2] == this->master_address_ &&
      f->raw[3] == 0xE0 && (f->raw[4] == 0x01 || f->raw[4] == 0x41)) {
    return true;
  }

  return false;
}

void ToshibaAbClimate::remember_tx_frame_for_echo_(const uint8_t *bytes, size_t size, bool tu2c) {
  if (bytes == nullptr || size == 0 || size > DATA_FRAME_MAX_SIZE + 3) {
    return;
  }

  const uint8_t *payload = bytes;
  size_t payload_size = size;
  bool payload_tu2c = tu2c;

  if (size >= 4 && bytes[0] == 0xF0 && bytes[1] == 0xF0 && bytes[size - 1] == 0xA0) {
    payload = bytes + 2;
    payload_size = size - 3;
    payload_tu2c = true;
  }

  if (payload_size == 0 || payload_size > DATA_FRAME_MAX_SIZE) {
    return;
  }

  DataFrame tx{};
  std::memcpy(tx.raw, payload, payload_size);
  if (tx.size() == 0 || tx.size() != payload_size) {
    return;
  }

  tx.set_tu2c(payload_tu2c);
  this->last_tx_frame_for_echo_ = tx;
  this->last_tx_frame_millis_ = millis();
}

void ToshibaAbClimate::update_frame_validation_() {
  const bool allow_same = this->autonomous_ && !this->announce_ack_received_;
  this->data_reader.set_allow_same_source_dest(allow_same);
}

void ToshibaAbClimate::confirm_frame_format_(FrameFormat format, const char *reason) {
  if (!this->should_auto_detect_frame_format_()) {
    return;
  }
  this->data_reader.set_frame_format(format);
  this->frame_format_confirmed_ = true;
  const char *fmt = "normal";
  switch (format) {
    case FrameFormat::TU2C: fmt = "tu2c"; break;
    case FrameFormat::A0: fmt = "a0"; break;
    case FrameFormat::HM: fmt = "hm"; break;
    case FrameFormat::ESTIA: fmt = "estia"; break;
    default: fmt = "n"; break;
  }
  ESP_LOGI(TAG, "Auto-detected frame_format=%s (%s)", fmt, reason != nullptr ? reason : "confirmed frame");
}

void ToshibaAbClimate::watch_auto_frame_format_byte_(uint8_t byte) {
  if (!this->should_auto_detect_frame_format_()) {
    return;
  }

  constexpr uint32_t AUTO_A0_BYTE_TIMEOUT_MS = 50;
  if (this->auto_a0_collecting_ && (millis() - this->auto_a0_last_byte_ms_) > AUTO_A0_BYTE_TIMEOUT_MS) {
    this->auto_a0_collecting_ = false;
    this->auto_a0_prefix_match_ = 0;
    this->auto_a0_index_ = 0;
    this->auto_a0_expected_total_ = 0;
  }

  if (!this->auto_a0_collecting_) {
    if (this->auto_a0_prefix_match_ == 1) {
      if (byte == 0x00) {
        this->auto_a0_collecting_ = true;
        this->auto_a0_buffer_[0] = 0xA0;
        this->auto_a0_buffer_[1] = 0x00;
        this->auto_a0_index_ = 2;
        this->auto_a0_expected_total_ = 0;
        this->auto_a0_last_byte_ms_ = millis();
      }
      this->auto_a0_prefix_match_ = 0;
      return;
    }
    if (byte == 0xA0) {
      this->auto_a0_prefix_match_ = 1;
    }
    return;
  }

  this->auto_a0_last_byte_ms_ = millis();
  if (this->auto_a0_index_ >= sizeof(this->auto_a0_buffer_)) {
    this->auto_a0_collecting_ = false;
    this->auto_a0_index_ = 0;
    this->auto_a0_expected_total_ = 0;
    return;
  }

  this->auto_a0_buffer_[this->auto_a0_index_++] = byte;
  if (this->auto_a0_index_ == 4) {
    const uint8_t len = this->auto_a0_buffer_[3];
    this->auto_a0_expected_total_ = len + 6;  // A0:00 prefix + type + len + len-payload + CRC16
    if (this->auto_a0_expected_total_ < 8 || this->auto_a0_expected_total_ > sizeof(this->auto_a0_buffer_)) {
      this->auto_a0_collecting_ = false;
      this->auto_a0_index_ = 0;
      this->auto_a0_expected_total_ = 0;
      return;
    }
  }

  if (this->auto_a0_expected_total_ > 0 && this->auto_a0_index_ >= this->auto_a0_expected_total_) {
    DataFrame candidate{};
    const uint16_t stored = this->auto_a0_expected_total_ - 2;
    for (uint16_t i = 0; i < stored && i < DATA_FRAME_MAX_SIZE; i++) {
      candidate.raw[i] = this->auto_a0_buffer_[i + 2];
    }
    candidate.set_estia(true);
    candidate.set_estia_len(this->auto_a0_buffer_[3]);
    const uint16_t src = (stored > 4) ? ((candidate.raw[3] << 8) | candidate.raw[4]) : 0;
    if (candidate.validate_estia_crc() && src == 0x0800) {
      this->confirm_frame_format_(FrameFormat::A0, "valid A0 frame from master");
    }
    this->auto_a0_collecting_ = false;
    this->auto_a0_index_ = 0;
    this->auto_a0_expected_total_ = 0;
  }
}

bool ToshibaAbClimate::is_hm_wire_frame_from_master_(const DataFrame *frame) const {
  if (frame == nullptr || frame->raw[0] != 0xA0 || frame->raw[1] != 0x00 || frame->data_length < 7) {
    return false;
  }
  const size_t total = frame->size();
  if (total < 12 || frame->raw[4] != 0x00 || frame->raw[9] != 0x00) {
    return false;
  }
  const uint8_t src = frame->raw[6];
  if (src == this->master_address_) {
    return true;
  }
  return this->master_address_auto_ && this->master_address_ == 0x00 && src == 0x00;
}

bool ToshibaAbClimate::maybe_auto_detect_hm_frame_(DataFrame *frame) {
  if (!this->should_auto_detect_frame_format_() || !this->is_hm_wire_frame_from_master_(frame)) {
    return false;
  }
  const uint8_t detected_master = frame->raw[6];
  this->confirm_frame_format_(FrameFormat::HM, "valid HM frame from master");
  if (this->master_address_auto_ && detected_master != this->master_address_) {
    this->master_address_ = detected_master;
  }
  return DataFrameReader::canonicalize_hm_frame(frame, frame->size());
}

bool ToshibaAbClimate::has_bus_quiet_time_elapsed_(uint32_t now) const {
  return (now - this->last_received_frame_millis_) >= FRAME_SEND_MILLIS_FROM_LAST_RECEIVE &&
         (now - this->last_sent_frame_millis_) >= FRAME_SEND_MILLIS_FROM_LAST_SEND;
}

bool ToshibaAbClimate::has_tu2c_quiet_time_elapsed_(uint32_t now) const {
  return (now - this->last_tu2c_received_frame_millis_) >= TU2C_FRAME_SEND_MILLIS_FROM_LAST_RECEIVE &&
         (now - this->last_tu2c_sent_frame_millis_) >= TU2C_FRAME_SEND_MILLIS_FROM_LAST_SEND;
}


bool ToshibaAbClimate::is_ack_for_pending_command_(const DataFrame *frame) const {
  if (frame == nullptr || !this->last_unconfirmed_command_.has_value()) {
    return false;
  }
  const auto &pending = this->last_unconfirmed_command_.value();

  if (frame->is_tu2c() || pending.is_tu2c()) {
    if (!frame->is_tu2c() || !pending.is_tu2c()) {
      return false;
    }

    if (frame->raw[1] != pending.raw[2] || frame->raw[2] != pending.raw[1]) {
      return false;
    }

    return this->is_tu2c_command_ack_frame_(frame);
  }

  if (frame->source != pending.dest || frame->dest != pending.source) {
    return false;
  }

  if (frame->opcode1 != OPCODE_ACK || frame->data_length < 2) {
    return false;
  }

  return frame->data[0] == this->command_mode_write_ && frame->data[1] == OPCODE2_PARAM_ACK;
}

bool ToshibaAbClimate::is_tu2c_command_ack_frame_(const DataFrame *frame) const {
  if (frame == nullptr || !frame->is_tu2c()) {
    return false;
  }

  const size_t size = frame->size();
  if (size < 7) {
    return false;
  }

  return frame->raw[0] == 0x0A && frame->raw[4] == 0x80 && frame->raw[5] == 0xA1;
}

bool ToshibaAbClimate::should_track_tu2c_command_ack_(const DataFrame &frame) const {
  if (!frame.is_tu2c() || frame.size() < 7) {
    return false;
  }

  const bool matches_tu2c_addresses = frame.raw[1] == this->tu2c_remote_address_ && frame.raw[2] == this->tu2c_master_address_;
  const bool matches_estia_addresses = this->data_reader.frame_format() == FrameFormat::ESTIA &&
                                      frame.raw[1] == this->remote_address_ && frame.raw[2] == this->master_address_;
  if (!matches_tu2c_addresses && !matches_estia_addresses) {
    return false;
  }

  if (frame.raw[4] != 0x01) {
    return false;
  }

  // Track ACKs for TU2C/first-gen Estia write commands.
  return frame.raw[5] == 0x21 || frame.raw[5] == 0x23 || frame.raw[5] == 0x2C;
}

bool ToshibaAbClimate::should_track_command_ack_(const DataFrame &frame) const {
  if (frame.is_tu2c()) {
    return this->should_track_tu2c_command_ack_(frame);
  }

  if (frame.opcode1 != OPCODE_PARAMETER || frame.data_length < 2) {
    return false;
  }

  return frame.data[1] == OPCODE2_SET_MODE || frame.data[1] == OPCODE2_SET_TEMP_WITH_FAN;
}

void ToshibaAbClimate::handle_pending_command_ack_(const DataFrame *frame) {
  if (!this->last_unconfirmed_command_.has_value()) {
    return;
  }

  if (this->is_ack_for_pending_command_(frame)) {
    ESP_LOGD(TAG, "Command ACK received for opcode 0x%02X after %u attempt(s)",
             this->last_unconfirmed_command_->opcode1,
             static_cast<unsigned>(this->last_unconfirmed_command_attempts_));
    this->last_unconfirmed_command_.reset();
    this->last_unconfirmed_command_attempts_ = 0;
    this->resend_last_unconfirmed_command_ = false;
    this->last_unconfirmed_command_sent_ms_ = 0;
    return;
  }

  // TU2C retries are timeout-driven. Keep legacy immediate retry behaviour
  // for non-TU2C traffic unchanged.
  if (!this->last_unconfirmed_command_->is_tu2c()) {
    this->resend_last_unconfirmed_command_ = true;
  }
}

void log_data_frame(const std::string &msg, const struct DataFrame *frame, size_t length = 0) {
  std::string res;
  char buf[8];

  size_t len = length > 0 ? length : frame->data_length;

  for (size_t i = 0; i < len; i++) {
    if (i > 0) {
      res += ':';
    }

    snprintf(buf, sizeof(buf), "%02X", frame->data[i]);

    if (i == 1) {
      // data[1] in yellow
      res += "\033[33m";
      res += buf;
      res += "\033[0m";
    } else {
      // remaining data bytes in dim grey
      res += "\033[2;37m";
      res += buf;
      res += "\033[0m";
    }
  }

  ESP_LOGD(
    "RX",
    "%s: \033[31m%02X\033[0m:\033[31m%02X\033[0m:\033[32m%02X\033[0m:%02X:%s:%02X",
    msg.c_str(),
    frame->source,
    frame->dest,
    frame->opcode1,
    frame->data_length,
    res.c_str(),
    frame->crc()
  );
}

void log_tu2c_data_frame(const std::string &msg, const struct DataFrame *frame) {
  if (frame == nullptr) return;
  const size_t size = frame->size();
  std::string res;
  res.reserve(size ? (size * 12) : 0);
  char buf[4];
  for (size_t i = 0; i < size; i++) {
    if (i > 0) res += ':';
    std::snprintf(buf, sizeof(buf), "%02X", frame->raw[i]);
    const bool highlight = i == 1 || i == 2 || i == 3 || i == 4 || i == 5;
    if (highlight) {
      res += (i == 1 || i == 2) ? "\033[31m" : "\033[32m";
      res += buf;
      res += "\033[0m";
    } else {
      res += "\033[2;37m";
      res += buf;
      res += "\033[0m";
    }
  }
  ESP_LOGD("RX", "%s: %s", msg.c_str(), res.c_str());
}

void log_raw_data(const std::string& prefix, const uint8_t raw[], size_t size) {
  std::string res;
  res.reserve(size ? (size * 3 - 1) : 0);  // pre-size: "AA:" per byte minus last colon
  char buf[3];
  for (size_t i = 0; i < size; i++) {
    if (i > 0) res += ':';
    std::snprintf(buf, sizeof(buf), "%02X", raw[i]);
    res += buf;
  }
  ESP_LOGV("RX", "%s: %s", prefix.c_str(), res.c_str());
}

std::string frame_to_hex_string(const DataFrame *frame) {
  if (frame == nullptr) {
    return "";
  }

  const size_t size = frame->size();
  std::string payload;
  payload.reserve(size ? (size * 3 - 1) : 0);

  for (size_t i = 0; i < size; i++) {
    if (i > 0) {
      payload += ':';
    }
    char buf[3];
    std::snprintf(buf, sizeof(buf), "%02X", frame->raw[i]);
    payload += buf;
  }

  return payload;
}


void write_set_parameter(struct DataFrame *command, uint8_t remote_address, uint8_t master_address, uint8_t command_mode_read, uint8_t opcode2,
                         uint8_t payload[], size_t payload_size) {
  command->source = remote_address;
  command->dest = master_address;
  command->opcode1 = OPCODE_PARAMETER;
  command->data_length = SET_PARAMETER_PAYLOAD_HEADER_SIZE + payload_size;
  command->data[0] = command_mode_read;
  command->data[1] = opcode2;

  for (size_t i = 0; i < payload_size; i++) {
    command->data[SET_PARAMETER_PAYLOAD_HEADER_SIZE + i] = payload[i];
  }

  command->data[SET_PARAMETER_PAYLOAD_HEADER_SIZE + payload_size] = command->calculate_crc();
}

void write_set_temperature(struct DataFrame *command, uint8_t remote_address, uint8_t master_address, uint8_t command_mode_read, uint8_t opcode2,
                           uint8_t payload[], size_t payload_size) {
  command->source = remote_address;
  command->dest = master_address;
  command->opcode1 = OPCODE_TEMPERATURE;
  command->data_length = SET_PARAMETER_PAYLOAD_HEADER_SIZE + payload_size;
  command->data[0] = command_mode_read;
  command->data[1] = opcode2;

  for (size_t i = 0; i < payload_size; i++) {
    command->data[SET_PARAMETER_PAYLOAD_HEADER_SIZE + i] = payload[i];
  }

  command->data[SET_PARAMETER_PAYLOAD_HEADER_SIZE + payload_size] = command->calculate_crc();
}


void write_set_parameter(struct DataFrame *command, uint8_t remote_address, uint8_t master_address, uint8_t command_mode_read, uint8_t opcode2,
                         uint8_t single_type_payload) {
  uint8_t payload[1] = {single_type_payload};
  write_set_parameter(command, remote_address, master_address, command_mode_read, opcode2, payload, 1);
}

void write_set_parameter_flags(struct DataFrame *command, uint8_t remote_address, uint8_t master_address, uint8_t command_mode_read,
                               const struct TccState *state, uint8_t set_flags, bool hm_variant) {
  // The first six payload bytes are identical in both variants. The HM
  // variant appends three trailing zero bytes that the RBC-ASCU11-E wired
  // remote sends; without them, the AC controller silently rejects the
  // command. Captured form for HM:
  //   4C : <mode|flags> : <fan|mask> : <temp> : 00 : 33 : 33 : 00 : 00 : 00
  if (hm_variant) {
    uint8_t payload[9] = {
        static_cast<uint8_t>(state->mode | set_flags),
        static_cast<uint8_t>(state->fan | get_fan_bit_mask_for_mode(state->mode)),
        temp_celcius_to_payload(state->target_temp),
        EMPTY_DATA,
        get_heat_cool_bits(state->mode),
        get_heat_cool_bits(state->mode),
        EMPTY_DATA,
        EMPTY_DATA,
        EMPTY_DATA,
    };
    write_set_parameter(command, remote_address, master_address, command_mode_read, OPCODE2_SET_TEMP_WITH_FAN, payload, sizeof(payload));
  } else {
    uint8_t payload[6] = {
        static_cast<uint8_t>(state->mode | set_flags),
        static_cast<uint8_t>(state->fan | get_fan_bit_mask_for_mode(state->mode)),
        temp_celcius_to_payload(state->target_temp),
        EMPTY_DATA,
        get_heat_cool_bits(state->mode),
        get_heat_cool_bits(state->mode),
    };
    write_set_parameter(command, remote_address, master_address, command_mode_read, OPCODE2_SET_TEMP_WITH_FAN, payload, sizeof(payload));
  }
}

void write_set_parameter_flags_tu2c(struct DataFrame *command, uint8_t remote_address, uint8_t master_address,
                                       const struct TccState *state, uint8_t set_flags) {
  constexpr uint8_t payload_length = 0x08;
  constexpr uint8_t tu2c_length = 0x10;
  constexpr uint8_t tu2c_header_marker_msb = 0x01;
  constexpr uint8_t tu2c_header_marker_lsb = 0x2C;

  command->set_tu2c(true);
  command->raw[0] = tu2c_length;
  command->raw[1] = remote_address;
  command->raw[2] = master_address;
  command->raw[3] = payload_length;
  command->raw[4] = tu2c_header_marker_msb;
  command->raw[5] = tu2c_header_marker_lsb;
  command->raw[6] = static_cast<uint8_t>(state->mode | set_flags);
  command->raw[7] = static_cast<uint8_t>(state->fan | get_fan_bit_mask_for_mode(state->mode));
  command->raw[8] = temp_celcius_to_payload(state->target_temp);
  command->raw[9] = EMPTY_DATA;
  command->raw[10] = get_heat_cool_bits(state->mode);
  command->raw[11] = get_heat_cool_bits(state->mode);

  command->raw[12] = calculate_tu2c_set_parameter_flags_crc(command->raw, 12);
}

void write_power_off_tu2c(struct DataFrame *command, uint8_t remote_address, uint8_t master_address) {
  constexpr uint8_t tu2c_length = 0x0B;
  constexpr uint8_t payload_length = 0x03;
  constexpr uint8_t marker_msb = 0x01;
  constexpr uint8_t marker_lsb = 0x21;
  constexpr uint8_t power_off_code = 0x02;

  command->set_tu2c(true);
  command->raw[0] = tu2c_length;
  command->raw[1] = remote_address;
  command->raw[2] = master_address;
  command->raw[3] = payload_length;
  command->raw[4] = marker_msb;
  command->raw[5] = marker_lsb;
  command->raw[6] = power_off_code;
  command->raw[7] = calculate_tu2c_power_off_crc(command->raw, 7);
}

void write_set_parameter_mode(struct DataFrame *command, uint8_t remote_address, uint8_t master_address, uint8_t command_mode_read,
                              const struct TccState *state) {
  write_set_parameter(command, remote_address, master_address, command_mode_read, OPCODE2_SET_MODE, state->mode);
}

void write_set_parameter_power(struct DataFrame *command, uint8_t remote_address, uint8_t master_address, uint8_t command_mode_read,
                               const struct TccState *state) {
  write_set_parameter(command, remote_address, master_address, command_mode_read, OPCODE2_SET_POWER, state->power | 0b0010);
}

void write_set_parameter_vent(struct DataFrame *command, uint8_t remote_address, uint8_t master_address, uint8_t command_mode_read,
                              const struct TccState *state) {
  write_set_parameter(command, remote_address, master_address, command_mode_read, OPCODE2_SET_VENT, state->vent);
}

void write_set_parameter_room_temp(struct DataFrame *command, uint8_t remote_address, uint8_t master_address, uint8_t command_mode_read,
                                   float temperature) {
  // Clamp temperature to a safe range (adjust if needed by protocol)
  float clamped = std::max(0.0f, std::min(temperature, 40.0f));

  // Round to nearest 0.5°C granularity
  float rounded = std::round(clamped * 2.0f) / 2.0f;

  uint8_t room_temp [2] = {0x00, temp_celcius_to_payload(rounded)};

  // Send using existing write_set_temperature
  write_set_temperature(command, remote_address, master_address, command_mode_read, OPCODE2_SENSOR_ROOM_TEMP, room_temp, sizeof(room_temp));
}

void write_read_envelope(DataFrame *cmd, uint8_t remote_address, uint8_t master_address,
                         uint8_t command_mode_read, uint8_t opcode2, const uint8_t payload[], size_t payload_size) {
// Writes a read envelope command
// This is used to read some data from the master unit, but not sensors
// And to send remote PING/KEEP_ALIVE commands
  cmd->source      = remote_address;
  cmd->dest        = master_address;
  cmd->opcode1     = OPCODE_ERROR_HISTORY;             // 0x15
  cmd->data_length = SET_PARAMETER_PAYLOAD_HEADER_SIZE + payload_size;
  cmd->data[0]     = command_mode_read;
  cmd->data[1]     = opcode2;                          // e.g., OPCODE2_PING_PONG (0x0C)

  for (size_t i = 0; i < payload_size; i++)
    cmd->data[SET_PARAMETER_PAYLOAD_HEADER_SIZE + i] = payload[i];

  cmd->data[SET_PARAMETER_PAYLOAD_HEADER_SIZE + payload_size] = cmd->calculate_crc();
}


// Sends room temp to AC unit with the sensor configured in yaml.
// Uses a dedicated source address offset from the runtime remote id
// (remote+1, capped at 0x49) to avoid colliding with normal remote traffic.
// Example: if remote id is 0x41, this frame source becomes 0x42.
void ToshibaAbClimate::send_remote_temp(float temp_c) {
  // sanity
  if (!std::isfinite(temp_c) || temp_c < -40.0f || temp_c > 80.0f) {
    ESP_LOGW(TAG, "send_remote_temp: invalid temp %.2f°C", temp_c);
    return;
  }

  // Encode raw = (C + OFFSET) * RATIO, mask per protocol
  const uint8_t raw = static_cast<uint8_t>(
      std::lround((temp_c + TEMPERATURE_CONVERSION_OFFSET) * TEMPERATURE_CONVERSION_RATIO));
  const float rounded_temp_c = static_cast<float>(raw) / TEMPERATURE_CONVERSION_RATIO - TEMPERATURE_CONVERSION_OFFSET;
  const uint8_t temp_source = std::min<uint8_t>(this->remote_address_ + 1, TOSHIBA_REMOTE_MAX);


  DataFrame cmd{};
  cmd.source      = temp_source;
  cmd.dest        = this->master_address_;  // usually 0x00
  cmd.opcode1     = OPCODE_PARAMETER;       // 0x11
  cmd.data_length = 4;                       // payload: 08 89 <raw> 46

  cmd.data[0] = this->command_mode_read_;
  cmd.data[1] = 0x89;
  cmd.data[2] = raw;   // encoded temperature
  cmd.data[3] = 0x46;


  cmd.data[cmd.data_length] = cmd.calculate_crc();

  ESP_LOGD(TAG, "Sending sensor temperature: %.1f°C", rounded_temp_c);
  this->send_command(cmd);  // enqueue; loop() sends when bus is idle
}



void ToshibaAbClimate::send_ping() { // Sends a PING command to the master unit, command goes into queue, sent when loop() finds it
  // typical: 40:00:15:07:08:0C:81:00:00:48:00:9F
  // where tail = 0x81, 0x00, 0x00, 0x48, 0x00  (matches the reference ping)
  DataFrame cmd{};

  const uint8_t tail[] = { OPCODE2_READ_STATUS, 0x00, 0x00, 0x48, 0x00 };
  write_read_envelope(&cmd, this->remote_address_, this->master_address_, this->command_mode_read_, OPCODE2_PING_PONG, tail, sizeof(tail));
  this->send_command(cmd);
}

void ToshibaAbClimate::send_read_block(uint8_t opcode2, uint16_t start, uint16_t length) {  //used to read a block of data from the master unit
  // 40 00 15 06 08 E8 00 01 00 9E 2C Seems to be sent every 1 min from remote, master answers with some sort of hour counter
  //  this function is also used to construct this frame if in autonomous mode
  DataFrame cmd{};

  // Tail is big-endian: start_hi, start_lo, len_hi, len_lo
  const uint8_t tail[4] = {
    static_cast<uint8_t>((start  >> 8) & 0xFF),
    static_cast<uint8_t>( start        & 0xFF),
    static_cast<uint8_t>((length >> 8) & 0xFF),
    static_cast<uint8_t>( length       & 0xFF),
  };

  write_read_envelope(&cmd, this->remote_address_, this->master_address_, this->command_mode_read_, opcode2, tail, sizeof(tail));
  this->send_command(cmd);  // enqueue; loop() will transmit when bus is idle
}

void ToshibaAbClimate::remote_announce() {
  // Build and enqueue a short announce/envelope frame from the remote.
  // Example format: 40:F0:15:02:00:0D:AA -> source=0x40, opcode=0x15, len=2, data={0x00,0x0D}
  DataFrame cmd{};
  cmd.source = this->remote_address_;
  cmd.dest = TOSHIBA_BROADCAST;
  cmd.opcode1 = OPCODE_ERROR_HISTORY; // 0x15
  cmd.data_length = 2;
  cmd.data[0] = 0x00;
  cmd.data[1] = 0x0D;
  cmd.data[cmd.data_length] = cmd.calculate_crc();
  ESP_LOGV(TAG, "remote_announce: enqueuing announce to broadcast (0x%02X)", TOSHIBA_BROADCAST);
  this->send_command(cmd);
}

void ToshibaAbClimate::tu2c_remote_announce() {
  DataFrame cmd{};
  const uint8_t source = this->tu2c_remote_address_;
  const uint8_t dest = this->tu2c_master_address_;

  cmd.set_tu2c(true);
  cmd.raw[0] = 0x0A;
  cmd.raw[1] = source;
  cmd.raw[2] = dest;
  cmd.raw[3] = 0x02;
  cmd.raw[4] = 0x49;
  cmd.raw[5] = 0x0D;
  cmd.raw[6] = calculate_tu2c_registration_query_crc(cmd.raw, 6);

  this->send_command(cmd);
}

void ToshibaAbClimate::tu2c_send_ping() {
  DataFrame cmd{};
  const uint8_t source = this->tu2c_remote_address_;
  const uint8_t dest = this->tu2c_master_address_;

  cmd.set_tu2c(true);
  cmd.raw[0] = 0x0C;
  cmd.raw[1] = source;
  cmd.raw[2] = dest;
  cmd.raw[3] = 0x04;
  cmd.raw[4] = 0x41;
  cmd.raw[5] = 0x5C;
  cmd.raw[6] = 0x90;
  cmd.raw[7] = 0xF3;
  cmd.raw[8] = calculate_tu2c_keepalive_crc(cmd.raw, 8);

  this->send_command(cmd);
}

bool ToshibaAbClimate::is_tu2c_registration_ack_(const DataFrame *frame) const {
  if (frame == nullptr || !frame->is_tu2c()) {
    return false;
  }

  const size_t size = frame->size();
  if (size < 7) {
    return false;
  }

  return frame->raw[1] == this->tu2c_master_address_ && frame->raw[4] == 0x80 && frame->raw[5] == 0x0D;
}

bool ToshibaAbClimate::is_tu2c_registration_query_(const DataFrame &frame) const {
  if (!frame.is_tu2c() || frame.size() < 7) {
    return false;
  }
  return frame.raw[0] == 0x0A && frame.raw[1] == this->tu2c_remote_address_ && frame.raw[2] == this->tu2c_master_address_ &&
         frame.raw[4] == 0x49 && frame.raw[5] == 0x0D;
}

void ToshibaAbClimate::add_polled_sensor(uint8_t id, float scale, uint32_t interval_ms, sensor::Sensor *sensor) {
  const size_t sensor_index = this->polled_sensors_.size();
  PolledSensor ps{ id, scale, interval_ms, sensor };
  this->polled_sensors_.push_back(ps);

  // Schedule this sensor's periodic query.
  // The periodic interval just appends the sensor ID to the pending queue;
  // the actual 0x17 frame is dispatched serially from drain_sensor_query_queue_()
  // so that short-form 0x1A replies (which carry no sensor ID) can be
  // attributed unambiguously to the most recently sent query.
  //
  // Do not start all pollers immediately after boot: ESP8266 Wi-Fi/API
  // reconnect plus software-serial RX is watchdog-sensitive. Start polling
  // after a grace period and stagger each sensor so the first burst cannot
  // swamp the bus or the API connection.
  if (interval_ms > 0) {
    const uint32_t first_poll_delay_ms = 15000 + static_cast<uint32_t>(sensor_index) * 2000;
    this->set_timeout(first_poll_delay_ms, [this, id, interval_ms]() {
      this->enqueue_sensor_query_(id);
      this->set_interval(interval_ms, [this, id]() { this->enqueue_sensor_query_(id); });
    });
  }
  
    // Ensure the read-only switch reports its initial state to Home Assistant
    if (this->read_only_switch_)
      this->read_only_switch_->publish_state(this->read_only_);
}

bool ToshibaAbClimate::enqueue_sensor_query_(uint8_t id) {
  if (this->pending_count_ >= MAX_PENDING_SENSOR_QUERIES) {
    ESP_LOGW(TAG, "Sensor query queue full; dropping query for 0x%02X", id);
    return false;
  }
  // Skip if this sensor is already pending — the next interval will retry.
  constexpr uint8_t mask = MAX_PENDING_SENSOR_QUERIES - 1;
  for (uint8_t i = 0; i < this->pending_count_; i++) {
    if (this->pending_sensor_queries_[(this->pending_head_ + i) & mask] == id) return false;
  }
  const uint8_t tail = (this->pending_head_ + this->pending_count_) & mask;
  this->pending_sensor_queries_[tail] = id;
  this->pending_count_++;
  return true;
}

void ToshibaAbClimate::send_sensor_query(uint8_t sensor_id) {
  if (this->data_reader.frame_format() == FrameFormat::ESTIA) {
    this->last_sensor_query_id_ = sensor_id;
    this->sensor_query_outstanding_ = true;
    this->last_sensor_query_ms_ = millis();
    this->send_estia_first_gen_request_data(sensor_id);
    return;
  }

  DataFrame cmd{};
  cmd.source      = this->remote_address_;           // 0x40
  cmd.dest        = this->master_address_;    // usually 0x00
  cmd.opcode1     = OPCODE_SENSOR_QUERY;      // 0x17
  cmd.data_length = 8;

  // Payload (common pattern observed in this family):
  // 08 80 EF 00 2C 08 00 <id>
  cmd.data[0] = this->command_mode_read_;   // READ
  cmd.data[1] = 0x80;
  cmd.data[2] = 0xEF;
  cmd.data[3] = 0x00;
  cmd.data[4] = 0x2C;   // sensor/value table marker
  cmd.data[5] = this->command_mode_read_;
  cmd.data[6] = 0x00;
  cmd.data[7] = sensor_id;
  
  cmd.data[cmd.data_length] = cmd.calculate_crc();
  this->last_sensor_query_id_ = sensor_id; // <-- for short replies
  this->sensor_query_outstanding_ = true;
  this->last_sensor_query_ms_     = millis();  // timestamp for timeout handling


  this->send_command(cmd);  // enqueue; loop() will transmit when idle
}


void ToshibaAbClimate::drain_sensor_query_queue_() {
  // sensor_query_outstanding_ is cleared by process_sensor_value_() on a
  // matching reply, or by the sensor-query timeout watchdog if a reply never
  // arrives — so a stuck query never deadlocks the queue.
  if (this->sensor_query_outstanding_) return;
  if (this->pending_count_ == 0) return;
  if (this->write_queue_.size() >= WRITE_QUEUE_THROTTLE) return;

  const uint8_t next = this->pending_sensor_queries_[this->pending_head_];
  this->pending_head_ = (this->pending_head_ + 1) & (MAX_PENDING_SENSOR_QUERIES - 1);
  this->pending_count_--;
  this->send_sensor_query(next);
}


void ToshibaAbClimate::process_sensor_value_(const DataFrame *frame) {
  if (frame == nullptr) {
    // Always release the guard even if frame is null
    this->sensor_query_outstanding_ = false;
    this->last_sensor_query_id_     = 0xFF;
    return;
  }

  const uint8_t prev_id = this->last_sensor_query_id_;  // remember for logging

  // Only accept replies from the indoor/master unit
  if (frame->source != this->master_address_) {
    ESP_LOGW(TAG, "0x1A from 0x%02X ignored (expected master=0x%02X), last_id=0x%02X",
             frame->source, this->master_address_, prev_id);
    this->sensor_query_outstanding_ = false;
    this->last_sensor_query_id_     = 0xFF;
    return;
  }

  // All observed 0x1A replies start with: 80 EF ...
  if (frame->data_length < 5 || frame->data[0] != this->command_mode_write_ || frame->data[1] != 0xEF) {
    ESP_LOGW(TAG, "0x1A unrecognized header (len=%u), last_id=0x%02X",
             static_cast<unsigned>(frame->data_length), prev_id);
    log_raw_data("0x1A unrecognized (bad header)", frame->raw, frame->size());
    this->sensor_query_outstanding_ = false;
    this->last_sensor_query_id_     = 0xFF;
    return;
  }

  // -------------------------
  // A2 tag (undefined / no value):
  // payload = 80 EF 80 00 A2                     (len == 5)
  // -------------------------
  if (frame->data_length == 5 &&
      frame->data[2] == this->command_mode_write_ && frame->data[3] == 0x00 && frame->data[4] == 0xA2) {

    ESP_LOGW(TAG, "0x1A: sensor id=0x%02X returned A2 (undefined/not supported).", prev_id);
    // Treat as a completed (but empty) reply so the next poll can proceed
    this->sensor_query_outstanding_ = false;
    this->last_sensor_query_id_     = 0xFF;
    return;
  }

  // -------------------------
  // Short reply (no ID):
  // payload = 80 EF 80 00 2C  <hi> <lo>         (len == 7)
  // -------------------------
  if (frame->data_length == 7 &&
      frame->data[2] == this->command_mode_write_ && frame->data[3] == 0x00 && frame->data[4] == 0x2C) {

    const uint16_t raw = (static_cast<uint16_t>(frame->data[5]) << 8) | frame->data[6];

    if (prev_id != 0xFF) {
      for (auto &ps : this->polled_sensors_) {
        if (ps.id == prev_id) {
          const float value = static_cast<float>(raw) * ps.scale;
          if (prev_id == ESTIA_FIRST_GEN_DHW_TEMP_REQUEST) {
            this->publish_dhw_current_temperature_(value);
          } else if (ps.sensor != nullptr) {
            ps.sensor->publish_state(value);
          }
          ESP_LOGD(TAG, "0x1A short: id=0x%02X raw=0x%04X -> %.3f", prev_id, raw, value);
          break;
        }
      }
    } else {
      ESP_LOGW(TAG, "0x1A short with no last id; ignoring value raw=0x%04X", raw);
    }

    // We got a reply—release the outstanding guard
    this->sensor_query_outstanding_ = false;
    this->last_sensor_query_id_     = 0xFF;
    return;
  }

  // -------------------------
  // Long reply (with ID):
  // payload = 80 EF 00 2C 08 00  <id>  <hi> <lo>   (len >= 9)
  // -------------------------
  if (frame->data_length >= 9 &&
      frame->data[2] == 0x00 && frame->data[3] == 0x2C &&
      frame->data[4] == this->command_mode_read_ && frame->data[5] == 0x00) {

    const uint8_t  id  = frame->data[6];
    const uint16_t raw = (static_cast<uint16_t>(frame->data[7]) << 8) | frame->data[8];

    for (auto &ps : this->polled_sensors_) {
      if (ps.id == id) {
        const float value = static_cast<float>(raw) * ps.scale;
        if (id == ESTIA_FIRST_GEN_DHW_TEMP_REQUEST) {
          this->publish_dhw_current_temperature_(value);
        } else if (ps.sensor != nullptr) {
          ps.sensor->publish_state(value);
        }
        ESP_LOGD(TAG, "0x1A long:  id=0x%02X raw=0x%04X -> %.3f", id, raw, value);
        break;
      }
    }

    // Clear outstanding (only one query at a time anyway)
    this->sensor_query_outstanding_ = false;
    this->last_sensor_query_id_     = 0xFF;
    return;
  }

  // -------------------------
  // Fallback for any new/unknown variant
  // -------------------------
  ESP_LOGW(TAG, "0x1A unrecognized (header ok, no pattern match), last_id=0x%02X", prev_id);
  log_raw_data("0x1A unrecognized (no pattern)", frame->raw, frame->size());
  this->sensor_query_outstanding_ = false;
  this->last_sensor_query_id_     = 0xFF;
}


void ToshibaAbClimate::publish_dhw_current_temperature_(float temperature) {
  if (temperature < 0 || temperature > 95) {
    ESP_LOGW(TAG, "Ignoring DHW current temperature %.1f °C outside expected range", temperature);
    return;
  }

  bool changed = false;
  if (this->current_temperature != temperature) {
    this->current_temperature = temperature;
    changed = true;
  }
  if (this->dhw_current_temp_sensor_ != nullptr) {
    this->dhw_current_temp_sensor_->publish_state(temperature);
  }
  if (changed) {
    this->publish_state();
  }
}

uint8_t to_tcc_power(const climate::ClimateMode mode) {
  switch (mode) {
    case climate::CLIMATE_MODE_OFF:
      return 0;
    default:
      return 1;
  }
}

uint8_t to_tcc_mode(const climate::ClimateMode mode) {
  switch (mode) {
    case climate::CLIMATE_MODE_OFF:
      return 0;
    case climate::CLIMATE_MODE_HEAT:
      return MODE_HEAT;
    case climate::CLIMATE_MODE_COOL:
      return MODE_COOL;
    case climate::CLIMATE_MODE_FAN_ONLY:
      return MODE_FAN_ONLY;
    case climate::CLIMATE_MODE_DRY:
      return MODE_DRY;
    case climate::CLIMATE_MODE_HEAT_COOL:
      return MODE_AUTO;
    default:
      return 0;
  }
}

uint8_t to_tcc_fan(const climate::ClimateFanMode fan) {
  switch (fan) {
    case climate::CLIMATE_FAN_AUTO:
      return FAN_PEED_AUTO;
    case climate::CLIMATE_FAN_LOW:
      return FAN_PEED_LOW;
    case climate::CLIMATE_FAN_MEDIUM:
      return FAN_PEED_MED;
    case climate::CLIMATE_FAN_HIGH:
      return FAN_PEED_HIGH;
    default:
      return FAN_PEED_LOW;
  }
}

climate::ClimateAction to_climate_action(const TccState* s) {
  if (!s || s->power == 0) return climate::CLIMATE_ACTION_OFF;

  switch (s->mode) {
    case MODE_HEAT:
      // Always report heating when mode is HEAT
      return climate::CLIMATE_ACTION_HEATING;

    case MODE_COOL:
      // Always report cooling when mode is COOL
      return climate::CLIMATE_ACTION_COOLING;

    case MODE_AUTO:
      // Fall back to flags if available; otherwise idle
      if (s->cooling) return climate::CLIMATE_ACTION_COOLING;
      if (s->heating) return climate::CLIMATE_ACTION_HEATING;
      return climate::CLIMATE_ACTION_IDLE;

    case MODE_FAN_ONLY:
      return climate::CLIMATE_ACTION_FAN;

    case MODE_DRY:
      return climate::CLIMATE_ACTION_DRYING;

    default:
      return climate::CLIMATE_ACTION_IDLE;
  }
}

climate::ClimateMode to_climate_mode(const struct TccState *state) {
  if (state->power == 0)
    return climate::CLIMATE_MODE_OFF;

  switch (state->mode) {
    case MODE_HEAT:
      return climate::CLIMATE_MODE_HEAT;
    case MODE_COOL:
      return climate::CLIMATE_MODE_COOL;
    case MODE_FAN_ONLY:
      return climate::CLIMATE_MODE_FAN_ONLY;
    case MODE_DRY:
      return climate::CLIMATE_MODE_DRY;
    case MODE_AUTO:
      return climate::CLIMATE_MODE_HEAT_COOL;
  }

  return climate::CLIMATE_MODE_OFF;
}

uint8_t decode_status_mode(const uint8_t mode_power, const bool tu2c_frame) {
  uint8_t mode = (mode_power & STATUS_DATA_MODE_MASK) >> STATUS_DATA_MODE_SHIFT_BITS;

  // TU2C status frames encode AUTO as 0b110 instead of 0b101.
  if (tu2c_frame && mode == 0x06) {
    mode = MODE_AUTO;
  }

  return mode;
}

climate::ClimateFanMode to_climate_fan(const struct TccState *state) {
  if (state->power == 0)
    return climate::CLIMATE_FAN_AUTO;

  switch (state->fan) {
    case FAN_PEED_AUTO:
      return climate::CLIMATE_FAN_AUTO;
    case FAN_PEED_LOW:
      return climate::CLIMATE_FAN_LOW;
    case FAN_PEED_MED:
      return climate::CLIMATE_FAN_MEDIUM;
    case FAN_PEED_HIGH:
      return climate::CLIMATE_FAN_HIGH;
  }

  return climate::CLIMATE_FAN_ON;
}

ToshibaAbClimate::ToshibaAbClimate() {
  target_temperature = NAN;
  this->traits_.add_feature_flags(climate::CLIMATE_SUPPORTS_ACTION);
  this->traits_.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);
  this->traits_.set_supported_modes({
      climate::CLIMATE_MODE_OFF,
      climate::CLIMATE_MODE_HEAT,
      climate::CLIMATE_MODE_COOL,
      climate::CLIMATE_MODE_FAN_ONLY,
      climate::CLIMATE_MODE_DRY,
      climate::CLIMATE_MODE_HEAT_COOL,
  });
  this->traits_.set_supported_fan_modes({
      climate::CLIMATE_FAN_AUTO,
      climate::CLIMATE_FAN_LOW,
      climate::CLIMATE_FAN_MEDIUM,
      climate::CLIMATE_FAN_HIGH,
  });
  this->traits_.set_visual_min_temperature(18);
  this->traits_.set_visual_max_temperature(29);
  this->traits_.set_visual_temperature_step(0.5);
}

climate::ClimateTraits ToshibaAbClimate::traits() { return traits_; }

void ToshibaAbClimate::dump_config() {
  ESP_LOGCONFIG(TAG, "Toshiba AB:");
#ifdef LOG_UART_DEVICE
  LOG_UART_DEVICE(this);
#else
  ESP_LOGCONFIG(TAG, "  UART: logging unavailable");
#endif
  this->dump_traits_(TAG);
  ESP_LOGCONFIG(TAG, "  Remote address: 0x%02X", this->remote_address_);
  ESP_LOGCONFIG(TAG, "  Master address: 0x%02X", this->master_address_);
  ESP_LOGCONFIG(TAG, "  Master address confirmed: %s", this->master_address_confirmed_ ? "true" : "false");
  ESP_LOGCONFIG(TAG, "  Master address auto: %s", this->master_address_auto_ ? "true" : "false");
  ESP_LOGCONFIG(TAG, "  Command mode read: 0x%02X", this->command_mode_read_);
  ESP_LOGCONFIG(TAG, "  Command mode write: 0x%02X", this->command_mode_write_);
  const char *fmt_label;
  switch (this->data_reader.frame_format()) {
    case FrameFormat::TU2C: fmt_label = "TU2C (U series)"; break;
    case FrameFormat::A0: fmt_label = "A0-protocol"; break;
    case FrameFormat::HM: fmt_label = "HM (RAV-RM/HM range)"; break;
    case FrameFormat::ESTIA: fmt_label = "Estia first generation (R410A)"; break;
    default: fmt_label = "TCC-Link"; break;
  }
  ESP_LOGCONFIG(TAG, "  Frame format: %s%s%s", fmt_label, this->frame_format_auto_ ? " (auto" : "",
                this->frame_format_auto_ ? (this->frame_format_confirmed_ ? ", confirmed)" : ", pending)") : "");
#ifdef USE_ESP8266
  if (this->hw_uart_rx_enabled_) {
    ESP_LOGCONFIG(TAG, "  Hardware UART0 RX: enabled");
    ESP_LOGCONFIG(TAG, "    Receive path: hardware UART0");
    ESP_LOGCONFIG(TAG, "    RX pin: GPIO%u%s", this->hw_uart_rx_pin_,
                  this->hw_uart_rx_pin_ == 13 ? " (UART0 swapped RX)" : "");
    ESP_LOGCONFIG(TAG, "    UART0 swap: %s", this->hw_uart_rx_pin_ == 13 ? "yes (GPIO3 -> GPIO13)" : "no");
  } else {
    ESP_LOGCONFIG(TAG, "  Hardware UART0 RX: disabled (using ESPHome UART RX)");
  }
#else
  ESP_LOGCONFIG(TAG, "  Hardware UART0 RX: %s",
                this->hw_uart_rx_enabled_ ? "configured but unsupported on this platform" : "disabled");
  if (this->hw_uart_rx_enabled_) {
    ESP_LOGCONFIG(TAG, "    Configured RX pin: GPIO%u", this->hw_uart_rx_pin_);
  }
#endif
  ESP_LOGCONFIG(TAG, "  Filter frames: %s", this->filter_frames_ ? "true" : "false");
  ESP_LOGCONFIG(TAG, "  Packet min wait: %" PRIu32 "ms", this->packet_min_wait_millis_);
  ESP_LOGCONFIG(TAG, "  Autonomous mode: %s", this->autonomous_ ? "true" : "false");
  ESP_LOGCONFIG(TAG, "  Ping: %s", this->ping_enabled_ ? "true" : "false");
  ESP_LOGCONFIG(TAG, "  Read only: %s", this->read_only_ ? "true" : "false");
  const bool has_ext_temp = this->ext_temp_sensor_ != nullptr;
  ESP_LOGCONFIG(TAG, "  External temp report: %s", (has_ext_temp && this->ext_temp_enabled_) ? "enabled" : "disabled");
  if (has_ext_temp) {
    ESP_LOGCONFIG(TAG, "    Sensor: %s", this->ext_temp_sensor_name_.c_str());
    ESP_LOGCONFIG(TAG, "    Interval: %" PRIu32 "ms", this->ext_temp_interval_ms_);
  }
  ESP_LOGCONFIG(TAG, "  Polled sensors configured: %zu", this->polled_sensors_.size());
  ESP_LOGCONFIG(TAG, "  Connected sensor: %s", this->connected_binary_sensor_ ? "yes" : "no");
  ESP_LOGCONFIG(TAG, "  Failed CRCs sensor: %s", this->failed_crcs_sensor_ ? "yes" : "no");
  ESP_LOGCONFIG(TAG, "  Filter alert sensor: %s", this->filter_alert_sensor_ ? "yes" : "no");
  ESP_LOGCONFIG(TAG, "  Vent switch: %s", this->vent_switch_ ? "yes" : "no");
  ESP_LOGCONFIG(TAG, "  Read-only switch: %s", this->read_only_switch_ ? "yes" : "no");
}

void ToshibaAbClimate::setup() {
  if (this->failed_crcs_sensor_ != nullptr) {
    this->failed_crcs_sensor_->publish_state(0);
  }
  ESP_LOGD("toshiba", "Setting up ToshibaClimate...");

  // Restore last-known mode/target_temp/fan_mode from preferences so we never
  // boot with target_temperature = NaN. Without this, a control command sent
  // before the first valid STATUS frame encodes NaN -> 0xFF -> 92.5°C and the
  // wall remote shows 92. `restore_from_flash: true` in YAML makes this also
  // survive a power loss; otherwise it survives OTA + WDT reboots via RTC.
  auto restore = this->restore_state_();
  if (restore.has_value()) {
    restore->apply(this);
  } else {
    this->target_temperature = 22.0f;
    this->mode = climate::CLIMATE_MODE_OFF;
  }

  // Override traits for Estia heat pump
  if (this->data_reader.frame_format() == FrameFormat::A0 || this->data_reader.frame_format() == FrameFormat::ESTIA) {
    if (this->data_reader.frame_format() == FrameFormat::ESTIA) {
      this->traits_.set_supported_modes({climate::CLIMATE_MODE_OFF, climate::CLIMATE_MODE_HEAT});
    } else {
      this->traits_.set_supported_modes({climate::CLIMATE_MODE_OFF, climate::CLIMATE_MODE_HEAT, climate::CLIMATE_MODE_COOL});
    }
    this->traits_.set_supported_fan_modes({});
    this->traits_.set_supported_swing_modes({});
    this->traits_.set_visual_min_temperature(20);
    this->traits_.set_visual_max_temperature(65);
    this->traits_.set_visual_temperature_step(0.5);
  }
  


  pinMode(16, OUTPUT); // Set GPIO16 low, only needed for my old board, to be removed soon
  digitalWrite(16, LOW);

  // --- Optional/auto hardware-UART RX (see set_hardware_uart_rx_pin) ---
  // When the bus RX pin is an ESP8266 hardware-UART0 pin (GPIO13 via swap, or
  // GPIO3 via explicit config) but TX is not, ESPHome would bit-bang both. The
  // software-serial RX ISR
  // busy-waits ~1 char time per byte at 2400 baud, starving Wi-Fi enough to trip
  // the watchdog on busy buses (makusets#88). Moving RX onto the real hardware
  // UART gives a FIFO-backed, zero-busy-wait receiver. TX stays on the ESPHome
  // software-serial parent (configured tx_pin). UART0 is free when logger
  // baud_rate is 0. Disabled by default, so all other configs are unaffected.
#ifdef USE_ESP8266
  if (this->hw_uart_rx_enabled_) {
    s_bus_serial.setRxBufferSize(512);
    SerialConfig hw_uart_config = SERIAL_8N1;
    if (this->hw_uart_parity_ == 1) {
      hw_uart_config = SERIAL_8E1;
    } else if (this->hw_uart_parity_ == 2) {
      hw_uart_config = SERIAL_8O1;
    }
    s_bus_serial.begin(2400, hw_uart_config);
    if (this->hw_uart_rx_pin_ == 13) {
      s_bus_serial.swap();  // UART0: RX GPIO3->GPIO13, TX GPIO1->GPIO15 (unused)
    }
    ESP_LOGCONFIG(TAG, "Hardware UART0 RX enabled on GPIO%u%s (%s parity)", this->hw_uart_rx_pin_,
                  this->hw_uart_rx_pin_ == 13 ? " (UART0 swapped RX)" : "",
                  this->hw_uart_parity_ == 1 ? "even" : (this->hw_uart_parity_ == 2 ? "odd" : "no"));
  }
#endif

  // If autonomous mode is enabled, we need to send ping, E8 read, and external temp periodically
  // Send all these if autonomous mode is enabled using set_interval calls
  if (this->autonomous_) {
    if (this->data_reader.frame_format() == FrameFormat::A0) {
      // Estia polling is handled in loop() so it respects runtime autonomous_ toggle
    } else if (this->data_reader.frame_format() == FrameFormat::TU2C) {
      this->announce_ack_received_ = false;
      this->set_interval(2000, [this]() {
        if (!this->announce_ack_received_) {
          ESP_LOGV(TAG, "Autonomous TU2C: sending registration query");
          this->tu2c_remote_announce();
        }
      });

      this->set_interval(this->ping_interval_ms_, [this]() {
        if (this->announce_ack_received_) {
          ESP_LOGV(TAG, "Autonomous TU2C: sending keepalive");
          this->tu2c_send_ping();
        }
      });
    } else {
      this->set_interval(this->ping_interval_ms_, [this]() { //30s
        // Ping (keep-alive) from remote
        this->send_ping();  // just enqueues; loop() will transmit
        ESP_LOGV(TAG, "Autonomous: enqueued PING (keep-alive)");

        float t = NAN;
        if (this->ext_temp_sensor_ && this->ext_temp_sensor_->has_state()) {
          t = this->ext_temp_sensor_->state;
        } else if (!std::isnan(this->target_temperature)) {
          t = this->target_temperature;
        } else {
          t = 20.0f;
        }

        if (std::isfinite(t)) {
          const uint8_t raw = temp_celcius_to_payload(t);
          ESP_LOGV(TAG, "Autonomous: enqueuing remote temperature %.1f°C (raw=0x%02X)", t, raw);
          DataFrame cmd{};
          cmd.source = this->remote_address_;
          cmd.dest = this->master_address_;
          cmd.opcode1 = OPCODE_TEMPERATURE;
          cmd.data_length = 5;
          cmd.data[0] = this->command_mode_read_;
          cmd.data[1] = 0x81;
          cmd.data[2] = 0x00;
          cmd.data[3] = raw;
          cmd.data[4] = 0x00;
          cmd.data[cmd.data_length] = cmd.calculate_crc();
          this->send_command(cmd);
          ESP_LOGV(TAG, "Autonomous: remote temperature frame enqueued (dest=0x%02X)", this->master_address_);
        }
      });
      this->set_interval(this->read08_interval_ms, [this]() {
        ESP_LOGV(TAG, "Autonomous: enqueuing read hourly counter block E8");
        this->send_read_block(0xE8, 0x0001, 0x009E);
      });

      this->announce_ack_received_ = false;
      this->set_interval(2000, [this]() {
        if (!this->announce_ack_received_) {
          ESP_LOGV(TAG, "Autonomous announce: sending broadcast announce");
          this->remote_announce();
        }
      });
    }
  } else if (this->ping_enabled_) {
    this->set_interval(this->ping_interval_ms_, [this]() {
      if (this->data_reader.frame_format() != FrameFormat::NORMAL) {
        return;
      }
      if (!this->remote_address_confirmed_) {
        ESP_LOGV(TAG, "Ping skipped: remote address is not confirmed yet");
        return;
      }
      if (!this->master_address_confirmed_) {
        ESP_LOGV(TAG, "Ping skipped: master address is not confirmed yet");
        return;
      }
      this->send_ping();
      ESP_LOGV(TAG, "Enqueued remote PING (keep-alive)");
    });
  }

  this->update_frame_validation_();

    // next block manages the temp reporting to the central unit using the external sensor approach if configured,
    // this sends a different temp frame to the one sent with the ping above,
    // the frame source is 0x42, which is the id for Toshiba external temp sensor, this will force the 
    // master unit to use this temp for room temp readings instead of it's own sensor, regardless of config parameters
    // frame format is: 42:00:11:04:08:89:<raw>:46:<crc> for temp in celcius
    // in autonomous mode the same sensor is used for the regular temp reporting with ping above

  if (this->ext_temp_enabled_ && this->ext_temp_sensor_ && this->ext_temp_interval_ms_ > 0) {
    this->set_interval(this->ext_temp_interval_ms_, [this]() {
      float t = NAN;
      if (this->ext_temp_sensor_->has_state())
        t = this->ext_temp_sensor_->state;

      if (std::isfinite(t) && t > -40.0f && t < 80.0f) {
        ESP_LOGV(TAG, "Using external temp: enqueuing temp %.1f°C", t);
        this->send_remote_temp(t);   // enqueues; loop() controls bus timing
      } else {
        ESP_LOGV(TAG, "report_sensor_temp: source has no valid state");
      }
    });
  }
  this->set_interval(1000, [this]() {
//checks if sensor query is still outstanding every second, if there hasn't been a reply after sensor_query_timeout_ms,
// it clears the outstanding flag so that the next sensor query can be sent
  if (this->sensor_query_outstanding_) {
    const uint32_t now = millis();
    if (now - this->last_sensor_query_ms_ > this->sensor_query_timeout_ms_) {
      this->sensor_query_outstanding_ = false;
      this->last_sensor_query_id_     = 0xFF;
      this->sensor_query_timeouts_++;  // optional stat
      ESP_LOGW(TAG, "Sensor query timed out; clearing outstanding flag (timeouts=%u)",
               this->sensor_query_timeouts_);
    }
  }
});
  // Ensure the read-only switch reports its initial state on startup (default OFF)
  if (this->read_only_switch_)
    this->read_only_switch_->publish_state(this->read_only_);

}

void ToshibaAbClimate::sync_from_received_state() {
  uint8_t changes = 0;

  auto new_mode = to_climate_mode(&tcc_state);
  if (new_mode != mode) {
    mode = new_mode;
    changes++;
  }

  auto new_action = to_climate_action(&tcc_state);
  if (new_action != action) {
    action = new_action;
    changes++;
  }

  auto new_fan_mode = to_climate_fan(&tcc_state);
  if (new_fan_mode != fan_mode) {
    fan_mode = new_fan_mode;
    changes++;
  }

  if (target_temperature != tcc_state.target_temp && tcc_state.target_temp >= 16 &&
      tcc_state.target_temp <= 29) {
    // only update target temperature if it's within the supported range
    // (16-29°C)
    //this filters out misreadings from the remote
    target_temperature = tcc_state.target_temp;
    changes++;
  }

  if (current_temperature != tcc_state.room_temp && tcc_state.room_temp >= 5 &&
      tcc_state.room_temp <= 35) {
    // only update current temperature if it's within normal range
    //this filters out misreadings from the remote
    current_temperature = tcc_state.room_temp;
    changes++;
  }
  if (this->filter_alert_state_ != tcc_state.filter_alert) {
    this->filter_alert_state_ = tcc_state.filter_alert;
    if (this->filter_alert_sensor_) {
      this->filter_alert_sensor_->publish_state(this->filter_alert_state_);
    }
    changes++;
  }

  if (changes > 0) {
    this->publish_state();
  }

  if (this->vent_switch_) {
    this->vent_switch_->publish_state(tcc_state.vent);
  }
}

void ToshibaAbClimate::autoreset_remote_error_() {
  ESP_LOGI(TAG, "Autoreset errors: resending current mode/fan/target temperature");
  auto command = DataFrame{};
  if (this->data_reader.frame_format() == FrameFormat::TU2C) {
    write_set_parameter_flags_tu2c(&command, this->tu2c_remote_address_, this->tu2c_master_address_, &this->tcc_state,
                                   COMMAND_SET_TEMP | COMMAND_SET_FAN);
  } else {
    write_set_parameter_flags(&command, this->remote_address_, this->master_address_, this->command_mode_read_, &this->tcc_state,
                              COMMAND_SET_TEMP | COMMAND_SET_FAN, this->is_hm_variant());
  }
  this->send_command(command);
}

void ToshibaAbClimate::process_received_data(const struct DataFrame *frame) {
  if (frame->source == this->master_address_) {
      if (!this->master_address_confirmed_) {
        this->master_address_confirmed_ = true;
        ESP_LOGI(TAG, "Master address confirmed from valid frame: 0x%02X", this->master_address_);
      }
      // status update
      ESP_LOGD(TAG, "Received data from master:");
      last_master_alive_millis_ = millis();
      if (this->connected_binary_sensor_) {
        this->connected_binary_sensor_->publish_state(true);
      }

      switch (frame->opcode1) {
        case OPCODE_PING: {
        log_data_frame("PING/ALIVE", frame);
        this->confirm_frame_format_(FrameFormat::NORMAL, "valid keepalive from master");
        break;
        }
        case OPCODE_ACK: {
        // ACK (maps to 0x18)
        log_data_frame("ACK", frame);
        this->handle_pending_command_ack_(frame);

        // Check for announce ACK pattern: 6th raw byte == 0x0D (index 5)
        // When this is received after boot, the remote can stop announcing itself
        // The full raw frame may be longer; guard on size()
        if (frame->size() > 5 && frame->raw[5] == 0x0D) {
          this->announce_ack_received_ = true;
          this->update_frame_validation_();
          ESP_LOGI(TAG, "Received announce ACK (0x0D) from 0x%02X, stopping announce", frame->source);
        }
        break;
      }
        case OPCODE_PARAMETER:
          // master reporting it's state
          // e.g. 01:52:11:04:80:86:A1:05:E4
          /*
          00 52 11 04 80 86 24 00 65  heat
                |-opc1      |  |- mode  bit7-bit5, power bit0, bit2 ???
                            |- 0010 0100 -> mode bit7-bit5  bit4-bit0 ???
                              ---
          */
          log_data_frame("MASTER PARAMETERS", frame);

          tcc_state.power = (frame->data[3] & STATUS_DATA_POWER_MASK);
          tcc_state.mode =
              (frame->data[STATUS_DATA_MODEPOWER_BYTE] & STATUS_DATA_MODE_MASK) >> STATUS_DATA_MODE_SHIFT_BITS;
          tcc_state.cooling = (frame->data[STATUS_DATA_MODEPOWER_BYTE] & 0b00001000) >> 3;
          tcc_state.heating = (frame->data[STATUS_DATA_MODEPOWER_BYTE] & 0b00000001);
          tcc_state.preheating = (frame->data[3] & 0b0100) >> 2;

          ESP_LOGD(TAG, "Mode: %02X, Cooling: %d, Heating: %d, Preheating: %d", tcc_state.mode, tcc_state.cooling,
                   tcc_state.heating, tcc_state.preheating);

          sync_from_received_state();

          break;
        case OPCODE_STATUS: {
          // sync power, mode, fan and target temp from the unit to the climate
          // component
          // Sanity-guard: the AC's 0x81 marker byte lives at a format-dependent
          // offset. Classic TCC-Link puts it at raw[5] (data[1]); HM puts it at
          // raw[4] (data[0]). Reject any STATUS frame missing the marker, but
          // check the right offset per variant — otherwise every HM STATUS
          // gets dropped and the climate component never syncs from the AC.
          const uint8_t marker_off = (this->data_reader.frame_format() == FrameFormat::HM) ? 4 : 5;
          if (frame->size() <= marker_off || frame->raw[marker_off] != 0x81) {
            log_data_frame("STATUS ignored (marker != 0x81)", frame);
            break;
          }

          log_data_frame("STATUS", frame);

          tcc_state.power = (frame->data[STATUS_DATA_MODEPOWER_BYTE] & STATUS_DATA_POWER_MASK);
          tcc_state.mode =
              (frame->data[STATUS_DATA_MODEPOWER_BYTE] & STATUS_DATA_MODE_MASK) >> STATUS_DATA_MODE_SHIFT_BITS;
          tcc_state.fan = (frame->data[STATUS_DATA_FANVENT_BYTE] & STATUS_DATA_FAN_MASK) >> STATUS_DATA_FAN_SHIFT_BITS;
          tcc_state.vent =
              (frame->data[STATUS_DATA_FANVENT_BYTE] & STATUS_DATA_VENT_MASK) >> STATUS_DATA_VENT_SHIFT_BITS;
          tcc_state.target_temp =
              static_cast<float>(frame->data[STATUS_DATA_TARGET_TEMP_BYTE]) /
                  TEMPERATURE_CONVERSION_RATIO -
              TEMPERATURE_CONVERSION_OFFSET;

          ESP_LOGD(TAG, "Power: %d, Mode: %02X, Fan: %02X, Vent: %02X, Target Temp: %.1f",
                   tcc_state.power, tcc_state.mode, tcc_state.fan, tcc_state.vent, tcc_state.target_temp);


          sync_from_received_state();

          break;
        }
        case OPCODE_EXTENDED_STATUS: {
          // sync power, mode, fan and target temp from the unit to the climate
          // component
          // See OPCODE_STATUS above for the rationale: 0x81 marker is at
          // raw[4] on HM, raw[5] on classic.
          const uint8_t marker_off_ext = (this->data_reader.frame_format() == FrameFormat::HM) ? 4 : 5;
          if (frame->size() <= marker_off_ext || frame->raw[marker_off_ext] != 0x81) {
            log_data_frame("EXTENDED STATUS ignored (marker != 0x81)", frame);
            break;
          }

          log_data_frame("EXTENDED STATUS", frame);

          constexpr uint8_t rt_off = STATUS_DATA_TARGET_TEMP_BYTE + 1;

          tcc_state.power = (frame->data[STATUS_DATA_MODEPOWER_BYTE] & STATUS_DATA_POWER_MASK);
          tcc_state.mode =
              (frame->data[STATUS_DATA_MODEPOWER_BYTE] & STATUS_DATA_MODE_MASK) >> STATUS_DATA_MODE_SHIFT_BITS;
          tcc_state.fan = (frame->data[STATUS_DATA_FANVENT_BYTE] & STATUS_DATA_FAN_MASK) >> STATUS_DATA_FAN_SHIFT_BITS;
          tcc_state.vent =
              (frame->data[STATUS_DATA_FANVENT_BYTE] & STATUS_DATA_VENT_MASK) >> STATUS_DATA_VENT_SHIFT_BITS;

          tcc_state.target_temp =
              static_cast<float>(frame->data[STATUS_DATA_TARGET_TEMP_BYTE]) /
                  TEMPERATURE_CONVERSION_RATIO -
              TEMPERATURE_CONVERSION_OFFSET;

          if (frame->data_length > rt_off && frame->data[rt_off] > 1) {
            tcc_state.room_temp =
                static_cast<float>(frame->data[rt_off]) / TEMPERATURE_CONVERSION_RATIO -
                TEMPERATURE_CONVERSION_OFFSET;
          }

          tcc_state.preheating  = (frame->data[STATUS_DATA_FLAGS_BYTE] & 0b00000010) >> 1;
          tcc_state.filter_alert = (frame->data[STATUS_DATA_FLAGS_BYTE] & 0b10000000) >> 7;
          ESP_LOGD(TAG, "Power: %d, Mode: %02X, Fan: %02X, Vent: %02X, Target Temp: %.1f, Room Temp: %.1f, Preheating: %d, Filter Alert: %d",
                   tcc_state.power, tcc_state.mode, tcc_state.fan, tcc_state.vent, tcc_state.target_temp,
                   tcc_state.room_temp, tcc_state.preheating, tcc_state.filter_alert);

          sync_from_received_state();

          break;
        }
        case OPCODE_SENSOR_VALUE:
            // sensor value received from master
          this->process_sensor_value_(frame);
          break;

        default:
          log_data_frame("MASTER", frame);
          break;
      }
    }else {
    const bool is_remote_source = (frame->source == this->remote_address_) ||
                                  (frame->source >= 0x40 && frame->source <= 0x49);
    if (is_remote_source) {
      // In remote auto-address mode we start at 0x40 and avoid collisions with
      // existing remotes on the bus. If any remote frame arrives using our
      // current address, shift to the next address up to 0x49.
      if (this->remote_address_auto_ &&
          frame->source == this->remote_address_ &&
          this->remote_address_ < TOSHIBA_REMOTE_MAX) {
        const uint8_t old = this->remote_address_;
        this->remote_address_++;
        this->remote_address_confirmed_ = true;
        ESP_LOGI(TAG, "Remote auto-address collision detected at 0x%02X; switching to confirmed address 0x%02X",
                 old, this->remote_address_);
      }
      ESP_LOGD(TAG, "Received data from remote:");

      // Remote temperature push: 40 00 55 05 08 81 01 6E 00 ..
      if (frame->opcode1 == OPCODE_TEMPERATURE &&
          frame->data_length >= 4 &&
          frame->data[1] == 0x81) {
        uint8_t raw = frame->data[3] & TEMPERATURE_DATA_MASK;  // raw[7]
        float rmt = static_cast<float>(raw) / TEMPERATURE_CONVERSION_RATIO - TEMPERATURE_CONVERSION_OFFSET;

        // tcc_state.room_temp = rmt; we don't update the state here, we wait for the next status update from master
        log_data_frame("Remote temperature", frame);
        ESP_LOGD(TAG, "Toshiba Wall Remote reports: %.1f °C", rmt);
        // sync_from_received_state(); we don't update the state, we wait for the next status update from master
        // remote temperature is sent regardless of DN32 setting, ac decides wether to use it or ignore it
        

      // Remote error report: 40 FE 55 02 0E 49 AE
      } else if (frame->opcode1 == OPCODE_TEMPERATURE &&
                 frame->data_length == 2 &&
                 frame->data[1] == 0x49) {
        log_data_frame("Remote error report", frame);
        if (this->autoreset_errors_) {
          this->autoreset_remote_error_();
        } else {
          ESP_LOGD(TAG, "Ignoring remote error report because autoreset_errors is disabled");
        }

      // Remote PING sent every 30s: 40 00 15 07 08 0C 81 00 00 48 00 ..
      } else if (frame->opcode1 == OPCODE_ERROR_HISTORY &&      // 0x15 envelope
                frame->data_length >= 3 &&
                frame->data[0] == this->command_mode_read_ &&
                frame->data[1] == OPCODE2_PING_PONG &&         // 0x0C
                frame->data[2] == OPCODE2_READ_STATUS) {       // 0x81
        log_data_frame("Remote PING", frame);
        
        // Auto-update master address if enabled and different from current
        if (this->master_address_auto_ &&
        frame->dest != this->master_address_) {
          ESP_LOGI(TAG, "Remote ping addressed to new master: 0x%02X, updating master address", frame->dest);
          this->master_address_ = frame->dest;
          this->master_address_confirmed_ = false;
        }
      
      // Remote 40:00:15:06:08:E8:00:01:00:9E:2C that is sent every minute, the master responds with an hourly counter (time on?)
      } else if (frame->opcode1 == OPCODE_ERROR_HISTORY &&      // 0x15 envelope
                frame->data_length >= 6 &&
                frame->data[0] == this->command_mode_read_ &&
                frame->data[1] == 0xE8 &&
                frame->data[3] == 0x01 &&                 
                frame->data[5] == 0x9E) {                       
                
        log_data_frame("Remote Timer Read", frame);

      } else if (frame->opcode1 == OPCODE_PARAMETER &&
                 frame->data_length >= 3) {
        // Remote command frames (opcode1 0x11)
        if (frame->data[1] == 0x4C) {
          log_data_frame("Remote command: fan/mode/temp change", frame);
        } else if (frame->data[1] == 0x41 && frame->data[2] == 0x02) {
          log_data_frame("Remote command: power off", frame);
        } else if (frame->data[1] == 0x41 && frame->data[2] == 0x03) {
          log_data_frame("Remote command: power on", frame);
        } else {
          log_data_frame("Remote command", frame);
        }
      } else {
        // unknown remote message
        log_data_frame("Unknown remote data", frame);
      }
    
    } else if (frame->opcode1 == OPCODE_PARAMETER &&
               frame->data_length == 4 &&
               frame->data[1] == 0x89) {
      // External room-temperature report frame. Do not key this on a fixed
      // source address (e.g. 0x42), because runtime remote id can vary.

        std::string label = this->ext_temp_sensor_name_.empty()
                    ? "Yaml temp sensor"
                    : this->ext_temp_sensor_name_;
        log_data_frame(label, frame);
        uint8_t raw = frame->data[2] & TEMPERATURE_DATA_MASK;  // raw[7]
        float sensor_temp = static_cast<float>(raw) / TEMPERATURE_CONVERSION_RATIO - TEMPERATURE_CONVERSION_OFFSET;
        ESP_LOGD(TAG, "%s: %.1f °C", label.c_str(), sensor_temp);
    } else {
    // Unknown source handling
    ESP_LOGD(TAG, "Received data from unknown source: %02X", frame->source);
    log_data_frame("Unknown source", frame);

    // Auto-detect master address from master parameters frame
      if (this->master_address_auto_) {
      // First, check for announce ACK pattern from an unknown source: if the
      // raw frame contains 0x0D at byte index 5, treat that source as the new
      // master.
      if (frame->size() > 5 && frame->raw[5] == 0x0D) {
        // Only act on announce ACK if we haven't already seen one
        if (!this->announce_ack_received_) {
          ESP_LOGI(TAG, "Auto-detected master address from announce ACK: 0x%02X, updating master address", frame->source);
          this->master_address_ = frame->source;
          // Mark that we've received the announce ACK so we don't repeatedly auto-update
          this->announce_ack_received_ = true;
          this->update_frame_validation_();
        } else {
          ESP_LOGV(TAG, "Announce ACK from 0x%02X ignored; announce_ack_received_ already true", frame->source);
        }
      } else {
        // Fallback: Check for master parameters pattern (opcode, length, etc.)
        if (frame->opcode1 == OPCODE_PARAMETER && frame->data_length >= 4) {
          ESP_LOGI(TAG, "Auto-detected master address: 0x%02X, updating master address", frame->source);
          this->master_address_ = frame->source;
        }
      }
      }
    }
    } 
  }

void ToshibaAbClimate::process_received_data_tu2c_(const struct DataFrame *frame) {
  if (frame == nullptr) {
    return;
  }

  const size_t size = frame->size();
  if (size < 4) {
    log_raw_data("TU2C frame too short", frame->raw, size);
    return;
  }

  const uint8_t frame_length = frame->raw[0];
  const uint8_t source = frame->raw[1];
  const uint8_t dest = frame->raw[2];
  const size_t payload_offset = 4;
  const size_t payload_available = size > payload_offset ? size - payload_offset : 0;
  const bool has_tail_signature =
      size >= 3 && frame->raw[size - 3] == 0x00 && frame->raw[size - 2] == 0x3A;

  if (frame_length == 0x0A && has_tail_signature) {
    log_raw_data("Master keepalive", frame->raw, size);
    ESP_LOGD(TAG, "Master %02X keepalive", source);
    last_master_alive_millis_ = millis();
    if (this->connected_binary_sensor_) {
      this->connected_binary_sensor_->publish_state(true);
    }

    if (this->master_address_auto_ && this->master_address_ == 0x00 && source != 0x00) {
      ESP_LOGI(TAG, "Auto-detected master address from TU2C keepalive: 0x%02X", source);
      this->set_master_address(source);
    }
    return;
  }

  if (frame_length == 0x12 && this->is_tu2c_registration_ack_(frame)) {
    log_raw_data("Master registration ACK frame", frame->raw, size);
    ESP_LOGD(TAG, "Registration ACK from master %02X", source);
    this->announce_ack_received_ = true;
    return;
  }

  if (frame_length == 0x0C && source == this->tu2c_master_address_ && payload_available >= 4 &&
      frame->raw[payload_offset] == 0x80) {
    log_raw_data("Master ACK frame", frame->raw, size);
    ESP_LOGD(TAG, "ACK from master %02X with counter: %02X:%02X", source, frame->raw[payload_offset + 2],
             frame->raw[payload_offset + 3]);
    return;
  }

  if (frame_length == 0x0C && payload_available >= 2 && frame->raw[payload_offset] == 0x41 &&
      frame->raw[payload_offset + 1] == 0x5C) {
    log_raw_data("Remote keepalive frame", frame->raw, size);
    ESP_LOGD(TAG, "Remote %02X keepalive", source);
    return;
  }

  if (frame_length == 0x0D && payload_available >= 4 && frame->raw[payload_offset] == 0x61 &&
      frame->raw[payload_offset + 1] == 0x38 && size > 7) {
    log_raw_data("Remote room temp frame", frame->raw, size);
    const uint8_t raw_temp = frame->raw[7];
    const float room_temp =
        static_cast<float>(raw_temp) / TEMPERATURE_CONVERSION_RATIO - TEMPERATURE_CONVERSION_OFFSET;
    tcc_state.room_temp = room_temp;
    ESP_LOGD(TAG, "Remote %02X room temp: %.1f", source, room_temp);
    return;
  }

  if (dest == 0xFF && payload_available >= STATUS_DATA_TARGET_TEMP_BYTE + 1) {
    const uint8_t status_group = frame->raw[payload_offset];
    const bool is_status_frame = status_group == 0xC0;
    const bool is_extended_status_frame = status_group == 0xA0;

    if ((!is_status_frame && !is_extended_status_frame) || frame->raw[payload_offset + 1] != 0x38) {
      log_raw_data("TU2C data", frame->raw, size);
      return;
    }
    const uint8_t *payload = &frame->raw[payload_offset];
    log_raw_data(is_extended_status_frame ? "TU2C extended status" : "TU2C status", frame->raw, size);
    tcc_state.power = (payload[STATUS_DATA_MODEPOWER_BYTE] & STATUS_DATA_POWER_MASK);
    tcc_state.mode = decode_status_mode(payload[STATUS_DATA_MODEPOWER_BYTE], true);
    tcc_state.fan = (payload[STATUS_DATA_FANVENT_BYTE] & STATUS_DATA_FAN_MASK) >> STATUS_DATA_FAN_SHIFT_BITS;
    tcc_state.vent =
        (payload[STATUS_DATA_FANVENT_BYTE] & STATUS_DATA_VENT_MASK) >> STATUS_DATA_VENT_SHIFT_BITS;
    tcc_state.target_temp =
        static_cast<float>(payload[STATUS_DATA_TARGET_TEMP_BYTE]) / TEMPERATURE_CONVERSION_RATIO -
        TEMPERATURE_CONVERSION_OFFSET;
    if (payload_available >= STATUS_DATA_TARGET_TEMP_BYTE + 2 &&
        payload[STATUS_DATA_TARGET_TEMP_BYTE + 1] > 1) {
      tcc_state.room_temp =
          static_cast<float>(payload[STATUS_DATA_TARGET_TEMP_BYTE + 1]) / TEMPERATURE_CONVERSION_RATIO -
          TEMPERATURE_CONVERSION_OFFSET;
    }
    tcc_state.preheating = (payload[STATUS_DATA_FLAGS_BYTE] & 0b00000010) >> 1;
    tcc_state.filter_alert = (payload[STATUS_DATA_FLAGS_BYTE] & 0b10000000) >> 7;

    ESP_LOGD(TAG,
             "TU2C %sstatus: power=%d mode=%02X fan=%02X vent=%02X target=%.1f room=%.1f preheat=%d filter=%d",
             is_extended_status_frame ? "extended " : "",
             tcc_state.power, tcc_state.mode, tcc_state.fan, tcc_state.vent, tcc_state.target_temp,
             tcc_state.room_temp, tcc_state.preheating, tcc_state.filter_alert);
    sync_from_received_state();
    return;
  }

  log_raw_data("TU2C data", frame->raw, size);
}

bool ToshibaAbClimate::receive_data(const std::vector<uint8_t> data) {
  auto frame = DataFrame();

  for (size_t i = 0; i < data.size(); i++) {
    frame.raw[i] = data[i];
  }

  return receive_data_frame(&frame);
}

bool ToshibaAbClimate::receive_data_frame(const struct DataFrame *frame) {
  if (frame->is_estia()) {
    // Estia A0-protocol: CRC-16/MCRF4XX
    size_t fsz = frame->estia_size();
    if (!frame->validate_estia_crc()) {
      ESP_LOGD(TAG, "CRC FAIL (recv=0x%04X calc=0x%04X size=%d)",
               frame->estia_crc_received(), frame->calculate_estia_crc(), fsz);
      // Dump full frame with decode attempt
      {
        std::string hex;
        char hbuf[4];
        // Reconstruct full frame with A0:00 prefix
        hex += "A0:00:";
        for (size_t i = 0; i < fsz; i++) {
          if (i > 0) hex += ':';
          snprintf(hbuf, sizeof(hbuf), "%02X", frame->raw[i]);
          hex += hbuf;
        }
        ESP_LOGD(TAG, "  raw: %s", hex.c_str());

        uint8_t ft = frame->raw[0];
        uint8_t fl = frame->raw[1];
        const char *type_name = "???";
        switch (ft) {
          case 0x10: type_name = "HEARTBEAT"; break;
          case 0x11: type_name = "COMMAND"; break;
          case 0x15: type_name = "DATA_REQ"; break;
          case 0x18: type_name = "ACK/RESP"; break;
          case 0x1C: type_name = "STATE_CHG"; break;
          case 0x55: type_name = "STATUS_SHORT"; break;
          case 0x58: type_name = "STATUS"; break;
        }
        uint16_t src = (fsz > 4) ? ((frame->raw[3] << 8) | frame->raw[4]) : 0;
        uint16_t dst = (fsz > 6) ? ((frame->raw[5] << 8) | frame->raw[6]) : 0;
        const char *src_name = "???";
        if (src == 0x0800) src_name = "MASTER";
        else if (src == 0x0040) src_name = "REMOTE";
        else if (src == 0x0041) src_name = "0-10V";
        else if (src == 0x0390) src_name = "KNX-GW";
        const char *dst_name = "???";
        if (dst == 0x0800) dst_name = "MASTER";
        else if (dst == 0x0040) dst_name = "REMOTE";
        else if (dst == 0x0041) dst_name = "0-10V";
        else if (dst == 0x00FE) dst_name = "BROADCAST";
        else if (dst == 0x0390) dst_name = "KNX-GW";
        ESP_LOGD(TAG, "  type=0x%02X(%s) len=%u src=0x%04X(%s) dst=0x%04X(%s)",
                 ft, type_name, fl, src, src_name, dst, dst_name);
        if (fsz > 8) {
          ESP_LOGD(TAG, "  dtype=%02X:%02X", frame->raw[7], frame->raw[8]);
        }
      }
      if (this->failed_crcs_sensor_ != nullptr) {
        this->failed_crcs_sensor_->publish_state(this->failed_crcs_sensor_->state + 1);
      }
      return false;
    }
    uint8_t frame_type = frame->raw[0];
    uint8_t frame_len = frame->raw[1];
    ESP_LOGV(TAG, "RX frame: type=0x%02X len=%d src=0x%02X%02X dst=0x%02X%02X",
             frame_type, frame_len,
             frame->raw[3], frame->raw[4],
             frame->raw[5], frame->raw[6]);
    log_raw_data("Estia", frame->raw, fsz);

    // Verbose decode of all Estia frames
    {
      uint16_t src_addr = (frame->raw[3] << 8) | frame->raw[4];
      uint16_t dst_addr = (frame->raw[5] << 8) | frame->raw[6];
      uint16_t dtype = (frame_len >= 7) ? ((frame->raw[7] << 8) | frame->raw[8]) : 0;

      const char *src_name = "???";
      if (src_addr == 0x0800) src_name = "MASTER";
      else if (src_addr == 0x0040) src_name = "REMOTE";
      else if (src_addr == 0x0041) src_name = "0-10V";
      else if (src_addr == 0x0390) src_name = "KNX-GW";

      const char *dst_name = "???";
      if (dst_addr == 0x0800) dst_name = "MASTER";
      else if (dst_addr == 0x0040) dst_name = "REMOTE";
      else if (dst_addr == 0x0041) dst_name = "0-10V";
      else if (dst_addr == 0x00FE) dst_name = "BROADCAST";
      else if (dst_addr == 0x0390) dst_name = "KNX-GW";

      const char *type_name = "???";
      switch (frame_type) {
        case 0x10: type_name = "HEARTBEAT"; break;
        case 0x11: type_name = "COMMAND"; break;
        case 0x15: type_name = "DATA_REQ"; break;
        case 0x18: type_name = "ACK/DATA_RESP"; break;
        case 0x1C: type_name = "STATE_CHANGE"; break;
        case 0x55: type_name = "STATUS_SHORT"; break;
        case 0x58: type_name = "STATUS"; break;
      }

      ESP_LOGV(TAG, "  %s(0x%02X) %s(0x%04X)->%s(0x%04X) dtype=%02X:%02X",
               type_name, frame_type, src_name, src_addr, dst_name, dst_addr,
               frame->raw[7], frame->raw[8]);

      // Decode known dtype payloads
      if (dtype == 0x03C6 && frame_len >= 15) {
        // Status/state change with temperatures
        uint8_t flags = frame->raw[9];
        float current = frame->raw[12] / 2.0f - 16.0f;
        float setpoint = frame->raw[13] / 2.0f - 16.0f;
        float outdoor = frame->raw[14] / 2.0f - 16.0f;
        ESP_LOGV(TAG, "    flags=0x%02X [%s%s%s] mode=0x%02X unknown=0x%02X",
                 flags,
                 (flags & 0x01) ? "POWER " : "",
                 (flags & 0x20) ? "COOL " : "",
                 (flags & 0x40) ? "HEAT " : "",
                 frame->raw[10], frame->raw[11]);
        ESP_LOGV(TAG, "    current=%.1f°C(0x%02X) setpoint=%.1f°C(0x%02X) outdoor=%.1f°C(0x%02X)",
                 current, frame->raw[12], setpoint, frame->raw[13], outdoor, frame->raw[14]);
        if (frame_len >= 18) {
          float r15 = frame->raw[15] / 2.0f - 16.0f;
          float r16 = frame->raw[16] / 2.0f - 16.0f;
          float r17 = frame->raw[17] / 2.0f - 16.0f;
          ESP_LOGV(TAG, "    repeat: [15]=%.1f°C(0x%02X) [16]=%.1f°C(0x%02X) [17]=%.1f°C(0x%02X)",
                   r15, frame->raw[15], r16, frame->raw[16], r17, frame->raw[17]);
        }
      } else if (dtype == 0x03C1 && frame_len >= 10) {
        // Setpoint command
        uint8_t subcmd = frame->raw[9];
        uint8_t temp_enc = frame->raw[10];
        float temp = temp_enc / 2.0f - 16.0f;
        ESP_LOGV(TAG, "    SETPOINT %s temp=%.1f°C(0x%02X)",
                 subcmd == 0x01 ? "COOL" : "HEAT", temp, temp_enc);
      } else if (dtype == 0x03C0) {
        // Mode command
        uint8_t cmd = frame->raw[9];
        ESP_LOGV(TAG, "    MODE %s(0x%02X)", cmd == 0x01 ? "COOL" : (cmd == 0x02 ? "HEAT" : "???"), cmd);
      } else if (dtype == 0x0041) {
        // Power command
        uint8_t cmd = frame->raw[9];
        ESP_LOGV(TAG, "    POWER %s(0x%02X)", cmd == 0x23 ? "ON" : (cmd == 0x22 ? "OFF" : "???"), cmd);
      } else if (dtype == 0x00A1) {
        // ACK — decode what was acknowledged
        uint8_t ack_d1 = frame->raw[9];
        uint8_t ack_d2 = frame->raw[10];
        const char *ack_what = "???";
        if (ack_d1 == 0x03 && ack_d2 == 0xC0) ack_what = "MODE";
        else if (ack_d1 == 0x03 && ack_d2 == 0xC1) ack_what = "SETPOINT";
        else if (ack_d1 == 0x00 && ack_d2 == 0x41) ack_what = "POWER";
        else if (ack_d1 == 0x00 && ack_d2 == 0x5F) ack_what = "DEMAND";
        ESP_LOGV(TAG, "    ACK for %s(%02X:%02X)", ack_what, ack_d1, ack_d2);
      } else if (dtype == 0x005F) {
        // 0-10V demand command
        uint8_t demand = frame->raw[9];
        ESP_LOGV(TAG, "    DEMAND cmd=%u/15 -> setpoint=%d°C", demand, demand > 0 ? 10 + demand * 3 : 20);
      } else if (dtype == 0x009F) {
        // 0-10V status
        uint8_t demand = frame->raw[10];
        uint8_t min_temp = frame->raw[11];
        ESP_LOGV(TAG, "    DEMAND status=%u/15 min_temp=%u°C -> setpoint=%d°C",
                 demand, min_temp, demand > 0 ? 10 + demand * 3 : 20);
      } else if (dtype == 0x00E8 && frame_len >= 10) {
        uint8_t subtype = frame->raw[9];
        if (frame_type == 0x15) {
          ESP_LOGV(TAG, "    DATA_REQ E8:%02X (%s)", subtype,
                   subtype == 0xC0 ? "temperatures" : (subtype == 0xC1 ? "operating hours" : "???"));
        } else if (frame_type == 0x18 && subtype == 0xC1 && frame_len >= 26) {
          uint16_t comp_h = (frame->raw[18] << 8) | frame->raw[19];
          uint16_t pump_h = (frame->raw[20] << 8) | frame->raw[21];
          uint16_t heat_h = (frame->raw[26] << 8) | frame->raw[27];
          ESP_LOGV(TAG, "    E8:C1 compressor=%uh waterpump=%uh backup_heater=%uh", comp_h, pump_h, heat_h);
          // Dump all data bytes for further analysis
          std::string data_hex;
          char hbuf[4];
          for (size_t i = 12; i < fsz - 2; i++) {
            snprintf(hbuf, sizeof(hbuf), "%02X ", frame->raw[i]);
            data_hex += hbuf;
          }
          ESP_LOGV(TAG, "    E8:C1 data[12..]: %s", data_hex.c_str());
        } else if (frame_type == 0x18 && subtype == 0xC0) {
          // E8:C0 temperature response — dump all bytes with temp decode attempt
          std::string decode;
          char dbuf[32];
          for (size_t i = 12; i < fsz - 2; i++) {
            float t = frame->raw[i] / 2.0f - 16.0f;
            snprintf(dbuf, sizeof(dbuf), "[%zu]=0x%02X(%.1f°C) ", i, frame->raw[i], t);
            decode += dbuf;
          }
          ESP_LOGV(TAG, "    E8:C0 data: %s", decode.c_str());
        }
      } else if (frame_type == 0x10) {
        // Heartbeat — just note it
        ESP_LOGV(TAG, "    heartbeat from master");
      } else if (dtype == 0x0049 || (frame_len >= 8 && frame->raw[8] == 0x49)) {
        // Possible error/alarm frame
        ESP_LOGV(TAG, "    ALARM/ERROR dtype=%02X:%02X", frame->raw[7], frame->raw[8]);
        for (size_t i = 9; i < fsz - 2 && i < (size_t)(frame_len + 2); i++) {
          ESP_LOGV(TAG, "    data[%zu]=0x%02X", i, frame->raw[i]);
        }
      } else {
        // Unknown dtype — dump payload
        if (frame_len > 7) {
          std::string payload_hex;
          char pbuf[4];
          for (size_t i = 9; i < fsz - 2; i++) {
            snprintf(pbuf, sizeof(pbuf), "%02X ", frame->raw[i]);
            payload_hex += pbuf;
          }
          ESP_LOGV(TAG, "    payload: %s", payload_hex.c_str());
        }
      }
    }

    // Heartbeat (0x10) — mark as connected
    if (frame_type == 0x10) {
      last_master_alive_millis_ = millis();
      if (this->connected_binary_sensor_) {
        this->connected_binary_sensor_->publish_state(true);
      }
    }

    // Master Status broadcast (0x58) with dtype 03:C6 — main status with temperatures
    // Layout after A0:00 prefix (raw[]):
    //   [0]=type [1]=len [2]=00 [3:4]=src [5:6]=dst
    //   [7:8]=dtype(03:C6) [9]=flags [10]=mode [11]=???
    //   [12]=current_temp [13]=setpoint [14]=outdoor_temp
    //   [15:17]=repeat of [12:14]
    if (frame_type == 0x58 && frame_len >= 15 && frame->raw[7] == 0x03 && frame->raw[8] == 0xC6) {
      uint8_t flags = frame->raw[9];
      uint8_t estia_mode = frame->raw[10];
      float current_temp = frame->raw[12] / 2.0f - 16.0f;
      float setpoint = frame->raw[13] / 2.0f - 16.0f;
      float outdoor_temp = frame->raw[14] / 2.0f - 16.0f;

      bool power_on = (flags & 0x01) != 0;
      bool is_cooling = (flags & 0x20) != 0;
      bool is_heating = (flags & 0x40) != 0;

      ESP_LOGV(TAG, "Status: power=%s flags=0x%02X %s current=%.1f°C setpoint=%.1f°C outdoor=%.1f°C",
               power_on ? "ON" : "OFF", flags,
               is_cooling ? "COOL" : (is_heating ? "HEAT" : "???"),
               current_temp, setpoint, outdoor_temp);

      // Initial status on first 0x58 after network is ready
      if (!estia_was_connected_ && network::is_connected()) {
        const char *mode_str = is_cooling ? "COOL" : (is_heating ? "HEAT" : "???");
        ESP_LOGI(TAG, "Heat pump connected — power: %s, mode: %s, setpoint: %.1f°C, current: %.1f°C, outdoor: %.1f°C",
                 power_on ? "ON" : "OFF", mode_str, setpoint, current_temp, outdoor_temp);
        estia_was_connected_ = true;
      }

      int changes = 0;

      // Mode
      climate::ClimateMode new_mode;
      if (!power_on) {
        new_mode = climate::CLIMATE_MODE_OFF;
      } else if (is_cooling) {
        new_mode = climate::CLIMATE_MODE_COOL;
      } else {
        new_mode = climate::CLIMATE_MODE_HEAT;
      }
      if (this->mode != new_mode) {
        const char *mode_str = (new_mode == climate::CLIMATE_MODE_OFF) ? "OFF" :
                               (new_mode == climate::CLIMATE_MODE_COOL) ? "COOL" : "HEAT";
        ESP_LOGI(TAG, "Status: mode=%s", mode_str);
        this->mode = new_mode;
        changes++;
      }

      // Target temperature
      if (this->target_temperature != setpoint) {
        ESP_LOGI(TAG, "Status: setpoint zone 1=%.1f°C", setpoint);
        this->target_temperature = setpoint;
        changes++;
      }

      // Current temperature (Vorlauf)
      if (this->current_temperature != current_temp) {
        this->current_temperature = current_temp;
        changes++;
      }

      // Track last active mode for power-on with mode change
      if (power_on) {
        estia_last_active_mode_ = new_mode;
      }

      if (changes > 0) {
        this->publish_state();
      }

      // Outdoor temperature sensor
      if (this->outdoor_temp_sensor_ != nullptr) {
        this->outdoor_temp_sensor_->publish_state(outdoor_temp);
      }

      last_master_alive_millis_ = millis();
      if (this->connected_binary_sensor_) {
        this->connected_binary_sensor_->publish_state(true);
      }
    }

    // State change broadcast (0x1C) with dtype 03:C6 — immediate update after power/mode/setpoint change
    // Layout same as 0x58: [9]=flags [12]=current [13]=setpoint [14]=outdoor
    if (frame_type == 0x1C && frame_len >= 15 && frame->raw[7] == 0x03 && frame->raw[8] == 0xC6) {
      uint8_t flags = frame->raw[9];
      bool power_on = (flags & 0x01) != 0;
      bool is_cooling = (flags & 0x20) != 0;
      bool is_heating = (flags & 0x40) != 0;
      float setpoint = frame->raw[13] / 2.0f - 16.0f;

      climate::ClimateMode new_mode;
      if (!power_on) {
        new_mode = climate::CLIMATE_MODE_OFF;
      } else if (is_cooling) {
        new_mode = climate::CLIMATE_MODE_COOL;
      } else {
        new_mode = climate::CLIMATE_MODE_HEAT;
      }

      ESP_LOGV(TAG, "State change: flags=0x%02X power=%s %s setpoint=%.1f°C",
               flags, power_on ? "ON" : "OFF",
               is_cooling ? "COOL" : (is_heating ? "HEAT" : "???"), setpoint);

      int changes = 0;
      if (this->mode != new_mode) {
        const char *mode_str = (new_mode == climate::CLIMATE_MODE_OFF) ? "OFF" :
                               (new_mode == climate::CLIMATE_MODE_COOL) ? "COOL" : "HEAT";
        ESP_LOGI(TAG, "Status: mode=%s", mode_str);
        this->mode = new_mode;
        changes++;
      }
      if (this->target_temperature != setpoint) {
        ESP_LOGI(TAG, "Status: setpoint zone 1=%.1f°C", setpoint);
        this->target_temperature = setpoint;
        changes++;
      }
      if (changes > 0) {
        this->publish_state();
      }
    }

    // ACK (0x18) with dtype 00:A1 — general ACK handler
    // Format: 18:LL:00:SRC:DST:00:A1:XX:YY:CRC where XX:YY = acknowledged dtype
    if (frame_type == 0x18 && frame_len >= 9 && frame->raw[7] == 0x00 && frame->raw[8] == 0xA1) {
      uint16_t acked_dtype = (frame->raw[9] << 8) | frame->raw[10];
      ESP_LOGD(TAG, "ACK for dtype %02X:%02X", frame->raw[9], frame->raw[10]);

      // Clear pending command if this ACK matches
      if (!estia_pending_cmd_.empty() && acked_dtype == estia_pending_ack_dtype_) {
        ESP_LOGD(TAG, "Command acknowledged (attempt %u)", estia_cmd_attempts_);
        estia_pending_cmd_.clear();
        estia_cmd_attempts_ = 0;
      }

      // Mode ACK triggers deferred power on
      if (acked_dtype == 0x03C0 && this->estia_power_on_pending_) {
        ESP_LOGD(TAG, "Mode ACK received, sending power on");
        this->estia_power_on_pending_ = false;
        this->cancel_timeout("estia_poweron");
        this->send_estia_power(true);
      }
    }

    // E8:C1 data response (0x18) — operating hours
    // Frame: 18:26:00:SRC:DST:00:E8:C1:01:00:DATA...
    // DATA offsets (16-bit big-endian, in hours):
    //   [6-7] = Heating_Compressor_Hours
    //   [8-9] = WaterPump_Hours
    //   [14-15] = BackupHeater_Hours
    if (frame_type == 0x18 && frame_len >= 26 &&
        frame->raw[7] == 0x00 && frame->raw[8] == 0xE8 &&
        frame->raw[9] == 0xC1) {
      // raw[]: 18:26:00:src:src:dst:dst:00:E8:C1:01:00:data...
      // Confirmed offsets (raw[]): [18:19]=compressor, [20:21]=waterpump, [26:27]=backup
      uint16_t compressor_h = (frame->raw[18] << 8) | frame->raw[19];
      uint16_t waterpump_h  = (frame->raw[20] << 8) | frame->raw[21];
      uint16_t backup_h     = (frame->raw[26] << 8) | frame->raw[27];

      ESP_LOGD(TAG, "E8:C1: compressor=%uh waterpump=%uh backup_heater=%uh",
               compressor_h, waterpump_h, backup_h);

      if (this->compressor_hours_sensor_ != nullptr) {
        this->compressor_hours_sensor_->publish_state(compressor_h);
      }
      if (this->waterpump_hours_sensor_ != nullptr) {
        this->waterpump_hours_sensor_->publish_state(waterpump_h);
      }
      if (this->backup_heater_hours_sensor_ != nullptr) {
        this->backup_heater_hours_sensor_->publish_state(backup_h);
      }
    }

    // 0-10V interface status (0x55 from 0x0041, dtype 00:9F)
    // Frame: 55:0C:00:00:41:08:00:00:9F:00:DD:14:00:00  (DD=demand, 0x14=min temp config)
    if (frame_type == 0x55 && frame->raw[3] == 0x00 && frame->raw[4] == 0x41 &&
        frame->raw[7] == 0x00 && frame->raw[8] == 0x9F) {
      uint8_t demand = frame->raw[10];
      ESP_LOGV(TAG, "0-10V status: demand=%u/15", demand);
      if (demand != estia_demand_value_) {
        estia_demand_value_ = demand;
        ESP_LOGI(TAG, "Status: demand=%u/15 (setpoint %d°C)", demand, demand > 0 ? 10 + demand * 3 : 20);
      }
      if (this->demand_sensor_ != nullptr) {
        this->demand_sensor_->publish_state(demand);
      }
    }

    // 0-10V interface command (0x11 from 0x0041, dtype 00:5F) — immediate demand update
    // Frame: 11:0A:00:00:41:08:00:00:5F:DD:00:00
    if (frame_type == 0x11 && frame->raw[3] == 0x00 && frame->raw[4] == 0x41 &&
        frame->raw[7] == 0x00 && frame->raw[8] == 0x5F) {
      uint8_t demand = frame->raw[9];
      ESP_LOGV(TAG, "0-10V command: demand=%u/15", demand);
      if (demand != estia_demand_value_) {
        estia_demand_value_ = demand;
        ESP_LOGI(TAG, "Status: demand=%u/15 (setpoint %d°C)", demand, demand > 0 ? 10 + demand * 3 : 20);
      }
      if (this->demand_sensor_ != nullptr) {
        this->demand_sensor_->publish_state(demand);
      }
    }

    // Alarm/error frames (dtype xx:49) — log at WARN level
    if (frame_len >= 8 && (frame->raw[8] == 0x49 || frame->raw[7] == 0x49)) {
      uint8_t error_byte = (frame_len > 9) ? frame->raw[9] : 0;
      const char *error_desc = "unknown";
      if (error_byte >= 0x01 && error_byte <= 0x0D) error_desc = "hydro unit (A-series)";
      else if (error_byte >= 0x41 && error_byte <= 0x52) error_desc = "communication (E-series)";
      else if (error_byte >= 0x63 && error_byte <= 0x7F) error_desc = "sensor (F-series)";
      else if (error_byte >= 0x81 && error_byte <= 0x84) error_desc = "compressor (H-series)";
      else if (error_byte >= 0xC2 && error_byte <= 0xDD) error_desc = "configuration (L-series)";
      else if (error_byte >= 0xE3 && error_byte <= 0xFF) error_desc = "protection (P-series)";
      ESP_LOGW(TAG, "ALARM: error=0x%02X (%s)", error_byte, error_desc);
    }

    return true;
  }
  DataFrame auto_detected_frame{};
  if (this->should_auto_detect_frame_format_() && this->is_hm_wire_frame_from_master_(frame)) {
    auto_detected_frame = *frame;
    if (this->maybe_auto_detect_hm_frame_(&auto_detected_frame)) {
      frame = &auto_detected_frame;
    }
  }

  if (!frame->is_tu2c() && frame->crc() != frame->calculate_crc()) {
    if (this->failed_crcs_sensor_ != nullptr) {
      this->failed_crcs_sensor_->publish_state(this->failed_crcs_sensor_->state + 1);
    }
    // Classic TCC-Link: the XOR CRC is the unit's actual algorithm, so a
    // mismatch means the frame is corrupt — drop it. The HM variant uses
    // a different (still-undecoded) CRC algorithm, so every frame fails
    // the XOR check by design; tolerate the mismatch and process anyway.
    if (!this->is_hm_variant()) {
      ESP_LOGW(TAG, "CRC check failed");
      log_data_frame("Failed frame", frame);
      return false;
    }
  }
  // >>> Drop our own TX echo frames (remote-labeled, identical bytes)
  if (this->is_own_tx_echo_(frame)) {
    ESP_LOGV(TAG, "Echo detected and ignored: [%s]", frame_to_hex_string(frame).c_str());
    return true;  // swallow quietly
  }

  this->handle_pending_command_ack_(frame);

  // still notify any listeners of real frames
  this->set_data_received_callback_.call(frame);
  if (frame->is_tu2c()) {
    if (this->data_reader.frame_format() == FrameFormat::ESTIA) {
      process_received_data_estia_first_gen_(frame);
    } else {
      process_received_data_tu2c_(frame);
    }
  } else {
    process_received_data(frame);
  }
  return true;
}


void ToshibaAbClimate::loop() {
  // TODO: check if last_unconfirmed_command_ was not confirmed after a timeout
  // and log warning/error

  // Drain the pending sensor-query queue at most one per loop iteration.
  // This must run BEFORE the TX dispatch below so an enqueued 0x17 has a
  // chance to be picked up in the same tick.
  this->drain_sensor_query_queue_();

  const uint32_t now = millis();
  const bool bus_can_send = this->has_bus_quiet_time_elapsed_(now);

  if (bus_can_send) {
    optional<DataFrame> frame_to_send{};

    if (this->last_unconfirmed_command_.has_value()) {
      const bool pending_is_tu2c = this->last_unconfirmed_command_->is_tu2c();
      const bool retry_due = pending_is_tu2c
                                 ? (this->last_unconfirmed_command_sent_ms_ != 0 &&
                                    (millis() - this->last_unconfirmed_command_sent_ms_) >= TU2C_ACK_TIMEOUT_MS)
                                 : this->resend_last_unconfirmed_command_;
      const uint8_t max_attempts = pending_is_tu2c ? TU2C_MAX_COMMAND_SEND_ATTEMPTS : MAX_COMMAND_SEND_ATTEMPTS;
      if (retry_due) {
        if (this->last_unconfirmed_command_attempts_ >= max_attempts) {
        ESP_LOGE(TAG, "Command opcode 0x%02X not acknowledged after %u attempts", this->last_unconfirmed_command_->opcode1,
                 static_cast<unsigned>(this->last_unconfirmed_command_attempts_));
        this->last_unconfirmed_command_.reset();
        this->last_unconfirmed_command_attempts_ = 0;
        this->resend_last_unconfirmed_command_ = false;
        this->last_unconfirmed_command_sent_ms_ = 0;
        } else {
          frame_to_send = this->last_unconfirmed_command_.value();
          this->last_unconfirmed_command_attempts_++;
          this->resend_last_unconfirmed_command_ = false;
          if (frame_to_send->is_tu2c()) {
            ESP_LOGD(TAG, "Resending TU2C command after ACK timeout (attempt %u/%u)",
                     static_cast<unsigned>(this->last_unconfirmed_command_attempts_),
                     static_cast<unsigned>(max_attempts));
          } else {
            ESP_LOGD(TAG, "Resending command opcode 0x%02X (attempt %u/%u)", frame_to_send->opcode1,
                     static_cast<unsigned>(this->last_unconfirmed_command_attempts_),
                     static_cast<unsigned>(max_attempts));
          }
        }
      }
    }

    if (!frame_to_send.has_value() && !this->raw_write_queue_.empty()) {
      last_sent_frame_millis_ = now;
      auto raw_frame = this->raw_write_queue_.front();
      this->raw_write_queue_.pop();

      std::string payload;
      payload.reserve(raw_frame.empty() ? 0 : raw_frame.size() * 3 - 1);
      char buf[3];
      for (size_t i = 0; i < raw_frame.size(); i++) {
        if (i > 0) {
          payload += ':';
        }
        std::snprintf(buf, sizeof(buf), "%02X", raw_frame[i]);
        payload += buf;
      }

      ESP_LOGD(TAG, "Write raw frame: %s", payload.c_str());
      this->remember_tx_frame_for_echo_(raw_frame.data(), raw_frame.size(), false);
      this->write_array(raw_frame.data(), raw_frame.size());

      if (this->raw_write_queue_.empty() && this->write_queue_.empty()) {
        ESP_LOGV(TAG, "All frames written");
      }
    }

    if (!frame_to_send.has_value() && !this->last_unconfirmed_command_.has_value() && !this->write_queue_.empty()) {
      frame_to_send = this->write_queue_.front();
      this->write_queue_.pop();
      if (this->should_track_command_ack_(frame_to_send.value())) {
        this->last_unconfirmed_command_ = frame_to_send.value();
        this->last_unconfirmed_command_attempts_ = 1;
        this->resend_last_unconfirmed_command_ = false;
        this->last_unconfirmed_command_sent_ms_ = now;
      } else {
        this->last_unconfirmed_command_.reset();
        this->last_unconfirmed_command_attempts_ = 0;
        this->resend_last_unconfirmed_command_ = false;
        this->last_unconfirmed_command_sent_ms_ = 0;
      }
    }

    if (frame_to_send.has_value()) {
      if (frame_to_send->is_tu2c() && this->data_reader.frame_format() == FrameFormat::TU2C &&
          !this->has_tu2c_quiet_time_elapsed_(now)) {
          return;
      }
      last_sent_frame_millis_ = now;
      if (frame_to_send->is_tu2c()) {
        last_tu2c_sent_frame_millis_ = last_sent_frame_millis_;
      }
      auto frame = frame_to_send.value();
      if (frame.is_tu2c()) {
        log_tu2c_data_frame("Write frame", &frame);
      } else {
        log_data_frame("Write frame", &frame);
      }
      this->remember_tx_frame_for_echo_(frame.raw, frame.size(), frame.is_tu2c());

      auto log_tx_bytes = [this](const uint8_t *bytes, size_t size) {
        std::string payload;
        payload.reserve(size ? (size * 3 - 1) : 0);
        char buf[3];
        for (size_t i = 0; i < size; i++) {
          if (i > 0) {
            payload += ':';
          }
          std::snprintf(buf, sizeof(buf), "%02X", bytes[i]);
          payload += buf;
        }
        ESP_LOGV(TAG, "TX bytes: %s", payload.c_str());
      };

      if (frame.is_tu2c()) {
        const size_t raw_size = frame.size();
        uint8_t tu2c_frame[DATA_FRAME_MAX_SIZE + 3];
        tu2c_frame[0] = 0xF0;
        tu2c_frame[1] = 0xF0;
        if (raw_size + 3 <= sizeof(tu2c_frame)) {
          std::memcpy(&tu2c_frame[2], frame.raw, raw_size);
          tu2c_frame[2 + raw_size] = 0xA0;
          log_tx_bytes(tu2c_frame, raw_size + 3);
          this->write_array(tu2c_frame, raw_size + 3);
        } else {
          ESP_LOGW(TAG, "TU2C frame too large to send (size=%u)", static_cast<unsigned>(raw_size));
        }
      } else {
        log_tx_bytes(frame.raw, frame.size());
        this->write_array(frame.raw, frame.size());
      }

      if (this->write_queue_.empty()) {
        ESP_LOGV(TAG, "All frames written");
      }
    }
  }

  // Estia command ACK timeout and retry
  if (!estia_pending_cmd_.empty() && (millis() - estia_cmd_sent_ms_) > ESTIA_CMD_ACK_TIMEOUT_MS) {
    if (estia_cmd_attempts_ >= ESTIA_MAX_CMD_ATTEMPTS) {
      ESP_LOGW(TAG, "Command not acknowledged after %u attempts (dtype %02X:%02X)",
               estia_cmd_attempts_,
               (estia_pending_ack_dtype_ >> 8) & 0xFF, estia_pending_ack_dtype_ & 0xFF);
      estia_pending_cmd_.clear();
      estia_cmd_attempts_ = 0;
    } else {
      estia_cmd_attempts_++;
      ESP_LOGD(TAG, "Command retry %u/%u (dtype %02X:%02X)",
               estia_cmd_attempts_, ESTIA_MAX_CMD_ATTEMPTS,
               (estia_pending_ack_dtype_ >> 8) & 0xFF, estia_pending_ack_dtype_ & 0xFF);
      estia_cmd_sent_ms_ = millis();
      this->raw_write_queue_.push(estia_pending_cmd_);
    }
  }

  // Estia autonomous polling (runtime-toggleable)
  if (this->autonomous_ && this->data_reader.frame_format() == FrameFormat::A0) {
    uint32_t now = millis();
    if (now - estia_last_e8c0_ms_ >= ESTIA_E8C0_INTERVAL_MS) {
      estia_last_e8c0_ms_ = now;
      this->send_estia_data_request(0xC0);
    }
    if (now - estia_last_e8c1_ms_ >= ESTIA_E8C1_INTERVAL_MS) {
      estia_last_e8c1_ms_ = now;
      this->send_estia_data_request(0xC1);
    }
  }

  // 0-10V demand emulation: periodic heartbeat as 0x0041
  if (this->demand_enabled_ && this->data_reader.frame_format() == FrameFormat::A0) {
    uint32_t now = millis();
    if (now - estia_last_demand_heartbeat_ms_ >= ESTIA_DEMAND_HEARTBEAT_MS) {
      estia_last_demand_heartbeat_ms_ = now;
      this->send_estia_demand_heartbeat();
    }
  }

  if (this->ext_temp_sensor_ && this->ext_temp_sensor_->has_state()) {
    if (millis() - last_temp_log_time_ >= 120000) {
      ESP_LOGI(TAG, "Ambient Temp: %.2f °C", this->ext_temp_sensor_->state);
      float current = std::round(this->ext_temp_sensor_->state * 2.0f) / 2.0f;  // round to 0.5°C
      if (std::isnan(last_sent_temp_) || std::abs(current - last_sent_temp_) >= 0.5f) {
        last_sent_temp_ = current; 
      }
      last_temp_log_time_ = millis();
    }
  }



  uint8_t bytes_read = 0;

  // Cap bytes drained per loop() tick. HM-variant master broadcasts can be
  // 60-80 bytes (≈370 ms of wire time at 2400 baud) — well past the
  // 30 ms-per-component budget. Unbounded draining keeps loop() inside the
  // soft-serial RX path long enough to starve lwIP/Wi-Fi and trip the WDT
  // (observed PCs all land at 0x401037xx, soft-serial ISR). 32 bytes ≈
  // 150 ms of wire time — bounded worst case; unread bytes wait in the
  // 2048-byte UART buffer for the next tick.
  constexpr uint8_t MAX_BYTES_PER_LOOP = 32;
  uint8_t loop_budget = MAX_BYTES_PER_LOOP;

  // Read from hardware UART0 RX when enabled (no busy-wait), else from the
  // ESPHome software-serial parent (existing behavior).
  while (loop_budget > 0) {
#ifdef USE_ESP8266
    const bool have = this->hw_uart_rx_enabled_ ? (s_bus_serial.available() > 0) : (this->available() > 0);
#else
    const bool have = this->available() > 0;
#endif
    if (!have)
      break;
    loop_budget--;
#ifdef USE_ESP8266
    int byte = this->hw_uart_rx_enabled_ ? s_bus_serial.read() : this->read();
#else
    int byte = this->read();
#endif
    if (byte >= 0) {
      bytes_read++;

      if (!can_read_packet)
        continue;  // wait until can read packet

      this->watch_auto_frame_format_byte_(static_cast<uint8_t>(byte));

      if (data_reader.put(byte)) {
        // packet complete

        last_received_frame_millis_ = millis();

        auto frame = data_reader.frame;
        if (frame.is_tu2c()) {
          last_tu2c_received_frame_millis_ = last_received_frame_millis_;
        }

        if (!receive_data_frame(&frame)) {
        }

        data_reader.reset();

        // read next packet (if any in the next loop)
        // the smallest packet (ALIVE) is 32ms wide,
        // which means there are max ~31 packets per second.
        // and the loop runs 33-50 times per second.
        // so should be enough throughput to process packets.
        // this ensure that each packet is interpreted separately
        break;
      }
    } else {
      ESP_LOGW(TAG, "Unable to read data");
    }
  }

  App.feed_wdt();

  if (bytes_read > 0) {
    loops_with_reads_++;
    loops_without_reads_ = 0;

    // ESP_LOGV(TAG, "Bytes of data read: %d", bytes_read);
    // if (!data_reader.complete) {
    //   log_data_frame("Pending", data_reader.frame);
    // }

    last_read_millis_ = millis();
  } else {
    loops_without_reads_++;
    loops_with_reads_ = 0;

    if (last_read_millis_ > 0) {
      auto millis_since_last_read = millis() - last_read_millis_;
      if (millis_since_last_read >= this->packet_min_wait_millis_) {
        // can start reading packet

        if (!data_reader.complete && data_reader.data_index_ > 0) {
          // ESP_LOGW(TAG, "Reset pending frame buffer (%d)",
          // data_reader.data_index_); log_raw_data("Pending: ",
          // data_reader.frame.raw, data_reader.data_index_);
        }
        can_read_packet = true;
        data_reader.reset();
        last_read_millis_ = 0;
      }
    }
  }

  if (last_master_alive_millis_ > 0 && (millis() - last_master_alive_millis_) > LAST_ALIVE_TIMEOUT_MILLIS) {
    if (estia_was_connected_) {
      ESP_LOGW(TAG, "Heat pump disconnected");
      estia_was_connected_ = false;
    }
    if (this->connected_binary_sensor_) {
      this->connected_binary_sensor_->publish_state(false);
    }
  }
}

size_t ToshibaAbClimate::send_new_state(const struct TccState *new_state) {
  auto commands = create_commands(new_state);
  if (commands.empty()) {
    ESP_LOGD(TAG, "New state has not changed. Nothing to send");
  } else {
    ESP_LOGD(TAG, "Send %d commands", commands.size());
    for (auto cmd : commands) {
      send_command(cmd);
    }
  }

  return commands.size();
}

std::vector<DataFrame> ToshibaAbClimate::create_commands(const struct TccState *new_state) {
  auto commands = std::vector<DataFrame>();
  const bool use_tu2c = this->data_reader.frame_format() == FrameFormat::TU2C;

  if (new_state->power != tcc_state.power) {
    if (new_state->power) {
      // turn on
      ESP_LOGD(TAG, "Turning on");
      auto command = DataFrame{};
      write_set_parameter_power(&command, this->remote_address_, this->master_address_, this->command_mode_read_, new_state);
      commands.push_back(command);
    } else {
      // turn off
      ESP_LOGD(TAG, "Turning off");
      auto command = DataFrame{};
      if (use_tu2c) {
        write_power_off_tu2c(&command, this->tu2c_remote_address_, this->tu2c_master_address_);
      } else {
        write_set_parameter_power(&command, this->remote_address_, this->master_address_, this->command_mode_read_, new_state);
      }
      commands.push_back(command);
      // don't process other changes when turning off
      return commands;
    }
  }

  if (new_state->mode != tcc_state.mode) {
    ESP_LOGD(TAG, "Changing mode");
    auto command = DataFrame{};
    write_set_parameter_mode(&command, this->remote_address_, this->master_address_, this->command_mode_read_, new_state);
    commands.push_back(command);
  }

  if (new_state->fan != tcc_state.fan) {
    ESP_LOGD(TAG, "Changing fan");
    auto command = DataFrame{};
    if (use_tu2c) {
      write_set_parameter_flags_tu2c(&command, this->tu2c_remote_address_, this->tu2c_master_address_, new_state,
                                        COMMAND_SET_FAN);
    } else {
      write_set_parameter_flags(&command, this->remote_address_, this->master_address_, this->command_mode_read_, new_state, COMMAND_SET_FAN, this->is_hm_variant());
    }
    commands.push_back(command);
  }

  if (new_state->target_temp != tcc_state.target_temp) {
    ESP_LOGD(TAG, "Changing target temperature");
    auto command = DataFrame{};
    if (use_tu2c) {
      write_set_parameter_flags_tu2c(&command, this->tu2c_remote_address_, this->tu2c_master_address_, new_state,
                                        COMMAND_SET_TEMP);
    } else {
      write_set_parameter_flags(&command, this->remote_address_, this->master_address_, this->command_mode_read_, new_state, COMMAND_SET_TEMP, this->is_hm_variant());
    }
    commands.push_back(command);
  }

  if (new_state->vent != tcc_state.vent) {
    ESP_LOGD(TAG, "Changing vent");
    auto command = DataFrame{};
    write_set_parameter_vent(&command, this->remote_address_, this->master_address_, this->command_mode_read_, new_state);
    commands.push_back(command);
  }

  return commands;
}

void ToshibaAbClimate::control(const climate::ClimateCall &call) {
  // First-generation Estia: the climate entity represents domestic hot water.
  // HEAT/OFF therefore maps to hot-water on/off, not space-heating auto mode.
  if (this->data_reader.frame_format() == FrameFormat::ESTIA) {
    if (call.get_mode().has_value()) {
      auto new_mode = call.get_mode().value();
      if (new_mode == climate::CLIMATE_MODE_HEAT) {
        this->send_estia_first_gen_dhw_on();
      } else if (new_mode == climate::CLIMATE_MODE_OFF) {
        this->send_estia_first_gen_dhw_off();
      }
    }
    if (call.get_target_temperature().has_value()) {
      this->send_estia_first_gen_dhw_setpoint(call.get_target_temperature().value());
    }
    return;
  }

  if (this->data_reader.frame_format() == FrameFormat::A0) {
    if (call.get_mode().has_value()) {
      ESP_LOGD(TAG, "Control: mode=%s", LOG_STR_ARG(climate::climate_mode_to_string(*call.get_mode())));
    }
    if (call.get_target_temperature().has_value()) {
      ESP_LOGD(TAG, "Control: setpoint zone 1=%.1f°C", *call.get_target_temperature());
    }
    if (call.get_mode().has_value()) {
      auto new_mode = call.get_mode().value();
      if (new_mode == climate::CLIMATE_MODE_OFF) {
        this->send_estia_power(false);
      } else if (this->mode == climate::CLIMATE_MODE_OFF) {
        if (new_mode != estia_last_active_mode_) {
          // Mode differs: send mode command first, power on after ACK
          // Retry mode command up to 3 times if no ACK received
          estia_pending_mode_cmd_ = (new_mode == climate::CLIMATE_MODE_COOL) ? 0x01 : 0x02;
          estia_power_on_pending_ = true;
          estia_mode_retries_ = 0;
          this->send_estia_mode(estia_pending_mode_cmd_);
          this->set_timeout("estia_poweron", 3000, [this]() {
            this->estia_mode_retry_timeout_();
          });
        } else {
          // Same mode: just power on
          this->send_estia_power(true);
        }
      } else {
        // Already on → switch mode (HEAT↔COOL)
        if (new_mode == climate::CLIMATE_MODE_HEAT) {
          this->send_estia_mode(0x02);
        } else if (new_mode == climate::CLIMATE_MODE_COOL) {
          this->send_estia_mode(0x01);
        }
      }
    }
    if (call.get_target_temperature().has_value()) {
      float temp = call.get_target_temperature().value();
      this->send_estia_setpoint(temp);
    }
    return;
  }

  TccState new_state = TccState{tcc_state};

  if (call.get_mode().has_value()) {
    ESP_LOGD(TAG, "Control mode");
    auto mode = call.get_mode().value();
    new_state.power = to_tcc_power(mode);
    new_state.mode = to_tcc_mode(mode);
  }

  if (call.get_fan_mode().has_value()) {
    ESP_LOGD(TAG, "Control fan");
    new_state.fan = to_tcc_fan(call.get_fan_mode().value());
  }

  if (call.get_target_temperature().has_value()) {
    ESP_LOGD(TAG, "Control target temperature");
    new_state.target_temp = call.get_target_temperature().value();
  }

  send_new_state(&new_state);
}

void ToshibaAbClimate::send_command(const struct DataFrame command) {
  // Read-only mode: do not send any commands
  if (this->read_only_) {
    ESP_LOGW(TAG, "Read-only mode enabled: dropping command");
    return;
  }
  if (this->autonomous_) {
    if (this->data_reader.frame_format() == FrameFormat::TU2C || this->data_reader.frame_format() == FrameFormat::ESTIA) {
      if (this->data_reader.frame_format() == FrameFormat::TU2C && !this->announce_ack_received_ &&
          !this->is_tu2c_registration_query_(command)) {
        ESP_LOGW(TAG, "Dropping TU2C command while awaiting registration ACK (autonomous mode)");
        return;
      }
    } else if (!this->announce_ack_received_) {
      bool is_announce = false;
      if (command.source == this->remote_address_ && command.dest == TOSHIBA_BROADCAST &&
          command.opcode1 == OPCODE_ERROR_HISTORY && command.data_length == 2 &&
          command.data[1] == 0x0D) {
        is_announce = true;
      }

      if (!is_announce) {
        ESP_LOGW(TAG, "Dropping command while awaiting announce ACK (autonomous mode)");
        return;
      }
    }
  }

  if (command.is_tu2c()) {
    log_tu2c_data_frame("Enqueue command", &command);
  } else {
    log_data_frame("Enqueue command", &command);
  }
  this->write_queue_.push(command);
}

bool ToshibaAbClimate::send_raw_frame_from_text(const std::string &frame_text) {
  std::vector<uint8_t> raw_frame;
  raw_frame.reserve(frame_text.size() / 2 + 1);

  auto is_hex = [](char c) { return std::isxdigit(static_cast<unsigned char>(c)) != 0; };

  size_t i = 0;
  while (i < frame_text.size()) {
    while (i < frame_text.size() && std::isspace(static_cast<unsigned char>(frame_text[i])) != 0) {
      i++;
    }

    if (i >= frame_text.size()) {
      break;
    }

    if (i + 1 >= frame_text.size() || !is_hex(frame_text[i]) || !is_hex(frame_text[i + 1])) {
      ESP_LOGW(TAG, "Invalid raw frame format at position %u: %s", static_cast<unsigned>(i), frame_text.c_str());
      return false;
    }

    const std::string byte_str = frame_text.substr(i, 2);
    raw_frame.push_back(static_cast<uint8_t>(std::strtoul(byte_str.c_str(), nullptr, 16)));
    i += 2;

    while (i < frame_text.size() && std::isspace(static_cast<unsigned char>(frame_text[i])) != 0) {
      i++;
    }

    if (i < frame_text.size()) {
      if (frame_text[i] != ':') {
        ESP_LOGW(TAG, "Invalid raw frame separator at position %u: %s", static_cast<unsigned>(i), frame_text.c_str());
        return false;
      }
      i++;
    }
  }

  if (raw_frame.empty()) {
    ESP_LOGW(TAG, "Raw frame is empty, nothing queued");
    return false;
  }

  std::string payload;
  payload.reserve(raw_frame.size() * 3 - 1);
  char buf[3];
  for (size_t idx = 0; idx < raw_frame.size(); idx++) {
    if (idx > 0) {
      payload += ':';
    }
    std::snprintf(buf, sizeof(buf), "%02X", raw_frame[idx]);
    payload += buf;
  }

  ESP_LOGI(TAG, "Queue raw frame from HA: %s", payload.c_str());
  this->raw_write_queue_.push(std::move(raw_frame));
  return true;
}

// ── Estia A0-protocol: build and enqueue raw frame ──

static uint16_t estia_crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 1)
        crc = (crc >> 1) ^ 0x8408;
      else
        crc >>= 1;
    }
  }
  return crc;
}

void ToshibaAbClimate::send_estia_tracked_(const uint8_t *frame, size_t len, uint16_t ack_dtype) {
  std::vector<uint8_t> raw(frame, frame + len);
  this->raw_write_queue_.push(raw);
  // Track for ACK/retry (only for commands, not heartbeats/requests)
  if (ack_dtype != 0) {
    estia_pending_cmd_ = std::move(raw);
    estia_pending_ack_dtype_ = ack_dtype;
    estia_cmd_attempts_ = 1;
    estia_cmd_sent_ms_ = millis();
  }
}

void ToshibaAbClimate::send_estia_setpoint(float target_temp) {
  if (this->read_only_) {
    ESP_LOGW(TAG, "Read-only mode: not sending Estia setpoint");
    return;
  }

  // Encode temperature: val = (°C + 16) * 2
  uint8_t encoded = static_cast<uint8_t>((target_temp + 16.0f) * 2.0f);
  uint16_t src = this->estia_source_address_;

  // Sub-command: 0x02 = heating setpoint, 0x01 = cooling setpoint
  uint8_t subcmd = (this->mode == climate::CLIMATE_MODE_COOL) ? 0x01 : 0x02;

  // Command frame: A0:00:11:0C:00:SRC_H:SRC_L:08:00:03:C1:SUBCMD:TEMP:00:00:00
  uint8_t frame[] = {
    0xA0, 0x00,                         // prefix
    0x11,                               // type: command
    0x0C,                               // length: 12
    0x00,                               // fixed
    (uint8_t)(src >> 8), (uint8_t)(src & 0xFF),  // source
    0x08, 0x00,                         // dest: master
    0x03, 0xC1,                         // command type: setpoint
    subcmd,                             // 0x02=heat, 0x01=cool
    encoded,                            // target temperature
    0x00, 0x00, 0x00,                   // padding
    0x00, 0x00                          // CRC placeholder
  };

  size_t crc_len = sizeof(frame) - 2;
  uint16_t crc = estia_crc16(frame, crc_len);
  frame[crc_len]     = (crc >> 8) & 0xFF;
  frame[crc_len + 1] = crc & 0xFF;

  ESP_LOGD(TAG, "TX: setpoint=%.1f°C (0x%02X) subcmd=0x%02X", target_temp, encoded, subcmd);
  log_raw_data("Estia TX", frame, sizeof(frame));

  this->send_estia_tracked_(frame, sizeof(frame), 0x03C1);  // ACK: 00:A1:03:C1
}

void ToshibaAbClimate::send_estia_power(bool on) {
  if (this->read_only_) {
    ESP_LOGW(TAG, "Read-only mode: not sending Estia power command");
    return;
  }

  uint16_t src = this->estia_source_address_;
  uint8_t power_cmd = on ? 0x23 : 0x22;

  // Power command: A0:00:11:08:00:SRC:08:00:00:41:CMD:CRC
  // Captured: 11:08:00:00:40:08:00:00:41:22 (power off from remote)
  uint8_t frame[] = {
    0xA0, 0x00,                         // prefix
    0x11,                               // type: command
    0x08,                               // length: 8
    0x00,                               // fixed
    (uint8_t)(src >> 8), (uint8_t)(src & 0xFF),  // source
    0x08, 0x00,                         // dest: master
    0x00, 0x41,                         // dtype: power control
    power_cmd,                          // 0x23=ON, 0x22=OFF
    0x00, 0x00                          // CRC placeholder
  };

  size_t crc_len = sizeof(frame) - 2;
  uint16_t crc = estia_crc16(frame, crc_len);
  frame[crc_len]     = (crc >> 8) & 0xFF;
  frame[crc_len + 1] = crc & 0xFF;

  ESP_LOGD(TAG, "TX: power %s (cmd=0x%02X)", on ? "ON" : "OFF", power_cmd);
  log_raw_data("Estia TX", frame, sizeof(frame));

  this->send_estia_tracked_(frame, sizeof(frame), 0x0041);  // ACK: 00:A1:00:41
}

void ToshibaAbClimate::send_estia_mode(uint8_t mode_cmd) {
  if (this->read_only_) {
    ESP_LOGW(TAG, "Read-only mode: not sending Estia mode command");
    return;
  }

  uint16_t src = this->estia_source_address_;

  // Mode command: A0:00:11:08:00:SRC:08:00:03:C0:CMD:CRC
  // 0x02=heating, 0x01=cooling
  uint8_t frame[] = {
    0xA0, 0x00,                         // prefix
    0x11,                               // type: command
    0x08,                               // length: 8
    0x00,                               // fixed
    (uint8_t)(src >> 8), (uint8_t)(src & 0xFF),  // source
    0x08, 0x00,                         // dest: master
    0x03, 0xC0,                         // dtype: mode control
    mode_cmd,                           // 0x02=heat, 0x01=cool
    0x00, 0x00                          // CRC placeholder
  };

  size_t crc_len = sizeof(frame) - 2;
  uint16_t crc = estia_crc16(frame, crc_len);
  frame[crc_len]     = (crc >> 8) & 0xFF;
  frame[crc_len + 1] = crc & 0xFF;

  ESP_LOGD(TAG, "TX: mode=%s (cmd=0x%02X)", mode_cmd == 0x01 ? "COOL" : "HEAT", mode_cmd);
  log_raw_data("Estia TX", frame, sizeof(frame));

  this->send_estia_tracked_(frame, sizeof(frame), 0x03C0);  // ACK: 00:A1:03:C0
}

void ToshibaAbClimate::estia_mode_retry_timeout_() {
  if (!this->estia_power_on_pending_) return;

  estia_mode_retries_++;
  if (estia_mode_retries_ < 3) {
    ESP_LOGW(TAG, "No mode ACK, retry %d/3", estia_mode_retries_);
    this->send_estia_mode(estia_pending_mode_cmd_);
    this->set_timeout("estia_poweron", 3000, [this]() {
      this->estia_mode_retry_timeout_();
    });
  } else {
    ESP_LOGW(TAG, "No mode ACK after 3 retries, sending power on anyway");
    this->estia_power_on_pending_ = false;
    this->send_estia_power(true);
  }
}

void ToshibaAbClimate::send_estia_demand(uint8_t demand) {
  if (this->read_only_) {
    ESP_LOGW(TAG, "Read-only mode: not sending Estia demand command");
    return;
  }
  if (demand > 15) demand = 15;
  estia_demand_value_ = demand;

  // 0-10V demand command: A0:00:11:0A:00:00:41:08:00:00:5F:DD:00:00:CRC
  // Uses source address 0x0041 (0-10V interface address)
  uint8_t frame[] = {
    0xA0, 0x00,                         // prefix
    0x11,                               // type: command
    0x0A,                               // length: 10
    0x00,                               // fixed
    0x00, 0x41,                         // source: 0-10V interface address
    0x08, 0x00,                         // dest: master
    0x00, 0x5F,                         // dtype: demand control
    demand,                             // demand value (0..15)
    0x00, 0x00,                         // padding
    0x00, 0x00                          // CRC placeholder
  };

  size_t crc_len = sizeof(frame) - 2;
  uint16_t crc = estia_crc16(frame, crc_len);
  frame[crc_len]     = (crc >> 8) & 0xFF;
  frame[crc_len + 1] = crc & 0xFF;

  ESP_LOGV(TAG, "TX: demand=%u/15", demand);
  log_raw_data("Estia TX", frame, sizeof(frame));

  this->send_estia_tracked_(frame, sizeof(frame), 0x005F);  // ACK: 00:A1:00:5F
}

void ToshibaAbClimate::send_estia_demand_heartbeat() {
  // Emulate 0-10V interface periodic status (0x55 from 0x0041)
  // Captured: 55:0C:00:00:41:08:00:00:9F:00:DD:14:00:00:CRC
  // raw[10]=demand, raw[11]=0x14 (min temp config = 20°C)
  uint8_t frame[] = {
    0xA0, 0x00,                         // prefix
    0x55,                               // type: remote status
    0x0C,                               // length: 12
    0x00,                               // fixed
    0x00, 0x41,                         // source: 0-10V interface address
    0x08, 0x00,                         // dest: master
    0x00, 0x9F,                         // dtype: 0-10V status
    0x00, estia_demand_value_,          // 0x00 + demand value
    0x14,                               // min temp config (20°C)
    0x00, 0x00,                         // padding
    0x00, 0x00                          // CRC placeholder
  };

  size_t crc_len = sizeof(frame) - 2;
  uint16_t crc = estia_crc16(frame, crc_len);
  frame[crc_len]     = (crc >> 8) & 0xFF;
  frame[crc_len + 1] = crc & 0xFF;

  ESP_LOGV(TAG, "TX: demand heartbeat (demand=%u/15)", estia_demand_value_);
  log_raw_data("Estia TX", frame, sizeof(frame));

  std::vector<uint8_t> raw(frame, frame + sizeof(frame));
  this->raw_write_queue_.push(std::move(raw));
}

void ToshibaAbClimate::send_estia_data_request(uint8_t subtype) {
  uint16_t src = this->estia_source_address_;

  // Data request frame: A0:00:15:0A:00:SRC:DST:00:E8:Cx:01:00:CRC
  // Mirrors what KNX gateway sends as 0x15 request
  uint8_t frame[] = {
    0xA0, 0x00,                         // prefix
    0x15,                               // type: data request
    0x0A,                               // length: 10
    0x00,                               // fixed
    (uint8_t)(src >> 8), (uint8_t)(src & 0xFF),  // source
    0x08, 0x00,                         // dest: master
    0x00, 0xE8,                         // data type
    subtype,                            // C0=temperatures, C1=counters
    0x01, 0x00,                         // request params
    0x00, 0x00                          // CRC placeholder
  };

  size_t crc_len = sizeof(frame) - 2;
  uint16_t crc = estia_crc16(frame, crc_len);
  frame[crc_len]     = (crc >> 8) & 0xFF;
  frame[crc_len + 1] = crc & 0xFF;

  ESP_LOGV(TAG, "TX: data request E8:%02X", subtype);
  log_raw_data("Estia TX", frame, sizeof(frame));

  std::vector<uint8_t> raw(frame, frame + sizeof(frame));
  this->raw_write_queue_.push(std::move(raw));
}

bool ToshibaAbClimate::control_vent(bool state) {
  if (!tcc_state.power) {
    ESP_LOGW(TAG, "Can't control vent when powered off");
    return false;
  }
  ESP_LOGD(TAG, "Control vent: %d", state);
  TccState new_state = TccState{tcc_state};
  new_state.vent = state;
  return send_new_state(&new_state) > 0;
}

static uint8_t estia_first_gen_raw_checksum(const uint8_t *raw, size_t raw_len) {
  uint16_t sum = 0;
  if (raw_len < 2) return 0;
  for (size_t i = 0; i < raw_len - 1; i++) sum += raw[i];
  return static_cast<uint8_t>(sum & 0xFF);
}

static DataFrame make_estia_first_gen_frame(uint8_t src, uint8_t dst, const uint8_t *payload, size_t payload_len) {
  DataFrame frame{};
  const size_t inner_size = payload_len + 4;
  frame.raw[0] = static_cast<uint8_t>(payload_len + 7);
  frame.raw[1] = src;
  frame.raw[2] = dst;
  for (size_t i = 0; i < payload_len; i++) frame.raw[3 + i] = payload[i];
  frame.raw[inner_size - 1] = estia_first_gen_raw_checksum(frame.raw, inner_size);
  // Do not assign frame.data_length here: it aliases raw[3], which is the
  // first-generation Estia payload marker (typically 0xE0). DataFrame::size()
  // derives TU2C/first-generation Estia length from raw[0].
  frame.set_tu2c(true);
  return frame;
}

void ToshibaAbClimate::send_estia_first_gen_zone1(bool on) {
  if (this->read_only_) return;
  const uint8_t payload[] = {0xE0, 0x01, 0x21, static_cast<uint8_t>(on ? 0x03 : 0x0A)};
  this->send_command(make_estia_first_gen_frame(this->remote_address_, this->master_address_, payload, sizeof(payload)));
}

void ToshibaAbClimate::send_estia_first_gen_dhw_on() {
  if (this->read_only_) return;
  const uint8_t payload[] = {0xE0, 0x01, 0x21, 0x0C};
  this->send_command(make_estia_first_gen_frame(this->remote_address_, this->master_address_, payload, sizeof(payload)));
}

void ToshibaAbClimate::send_estia_first_gen_dhw_off() {
  if (this->read_only_) return;
  const uint8_t payload[] = {0xE0, 0x01, 0x21, 0x08};
  this->send_command(make_estia_first_gen_frame(this->remote_address_, this->master_address_, payload, sizeof(payload)));
}

void ToshibaAbClimate::send_estia_first_gen_dhw_boost(bool on) {
  // TODO: Replace this fallback once the first-generation Estia DHW boost command is known.
  if (on) {
    this->send_estia_first_gen_dhw_on();
  } else {
    this->send_estia_first_gen_dhw_off();
  }
}

void ToshibaAbClimate::send_estia_first_gen_auto_mode(bool on) {
  if (this->read_only_) return;
  const uint8_t payload[] = {0xE0, 0x01, 0x24, 0x01, static_cast<uint8_t>(on ? 0x01 : 0x00)};
  this->send_command(make_estia_first_gen_frame(this->remote_address_, this->master_address_, payload, sizeof(payload)));
}

void ToshibaAbClimate::send_estia_first_gen_dhw_setpoint(float target_temp) {
  if (this->read_only_) return;
  uint8_t encoded = static_cast<uint8_t>(std::round((target_temp + 16.0f) * 2.0f));
  const uint8_t payload[] = {0xE0, 0x01, 0x23, 0x08, 0x00, 0x00, 0xA0, encoded};
  this->send_command(make_estia_first_gen_frame(this->remote_address_, this->master_address_, payload, sizeof(payload)));
}

void ToshibaAbClimate::send_estia_first_gen_request_data(uint8_t request_code) {
  if (this->read_only_) return;
  const uint8_t payload[] = {0xE0, 0x41, 0x5C, 0x70, request_code};
  this->send_command(make_estia_first_gen_frame(this->remote_address_, this->master_address_, payload, sizeof(payload)));
}

void ToshibaAbClimate::process_received_data_estia_first_gen_(const DataFrame *frame) {
  if (frame == nullptr) return;
  const uint8_t len = frame->raw[0];
  const uint8_t raw_len = len > 3 ? len - 3 : 0;
  if (len < 7 || raw_len > DATA_FRAME_MAX_SIZE) return;

  const uint8_t src = frame->raw[1];
  const bool has_master_status_signature = frame->raw[3] == 0xE0 && frame->raw[5] == 0x31;
  const bool has_master_keepalive_signature = len == 0x0A && frame->raw[5] == 0x3A;
  const bool is_master_source = src == this->master_address_ ||
                                (this->master_address_auto_ &&
                                 (has_master_status_signature || has_master_keepalive_signature));
  const bool is_remote_source = src == this->remote_address_ || (src >= 0x60 && src <= TOSHIBA_ESTIA_REMOTE_MAX);
  auto unknown_label = [&]() -> std::string {
    char buf[40];
    if (is_master_source) {
      return "unknown data from master";
    }
    if (is_remote_source) {
      std::snprintf(buf, sizeof(buf), "unknown data from remote %02X", src);
      return buf;
    }
    std::snprintf(buf, sizeof(buf), "unknown data from source %02X", src);
    return buf;
  };
  auto frame_label = [&]() -> std::string {
    if (is_master_source && has_master_keepalive_signature) {
      return "Master keepalive";
    }
    if (is_master_source && has_master_status_signature) {
      return "MASTER PARAMETERS";
    }
    if (is_master_source && frame->raw[4] == 0x80 && frame->raw[5] == 0x5C) {
      return "Master reporting data/sensor value";
    }
    if (is_master_source && frame->raw[4] == 0x80 && frame->raw[5] == 0xA2) {
      return "Master reporting data/sensor not available";
    }
    if (is_master_source) {
      return unknown_label();
    }
    if (is_remote_source && len == 0x0C && frame->raw[4] == 0x41 && frame->raw[5] == 0x5C) {
      char buf[40];
      std::snprintf(buf, sizeof(buf), "remote data/sensor 0x%02X query", frame->raw[6]);
      return buf;
    }
    if (is_remote_source && frame->raw[3] == 0xE0 && frame->raw[4] == 0x41 && frame->raw[5] == 0x5C) {
      return "Remote Timer Read";
    }
    if (is_remote_source && frame->raw[3] == 0xE0 && frame->raw[4] == 0x01 && frame->raw[5] == 0x21) {
      return "Remote command";
    }
    if (is_remote_source && frame->raw[3] == 0xE0 && frame->raw[4] == 0x01 && frame->raw[5] == 0x23) {
      return "Remote command: setpoint change";
    }
    if (is_remote_source && frame->raw[3] == 0xE0 && frame->raw[5] == 0x31) {
      return "Remote status";
    }
    return unknown_label();
  };
  const std::string label = frame_label();
  log_tu2c_data_frame(label, frame);

  if (frame->raw[raw_len - 1] != estia_first_gen_raw_checksum(frame->raw, raw_len)) {
    ESP_LOGD(TAG, "Estia first-gen checksum fail for %s", label.c_str());
    return;
  }
  if (this->master_address_auto_ && frame->raw[3] == 0xE0 && frame->raw[5] == 0x31) {
    this->master_address_ = src;
    this->master_address_confirmed_ = true;
  }
  if (this->remote_address_auto_ && src == this->remote_address_ && this->remote_address_ < TOSHIBA_ESTIA_REMOTE_MAX) {
    this->remote_address_++;
    ESP_LOGI(TAG, "Estia remote-address collision; switching to 0x%02X", this->remote_address_);
  }
  if (src == this->master_address_) {
    this->last_master_alive_millis_ = millis();
    if (this->connected_binary_sensor_) this->connected_binary_sensor_->publish_state(true);
  }
  if (src == this->master_address_ && frame->raw[3] == 0xE0 && frame->raw[5] == 0x31 && len > 11) {
    this->estia_first_gen_zone1_active_ = frame->raw[6] & 0x01;
    this->estia_first_gen_dhw_active_ = frame->raw[6] & 0x02;
    this->estia_first_gen_dhw_boost_ = this->estia_first_gen_dhw_active_;
    this->estia_first_gen_auto_mode_active_ = frame->raw[7] & 0x04;
    this->estia_first_gen_hotwater_resistor_heating_ = frame->raw[8] & 0x04;
    this->estia_first_gen_hotwater_pump_heating_ = frame->raw[8] & 0x08;
    this->estia_first_gen_dhw_encoded_ = frame->raw[9];
    this->estia_first_gen_zone1_encoded_ = frame->raw[10];
    this->mode = (this->estia_first_gen_auto_mode_active_ || this->estia_first_gen_dhw_active_)
                     ? climate::CLIMATE_MODE_HEAT
                     : climate::CLIMATE_MODE_OFF;
    this->target_temperature = static_cast<float>(frame->raw[9]) / 2.0f - 16.0f;
    this->publish_state();
    if (this->zone1_switch_) this->zone1_switch_->publish_state(this->estia_first_gen_zone1_active_);
    if (this->dhw_boost_switch_) this->dhw_boost_switch_->publish_state(this->estia_first_gen_dhw_boost_);
    if (this->hotwater_pump_heating_binary_sensor_)
      this->hotwater_pump_heating_binary_sensor_->publish_state(this->estia_first_gen_hotwater_pump_heating_);
    if (this->hotwater_resistor_heating_binary_sensor_)
      this->hotwater_resistor_heating_binary_sensor_->publish_state(this->estia_first_gen_hotwater_resistor_heating_);
    if (this->zone1_target_temperature_sensor_) this->zone1_target_temperature_sensor_->publish_state(static_cast<float>(frame->raw[10]) / 2.0f - 16.0f);
    if (len > 18 && this->zone1_water_temp_sensor_) this->zone1_water_temp_sensor_->publish_state(static_cast<float>(frame->raw[14]) / 2.0f - 16.0f);
    if (len > 14) {
      ESP_LOGD(TAG, "Estia first-gen status temperatures: temperature 1=%.1f°C (raw 12=0x%02X), temperature 2=%.1f°C (raw 13=0x%02X), temperature 3=%.1f°C (raw 14=0x%02X)",
               static_cast<float>(frame->raw[12]) / 2.0f - 16.0f, frame->raw[12],
               static_cast<float>(frame->raw[13]) / 2.0f - 16.0f, frame->raw[13],
               static_cast<float>(frame->raw[14]) / 2.0f - 16.0f, frame->raw[14]);
    }
    this->estia_first_gen_pump_state_known_ = true;
  } else if (src == this->master_address_ && frame->raw[4] == 0x80 && frame->raw[5] == 0x5C && len > 8) {
    const uint16_t value = encode_uint16(frame->raw[6], frame->raw[7]);
    bool published_polled_sensor = false;
    if (this->last_sensor_query_id_ != 0xFF) {
      for (auto &ps : this->polled_sensors_) {
        if (ps.id == this->last_sensor_query_id_) {
          if (this->last_sensor_query_id_ == ESTIA_FIRST_GEN_DHW_TEMP_REQUEST) {
            this->publish_dhw_current_temperature_(static_cast<float>(value) * ps.scale);
          } else if (ps.sensor != nullptr) {
            ps.sensor->publish_state(static_cast<float>(value) * ps.scale);
          }
          published_polled_sensor = true;
          ESP_LOGD(TAG, "Estia first-gen sensor: id=0x%02X raw=0x%04X", this->last_sensor_query_id_, value);
          break;
        }
      }
      this->sensor_query_outstanding_ = false;
      this->last_sensor_query_id_ = 0xFF;
    }
    if (!published_polled_sensor && this->outdoor_temp_sensor_) this->outdoor_temp_sensor_->publish_state(value);
  } else if (src == this->master_address_ && frame->raw[4] == 0x80 && frame->raw[5] == 0xA2) {
    if (this->sensor_query_outstanding_ || this->last_sensor_query_id_ != 0xFF) {
      ESP_LOGD(TAG, "Estia first-gen sensor not available: id=0x%02X", this->last_sensor_query_id_);
    }
    this->sensor_query_outstanding_ = false;
    this->last_sensor_query_id_ = 0xFF;
  } else {
    log_raw_data("Estia first-gen", frame->raw, len);
  }
}

void ToshibaAbEstiaZone1Switch::write_state(bool state) {
  this->climate_->send_estia_first_gen_zone1(state);
}

void ToshibaAbEstiaDhwBoostSwitch::write_state(bool state) {
  this->climate_->send_estia_first_gen_dhw_boost(state);
}

void ToshibaAbVentSwitch::write_state(bool state) {
  if (this->climate_->control_vent(state)) {
    // don't publish state. wait for the unit to report it's state
  }
}

void ToshibaAbReadOnlySwitch::write_state(bool state) {
  // Toggle read-only mode on the climate component
  this->climate_->set_read_only(state);
  // Publish the new state so Home Assistant UI reflects the change immediately
  this->publish_state(state);
}

void ToshibaAbClimate::set_master_address(uint8_t address) {
  this->master_address_ = address;
  this->master_address_confirmed_ = false;
}

}  // namespace toshiba_ab
}  // namespace esphome
