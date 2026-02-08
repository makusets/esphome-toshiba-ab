#include "toshiba_ab.h"
#include "esphome.h"
#include <inttypes.h>
#include <cstring>


namespace esphome {
namespace toshiba_ab {



static const char *const TAG = "tcc_link.climate";


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

bool ToshibaAbClimate::is_own_tx_echo_(const DataFrame *f) const { // used to filter out echo from our last command sent
  if (!this->last_unconfirmed_command_.has_value() || f == nullptr) return false;
  const auto &tx = this->last_unconfirmed_command_.value();  // last frame we wrote
  if (f->size() != tx.size()) return false;
  return std::memcmp(f->raw, tx.raw, f->size()) == 0;
}

void ToshibaAbClimate::update_frame_validation_() {
  const bool allow_same = this->autonomous_ && !this->announce_ack_received_;
  this->data_reader.set_allow_same_source_dest(allow_same);
}


bool ToshibaAbClimate::is_ack_for_pending_command_(const DataFrame *frame) const {
  if (frame == nullptr || !this->last_unconfirmed_command_.has_value()) {
    return false;
  }
  const auto &pending = this->last_unconfirmed_command_.value();

  // ACK/retry logic is only for classic TCCLink frames, not TU2C.
  if (frame->is_tu2c() || pending.is_tu2c()) {
    return false;
  }

  if (frame->source != pending.dest || frame->dest != pending.source) {
    return false;
  }

  if (frame->opcode1 != OPCODE_ACK || frame->data_length < 2) {
    return false;
  }

  return frame->data[0] == this->command_mode_write_ && frame->data[1] == OPCODE2_PARAM_ACK;
}

bool ToshibaAbClimate::should_track_command_ack_(const DataFrame &frame) const {
  if (frame.is_tu2c()) {
    return false;
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
    return;
  }

  this->resend_last_unconfirmed_command_ = true;
}

void log_data_frame(const std::string msg, const struct DataFrame *frame, size_t length = 0) {
  std::string res;
  char buf[5];
  size_t len = length > 0 ? length : frame->data_length;
  for (size_t i = 0; i < len; i++) {
    if (i > 0) {
      res += ':';
    }
    sprintf(buf, "%02X", frame->data[i]);
    res += buf;
  }
  ESP_LOGD("RX", "%s: %02X:%02X:\x1B[32m%02X\033[0m:%02X:\033[2;100;37m%s\033[0m:%02X", msg.c_str(), frame->source,
           frame->dest, frame->opcode1, frame->data_length, res.c_str(), frame->crc());
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
  ESP_LOGD("RX", "%s", "%s", prefix.c_str(), res.c_str());
}


void write_set_parameter(struct DataFrame *command, uint8_t master_address, uint8_t command_mode_read, uint8_t opcode2,
                         uint8_t payload[], size_t payload_size) {
  command->source = TOSHIBA_REMOTE;
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

void write_set_temperature(struct DataFrame *command, uint8_t master_address, uint8_t command_mode_read, uint8_t opcode2,
                           uint8_t payload[], size_t payload_size) {
  command->source = TOSHIBA_REMOTE;
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


void write_set_parameter(struct DataFrame *command, uint8_t master_address, uint8_t command_mode_read, uint8_t opcode2,
                         uint8_t single_type_payload) {
  uint8_t payload[1] = {single_type_payload};
  write_set_parameter(command, master_address, command_mode_read, opcode2, payload, 1);
}

void write_set_parameter_flags(struct DataFrame *command, uint8_t master_address, uint8_t command_mode_read,
                               const struct TccState *state, uint8_t set_flags) {
  uint8_t payload[6] = {
      static_cast<uint8_t>(state->mode | set_flags),
      static_cast<uint8_t>(state->fan | get_fan_bit_mask_for_mode(state->mode)),
      temp_celcius_to_payload(state->target_temp),
      EMPTY_DATA,
      get_heat_cool_bits(state->mode),
      get_heat_cool_bits(state->mode),
  };
  write_set_parameter(command, master_address, command_mode_read, OPCODE2_SET_TEMP_WITH_FAN, payload, sizeof(payload));
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

void write_set_parameter_mode(struct DataFrame *command, uint8_t master_address, uint8_t command_mode_read,
                              const struct TccState *state) {
  write_set_parameter(command, master_address, command_mode_read, OPCODE2_SET_MODE, state->mode);
}

void write_set_parameter_power(struct DataFrame *command, uint8_t master_address, uint8_t command_mode_read,
                               const struct TccState *state) {
  write_set_parameter(command, master_address, command_mode_read, OPCODE2_SET_POWER, state->power | 0b0010);
}

void write_set_parameter_vent(struct DataFrame *command, uint8_t master_address, uint8_t command_mode_read,
                              const struct TccState *state) {
  write_set_parameter(command, master_address, command_mode_read, OPCODE2_SET_VENT, state->vent);
}

void write_set_parameter_room_temp(struct DataFrame *command, uint8_t master_address, uint8_t command_mode_read,
                                   float temperature) {
  // Clamp temperature to a safe range (adjust if needed by protocol)
  float clamped = std::max(0.0f, std::min(temperature, 40.0f));

  // Round to nearest 0.5°C granularity
  float rounded = std::round(clamped * 2.0f) / 2.0f;

  uint8_t room_temp [2] = {0x00, temp_celcius_to_payload(rounded)};

  // Send using existing write_set_temperature
  write_set_temperature(command, master_address, command_mode_read, OPCODE2_SENSOR_ROOM_TEMP, room_temp, sizeof(room_temp));
}

void write_read_envelope(DataFrame *cmd, uint8_t master_address,
                         uint8_t command_mode_read, uint8_t opcode2, const uint8_t payload[], size_t payload_size) {
// Writes a read envelope command
// This is used to read some data from the master unit, but not sensors
// And to send remote PING/KEEP_ALIVE commands
  cmd->source      = TOSHIBA_REMOTE;
  cmd->dest        = master_address;
  cmd->opcode1     = OPCODE_ERROR_HISTORY;             // 0x15
  cmd->data_length = SET_PARAMETER_PAYLOAD_HEADER_SIZE + payload_size;
  cmd->data[0]     = command_mode_read;
  cmd->data[1]     = opcode2;                          // e.g., OPCODE2_PING_PONG (0x0C)

  for (size_t i = 0; i < payload_size; i++)
    cmd->data[SET_PARAMETER_PAYLOAD_HEADER_SIZE + i] = payload[i];

  cmd->data[SET_PARAMETER_PAYLOAD_HEADER_SIZE + payload_size] = cmd->calculate_crc();
}


// Sends room temp to AC unit with the sensor configured in yaml
// example frame: 42:00:11:04:08:89:72:46:E2 for 22 degrees
void ToshibaAbClimate::send_remote_temp(float temp_c) {
  // sanity
  if (!std::isfinite(temp_c) || temp_c < -40.0f || temp_c > 80.0f) {
    ESP_LOGW(TAG, "send_remote_temp: invalid temp %.2f°C", temp_c);
    return;
  }

  // Encode raw = (C + OFFSET) * RATIO, mask per protocol
  const uint8_t raw = static_cast<uint8_t>(
      std::lround((temp_c + TEMPERATURE_CONVERSION_OFFSET) * TEMPERATURE_CONVERSION_RATIO));


  DataFrame cmd{};
  cmd.source      = TOSHIBA_TEMP_SENSOR;        // 0x42
  cmd.dest        = this->master_address_;  // usually 0x00
  cmd.opcode1     = OPCODE_PARAMETER;       // 0x11
  cmd.data_length = 4;                       // payload: 08 89 <raw> 46

  cmd.data[0] = this->command_mode_read_;
  cmd.data[1] = 0x89;
  cmd.data[2] = raw;   // encoded temperature
  cmd.data[3] = 0x46;


  cmd.data[cmd.data_length] = cmd.calculate_crc();

  this->send_command(cmd);  // enqueue; loop() sends when bus is idle
}



void ToshibaAbClimate::send_ping() { // Sends a PING command to the master unit, command goes into queue, sent when loop() finds it
  // typical: 40:00:15:07:08:0C:81:00:00:48:00:9F
  // where tail = 0x81, 0x00, 0x00, 0x48, 0x00  (matches the reference ping)
  DataFrame cmd{};

  const uint8_t tail[] = { OPCODE2_READ_STATUS, 0x00, 0x00, 0x48, 0x00 };
  write_read_envelope(&cmd, this->master_address_, this->command_mode_read_, OPCODE2_PING_PONG, tail, sizeof(tail));
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

  write_read_envelope(&cmd, this->master_address_, this->command_mode_read_, opcode2, tail, sizeof(tail));
  this->send_command(cmd);  // enqueue; loop() will transmit when bus is idle
}

void ToshibaAbClimate::remote_announce() {
  // Build and enqueue a short announce/envelope frame from the remote.
  // Example format: 40:F0:15:02:00:0D:AA -> source=0x40, opcode=0x15, len=2, data={0x00,0x0D}
  DataFrame cmd{};
  cmd.source = TOSHIBA_REMOTE;
  cmd.dest = TOSHIBA_BROADCAST;
  cmd.opcode1 = OPCODE_ERROR_HISTORY; // 0x15
  cmd.data_length = 2;
  cmd.data[0] = 0x00;
  cmd.data[1] = 0x0D;
  cmd.data[cmd.data_length] = cmd.calculate_crc();
  ESP_LOGV(TAG, "remote_announce: enqueuing announce to broadcast (0x%02X)", TOSHIBA_BROADCAST);
  this->send_command(cmd);
}

void ToshibaAbClimate::add_polled_sensor(uint8_t id, float scale, uint32_t interval_ms, sensor::Sensor *sensor) {
  PolledSensor ps{ id, scale, interval_ms, sensor };
  this->polled_sensors_.push_back(ps);

  // Schedule this sensor’s periodic query
  // It doens't need to be added to loop it is added here for every sensor configured
  if (interval_ms > 0) {
    this->set_interval(interval_ms, [this, id]() {
      // Light back-pressure: avoid growing queue if it’s already busy
      if (this->write_queue_.size() < 3) {
        this->send_sensor_query(id);  // enqueue; loop() will handle bus timing
      }
    });
  }
  
    // Ensure the read-only switch reports its initial state to Home Assistant
    if (this->read_only_switch_)
      this->read_only_switch_->publish_state(this->read_only_);
}

void ToshibaAbClimate::send_sensor_query(uint8_t sensor_id) {
  DataFrame cmd{};
  cmd.source      = TOSHIBA_REMOTE;           // 0x40
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
        if (ps.id == prev_id && ps.sensor != nullptr) {
          const float value = static_cast<float>(raw) * ps.scale;
          ps.sensor->publish_state(value);
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
      if (ps.id == id && ps.sensor != nullptr) {
        const float value = static_cast<float>(raw) * ps.scale;
        ps.sensor->publish_state(value);
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
    return climate::CLIMATE_FAN_OFF;

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
      climate::CLIMATE_FAN_OFF,
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
  ESP_LOGCONFIG(TAG, "  Master address: 0x%02X", this->master_address_);
  ESP_LOGCONFIG(TAG, "  Master address auto: %s", this->master_address_auto_ ? "true" : "false");
  ESP_LOGCONFIG(TAG, "  Command mode read: 0x%02X", this->command_mode_read_);
  ESP_LOGCONFIG(TAG, "  Command mode write: 0x%02X", this->command_mode_write_);
  ESP_LOGCONFIG(TAG, "  Frame format: %s",
                this->data_reader.frame_format() == FrameFormat::TU2C ? "TU2C (U series)" : "TCC-Link");
  ESP_LOGCONFIG(TAG, "  Filter frames: %s", this->filter_frames_ ? "true" : "false");
  ESP_LOGCONFIG(TAG, "  Autonomous mode: %s", this->autonomous_ ? "true" : "false");
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
  


  pinMode(16, OUTPUT); // Set GPIO16 low, only needed for my old board, to be removed soon
  digitalWrite(16, LOW);

  // If autonomous mode is enabled, we need to send ping, E8 read, and external temp periodically
  // Send all these if autonomous mode is enabled using set_interval calls
  if (this->autonomous_) {
  this->set_interval(this->ping_interval_ms_, [this]() { //30s
    // Ping (keep-alive) from remote
    this->send_ping();  // just enqueues; loop() will transmit
    ESP_LOGV(TAG, "Autonomous: enqueued PING (keep-alive)");

    // Send remote temperature along with ping in the remote format (source=0x40,
    // opcode=OPCODE_TEMPERATURE, payload: 08 81 00 <raw> 00, CRC). Prefer
    // configured external sensor, otherwise use current target temperature,
    // otherwise default to 20°C.
    // This should still work if the AC is configured to use return duct temp sensor for room temp,
    // as the value here will be disregarded by the master unit in that case.

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
      cmd.source = TOSHIBA_REMOTE;                 // 0x40
      cmd.dest = this->master_address_;
      cmd.opcode1 = OPCODE_TEMPERATURE;            // 0x55
      cmd.data_length = 5;                         // 08 81 00 <raw> 00
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
  this->set_interval(this->read08_interval_ms, [this]() { // send 40:00:15:06:08:E8:00:01:00:9E:2C every min as remote does

    ESP_LOGV(TAG, "Autonomous: enqueuing read hourly counter block E8");
    this->send_read_block(0xE8, 0x0001, 0x009E);  // enqueues; loop() will transmit

    });

  // Announce loop: broadcast remote_announce() every 2s until an ACK with 0x0D is seen
  this->announce_ack_received_ = false;
  this->set_interval(2000, [this]() {
    if (!this->announce_ack_received_) {
      ESP_LOGV(TAG, "Autonomous announce: sending broadcast announce");
      this->remote_announce();
    } 
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

void ToshibaAbClimate::process_received_data(const struct DataFrame *frame) {
  if (frame->source == this->master_address_) {
      // status update
      ESP_LOGD(TAG, "Received data from master:");
      last_master_alive_millis_ = millis();
      if (this->connected_binary_sensor_) {
        this->connected_binary_sensor_->publish_state(true);
      }

      switch (frame->opcode1) {
        case OPCODE_PING: {
        log_data_frame("PING/ALIVE", frame);
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
        case OPCODE_STATUS:
          // sync power, mode, fan and target temp from the unit to the climate
          // component

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
        case OPCODE_EXTENDED_STATUS:
          // sync power, mode, fan and target temp from the unit to the climate
          // component

          log_data_frame("EXTENDED STATUS", frame);

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

          
          if (frame->data[STATUS_DATA_TARGET_TEMP_BYTE + 1] > 1) {
            tcc_state.room_temp =
                static_cast<float>(frame->data[STATUS_DATA_TARGET_TEMP_BYTE + 1]) / TEMPERATURE_CONVERSION_RATIO -
                TEMPERATURE_CONVERSION_OFFSET;
          }

          tcc_state.preheating  = (frame->data[STATUS_DATA_FLAGS_BYTE] & 0b00000010) >> 1;
          tcc_state.filter_alert = (frame->data[STATUS_DATA_FLAGS_BYTE] & 0b10000000) >> 7;
          ESP_LOGD(TAG, "Power: %d, Mode: %02X, Fan: %02X, Vent: %02X, Target Temp: %.1f, Room Temp: %.1f, Preheating: %d, Filter Alert: %d",
                   tcc_state.power, tcc_state.mode, tcc_state.fan, tcc_state.vent, tcc_state.target_temp,
                   tcc_state.room_temp, tcc_state.preheating, tcc_state.filter_alert);

          sync_from_received_state();

          break;
        case OPCODE_SENSOR_VALUE:
            // sensor value received from master
          this->process_sensor_value_(frame);
          break;

        default:
          log_data_frame("MASTER", frame);
          break;
      }
    }else {
    if (frame->source == TOSHIBA_REMOTE) {
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
        }
      
      // Remote 40:00:15:06:08:E8:00:01:00:9E:2C that is sent every minute, the master responds with an hourly counter (time on?)
      } else if (frame->opcode1 == OPCODE_ERROR_HISTORY &&      // 0x15 envelope
                frame->data_length >= 6 &&
                frame->data[0] == this->command_mode_read_ &&
                frame->data[1] == 0xE8 &&
                frame->data[3] == 0x01 &&                 
                frame->data[5] == 0x9E) {                       
                
        log_data_frame("Remote Timer Read", frame);

      } else {
        // unknown remote message
        log_data_frame("Unknown remote data", frame);
      }
    
    } else if (frame->source == TOSHIBA_TEMP_SENSOR) {
      // message from configured temp sensor in yaml
      // example:   42:00:11:04:08:89:72:46:E2  for 22 degrees, this message follows Toshiba standalone temp sensor format
      if (frame->opcode1 == OPCODE_PARAMETER &&
          frame->data_length == 4 &&
          frame->data[1] == 0x89) {

        std::string label = this->ext_temp_sensor_name_.empty()
                    ? "Yaml temp sensor"
                    : this->ext_temp_sensor_name_;
        log_data_frame(label, frame);
        uint8_t raw = frame->data[2] & TEMPERATURE_DATA_MASK;  // raw[7]
        float sensor_temp = static_cast<float>(raw) / TEMPERATURE_CONVERSION_RATIO - TEMPERATURE_CONVERSION_OFFSET;
        ESP_LOGD(TAG, "%s: %.1f °C", label.c_str(), sensor_temp);
      } else {
        log_data_frame("Unknown 0x42 data", frame);
      }
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
    log_raw_data("TU2C frame too short: ", frame->raw, size);
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
    log_raw_data("Master keepalive: ", frame->raw, size);
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

  if (frame_length == 0x0C && source == this->tu2c_master_address_ && payload_available >= 4 &&
      frame->raw[payload_offset] == 0x80) {
    log_raw_data("Master ACK frame: ", frame->raw, size);
    ESP_LOGD(TAG, "ACK from master %02X with counter: %02X:%02X", source, frame->raw[payload_offset + 2],
             frame->raw[payload_offset + 3]);
    return;
  }

  if (frame_length == 0x0C && payload_available >= 2 && frame->raw[payload_offset] == 0x41 &&
      frame->raw[payload_offset + 1] == 0x5C) {
    log_raw_data("Remote keepalive frame: ", frame->raw, size);
    ESP_LOGD(TAG, "Remote %02X keepalive", source);
    return;
  }

  if (frame_length == 0x0D && payload_available >= 4 && frame->raw[payload_offset] == 0x61 &&
      frame->raw[payload_offset + 1] == 0x38 && size > 7) {
    log_raw_data("Remote room temp frame: ", frame->raw, size);
    const uint8_t raw_temp = frame->raw[7];
    const float room_temp =
        static_cast<float>(raw_temp) / TEMPERATURE_CONVERSION_RATIO - TEMPERATURE_CONVERSION_OFFSET;
    tcc_state.room_temp = room_temp;
    ESP_LOGD(TAG, "Remote %02X room temp: %.1f", source, room_temp);
    return;
  }

  if (dest == 0xFF && payload_available >= STATUS_DATA_TARGET_TEMP_BYTE + 1) {
    if (frame->raw[payload_offset] != 0xC0 || frame->raw[payload_offset + 1] != 0x38) {
      log_raw_data("TU2C data: ", frame->raw, size);
      return;
    }
    const uint8_t *payload = &frame->raw[payload_offset];
    log_raw_data("TU2C status: ", frame->raw, size);
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
             "TU2C status: power=%d mode=%02X fan=%02X vent=%02X target=%.1f room=%.1f preheat=%d filter=%d",
             tcc_state.power, tcc_state.mode, tcc_state.fan, tcc_state.vent, tcc_state.target_temp,
             tcc_state.room_temp, tcc_state.preheating, tcc_state.filter_alert);
    sync_from_received_state();
    return;
  }

  log_raw_data("TU2C data: ", frame->raw, size);
}

bool ToshibaAbClimate::receive_data(const std::vector<uint8_t> data) {
  auto frame = DataFrame();

  for (size_t i = 0; i < data.size(); i++) {
    frame.raw[i] = data[i];
  }

  return receive_data_frame(&frame);
}

bool ToshibaAbClimate::receive_data_frame(const struct DataFrame *frame) {
  if (!frame->is_tu2c() && frame->crc() != frame->calculate_crc()) {
    ESP_LOGW(TAG, "CRC check failed");
    log_data_frame("Failed frame", frame);

    if (this->failed_crcs_sensor_ != nullptr) {
      this->failed_crcs_sensor_->publish_state(this->failed_crcs_sensor_->state + 1);
    }

    return false;
  }
  // >>> Drop our own TX echo frames (remote-labeled, identical bytes)
  if (frame->source == TOSHIBA_REMOTE && this->is_own_tx_echo_(frame)) {
    ESP_LOGV(TAG, "Ignoring TX echo of our own frame");
    return true;  // swallow quietly
  }

  if (this->last_unconfirmed_command_.has_value() && !frame->is_tu2c() &&
      !this->last_unconfirmed_command_->is_tu2c() && !this->is_ack_for_pending_command_(frame)) {
    this->resend_last_unconfirmed_command_ = true;
  }

  // still notify any listeners of real frames
  this->set_data_received_callback_.call(frame);
  if (frame->is_tu2c()) {
    process_received_data_tu2c_(frame);
  } else {
    process_received_data(frame);
  }
  return true;
}


void ToshibaAbClimate::loop() {
  // TODO: check if last_unconfirmed_command_ was not confirmed after a timeout
  // and log warning/error

  const bool bus_can_send = (millis() - last_received_frame_millis_) >= FRAME_SEND_MILLIS_FROM_LAST_RECEIVE &&
                            (millis() - last_sent_frame_millis_) >= FRAME_SEND_MILLIS_FROM_LAST_SEND;

  if (bus_can_send) {
    optional<DataFrame> frame_to_send{};

    if (this->resend_last_unconfirmed_command_ && this->last_unconfirmed_command_.has_value()) {
      if (this->last_unconfirmed_command_attempts_ >= MAX_COMMAND_SEND_ATTEMPTS) {
        ESP_LOGE(TAG, "Command opcode 0x%02X not acknowledged after %u attempts", this->last_unconfirmed_command_->opcode1,
                 static_cast<unsigned>(this->last_unconfirmed_command_attempts_));
        this->last_unconfirmed_command_.reset();
        this->last_unconfirmed_command_attempts_ = 0;
        this->resend_last_unconfirmed_command_ = false;
      } else {
        frame_to_send = this->last_unconfirmed_command_.value();
        this->last_unconfirmed_command_attempts_++;
        this->resend_last_unconfirmed_command_ = false;
        ESP_LOGW(TAG, "Resending command opcode 0x%02X (attempt %u/%u)", frame_to_send->opcode1,
                 static_cast<unsigned>(this->last_unconfirmed_command_attempts_),
                 static_cast<unsigned>(MAX_COMMAND_SEND_ATTEMPTS));
      }
    }

    if (!frame_to_send.has_value() && !this->last_unconfirmed_command_.has_value() && !this->write_queue_.empty()) {
      frame_to_send = this->write_queue_.front();
      this->write_queue_.pop();
      if (this->should_track_command_ack_(frame_to_send.value())) {
        this->last_unconfirmed_command_ = frame_to_send.value();
        this->last_unconfirmed_command_attempts_ = 1;
        this->resend_last_unconfirmed_command_ = false;
      } else {
        this->last_unconfirmed_command_.reset();
        this->last_unconfirmed_command_attempts_ = 0;
        this->resend_last_unconfirmed_command_ = false;
      }
    }

    if (frame_to_send.has_value()) {
      last_sent_frame_millis_ = millis();
      auto frame = frame_to_send.value();
      log_data_frame("Write frame", &frame);
      if (frame.is_tu2c()) {
        const size_t raw_size = frame.size();
        uint8_t tu2c_frame[DATA_FRAME_MAX_SIZE + 3];
        tu2c_frame[0] = 0xF0;
        tu2c_frame[1] = 0xF0;
        if (raw_size + 3 <= sizeof(tu2c_frame)) {
          std::memcpy(&tu2c_frame[2], frame.raw, raw_size);
          tu2c_frame[2 + raw_size] = 0xA0;
          this->write_array(tu2c_frame, raw_size + 3);
        } else {
          ESP_LOGW(TAG, "TU2C frame too large to send (size=%u)", static_cast<unsigned>(raw_size));
        }
      } else {
        this->write_array(frame.raw, frame.size());
      }

      if (this->write_queue_.empty()) {
        ESP_LOGD(TAG, "All frames written");
      }
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

  while (available()) {
    int byte = read();
    if (byte >= 0) {
      bytes_read++;

      if (!can_read_packet)
        continue;  // wait until can read packet

      if (data_reader.put(byte)) {
        // packet complete

        last_received_frame_millis_ = millis();

        auto frame = data_reader.frame;

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
      if (millis_since_last_read >= PACKET_MIN_WAIT_MILLIS) {
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
    // not connected
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
      write_set_parameter_power(&command, this->master_address_, this->command_mode_read_, new_state);
      commands.push_back(command);
    } else {
      // turn off
      ESP_LOGD(TAG, "Turning off");
      auto command = DataFrame{};
      if (use_tu2c) {
        write_power_off_tu2c(&command, this->tu2c_remote_address_, this->tu2c_master_address_);
      } else {
        write_set_parameter_power(&command, this->master_address_, this->command_mode_read_, new_state);
      }
      commands.push_back(command);
      // don't process other changes when turning off
      return commands;
    }
  }

  if (new_state->mode != tcc_state.mode) {
    ESP_LOGD(TAG, "Changing mode");
    auto command = DataFrame{};
    write_set_parameter_mode(&command, this->master_address_, this->command_mode_read_, new_state);
    commands.push_back(command);
  }

  if (new_state->fan != tcc_state.fan) {
    ESP_LOGD(TAG, "Changing fan");
    auto command = DataFrame{};
    if (use_tu2c) {
      write_set_parameter_flags_tu2c(&command, this->tu2c_remote_address_, this->tu2c_master_address_, new_state,
                                        COMMAND_SET_FAN);
    } else {
      write_set_parameter_flags(&command, this->master_address_, this->command_mode_read_, new_state, COMMAND_SET_FAN);
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
      write_set_parameter_flags(&command, this->master_address_, this->command_mode_read_, new_state, COMMAND_SET_TEMP);
    }
    commands.push_back(command);
  }

  if (new_state->vent != tcc_state.vent) {
    ESP_LOGD(TAG, "Changing vent");
    auto command = DataFrame{};
    write_set_parameter_vent(&command, this->master_address_, this->command_mode_read_, new_state);
    commands.push_back(command);
  }

  return commands;
}

void ToshibaAbClimate::control(const climate::ClimateCall &call) {
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
  // While waiting for announce ACK, only allow the broadcast announce frame.
  // This restriction is only relevant when autonomous mode is enabled.
  if (this->autonomous_ && !this->announce_ack_received_) {
    bool is_announce = false;
    if (command.source == TOSHIBA_REMOTE && command.dest == TOSHIBA_BROADCAST &&
        command.opcode1 == OPCODE_ERROR_HISTORY && command.data_length == 2 &&
        command.data[1] == 0x0D) {
      is_announce = true;
    }

    if (!is_announce) {
      ESP_LOGW(TAG, "Dropping command while awaiting announce ACK (autonomous mode)");
      return;
    }
  }

  log_data_frame("Enqueue command", &command);
  this->write_queue_.push(command);
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
}

}  // namespace toshiba_ab
}  // namespace esphome
