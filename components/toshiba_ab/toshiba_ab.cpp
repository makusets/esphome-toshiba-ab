#include "toshiba_ab.h"
#include "esphome/core/log.h"
#include <cstdio>

#ifdef USE_ESP8266
#include <HardwareSerial.h>
#endif

namespace esphome {
namespace toshiba_ab {

static const char *const TAG = "toshiba_ab";

#ifdef USE_ESP8266
static HardwareSerial bus_serial(UART0);
#endif

void ToshibaAbClimate::setup() {
  boot_ms_ = millis();
  discovery_stage_ms_ = boot_ms_;
  master_address_ = master_setting_;
  this->mode = climate::CLIMATE_MODE_OFF;
  this->target_temperature = 22.0f;
  set_runtime_parity_(uart::UART_CONFIG_PARITY_EVEN);
  diagnostic_("Scanning TCC for a master keepalive (EVEN parity)");

#ifdef USE_ESP8266
  if (hardware_uart_rx_pin_ == 13)
    ESP_LOGCONFIG(TAG, "UART0 RX swapped from GPIO3 to GPIO13");
#endif
}

void ToshibaAbClimate::loop() {
  const uint32_t now = millis();
  check_reader_timeout_(now);
  if (discovery_state_ != DiscoveryState::COMPLETE && now - discovery_stage_ms_ >= SCAN_PERIOD_MS)
    advance_discovery_();

  uint8_t byte;
  size_t bytes_read = 0;
#ifdef USE_ESP8266
  if (hardware_uart_rx_pin_ == 13) {
    while (bytes_read < MAX_BYTES_PER_LOOP && bus_serial.available() && bus_serial.readBytes(&byte, 1) == 1) {
      read_byte_(byte);
      bytes_read++;
    }
    return;
  }
#endif
  while (bytes_read < MAX_BYTES_PER_LOOP && available() && read_byte(&byte)) {
    read_byte_(byte);
    bytes_read++;
  }
}

void ToshibaAbClimate::read_byte_(uint8_t byte) {
  ESP_LOGVV(TAG, "Reader accepted byte 0x%02X", byte);
  last_reader_byte_ms_ = millis();
  switch (discovery_state_) {
    case DiscoveryState::SCAN_TCC:
      // Once discovery is complete, protocol_detected_ selects the reader.
      read_even_byte_(byte);
      break;
    case DiscoveryState::SCAN_A0:
      read_even_byte_(byte);
      break;
    case DiscoveryState::SCAN_TU2C:
      read_tu2c_byte_(byte);
      break;
    case DiscoveryState::COMPLETE:
      switch (protocol_detected_) {
        case Protocol::TCC:
          read_even_byte_(byte);
          break;
        case Protocol::A0:
          read_even_byte_(byte);
          break;
        case Protocol::TU2C:
          read_tu2c_byte_(byte);
          break;
        default:
          break;
      }
      break;
  }
}

void ToshibaAbClimate::read_even_byte_(uint8_t byte) {
  const bool read_a0 = discovery_state_ == DiscoveryState::SCAN_A0 ||
                       (discovery_state_ == DiscoveryState::COMPLETE && protocol_detected_ == Protocol::A0);
  if (read_a0 && a0_size_ == 0) {
    if (a0_sync_ == 0 && byte == 0xA0)
      a0_sync_ = 1;
    else if (a0_sync_ == 1 && byte == 0x00) {
      a0_[0] = 0xA0;
      a0_[1] = 0x00;
      a0_size_ = 2;
      a0_sync_ = 0;
    } else
      a0_sync_ = byte == 0xA0 ? 1 : 0;
  } else if (read_a0) {
    if (a0_size_ < a0_.size())
      a0_[a0_size_++] = byte;
    if (a0_size_ == 4) {
      a0_expected_ = static_cast<size_t>(a0_[3]) + 6;  // wrapper, type, length, body, CRC16
      if (a0_expected_ < 8 || a0_expected_ > a0_.size())
        a0_size_ = a0_expected_ = 0;
    }
    if (a0_expected_ && a0_size_ == a0_expected_) {
      const uint16_t received = (static_cast<uint16_t>(a0_[a0_size_ - 2]) << 8) | a0_[a0_size_ - 1];
      process_frame_(Protocol::A0, a0_.data(), a0_size_, received == crc16_mcrf4xx_(a0_.data(), a0_size_ - 2));
      a0_size_ = a0_expected_ = 0;
    }
  }

  const bool read_tcc = discovery_state_ == DiscoveryState::SCAN_TCC ||
                        (discovery_state_ == DiscoveryState::COMPLETE && protocol_detected_ == Protocol::TCC);
  if (!read_tcc)
    return;

  // TCC has no wrapper. Keep a sliding buffer so noise, a truncated candidate,
  // or a failed CRC cannot hide the beginning of the next valid frame.
  if (tcc_size_ == tcc_.size()) {
    for (size_t i = 1; i < tcc_size_; i++)
      tcc_[i - 1] = tcc_[i];
    tcc_size_--;
    ESP_LOGV(TAG, "TCC reader buffer full; discarded oldest byte");
  }
  tcc_[tcc_size_++] = byte;
  while (tcc_size_ >= 4) {
    const uint8_t length = tcc_[3];
    const bool plausible_header = tcc_[0] <= 0xA0 && tcc_[0] != tcc_[1] && length >= 2 && length <= 124;
    if (!plausible_header) {
      for (size_t i = 1; i < tcc_size_; i++)
        tcc_[i - 1] = tcc_[i];
      tcc_size_--;
      ESP_LOGV(TAG, "TCC reader discarded byte while seeking a plausible header");
      continue;
    }
    tcc_expected_ = static_cast<size_t>(length) + 5;
    if (tcc_size_ < tcc_expected_)
      break;

    uint8_t crc = 0;
    for (size_t i = 0; i + 1 < tcc_expected_; i++)
      crc ^= tcc_[i];
    const bool valid = crc == tcc_[tcc_expected_ - 1];
    process_frame_(Protocol::TCC, tcc_.data(), tcc_expected_, valid);
    const size_t consumed = valid ? tcc_expected_ : 1;
    for (size_t i = consumed; i < tcc_size_; i++)
      tcc_[i - consumed] = tcc_[i];
    tcc_size_ -= consumed;
    tcc_expected_ = 0;
  }
}

void ToshibaAbClimate::read_tu2c_byte_(uint8_t byte) {
  if (tu2c_size_ == 0) {
    if (tu2c_sync_ == 0 && byte == 0xF0)
      tu2c_sync_ = 1;
    else if (tu2c_sync_ == 1 && byte == 0xF0) {
      tu2c_[0] = tu2c_[1] = 0xF0;
      tu2c_size_ = 2;
      tu2c_sync_ = 0;
    } else
      tu2c_sync_ = byte == 0xF0 ? 1 : 0;
    return;
  }
  if (tu2c_size_ < tu2c_.size())
    tu2c_[tu2c_size_++] = byte;
  if (tu2c_size_ == 3) {
    tu2c_expected_ = tu2c_[2];  // The length includes both F0 bytes and trailing A0.
    if (tu2c_expected_ < 7 || tu2c_expected_ > tu2c_.size()) {
      tu2c_sync_ = byte == 0xF0 ? 1 : 0;
      tu2c_size_ = tu2c_expected_ = 0;
      ESP_LOGV(TAG, "TU2C reader rejected invalid frame length");
    }
  }
  if (tu2c_expected_ && tu2c_size_ == tu2c_expected_) {
    uint8_t sum = 0;
    for (size_t i = 2; i + 2 < tu2c_size_; i++)
      sum += tu2c_[i];
    const bool valid = tu2c_[tu2c_size_ - 1] == 0xA0 && sum == tu2c_[tu2c_size_ - 2];
    process_frame_(Protocol::TU2C, tu2c_.data(), tu2c_size_, valid);
    tu2c_size_ = tu2c_expected_ = 0;
  }
}

void ToshibaAbClimate::check_reader_timeout_(uint32_t now) {
  if (last_reader_byte_ms_ == 0 || now - last_reader_byte_ms_ <= FRAME_TIMEOUT_MS)
    return;
  const bool incomplete = tcc_size_ > 0 || a0_size_ > 0 || a0_sync_ > 0 || tu2c_size_ > 0 || tu2c_sync_ > 0;
  if (incomplete) {
    ESP_LOGV(TAG, "Reader inter-byte timeout after %u ms; resetting partial frame",
             static_cast<unsigned>(now - last_reader_byte_ms_));
    reset_readers_();
  }
  last_reader_byte_ms_ = 0;
}

void ToshibaAbClimate::reset_readers_() {
  tcc_size_ = tcc_expected_ = 0;
  a0_size_ = a0_expected_ = 0;
  a0_sync_ = 0;
  tu2c_size_ = tu2c_expected_ = 0;
  tu2c_sync_ = 0;
}

void ToshibaAbClimate::advance_discovery_() {
  reset_readers_();
  discovery_stage_ms_ = millis();
  switch (discovery_state_) {
    case DiscoveryState::SCAN_TCC:
      discovery_state_ = DiscoveryState::SCAN_A0;
      diagnostic_("No TCC keepalive found; scanning A0 (EVEN parity)");
      break;
    case DiscoveryState::SCAN_A0:
      discovery_state_ = DiscoveryState::SCAN_TU2C;
      set_runtime_parity_(uart::UART_CONFIG_PARITY_NONE);
      diagnostic_("No A0 keepalive found; scanning TU2C (NONE parity)");
      break;
    case DiscoveryState::SCAN_TU2C:
      discovery_state_ = DiscoveryState::SCAN_TCC;
      set_runtime_parity_(uart::UART_CONFIG_PARITY_EVEN);
      diagnostic_("No TU2C keepalive found; restarting discovery with TCC (EVEN parity)");
      break;
    case DiscoveryState::COMPLETE:
      break;
  }
}

void ToshibaAbClimate::process_frame_(Protocol protocol, const uint8_t *data, size_t size, bool crc_ok) {
  uint8_t source = 0;
  uint8_t opcode1 = 0;
  uint8_t opcode2 = 0;
  frame_opcodes_(protocol, data, size, opcode1, opcode2);
  const bool keepalive = crc_ok && is_master_keepalive_(protocol, data, size, source);
  const char *description = keepalive ? "master keepalive" : (crc_ok ? "non-keepalive (ignored)" : "CRC failed");
  ESP_LOGD(TAG, "RX %s opcode1=0x%02X(%s) opcode2=0x%02X(%s) [%s] bytes=%s", protocol_name_(protocol), opcode1,
           opcode1_name_(opcode1), opcode2, opcode2_name_(opcode2), description, hex_(data, size).c_str());
  if (!crc_ok)
    return;
  if (keepalive)
    consider_keepalive_(protocol, source);
}

bool ToshibaAbClimate::is_master_keepalive_(Protocol protocol, const uint8_t *data, size_t size,
                                            uint8_t &source) const {
  switch (protocol) {
    case Protocol::TCC:
      if (size == 7 && data[2] == 0x10 && data[3] == 0x02 && data[4] == 0x80 && data[5] == 0x8A) {
        source = data[0];
        return true;
      }
      return false;
    case Protocol::TU2C:
      // TU2C keepalives are protocol-specific, not system-type-specific.
      if (size == 0x0A && data[7] == 0x3A) {
        source = data[3];
        return true;
      }
      return false;
    case Protocol::A0:
      // A0/HM: 00 before SRC and the mode bytes before SRC/DST are not IDs.
      // Wire layout is A0:00:TYPE:LEN:00:SRC_MODE:SRC:DST_MODE:DST:...
      // A0 water heartbeats and HM/A0 air keepalives use the same outer
      // PING/ALIVE opcode. CRC, complete framing, and the one-byte source are
      // sufficient; the inner data type is not required to be TCC's 0x8A.
      if (size >= 12 && data[2] == 0x10) {
        source = data[6];
        return true;
      }
      return false;
    default:
      return false;
  }
}

void ToshibaAbClimate::consider_keepalive_(Protocol protocol, uint8_t source) {
  protocol_detected_ = protocol;
  discovery_state_ = DiscoveryState::COMPLETE;

  const bool protocol_matches = protocol_setting_ == Protocol::AUTO || protocol_setting_ == protocol;
  const bool master_matches = master_setting_ == AUTO_ADDRESS || master_setting_ == source;
  protocol_confirmed_ = protocol_matches;
  master_address_confirmed_ = master_matches;

  if (!protocol_matches) {
    diagnostic_(std::string("Detected ") + protocol_name_(protocol) + " but YAML format is " +
                protocol_name_(protocol_setting_));
  }
  if (!master_matches) {
    char message[96];
    std::snprintf(message, sizeof(message), "Detected master 0x%02X but YAML master_address is 0x%02X", source,
                  master_setting_);
    diagnostic_(message);
  }
  if (master_setting_ == AUTO_ADDRESS)
    master_address_ = source;
  if (protocol_matches && master_matches) {
    char message[64];
    std::snprintf(message, sizeof(message), "Confirmed %s master 0x%02X", protocol_name_(protocol), source);
    diagnostic_(message);
  }
}

void ToshibaAbClimate::set_runtime_parity_(uart::UARTParityOptions parity) {
#ifdef USE_ESP8266
  if (hardware_uart_rx_pin_ == 13 && boot_ms_ != 0) {
    bus_serial.end();
    bus_serial.begin(2400, parity == uart::UART_CONFIG_PARITY_EVEN ? SERIAL_8E1 : SERIAL_8N1);
    bus_serial.swap();
  }
#endif
  if (parent_ != nullptr) {
    parent_->set_parity(parity);
    parent_->load_settings();
  }
}

void ToshibaAbClimate::diagnostic_(const std::string &message) {
  if (!diagnostic_history_.empty()) {
    const size_t last_separator = diagnostic_history_.rfind(" | ");
    const std::string last =
        last_separator == std::string::npos ? diagnostic_history_ : diagnostic_history_.substr(last_separator + 3);
    if (last == message)
      return;
    diagnostic_history_ += " | ";
  }
  diagnostic_history_ += message;
  while (diagnostic_history_.size() > 255) {
    const size_t separator = diagnostic_history_.find(" | ");
    if (separator == std::string::npos) {
      diagnostic_history_.erase(0, diagnostic_history_.size() - 255);
      break;
    }
    diagnostic_history_.erase(0, separator + 3);
  }
  ESP_LOGI(TAG, "%s", message.c_str());
  if (diagnostic_ != nullptr)
    diagnostic_->publish_state(diagnostic_history_);
}

const char *ToshibaAbClimate::protocol_name_(Protocol protocol) {
  switch (protocol) {
    case Protocol::TCC:
      return "TCC";
    case Protocol::TU2C:
      return "TU2C";
    case Protocol::A0:
      return "A0";
    default:
      return "auto";
  }
}

const char *ToshibaAbClimate::opcode1_name_(uint8_t opcode) {
  switch (opcode) {
    case 0x10:
      return "PING/ALIVE";
    case 0x11:
      return "WRITE/CONTROL";
    case 0x15:
      return "REQUEST/QUERY";
    case 0x17:
      return "SENSOR QUERY";
    case 0x18:
      return "RESPONSE/REPORT";
    case 0x1A:
      return "SENSOR ANSWER";
    case 0x1C:
      return "STATUS";
    case 0x52:
      return "REMOTE START";
    case 0x55:
      return "REMOTE TEMPERATURE";
    case 0x58:
      return "EXTENDED STATUS";
    default:
      return "UNKNOWN";
  }
}

const char *ToshibaAbClimate::opcode2_name_(uint8_t opcode) {
  switch (opcode) {
    case 0x02:
      return "DN CODE";
    case 0x07:
      return "SAVE RATIO";
    case 0x08:
      return "MODEL INFO";
    case 0x0A:
      return "TEMPERATURE LIMITS";
    case 0x0C:
      return "PING/PONG";
    case 0x0D:
      return "ANNOUNCE";
    case 0x0F:
      return "SETPOINTS";
    case 0x10:
      return "FAN CAPABILITIES";
    case 0x27:
      return "ERRORS";
    case 0x31:
      return "STATUS";
    case 0x3A:
      return "ALIVE";
    case 0x41:
      return "POWER";
    case 0x42:
      return "MODE";
    case 0x4C:
      return "TEMPERATURE/FAN";
    case 0x54:
      return "SAVE";
    case 0x5C:
      return "DATA VALUE";
    case 0x80:
      return "SENSOR ID";
    case 0x81:
      return "STATUS";
    case 0x86:
      return "MODE STATUS";
    case 0x8A:
      return "ALIVE";
    case 0xA1:
      return "ACK";
    case 0xA2:
      return "DATA UNAVAILABLE";
    case 0xA3:
      return "BUSY";
    case 0xE8:
      return "TIME COUNTER";
    case 0xEF:
      return "SENSOR VALUE";
    default:
      return "UNKNOWN";
  }
}

void ToshibaAbClimate::frame_opcodes_(Protocol protocol, const uint8_t *data, size_t size, uint8_t &opcode1,
                                      uint8_t &opcode2) {
  opcode1 = opcode2 = 0;
  switch (protocol) {
    case Protocol::TCC:
      if (size > 2)
        opcode1 = data[2];
      if (size > 5)
        opcode2 = data[5];
      break;
    case Protocol::A0:
      if (size > 2)
        opcode1 = data[2];
      if (size > 10)
        opcode2 = data[10];
      break;
    case Protocol::TU2C:
      if (size > 6)
        opcode1 = data[6];
      if (size > 7)
        opcode2 = data[7];
      break;
    default:
      break;
  }
}

std::string ToshibaAbClimate::hex_(const uint8_t *data, size_t size) {
  std::string output;
  char byte[4];
  for (size_t i = 0; i < size; i++) {
    std::snprintf(byte, sizeof(byte), "%02X", data[i]);
    if (i)
      output += ':';
    output += byte;
  }
  return output;
}

uint16_t ToshibaAbClimate::crc16_mcrf4xx_(const uint8_t *data, size_t size) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < size; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++)
      crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : crc >> 1;
  }
  return crc;
}

climate::ClimateTraits ToshibaAbClimate::traits() {
  auto traits = climate::ClimateTraits();
  traits.set_supported_modes({climate::CLIMATE_MODE_OFF});
  traits.set_visual_min_temperature(5);
  traits.set_visual_max_temperature(35);
  traits.set_visual_temperature_step(0.5);
  return traits;
}

void ToshibaAbClimate::control(const climate::ClimateCall &call) {
  ESP_LOGV(TAG, "Climate control ignored while the component is identification-only");
}

void ToshibaAbClimate::dump_config() {
  ESP_LOGCONFIG(TAG, "Toshiba AB thermostat (identification only)");
  ESP_LOGCONFIG(TAG, "  System type: %s", system_type_ == SystemType::AIR ? "Air" : "Water");
  ESP_LOGCONFIG(TAG, "  Configured format: %s", protocol_name_(protocol_setting_));
  ESP_LOGCONFIG(TAG, "  Master address: %s",
                master_setting_ == AUTO_ADDRESS ? "auto" : hex_(&master_setting_, 1).c_str());
  ESP_LOGCONFIG(
      TAG, "  ESP address: %s",
      esp_address_ == AUTO_ADDRESS ? "auto (unresolved while identification-only)" : hex_(&esp_address_, 1).c_str());
  ESP_LOGCONFIG(TAG, "  Protocol confirmed: %s", YESNO(protocol_confirmed_));
  ESP_LOGCONFIG(TAG, "  Master confirmed: %s", YESNO(master_address_confirmed_));
}

}  // namespace toshiba_ab
}  // namespace esphome
