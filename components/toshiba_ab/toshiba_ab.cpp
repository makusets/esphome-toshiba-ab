#include "toshiba_ab.h"
#include "esphome/core/log.h"
#include <cstdio>

#ifdef USE_ESP8266
#include <HardwareSerial.h>
#endif

namespace esphome {
namespace toshiba_ab {

static const char *const TAG = "toshiba_ab";
constexpr ProtocolOpcodes ToshibaAbClimate::KEEPALIVE_OPCODE1;

#ifdef USE_ESP8266
static HardwareSerial bus_serial(UART0);
#endif

void ToshibaAbClimate::setup() {
  boot_ms_ = millis();
  master_address_ = master_setting_;
  this->mode = climate::CLIMATE_MODE_OFF;
  this->target_temperature = 22.0f;
  select_scan_protocol_(protocol_setting_ == Protocol::AUTO ? Protocol::TCC : protocol_setting_);
  diagnostic_(protocol_setting_ == Protocol::AUTO
                  ? "Discovery started: scanning TCC keepalives (0-20s)"
                  : std::string("Listening for a ") + protocol_name_(protocol_setting_) + " keepalive");

#ifdef USE_ESP8266
  if (hardware_uart_rx_pin_ == 13)
    ESP_LOGCONFIG(TAG, "UART0 RX swapped from GPIO3 to GPIO13");
#endif
}

void ToshibaAbClimate::loop() {
  const uint32_t now = millis();
  update_discovery_(now);
  check_reader_timeout_(now);

  uint8_t byte;
#ifdef USE_ESP8266
  if (hardware_uart_rx_pin_ == 13) {
    while (bus_serial.available() && bus_serial.readBytes(&byte, 1) == 1)
      read_byte_(byte);
    return;
  }
#endif
  while (available() && read_byte(&byte))
    read_byte_(byte);
}

void ToshibaAbClimate::read_byte_(uint8_t byte) {
  ESP_LOGVV(TAG, "Reader accepted byte 0x%02X", byte);
  last_byte_ms_ = millis();
  if (scan_protocol_ == Protocol::TU2C)
    read_tu2c_byte_(byte);
  else if (scan_protocol_ == Protocol::TCC)
    read_even_byte_(byte);
  else if (scan_protocol_ == Protocol::A0)
    read_even_byte_(byte);
}

void ToshibaAbClimate::read_even_byte_(uint8_t byte) {
  if (scan_protocol_ == Protocol::A0) {
    // A0 has an unambiguous two-byte wrapper.
    if (a0_size_ == 0) {
      if (a0_sync_ == 0 && byte == 0xA0)
        a0_sync_ = 1;
      else if (a0_sync_ == 1 && byte == 0x00) {
        a0_[0] = 0xA0;
        a0_[1] = 0x00;
        a0_size_ = 2;
        a0_sync_ = 0;
      } else
        a0_sync_ = byte == 0xA0 ? 1 : 0;
    } else {
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
    return;
  }

  // TCC has no sync marker. Keep a sliding candidate so a bad length or CRC
  // cannot leave the reader permanently aligned to noise/a truncated frame.
  if (tcc_size_ < tcc_.size())
    tcc_[tcc_size_++] = byte;
  else {
    reset_readers_();
    tcc_[tcc_size_++] = byte;
  }
  while (tcc_size_ >= 4) {
    if (tcc_expected_ == 0)
      tcc_expected_ = static_cast<size_t>(tcc_[3]) + 5;
    if (tcc_[3] < 2 || tcc_expected_ > tcc_.size()) {
      for (size_t i = 1; i < tcc_size_; i++)
        tcc_[i - 1] = tcc_[i];
      tcc_size_--;
      tcc_expected_ = 0;
      reader_reset_count_++;
      continue;
    }
    if (tcc_size_ < tcc_expected_)
      break;
    uint8_t crc = 0;
    for (size_t i = 0; i + 1 < tcc_expected_; i++)
      crc ^= tcc_[i];
    const bool valid = crc == tcc_[tcc_expected_ - 1];
    process_frame_(Protocol::TCC, tcc_.data(), tcc_expected_, valid);
    if (valid) {
      const size_t consumed = tcc_expected_;
      for (size_t i = consumed; i < tcc_size_; i++)
        tcc_[i - consumed] = tcc_[i];
      tcc_size_ -= consumed;
    } else {
      for (size_t i = 1; i < tcc_size_; i++)
        tcc_[i - 1] = tcc_[i];
      tcc_size_--;
      reader_reset_count_++;
    }
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
    if (tu2c_expected_ < 7 || tu2c_expected_ > tu2c_.size())
      tu2c_size_ = tu2c_expected_ = 0;
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

void ToshibaAbClimate::update_discovery_(uint32_t now) {
  if (protocol_setting_ != Protocol::AUTO || protocol_confirmed_ || discovery_finished_)
    return;

  const uint32_t elapsed = now - boot_ms_;
  Protocol wanted =
      elapsed < PROTOCOL_SCAN_MS ? Protocol::TCC : (elapsed < 2 * PROTOCOL_SCAN_MS ? Protocol::A0 : Protocol::TU2C);
  if (elapsed >= DISCOVERY_MS) {
    discovery_finished_ = true;
    char message[128];
    std::snprintf(message, sizeof(message), "Discovery finished: no keepalive found in 60s (%u reader resyncs)",
                  static_cast<unsigned>(reader_reset_count_));
    diagnostic_(message);
    return;
  }
  if (wanted != scan_protocol_) {
    select_scan_protocol_(wanted);
    char message[96];
    std::snprintf(message, sizeof(message), "Discovery: scanning %s keepalives (%us-%us)", protocol_name_(wanted),
                  static_cast<unsigned>(elapsed / 20000 * 20), static_cast<unsigned>(elapsed / 20000 * 20 + 20));
    diagnostic_(message);
  }
}

void ToshibaAbClimate::select_scan_protocol_(Protocol protocol) {
  scan_protocol_ = protocol;
  reset_readers_();
  set_runtime_parity_(protocol == Protocol::TU2C ? uart::UART_CONFIG_PARITY_NONE : uart::UART_CONFIG_PARITY_EVEN);
}

void ToshibaAbClimate::reset_readers_(bool timeout) {
  if (timeout)
    reader_reset_count_++;
  tcc_size_ = tcc_expected_ = 0;
  a0_size_ = a0_expected_ = 0;
  a0_sync_ = 0;
  tu2c_size_ = tu2c_expected_ = 0;
  tu2c_sync_ = 0;
  last_byte_ms_ = 0;
}

void ToshibaAbClimate::check_reader_timeout_(uint32_t now) {
  const bool partial = tcc_size_ != 0 || a0_size_ != 0 || a0_sync_ != 0 || tu2c_size_ != 0 || tu2c_sync_ != 0;
  if (partial && last_byte_ms_ != 0 && now - last_byte_ms_ > BYTE_TIMEOUT_MS) {
    ESP_LOGV(TAG, "%s inter-byte timeout after %ums; discarding partial frame", protocol_name_(scan_protocol_),
             static_cast<unsigned>(now - last_byte_ms_));
    reset_readers_(true);
  }
}

void ToshibaAbClimate::process_frame_(Protocol protocol, const uint8_t *data, size_t size, bool crc_ok) {
  uint8_t source = 0;
  const bool keepalive = crc_ok && is_master_keepalive_(protocol, data, size, source);
  const char *description = keepalive ? "master keepalive" : (crc_ok ? "non-keepalive (ignored)" : "CRC failed");
  ESP_LOGD(TAG, "RX %s opcode1=0x%02X opcode2=0x%04X: %s [%s]", protocol_name_(protocol),
           opcode1_(protocol, data, size), opcode2_(protocol, data, size), hex_(data, size).c_str(), description);
  if (!crc_ok)
    return;
  if (keepalive)
    consider_keepalive_(protocol, source);
}

bool ToshibaAbClimate::is_master_keepalive_(Protocol protocol, const uint8_t *data, size_t size,
                                            uint8_t &source) const {
  switch (protocol) {
    case Protocol::TCC:
      // Main's normal-protocol path first validates the complete frame, then
      // only treats OPCODE_PING from the master as a keepalive. During auto
      // discovery the master is not known yet, so also require the canonical
      // read-mode/opcode2 keepalive signature (80:8A) and its exact LEN=2
      // shape instead of accepting every opcode1=0x10 frame.
      if (size == 7 && data[3] == 0x02 && data[4] == 0x80 &&
          opcode1_(protocol, data, size) == KEEPALIVE_OPCODE1.for_protocol(protocol) &&
          opcode2_(protocol, data, size) == 0x8A && (master_setting_ == AUTO_ADDRESS || data[0] == master_setting_)) {
        source = data[0];
        return true;
      }
      return false;
    case Protocol::TU2C:
      // Main identifies this with the total length and the complete 00:3A
      // tail signature. Air TU2C and first-generation water systems share it.
      if (size == 0x0A && data[2] == 0x0A && data[size - 4] == 0x00 &&
          opcode1_(protocol, data, size) == KEEPALIVE_OPCODE1.for_protocol(protocol) &&
          (master_setting_ == AUTO_ADDRESS || data[3] == master_setting_)) {
        source = data[3];
        return true;
      }
      return false;
    case Protocol::A0:
      // A0 water and air units use the same type 0x10 keepalive.
      // Wire layout is A0:00:TYPE:LEN:00:SRC_MODE:SRC:DST_MODE:DST:...
      // Main additionally identifies 08:00 as the A0 master source; without
      // that check a valid type 0x10 frame from another bus participant could
      // incorrectly end discovery.
      if (size >= 12 && data[4] == 0x00 && data[5] == 0x08 && data[6] == 0x00 &&
          opcode1_(protocol, data, size) == KEEPALIVE_OPCODE1.for_protocol(protocol) &&
          (master_setting_ == AUTO_ADDRESS || data[6] == master_setting_)) {
        source = data[6];
        return true;
      }
      return false;
    default:
      return false;
  }
}

void ToshibaAbClimate::consider_keepalive_(Protocol protocol, uint8_t source) {
  if (protocol_setting_ != Protocol::AUTO && protocol_setting_ != protocol) {
    diagnostic_(std::string("Detected ") + protocol_name_(protocol) + " but YAML format is " +
                protocol_name_(protocol_setting_));
    return;
  }
  protocol_detected_ = protocol;
  protocol_confirmed_ = true;
  discovery_finished_ = true;

  if (master_setting_ != AUTO_ADDRESS && master_setting_ != source) {
    char message[96];
    std::snprintf(message, sizeof(message), "Detected master 0x%02X but YAML master_address is 0x%02X", source,
                  master_setting_);
    diagnostic_(message);
    return;
  }
  master_address_ = source;
  master_address_confirmed_ = true;
  diagnostic_(std::string("Confirmed ") + protocol_name_(protocol) + " master " + hex_(&source, 1));
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
  ESP_LOGI(TAG, "%s", message.c_str());
  if (!diagnostic_history_.empty())
    diagnostic_history_ += '\n';
  diagnostic_history_ += message;
  while (diagnostic_history_.size() > 255) {
    const size_t newline = diagnostic_history_.find('\n');
    if (newline == std::string::npos) {
      diagnostic_history_.erase(0, diagnostic_history_.size() - 255);
      break;
    }
    diagnostic_history_.erase(0, newline + 1);
  }
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

uint8_t ToshibaAbClimate::opcode1_(Protocol protocol, const uint8_t *data, size_t size) {
  if (protocol == Protocol::TCC)
    return size > 2 ? data[2] : 0;
  if (protocol == Protocol::TU2C)
    return size > 7 ? data[7] : 0;  // TU2C semantic opcode is at stripped raw[5].
  return size > 2 ? data[2] : 0;
}

uint16_t ToshibaAbClimate::opcode2_(Protocol protocol, const uint8_t *data, size_t size) {
  if (protocol == Protocol::TCC)
    return size > 5 ? data[5] : 0;
  if (protocol == Protocol::TU2C)
    return size > 6 ? data[6] : 0;
  return size > 10 ? (static_cast<uint16_t>(data[9]) << 8) | data[10] : 0;
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
  ESP_LOGCONFIG(TAG, "  ESP address: %s", esp_address_ == AUTO_ADDRESS ? "auto" : hex_(&esp_address_, 1).c_str());
}

}  // namespace toshiba_ab
}  // namespace esphome
