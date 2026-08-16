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
  master_address_ = master_setting_;
  this->mode = climate::CLIMATE_MODE_OFF;
  this->target_temperature = 22.0f;
  set_runtime_parity_(uart::UART_CONFIG_PARITY_EVEN);
  diagnostic_("Listening for a master keepalive");

#ifdef USE_ESP8266
  if (hardware_uart_rx_pin_ == 13)
    ESP_LOGCONFIG(TAG, "UART0 RX swapped from GPIO3 to GPIO13");
#endif
}

void ToshibaAbClimate::loop() {
  const uint32_t elapsed = millis() - boot_ms_;
  if (!using_none_parity_ && elapsed >= LISTEN_ONLY_MS && !protocol_confirmed_) {
    using_none_parity_ = true;
    tcc_size_ = a0_size_ = tu2c_size_ = 0;
    set_runtime_parity_(uart::UART_CONFIG_PARITY_NONE);
    ESP_LOGI(TAG, "No matching even-parity keepalive; scanning TU2C with no parity");
  }

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
  if (using_none_parity_)
    read_tu2c_byte_(byte);
  else
    read_even_byte_(byte);
}

void ToshibaAbClimate::read_even_byte_(uint8_t byte) {
  // A0 has an unambiguous two-byte wrapper, so collect it independently.
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

  // TCC frames have no wrapper: a length byte at offset 3 determines the end.
  if (tcc_size_ < tcc_.size())
    tcc_[tcc_size_++] = byte;
  if (tcc_size_ == 4) {
    tcc_expected_ = static_cast<size_t>(tcc_[3]) + 5;
    if (tcc_[3] < 2 || tcc_expected_ > tcc_.size()) {
      for (size_t i = 1; i < tcc_size_; i++)
        tcc_[i - 1] = tcc_[i];
      tcc_size_--;
      tcc_expected_ = 0;
    }
  }
  if (tcc_expected_ && tcc_size_ == tcc_expected_) {
    uint8_t crc = 0;
    for (size_t i = 0; i + 1 < tcc_size_; i++)
      crc ^= tcc_[i];
    process_frame_(Protocol::TCC, tcc_.data(), tcc_size_, crc == tcc_[tcc_size_ - 1]);
    tcc_size_ = tcc_expected_ = 0;
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

void ToshibaAbClimate::process_frame_(Protocol protocol, const uint8_t *data, size_t size, bool crc_ok) {
  uint8_t source = 0;
  const bool keepalive = crc_ok && is_master_keepalive_(protocol, data, size, source);
  const char *description = keepalive ? "master keepalive" : (crc_ok ? "non-keepalive (ignored)" : "CRC failed");
  ESP_LOGD(TAG, "RX %s: %s [%s]", protocol_name_(protocol), hex_(data, size).c_str(), description);
  if (!crc_ok)
    return;
  if (keepalive)
    consider_keepalive_(protocol, source);
}

bool ToshibaAbClimate::is_master_keepalive_(Protocol protocol, const uint8_t *data, size_t size,
                                            uint8_t &source) const {
  switch (protocol) {
    case Protocol::TCC:
      if (system_type_ == SystemType::AIR && size >= 7 && data[2] == 0x10) {
        source = data[0];
        return true;
      }
      return false;
    case Protocol::TU2C:
      // Air TU2C and first-generation water systems share the 0x0A/0x3A
      // master keepalive, but are deliberately selected by system_type here.
      if (size == 0x0A && data[7] == 0x3A) {
        source = data[3];
        return true;
      }
      return false;
    case Protocol::A0:
      // A0/HM: 00 before SRC and the mode bytes before SRC/DST are not IDs.
      // Wire layout is A0:00:TYPE:LEN:00:SRC_MODE:SRC:DST_MODE:DST:...
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
  if (protocol_setting_ != Protocol::AUTO && protocol_setting_ != protocol) {
    diagnostic_(std::string("Detected ") + protocol_name_(protocol) + " but YAML format is " +
                protocol_name_(protocol_setting_));
    return;
  }
  protocol_detected_ = protocol;
  protocol_confirmed_ = true;

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
  if (diagnostic_ != nullptr)
    diagnostic_->publish_state(message);
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
