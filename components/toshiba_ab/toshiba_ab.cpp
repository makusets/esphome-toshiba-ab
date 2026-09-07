#include "toshiba_ab.h"
#include "esphome/core/log.h"
#include <algorithm>
#include <cstdio>

#ifdef USE_ESP8266
#include <HardwareSerial.h>
#endif

namespace esphome {
namespace toshiba_ab {

static const char *const TAG = "toshiba_ab";
constexpr ProtocolValue ToshibaAbClimate::MASTER_KEEPALIVE_OPCODE;
constexpr ProtocolValue ToshibaAbClimate::MASTER_KEEPALIVE_LENGTH;
constexpr ProtocolValue ToshibaAbClimate::MASTER_KEEPALIVE_DATA_TYPE;
constexpr ProtocolValue ToshibaAbClimate::REMOTE_PING_OPCODE;
constexpr ProtocolValue ToshibaAbClimate::REMOTE_PING_LENGTH;
constexpr ProtocolValue ToshibaAbClimate::REMOTE_PING_DATA_TYPE;

ToshibaAbThermostat::ToshibaAbThermostat(ToshibaAbClimate *parent, WaterCircuit circuit)
    : parent_(parent), circuit_(circuit) {
  this->mode = climate::CLIMATE_MODE_OFF;
  this->target_temperature = circuit == WaterCircuit::DHW ? 50.0f : 22.0f;
}

climate::ClimateTraits ToshibaAbThermostat::traits() {
  auto traits = climate::ClimateTraits();
  traits.set_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE | climate::CLIMATE_SUPPORTS_ACTION);
  // Hydronic controllers expose these operating presets independently of the
  // circuit's heat/cool mode. Keep them on DHW and both zones so each entity
  // can eventually report and control the corresponding water-system state.
  traits.set_supported_presets(
      {climate::CLIMATE_PRESET_NONE, climate::CLIMATE_PRESET_BOOST, climate::CLIMATE_PRESET_ECO});
  if (circuit_ == WaterCircuit::DHW) {
    traits.set_supported_modes({climate::CLIMATE_MODE_OFF, climate::CLIMATE_MODE_HEAT});
    traits.set_visual_min_temperature(45);
    traits.set_visual_max_temperature(60);
  } else {
    traits.set_supported_modes({climate::CLIMATE_MODE_OFF, climate::CLIMATE_MODE_HEAT, climate::CLIMATE_MODE_COOL});
    traits.set_visual_min_temperature(20);
    traits.set_visual_max_temperature(65);
  }
  traits.set_visual_temperature_step(0.5);
  return traits;
}

void ToshibaAbThermostat::control(const climate::ClimateCall &call) { parent_->control_water(circuit_, call); }

void ResetButton::press_action() {
  if (parent_ != nullptr)
    parent_->reset();
}

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
                  ? "Discovery started: scanning TCC master keepalives (0-20s)"
                  : std::string("Listening for a ") + protocol_name_(protocol_setting_) + " master keepalive");

#ifdef USE_ESP8266
  if (hardware_uart_rx_pin_ == 13)
    ESP_LOGCONFIG(TAG, "UART0 RX swapped from GPIO3 to GPIO13");
#endif
}

void ToshibaAbClimate::reset() {
  boot_ms_ = millis();
  master_address_ = master_setting_;
  master_address_confirmed_ = false;
  protocol_detected_ = Protocol::AUTO;
  protocol_confirmed_ = false;
  discovery_finished_ = false;
  reader_reset_count_ = 0;
  remotes_.clear();
  select_scan_protocol_(protocol_setting_ == Protocol::AUTO ? Protocol::TCC : protocol_setting_);
  diagnostic_(protocol_setting_ == Protocol::AUTO
                  ? "Reset: scanning TCC master keepalives (0-20s)"
                  : std::string("Reset: listening for a ") + protocol_name_(protocol_setting_) + " master keepalive");
}

void ToshibaAbClimate::loop() {
  const uint32_t now = millis();
  update_discovery_(now);
  check_reader_timeout_(now);
  expire_remotes_(now);

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
        a0_expected_ = static_cast<size_t>(a0_[3]) + 6;  // wrapper, opcode, length, body, CRC16
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

  // TCC starts directly with source/destination bytes; unlike A0 and TU2C it
  // has no reserved framing prefix that identifies a frame boundary. Keep a
  // sliding candidate so a bad length or CRC cannot leave the reader
  // permanently aligned to noise/a truncated frame.
  if (tcc_size_ >= tcc_.size())
    reset_readers_();

  // The first received byte is the source address. Values above 0xA0 are not
  // valid TCC participants and are most likely line noise; discard the byte,
  // reset the reader, and keep waiting for a valid source byte.
  if (tcc_size_ == 0 && byte > 0xA0) {
    reset_readers_();
    reader_reset_count_++;
    return;
  }
  tcc_[tcc_size_++] = byte;

  while (tcc_size_ >= 4) {
    // Recovery may have shifted a later byte into the source position. Apply
    // the same rule there and abandon the candidate buffer completely.
    if (tcc_[0] > 0xA0) {
      reset_readers_();
      reader_reset_count_++;
      break;
    }
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
    std::snprintf(message, sizeof(message), "Discovery finished: no master keepalive found in 60s (%u reader resyncs)",
                  static_cast<unsigned>(reader_reset_count_));
    diagnostic_(message);
    return;
  }
  if (wanted != scan_protocol_) {
    select_scan_protocol_(wanted);
    char message[96];
    std::snprintf(message, sizeof(message), "Discovery: scanning %s master keepalives (%us-%us)",
                  protocol_name_(wanted), static_cast<unsigned>(elapsed / 20000 * 20),
                  static_cast<unsigned>(elapsed / 20000 * 20 + 20));
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
  const bool master_keepalive = crc_ok && is_master_keepalive_(protocol, data, size, source);
  const bool remote_ping = crc_ok && !master_keepalive && is_remote_ping_(protocol, data, size, source);
  std::string description;
  if (!crc_ok) {
    description = "CRC failed";
  } else if (master_keepalive) {
    description = "master keepalive " + hex_(&source, 1);
  } else if (remote_ping) {
    description = "remote ping " + hex_(&source, 1);
  }
  ESP_LOGD(TAG, "RX %s: %s [%s%s%s]", protocol_name_(protocol), colored_hex_(protocol, data, size, crc_ok).c_str(),
           ESPHOME_LOG_COLOR(ESPHOME_LOG_COLOR_GREEN), description.c_str(), ESPHOME_LOG_RESET_COLOR);
  if (!crc_ok)
    return;
  if (master_keepalive)
    consider_keepalive_(protocol, source);
  else if (remote_ping)
    observe_remote_(source, millis());
}

bool ToshibaAbClimate::is_master_keepalive_(Protocol protocol, const uint8_t *data, size_t size,
                                            uint8_t &source) const {
  switch (protocol) {
    case Protocol::TCC:
      // Main's normal-protocol path first validates the complete frame, then
      // only treats OPCODE_PING from the master as a keepalive. During auto
      // discovery the master is not known yet, so also require the canonical
      // data-type keepalive signature and its exact length
      // instead of accepting every opcode=0x10 frame.
      source = size > 0 ? data[0] : 0;
      break;
    case Protocol::TU2C:
      // Air TU2C and first-generation water systems share the 00:3A tail.
      source = size > 3 ? data[3] : 0;
      break;
    case Protocol::A0:
      // A0 water and air units use the same opcode 0x10 keepalive.
      // Wire layout is A0:00:OPCODE:LEN:00:SRC_MODE:SRC:DST_MODE:DST:...
      source = size > 6 ? data[6] : 0;
      break;
    default:
      return false;
  }

  const bool signature_matches =
      frame_length_(protocol, data, size) == MASTER_KEEPALIVE_LENGTH.for_protocol(protocol) &&
      opcode_(protocol, data, size) == MASTER_KEEPALIVE_OPCODE.for_protocol(protocol) &&
      data_type_(protocol, data, size) == MASTER_KEEPALIVE_DATA_TYPE.for_protocol(protocol);

  // The first master keepalive establishes the source address. Once discovery
  // has completed, do not treat traffic from another participant as a master
  // keepalive. Remote pings have separate opcodes and signatures.
  return signature_matches && (!protocol_confirmed_ || source == master_address_);
}

bool ToshibaAbClimate::is_remote_ping_(Protocol protocol, const uint8_t *data, size_t size, uint8_t &source) const {
  if (!protocol_confirmed_ || !master_address_confirmed_)
    return false;

  uint8_t destination = 0;
  switch (protocol) {
    case Protocol::TCC:
      source = size > 0 ? data[0] : 0;
      destination = size > 1 ? data[1] : 0;
      break;
    case Protocol::TU2C:
      source = size > 3 ? data[3] : 0;
      destination = size > 4 ? data[4] : 0;
      break;
    case Protocol::A0:
      source = size > 6 ? data[6] : 0;
      destination = size > 8 ? data[8] : 0;
      break;
    default:
      return false;
  }

  // Remote pings are requests sent to the master. Do not identify traffic for
  // another indoor unit as a remote attached to this one.
  if (destination != master_address_ || source == master_address_)
    return false;

  const uint16_t data_type = data_type_(protocol, data, size);
  const bool data_type_matches = data_type == REMOTE_PING_DATA_TYPE.for_protocol(protocol) ||
                                 (protocol == Protocol::TU2C && data_type == TU2C_FIRST_GEN_REMOTE_PING_DATA_TYPE);
  return frame_length_(protocol, data, size) == REMOTE_PING_LENGTH.for_protocol(protocol) &&
         opcode_(protocol, data, size) == REMOTE_PING_OPCODE.for_protocol(protocol) && data_type_matches;
}

void ToshibaAbClimate::consider_keepalive_(Protocol protocol, uint8_t source) {
  // Keepalives continue for the lifetime of the bus, but confirmation is a
  // discovery transition rather than a periodic event.
  if (protocol_confirmed_ && master_address_confirmed_)
    return;

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

void ToshibaAbClimate::observe_remote_(uint8_t address, uint32_t now) {
  for (auto &remote : remotes_) {
    if (remote.address == address) {
      remote.last_seen = now;
      return;
    }
  }

  remotes_.push_back({address, now});
  std::sort(remotes_.begin(), remotes_.end(),
            [](const RemotePresence &left, const RemotePresence &right) { return left.address < right.address; });
  diagnostic_(std::string("Remote discovered: ") + hex_(&address, 1));
}

void ToshibaAbClimate::expire_remotes_(uint32_t now) {
  std::vector<uint8_t> expired;
  remotes_.erase(std::remove_if(remotes_.begin(), remotes_.end(),
                                [now, &expired](const RemotePresence &remote) {
                                  // Unsigned subtraction keeps this correct when millis() wraps.
                                  if (now - remote.last_seen < REMOTE_EXPIRY_MS)
                                    return false;
                                  expired.push_back(remote.address);
                                  return true;
                                }),
                 remotes_.end());
  for (uint8_t address : expired)
    diagnostic_(std::string("Remote removed: ") + hex_(&address, 1));
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
  if (diagnostic_sensor_ != nullptr)
    diagnostic_sensor_->publish_state(message);
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

uint8_t ToshibaAbClimate::opcode_(Protocol protocol, const uint8_t *data, size_t size) {
  if (protocol == Protocol::TCC)
    return size > 2 ? data[2] : 0;
  if (protocol == Protocol::TU2C)
    return size > 6 ? data[6] : 0;
  return size > 2 ? data[2] : 0;
}

uint8_t ToshibaAbClimate::frame_length_(Protocol protocol, const uint8_t *data, size_t size) {
  const size_t offset = protocol == Protocol::TU2C ? 2 : 3;
  return size > offset ? data[offset] : 0;
}

uint16_t ToshibaAbClimate::data_type_(Protocol protocol, const uint8_t *data, size_t size) {
  if (protocol == Protocol::TCC)
    return size > 5 ? data[5] : 0;
  if (protocol == Protocol::TU2C)
    return size > 7 ? data[7] : 0;
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

std::string ToshibaAbClimate::colored_hex_(Protocol protocol, const uint8_t *data, size_t size, bool crc_ok) {
  if (!crc_ok)
    return ESPHOME_LOG_COLOR(ESPHOME_LOG_COLOR_YELLOW) + hex_(data, size) + ESPHOME_LOG_RESET_COLOR;

  std::string output;
  char byte[4];
  for (size_t i = 0; i < size; i++) {
    if (i)
      output += ':';

    bool address = false;
    bool command = false;
    switch (protocol) {
      case Protocol::TCC:
        address = i == 0 || i == 1;
        command = i == 2 || i == 5;
        break;
      case Protocol::TU2C:
        address = i == 3 || i == 4;
        command = i == 6 || i == 7;
        break;
      case Protocol::A0:
        address = i == 5 || i == 6 || i == 7 || i == 8;
        command = i == 2 || i == 9 || i == 10;
        break;
      default:
        break;
    }

    if (address)
      output += ESPHOME_LOG_COLOR(ESPHOME_LOG_COLOR_RED);
    else if (command)
      output += ESPHOME_LOG_COLOR(ESPHOME_LOG_COLOR_YELLOW);
    std::snprintf(byte, sizeof(byte), "%02X", data[i]);
    output += byte;
    if (address || command)
      output += ESPHOME_LOG_RESET_COLOR;
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
  if (system_type_ == SystemType::AIR) {
    // Identification mode must still expose the complete climate capability
    // schema. This makes every field available in the native API response
    // while protocol support is being rebuilt incrementally.
    traits.set_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE | climate::CLIMATE_SUPPORTS_ACTION);
    traits.set_supported_modes({climate::CLIMATE_MODE_OFF, climate::CLIMATE_MODE_HEAT_COOL, climate::CLIMATE_MODE_COOL,
                                climate::CLIMATE_MODE_HEAT, climate::CLIMATE_MODE_FAN_ONLY, climate::CLIMATE_MODE_DRY,
                                climate::CLIMATE_MODE_AUTO});
    traits.set_supported_fan_modes(
        {climate::CLIMATE_FAN_AUTO, climate::CLIMATE_FAN_LOW, climate::CLIMATE_FAN_MEDIUM, climate::CLIMATE_FAN_HIGH});
    traits.set_supported_swing_modes({climate::CLIMATE_SWING_OFF, climate::CLIMATE_SWING_BOTH,
                                      climate::CLIMATE_SWING_VERTICAL, climate::CLIMATE_SWING_HORIZONTAL});
    traits.set_supported_presets({climate::CLIMATE_PRESET_NONE, climate::CLIMATE_PRESET_BOOST,
                                  climate::CLIMATE_PRESET_ECO, climate::CLIMATE_PRESET_SLEEP});
  } else {
    traits.set_supported_modes({climate::CLIMATE_MODE_OFF});
  }
  traits.set_visual_min_temperature(16);
  traits.set_visual_max_temperature(32);
  traits.set_visual_temperature_step(0.5);
  return traits;
}

void ToshibaAbClimate::control(const climate::ClimateCall &call) {
  ESP_LOGV(TAG, "Climate control ignored while the component is identification-only");
}

void ToshibaAbClimate::control_water(WaterCircuit circuit, const climate::ClimateCall &call) {
  const char *name = circuit == WaterCircuit::DHW ? "DHW" : (circuit == WaterCircuit::ZONE_1 ? "Zone 1" : "Zone 2");
  ESP_LOGV(TAG, "%s climate control ignored while the component is identification-only", name);
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
