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

// Offsets refer to the complete wire frame, including wrappers. The HM
// normalisation in main inserted padding; the unified A0 reader does not.
static constexpr uint16_t NO_FIELD = 0xFFFF;
static constexpr ProtocolValue AIR_MODE_POWER_OFFSET{6, 8, 11};
static constexpr ProtocolValue AIR_FAN_VENT_OFFSET{7, 9, 12};
static constexpr ProtocolValue AIR_FLAGS_OFFSET{8, 10, 13};
static constexpr ProtocolValue AIR_TARGET_OFFSET{10, 12, 15};
static constexpr ProtocolValue AIR_CURRENT_OFFSET{11, 13, 16};
static constexpr ProtocolValue AIR_PRESET_OFFSET{NO_FIELD, 14, NO_FIELD};
static constexpr ProtocolValue AIR_EXTENDED_PRESET_OFFSET{NO_FIELD, 17, NO_FIELD};
static constexpr ProtocolValue WATER_FLAGS_OFFSET{NO_FIELD, 8, 11};
static constexpr ProtocolValue WATER_MODE_FLAGS_OFFSET{NO_FIELD, 9, 12};
static constexpr ProtocolValue WATER_HEATER_FLAGS_OFFSET{NO_FIELD, 10, NO_FIELD};
static constexpr ProtocolValue WATER_DHW_TARGET_OFFSET{NO_FIELD, 11, 14};
static constexpr ProtocolValue WATER_ZONE1_TARGET_OFFSET{NO_FIELD, 12, 15};
static constexpr ProtocolValue WATER_ZONE2_TARGET_OFFSET{NO_FIELD, NO_FIELD, 16};
static constexpr ProtocolValue WATER_DHW_CURRENT_OFFSET{NO_FIELD, 14, NO_FIELD};
static constexpr ProtocolValue WATER_ZONE1_CURRENT_OFFSET{NO_FIELD, 16, NO_FIELD};
static constexpr ProtocolValue WATER_UNKNOWN_TEMP_OFFSET{NO_FIELD, 15, NO_FIELD};

static constexpr std::array<ProtocolValue, 3> WATER_REPEATED_TARGET_OFFSETS{{
    {NO_FIELD, NO_FIELD, 17}, {NO_FIELD, NO_FIELD, 18}, {NO_FIELD, NO_FIELD, 19}}};

static bool update_temperature(float &destination, float value) {
  if ((std::isnan(destination) && std::isnan(value)) || destination == value)
    return false;
  destination = value;
  return true;
}

static climate::ClimateMode air_mode(uint8_t mode) {
  switch (mode) {
    case 1: return climate::CLIMATE_MODE_HEAT;
    case 2: return climate::CLIMATE_MODE_COOL;
    case 3: return climate::CLIMATE_MODE_FAN_ONLY;
    case 4: return climate::CLIMATE_MODE_DRY;
    case 5: return climate::CLIMATE_MODE_HEAT_COOL;
    default: return climate::CLIMATE_MODE_OFF;
  }
}

static const char *air_mode_name(uint8_t mode) {
  switch (mode) {
    case 1: return "heat";
    case 2: return "cool";
    case 3: return "fan";
    case 4: return "dry";
    case 5: return "auto";
    default: return "unknown";
  }
}
static const char *air_fan_name(uint8_t fan) {
  switch (fan) {
    case 2: return "auto";
    case 5: return "low";
    case 4: return "medium";
    case 3: return "high";
    default: return "unknown";
  }
}

static const char *air_preset_name(uint8_t preset) {
  switch (preset) {
    case 0x00: return "none";
    case 0x01: return "boost";
    case 0x03: return "eco";
    case 0x10: return "sleep";
    default: return "unknown";
  }
}

static const char *air_louvre_name(uint8_t position) {
  switch (position) {
    case 1: return "swing";
    case 2: return "top";
    case 3: return "middle";
    case 4: return "bottom";
    default: return "unknown";
  }
}

constexpr ProtocolValue ToshibaAbClimate::MASTER_KEEPALIVE_OPCODE;
constexpr ProtocolValue ToshibaAbClimate::MASTER_KEEPALIVE_LENGTH;
constexpr ProtocolValue ToshibaAbClimate::MASTER_KEEPALIVE_DATA_TYPE;
constexpr ProtocolValue ToshibaAbClimate::REMOTE_PING_OPCODE;
constexpr ProtocolValue ToshibaAbClimate::REMOTE_PING_LENGTH;
constexpr ProtocolValue ToshibaAbClimate::REMOTE_PING_DATA_TYPE;
constexpr ProtocolValue ToshibaAbClimate::MASTER_STATUS_OPCODE;
constexpr ProtocolValue ToshibaAbClimate::MASTER_STATUS_MIN_LENGTH;
constexpr ProtocolValue ToshibaAbClimate::MASTER_STATUS_DATA_TYPE;
constexpr ProtocolValue ToshibaAbClimate::MASTER_EXTENDED_STATUS_OPCODE;
constexpr ProtocolValue ToshibaAbClimate::MASTER_EXTENDED_STATUS_MIN_LENGTH;
constexpr ProtocolValue ToshibaAbClimate::MASTER_EXTENDED_STATUS_DATA_TYPE;
constexpr ProtocolValue ToshibaAbClimate::MASTER_STATUS_BROADCAST_ADDRESS;
constexpr ProtocolValue ToshibaAbClimate::WATER_MASTER_STATUS_OPCODE;
constexpr ProtocolValue ToshibaAbClimate::WATER_MASTER_STATUS_MIN_LENGTH;
constexpr ProtocolValue ToshibaAbClimate::WATER_MASTER_STATUS_DATA_TYPE;
constexpr ProtocolValue ToshibaAbClimate::WATER_MASTER_EXTENDED_STATUS_OPCODE;
constexpr ProtocolValue ToshibaAbClimate::WATER_MASTER_EXTENDED_STATUS_MIN_LENGTH;
constexpr ProtocolValue ToshibaAbClimate::WATER_MASTER_EXTENDED_STATUS_DATA_TYPE;
constexpr ProtocolValue ToshibaAbClimate::WATER_MASTER_STATUS_MARKER;

ToshibaAbThermostat::ToshibaAbThermostat(ToshibaAbClimate *parent, WaterCircuit circuit)
    : parent_(parent), circuit_(circuit) {
  parent_->register_thermostat(this, circuit);
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
    traits.set_supported_modes({climate::CLIMATE_MODE_OFF, climate::CLIMATE_MODE_HEAT, climate::CLIMATE_MODE_COOL,
                                climate::CLIMATE_MODE_AUTO});
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
  if (esp_address_auto_)
    esp_address_ = AUTO_ADDRESS;
  esp_address_unavailable_reported_ = false;
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
  // Longer TU2C water status shares the short status signature, so classify
  // extended status first. Status frames never establish master identity.
  const bool extended_status = crc_ok && !master_keepalive && !remote_ping &&
                               is_master_status_(protocol, data, size, true, source);
  const bool status = crc_ok && !master_keepalive && !remote_ping && !extended_status &&
                      is_master_status_(protocol, data, size, false, source);
  std::string description;
  if (!crc_ok) {
    description = "CRC failed";
  } else if (master_keepalive) {
    description = "master keepalive " + hex_(&source, 1);
  } else if (remote_ping) {
    description = "remote ping " + hex_(&source, 1);
  } else if (extended_status) {
    description = "master extended status " + hex_(&source, 1);
  } else if (status) {
    description = "master status " + hex_(&source, 1);
  }
  ESP_LOGD(TAG, "RX %s: %s [%s%s%s]", protocol_name_(protocol), colored_hex_(protocol, data, size, crc_ok).c_str(),
           ESPHOME_LOG_COLOR(ESPHOME_LOG_COLOR_GREEN), description.c_str(), ESPHOME_LOG_RESET_COLOR);
  if (!crc_ok)
    return;
  if (master_keepalive)
    consider_keepalive_(protocol, source);
  else if (remote_ping)
    observe_remote_(source, millis());
  else if (status || extended_status)
    process_master_status_(protocol, data, size, extended_status);
}

void ToshibaAbClimate::process_master_status_(Protocol protocol, const uint8_t *data, size_t size, bool extended) {
  DecodedStatus decoded;
  decoded.water = system_type_ == SystemType::WATER;
  decoded.extended = extended;
  // Exclude CRC bytes and TU2C's trailing A0 from every payload access.
  const size_t payload_end = size - (protocol == Protocol::TCC ? 1 : 2);
  const auto present = [protocol, payload_end](const ProtocolValue &offset) {
    return offset.for_protocol(protocol) < payload_end;
  };
  const auto read = [protocol, data](const ProtocolValue &offset) { return data[offset.for_protocol(protocol)]; };
  const auto temperature = [&](const ProtocolValue &offset, float conversion_offset) {
    return present(offset) ? read(offset) / 2.0f - conversion_offset : NAN;
  };

  if (!decoded.water) {
    if (present(AIR_MODE_POWER_OFFSET) && present(AIR_FAN_VENT_OFFSET)) {
      decoded.has_air_state = true;
      const uint8_t mode_power = read(AIR_MODE_POWER_OFFSET);
      decoded.power = (mode_power & 0x01) != 0;
      decoded.mode = (mode_power & 0xE0) >> 5;
      if (protocol == Protocol::TU2C && decoded.mode == 6)
        decoded.mode = 5;  // TU2C auto -> common air auto.
      decoded.fan = (read(AIR_FAN_VENT_OFFSET) & 0xE0) >> 5;
      decoded.ventilation = (read(AIR_FAN_VENT_OFFSET) & 0x04) != 0;
      if (protocol == Protocol::TCC) {
        decoded.has_louvre = true;
        decoded.louvre = (mode_power >> 2) & 0x07;
      }
    }
    decoded.target[0] = temperature(AIR_TARGET_OFFSET, 35.0f);
    if ((extended || protocol == Protocol::TU2C) && present(AIR_CURRENT_OFFSET) && read(AIR_CURRENT_OFFSET) > 1)
      decoded.current[0] = temperature(AIR_CURRENT_OFFSET, 35.0f);
    if ((extended || protocol == Protocol::TU2C) && present(AIR_FLAGS_OFFSET)) {
      decoded.has_air_flags = true;
      decoded.flags = read(AIR_FLAGS_OFFSET);
      decoded.preheating = (decoded.flags & 0x02) != 0;
      decoded.filter_alert = (decoded.flags & 0x80) != 0;
    }
    const auto &preset_offset = extended ? AIR_EXTENDED_PRESET_OFFSET : AIR_PRESET_OFFSET;
    if (present(preset_offset)) {
      decoded.has_preset = true;
      decoded.preset = read(preset_offset);
    }
  } else {
    if (present(WATER_FLAGS_OFFSET) && present(WATER_MODE_FLAGS_OFFSET)) {
      decoded.has_water_flags = true;
      decoded.flags = read(WATER_FLAGS_OFFSET);
      decoded.mode_flags = read(WATER_MODE_FLAGS_OFFSET);
      decoded.zone1_enabled = (decoded.flags & 0x01) != 0;
      decoded.dhw_enabled = (decoded.flags & 0x02) != 0;
      decoded.cooling = (decoded.flags & 0x20) != 0;
      decoded.heating = (decoded.flags & 0x40) != 0;
      decoded.automatic = (decoded.mode_flags & 0x04) != 0;
      decoded.boost = (decoded.mode_flags & 0x40) != 0;
      if (protocol == Protocol::TU2C) {
        decoded.has_antibacteria = true;
        decoded.antibacteria = (decoded.mode_flags & 0x80) != 0;
      }
    }
    if (present(WATER_HEATER_FLAGS_OFFSET)) {
      decoded.has_heater_flags = true;
      decoded.heater_flags = read(WATER_HEATER_FLAGS_OFFSET);
      decoded.resistor = (decoded.heater_flags & 0x04) != 0;
      decoded.heat_pump = (decoded.heater_flags & 0x08) != 0;
    }
    decoded.target[0] = temperature(WATER_DHW_TARGET_OFFSET, 16.0f);
    decoded.target[1] = temperature(WATER_ZONE1_TARGET_OFFSET, 16.0f);
    decoded.target[2] = temperature(WATER_ZONE2_TARGET_OFFSET, 16.0f);
    if (protocol == Protocol::TU2C && extended) {
      // Retain main's confirmed tank->DHW and water-outlet->Zone 1 mapping.
      decoded.current[0] = temperature(WATER_DHW_CURRENT_OFFSET, 16.0f);
      decoded.current[1] = temperature(WATER_ZONE1_CURRENT_OFFSET, 16.0f);
      decoded.unknown_temperature = temperature(WATER_UNKNOWN_TEMP_OFFSET, 16.0f);
    }
    if (protocol == Protocol::A0 && extended) {
      for (size_t i = 0; i < 3; i++)
        decoded.repeated_target[i] = temperature(WATER_REPEATED_TARGET_OFFSETS[i], 16.0f);
    }
  }
  decoded_status_ = decoded;
  log_decoded_status_(protocol, decoded);
  publish_decoded_status_(decoded);
}

void ToshibaAbClimate::log_decoded_status_(Protocol protocol, const DecodedStatus &status) {
  std::string fields;
  const auto setting = [&fields](const char *name, const char *value) {
    fields += std::string(name) + "=" + value + " ";
  };
  const auto flag = [&setting](const char *name, bool value) {
    setting(name, value ? "on" : "off");
  };
  const auto temperature = [&fields](const char *name, float value) {
    if (std::isnan(value))
      return;
    char field[64];
    std::snprintf(field, sizeof(field), "%s=%.1f ", name, value);
    fields += field;
  };
  if (!status.water) {
    if (status.has_air_state) {
      flag("power", status.power);
      setting("mode", air_mode_name(status.mode));
      setting("fan", air_fan_name(status.fan));
      flag("ventilation", status.ventilation);
    }
    if (status.has_air_flags) {
      flag("preheating", status.preheating);
      flag("filter_alert", status.filter_alert);
    }
    if (status.has_louvre)
      setting("louvre_position", air_louvre_name(status.louvre));
    if (status.has_preset)
      setting("preset", air_preset_name(status.preset));
    temperature("target", status.target[0]);
    temperature("room", status.current[0]);
  } else {
    if (status.has_water_flags) {
      flag("dhw_enabled", status.dhw_enabled);
      flag("zone1_enabled", status.zone1_enabled);
      flag("heating", status.heating);
      flag("cooling", status.cooling);
      flag("auto", status.automatic);
      flag("dhw_boost", status.boost);
    }
    if (status.has_antibacteria)
      flag("antibacteria", status.antibacteria);
    if (status.has_heater_flags) {
      flag("dhw_heat_pump", status.heat_pump);
      flag("dhw_resistor", status.resistor);
    }
    temperature("dhw_target", status.target[0]);
    temperature("zone1_target", status.target[1]);
    temperature("zone2_target", status.target[2]);
    temperature("dhw_current", status.current[0]);
    temperature("zone1_current", status.current[1]);
    temperature("unknown_temperature", status.unknown_temperature);
    temperature("repeated_dhw_target", status.repeated_target[0]);
    temperature("repeated_zone1_target", status.repeated_target[1]);
    temperature("repeated_zone2_target", status.repeated_target[2]);
  }
  if (!fields.empty())
    fields.pop_back();
  ESP_LOGD(TAG, "Decoded %s %s: %s", protocol_name_(protocol), status.extended ? "extended status" : "status",
           fields.c_str());
}

void ToshibaAbClimate::publish_decoded_status_(const DecodedStatus &status) {
  if (!status.water) {
    bool changed = false;
    if (status.has_air_state) {
      const auto new_mode = status.power ? air_mode(status.mode) : climate::CLIMATE_MODE_OFF;
      if (this->mode != new_mode) { this->mode = new_mode; changed = true; }
      climate::ClimateAction new_action = climate::CLIMATE_ACTION_OFF;
      if (status.power) {
        switch (status.mode) {
          case 1: new_action = climate::CLIMATE_ACTION_HEATING; break;
          case 2: new_action = climate::CLIMATE_ACTION_COOLING; break;
          case 3: new_action = climate::CLIMATE_ACTION_FAN; break;
          case 4: new_action = climate::CLIMATE_ACTION_DRYING; break;
          default: new_action = climate::CLIMATE_ACTION_IDLE; break;
        }
      }
      if (this->action != new_action) { this->action = new_action; changed = true; }
      climate::ClimateFanMode new_fan = climate::CLIMATE_FAN_AUTO;
      bool valid_fan = true;
      if (status.power) {
        switch (status.fan) {
          case 2: new_fan = climate::CLIMATE_FAN_AUTO; break;
          case 5: new_fan = climate::CLIMATE_FAN_LOW; break;
          case 4: new_fan = climate::CLIMATE_FAN_MEDIUM; break;
          case 3: new_fan = climate::CLIMATE_FAN_HIGH; break;
          default: valid_fan = false; break;
        }
      }
      if (valid_fan && (!this->fan_mode.has_value() || *this->fan_mode != new_fan)) {
        this->fan_mode = new_fan;
        changed = true;
      }
    }
    if (status.has_preset) {
      climate::ClimatePreset new_preset = climate::CLIMATE_PRESET_NONE;
      bool valid_preset = true;
      switch (status.preset) {
        case 0x00: break;
        case 0x01: new_preset = climate::CLIMATE_PRESET_BOOST; break;
        case 0x03: new_preset = climate::CLIMATE_PRESET_ECO; break;
        case 0x10: new_preset = climate::CLIMATE_PRESET_SLEEP; break;
        default: valid_preset = false; break;
      }
      if (valid_preset && (!this->preset.has_value() || *this->preset != new_preset)) {
        this->preset = new_preset;
        changed = true;
      }
    }
    // Keep main's plausibility limits while logging all received values.
    if (status.target[0] >= 16 && status.target[0] <= 29)
      changed |= update_temperature(this->target_temperature, status.target[0]);
    if (status.current[0] >= 5 && status.current[0] <= 35)
      changed |= update_temperature(this->current_temperature, status.current[0]);
    if (changed)
      this->publish_state();
    return;
  }

  for (size_t circuit = 0; circuit < water_thermostats_.size(); circuit++) {
    auto *thermostat = water_thermostats_[circuit];
    if (thermostat == nullptr)
      continue;
    bool changed = false;
    if (status.has_water_flags && circuit < 2) {
      climate::ClimateMode new_mode = climate::CLIMATE_MODE_OFF;
      if (circuit == 0) {
        if (status.dhw_enabled)
          new_mode = climate::CLIMATE_MODE_HEAT;
      } else if (status.zone1_enabled) {
        new_mode = status.automatic ? climate::CLIMATE_MODE_AUTO
                                   : (status.cooling ? climate::CLIMATE_MODE_COOL : climate::CLIMATE_MODE_HEAT);
      }
      if (thermostat->mode != new_mode) { thermostat->mode = new_mode; changed = true; }
      // Enable/mode flags do not establish active heat transfer. Only TU2C
      // DHW pump/resistor flags do; otherwise an enabled circuit is idle.
      const auto new_action = new_mode == climate::CLIMATE_MODE_OFF ? climate::CLIMATE_ACTION_OFF
                              : circuit == 0 && status.has_heater_flags && (status.heat_pump || status.resistor)
                                  ? climate::CLIMATE_ACTION_HEATING : climate::CLIMATE_ACTION_IDLE;
      if (thermostat->action != new_action) { thermostat->action = new_action; changed = true; }
      if (circuit == 0) {
        const auto new_preset = status.boost ? climate::CLIMATE_PRESET_BOOST : climate::CLIMATE_PRESET_NONE;
        if (!thermostat->preset.has_value() || *thermostat->preset != new_preset) {
          thermostat->preset = new_preset;
          changed = true;
        }
      }
    }
    // Auto has no fixed setpoint, even when a short status omits temperatures.
    if (circuit == 1 && thermostat->mode == climate::CLIMATE_MODE_AUTO)
      changed |= update_temperature(thermostat->target_temperature, NAN);
    else if (!std::isnan(status.target[circuit]))
      changed |= update_temperature(thermostat->target_temperature, status.target[circuit]);
    if (!std::isnan(status.current[circuit]))
      changed |= update_temperature(thermostat->current_temperature, status.current[circuit]);
    if (changed)
      thermostat->publish_state();
  }
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

bool ToshibaAbClimate::is_master_status_(Protocol protocol, const uint8_t *data, size_t size,
                                        bool extended, uint8_t &source) const {
  if (data == nullptr || !protocol_confirmed_ || !master_address_confirmed_ || protocol != protocol_detected_)
    return false;

  const bool water = system_type_ == SystemType::WATER;
  if (water && protocol == Protocol::TCC)
    return false;  // No water TCC signature is established in main.

  const uint8_t length = frame_length_(protocol, data, size);
  uint8_t destination = 0;
  switch (protocol) {
    case Protocol::TCC:
      if (size != static_cast<size_t>(length) + 5)
        return false;
      source = data[0];
      destination = data[1];
      break;
    case Protocol::TU2C:
      if (size < 8 || size != length)
        return false;
      source = data[3];
      destination = data[4];
      if (water && data[5] != WATER_MASTER_STATUS_MARKER.for_protocol(protocol))
        return false;
      if (!water && destination != MASTER_STATUS_BROADCAST_ADDRESS.for_protocol(protocol))
        return false;
      break;
    case Protocol::A0:
      if (size < 11 || size != static_cast<size_t>(length) + 6)
        return false;
      source = data[6];
      destination = data[8];
      break;
    default:
      return false;
  }
  if (source != master_address_)
    return false;

  const auto &opcodes = water ? (extended ? WATER_MASTER_EXTENDED_STATUS_OPCODE : WATER_MASTER_STATUS_OPCODE)
                              : (extended ? MASTER_EXTENDED_STATUS_OPCODE : MASTER_STATUS_OPCODE);
  const auto &lengths = water ? (extended ? WATER_MASTER_EXTENDED_STATUS_MIN_LENGTH : WATER_MASTER_STATUS_MIN_LENGTH)
                              : (extended ? MASTER_EXTENDED_STATUS_MIN_LENGTH : MASTER_STATUS_MIN_LENGTH);
  const auto &types = water ? (extended ? WATER_MASTER_EXTENDED_STATUS_DATA_TYPE : WATER_MASTER_STATUS_DATA_TYPE)
                            : (extended ? MASTER_EXTENDED_STATUS_DATA_TYPE : MASTER_STATUS_DATA_TYPE);
  const uint16_t expected_opcode = opcodes.for_protocol(protocol);
  return length >= lengths.for_protocol(protocol) &&
         (expected_opcode == UNSPECIFIED_OPCODE || opcode_(protocol, data, size) == expected_opcode) &&
         data_type_(protocol, data, size) == types.for_protocol(protocol);
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
  update_esp_address_();
}

void ToshibaAbClimate::observe_remote_(uint8_t address, uint32_t now) {
  for (auto &remote : remotes_) {
    if (remote.address == address) {
      remote.last_seen = now;
      update_esp_address_();
      return;
    }
  }

  remotes_.push_back({address, now});
  std::sort(remotes_.begin(), remotes_.end(),
            [](const RemotePresence &left, const RemotePresence &right) { return left.address < right.address; });
  diagnostic_(std::string("Remote discovered: ") + hex_(&address, 1));
  update_esp_address_();
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
  if (!expired.empty())
    update_esp_address_();
}

void ToshibaAbClimate::update_esp_address_() {
  if (!protocol_confirmed_ || !master_address_confirmed_)
    return;

  const auto address_in_use = [this](uint8_t address) {
    return address == master_address_ ||
           std::any_of(remotes_.begin(), remotes_.end(),
                       [address](const RemotePresence &remote) { return remote.address == address; });
  };

  if (!esp_address_auto_) {
    if (!esp_address_collision_reported_ && address_in_use(esp_address_)) {
      esp_address_collision_reported_ = true;
      diagnostic_(std::string("ESP address collision: ") + hex_(&esp_address_, 1) +
                  " is in use; keeping explicit YAML address");
    }
    return;
  }

  const auto candidates = esp_address_candidates(protocol_detected_, system_type_);
  uint8_t selected = AUTO_ADDRESS;
  for (size_t i = 0; i < candidates.size; i++) {
    if (!address_in_use(candidates.data[i])) {
      selected = candidates.data[i];
      break;
    }
  }

  // Never retain a colliding address when all candidates are occupied.
  const uint8_t previous = esp_address_;
  esp_address_ = selected;
  if (selected == AUTO_ADDRESS) {
    if (!esp_address_unavailable_reported_) {
      esp_address_unavailable_reported_ = true;
      diagnostic_(candidates.size == 0 ? "ESP auto address unavailable: unsupported protocol/system combination"
                                       : "ESP auto address unavailable: all candidate addresses are in use");
    }
    return;
  }

  esp_address_unavailable_reported_ = false;
  if (selected != previous)
    diagnostic_(std::string("ESP auto address selected: ") + hex_(&selected, 1));
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
#if defined(USE_ESP8266) || defined(USE_ESP32)
    parent_->load_settings();
#endif
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
  ESP_LOGCONFIG(TAG, "  ESP address mode: %s", esp_address_auto_ ? "auto" : "explicit");
  ESP_LOGCONFIG(TAG, "  ESP address: %s",
                esp_address_ == AUTO_ADDRESS ? "unassigned" : hex_(&esp_address_, 1).c_str());
}

}  // namespace toshiba_ab
}  // namespace esphome
