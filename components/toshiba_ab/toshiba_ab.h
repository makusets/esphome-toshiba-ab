#pragma once

#include "esphome/components/button/button.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include <array>
#include <string>
#include <vector>

namespace esphome {
namespace toshiba_ab {

enum class Protocol : uint8_t { AUTO, TCC, TU2C, A0 };
enum class SystemType : uint8_t { AIR, WATER };
enum class WaterCircuit : uint8_t { DHW, ZONE_1, ZONE_2 };

// Ordered from most to least preferred. An empty list means that automatic
// assignment is not supported for this protocol/system combination.
struct EspAddressCandidates {
  const uint8_t *data;
  size_t size;
};

static constexpr std::array<uint8_t, 9> TCC_AIR_ESP_ADDRESSES{
    {0x40, 0x41, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49}};
static constexpr std::array<uint8_t, 9> TU2C_AIR_ESP_ADDRESSES{
    {0x50, 0x51, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59}};
static constexpr std::array<uint8_t, 10> TU2C_WATER_ESP_ADDRESSES{
    {0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69}};
// A0 addresses use a zero mode byte on the wire (for example 00:40).
static constexpr std::array<uint8_t, 10> A0_WATER_ESP_ADDRESSES{
    {0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49}};

inline EspAddressCandidates esp_address_candidates(Protocol protocol, SystemType system_type) {
  if (protocol == Protocol::TCC && system_type == SystemType::AIR)
    return {TCC_AIR_ESP_ADDRESSES.data(), TCC_AIR_ESP_ADDRESSES.size()};
  if (protocol == Protocol::TU2C && system_type == SystemType::AIR)
    return {TU2C_AIR_ESP_ADDRESSES.data(), TU2C_AIR_ESP_ADDRESSES.size()};
  if (protocol == Protocol::TU2C && system_type == SystemType::WATER)
    return {TU2C_WATER_ESP_ADDRESSES.data(), TU2C_WATER_ESP_ADDRESSES.size()};
  if (protocol == Protocol::A0 && system_type == SystemType::WATER)
    return {A0_WATER_ESP_ADDRESSES.data(), A0_WATER_ESP_ADDRESSES.size()};
  return {nullptr, 0};
}

class ToshibaAbClimate;

class ToshibaAbThermostat : public climate::Climate {
 public:
  ToshibaAbThermostat(ToshibaAbClimate *parent, WaterCircuit circuit);
  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;

 protected:
  ToshibaAbClimate *parent_;
  WaterCircuit circuit_;
};

class ResetButton : public button::Button {
 public:
  void set_parent(ToshibaAbClimate *parent) { parent_ = parent; }

 protected:
  void press_action() override;
  ToshibaAbClimate *parent_{nullptr};
};

// A semantic field can have a different wire value in every protocol. Keep
// protocol-specific values together rather than spreading magic numbers
// through the readers.
struct ProtocolValue {
  uint16_t tcc;
  uint16_t tu2c;
  uint16_t a0;

  uint16_t for_protocol(Protocol protocol) const {
    return protocol == Protocol::TCC ? tcc : (protocol == Protocol::TU2C ? tu2c : a0);
  }
};

class DiagnosticTextSensor : public text_sensor::TextSensor {};

class ToshibaAbClimate : public climate::Climate, public uart::UARTDevice, public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }
  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;
  void reset();
  void control_water(WaterCircuit circuit, const climate::ClimateCall &call);

  void set_master_address(uint8_t address) { master_setting_ = address; }
  void set_esp_address(uint8_t address) {
    esp_address_ = address;
    esp_address_auto_ = address == AUTO_ADDRESS;
  }
  void set_protocol(Protocol protocol) { protocol_setting_ = protocol; }
  void set_system_type(SystemType type) { system_type_ = type; }
  void set_diagnostic_sensor(text_sensor::TextSensor *sensor) { diagnostic_sensor_ = sensor; }
  void set_hardware_uart_rx_pin(uint8_t pin) { hardware_uart_rx_pin_ = pin; }

 protected:
  static constexpr uint8_t AUTO_ADDRESS = 0xAA;
  static constexpr uint32_t PROTOCOL_SCAN_MS = 20000;
  static constexpr uint32_t DISCOVERY_MS = 3 * PROTOCOL_SCAN_MS;
  static constexpr uint32_t BYTE_TIMEOUT_MS = 25;
  static constexpr uint32_t REMOTE_EXPIRY_MS = 5 * 60 * 1000;
  static constexpr size_t MAX_FRAME_SIZE = 132;
  static constexpr ProtocolValue MASTER_KEEPALIVE_OPCODE{0x10, 0x00, 0x10};
  // Length is the value carried by each protocol's length byte, rather than
  // the size of the reader's complete frame buffer.
  static constexpr ProtocolValue MASTER_KEEPALIVE_LENGTH{0x02, 0x0A, 0x07};
  // TCC carries data type 0x8A. In the TU2C 00:3A tail, 0x00 is the opcode
  // and 0x3A is the data type. A0 master heartbeats carry the two-byte data
  // type 00:8A in the corresponding field.
  static constexpr ProtocolValue MASTER_KEEPALIVE_DATA_TYPE{0x8A, 0x3A, 0x008A};
  static constexpr ProtocolValue REMOTE_PING_OPCODE{0x15, 0x41, 0x15};
  static constexpr ProtocolValue REMOTE_PING_LENGTH{0x07, 0x0C, 0x0C};
  static constexpr ProtocolValue REMOTE_PING_DATA_TYPE{0x0C, 0x5C, 0x0C81};
  // First-generation Estia shares the TU2C wire format but uses data type
  // 0x0C for its remote ping instead of the air protocol's 0x5C.
  static constexpr uint16_t TU2C_FIRST_GEN_REMOTE_PING_DATA_TYPE = 0x0C;

  // Status lengths are minimum encoded lengths, not complete buffer sizes.
  // TCC and A0 air share the 0x81 status marker (A0 carries 00:81).
  // Air TU2C uses C0:38 / A0:38 and broadcasts to 0xFF.
  static constexpr ProtocolValue MASTER_STATUS_OPCODE{0x1C, 0xC0, 0x1C};
  static constexpr ProtocolValue MASTER_STATUS_MIN_LENGTH{0x07, 0x0F, 0x0C};
  static constexpr ProtocolValue MASTER_STATUS_DATA_TYPE{0x81, 0x38, 0x0081};
  static constexpr ProtocolValue MASTER_EXTENDED_STATUS_OPCODE{0x58, 0xA0, 0x58};
  static constexpr ProtocolValue MASTER_EXTENDED_STATUS_MIN_LENGTH{0x07, 0x0F, 0x0C};
  static constexpr ProtocolValue MASTER_EXTENDED_STATUS_DATA_TYPE{0x81, 0x38, 0x0081};
  static constexpr ProtocolValue MASTER_STATUS_BROADCAST_ADDRESS{0xF0, 0xFF, 0xFE};

  // Main identifies TU2C water status by E0:<reserved opcode>:31; it does
  // not constrain that opcode. 0x100 denotes an unspecified 8-bit opcode.
  static constexpr uint16_t UNSPECIFIED_OPCODE = 0x100;
  static constexpr ProtocolValue WATER_MASTER_STATUS_OPCODE{0, UNSPECIFIED_OPCODE, 0x1C};
  static constexpr ProtocolValue WATER_MASTER_STATUS_MIN_LENGTH{0, 0x0C, 0x0F};
  static constexpr ProtocolValue WATER_MASTER_STATUS_DATA_TYPE{0, 0x31, 0x03C6};
  static constexpr ProtocolValue WATER_MASTER_EXTENDED_STATUS_OPCODE{0, UNSPECIFIED_OPCODE, 0x58};
  static constexpr ProtocolValue WATER_MASTER_EXTENDED_STATUS_MIN_LENGTH{0, 0x15, 0x0F};
  static constexpr ProtocolValue WATER_MASTER_EXTENDED_STATUS_DATA_TYPE{0, 0x31, 0x03C6};
  static constexpr ProtocolValue WATER_MASTER_STATUS_MARKER{0, 0xE0, 0};

  void read_byte_(uint8_t byte);
  void read_even_byte_(uint8_t byte);
  void read_tu2c_byte_(uint8_t byte);
  void update_discovery_(uint32_t now);
  void select_scan_protocol_(Protocol protocol);
  void reset_readers_(bool timeout = false);
  void check_reader_timeout_(uint32_t now);
  void process_frame_(Protocol protocol, const uint8_t *data, size_t size, bool crc_ok);
  bool is_master_keepalive_(Protocol protocol, const uint8_t *data, size_t size, uint8_t &source) const;
  bool is_remote_ping_(Protocol protocol, const uint8_t *data, size_t size, uint8_t &source) const;
  bool is_master_status_(Protocol protocol, const uint8_t *data, size_t size, bool extended,
                         uint8_t &source) const;
  void consider_keepalive_(Protocol protocol, uint8_t source);
  void observe_remote_(uint8_t address, uint32_t now);
  void expire_remotes_(uint32_t now);
  void update_esp_address_();
  void set_runtime_parity_(uart::UARTParityOptions parity);
  void diagnostic_(const std::string &message);
  static const char *protocol_name_(Protocol protocol);
  static uint8_t opcode_(Protocol protocol, const uint8_t *data, size_t size);
  static uint8_t frame_length_(Protocol protocol, const uint8_t *data, size_t size);
  static uint16_t data_type_(Protocol protocol, const uint8_t *data, size_t size);
  static std::string hex_(const uint8_t *data, size_t size);
  static std::string colored_hex_(Protocol protocol, const uint8_t *data, size_t size, bool crc_ok);
  static uint16_t crc16_mcrf4xx_(const uint8_t *data, size_t size);

  Protocol protocol_setting_{Protocol::AUTO};
  Protocol protocol_detected_{Protocol::AUTO};
  SystemType system_type_{SystemType::AIR};
  uint8_t master_setting_{AUTO_ADDRESS};
  uint8_t master_address_{AUTO_ADDRESS};
  uint8_t esp_address_{AUTO_ADDRESS};
  bool esp_address_auto_{true};
  bool esp_address_collision_reported_{false};
  bool esp_address_unavailable_reported_{false};
  bool master_address_confirmed_{false};
  bool protocol_confirmed_{false};
  Protocol scan_protocol_{Protocol::AUTO};
  bool discovery_finished_{false};
  uint32_t boot_ms_{0};
  uint32_t last_byte_ms_{0};
  uint32_t reader_reset_count_{0};
  struct RemotePresence {
    uint8_t address;
    uint32_t last_seen;
  };
  std::vector<RemotePresence> remotes_;
  text_sensor::TextSensor *diagnostic_sensor_{nullptr};
  uint8_t hardware_uart_rx_pin_{0xFF};

  std::array<uint8_t, MAX_FRAME_SIZE> tcc_{};
  size_t tcc_size_{0};
  size_t tcc_expected_{0};
  std::array<uint8_t, MAX_FRAME_SIZE> a0_{};
  size_t a0_size_{0};
  size_t a0_expected_{0};
  uint8_t a0_sync_{0};
  std::array<uint8_t, MAX_FRAME_SIZE> tu2c_{};
  size_t tu2c_size_{0};
  size_t tu2c_expected_{0};
  uint8_t tu2c_sync_{0};
};

}  // namespace toshiba_ab
}  // namespace esphome
