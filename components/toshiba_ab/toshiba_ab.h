#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include <array>
#include <string>

namespace esphome {
namespace toshiba_ab {

enum class Protocol : uint8_t { AUTO, TCC, TU2C, A0 };
enum class SystemType : uint8_t { AIR, WATER };

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

  void set_master_address(uint8_t address) { master_setting_ = address; }
  void set_esp_address(uint8_t address) { esp_address_ = address; }
  void set_protocol(Protocol protocol) { protocol_setting_ = protocol; }
  void set_system_type(SystemType type) { system_type_ = type; }
  void set_diagnostic_sensor(text_sensor::TextSensor *sensor) { diagnostic_sensor_ = sensor; }
  void set_hardware_uart_rx_pin(uint8_t pin) { hardware_uart_rx_pin_ = pin; }

 protected:
  static constexpr uint8_t AUTO_ADDRESS = 0xAA;
  static constexpr uint32_t PROTOCOL_SCAN_MS = 20000;
  static constexpr uint32_t DISCOVERY_MS = 3 * PROTOCOL_SCAN_MS;
  static constexpr uint32_t BYTE_TIMEOUT_MS = 25;
  static constexpr size_t MAX_FRAME_SIZE = 132;
  static constexpr ProtocolValue MASTER_KEEPALIVE_OPCODE{0x10, 0x00, 0x10};
  // TCC carries data type 0x8A. In the TU2C 00:3A tail, 0x00 is the opcode
  // and 0x3A is the data type. A0 master heartbeats carry the two-byte data
  // type 00:8A in the corresponding field.
  static constexpr ProtocolValue MASTER_KEEPALIVE_DATA_TYPE{0x8A, 0x3A, 0x008A};

  void read_byte_(uint8_t byte);
  void read_even_byte_(uint8_t byte);
  void read_tu2c_byte_(uint8_t byte);
  void update_discovery_(uint32_t now);
  void select_scan_protocol_(Protocol protocol);
  void reset_readers_(bool timeout = false);
  void check_reader_timeout_(uint32_t now);
  void process_frame_(Protocol protocol, const uint8_t *data, size_t size, bool crc_ok);
  bool is_master_keepalive_(Protocol protocol, const uint8_t *data, size_t size, uint8_t &source) const;
  void consider_keepalive_(Protocol protocol, uint8_t source);
  void set_runtime_parity_(uart::UARTParityOptions parity);
  void diagnostic_(const std::string &message);
  static const char *protocol_name_(Protocol protocol);
  static uint8_t opcode_(Protocol protocol, const uint8_t *data, size_t size);
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
  bool master_address_confirmed_{false};
  bool protocol_confirmed_{false};
  Protocol scan_protocol_{Protocol::AUTO};
  bool discovery_finished_{false};
  uint32_t boot_ms_{0};
  uint32_t last_byte_ms_{0};
  uint32_t reader_reset_count_{0};
  std::string diagnostic_history_;
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
