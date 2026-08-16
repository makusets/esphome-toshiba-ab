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
enum class DiscoveryState : uint8_t { SCAN_TCC, SCAN_A0, SCAN_TU2C, COMPLETE };

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
  void set_diagnostic_sensor(text_sensor::TextSensor *sensor) { diagnostic_ = sensor; }
  void set_hardware_uart_rx_pin(uint8_t pin) { hardware_uart_rx_pin_ = pin; }

 protected:
  static constexpr uint8_t AUTO_ADDRESS = 0xAA;
  static constexpr uint32_t SCAN_PERIOD_MS = 20000;
  static constexpr uint32_t FRAME_TIMEOUT_MS = 30;
  static constexpr size_t MAX_FRAME_SIZE = 132;
  static constexpr size_t MAX_BYTES_PER_LOOP = 256;

  void read_byte_(uint8_t byte);
  void read_even_byte_(uint8_t byte);
  void read_tu2c_byte_(uint8_t byte);
  void check_reader_timeout_(uint32_t now);
  void advance_discovery_();
  void reset_readers_();
  void process_frame_(Protocol protocol, const uint8_t *data, size_t size, bool crc_ok);
  bool is_master_keepalive_(Protocol protocol, const uint8_t *data, size_t size, uint8_t &source) const;
  void consider_keepalive_(Protocol protocol, uint8_t source);
  void set_runtime_parity_(uart::UARTParityOptions parity);
  void diagnostic_(const std::string &message);
  static const char *protocol_name_(Protocol protocol);
  static const char *opcode1_name_(uint8_t opcode);
  static const char *opcode2_name_(uint8_t opcode);
  static void frame_opcodes_(Protocol protocol, const uint8_t *data, size_t size, uint8_t &opcode1, uint8_t &opcode2);
  static std::string hex_(const uint8_t *data, size_t size);
  static uint16_t crc16_mcrf4xx_(const uint8_t *data, size_t size);

  Protocol protocol_setting_{Protocol::AUTO};
  Protocol protocol_detected_{Protocol::AUTO};
  SystemType system_type_{SystemType::AIR};
  uint8_t master_setting_{AUTO_ADDRESS};
  uint8_t master_address_{AUTO_ADDRESS};
  uint8_t esp_address_{AUTO_ADDRESS};
  bool master_address_confirmed_{false};
  bool protocol_confirmed_{false};
  uint32_t boot_ms_{0};
  uint32_t discovery_stage_ms_{0};
  uint32_t last_reader_byte_ms_{0};
  DiscoveryState discovery_state_{DiscoveryState::SCAN_TCC};
  text_sensor::TextSensor *diagnostic_{nullptr};
  std::string diagnostic_history_;
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
