#!/usr/bin/env python3
"""Compile the real component against small host API stubs, then run bus fixtures."""
import pathlib
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[1]
STUB = r'''
#pragma once
#include <cstdint>
#include <initializer_list>
#include <string>
#include <vector>
extern uint32_t test_now;
namespace esphome {
inline uint32_t millis() { return test_now; }
namespace setup_priority { constexpr float DATA = 0; }
class Component {
 public:
  virtual void setup() {}
  virtual void loop() {}
  virtual void dump_config() {}
  virtual float get_setup_priority() const { return 0; }
};
namespace climate {
enum Value { CLIMATE_MODE_OFF, CLIMATE_MODE_HEAT, CLIMATE_MODE_COOL,
  CLIMATE_MODE_HEAT_COOL, CLIMATE_MODE_FAN_ONLY, CLIMATE_MODE_DRY, CLIMATE_MODE_AUTO,
  CLIMATE_PRESET_NONE, CLIMATE_PRESET_BOOST, CLIMATE_PRESET_ECO, CLIMATE_PRESET_SLEEP,
  CLIMATE_FAN_AUTO, CLIMATE_FAN_LOW, CLIMATE_FAN_MEDIUM, CLIMATE_FAN_HIGH,
  CLIMATE_SWING_OFF, CLIMATE_SWING_BOTH, CLIMATE_SWING_VERTICAL, CLIMATE_SWING_HORIZONTAL };
constexpr int CLIMATE_SUPPORTS_CURRENT_TEMPERATURE = 1;
constexpr int CLIMATE_SUPPORTS_ACTION = 2;
class ClimateCall {};
class ClimateTraits {
 public:
  void set_feature_flags(int) {}
  void set_supported_modes(std::initializer_list<Value>) {}
  void set_supported_presets(std::initializer_list<Value>) {}
  void set_supported_fan_modes(std::initializer_list<Value>) {}
  void set_supported_swing_modes(std::initializer_list<Value>) {}
  void set_visual_min_temperature(float) {}
  void set_visual_max_temperature(float) {}
  void set_visual_temperature_step(float) {}
};
class Climate {
 public:
  Value mode{CLIMATE_MODE_OFF};
  float target_temperature{};
  virtual ClimateTraits traits() { return {}; }
  virtual void control(const ClimateCall &) {}
};
}
namespace uart {
enum UARTParityOptions { UART_CONFIG_PARITY_NONE, UART_CONFIG_PARITY_EVEN };
class UARTComponent {
 public:
  void set_parity(UARTParityOptions) {}
  void load_settings() {}
};
class UARTDevice {
 protected:
  UARTComponent *parent_{nullptr};
 public:
  bool available() { return false; }
  bool read_byte(uint8_t *) { return false; }
};
}
namespace button { class Button { public: virtual void press_action() {} }; }
namespace text_sensor {
class TextSensor {
 public:
  std::vector<std::string> events;
  void publish_state(const std::string &value) { events.push_back(value); }
};
}
}
#define ESP_LOGVV(...) ((void) 0)
#define ESP_LOGV(...) ((void) 0)
#define ESP_LOGD(...) ((void) 0)
#define ESP_LOGI(...) ((void) 0)
#define ESP_LOGCONFIG(...) ((void) 0)
#define ESPHOME_LOG_COLOR(x) ""
#define ESPHOME_LOG_RESET_COLOR ""
'''

def run_test(source, stub=STUB):
    with tempfile.TemporaryDirectory(prefix="toshiba-address-tests-") as directory:
        temp = pathlib.Path(directory)
        for header in ("components/button/button.h", "components/climate/climate.h",
                       "components/text_sensor/text_sensor.h", "components/uart/uart.h",
                       "core/component.h", "core/log.h"):
            path = temp / "esphome" / header
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('#include "host_stubs.h"\n')
        (temp / "host_stubs.h").write_text(stub)
        executable = temp / "address-tests"
        subprocess.run([
            "g++", "-std=c++11", "-Wall", "-Wextra", "-Wno-unused-parameter",
            "-Wno-unused-variable", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
            "-I", str(temp), "-I", str(ROOT / "components/toshiba_ab"),
            str(ROOT / "tests" / source),
            str(ROOT / "components/toshiba_ab/toshiba_ab.cpp"), "-o", str(executable),
        ], check=True)
        subprocess.run([str(executable)], check=True)

def main():
    run_test("esp_address_test.cpp")

if __name__ == "__main__":
    main()
