// Auto generated code by esphome
// ========== AUTO GENERATED INCLUDE BLOCK BEGIN ===========
#include "esphome.h"
using namespace esphome;
alignas(logger::Logger) static unsigned char logger__logger_logger_id__pstorage[sizeof(logger::Logger)];
static logger::Logger *const logger_logger_id = reinterpret_cast<logger::Logger *>(logger__logger_logger_id__pstorage);
#ifndef __PICOLIBC__
using std::isnan;
#endif
using std::min;
using std::max;
#include <new>
using namespace climate;
using namespace binary_sensor;
using namespace sensor;
using namespace select;
using namespace switch_;
using namespace text_sensor;
alignas(mdns::MDNSComponent) static unsigned char mdns__mdns_mdnscomponent_id__pstorage[sizeof(mdns::MDNSComponent)];
static mdns::MDNSComponent *const mdns_mdnscomponent_id = reinterpret_cast<mdns::MDNSComponent *>(mdns__mdns_mdnscomponent_id__pstorage);
alignas(preferences::IntervalSyncer) static unsigned char preferences__preferences_intervalsyncer_id__pstorage[sizeof(preferences::IntervalSyncer)];
static preferences::IntervalSyncer *const preferences_intervalsyncer_id = reinterpret_cast<preferences::IntervalSyncer *>(preferences__preferences_intervalsyncer_id__pstorage);
using namespace uart;
alignas(uart::HostUartComponent) static unsigned char uart__uart_hostuartcomponent_id__pstorage[sizeof(uart::HostUartComponent)];
static uart::HostUartComponent *const uart_hostuartcomponent_id = reinterpret_cast<uart::HostUartComponent *>(uart__uart_hostuartcomponent_id__pstorage);
alignas(toshiba_ab::ToshibaAbClimate) static unsigned char toshiba_ab__toshiba_ab_toshibaabclimate_id__pstorage[sizeof(toshiba_ab::ToshibaAbClimate)];
static toshiba_ab::ToshibaAbClimate *const toshiba_ab_toshibaabclimate_id = reinterpret_cast<toshiba_ab::ToshibaAbClimate *>(toshiba_ab__toshiba_ab_toshibaabclimate_id__pstorage);
alignas(toshiba_ab::ToshibaAbEstiaZone1Climate) static unsigned char toshiba_ab__toshiba_ab_toshibaabestiazone1climate_id__pstorage[sizeof(toshiba_ab::ToshibaAbEstiaZone1Climate)];
static toshiba_ab::ToshibaAbEstiaZone1Climate *const toshiba_ab_toshibaabestiazone1climate_id = reinterpret_cast<toshiba_ab::ToshibaAbEstiaZone1Climate *>(toshiba_ab__toshiba_ab_toshibaabestiazone1climate_id__pstorage);
alignas(toshiba_ab::ToshibaAbEstiaZone1Climate) static unsigned char toshiba_ab__toshiba_ab_toshibaabestiazone1climate_id_2__pstorage[sizeof(toshiba_ab::ToshibaAbEstiaZone1Climate)];
static toshiba_ab::ToshibaAbEstiaZone1Climate *const toshiba_ab_toshibaabestiazone1climate_id_2 = reinterpret_cast<toshiba_ab::ToshibaAbEstiaZone1Climate *>(toshiba_ab__toshiba_ab_toshibaabestiazone1climate_id_2__pstorage);
alignas(toshiba_ab::ToshibaAbRemoteAddressSelect) static unsigned char toshiba_ab__toshiba_ab_toshibaabremoteaddressselect_id__pstorage[sizeof(toshiba_ab::ToshibaAbRemoteAddressSelect)];
static toshiba_ab::ToshibaAbRemoteAddressSelect *const toshiba_ab_toshibaabremoteaddressselect_id = reinterpret_cast<toshiba_ab::ToshibaAbRemoteAddressSelect *>(toshiba_ab__toshiba_ab_toshibaabremoteaddressselect_id__pstorage);
alignas(sensor::Sensor) static unsigned char sensor__sensor_sensor_id__pstorage[sizeof(sensor::Sensor)];
static sensor::Sensor *const sensor_sensor_id = reinterpret_cast<sensor::Sensor *>(sensor__sensor_sensor_id__pstorage);
alignas(sensor::Sensor) static unsigned char sensor__sensor_sensor_id_2__pstorage[sizeof(sensor::Sensor)];
static sensor::Sensor *const sensor_sensor_id_2 = reinterpret_cast<sensor::Sensor *>(sensor__sensor_sensor_id_2__pstorage);
alignas(text_sensor::TextSensor) static unsigned char text_sensor__text_sensor_textsensor_id__pstorage[sizeof(text_sensor::TextSensor)];
static text_sensor::TextSensor *const text_sensor_textsensor_id = reinterpret_cast<text_sensor::TextSensor *>(text_sensor__text_sensor_textsensor_id__pstorage);
alignas(text_sensor::TextSensor) static unsigned char text_sensor__text_sensor_textsensor_id_2__pstorage[sizeof(text_sensor::TextSensor)];
static text_sensor::TextSensor *const text_sensor_textsensor_id_2 = reinterpret_cast<text_sensor::TextSensor *>(text_sensor__text_sensor_textsensor_id_2__pstorage);
alignas(text_sensor::TextSensor) static unsigned char text_sensor__text_sensor_textsensor_id_3__pstorage[sizeof(text_sensor::TextSensor)];
static text_sensor::TextSensor *const text_sensor_textsensor_id_3 = reinterpret_cast<text_sensor::TextSensor *>(text_sensor__text_sensor_textsensor_id_3__pstorage);
alignas(text_sensor::TextSensor) static unsigned char text_sensor__text_sensor_textsensor_id_4__pstorage[sizeof(text_sensor::TextSensor)];
static text_sensor::TextSensor *const text_sensor_textsensor_id_4 = reinterpret_cast<text_sensor::TextSensor *>(text_sensor__text_sensor_textsensor_id_4__pstorage);
alignas(toshiba_ab::ToshibaAbEstiaNightSetbackSwitch) static unsigned char toshiba_ab__toshiba_ab_toshibaabestianightsetbackswitch_id__pstorage[sizeof(toshiba_ab::ToshibaAbEstiaNightSetbackSwitch)];
static toshiba_ab::ToshibaAbEstiaNightSetbackSwitch *const toshiba_ab_toshibaabestianightsetbackswitch_id = reinterpret_cast<toshiba_ab::ToshibaAbEstiaNightSetbackSwitch *>(toshiba_ab__toshiba_ab_toshibaabestianightsetbackswitch_id__pstorage);
static constexpr size_t ESPHOME_LOOPING_COMPONENT_COUNT = \
  (1 * HasLoopOverride<logger::Logger>::value) + \
  (1 * HasLoopOverride<mdns::MDNSComponent>::value) + \
  (1 * HasLoopOverride<preferences::IntervalSyncer>::value) + \
  (1 * HasLoopOverride<uart::HostUartComponent>::value) + \
  (1 * HasLoopOverride<toshiba_ab::ToshibaAbClimate>::value);
namespace esphome {
static const char COMP_SRC_TABLE_STR_0[] PROGMEM = "logger";
static const char COMP_SRC_TABLE_STR_1[] PROGMEM = "mdns";
static const char COMP_SRC_TABLE_STR_2[] PROGMEM = "preferences";
static const char COMP_SRC_TABLE_STR_3[] PROGMEM = "uart";
static const char COMP_SRC_TABLE_STR_4[] PROGMEM = "toshiba_ab.climate";
static const char *const COMP_SRC_TABLE[] PROGMEM = {COMP_SRC_TABLE_STR_0, COMP_SRC_TABLE_STR_1, COMP_SRC_TABLE_STR_2, COMP_SRC_TABLE_STR_3, COMP_SRC_TABLE_STR_4};
const LogString *component_source_lookup(uint8_t index) {
  if (index == 0 || index > 5) return LOG_STR("<unknown>");
  return reinterpret_cast<const LogString *>(
    progmem_read_ptr(&COMP_SRC_TABLE[index - 1]));
}
}  // namespace esphome
namespace esphome {
static const char *const ENTITY_UOM_TABLE[] PROGMEM = {"errors/min", "failures/5min"};
const char *entity_uom_lookup(uint8_t index) {
  if (index == 0 || index > 2) return "";
  return progmem_read_ptr(&ENTITY_UOM_TABLE[index - 1]);
}

}  // namespace esphome
// ========== AUTO GENERATED INCLUDE BLOCK END ==========="

void setup() {
  // ========== AUTO GENERATED CODE BEGIN ===========
  // logger:
  //   baud_rate: 0
  //   deassert_rts_dtr: false
  //   id: logger_logger_id
  //   level: DEBUG
  //   logs: {}
  //   runtime_tag_levels: false
  //   tx_buffer_size: 512
  new(logger_logger_id) logger::Logger(0);
  logger_logger_id->create_pthread_key();
  logger_logger_id->pre_setup();
  logger_logger_id->set_log_level(ESPHOME_LOG_LEVEL_DEBUG);
  // network:
  //   enable_ipv6: false
  //   id: network_networkcomponent_id
  //   min_ipv6_addr_count: 0
  // esphome:
  //   areas: []
  //   build_flags: []
  //   build_path: ../../work/estia-host
  //   debug_scheduler: false
  //   devices: []
  //   environment_variables: {}
  //   friendly_name: ''
  //   includes: []
  //   includes_c: []
  //   libraries: []
  //   merge_warnings: true
  //   min_version: 2026.9.1
  //   name: estia-tests
  //   name_add_mac_suffix: false
  //   platformio_options: {}
  new (&App) Application();
  App.pre_setup("estia-tests", 11, "", 0);
  App.looping_components_.init(ESPHOME_LOOPING_COMPONENT_COUNT);
  // climate:
  // binary_sensor:
  // sensor:
  // select:
  // switch:
  // text_sensor:
  App.register_component_(logger_logger_id, 1);
  // mdns:
  //   disabled: false
  //   id: mdns_mdnscomponent_id
  //   services: []
  new(mdns_mdnscomponent_id) mdns::MDNSComponent();
  App.register_component_(mdns_mdnscomponent_id, 2);
  // preferences:
  //   flash_write_interval: 60s
  //   id: preferences_intervalsyncer_id
  new(preferences_intervalsyncer_id) preferences::IntervalSyncer();
  preferences_intervalsyncer_id->set_update_interval(60000);
  App.register_component_(preferences_intervalsyncer_id, 3);
  // host:
  //   mac_address: 98:35:69:AB:F6:79
  // external_components:
  //   - components: all
  //     refresh: 1d
  //     source:
  //       path: /tmp/opencode/etab/tests/../components
  //       type: local
  // uart:
  //   baud_rate: 2400
  //   data_bits: 8
  //   id: uart_hostuartcomponent_id
  //   parity: EVEN
  //   port: /dev/null
  //   rx_buffer_size: 256
  //   stop_bits: 1
  new(uart_hostuartcomponent_id) uart::HostUartComponent();
  App.register_component_(uart_hostuartcomponent_id, 4);
  uart_hostuartcomponent_id->set_baud_rate(2400);
  uart_hostuartcomponent_id->set_name("/dev/null");
  uart_hostuartcomponent_id->set_rx_buffer_size(256);
  uart_hostuartcomponent_id->set_stop_bits(1);
  uart_hostuartcomponent_id->set_data_bits(8);
  uart_hostuartcomponent_id->set_parity(uart::UART_CONFIG_PARITY_EVEN);
  // climate.toshiba_ab:
  //   autonomous: true
  //   autoreset_errors: false
  //   command_mode_read: 8
  //   command_mode_write: 128
  //   crc_failures_5min:
  //     accuracy_decimals: 0
  //     disabled_by_default: false
  //     entity_category: diagnostic
  //     force_update: false
  //     id: sensor_sensor_id_2
  //     name: Toshiba CRC Failures (5 min)
  //     state_class: measurement
  //     unit_of_measurement: failures/5min
  //   demand_enabled: false
  //   disabled_by_default: false
  //   filter_frames: true
  //   frame_format: a0
  //   id: toshiba_ab_toshibaabclimate_id
  //   indoor_unit_count:
  //     disabled_by_default: false
  //     entity_category: diagnostic
  //     id: text_sensor_textsensor_id
  //     name: Toshiba Indoor Unit Count
  //   indoor_units:
  //     disabled_by_default: false
  //     entity_category: diagnostic
  //     id: text_sensor_textsensor_id_2
  //     name: Toshiba Indoor Units
  //   name: Estia DHW
  //   night_setback:
  //     disabled_by_default: false
  //     id: toshiba_ab_toshibaabestianightsetbackswitch_id
  //     name: Night Setback
  //     restore_mode: ALWAYS_OFF
  //   noise_rate:
  //     accuracy_decimals: 2
  //     disabled_by_default: false
  //     entity_category: diagnostic
  //     force_update: false
  //     id: sensor_sensor_id
  //     name: Toshiba Noise Rate
  //     state_class: measurement
  //     unit_of_measurement: errors/min
  //   packet_min_wait: 200ms
  //   ping: true
  //   platform: toshiba_ab
  //   read_only: false
  //   remote_address_select:
  //     disabled_by_default: false
  //     entity_category: config
  //     id: toshiba_ab_toshibaabremoteaddressselect_id
  //     name: Toshiba Remote Address
  //   remote_addresses:
  //     disabled_by_default: false
  //     entity_category: diagnostic
  //     id: text_sensor_textsensor_id_4
  //     name: Toshiba Remote Addresses
  //   remote_count:
  //     disabled_by_default: false
  //     entity_category: diagnostic
  //     id: text_sensor_textsensor_id_3
  //     name: Toshiba Remote Count
  //   sensors: []
  //   uart_id: uart_hostuartcomponent_id
  //   visual: {}
  //   zone1_climate:
  //     disabled_by_default: false
  //     id: toshiba_ab_toshibaabestiazone1climate_id_2
  //     name: Toshiba Estia Zone 1 Water
  //     visual: {}
  //   zone2_climate:
  //     disabled_by_default: false
  //     id: toshiba_ab_toshibaabestiazone1climate_id
  //     name: Toshiba Estia Zone 2 Water
  //     visual: {}
  new(toshiba_ab_toshibaabclimate_id) toshiba_ab::ToshibaAbClimate();
  App.register_component_(toshiba_ab_toshibaabclimate_id, 5);
  App.register_climate(toshiba_ab_toshibaabclimate_id, "Estia DHW", 4114319247UL, 0);
  toshiba_ab_toshibaabclimate_id->set_uart_parent(uart_hostuartcomponent_id);
  new(toshiba_ab_toshibaabestiazone1climate_id) toshiba_ab::ToshibaAbEstiaZone1Climate(toshiba_ab_toshibaabclimate_id, 2);
  App.register_climate(toshiba_ab_toshibaabestiazone1climate_id, "Toshiba Estia Zone 2 Water", 4252356442UL, 0);
  toshiba_ab_toshibaabclimate_id->set_zone2_climate(toshiba_ab_toshibaabestiazone1climate_id);
  new(toshiba_ab_toshibaabestiazone1climate_id_2) toshiba_ab::ToshibaAbEstiaZone1Climate(toshiba_ab_toshibaabclimate_id);
  App.register_climate(toshiba_ab_toshibaabestiazone1climate_id_2, "Toshiba Estia Zone 1 Water", 4132320669UL, 0);
  toshiba_ab_toshibaabclimate_id->set_zone1_climate(toshiba_ab_toshibaabestiazone1climate_id_2);
  new(toshiba_ab_toshibaabremoteaddressselect_id) toshiba_ab::ToshibaAbRemoteAddressSelect(toshiba_ab_toshibaabclimate_id);
  toshiba_ab_toshibaabremoteaddressselect_id->traits.set_options({"0x40", "0x41"});
  App.register_select(toshiba_ab_toshibaabremoteaddressselect_id, "Toshiba Remote Address", 956959085, 67108864);  // category:config
  toshiba_ab_toshibaabclimate_id->set_remote_address_select(toshiba_ab_toshibaabremoteaddressselect_id);
  toshiba_ab_toshibaabclimate_id->set_command_mode_read(8);
  toshiba_ab_toshibaabclimate_id->set_command_mode_write(128);
  toshiba_ab_toshibaabclimate_id->set_frame_format(toshiba_ab::A0);
  toshiba_ab_toshibaabclimate_id->set_filter_frames(true);
  toshiba_ab_toshibaabclimate_id->set_packet_min_wait(uint32_t(200));
  toshiba_ab_toshibaabclimate_id->set_autoreset_errors(false);
  new(sensor_sensor_id) sensor::Sensor();
  sensor_sensor_id->set_state_class(sensor::STATE_CLASS_MEASUREMENT);
  sensor_sensor_id->set_accuracy_decimals(2);
  App.register_sensor(sensor_sensor_id, "Toshiba Noise Rate", 2574805693UL, 134217984);  // category:diagnostic, uom:errors/min
  toshiba_ab_toshibaabclimate_id->set_noise_rate_sensor(sensor_sensor_id);
  new(sensor_sensor_id_2) sensor::Sensor();
  sensor_sensor_id_2->set_state_class(sensor::STATE_CLASS_MEASUREMENT);
  sensor_sensor_id_2->set_accuracy_decimals(0);
  App.register_sensor(sensor_sensor_id_2, "Toshiba CRC Failures (5 min)", 1337786539, 134218240);  // category:diagnostic, uom:failures/5min
  toshiba_ab_toshibaabclimate_id->set_crc_failures_5min_sensor(sensor_sensor_id_2);
  new(text_sensor_textsensor_id) text_sensor::TextSensor();
  App.register_text_sensor(text_sensor_textsensor_id, "Toshiba Indoor Unit Count", 2814419716UL, 134217728);  // category:diagnostic
  toshiba_ab_toshibaabclimate_id->set_indoor_unit_count_text_sensor(text_sensor_textsensor_id);
  new(text_sensor_textsensor_id_2) text_sensor::TextSensor();
  App.register_text_sensor(text_sensor_textsensor_id_2, "Toshiba Indoor Units", 690264403, 134217728);  // category:diagnostic
  toshiba_ab_toshibaabclimate_id->set_indoor_units_text_sensor(text_sensor_textsensor_id_2);
  new(text_sensor_textsensor_id_3) text_sensor::TextSensor();
  App.register_text_sensor(text_sensor_textsensor_id_3, "Toshiba Remote Count", 2998620752UL, 134217728);  // category:diagnostic
  toshiba_ab_toshibaabclimate_id->set_remote_count_text_sensor(text_sensor_textsensor_id_3);
  new(text_sensor_textsensor_id_4) text_sensor::TextSensor();
  App.register_text_sensor(text_sensor_textsensor_id_4, "Toshiba Remote Addresses", 3379977861UL, 134217728);  // category:diagnostic
  toshiba_ab_toshibaabclimate_id->set_remote_addresses_text_sensor(text_sensor_textsensor_id_4);
  new(toshiba_ab_toshibaabestianightsetbackswitch_id) toshiba_ab::ToshibaAbEstiaNightSetbackSwitch(toshiba_ab_toshibaabclimate_id);
  toshiba_ab_toshibaabestianightsetbackswitch_id->set_restore_mode(switch_::SWITCH_ALWAYS_OFF);
  App.register_switch(toshiba_ab_toshibaabestianightsetbackswitch_id, "Night Setback", 1198415361, 0);
  toshiba_ab_toshibaabclimate_id->set_night_setback_switch(toshiba_ab_toshibaabestianightsetbackswitch_id);
  toshiba_ab_toshibaabclimate_id->set_autonomous(true);
  toshiba_ab_toshibaabclimate_id->set_read_only(false);
  toshiba_ab_toshibaabclimate_id->set_ping_enabled(true);
  toshiba_ab_toshibaabclimate_id->set_demand_enabled(false);
  // =========== AUTO GENERATED CODE END ============
  App.setup();
}

void loop() {
  App.loop();
}
