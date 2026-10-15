#include "esphome/components/toshiba_ab/toshiba_ab.h"
#include <cassert>
#include <cmath>
#include <iostream>

using namespace esphome;
using namespace esphome::toshiba_ab;

struct TestEstia : ToshibaAbClimate {
  using ToshibaAbClimate::publish_estia_telemetry_;
};

static DataFrame frame(uint8_t type, uint16_t dtype, size_t payload_end) {
  DataFrame f{};
  f.set_estia(true);
  f.set_estia_len(payload_end - 2);
  f.raw[0] = type;
  f.raw[1] = payload_end - 2;
  f.raw[7] = dtype >> 8;
  f.raw[8] = dtype;
  return f;
}
static void near(float actual, float expected) {
  if (!(std::fabs(actual - expected) < 0.001f)) {
    std::cerr << "Expected " << expected << ", got " << actual << "\n";
    std::abort();
  }
}

int main() {
  TestEstia unit;
  unit.set_frame_format(FrameFormat::A0);
  ToshibaAbEstiaZone1Climate z1(&unit), z2(&unit, 2);
  unit.set_zone1_climate(&z1);
  unit.set_zone2_climate(&z2);
  const auto traits1 = z1.traits();
  const auto traits2 = z2.traits();
  assert(traits1.get_visual_min_temperature() == 20);
  assert(traits1.get_visual_max_temperature() == 65);
  assert(traits2.get_visual_max_temperature() == 40);
  sensor::Sensor td, to, te, ts, tl, twi, tfi, ttw, flow, pressure, freq, fan, mix, duty, dhw_hours, heat_hours;
  unit.set_compressor_discharge_temp_sensor(&td);
  unit.set_outdoor_temp_sensor(&to);
  unit.set_outdoor_coil_temperature_sensor(&te);
  unit.set_suction_temperature_sensor(&ts);
  unit.set_outdoor_pipe_temperature_sensor(&tl);
  unit.set_water_inlet_temp_sensor(&twi);
  unit.set_zone2_water_temp_sensor(&tfi);
  unit.set_dhw_current_temp_sensor(&ttw);
  unit.set_water_flow_sensor(&flow);
  unit.set_low_pressure_sensor(&pressure);
  unit.set_compressor_frequency_sensor(&freq);
  unit.set_fan_speed_sensor(&fan);
  unit.set_mixing_valve_position_sensor(&mix);
  unit.set_waterpump_duty_sensor(&duty);
  unit.set_dhw_compressor_hours_sensor(&dhw_hours);
  unit.set_compressor_hours_sensor(&heat_hours);
  auto c0 = frame(0x18, 0x00E8, 28);
  c0.raw[9] = 0xC0;
  c0.raw[12] = 180;
  c0.raw[14] = 100; c0.raw[15] = 80; c0.raw[16] = 90; c0.raw[17] = 110;
  c0.raw[19] = 90; c0.raw[20] = 100; c0.raw[21] = 110; c0.raw[23] = 120; c0.raw[24] = 130;
  c0.raw[26] = 1; c0.raw[27] = 44;
  unit.publish_estia_telemetry_(&c0);
  near(td.state, 76); near(to.state, 31); near(te.state, 16);
  near(ts.state, 26); near(tl.state, 21); near(twi.state, 26.5);
  near(tfi.state, 36.5); near(ttw.state, 41.5); near(flow.state, 30); near(pressure.state, 1.21);
  near(z1.current_temperature, 31.5); near(z2.current_temperature, 36.5);
  for (auto sample : {160, 161}) {
    c0.raw[12] = sample;
    unit.publish_estia_telemetry_(&c0);
    near(td.state, sample == 160 ? 56 : 57);
  }
  // Missing sensors must not overwrite the last valid temperature.
  c0.raw[12] = 0xFF; c0.raw[23] = 0;
  unit.publish_estia_telemetry_(&c0);
  near(td.state, 57); near(z2.current_temperature, 36.5);
  auto status = frame(0x58, 0x03C6, 27);
  status.raw[24] = 130; status.raw[25] = 100; status.raw[26] = 120;
  status.raw[19] = 0x10;
  unit.preset = climate::CLIMATE_PRESET_BOOST;
  unit.publish_estia_telemetry_(&status);
  near(ttw.state, 49.5); near(twi.state, 34.5); near(tfi.state, 44.5);
  near(unit.current_temperature, 49.5); near(z2.current_temperature, 44.5);
  assert(z1.get_custom_preset() == "Night setback");
  assert(z2.get_custom_preset() == "Night setback");
  assert(unit.preset == climate::CLIMATE_PRESET_BOOST);
  auto event = frame(0x1C, 0x03C6, 16);
  event.raw[15] = 0;
  unit.publish_estia_telemetry_(&event);
  assert(z1.preset == climate::CLIMATE_PRESET_NONE);
  assert(z2.preset == climate::CLIMATE_PRESET_NONE);
  assert(!z1.has_custom_preset());
  // The applied flag is independent of the feature-enable flag in raw[10].
  status.raw[10] = 0x20; status.raw[19] = 0;
  unit.publish_estia_telemetry_(&status);
  assert(!z1.has_custom_preset());
  event.raw[15] = 0x10;
  unit.publish_estia_telemetry_(&event);
  assert(z1.has_custom_preset());
  auto c1 = frame(0x18, 0x00E8, 38);
  c1.raw[9] = 0xC1;
  c1.raw[14] = 1; c1.raw[15] = 2;
  c1.raw[18] = 2; c1.raw[19] = 3;
  c1.raw[29] = 63; c1.raw[31] = 50;
  c1.raw[32] = 3; c1.raw[33] = 232; c1.raw[37] = 75;
  unit.publish_estia_telemetry_(&c1);
  near(dhw_hours.state, 258); near(heat_hours.state, 515);
  near(mix.state, 63); near(freq.state, 50); near(fan.state, 1000); near(duty.state, 75);
  // A truncated frame cannot publish the second byte or CRC as telemetry.
  auto short_c0 = frame(0x18, 0x00E8, 27);
  short_c0.raw[9] = 0xC0; short_c0.raw[26] = 0xFF; short_c0.raw[27] = 0xFF;
  unit.publish_estia_telemetry_(&short_c0);
  near(flow.state, 30);
  TestEstia no_zones;
  no_zones.publish_estia_telemetry_(&status);
  // Full receive path validates CRC before publishing the decoded values.
  const size_t crc_offset = c0.estia_size() - 2;
  const uint16_t crc = c0.calculate_estia_crc();
  c0.raw[crc_offset] = crc >> 8;
  c0.raw[crc_offset + 1] = crc;
  assert(unit.receive_data_frame(&c0));
  near(to.state, 31);

  std::cout << "Estia telemetry checks passed\n";
}
