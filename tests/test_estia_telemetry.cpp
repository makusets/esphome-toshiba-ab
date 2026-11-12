#include "esphome/components/toshiba_ab/toshiba_ab.h"
#include "esphome/core/application.h"
#include "esphome/components/host/preferences.h"
#include <cassert>
#include <array>
#include <cmath>
#include <iostream>

using namespace esphome;
using namespace esphome::toshiba_ab;

struct TestEstia : ToshibaAbClimate {
  using ToshibaAbClimate::publish_estia_telemetry_;
  // The host runner enables access to internal queues to verify exact wire commands.
  std::vector<uint8_t> pop_command() {
    assert(!raw_write_queue_.empty());
    auto command = raw_write_queue_.front();
    raw_write_queue_.pop();
    DataFrame decoded{};
    decoded.set_estia(true);
    decoded.set_estia_len(command[3]);
    std::copy(command.begin() + 2, command.end(), decoded.raw);
    assert(decoded.validate_estia_crc());
    return command;
  }
  bool commands_empty() const { return raw_write_queue_.empty(); }
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

static void acknowledge(TestEstia &unit, uint16_t dtype) {
  auto ack = frame(0x18, 0x00A1, 11);
  ack.raw[3] = 0x08;
  ack.raw[5] = 0x00; ack.raw[6] = 0x40;
  ack.raw[9] = dtype >> 8; ack.raw[10] = dtype;
  const auto crc = ack.calculate_estia_crc();
  ack.raw[11] = crc >> 8; ack.raw[12] = crc;
  assert(unit.receive_data_frame(&ack));
}

static void control_checks() {
  TestEstia unit;
  unit.set_frame_format(FrameFormat::A0);
  ToshibaAbEstiaZoneClimate zone(&unit, 2);
  unit.set_zone2_climate(&zone);  // Zone 2 must also work without a Zone 1 entity.
  zone.mode = climate::CLIMATE_MODE_HEAT;
  zone.set_visual_min_temperature_override(20);
  zone.set_visual_max_temperature_override(40);
  for (float target : {7.0f, 18.5f, 40.0f, 65.0f}) {
    zone.make_call().set_target_temperature(target).perform();
    auto command = unit.pop_command();
    assert(command[9] == 0x03 && command[10] == 0xC1);
    assert(command[11] == 0x04 && command[12] == static_cast<uint8_t>((target + 16) * 2));
  }
  for (float target : {6.5f, 65.5f, NAN, INFINITY, -INFINITY})
    unit.send_estia_setpoint(target, climate::CLIMATE_MODE_HEAT, 2);
  assert(unit.commands_empty());
  unit.set_read_only(true);
  zone.make_call().set_mode(climate::CLIMATE_MODE_COOL).set_target_temperature(18).perform();
  assert(unit.commands_empty());
  unit.set_read_only(false);
  zone.make_call().set_mode(climate::CLIMATE_MODE_AUTO).set_target_temperature(35).perform();
  auto automatic = unit.pop_command();
  assert(automatic[10] == 0xC4 && automatic[12] == 1);
  assert(unit.commands_empty());  // No fixed target in Auto.
  acknowledge(unit, 0x03C4);
  unit.pop_command();  // shared heating mode
  zone.mode = climate::CLIMATE_MODE_AUTO;
  zone.make_call().set_target_temperature(35).perform();
  assert(unit.commands_empty());
  zone.make_call().set_mode(climate::CLIMATE_MODE_OFF).perform();
  auto off = unit.pop_command();
  assert(off[9] == 0 && off[10] == 0x41 && off[11] == 0x22);
  acknowledge(unit, 0x03C4);  // A late ACK cannot undo the newer Off request.
  assert(unit.commands_empty());
  zone.mode = climate::CLIMATE_MODE_OFF;
  zone.make_call().set_mode(climate::CLIMATE_MODE_COOL).perform();
  auto fixed = unit.pop_command();
  assert(fixed[10] == 0xC4 && fixed[12] == 0);
  acknowledge(unit, 0x03C4);
  auto cool = unit.pop_command();
  assert(cool[10] == 0xC0 && cool[11] == 0x01);
  acknowledge(unit, 0x03C0);
  auto on = unit.pop_command();
  assert(on[10] == 0x41 && on[11] == 0x23);
  assert(unit.commands_empty());
}

static uint8_t dn_checksum(const uint8_t *bytes, size_t size) {
  unsigned sum = 0;
  for (size_t i = 0; i < size; ++i) sum += bytes[i];
  return static_cast<uint8_t>(sum);
}

static void first_gen_cooling_setpoint_checks() {
  TestEstia unit;
  unit.set_frame_format(FrameFormat::ESTIA);
  unit.set_remote_address(0x60);
  unit.set_master_address(0x70);
  unit.boot_millis_ = millis() - INITIAL_FRAME_SEND_BLOCK_MILLIS;
  ToshibaAbEstiaZoneClimate zone(&unit);
  unit.set_zone1_climate(&zone);
  zone.mode = climate::CLIMATE_MODE_COOL;
  zone.make_call().set_target_temperature(20).perform();
  assert(unit.write_queue_.size() == 1);
  const auto command = unit.write_queue_.front();
  // Physical auxiliary controller's accepted 20 C cooling command, including CRC.
  const uint8_t captured[] = {0x0F, 0x60, 0x70, 0xE0, 0x01, 0x23,
                              0x01, 0x48, 0x48, 0xA2, 0x48, 0x5E};
  if (!std::equal(std::begin(captured), std::end(captured), command.raw)) {
    std::cerr << "Cooling20 packet does not match accepted wall-controller capture\n";
    for (size_t i = 0; i < sizeof(captured); i++)
      std::cerr << std::hex << static_cast<unsigned>(command.raw[i]) << ':';
    std::cerr << '\n';
    std::abort();
  }
  unit.write_queue_.pop();
  auto expect_target = [&](float target, bool cooling, uint8_t remote = 0x60) {
    assert(unit.write_queue_.size() == 1);
    const auto next = unit.write_queue_.front();
    unit.write_queue_.pop();
    const uint8_t encoded = static_cast<uint8_t>(std::round((target + 16) * 2));
    std::vector<uint8_t> expected{0x0F, remote, 0x70, 0xE0, 0x01, 0x23,
                                static_cast<uint8_t>(cooling ? 0x01 : 0x02), encoded,
                                static_cast<uint8_t>(cooling ? encoded : 0x66), 0xA2, encoded};
    expected.push_back(dn_checksum(expected.data(), expected.size()));
    assert(std::equal(expected.begin(), expected.end(), next.raw));
  };
  for (uint8_t remote : {0x60, 0x61}) {
    unit.set_remote_address(remote);
    zone.mode = climate::CLIMATE_MODE_COOL;
    for (float target : {7.0f, 15.0f, 20.0f, 24.5f, 25.0f}) {
      zone.make_call().set_target_temperature(target).perform();
      expect_target(target, true, remote);
    }
    for (float target : {6.5f, 25.5f, NAN, INFINITY, -INFINITY})
      unit.send_estia_first_gen_zone1_setpoint(target, climate::CLIMATE_MODE_COOL);
    assert(unit.write_queue_.empty());
    zone.mode = climate::CLIMATE_MODE_HEAT;
    for (float target : {20.0f, 30.0f, 49.5f, 65.0f}) {
      zone.make_call().set_target_temperature(target).perform();
      expect_target(target, false, remote);
    }
    for (float target : {19.5f, 65.5f, NAN, INFINITY, -INFINITY})
      unit.send_estia_first_gen_zone1_setpoint(target);
    assert(unit.write_queue_.empty());
  }
  unit.set_remote_address(0x60);
  // The physical status retains the selected mode even with Zone 1 switched Off.
  for (auto selected_mode : {climate::CLIMATE_MODE_HEAT, climate::CLIMATE_MODE_COOL, climate::CLIMATE_MODE_AUTO}) {
    const bool cooling = selected_mode == climate::CLIMATE_MODE_COOL;
    const bool automatic = selected_mode == climate::CLIMATE_MODE_AUTO;
    DataFrame status{};
    status.set_tu2c(true);
    const uint8_t report[] = {0x15, 0x70, 0xFF, 0xE0, 0xA0, 0x31, 0x40, 0x00, 0x10,
                              0xA2, 0x60, 0x66, 0x00, 0x48, 0x00, 0x00, 0x00, 0x35};
    std::copy(std::begin(report), std::end(report), status.raw);
    status.raw[6] = cooling ? 0x20 : 0x40;
    status.raw[7] = automatic ? 0x04 : 0x00;
    status.raw[17] = dn_checksum(status.raw, 17);
    assert(unit.receive_data_frame(&status));
    assert(zone.mode == climate::CLIMATE_MODE_OFF);
    zone.make_call().set_target_temperature(cooling ? 15 : 30).perform();
    if (automatic) {
      assert(unit.write_queue_.empty());  // Off must not hide retained Auto.
      status.raw[6] |= 1;
      status.raw[17] = dn_checksum(status.raw, 17);
      assert(unit.receive_data_frame(&status));
      assert(zone.mode == climate::CLIMATE_MODE_AUTO);
      zone.make_call().set_target_temperature(30).perform();
      assert(unit.write_queue_.empty());  // Running Auto is also curve-controlled.
      status.raw[6] &= ~1;
      status.raw[17] = dn_checksum(status.raw, 17);
      assert(unit.receive_data_frame(&status));
    } else {
      expect_target(cooling ? 15 : 30, cooling);
    }
  }
  // An explicit or deferred mode takes precedence over the previous report.
  zone.make_call().set_mode(climate::CLIMATE_MODE_HEAT).set_target_temperature(30).perform();
  unit.write_queue_.pop();  // On; mode is deferred until an active status arrives.
  expect_target(30, false);
  zone.make_call().set_mode(climate::CLIMATE_MODE_COOL).perform();
  unit.write_queue_.pop();  // A replacement pending Cool request.
  zone.make_call().set_target_temperature(15).perform();
  expect_target(15, true);
  zone.make_call().set_mode(climate::CLIMATE_MODE_AUTO).set_target_temperature(35).perform();
  unit.write_queue_.pop();
  assert(unit.write_queue_.empty());  // No fixed target in weather compensation.
  unit.set_read_only(true);
  zone.make_call().set_mode(climate::CLIMATE_MODE_COOL).set_target_temperature(15).perform();
  assert(unit.write_queue_.empty());
  unit.set_read_only(false);
  unit.boot_millis_ = millis();
  unit.send_estia_first_gen_zone1_setpoint(15, climate::CLIMATE_MODE_COOL);
  assert(unit.write_queue_.empty());
}

static void dn_checks() {
  TestEstia unit;
  unit.set_frame_format(FrameFormat::ESTIA);
  unit.set_remote_address(0x61);
  unit.set_master_address(0x70);
  ToshibaAbEstiaCurveShiftSelect curve(&unit);
  curve.traits.set_options({"-5", "-4", "-3", "-2", "-1", "0", "1", "2", "3", "4", "5"});
  unit.set_curve_shift_select(&curve);
  unit.boot_millis_ = millis();
  assert(!unit.read_estia_first_gen_dn(0x27, {})); // Boot guard.
  unit.boot_millis_ = millis() - INITIAL_FRAME_SEND_BLOCK_MILLIS;
  auto pop = [&]() {
    assert(!unit.write_queue_.empty());
    auto f = unit.write_queue_.front(); unit.write_queue_.pop();
    assert(f.raw[f.size() - 1] == dn_checksum(f.raw, f.size() - 1));
    return f;
  };
  curve.control("-2");
  auto write = pop();
  assert(write.raw[0] == 0x0C && write.raw[1] == 0x61 && write.raw[2] == 0x70);
  assert(write.raw[3] == 0xE0 && write.raw[4] == 1 && write.raw[5] == 0x1C);
  assert(write.raw[6] == 0x27 && write.raw[7] == 0xFE);
  assert(!curve.has_state());
  unit.update_estia_dn_();
  auto read = pop();
  assert(read.raw[0] == 0x0D && read.raw[4] == 0x41 && read.raw[5] == 0x1A);
  assert(read.raw[6] == 0xF5 && read.raw[7] == 0x27 && read.raw[8] == 0);
  assert(!unit.read_estia_first_gen_dn(0x21, {}));
  DataFrame reply{};
  reply.set_tu2c(true);
  const uint8_t bytes[] = {0x0C, 0x70, 0x61, 0xE0, 0x80, 0x1A, 0x27, 0xFE, 0};
  std::copy(std::begin(bytes), std::end(bytes), reply.raw);
  auto receive = [&]() {
    reply.raw[8] = dn_checksum(reply.raw, 8);
    unit.process_received_data_estia_first_gen_(&reply);
  };
  receive(); assert(!curve.has_state()); // Query still queued, not transmitted.
  // Simulate dispatch independently of host UART timing.
  unit.estia_dn_sent_ = true; unit.estia_dn_sent_ms_ = millis();
  reply.raw[2] = 0x62; receive(); assert(!curve.has_state());
  reply.raw[2] = 0x61; reply.raw[1] = 0x71; receive(); assert(!curve.has_state());
  reply.raw[1] = 0x70; reply.raw[6] = 0x29; receive(); assert(!curve.has_state());
  reply.raw[6] = 0x27; reply.raw[3] = 0xC0; receive(); assert(!curve.has_state());
  reply.raw[3] = 0xE0; reply.raw[0] = 0x0B; receive(); assert(!curve.has_state());
  reply.raw[0] = 0x0C; reply.raw[8] = 0;
  unit.process_received_data_estia_first_gen_(&reply); assert(!curve.has_state());
  receive(); assert(curve.current_option() == "-2" && !unit.estia_dn_pending_);
  reply.raw[7] = 3; receive(); assert(curve.current_option() == "-2"); // Unsolicited.
  curve.control("6"); curve.control("254"); curve.control("bad"); unit.set_curve_shift(-6);
  assert(unit.write_queue_.empty());
  bool completed = false;
  assert(unit.read_estia_first_gen_dn(0x21, [&](bool ok, uint8_t value) {
    assert(ok && value == 45); completed = true;
  })); pop();
  unit.estia_dn_sent_ = true; unit.estia_dn_sent_ms_ = millis();
  reply.raw[6] = 0x21; reply.raw[7] = 45; receive(); assert(completed);
  bool timed_out = false;
  assert(unit.read_estia_first_gen_dn(0x28, [&](bool ok, uint8_t) { timed_out = !ok; })); pop();
  unit.estia_dn_sent_ = true; unit.estia_dn_sent_ms_ = millis() - 10000;
  unit.update_estia_dn_(); assert(timed_out && !unit.estia_dn_pending_);
  assert(unit.read_estia_first_gen_dn(0x27, {})); pop();
  unit.estia_dn_pending_ = false;
  unit.curve_shift_refresh_ = true; unit.update_estia_dn_(); pop();
  unit.estia_dn_sent_ = true; unit.estia_dn_sent_ms_ = millis();
  reply.raw[6] = 0x27; reply.raw[7] = 0xF6; receive();
  assert(curve.current_option() == "-2"); // Out-of-range response.
  unit.set_read_only(true);
  assert(!unit.read_estia_first_gen_dn(0x27, {}));
  assert(!unit.write_estia_first_gen_dn(0x27, 0));
  unit.set_read_only(false); unit.set_frame_format(FrameFormat::A0);
  assert(!unit.read_estia_first_gen_dn(0x27, {}));
  assert(!unit.write_estia_first_gen_dn(0x27, 0));
}

static DataFrame tu2c_remote_frame(uint8_t source) {
  DataFrame f{};
  f.set_tu2c(true);
  f.raw[0] = 0x0C;
  f.raw[1] = source;
  f.raw[2] = 0x90;
  f.raw[3] = 0xC0;
  f.raw[4] = 0x41;
  f.raw[5] = 0x5C;
  f.raw[6] = 0x90;
  f.raw[7] = 0xF3;
  for (size_t i = 0; i < 8; ++i) f.raw[8] += f.raw[i];
  return f;
}

static void tu2c_address_checks() {
  TestEstia unit;
  unit.boot_millis_ = millis() - INITIAL_FRAME_SEND_BLOCK_MILLIS;
  unit.set_frame_format(FrameFormat::TU2C);
  assert(unit.remote_address_ == 0x50);
  text_sensor::TextSensor remotes;
  unit.set_remote_addresses_text_sensor(&remotes);
  auto f = tu2c_remote_frame(0x50);
  // Own echoes must neither cause collisions nor appear in the inventory.
  unit.remember_tx_frame_for_echo_(f.raw, f.size(), true);
  assert(unit.receive_data_frame(&f));
  assert(unit.remote_address_ == 0x50);
  assert(unit.discovered_remotes_.none());
  unit.last_tx_frame_for_echo_.reset();
  assert(unit.receive_data_frame(&f));
  assert(unit.remote_address_ == 0x51);
  assert(remotes.state == "0x50");
  assert(!unit.announce_ack_received_);
  unit.tu2c_send_ping();  // Uses the same address as registration/control.
  assert(unit.write_queue_.back().raw[1] == 0x51);
  f = tu2c_remote_frame(0x51);
  f.raw[8] ^= 1;
  assert(!unit.receive_data_frame(&f));
  assert(unit.remote_address_ == 0x51);
  f.raw[8] ^= 1;
  assert(unit.receive_data_frame(&f));
  assert(unit.remote_address_ == 0x53);  // 0x52 is reserved.
  assert(unit.write_queue_.size() == 1);  // No stale-address registrations.
  assert(unit.write_queue_.back().raw[1] == 0x53);
  assert(remotes.state == "0x50, 0x51");
  for (uint8_t address = 0x53; address < 0x59; ++address) {
    f = tu2c_remote_frame(address);
    assert(unit.receive_data_frame(&f));
    assert(unit.remote_address_ == address + 1);
  }
  f = tu2c_remote_frame(0x59);
  assert(unit.receive_data_frame(&f));
  assert(unit.remote_address_ == 0x59);
  assert(!unit.remote_address_available_);
  unit.tu2c_remote_announce();
  assert(unit.write_queue_.empty());

  // Discovery also recognizes non-keepalive remote traffic.
  TestEstia discovery;
  discovery.boot_millis_ = millis();
  discovery.set_frame_format(FrameFormat::TU2C);
  f = tu2c_remote_frame(0x57);
  f.raw[4] = 0x61;
  f.raw[8] += 0x20;
  assert(discovery.receive_data_frame(&f));
  assert(discovery.discovered_remotes_.test(0x57));
  f = tu2c_remote_frame(0x52);
  assert(discovery.receive_data_frame(&f));
  assert(!discovery.discovered_remotes_.test(0x52));

  // Explicit configuration uses the same address everywhere and disables auto movement.
  TestEstia fixed;
  fixed.set_remote_address(0x55);
  fixed.set_frame_format(FrameFormat::TU2C);
  f = tu2c_remote_frame(0x55);
  assert(fixed.receive_data_frame(&f));
  assert(fixed.remote_address_ == 0x55);
  fixed.tu2c_remote_announce();
  assert(fixed.write_queue_.back().raw[1] == 0x55);
  auto ack = tu2c_remote_frame(0x90);
  ack.raw[0] = 0x12;
  ack.raw[2] = 0x54;
  ack.raw[4] = 0x80;
  ack.raw[5] = 0x0D;
  assert(!fixed.is_tu2c_registration_ack_(&ack));
  ack.raw[2] = 0x55;
  assert(fixed.is_tu2c_registration_ack_(&ack));

  TestEstia normal;
  normal.set_frame_format(FrameFormat::NORMAL);
  assert(normal.remote_address_ == 0x40);
  normal.handle_remote_address_collision_(0x40, "test");
  assert(normal.remote_address_ == 0x41);
  normal.handle_remote_address_collision_(0x41, "test");
  assert(normal.remote_address_ == 0x43);
}

static void startup_address_checks() {
  static_assert(INITIAL_FRAME_SEND_BLOCK_MILLIS == 40000);
  for (auto format : {FrameFormat::TU2C, FrameFormat::NORMAL, FrameFormat::HM, FrameFormat::ESTIA}) {
    TestEstia unit;
    unit.set_frame_format(format);
    unit.boot_millis_ = millis();
    const bool estia = format == FrameFormat::ESTIA;
    const uint8_t base = estia ? 0x60 : format == FrameFormat::TU2C ? 0x50 : 0x40;
    // Higher addresses arrive first; remember them before our candidate collides.
    unit.handle_remote_address_collision_(base + 3, "test");
    unit.handle_remote_address_collision_(base + 1, "test");
    assert(unit.remote_address_ == base);
    unit.handle_remote_address_collision_(base, "test");
    assert(unit.remote_address_ == base + (estia ? 2 : 4));
    if (estia) {
      // Estia retains its existing range with no reserved address at +2.
      unit.handle_remote_address_collision_(base + 2, "test");
      assert(unit.remote_address_ == base + 4);
    }
    assert(unit.write_queue_.empty());
    if (format == FrameFormat::TU2C) unit.tu2c_remote_announce();
    else if (estia) unit.send_estia_first_gen_zone1(true);
    else unit.remote_announce();
    assert(unit.write_queue_.empty());  // No startup registrations queued.
    assert(unit.is_initial_frame_send_block_active_(unit.boot_millis_ + 39999));
    assert(!unit.is_initial_frame_send_block_active_(unit.boot_millis_ + 40000));
    unit.boot_millis_ = millis() - INITIAL_FRAME_SEND_BLOCK_MILLIS;
    if (format == FrameFormat::TU2C) unit.tu2c_remote_announce();
    else if (estia) unit.send_estia_first_gen_zone1(true);
    else unit.remote_announce();
    assert(unit.write_queue_.size() == 1);
    const auto &registration = unit.write_queue_.front();
    assert((registration.is_tu2c() ? registration.raw[1] : registration.source) == base + 4);
    // Late collisions still skip addresses remembered during initial listening.
    unit.handle_remote_address_collision_(base + 6, "test");
    unit.handle_remote_address_collision_(base + 5, "test");
    unit.handle_remote_address_collision_(base + 4, "test");
    assert(unit.remote_address_ == base + 7);
  }
}

static void first_gen_temperature_checks() {
  TestEstia unit;
  unit.set_frame_format(FrameFormat::ESTIA);
  unit.set_master_address(0x70);
  ToshibaAbEstiaZoneClimate zone(&unit);
  sensor::Sensor water, dhw;
  unit.set_zone1_climate(&zone);
  unit.set_zone1_water_temp_sensor(&water);
  unit.set_dhw_current_temp_sensor(&dhw);

  // Exact 18-byte status from #242: length 0x15 includes three framing bytes.
  DataFrame status{};
  status.set_tu2c(true);
  const uint8_t captured[] = {0x15, 0x70, 0xFF, 0xE0, 0xA0, 0x31, 0x41, 0x00, 0x10,
                              0xA2, 0x60, 0x66, 0x00, 0x48, 0x00, 0x00, 0x00, 0x36};
  std::copy(std::begin(captured), std::end(captured), status.raw);
  assert(dn_checksum(status.raw, 17) == status.raw[17]);
  auto receive = [&]() {
    const size_t checksum_offset = status.raw[0] - 4;
    status.raw[checksum_offset] = dn_checksum(status.raw, checksum_offset);
    unit.process_received_data_estia_first_gen_(&status);
  };
  receive();
  near(water.state, 20.0f); near(zone.current_temperature, 20.0f);
  assert(!dhw.has_state() && std::isnan(unit.current_temperature));
  status.raw[13] = 0x5F; receive();
  near(water.state, 31.5f); near(zone.current_temperature, 31.5f);

  // Same length, byte-14 layout: preserve its precedence even if both are populated.
  status.raw[14] = 0x64; status.raw[12] = 0x84; receive();
  near(water.state, 34.0f); near(zone.current_temperature, 34.0f);
  near(dhw.state, 50.0f); near(unit.current_temperature, 50.0f);
  status.raw[13] = 0; receive(); near(water.state, 34.0f);

  // Empty slots must not overwrite valid readings with -16 degrees.
  status.raw[12] = 0; status.raw[14] = 0; receive();
  near(water.state, 34.0f); near(zone.current_temperature, 34.0f);
  near(dhw.state, 50.0f); near(unit.current_temperature, 50.0f);

  // Short status and bad checksums cannot supply current temperatures.
  status.raw[13] = 0x50; status.raw[14] = 0x60; status.raw[12] = 0x80;
  status.raw[17] = dn_checksum(status.raw, 17) ^ 1;
  unit.process_received_data_estia_first_gen_(&status);
  near(water.state, 34.0f); near(dhw.state, 50.0f);
  status.raw[0] = 0x0F; receive();
  near(water.state, 34.0f); near(zone.current_temperature, 34.0f);
  near(dhw.state, 50.0f);
}

static DataFrame first_gen_mode_status(uint8_t flags, bool long_status) {
  DataFrame status{};
  status.set_tu2c(true);
  // Captured Series 5 status layouts, with mode/availability varied below.
  const uint8_t captured[] = {0x15, 0x70, 0xFF, 0xE0, 0xA0, 0x31, 0xA0, 0x00, 0x00,
                              0xA2, 0x48, 0x48, 0x00, 0x7A, 0x52, 0x00, 0x00, 0xD3};
  std::copy(std::begin(captured), std::end(captured), status.raw);
  status.raw[6] = flags;
  if (!long_status) {
    status.raw[0] = 0x11;
    status.raw[4] = 0xC0;
  }
  const size_t end = status.raw[0] - 4;
  status.raw[end] = dn_checksum(status.raw, end);
  return status;
}

static void first_gen_mode_setup(TestEstia &unit) {
  unit.set_frame_format(FrameFormat::ESTIA);
  unit.set_remote_address(0x61);
  unit.set_master_address(0x70);
  unit.boot_millis_ = millis() - INITIAL_FRAME_SEND_BLOCK_MILLIS;
}

static DataFrame pop_first_gen_mode_command(TestEstia &unit) {
  assert(!unit.write_queue_.empty());
  auto command = unit.write_queue_.front();
  unit.write_queue_.pop();
  assert(command.raw[1] == 0x61 && command.raw[2] == 0x70);
  assert(command.raw[command.size() - 1] == dn_checksum(command.raw, command.size() - 1));
  return command;
}

static void check_first_gen_mode_commands(TestEstia &unit, climate::ClimateMode mode, bool dhw_available) {
  auto automatic = pop_first_gen_mode_command(unit);
  assert(automatic.raw[5] == 0x24 && automatic.raw[6] == 0x01);
  assert(automatic.raw[7] == (mode == climate::CLIMATE_MODE_AUTO ? 1 : 0));
  auto command = pop_first_gen_mode_command(unit);
  const uint8_t expected = (mode == climate::CLIMATE_MODE_COOL ? 0x01 : 0x02) | (dhw_available ? 0x04 : 0);
  assert(command.raw[5] == 0x22 && command.raw[6] == expected && command.raw[7] == expected);
  assert(unit.write_queue_.empty());
}

static void first_gen_dhw_mode_checks() {
  // The physical DIP reversal changes status bit 7, independently of DHW On/Off.
  for (bool available : {false, true}) {
    for (bool long_status : {false, true}) {
      for (bool dhw_on : {false, true}) {
        for (auto mode : {climate::CLIMATE_MODE_HEAT, climate::CLIMATE_MODE_COOL, climate::CLIMATE_MODE_AUTO}) {
          TestEstia unit;
          first_gen_mode_setup(unit);
          ToshibaAbEstiaZoneClimate zone(&unit);
          unit.set_zone1_climate(&zone);
          auto status = first_gen_mode_status(0x21 | (available ? 0x80 : 0) | (dhw_on ? 2 : 0), long_status);
          assert(unit.receive_data_frame(&status));
          zone.make_call().set_mode(mode).perform();
          check_first_gen_mode_commands(unit, mode, available);
        }
      }
    }
  }

  // Before the first status, retain the request and enable only; learn availability
  // from the first active report before sending the deferred mode.
  for (bool available : {false, true}) {
    TestEstia unit;
    first_gen_mode_setup(unit);
    ToshibaAbEstiaZoneClimate zone(&unit);
    unit.set_zone1_climate(&zone);
    zone.make_call().set_mode(climate::CLIMATE_MODE_AUTO).perform();
    auto on = pop_first_gen_mode_command(unit);
    assert(on.raw[5] == 0x21 && on.raw[6] == 0x03 && unit.write_queue_.empty());
    auto inactive = first_gen_mode_status(available ? 0xC0 : 0x40, true);
    assert(unit.receive_data_frame(&inactive));
    assert(unit.write_queue_.empty());
    auto active = first_gen_mode_status(available ? 0xC1 : 0x41, false);
    assert(unit.receive_data_frame(&active));
    check_first_gen_mode_commands(unit, climate::CLIMATE_MODE_AUTO, available);
    assert(unit.receive_data_frame(&active));
    assert(unit.write_queue_.empty());  // No repeated request on later status.
  }

  TestEstia unit;
  first_gen_mode_setup(unit);
  ToshibaAbEstiaZoneClimate zone(&unit);
  unit.set_zone1_climate(&zone);
  unit.send_estia_first_gen_zone1_mode(climate::CLIMATE_MODE_HEAT);
  assert(unit.write_queue_.empty());  // Unknown must not mean "no DHW".
  auto valid = first_gen_mode_status(0xC1, false);
  for (int malformed = 0; malformed < 6; ++malformed) {
    auto invalid = valid;
    if (malformed == 0) invalid.raw[13] ^= 1;  // Bad checksum.
    if (malformed == 1) invalid.raw[1] = 0x71;  // Another master.
    if (malformed == 2) invalid.raw[3] = 0xC0;  // Other protocol signature.
    if (malformed >= 3) invalid.raw[0] = 0x0C + (malformed - 3);  // Missing target fields/checksum space.
    if (malformed != 0) {
      const size_t end = invalid.raw[0] - 4;
      invalid.raw[end] = dn_checksum(invalid.raw, end);
    }
    unit.receive_data_frame(&invalid);
    assert(unit.write_queue_.empty());
  }
  assert(unit.receive_data_frame(&valid));
  check_first_gen_mode_commands(unit, climate::CLIMATE_MODE_HEAT, true);

  // A later valid DIP report changes the next mode command, even with DHW OFF.
  auto no_dhw = first_gen_mode_status(0x41, true);
  assert(unit.receive_data_frame(&no_dhw));
  zone.make_call().set_mode(climate::CLIMATE_MODE_COOL).perform();
  check_first_gen_mode_commands(unit, climate::CLIMATE_MODE_COOL, false);
  assert(unit.receive_data_frame(&valid));
  zone.make_call().set_mode(climate::CLIMATE_MODE_HEAT).perform();
  check_first_gen_mode_commands(unit, climate::CLIMATE_MODE_HEAT, true);
  unit.set_read_only(true);
  zone.make_call().set_mode(climate::CLIMATE_MODE_COOL).perform();
  assert(unit.write_queue_.empty());
  unit.set_read_only(false);
  unit.boot_millis_ = millis();
  zone.make_call().set_mode(climate::CLIMATE_MODE_COOL).perform();
  assert(unit.write_queue_.empty());

  // Off cancels a pre-status request; later status cannot revive it.
  TestEstia cancelled;
  first_gen_mode_setup(cancelled);
  ToshibaAbEstiaZoneClimate cancelled_zone(&cancelled);
  cancelled.set_zone1_climate(&cancelled_zone);
  cancelled_zone.make_call().set_mode(climate::CLIMATE_MODE_HEAT).perform();
  pop_first_gen_mode_command(cancelled);
  cancelled_zone.make_call().set_mode(climate::CLIMATE_MODE_OFF).perform();
  auto off = pop_first_gen_mode_command(cancelled);
  assert(off.raw[5] == 0x21 && off.raw[6] == 0x02);
  assert(cancelled.receive_data_frame(&valid));
  assert(cancelled.write_queue_.empty());
}

static void intake_air_temperature_checks() {
  TestEstia unit;
  unit.set_frame_format(FrameFormat::NORMAL);
  sensor::Sensor intake;
  unit.set_intake_air_temperature_sensor(&intake);
  sensor::Sensor yaml_ta;
  unit.add_polled_sensor(0x02, 2.0f, 0, &yaml_ta);
  // Short TCC-Link response carries no ID: associate it with query 02.
  DataFrame short_reply{};
  short_reply.source = 0x00;
  short_reply.data_length = 7;
  const uint8_t short_data[] = {0x80, 0xEF, 0x80, 0x00, 0x2C, 0x00, 27};
  std::copy(std::begin(short_data), std::end(short_data), short_reply.data);
  unit.last_sensor_query_id_ = 0x02;
  unit.sensor_query_outstanding_ = true;
  unit.process_sensor_value_(&short_reply);
  near(unit.intake_air_temperature_, 27.0f);
  near(intake.state, 27.0f);
  near(yaml_ta.state, 54.0f);
  assert(!unit.sensor_query_outstanding_);
  // Long response identifies TA itself; negative values are signed degrees.
  DataFrame long_reply{};
  long_reply.source = 0x00;
  long_reply.data_length = 9;
  const uint8_t long_data[] = {0x80, 0xEF, 0x00, 0x2C, 0x08, 0x00, 0x02, 0xFF, 0xFE};
  std::copy(std::begin(long_data), std::end(long_data), long_reply.data);
  unit.process_sensor_value_(&long_reply);
  near(intake.state, -2.0f);
  near(yaml_ta.state, -4.0f);

  TestEstia tu;
  tu.set_frame_format(FrameFormat::TU2C);
  tu.set_remote_address(0x51);
  tu.boot_millis_ = millis() - INITIAL_FRAME_SEND_BLOCK_MILLIS;
  tu.announce_ack_received_ = true;
  tu.send_sensor_query(0x02);
  assert(tu.write_queue_.size() == 1);
  auto query = tu.write_queue_.front(); tu.write_queue_.pop();
  assert(query.raw[0] == 0x0C && query.raw[1] == 0x51 && query.raw[2] == 0x90);
  assert(query.raw[4] == 0x41 && query.raw[5] == 0x5C && query.raw[6] == 0x90 && query.raw[7] == 0x02);
  assert(query.raw[8] == dn_checksum(query.raw, 8));
  // Observe another remote's query, then replay a CRC-validated master reply.
  query.raw[1] = 0x50; query.raw[8] = dn_checksum(query.raw, 8);
  assert(tu.receive_data_frame(&query));
  DataFrame reply{};
  reply.set_tu2c(true);
  const uint8_t data[] = {0x0C, 0x90, 0x50, 0xC0, 0x80, 0x5C, 0x00, 27, 0x00};
  std::copy(std::begin(data), std::end(data), reply.raw);
  reply.raw[8] = dn_checksum(reply.raw, 8);
  assert(tu.receive_data_frame(&reply));
  near(tu.intake_air_temperature_, 27.0f);
  assert(tu.sensor_query_outstanding_);  // Reply to another remote is not our ACK.
  // A keepalive query replaces attribution; F3 must never overwrite TA.
  query.raw[7] = 0xF3; query.raw[8] = dn_checksum(query.raw, 8);
  assert(tu.receive_data_frame(&query));
  reply.raw[7] = 99; reply.raw[8] = dn_checksum(reply.raw, 8);
  assert(tu.receive_data_frame(&reply));
  near(tu.intake_air_temperature_, 27.0f);
  query.raw[7] = 0x02; query.raw[8] = dn_checksum(query.raw, 8);
  assert(tu.receive_data_frame(&query));
  reply.raw[7] = 28; reply.raw[8] = dn_checksum(reply.raw, 8) ^ 1;
  assert(!tu.receive_data_frame(&reply));
  near(tu.intake_air_temperature_, 27.0f);
  sensor::Sensor current;
  tu.add_polled_sensor(0x6A, 0.1f, 0, &current);
  // YAML service sensors use the same queued TU2C query and reply dispatcher.
  assert(tu.enqueue_sensor_query_(0x6A));
  assert(!tu.enqueue_sensor_query_(0x6A));  // Pending IDs are deduplicated.
  tu.sensor_query_outstanding_ = false;
  tu.drain_sensor_query_queue_();
  assert(tu.write_queue_.size() == 1);
  auto service_query = tu.write_queue_.front(); tu.write_queue_.pop();
  assert(service_query.raw[7] == 0x6A);
  // The transmitter records this attribution when the queued query reaches the wire.
  tu.tu2c_monitor_query_id_ = 0x6A;
  tu.tu2c_monitor_requester_ = 0x51;
  reply.raw[2] = 0x51; reply.raw[7] = 42;
  reply.raw[8] = dn_checksum(reply.raw, 8);
  assert(tu.receive_data_frame(&reply));
  near(current.state, 4.2f);
  near(tu.intake_air_temperature_, 27.0f);
  assert(!tu.sensor_query_outstanding_);
  // Unsupported monitor IDs complete the request without publishing bogus data.
  tu.last_sensor_query_id_ = 0x6A;
  tu.sensor_query_outstanding_ = true;
  tu.tu2c_monitor_query_id_ = 0x6A;
  DataFrame missing{}; missing.set_tu2c(true);
  const uint8_t no_data[] = {0x0A, 0x90, 0x51, 0xC0, 0x80, 0xA2, 0x00};
  std::copy(std::begin(no_data), std::end(no_data), missing.raw);
  missing.raw[6] = dn_checksum(missing.raw, 6);
  assert(tu.receive_data_frame(&missing));
  assert(!tu.sensor_query_outstanding_);
  near(current.state, 4.2f);
}

static void first_gen_keepalive_checks() {
  TestEstia unit;
  unit.set_frame_format(FrameFormat::ESTIA);
  unit.set_master_address(0x70);
  auto keepalive = [&]() {
    unit.send_estia_first_gen_ping_();
    unit.send_estia_first_gen_header_status_();
  };
  auto expect_ping = [&](uint8_t source, uint8_t checksum) {
    assert(!unit.write_queue_.empty());
    const auto ping = unit.write_queue_.front();
    const std::vector<uint8_t> captured{0x0F, source, 0x70, 0xE0, 0x41, 0x0C,
                                      0x81, 0x00, 0x00, 0x48, 0x00, checksum};
    assert(ping.is_tu2c() && ping.get_data() == captured);
    assert(!unit.should_track_command_ack_(ping));
    unit.write_queue_.pop();
  };
  unit.boot_millis_ = millis();
  keepalive();
  assert(unit.write_queue_.empty());  // Preserve startup silence.
  unit.boot_millis_ = millis() - INITIAL_FRAME_SEND_BLOCK_MILLIS;
  unit.master_address_confirmed_ = false;
  keepalive();
  assert(unit.write_queue_.empty());
  unit.master_address_confirmed_ = true;
  keepalive();
  assert(unit.write_queue_.size() == 2);
  expect_ping(0x60, 0xD5);
  const auto command = unit.write_queue_.front();
  const std::vector<uint8_t> captured{0x0B, 0x60, 0x70, 0xE0, 0x61, 0x31, 0x00, 0x4D};
  assert(command.is_tu2c() && command.get_data() == captured);
  assert(!unit.should_track_command_ack_(command));
  unit.write_queue_.pop();
  unit.set_read_only(true);
  keepalive();
  assert(unit.write_queue_.empty());
  unit.set_read_only(false);
  unit.set_ping_enabled(false);
  keepalive();
  assert(unit.write_queue_.empty());
  unit.set_ping_enabled(true);
  keepalive();
  assert(unit.write_queue_.size() == 2);
  unit.handle_remote_address_collision_(0x60, "test");
  assert(unit.remote_address_ == 0x61 && unit.write_queue_.empty());
  keepalive();
  assert(unit.write_queue_.size() == 1);  // A Second sends PING, without Header status.
  expect_ping(0x61, 0xD6);
  for (auto format : {FrameFormat::NORMAL, FrameFormat::HM, FrameFormat::TU2C, FrameFormat::A0}) {
    unit.set_frame_format(format);
    unit.set_remote_address(0x60);
    keepalive();
    assert(unit.write_queue_.empty());
  }
}

static void regular_temperature_checks() {
  TestEstia unit;
  unit.set_frame_format(FrameFormat::TU2C);
  unit.boot_millis_ = millis();
  unit.announce_ack_received_ = true;
  unit.send_regular_remote_temperature_();
  assert(unit.write_queue_.empty());  // Startup silence.
  unit.boot_millis_ = millis() - INITIAL_FRAME_SEND_BLOCK_MILLIS;
  unit.announce_ack_received_ = false;
  unit.send_regular_remote_temperature_();
  assert(unit.write_queue_.empty());  // Must register first in autonomous mode.
  unit.announce_ack_received_ = true;
  unit.current_temperature = 10.0f;
  unit.intake_air_temperature_ = 24.0f;
  auto report = [&](uint8_t flag, uint8_t temperature) {
    unit.send_regular_remote_temperature_();
    assert(unit.write_queue_.size() == 1);
    auto f = unit.write_queue_.front(); unit.write_queue_.pop();
    assert(f.is_tu2c());
    assert(f.get_data() == std::vector<uint8_t>({0x0D, 0x50, 0x90, 0xC0, 0x61, 0x38,
                                               flag, temperature, 0x00, f.raw[9]}));
    assert(f.raw[9] == dn_checksum(f.raw, 9));
    assert(!unit.should_track_command_ack_(f));
    return f;
  };
  report(0x01, 117);  // TA 24 C minus 0.5 C, independent of control temperature.
  unit.intake_air_temperature_ = NAN;
  unit.target_temperature = 25.0f;
  report(0x01, 120);
  unit.target_temperature = NAN;
  report(0x01, 110);  // Default 20 C.
  sensor::Sensor room;
  unit.set_ext_temp_source(&room);
  room.publish_state(22.5f);
  auto f = report(0x01, 115);  // Configured sensors also use the regular report flag.
  // The decoder reads raw[7], while leaving control temperature to unit status.
  TestEstia receiver;
  receiver.set_frame_format(FrameFormat::TU2C);
  receiver.set_remote_address(0x51);
  receiver.tcc_state.room_temp = 19.0f;
  assert(receiver.receive_data_frame(&f));
  near(receiver.tcc_state.room_temp, 19.0f);
  unit.set_read_only(true);
  unit.send_regular_remote_temperature_();
  assert(unit.write_queue_.empty());
  unit.set_read_only(false);
  for (uint8_t address = 0x51; address <= 0x59; ++address) {
    unit.set_remote_address(address);
    unit.send_regular_remote_temperature_();
    assert(unit.write_queue_.empty());
  }
  unit.set_always_report_temperature(true);
  for (uint8_t address = 0x51; address <= 0x59; ++address) {
    if (address == TOSHIBA_TU2C_TEMP_SENSOR) continue;
    unit.set_remote_address(address);
    unit.send_regular_remote_temperature_();
    assert(unit.write_queue_.size() == 1);
    const auto &f = unit.write_queue_.front();
    assert(f.raw[1] == address && f.raw[6] == 0x01 && f.raw[7] == 115);
    assert(f.raw[9] == dn_checksum(f.raw, 9));
    unit.write_queue_.pop();
  }
  TestEstia collided;
  collided.set_frame_format(FrameFormat::TU2C);
  collided.boot_millis_ = millis() - INITIAL_FRAME_SEND_BLOCK_MILLIS;
  collided.handle_remote_address_collision_(0x50, "test");
  collided.write_queue_ = {};
  collided.announce_ack_received_ = true;
  collided.send_regular_remote_temperature_();
  assert(collided.write_queue_.empty());
  for (auto format : {FrameFormat::NORMAL, FrameFormat::HM}) {
    TestEstia normal;
    normal.set_frame_format(format);
    normal.boot_millis_ = millis() - INITIAL_FRAME_SEND_BLOCK_MILLIS;
    normal.announce_ack_received_ = true;
    normal.current_temperature = 10.0f;
    normal.intake_air_temperature_ = 24.0f;
    normal.send_regular_remote_temperature_();
    assert(normal.write_queue_.size() == 1);
    auto command = normal.write_queue_.front();
    assert(command.source == 0x40 && command.opcode1 == OPCODE_TEMPERATURE);
    assert(command.data[1] == 0x81 && command.data[3] == 117);
    assert(command.crc() == command.calculate_crc());
    normal.write_queue_.pop();
    normal.set_remote_address(0x41);
    normal.send_regular_remote_temperature_();
    assert(normal.write_queue_.empty());
    normal.set_always_report_temperature(true);
    normal.send_regular_remote_temperature_();
    assert(normal.write_queue_.size() == 1);
    assert(normal.write_queue_.front().source == 0x41);
  }
  for (auto format : {FrameFormat::A0, FrameFormat::ESTIA}) {
    TestEstia other;
    other.set_frame_format(format);
    other.send_regular_remote_temperature_();
    assert(other.write_queue_.empty());
  }
}

struct RecordingUart : uart::UARTComponent {
  size_t transmitted{0};
  std::vector<std::vector<uint8_t>> writes;
  std::vector<uint8_t> rx;
  size_t rx_offset{0};
  void write_array(const uint8_t *data, size_t len) override {
    transmitted += len;
    writes.emplace_back(data, data + len);
  }
  bool peek_byte(uint8_t *data) override {
    if (!available()) return false;
    *data = rx[rx_offset];
    return true;
  }
  bool read_array(uint8_t *data, size_t len) override {
    if (len > available()) return false;
    std::copy_n(rx.begin() + rx_offset, len, data);
    rx_offset += len;
    return true;
  }
  size_t available() override { return rx.size() - rx_offset; }
  uart::UARTFlushResult flush() override { return uart::UARTFlushResult::UART_FLUSH_RESULT_SUCCESS; }
  void check_logger_conflict() override {}
};

static DataFrame first_gen_status(uint8_t enables, bool automatic, uint8_t activity = 0) {
  DataFrame status{};
  status.set_tu2c(true);
  const uint8_t bytes[] = {0x15, 0x70, 0xFF, 0xE0, 0xA0, 0x31, enables,
                           static_cast<uint8_t>(automatic ? 0x04 : 0), activity,
                           0xA2, 0x7A, 0x66, 0x00, 0x48, 0x00, 0x00, 0x00, 0x00};
  std::copy(std::begin(bytes), std::end(bytes), status.raw);
  status.raw[17] = dn_checksum(status.raw, 17);
  return status;
}

static void first_gen_activity_checks() {
  struct Step {
    uint8_t enables;
    uint8_t activity;
    bool waterpump;
    bool heating_cooling_compressor;
    bool dhw_compressor;
    bool dhw_resistor;
  };
  // Heat/Cool starts, circulation run-on, DHW and the observed changeover.
  const Step steps[] = {
      {0x40, 0x00, false, false, false, false},
      {0x41, 0x00, false, false, false, false},
      {0x41, 0x10, true, false, false, false},
      {0x41, 0x12, true, true, false, false},
      {0x40, 0x10, true, false, false, false},
      {0x40, 0x00, false, false, false, false},
      {0x21, 0x12, true, true, false, false},
      {0x20, 0x10, true, false, false, false},
      {0xC2, 0x10, true, false, false, false},
      {0xC2, 0x18, true, false, true, false},
      {0xC1, 0x02, false, true, false, false},
      {0xC1, 0x00, false, false, false, false},
      {0xC1, 0x10, true, false, false, false},
      // Independent resistor and activity bits, including unrelated bits.
      {0xC0, 0x04, false, false, false, true},
      {0xC0, 0x1E, true, true, true, true},
      {0xC3, 0xE1, false, false, false, false},
  };
  for (bool full_status : {false, true}) {
    RecordingUart uart;
    TestEstia unit;
    unit.set_uart_parent(&uart);
    unit.set_frame_format(FrameFormat::ESTIA);
    unit.set_master_address(0x70);
    unit.set_read_only(true);
    binary_sensor::BinarySensor waterpump, heating_cooling_compressor, dhw_compressor, dhw_resistor;
    unit.set_waterpump_active_binary_sensor(&waterpump);
    unit.set_heating_cooling_compressor_active_binary_sensor(&heating_cooling_compressor);
    unit.set_dhw_compressor_active_binary_sensor(&dhw_compressor);
    unit.set_dhw_resistor_active_binary_sensor(&dhw_resistor);
    assert(!waterpump.has_state() && !heating_cooling_compressor.has_state() && !dhw_compressor.has_state());
    for (const auto &step : steps) {
      auto status = first_gen_status(step.enables, false, step.activity);
      const size_t checksum_index = full_status ? 17 : 13;
      if (!full_status) {
        status.raw[0] = 0x11;
        status.raw[4] = 0xC0;
      }
      status.raw[checksum_index] = dn_checksum(status.raw, checksum_index);
      assert(unit.receive_data_frame(&status));
      assert(waterpump.has_state() && waterpump.state == step.waterpump);
      assert(heating_cooling_compressor.has_state() &&
             heating_cooling_compressor.state == step.heating_cooling_compressor);
      assert(dhw_compressor.has_state() && dhw_compressor.state == step.dhw_compressor);
      assert(dhw_resistor.has_state() && dhw_resistor.state == step.dhw_resistor);
      // Neither corrupted packets nor another source may change these states.
      status.raw[8] ^= 0x1E;
      unit.receive_data_frame(&status);
      assert(waterpump.state == step.waterpump &&
             heating_cooling_compressor.state == step.heating_cooling_compressor);
      assert(dhw_compressor.state == step.dhw_compressor && dhw_resistor.state == step.dhw_resistor);
      status.raw[1] = 0x71;
      status.raw[checksum_index] = dn_checksum(status.raw, checksum_index);
      unit.receive_data_frame(&status);
      assert(waterpump.state == step.waterpump &&
             heating_cooling_compressor.state == step.heating_cooling_compressor);
      assert(dhw_compressor.state == step.dhw_compressor && dhw_resistor.state == step.dhw_resistor);
    }
    assert(uart.transmitted == 0 && unit.commands_empty());
  }
}

static void first_gen_feature_control_checks() {
  for (bool dhw_available : {false, true}) {
    for (uint8_t remote : {0x60, 0x61}) {
      TestEstia unit;
      unit.set_frame_format(FrameFormat::ESTIA);
      unit.set_remote_address(remote);
      unit.set_master_address(0x70);
      unit.boot_millis_ = millis() - INITIAL_FRAME_SEND_BLOCK_MILLIS;
      ToshibaAbEstiaZoneClimate zone(&unit);
      ToshibaAbEstiaFrostProtectionSwitch frost(&unit);
      ToshibaAbEstiaNightSetbackSwitch night(&unit);
      ToshibaAbEstiaSilentModeSwitch silent(&unit);
      const std::array<switch_::Switch *, 3> switches{&frost, &night, &silent};
      std::array<optional<bool>, 3> reported_switch_states{};
      for (size_t index = 0; index < switches.size(); index++) {
        switches[index]->add_on_state_callback(
            [&, index](bool state) { reported_switch_states[index] = state; });
      }
      unit.set_zone1_climate(&zone);
      unit.set_frost_protection_switch(&frost);
      unit.set_night_setback_switch(&night);
      unit.set_silent_mode_switch(&silent);
      auto status = first_gen_status(dhw_available ? 0xC0 : 0x40, false, 0);
      auto receive = [&]() {
        status.raw[17] = dn_checksum(status.raw, 17);
        assert(unit.receive_data_frame(&status));
        assert(zone.mode == climate::CLIMATE_MODE_OFF && unit.mode == climate::CLIMATE_MODE_OFF);
        near(zone.target_temperature, 45);
        assert(unit.write_queue_.empty() && unit.commands_empty());
      };
      receive();
      for (const auto &state : reported_switch_states) {
        assert(state.has_value() && !state.value());
      }
      struct Feature {
        switch_::Switch *sw;
        std::array<uint8_t, 9> on;
        std::array<uint8_t, 9> off;
        uint8_t mask;
      };
      // Literal wall-controller captures. Frost was confirmed on main and aux;
      // Night Setback and Silent Mode were captured on aux.
      Feature features[] = {
          {&frost, {0x0C, 0x60, 0x70, 0xE0, 0x01, 0x24, 0x02, 0x02, 0xE5},
                   {0x0C, 0x60, 0x70, 0xE0, 0x01, 0x24, 0x02, 0x00, 0xE3}, 0x08},
          {&night, {0x0C, 0x60, 0x70, 0xE0, 0x01, 0x24, 0x88, 0x08, 0x71},
                   {0x0C, 0x60, 0x70, 0xE0, 0x01, 0x24, 0x88, 0x00, 0x69}, 0x20},
          {&silent, {0x0C, 0x60, 0x70, 0xE0, 0x01, 0x24, 0x04, 0x04, 0xE9},
                    {0x0C, 0x60, 0x70, 0xE0, 0x01, 0x24, 0x04, 0x00, 0xE5}, 0x10},
      };
      for (auto &feature : features) {
        assert(!feature.sw->state);
        auto expect_command = [&](bool on) {
          auto expected = on ? feature.on : feature.off;
          expected[1] = remote;
          expected.back() = dn_checksum(expected.data(), expected.size() - 1);
          assert(unit.write_queue_.size() == 1 && unit.commands_empty());
          const auto command = unit.write_queue_.front();
          unit.write_queue_.pop();
          assert(std::equal(expected.begin(), expected.end(), command.raw));
        };
        feature.sw->turn_on();
        expect_command(true);  // No accompanying heating-On or setpoint command.
        assert(!feature.sw->state && zone.mode == climate::CLIMATE_MODE_OFF);
        status.raw[7] = feature.mask;
        receive();
        assert(feature.sw->state);
        feature.sw->turn_off();
        expect_command(false);
        assert(feature.sw->state);  // Wait for master feedback, not optimistic state.
        status.raw[7] = 0;
        receive();
        assert(!feature.sw->state);
      }
      unit.set_read_only(true);
      for (auto &feature : features) {
        feature.sw->turn_on();
        feature.sw->turn_off();
        assert(!feature.sw->state);
      }
      assert(unit.write_queue_.empty() && unit.commands_empty());
    }
  }
  // Preserve both existing A0 routes. Silent Mode has no A0 implementation.
  TestEstia a0;
  a0.set_frame_format(FrameFormat::A0);
  ToshibaAbEstiaFrostProtectionSwitch frost(&a0);
  ToshibaAbEstiaNightSetbackSwitch night(&a0);
  ToshibaAbEstiaSilentModeSwitch silent(&a0);
  switch_::Switch *switches[] = {&frost, &night};
  for (size_t index = 0; index < 2; index++) {
    for (bool on : {true, false}) {
      if (on) switches[index]->turn_on(); else switches[index]->turn_off();
      const auto command = a0.pop_command();
      assert(command[9] == 0x03 && command[10] == 0xC4);
      assert(command[11] == (index == 0 ? 0x02 : 0x88));
      assert(command[12] == (on ? (index == 0 ? 0x02 : 0x08) : 0x00));
      assert(a0.commands_empty() && a0.write_queue_.empty());
    }
  }
  silent.turn_on();
  silent.turn_off();
  assert(a0.commands_empty() && a0.write_queue_.empty());
  a0.set_read_only(true);
  frost.turn_on();
  frost.turn_off();
  night.turn_on();
  night.turn_off();
  assert(a0.commands_empty() && a0.write_queue_.empty());
}

static void first_gen_indicator_checks() {
  struct Case {
    uint8_t operation;
    uint8_t features;
    uint8_t activity;
    uint16_t expected;
  };
  // Controller-icon replay cases; expected bits follow the sensor order below.
  const Case cases[] = {
      {0x20, 0x00, 0x00, 0x000},
      {0x20, 0x00, 0x10, 0x001},  // water pump, including run-on with operation Off
      {0x20, 0x00, 0x02, 0x002},  // heating/cooling compressor
      {0x20, 0x00, 0x01, 0x004},  // hydro-unit resistor
      {0x20, 0x00, 0x08, 0x008},  // DHW compressor
      {0x20, 0x00, 0x04, 0x010},  // DHW cylinder resistor
      {0x41, 0x00, 0x80, 0x020},  // external boiler/booster
      {0x41, 0x08, 0x00, 0x000},  // frost protection is reported by its existing switch
      {0x40, 0x08, 0x00, 0x000},  // frost status does not imply heating operation On
      {0x41, 0x10, 0x00, 0x000},  // silent mode is reported by its switch
      {0x41, 0x20, 0x00, 0x000},  // night setback is reported by its switch
      {0xA2, 0x40, 0x00, 0x000},  // DHW boost is reported by its existing switch
      {0xA2, 0x80, 0x00, 0x000},  // antibacterial programme is reported by its existing switch
      {0x45, 0x00, 0x00, 0x040},  // test-mode wrench and flashing Zone 1
      {0x40, 0x20, 0x00, 0x000},  // enabled setting survives operation Off
      {0xC0, 0x80, 0x00, 0x000},  // enabled setting is not an active DHW cycle
      {0x45, 0xF8, 0x9F, 0x07F},  // all known flags remain independent
      {0x59, 0x07, 0x60, 0x000},  // unknown flags and Auto are not new indicators
  };
  for (bool full_status : {false, true}) {
    RecordingUart uart;
    TestEstia unit;
    unit.set_uart_parent(&uart);
    unit.set_frame_format(FrameFormat::ESTIA);
    unit.set_master_address(0x70);
    unit.set_read_only(true);
    std::array<binary_sensor::BinarySensor, 7> sensors;
    ToshibaAbEstiaDhwBoostSwitch dhw_boost(&unit);
    ToshibaAbEstiaAntibacteriaSwitch antibacteria(&unit);
    ToshibaAbEstiaFrostProtectionSwitch frost_protection(&unit);
    ToshibaAbEstiaNightSetbackSwitch night_setback(&unit);
    ToshibaAbEstiaSilentModeSwitch silent_mode(&unit);
    const std::array<switch_::Switch *, 5> switches{
        &dhw_boost, &antibacteria, &frost_protection, &night_setback, &silent_mode};
    const std::array<uint8_t, 5> switch_masks{0x40, 0x80, 0x08, 0x20, 0x10};
    std::array<optional<bool>, 5> reported_switch_states{};
    for (size_t index = 0; index < switches.size(); index++) {
      switches[index]->add_on_state_callback(
          [&, index](bool state) { reported_switch_states[index] = state; });
    }
    unit.set_dhw_boost_switch(&dhw_boost);
    unit.set_antibacteria_switch(&antibacteria);
    unit.set_frost_protection_switch(&frost_protection);
    unit.set_night_setback_switch(&night_setback);
    unit.set_silent_mode_switch(&silent_mode);
    unit.set_waterpump_active_binary_sensor(&sensors[0]);
    unit.set_heating_cooling_compressor_active_binary_sensor(&sensors[1]);
    unit.set_heating_resistor_active_binary_sensor(&sensors[2]);
    unit.set_dhw_compressor_active_binary_sensor(&sensors[3]);
    unit.set_dhw_resistor_active_binary_sensor(&sensors[4]);
    unit.set_heating_external_booster_active_binary_sensor(&sensors[5]);
    unit.set_test_mode_active_binary_sensor(&sensors[6]);
    for (const auto &sensor : sensors) assert(!sensor.has_state());
    for (const auto &state : reported_switch_states) assert(!state.has_value());
    for (const auto &test : cases) {
      auto status = first_gen_status(test.operation, false, test.activity);
      status.raw[7] = test.features;
      const size_t checksum_index = full_status ? 17 : 13;
      if (!full_status) {
        status.raw[0] = 0x11;
        status.raw[4] = 0xC0;
      }
      status.raw[checksum_index] = dn_checksum(status.raw, checksum_index);
      assert(unit.receive_data_frame(&status));
      uint16_t actual = 0;
      for (size_t index = 0; index < sensors.size(); index++) {
        assert(sensors[index].has_state());
        if (sensors[index].state) actual |= 1U << index;
      }
      assert(actual == test.expected);
      // ESPHome 2026.9.1 publishes callbacks without setting Switch::has_state().
      for (size_t index = 0; index < switches.size(); index++) {
        assert(reported_switch_states[index].has_value());
        assert(reported_switch_states[index].value() == bool(test.features & switch_masks[index]));
        assert(switches[index]->state == bool(test.features & switch_masks[index]));
      }
      status.raw[6] ^= 0x04;
      status.raw[7] ^= 0xF8;
      status.raw[8] ^= 0x9F;
      unit.receive_data_frame(&status);  // corrupted checksum
      status.raw[1] = 0x71;
      status.raw[checksum_index] = dn_checksum(status.raw, checksum_index);
      unit.receive_data_frame(&status);  // valid checksum, different source
      for (size_t index = 0; index < sensors.size(); index++)
        assert(sensors[index].state == bool(test.expected & (1U << index)));
      for (size_t index = 0; index < switches.size(); index++) {
        assert(reported_switch_states[index].value() == bool(test.features & switch_masks[index]));
        assert(switches[index]->state == bool(test.features & switch_masks[index]));
      }
    }
    assert(uart.transmitted == 0 && unit.commands_empty());
  }
}

static void first_gen_dhw_demand_checks() {
  RecordingUart uart;
  TestEstia unit;
  unit.set_uart_parent(&uart);
  unit.set_frame_format(FrameFormat::ESTIA);
  unit.set_master_address(0x70);
  unit.set_read_only(true);
  ToshibaAbEstiaZoneClimate zone(&unit);
  ToshibaAbEstiaZone1Switch enabled(&unit);
  binary_sensor::BinarySensor dhw_resistor, dhw_compressor;
  unit.set_zone1_climate(&zone);
  unit.set_zone1_switch(&enabled);
  // Exercise the retained C++ setter aliases against the canonical DHW indicators.
  unit.set_hotwater_resistor_heating_binary_sensor(&dhw_resistor);
  unit.set_hotwater_pump_heating_binary_sensor(&dhw_compressor);
  for (bool automatic : {false, true}) {
    for (bool dhw : {false, true}) {
      for (bool zone_on : {false, true}) {
        for (uint8_t activity : {0x00, 0x04, 0x08, 0x0C}) {
          auto status = first_gen_status((zone_on ? 1 : 0) | (dhw ? 2 : 0), automatic, activity);
          assert(unit.receive_data_frame(&status));
          assert(unit.mode == (dhw ? climate::CLIMATE_MODE_HEAT : climate::CLIMATE_MODE_OFF));
          assert(enabled.state == zone_on);
          assert(zone.mode == (!zone_on ? climate::CLIMATE_MODE_OFF
                              : automatic ? climate::CLIMATE_MODE_AUTO : climate::CLIMATE_MODE_HEAT));
          assert(dhw_resistor.has_state() && dhw_resistor.state == bool(activity & 0x04));
          assert(dhw_compressor.has_state() && dhw_compressor.state == bool(activity & 0x08));
          near(unit.target_temperature, 65); near(zone.target_temperature, 45);
        }
      }
    }
  }
  assert(uart.transmitted == 0 && unit.write_queue_.empty() && unit.commands_empty());
}

static DataFrame master_keepalive(FrameFormat format) {
  DataFrame f{};
  if (format == FrameFormat::A0) {
    f = frame(0x10, 0x008A, 9);
    f.raw[3] = 0x08;
    f.raw[6] = 0xFE;
    const auto crc = f.calculate_estia_crc();
    f.raw[9] = crc >> 8; f.raw[10] = crc;
  } else if (format == FrameFormat::TU2C || format == FrameFormat::ESTIA) {
    f.set_tu2c(true);
    const uint8_t bytes[] = {0x0A, uint8_t(format == FrameFormat::TU2C ? 0x90 : 0x70),
                             0xFF, 0xFF, 0x00, 0x3A, 0x00};
    std::copy(std::begin(bytes), std::end(bytes), f.raw);
    f.raw[6] = dn_checksum(f.raw, 6);
  } else if (format == FrameFormat::HM) {
    // Exercise the same conversion used by the HM wire reader. Its checksum
    // algorithm is unknown; receive_data_frame intentionally tolerates it.
    const uint8_t wire[] = {0xA0, 0x00, 0x10, 0x07, 0x00, 0x08, 0x00, 0x00, 0xFE, 0x00, 0x8A, 0x00};
    std::copy(std::begin(wire), std::end(wire), f.raw);
    assert(DataFrameReader::canonicalize_hm_frame(&f, sizeof(wire)));
  } else {
    f.source = 0x00; f.dest = 0xFE; f.opcode1 = OPCODE_PING; f.data_length = 2;
    f.data[0] = 0x80; f.data[1] = 0x8A;
    f.data[2] = f.calculate_crc();
  }
  return f;
}

static void master_keepalive_silence_checks() {
  TestEstia timing;
  assert(!timing.is_master_keepalive_silence_active_(5000));  // Nothing observed yet.
  for (uint32_t last : {0U, 10000U, UINT32_MAX - 2000U}) {
    timing.last_master_keepalive_ms_ = last;
    assert(!timing.is_master_keepalive_silence_active_(last + 4499));
    assert(timing.is_master_keepalive_silence_active_(last + 4500));
    assert(timing.is_master_keepalive_silence_active_(last + 5000));
    assert(timing.is_master_keepalive_silence_active_(last + 5300));  // Late keepalive.
    assert(timing.is_master_keepalive_silence_active_(last + 5999));
    assert(!timing.is_master_keepalive_silence_active_(last + 6000));
    assert(!timing.is_master_keepalive_silence_active_(last + 10000));
  }
  for (auto format : {FrameFormat::NORMAL, FrameFormat::HM, FrameFormat::TU2C, FrameFormat::ESTIA, FrameFormat::A0}) {
    RecordingUart uart;
    TestEstia unit;
    unit.set_uart_parent(&uart);
    unit.set_frame_format(format);
    const uint8_t master = format == FrameFormat::TU2C ? 0x90 : format == FrameFormat::ESTIA ? 0x70 : 0x00;
    unit.set_master_address(master);
    unit.set_autonomous(false);
    unit.boot_millis_ = millis() - INITIAL_FRAME_SEND_BLOCK_MILLIS;
    unit.announce_ack_received_ = true;
    const auto ping = master_keepalive(format);
    assert(unit.receive_data_frame(&ping));
    assert(unit.last_master_keepalive_ms_.has_value());
    // Ordinary master traffic refreshes connectivity, not the periodic phase.
    unit.last_master_keepalive_ms_ = 123456;
    auto other = ping;
    if (format == FrameFormat::A0) {
      other.raw[0] = 0x58;
      const auto crc = other.calculate_estia_crc();
      other.raw[9] = crc >> 8; other.raw[10] = crc;
    } else if (other.is_tu2c()) {
      other.raw[5] = 0x39;
      other.raw[6] = dn_checksum(other.raw, 6);
    } else {
      other.opcode1 = 0x7F;
      other.data[2] = other.calculate_crc();
    }
    assert(unit.receive_data_frame(&other));
    assert(*unit.last_master_keepalive_ms_ == 123456);
    other = ping;
    if (format == FrameFormat::A0) {
      other.raw[4] ^= 1;
      const auto crc = other.calculate_estia_crc();
      other.raw[9] = crc >> 8; other.raw[10] = crc;
    } else if (other.is_tu2c()) {
      other.raw[1] ^= 1;
      other.raw[6] = dn_checksum(other.raw, 6);
    } else {
      other.source ^= 1;
      other.data[2] = other.calculate_crc();
    }
    assert(unit.receive_data_frame(&other));
    assert(*unit.last_master_keepalive_ms_ == 123456);  // Another source cannot rephase us.
    if (format != FrameFormat::HM) {
      other = ping;
      const size_t size = other.is_estia() ? other.estia_size() : other.size();
      other.raw[size - 1] ^= 1;
      assert(!unit.receive_data_frame(&other));
      assert(*unit.last_master_keepalive_ms_ == 123456);
    }
    auto quiet_bus = [&]() {
      unit.last_received_frame_millis_ = millis() - 1000;
      unit.last_sent_frame_millis_ = millis() - 1000;
      unit.last_tu2c_received_frame_millis_ = millis() - 1000;
      unit.last_tu2c_sent_frame_millis_ = millis() - 1000;
    };
    DataFrame command{};
    command.data_length = 2;
    assert(unit.enqueue_command_(command));
    assert(unit.enqueue_raw_frame_({0xA0, 0x00}));
    quiet_bus();
    unit.last_master_keepalive_ms_ = millis() - 4500;
    unit.loop();
    assert(uart.transmitted == 0 && unit.write_queue_.size() == 1 && unit.raw_write_queue_.size() == 1);
    unit.last_master_keepalive_ms_ = millis() - 6000;
    unit.loop();
    assert(uart.transmitted == 2 && unit.raw_write_queue_.empty() && unit.write_queue_.size() == 1);
    assert(unit.enqueue_raw_frame_({0xA0, 0x00}));
    uart.transmitted = 0;
    assert(unit.receive_data_frame(&ping));
    assert(!unit.is_master_keepalive_silence_active_(millis()));
    unit.last_received_frame_millis_ = millis();
    unit.loop();
    assert(uart.transmitted == 0);  // Existing post-RX silence still applies.
    quiet_bus();
    unit.loop();
    assert(uart.transmitted == 2 && unit.raw_write_queue_.empty() && unit.write_queue_.size() == 1);
    quiet_bus();
    unit.loop();
    assert(uart.transmitted > 2 && unit.write_queue_.empty());  // One frame per TX slot.
    uart.transmitted = 0;
    assert(unit.enqueue_sensor_query_(0x02));
    quiet_bus();
    unit.last_master_keepalive_ms_ = millis() - 4500;
    unit.loop();
    assert(unit.pending_count_ == 1 && !unit.sensor_query_outstanding_ && uart.transmitted == 0);
    assert(unit.receive_data_frame(&ping));
    quiet_bus();
    unit.loop();
    assert(unit.pending_count_ == 0 && unit.sensor_query_outstanding_ && uart.transmitted > 0);
    unit.set_read_only(true); unit.set_read_only(false);  // Clear queues before retry checks.
    uart.transmitted = 0;
    quiet_bus();
    unit.last_master_keepalive_ms_ = millis() - 4500;
    if (format == FrameFormat::A0) {
      unit.estia_pending_cmd_ = {0xA0, 0x00};
      unit.estia_cmd_attempts_ = 1;
      unit.estia_cmd_sent_ms_ = millis() - 4000;
      unit.loop();
      assert(unit.estia_cmd_attempts_ == 1 && unit.raw_write_queue_.empty() && uart.transmitted == 0);
      assert(unit.receive_data_frame(&ping));
      quiet_bus(); unit.loop();
      assert(unit.estia_cmd_attempts_ == 2 && unit.raw_write_queue_.size() == 1);
      unit.loop();
      assert(uart.transmitted == 2);
    } else {
      unit.last_unconfirmed_command_ = command;
      unit.last_unconfirmed_command_attempts_ = 1;
      unit.last_unconfirmed_command_sent_ms_ = millis() - 1000;
      unit.loop();
      assert(unit.last_unconfirmed_command_attempts_ == 1 && uart.transmitted == 0);
      assert(unit.receive_data_frame(&ping));
      quiet_bus(); unit.loop();
      assert(unit.last_unconfirmed_command_attempts_ == 2 && uart.transmitted > 0);
    }
    unit.set_master_address(master);
    assert(!unit.last_master_keepalive_ms_.has_value());
    assert(unit.receive_data_frame(&ping));
    unit.set_frame_format(format);
    assert(!unit.last_master_keepalive_ms_.has_value());
  }
}

static void first_gen_fault_checks() {
  for (bool read_only : {false, true}) {
    RecordingUart uart;
    TestEstia unit;
    unit.set_uart_parent(&uart);
    unit.set_frame_format(FrameFormat::ESTIA);
    unit.set_remote_address(0x61);
    unit.set_read_only(read_only);
    text_sensor::TextSensor code;
    binary_sensor::BinarySensor active, remote_error;
    unit.set_fault_code_text_sensor(&code);
    unit.set_fault_active_binary_sensor(&active);
    unit.set_remote_error_binary_sensor(&remote_error);
    unit.setup();
    assert(unit.last_master_alive_millis_ == 0);  // Boot default is not a received report.
    unit.boot_millis_ = millis() - INITIAL_FRAME_SEND_BLOCK_MILLIS;
    unit.loop();
    assert(code.has_state() && code.state == "None");
    assert(active.has_state() && !active.state);  // No report is needed at boot.
    unsigned publications = 0;
    code.add_on_state_callback([&](const std::string &) { ++publications; });
    auto expect = [&](const char *value, bool fault) {
      assert(code.has_state() && code.state == value);
      assert(active.has_state() && active.state == fault);
    };
    // Actual Series 5 captures: announcements, responses to a wall controller,
    // and unsolicited all-clear announcements. All use the real CRC receive path.
    const uint8_t captures[][9] = {
        {0x0C, 0x70, 0xFF, 0xE0, 0xC0, 0x29, 0x70, 0x01, 0xB5},
        {0x0C, 0x70, 0xFF, 0xE0, 0xA0, 0x29, 0x70, 0x01, 0x95},
        {0x0C, 0x70, 0xFF, 0xE0, 0xC0, 0x29, 0x70, 0x00, 0xB4},
        {0x0C, 0x70, 0xFF, 0xE0, 0xC0, 0x29, 0x70, 0x77, 0x2B},
        {0x0C, 0x70, 0xFF, 0xE0, 0xA0, 0x29, 0x70, 0x77, 0x0B},
        {0x0C, 0x70, 0xFF, 0xE0, 0xA0, 0x29, 0x70, 0x00, 0x94},
        {0x0C, 0x70, 0xFF, 0xE0, 0xA0, 0x29, 0x70, 0x49, 0xDD},
    };
    const char *expected[] = {"A01", "A01", "None", "F23", "F23", "None", "E09"};
    for (size_t i = 0; i < 7; ++i) {
      DataFrame f{};
      f.set_tu2c(true);
      std::copy(std::begin(captures[i]), std::end(captures[i]), f.raw);
      assert(unit.receive_data_frame(&f));
      expect(expected[i], captures[i][7] != 0);
      unit.loop();
      assert(uart.transmitted == 0 && unit.write_queue_.empty() && unit.commands_empty());
    }
    const auto previous_publications = publications;
    DataFrame report{};
    report.set_tu2c(true);
    std::copy(std::begin(captures[6]), std::end(captures[6]), report.raw);
    assert(unit.receive_data_frame(&report));
    assert(publications == previous_publications);  // Suppress duplicate updates.
    auto bad = report;
    bad.raw[7] = 0;
    assert(!unit.receive_data_frame(&bad));
    expect("E09", true);
    for (unsigned changed_byte : {1, 2, 3, 4, 5}) {
      bad = report;
      bad.raw[7] = 0;
      bad.raw[changed_byte] ^= 1;
      bad.raw[8] = dn_checksum(bad.raw, 8);
      assert(unit.receive_data_frame(&bad));
      expect("E09", true);
    }
    bad = report;
    bad.raw[0] = 0x0B;
    bad.raw[7] = dn_checksum(bad.raw, 7);
    assert(unit.receive_data_frame(&bad));  // Missing code is not an all-clear.
    expect("E09", true);
    auto status = first_gen_status(0x40, false, 0);
    assert(unit.receive_data_frame(&status));
    expect("E09", true);  // Ordinary status cannot clear the separate fault state.
    DataFrame remote{};
    remote.set_tu2c(true);
    const uint8_t remote_bytes[] = {0x0B, 0x60, 0x70, 0xE0, 0x61, 0x31, 0x00, 0x4D};
    std::copy(std::begin(remote_bytes), std::end(remote_bytes), remote.raw);
    remote.raw[6] = 0x49;
    remote.raw[7] = dn_checksum(remote.raw, 7);
    assert(unit.receive_data_frame(&remote));
    assert(remote_error.has_state() && remote_error.state);
    remote.raw[6] = 0;
    remote.raw[7] = dn_checksum(remote.raw, 7);
    assert(unit.receive_data_frame(&remote));
    assert(remote_error.has_state() && !remote_error.state);
    expect("E09", true);  // Remote-controller status is independent.
    report.raw[7] = 0x34;
    report.raw[8] = dn_checksum(report.raw, 8);
    assert(unit.receive_data_frame(&report));
    expect("0x34", true);
    unit.last_master_alive_millis_ = millis() - LAST_ALIVE_TIMEOUT_MILLIS - 1;
    unit.loop(); unit.loop();
    assert(!code.has_state() && !active.has_state());
    assert(unit.receive_data_frame(&status));
    assert(!code.has_state() && !active.has_state());  // Reconnection is not a clear.
    assert(unit.receive_data_frame(&report));
    expect("0x34", true);  // Identical fault must republish after a timeout.
    report.raw[7] = 0;
    report.raw[8] = dn_checksum(report.raw, 8);
    assert(unit.receive_data_frame(&report));
    expect("None", false);
    assert(uart.transmitted == 0 && unit.write_queue_.empty() && unit.commands_empty());
  }
  // Honor an explicitly configured master instead of assuming address 0x70.
  TestEstia configured;
  configured.set_frame_format(FrameFormat::ESTIA);
  configured.set_master_address(0x71);
  text_sensor::TextSensor code;
  configured.set_fault_code_text_sensor(&code);
  DataFrame report{};
  report.set_tu2c(true);
  const uint8_t bytes[] = {0x0C, 0x71, 0xFF, 0xE0, 0xC0, 0x29, 0x70, 0x77, 0x00};
  std::copy(std::begin(bytes), std::end(bytes), report.raw);
  report.raw[8] = dn_checksum(report.raw, 8);
  assert(configured.receive_data_frame(&report));
  assert(code.has_state() && code.state == "F23");
}

static void read_only_transmission_checks() {
  for (auto format : {FrameFormat::A0, FrameFormat::NORMAL, FrameFormat::HM,
                      FrameFormat::TU2C, FrameFormat::ESTIA}) {
    RecordingUart uart;
    TestEstia unit;
    unit.set_uart_parent(&uart);
    unit.set_frame_format(format);
    unit.boot_millis_ = millis() - INITIAL_FRAME_SEND_BLOCK_MILLIS;
    unit.last_received_frame_millis_ = millis() - 1000;
    unit.last_sent_frame_millis_ = millis() - 1000;
    unit.set_autonomous(true);
    unit.demand_enabled_ = true;
    unit.estia_last_e8c0_ms_ = millis() - 30000;
    unit.estia_last_e8c1_ms_ = millis() - 60000;
    unit.estia_last_demand_heartbeat_ms_ = millis() - 5000;
    DataFrame command{};
    command.data_length = 2;
    assert(unit.enqueue_command_(command));
    assert(unit.enqueue_raw_frame_({0xA0, 0x00}));
    assert(unit.enqueue_sensor_query_(0x02));
    unit.last_unconfirmed_command_ = command;
    unit.estia_pending_cmd_ = {0xA0, 0x00};
    unit.estia_cmd_attempts_ = 1;
    unit.set_read_only(true);
    assert(unit.write_queue_.empty() && unit.commands_empty());
    assert(unit.pending_count_ == 0);
    assert(!unit.last_unconfirmed_command_.has_value());
    assert(unit.estia_pending_cmd_.empty());
    assert(!unit.enqueue_command_(command));
    assert(!unit.enqueue_sensor_query_(0x02));
    assert(!unit.send_raw_frame_from_text("A0:00"));
    unit.send_estia_data_request(0xC0);
    unit.send_estia_data_request(0xC1);
    unit.send_estia_demand_heartbeat();
    const uint8_t tracked[] = {0xA0, 0x00};
    unit.send_estia_tracked_(tracked, sizeof(tracked), 0x0041);
    assert(unit.commands_empty() && unit.estia_pending_cmd_.empty());
    unit.loop();
    assert(uart.transmitted == 0);
    assert(unit.commands_empty());
    // Even an already-populated queue cannot bypass the final TX guard.
    unit.raw_write_queue_.push({0xA0, 0x00});
    unit.write_queue_.push(command);
    unit.loop();
    assert(uart.transmitted == 0);
    unit.set_read_only(true);
    unit.set_read_only(false);
    unit.set_autonomous(false);
    unit.demand_enabled_ = false;
    unit.loop();
    assert(uart.transmitted == 0);
    assert(unit.send_raw_frame_from_text("A0:00"));
    unit.loop();
    assert(uart.transmitted == 2);
  }
}

static void command_retry_checks() {
  for (auto format : {FrameFormat::NORMAL, FrameFormat::HM, FrameFormat::TU2C, FrameFormat::ESTIA}) {
    for (uint8_t power : {0, 1}) {
      RecordingUart uart;
      TestEstia unit;
      unit.set_uart_parent(&uart);
      unit.set_frame_format(format);
      unit.set_autonomous(false);
      unit.boot_millis_ = millis() - INITIAL_FRAME_SEND_BLOCK_MILLIS;
      unit.tcc_state.power = !power;
      unit.tcc_state.mode = MODE_HEAT;
      unit.tcc_state.target_temp = 20;
      auto requested = unit.tcc_state;
      requested.power = power;
      auto commands = unit.create_commands(&requested);
      if (format == FrameFormat::ESTIA) {
        if (power) unit.send_estia_first_gen_dhw_on();
        else unit.send_estia_first_gen_dhw_off();
        assert(unit.write_queue_.size() == 1);
        commands = {unit.write_queue_.front()};
        unit.write_queue_.pop();
      }
      assert(commands.size() == 1 && unit.should_track_command_ack_(commands[0]));
      const auto command = commands[0];
      auto quiet = [&]() {
        unit.last_received_frame_millis_ = millis() - 1000;
        unit.last_sent_frame_millis_ = millis() - 1000;
        unit.last_tu2c_received_frame_millis_ = millis() - 1000;
        unit.last_tu2c_sent_frame_millis_ = millis() - 1000;
      };
      auto start = [&]() {
        quiet();
        assert(unit.enqueue_command_(command));
        unit.loop();
        assert(unit.last_unconfirmed_command_.has_value());
        assert(unit.last_unconfirmed_command_attempts_ == 1);
      };
      start();
      const auto wire_command = uart.writes.back();
      const size_t first_write = uart.writes.size();
      // Timeout starts at zero too, and subtraction remains valid across rollover.
      unit.last_unconfirmed_command_sent_ms_ = 0;
      assert(!unit.command_ack_timeout_elapsed_(199));
      assert(unit.command_ack_timeout_elapsed_(200));
      unit.last_unconfirmed_command_sent_ms_ = UINT32_MAX - 99;
      assert(!unit.command_ack_timeout_elapsed_(99));
      assert(unit.command_ack_timeout_elapsed_(100));

      auto seal = [](DataFrame &frame) {
        const size_t n = frame.size();
        if (frame.is_tu2c()) {
          uint8_t sum = 0;
          for (size_t i = 0; i + 1 < n; ++i) sum += frame.raw[i];
          frame.raw[n - 1] = sum;
        } else {
          frame.raw[n - 1] = frame.calculate_crc();
        }
      };
      DataFrame ack{};
      if (command.is_tu2c()) {
        ack.set_tu2c(true);
        const uint8_t bytes[] = {0x0A, command.raw[2], command.raw[1], command.raw[3], 0x80, 0xA1, 0};
        std::copy(std::begin(bytes), std::end(bytes), ack.raw);
      } else {
        ack.source = command.dest;
        ack.dest = command.source;
        ack.opcode1 = OPCODE_ACK;
        ack.data_length = 2;
        ack.data[0] = 0x80;
        ack.data[1] = OPCODE2_PARAM_ACK;
      }
      seal(ack);
      auto ignored = [&](DataFrame other, bool valid = true) {
        unit.last_unconfirmed_command_sent_ms_ = millis();
        const auto sent_ms = unit.last_unconfirmed_command_sent_ms_;
        assert(unit.receive_data_frame(&other) == valid);
        assert(unit.last_unconfirmed_command_.has_value());
        assert(unit.last_unconfirmed_command_sent_ms_ == sent_ms);
        quiet();
        unit.loop();
        assert(uart.writes.size() == first_write);
      };
      // Valid unrelated ACKs, wrong addresses, status/echo traffic and corrupt ACKs
      // neither complete a command nor trigger an immediate resend.
      auto other = ack;
      other.raw[5] = 0x0C;
      seal(other); ignored(other);
      other = ack; other.raw[command.is_tu2c() ? 1 : 0] ^= 1;
      seal(other); ignored(other);
      other = ack; other.raw[command.is_tu2c() ? 2 : 1] ^= 1;
      seal(other); ignored(other);
      other = ack;
      if (command.is_tu2c()) other.raw[3] ^= 0x20;
      else other.opcode1 = OPCODE_PING;
      seal(other); ignored(other);
      unit.handle_pending_command_ack_(&command);
      assert(unit.last_unconfirmed_command_.has_value());
      other = ack; other.raw[other.size() - 1] ^= 1;
      if (format != FrameFormat::HM) ignored(other, false);

      // Silent bus: five identical sends total, each with a fresh timeout.
      for (unsigned attempt = 2; attempt <= 5; ++attempt) {
        quiet();
        unit.last_unconfirmed_command_sent_ms_ = millis() - 201;
        unit.loop();
        assert(unit.last_unconfirmed_command_attempts_ == attempt);
        assert(uart.writes.size() == first_write + attempt - 1);
        assert(uart.writes.back() == wire_command);
        assert(!unit.command_ack_timeout_elapsed_(unit.last_unconfirmed_command_sent_ms_ + 199));
      }
      quiet();
      unit.last_unconfirmed_command_sent_ms_ = millis() - 201;
      unit.loop();
      assert(!unit.last_unconfirmed_command_.has_value());
      assert(uart.writes.size() == first_write + 4);

      // A buffered valid ACK wins over an expired timer; subsequent queued
      // commands/raw requests must wait for that ACK, not a telemetry response.
      start();
      const size_t before_ack = uart.writes.size();
      assert(unit.enqueue_command_(command));
      assert(unit.enqueue_raw_frame_({0xA0, 0x00}));
      quiet();
      unit.last_unconfirmed_command_sent_ms_ = millis();
      unit.loop();
      assert(uart.writes.size() == before_ack && !unit.raw_write_queue_.empty());
      assert(unit.write_queue_.size() == 1);
      uart.rx.assign(ack.raw, ack.raw + ack.size());
      if (command.is_tu2c()) {
        uart.rx.insert(uart.rx.begin(), {0xF0, 0xF0});
        uart.rx.push_back(0xA0);
      }
      unit.can_read_packet = true;
      unit.last_unconfirmed_command_sent_ms_ = millis() - 201;
      quiet();
      unit.loop();
      assert(!unit.last_unconfirmed_command_.has_value());
      assert(uart.writes.size() == before_ack);
      assert(unit.write_queue_.size() == 1);
      unit.raw_write_queue_.pop();
      quiet();
      unit.loop();
      assert(uart.writes.size() == before_ack + 1);
      assert(unit.last_unconfirmed_command_attempts_ == 1);
      assert(unit.write_queue_.empty());
      assert(unit.receive_data_frame(&ack));
      quiet();
      unit.last_unconfirmed_command_sent_ms_ = millis() - 201;
      unit.loop();
      assert(uart.writes.size() == before_ack + 1);
    }
  }
}

static void tu2c_fan_level_checks() {
  const uint8_t status_fans[] = {0x40, 0xA0, 0xB0, 0x80, 0x90, 0x60};
  const uint8_t command_fans[] = {0x24, 0x2A, 0x2B, 0x28, 0x29, 0x26};
  const climate::ClimateFanMode standard[] = {
      climate::CLIMATE_FAN_AUTO, climate::CLIMATE_FAN_LOW, climate::CLIMATE_FAN_LOW,
      climate::CLIMATE_FAN_MEDIUM, climate::CLIMATE_FAN_MEDIUM, climate::CLIMATE_FAN_HIGH};
  for (uint8_t levels : {3, 5}) {
    TestEstia unit;
    unit.set_frame_format(FrameFormat::TU2C);
    if (levels == 3) unit.set_fan_levels(3);  // Five is the TU2C default.
    unit.announce_ack_received_ = true;
    unit.boot_millis_ = millis() - INITIAL_FRAME_SEND_BLOCK_MILLIS;
    assert(unit.get_traits().get_supported_custom_fan_modes().size() == (levels == 5 ? 2 : 0));
    auto pop_fan = [&](uint8_t expected) {
      assert(unit.write_queue_.size() == 1);
      auto command = unit.write_queue_.front(); unit.write_queue_.pop();
      assert(command.is_tu2c() && command.raw[7] == expected);
      assert(command.raw[command.size() - 1] == dn_checksum(command.raw, command.size() - 1));
    };
    for (uint8_t group : {0xC0, 0xA0}) {
      for (size_t level = 0; level < 6; ++level) {
        DataFrame status{};
        status.set_tu2c(true);
        const uint8_t bytes[] = {0x12, 0x90, 0xFF, 0xC0, group, 0x38, 0x41, status_fans[level],
                                 0x00, 0x00, 120, 110, 0, 0, 0};
        std::copy(std::begin(bytes), std::end(bytes), status.raw);
        status.raw[14] = dn_checksum(status.raw, 14);
        assert(unit.receive_data_frame(&status));
        const bool half = level == 2 || level == 4;
        assert(unit.tcc_state.fan_half == half);
        if (levels == 5 && half) {
          assert(unit.has_custom_fan_mode());
          assert(unit.get_custom_fan_mode() == (level == 2 ? TU2C_FAN_LOW_MEDIUM : TU2C_FAN_MEDIUM_HIGH));
        } else {
          assert(!unit.has_custom_fan_mode() && unit.fan_mode == standard[level]);
        }
        if (!half || levels == 5) {
          // Exercise all selectable levels through the public climate call path.
          unit.tcc_state.fan = level == 5 ? FAN_PEED_LOW : FAN_PEED_HIGH;
          unit.tcc_state.fan_half = false;
          if (half) unit.make_call().set_fan_mode(level == 2 ? TU2C_FAN_LOW_MEDIUM : TU2C_FAN_MEDIUM_HIGH).perform();
          else unit.make_call().set_fan_mode(standard[level]).perform();
          pop_fan(command_fans[level]);
          assert(unit.receive_data_frame(&status));
        }
        // Temperature changes must preserve the full fan byte from the remote,
        // including hidden half steps when only three levels are exposed.
        unit.make_call().set_target_temperature(26).perform();
        pop_fan(command_fans[level]);
        unit.make_call().set_mode(climate::CLIMATE_MODE_HEAT).perform();
        assert(unit.write_queue_.size() == 1);
        auto mode_command = unit.write_queue_.front(); unit.write_queue_.pop();
        assert(mode_command.raw[5] == 0x22);  // Mode-only commands carry no fan byte.
        assert(unit.tcc_state.fan_half == half);
        // A standard selection clears the half-step bit even if its base speed matches.
        if (half) {
          unit.make_call().set_fan_mode(standard[level]).perform();
          pop_fan(command_fans[level] & ~1);
        }
      }
    }
    if (levels == 5) {
      for (const char *mode : {TU2C_FAN_LOW_MEDIUM, TU2C_FAN_MEDIUM_HIGH}) {
        unit.make_call().set_fan_mode(mode).perform();
        pop_fan(mode == TU2C_FAN_LOW_MEDIUM ? 0x2B : 0x29);
      }
      unit.set_read_only(true);
      unit.make_call().set_fan_mode(TU2C_FAN_LOW_MEDIUM).perform();
      assert(unit.write_queue_.empty());
      unit.set_read_only(false);
      unit.boot_millis_ = millis();
      unit.make_call().set_fan_mode(TU2C_FAN_LOW_MEDIUM).perform();
      assert(unit.write_queue_.empty());
    }
  }
  for (auto format : {FrameFormat::NORMAL, FrameFormat::HM, FrameFormat::A0, FrameFormat::ESTIA}) {
    TestEstia unit;
    unit.set_frame_format(format);
    assert(unit.get_traits().get_supported_custom_fan_modes().empty());
    unit.tcc_state.power = 1;
    unit.tcc_state.mode = MODE_COOL;
    unit.tcc_state.fan = FAN_PEED_LOW;
    unit.tcc_state.fan_half = true;
    unit.tcc_state.target_temp = 25;
    unit.sync_from_received_state();
    assert(!unit.has_custom_fan_mode());
    unit.make_call().set_fan_mode(TU2C_FAN_LOW_MEDIUM).perform();
    assert(unit.write_queue_.empty() && unit.commands_empty());
  }
}

static void zone_mode_capability_checks() {
  for (auto format : {FrameFormat::ESTIA, FrameFormat::A0}) {
    for (uint8_t zone_number : {1, 2}) {
      for (uint8_t modes = 0; modes < 8; modes++) {
        TestEstia unit;
        unit.boot_millis_ = millis() - INITIAL_FRAME_SEND_BLOCK_MILLIS;
        unit.set_frame_format(format);
        ToshibaAbEstiaZoneClimate zone(&unit, zone_number);
        if (zone_number == 1) unit.set_zone1_climate(&zone);
        else unit.set_zone2_climate(&zone);
        assert(zone.traits().supports_mode(climate::CLIMATE_MODE_HEAT));
        assert(zone.traits().supports_mode(climate::CLIMATE_MODE_COOL));
        assert(zone.traits().supports_mode(climate::CLIMATE_MODE_AUTO));
        zone.set_supports_heat(modes & 1);
        zone.set_supports_cool(modes & 2);
        zone.set_supports_auto(modes & 4);
        const auto traits = zone.traits();
        assert(traits.supports_mode(climate::CLIMATE_MODE_OFF));
        assert(traits.supports_mode(climate::CLIMATE_MODE_HEAT) == bool(modes & 1));
        assert(traits.supports_mode(climate::CLIMATE_MODE_COOL) == bool(modes & 2));
        assert(traits.supports_mode(climate::CLIMATE_MODE_AUTO) == bool(modes & 4));
        for (auto mode : {climate::CLIMATE_MODE_HEAT, climate::CLIMATE_MODE_COOL, climate::CLIMATE_MODE_AUTO}) {
          if (!traits.supports_mode(mode)) zone.make_call().set_mode(mode).perform();
        }
        assert(unit.write_queue_.empty() && unit.raw_write_queue_.empty());
      }
    }
  }
}

int main() {
  zone_mode_capability_checks();
  new (&App) Application();
  host::setup_preferences();
  master_keepalive_silence_checks();
  first_gen_fault_checks();
  first_gen_cooling_setpoint_checks();
  first_gen_dhw_mode_checks();
  first_gen_temperature_checks();
  dn_checks();
  new (&App) Application();
  first_gen_dhw_demand_checks();
  first_gen_activity_checks();
  first_gen_indicator_checks();
  first_gen_feature_control_checks();
  control_checks();
  tu2c_address_checks();
  startup_address_checks();
  intake_air_temperature_checks();
  regular_temperature_checks();
  first_gen_keepalive_checks();
  tu2c_fan_level_checks();
  read_only_transmission_checks();
  command_retry_checks();
  TestEstia unit;
  unit.set_frame_format(FrameFormat::A0);
  ToshibaAbEstiaZoneClimate z1(&unit), z2(&unit, 2);
  unit.set_zone1_climate(&z1);
  unit.set_zone2_climate(&z2);
  const auto traits1 = z1.traits();
  const auto traits2 = z2.traits();
  assert(traits1.get_visual_min_temperature() == 7);
  assert(traits1.get_visual_max_temperature() == 65);
  assert(traits2.get_visual_max_temperature() == 65);
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
  // STATUS and event frames use identical shared mode handling and independent targets.
  for (uint8_t type : {0x58, 0x1C}) {
    auto state = frame(type, 0x03C6, 18);
    state.raw[9] = 0x21;  // heating/cooling operation on, cooling selected
    state.raw[12] = 132; state.raw[13] = 68; state.raw[14] = 82;
    auto receive = [&]() {
      const auto crc = state.calculate_estia_crc();
      state.raw[18] = crc >> 8; state.raw[19] = crc;
      assert(unit.receive_data_frame(&state));
    };
    receive();
    assert(z1.mode == climate::CLIMATE_MODE_COOL && z2.mode == z1.mode);
    near(z1.target_temperature, 18); near(z2.target_temperature, 25);
    state.raw[10] = 4;
    receive();
    assert(z1.mode == climate::CLIMATE_MODE_AUTO && z2.mode == z1.mode);
    assert(std::isnan(z1.target_temperature) && std::isnan(z2.target_temperature));
    state.raw[9] = 0; state.raw[10] = 0;
    receive();
    assert(z1.mode == climate::CLIMATE_MODE_OFF && z2.mode == z1.mode);
  }
  // Full receive path validates CRC before publishing the decoded values.
  const size_t crc_offset = c0.estia_size() - 2;
  const uint16_t crc = c0.calculate_estia_crc();
  c0.raw[crc_offset] = crc >> 8;
  c0.raw[crc_offset + 1] = crc;
  assert(unit.receive_data_frame(&c0));
  near(to.state, 31);

  std::cout << "Estia telemetry checks passed\n";
}
