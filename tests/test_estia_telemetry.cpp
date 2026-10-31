#include "esphome/components/toshiba_ab/toshiba_ab.h"
#include "esphome/core/application.h"
#include <cassert>
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

int main() {
  first_gen_temperature_checks();
  dn_checks();
  new (&App) Application();
  control_checks();
  tu2c_address_checks();
  startup_address_checks();
  intake_air_temperature_checks();
  regular_temperature_checks();
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
