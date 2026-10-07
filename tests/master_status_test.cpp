#include "toshiba_ab.h"
#include <cassert>
#include <iostream>

uint32_t test_now = 1;
std::vector<std::string> debug_logs;
using namespace esphome::toshiba_ab;

class Bus : public ToshibaAbClimate {
 public:
  esphome::text_sensor::TextSensor sensor;
  Bus(Protocol protocol, SystemType type) {
    set_protocol(protocol);
    set_system_type(type);
    set_diagnostic_sensor(&sensor);
    setup();
  }
  static std::vector<uint8_t> frame(Protocol protocol, uint8_t opcode, uint16_t type,
                                  uint8_t length, uint8_t source = 0, uint8_t destination = 0xFF,
                                  uint8_t marker = 0xC0) {
    std::vector<uint8_t> data(length + (protocol == Protocol::TCC ? 5 : protocol == Protocol::A0 ? 6 : 0), 0);
    if (protocol == Protocol::TCC) {
      data[0] = source; data[1] = destination; data[2] = opcode; data[3] = length; data[5] = type;
      for (size_t i = 0; i + 1 < data.size(); i++) data.back() ^= data[i];
    } else if (protocol == Protocol::TU2C) {
      data[0] = data[1] = 0xF0; data[2] = length; data[3] = source;
      data[4] = destination; data[5] = marker; data[6] = opcode; data[7] = type;
      for (size_t i = 2; i + 2 < data.size(); i++) data[data.size() - 2] += data[i];
      data.back() = 0xA0;
    } else {
      data[0] = 0xA0; data[2] = opcode; data[3] = length; data[5] = 8;
      data[6] = source; data[8] = destination; data[9] = type >> 8; data[10] = type & 0xFF;
      const uint16_t crc = crc16_mcrf4xx_(data.data(), data.size() - 2);
      data[data.size() - 2] = crc >> 8; data.back() = crc & 0xFF;
    }
    return data;
  }
  void feed(const std::vector<uint8_t> &data) {
    reset_readers_();  // Separate fixtures; a bad TCC CRC can leave a resync candidate.
    debug_logs.clear();
    for (auto byte : data) read_byte_(byte);
  }
  void confirm(Protocol protocol) {
    feed(frame(protocol, protocol == Protocol::TU2C ? 0 : 0x10,
               protocol == Protocol::TU2C ? 0x3A : 0x8A,
               protocol == Protocol::TCC ? 2 : protocol == Protocol::TU2C ? 10 : 7));
    assert(master_address_confirmed_);
  }
  bool label(const std::string &label) const {
    for (const auto &line : debug_logs)
      if (line.find("[" + label + " 00]") != std::string::npos) return true;
    return false;
  }
  bool recognizes(Protocol protocol, const uint8_t *data, size_t size, bool extended) const {
    uint8_t source = 0;
    return is_master_status_(protocol, data, size, extended, source);
  }
  bool unknown_status() const { return !label("master status") && !label("master extended status"); }
  size_t remotes() const { return remotes_.size(); }
};

struct Fixture {
  Protocol protocol;
  SystemType system;
  uint8_t status_opcode, extended_opcode;
  uint16_t type;
  uint8_t minimum, extended_minimum, marker;
};

int main() {
  const Fixture fixtures[] = {
      {Protocol::TCC, SystemType::AIR, 0x1C, 0x58, 0x81, 7, 7, 0},
      {Protocol::A0, SystemType::AIR, 0x1C, 0x58, 0x0081, 12, 12, 0},
      {Protocol::TU2C, SystemType::AIR, 0xC0, 0xA0, 0x38, 15, 15, 0xC0},
      {Protocol::A0, SystemType::WATER, 0x1C, 0x58, 0x03C6, 15, 15, 0},
      {Protocol::TU2C, SystemType::WATER, 0xC0, 0xC0, 0x31, 12, 21, 0xE0},
  };
  for (const auto &f : fixtures) {
    Bus bus(f.protocol, f.system);
    auto status = Bus::frame(f.protocol, f.status_opcode, f.type, f.minimum, 0, 0xFF, f.marker);
    bus.feed(status);
    assert(bus.unknown_status());
    bus.confirm(f.protocol);
    const size_t diagnostics = bus.sensor.events.size();
    bus.feed(status);
    assert(bus.label("master status") && !bus.label("master extended status"));
    auto extended = Bus::frame(f.protocol, f.extended_opcode, f.type, f.extended_minimum, 0, 0xFF, f.marker);
    bus.feed(extended);
    assert(bus.label("master extended status") && !bus.label("master status"));
    bus.feed(Bus::frame(f.protocol, f.extended_opcode, f.type, f.extended_minimum + 4, 0, 0xFF, f.marker));
    assert(bus.label("master extended status"));
    bus.feed(Bus::frame(f.protocol, f.status_opcode, f.type, f.minimum - 1, 0, 0xFF, f.marker));
    assert(bus.unknown_status());
    bus.feed(Bus::frame(f.protocol, f.status_opcode, f.type ^ 1, f.minimum, 0, 0xFF, f.marker));
    assert(bus.unknown_status());
    bus.feed(Bus::frame(f.protocol, f.status_opcode, f.type, f.minimum, 0x40, 0xFF, f.marker));
    assert(bus.unknown_status());
    if (f.system == SystemType::AIR || f.protocol == Protocol::A0) {
      bus.feed(Bus::frame(f.protocol, 0x11, f.type, f.minimum, 0, 0xFF, f.marker));
      assert(bus.unknown_status());
    }
    status[status.size() - (f.protocol == Protocol::TCC ? 1 : 2)] ^= 1;
    bus.feed(status);
    assert(bus.unknown_status());
    assert(bus.remotes() == 0 && bus.sensor.events.size() == diagnostics);
    assert(bus.mode == esphome::climate::CLIMATE_MODE_OFF && bus.target_temperature == 22.0f);
    for (size_t size = 0; size < status.size(); size++)
      assert(!bus.recognizes(f.protocol, status.data(), size, false));
    assert(!bus.recognizes(f.protocol, nullptr, 0, true));
    assert(!bus.recognizes(Protocol::AUTO, status.data(), status.size(), false));
  }
  Bus water(Protocol::TU2C, SystemType::WATER);
  water.confirm(Protocol::TU2C);
  for (uint8_t length : {uint8_t(12), uint8_t(17), uint8_t(20), uint8_t(21), uint8_t(24)}) {
    water.feed(Bus::frame(Protocol::TU2C, 0x00, 0x31, length, 0, 0x60, 0xE0));
    assert(water.label(length < 21 ? "master status" : "master extended status"));
  }
  water.feed(Bus::frame(Protocol::TU2C, 0xC0, 0x31, 21, 0, 0xFF, 0xC0));
  assert(water.unknown_status());
  Bus air(Protocol::TU2C, SystemType::AIR);
  air.confirm(Protocol::TU2C);
  air.feed(Bus::frame(Protocol::TU2C, 0xC0, 0x38, 15, 0, 0x50));
  assert(air.unknown_status());
  Bus tcc_water(Protocol::TCC, SystemType::WATER);
  tcc_water.confirm(Protocol::TCC);
  tcc_water.feed(Bus::frame(Protocol::TCC, 0x1C, 0x81, 7));
  assert(tcc_water.unknown_status());
  std::cout << "Master status tests passed (five combinations, signatures, minimum lengths, CRC, source, labels).\n";
}
