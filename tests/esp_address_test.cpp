#include "toshiba_ab.h"
#include <cassert>
#include <iostream>

uint32_t test_now = 1;
using namespace esphome::toshiba_ab;

class Bus : public ToshibaAbClimate {
 public:
  esphome::text_sensor::TextSensor sensor;
  Bus(Protocol protocol, SystemType type, uint8_t address = 0xAA) {
    set_protocol(protocol);
    set_system_type(type);
    set_esp_address(address);
    set_diagnostic_sensor(&sensor);
    setup();
  }
  uint8_t address() const { return esp_address_; }
  bool confirmed() const { return master_address_confirmed_; }
  size_t remotes() const { return remotes_.size(); }
  size_t events(const std::string &prefix) const {
    size_t count = 0;
    for (const auto &event : sensor.events)
      if (event.find(prefix) == 0) count++;
    return count;
  }
  void feed(std::vector<uint8_t> bytes, Protocol protocol, bool valid = true) {
    if (protocol == Protocol::TCC) {
      uint8_t crc = 0;
      for (auto byte : bytes) crc ^= byte;
      bytes.push_back(crc);
    } else if (protocol == Protocol::TU2C) {
      uint8_t sum = 0;
      for (size_t i = 2; i < bytes.size(); i++) sum += bytes[i];
      bytes.push_back(sum);
      bytes.push_back(0xA0);
    } else {
      const uint16_t crc = crc16_mcrf4xx_(bytes.data(), bytes.size());
      bytes.push_back(crc >> 8);
      bytes.push_back(crc & 0xFF);
    }
    if (!valid) bytes[bytes.size() - (protocol == Protocol::TCC ? 1 : 2)] ^= 1;
    for (auto byte : bytes) read_byte_(byte);
  }
  void master(Protocol protocol, uint8_t source = 0x00) {
    if (protocol == Protocol::TCC) feed({source, 0xF0, 0x10, 2, 0, 0x8A}, protocol);
    else if (protocol == Protocol::TU2C) feed({0xF0, 0xF0, 0x0A, source, 0xFF, 0xC0, 0, 0x3A}, protocol);
    else feed({0xA0, 0, 0x10, 7, 0, 8, source, 0, 0xFE, 0, 0x8A}, protocol);
  }
  void ping(Protocol protocol, uint8_t source, uint8_t destination = 0x00,
            bool valid = true, bool water = false) {
    if (protocol == Protocol::TCC)
      feed({source, destination, 0x15, 7, 8, 0x0C, 0x81, 0, 0, 0x48, 0}, protocol, valid);
    else if (protocol == Protocol::TU2C)
      feed({0xF0, 0xF0, 0x0C, source, destination, uint8_t(water ? 0xE0 : 0xC0),
            0x41, uint8_t(water ? 0x0C : 0x5C), 0, 0}, protocol, valid);
    else
      feed({0xA0, 0, 0x15, 0x0C, 0, 0, source, 8, destination, 0x0C, 0x81, 0, 0, 0, 0, 0},
           protocol, valid);
  }
};

int main() {
  const Protocol protocols[] = {Protocol::TCC, Protocol::TU2C, Protocol::TU2C, Protocol::A0};
  const SystemType types[] = {SystemType::AIR, SystemType::AIR, SystemType::WATER, SystemType::WATER};
  for (size_t case_index = 0; case_index < 4; case_index++) {
    test_now = 1;
    const auto protocol = protocols[case_index];
    const auto type = types[case_index];
    const auto candidates = esp_address_candidates(protocol, type);
    Bus bus(protocol, type);
    assert(bus.address() == 0xAA);
    bus.ping(protocol, candidates.data[0]);
    assert(bus.remotes() == 0);  // No remote classification before master confirmation.
    bus.master(protocol);
    assert(bus.confirmed() && bus.address() == candidates.data[0]);
    for (size_t i = 0; i < candidates.size; i++) {
      bus.ping(protocol, candidates.data[i], 0, true, type == SystemType::WATER);
      assert(bus.address() == (i + 1 < candidates.size ? candidates.data[i + 1] : 0xAA));
    }
    assert(bus.events("ESP auto address unavailable:") == 1);
    bus.ping(protocol, candidates.data[0]);
    assert(bus.events("ESP auto address unavailable:") == 1);
    // Refresh every address except the lowest, then expire only the lowest.
    test_now = 300000;
    for (size_t i = 1; i < candidates.size; i++)
      bus.ping(protocol, candidates.data[i], 0, true, type == SystemType::WATER);
    test_now = 300001;
    bus.loop();
    assert(bus.remotes() == candidates.size - 1 && bus.address() == candidates.data[0]);
    bus.reset();
    assert(bus.address() == 0xAA && bus.remotes() == 0);
    bus.master(protocol);
    assert(bus.address() == candidates.data[0]);

    Bus fixed(protocol, type, candidates.data[0]);
    fixed.master(protocol);
    fixed.ping(protocol, candidates.data[0], 0, true, type == SystemType::WATER);
    fixed.ping(protocol, candidates.data[0], 0, true, type == SystemType::WATER);
    assert(fixed.address() == candidates.data[0]);
    assert(fixed.events("ESP address collision:") == 1);
    test_now += 300000;
    fixed.loop();
    fixed.ping(protocol, candidates.data[0], 0, true, type == SystemType::WATER);
    fixed.reset();
    fixed.master(protocol);
    fixed.ping(protocol, candidates.data[0], 0, true, type == SystemType::WATER);
    assert(fixed.address() == candidates.data[0] && fixed.events("ESP address collision:") == 1);
  }

  // Invalid checksums and traffic for another master cannot move the address.
  for (auto protocol : {Protocol::TCC, Protocol::TU2C, Protocol::A0}) {
    test_now = 1;
    Bus bus(protocol, protocol == Protocol::A0 ? SystemType::WATER : SystemType::AIR);
    bus.master(protocol);
    const auto first = bus.address();
    bus.ping(protocol, first, 0, false);
    test_now += 26;
    bus.loop();  // Discard any TCC resync candidate left by the invalid frame.
    bus.ping(protocol, first, 1);
    assert(bus.address() == first && bus.remotes() == 0);
  }

  // A lower free address is preferred even while a higher chosen address is free.
  test_now = 1;
  Bus tcc(Protocol::TCC, SystemType::AIR);
  tcc.master(Protocol::TCC);
  tcc.ping(Protocol::TCC, 0x40);
  tcc.ping(Protocol::TCC, 0x41);
  assert(tcc.address() == 0x43);  // Reserved 0x42 is skipped.
  test_now = 300000;
  tcc.ping(Protocol::TCC, 0x40);
  test_now = 300001;
  tcc.loop();
  assert(tcc.address() == 0x41);

  test_now = 0xFFFFFF00;
  Bus wrapped(Protocol::TCC, SystemType::AIR);
  wrapped.master(Protocol::TCC);
  wrapped.ping(Protocol::TCC, 0x40);
  test_now += 300000;  // Expiration also works across millis() wraparound.
  wrapped.loop();
  assert(wrapped.address() == 0x40 && wrapped.remotes() == 0);

  Bus mismatch(Protocol::TCC, SystemType::AIR);
  mismatch.set_master_address(1);
  mismatch.setup();
  mismatch.master(Protocol::TCC, 0);
  assert(!mismatch.confirmed() && mismatch.address() == 0xAA);
  mismatch.master(Protocol::TCC, 1);
  assert(mismatch.confirmed() && mismatch.address() == 0x40);

  Bus master_collision(Protocol::TCC, SystemType::AIR, 0x40);
  master_collision.master(Protocol::TCC, 0x40);
  assert(master_collision.address() == 0x40 && master_collision.events("ESP address collision:") == 1);
  Bus avoid_master(Protocol::TCC, SystemType::AIR);
  avoid_master.master(Protocol::TCC, 0x40);
  assert(avoid_master.address() == 0x41);

  for (auto protocol : {Protocol::TCC, Protocol::A0}) {
    Bus unsupported(protocol, protocol == Protocol::TCC ? SystemType::WATER : SystemType::AIR);
    unsupported.master(protocol);
    assert(unsupported.address() == 0xAA && unsupported.events("ESP auto address unavailable:") == 1);
  }
  test_now = 1;
  Bus automatic(Protocol::AUTO, SystemType::AIR);
  automatic.master(Protocol::TCC);
  automatic.ping(Protocol::TCC, 0x40);
  test_now = 60001;
  automatic.loop();
  automatic.ping(Protocol::TCC, 0x41);
  assert(automatic.address() == 0x43);  // Assignment follows the confirmed, not YAML, protocol.
  std::cout << "ESP address tests passed (all four ranges, collision, expiration, reset, filtering, wraparound).\n";
}
