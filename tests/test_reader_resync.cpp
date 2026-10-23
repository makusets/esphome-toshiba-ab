// Build with ESPHome's generated host headers; see tests/README.md.
#include "components/toshiba_ab/toshiba_ab.h"
#include <cassert>
#include <iostream>
#include <vector>

namespace esphome {
uint32_t millis() { return 0; }
}

using esphome::toshiba_ab::DataFrameReader;

static const std::vector<uint8_t> PING{0x00, 0xFE, 0x10, 0x02, 0x80, 0x8A, 0xE6};

static void check_pings(unsigned leading_zeros, unsigned frame_count) {
  DataFrameReader reader;
  for (unsigned i = 0; i < leading_zeros; ++i)
    assert(!reader.put(0x00));
  unsigned received = 0;
  for (unsigned i = 0; i < frame_count; ++i) {
    for (size_t j = 0; j < PING.size(); ++j) {
      const bool complete = reader.put(PING[j]);
      assert(complete == (j == PING.size() - 1));
      if (complete) {
        assert(reader.crc_valid);
        assert(reader.frame.get_data() == PING);
        ++received;
      }
    }
  }
  assert(received == frame_count);
  assert(reader.consume_reset_count() == leading_zeros);
}

int main() {
  check_pings(0, 1);
  check_pings(1, 1);
  check_pings(8, 1);
  check_pings(0, 3);
  check_pings(3, 3);

  // Rejecting equal addresses counts the discarded source byte as noise,
  // while retaining the destination byte as the next possible source.
  DataFrameReader rejected;
  assert(!rejected.put(0x00));
  assert(rejected.consume_reset_count() == 0);
  assert(!rejected.put(0x00));
  assert(rejected.consume_reset_count() == 1);
  assert(rejected.consume_reset_count() == 0);
  assert(rejected.data_index_ == 1);
  for (size_t i = 1; i < PING.size(); ++i)
    assert(rejected.put(PING[i]) == (i == PING.size() - 1));
  assert(rejected.crc_valid);
  assert(rejected.frame.get_data() == PING);
  assert(rejected.consume_reset_count() == 0);

  // The exception for dialects that permit equal addresses still works.
  DataFrameReader allowed;
  allowed.set_allow_same_source_dest(true);
  const std::vector<uint8_t> same_address{0x00, 0x00, 0x10, 0x02, 0x80, 0x8A, 0x18};
  for (size_t i = 0; i < same_address.size(); ++i)
    assert(allowed.put(same_address[i]) == (i == same_address.size() - 1));
  assert(allowed.crc_valid);
  assert(allowed.frame.get_data() == same_address);
  assert(allowed.consume_reset_count() == 0);

  // Recovery does not bypass CRC validation.
  DataFrameReader corrupted;
  assert(!corrupted.put(0x00));
  auto bad_ping = PING;
  bad_ping.back() ^= 1;
  for (size_t i = 0; i < bad_ping.size(); ++i)
    assert(corrupted.put(bad_ping[i]) == (i == bad_ping.size() - 1));
  assert(!corrupted.crc_valid);
  std::cout << "Reader resynchronization checks passed\n";
}
