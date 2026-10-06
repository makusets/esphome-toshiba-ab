#include "components/toshiba_ab/wall_mounted_louvre.h"
#include <cassert>

using namespace esphome::toshiba_ab;

int main() {
  // Exact Intesis command captures, including their verified XOR checksums.
  const std::array<std::array<uint8_t, 10>, 4> captures{{
      {0xDE, 0xF0, 0x11, 0x05, 0x00, 0x4C, 0x22, 0x0B, 0x76, 0x29},
      {0xDE, 0xF0, 0x11, 0x05, 0x00, 0x4C, 0x22, 0x13, 0x76, 0x31},
      {0xDE, 0xF0, 0x11, 0x05, 0x00, 0x4C, 0x22, 0x1B, 0x76, 0x39},
      {0xDE, 0xF0, 0x11, 0x05, 0x00, 0x4C, 0x22, 0x23, 0x76, 0x01},
  }};
  const std::array<uint8_t, 4> reported_mode_power{0x45, 0x49, 0x4D, 0x51};
  for (uint8_t position = 1; position <= 4; position++) {
    assert(encode_wall_mounted_louvre_command(0xDE, 0xF0, 0x02, position, 0x76) == captures[position - 1]);
    assert(decode_wall_mounted_louvre_position(reported_mode_power[position - 1]) == position);
    // Power and operating mode must not affect position decoding.
    assert(decode_wall_mounted_louvre_position(reported_mode_power[position - 1] ^ 0xE1) == position);
  }
  // Use the configured remote address, not the captured Intesis address.
  const auto local = encode_wall_mounted_louvre_command(0x40, 0x00, 0x02, 1, 0x74);
  assert(local[0] == 0x40 && local[1] == 0x00 && local[8] == 0x74 && local[9] == 0x45);
  const auto alternate = encode_wall_mounted_louvre_command(0x41, 0x02, 0x02, 1, 0x74);
  assert(alternate[0] == 0x41 && alternate[1] == 0x02 && alternate[9] == 0x46);
  assert((local[6] & 0x18) == 0);  // Neither temperature nor fan change requested.
  assert(decode_wall_mounted_louvre_position(0x41) == 0);
  assert(decode_wall_mounted_louvre_position(0x5D) == 7);
}
