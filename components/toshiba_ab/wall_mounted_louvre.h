#pragma once

#include <array>
#include <cstdint>

namespace esphome {
namespace toshiba_ab {

// Classic TCC-Link wall-mounted unit captures from issue #221.
inline uint8_t decode_wall_mounted_louvre_position(uint8_t mode_power) {
  return (mode_power >> 2) & 0x07;
}

inline std::array<uint8_t, 10> encode_wall_mounted_louvre_command(uint8_t remote, uint8_t master, uint8_t mode,
                                                               uint8_t position, uint8_t target) {
  std::array<uint8_t, 10> frame{
      remote, master, 0x11, 0x05, 0x00, 0x4C, static_cast<uint8_t>(0x20 | (mode & 0x07)),
      static_cast<uint8_t>((position << 3) | 0x03), target, 0x00};
  for (std::size_t i = 0; i < frame.size() - 1; i++) {
    frame.back() ^= frame[i];
  }
  return frame;
}

}  // namespace toshiba_ab
}  // namespace esphome
