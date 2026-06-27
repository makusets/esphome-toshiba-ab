#pragma once

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/uart/uart.h"
#include <algorithm>
#include <array>
#include <bitset>
#include <cctype>
#include <queue>

namespace esphome {
namespace toshiba_ab {

const uint32_t ALIVE_MESSAGE_PERIOD_MILLIS = 5000;
const uint32_t LAST_ALIVE_TIMEOUT_MILLIS = ALIVE_MESSAGE_PERIOD_MILLIS * 3 +1000;

const uint32_t PACKET_MIN_WAIT_MILLIS = 200;
const uint32_t FRAME_SEND_MILLIS_FROM_LAST_RECEIVE = 500;
const uint32_t FRAME_SEND_MILLIS_FROM_LAST_SEND = 500;

// const uint8_t TOSHIBA_MASTER = 0x00;  replaced by master_address_ which is set up in yaml
const uint8_t TOSHIBA_REMOTE_DEFAULT = 0x40;
const uint8_t TOSHIBA_REMOTE_MAX = 0x49;
const uint8_t TOSHIBA_TU2C_REMOTE_DEFAULT = 0x50;
const uint8_t TOSHIBA_TU2C_MASTER_DEFAULT = 0x90;
const uint8_t TOSHIBA_ESTIA_REMOTE_DEFAULT = 0x60;
const uint8_t TOSHIBA_ESTIA_REMOTE_MAX = 0x69;
const uint8_t TOSHIBA_ESTIA_MASTER_DEFAULT = 0x70;
const uint8_t TOSHIBA_TEMP_SENSOR = 0x42;
const uint8_t TOSHIBA_BROADCAST = 0xF0;
const uint8_t TOSHIBA_REPORT = 0x52;

const uint8_t OPCODE_PING = 0x10;  
const uint8_t OPCODE_PARAMETER = 0x11;
const uint8_t OPCODE_ERROR_HISTORY = 0x15;
const uint8_t OPCODE_SENSOR_QUERY = 0x17;
const uint8_t OPCODE_ACK = 0x18;
const uint8_t OPCODE_SENSOR_VALUE = 0x1A;
const uint8_t OPCODE_STATUS = 0x1C;
const uint8_t OPCODE_TEMPERATURE = 0x55;
const uint8_t OPCODE_EXTENDED_STATUS = 0x58;

// Classic TCC-Link STATUS / EXTENDED_STATUS data byte offsets.
const uint8_t STATUS_DATA_MODEPOWER_BYTE = 2;
const uint8_t STATUS_DATA_POWER_MASK = 0b00000001;
const uint8_t STATUS_DATA_MODE_MASK = 0b11100000;
const uint8_t STATUS_DATA_MODE_SHIFT_BITS = 5;
const uint8_t STATUS_DATA_FLAGS_BYTE = 4;
// const uint8_t STATUS_DATA_COOLING_MASK = 0b00001000; // e.g. 48 in
// 01:52:11:04:80:86:48:01:09 const uint8_t STATUS_DATA_COOLING_SHIFT_BITS = 3;
// const uint8_t STATUS_DATA_HEATING_MASK = 0b00000001; // e.g. 81 in
// 01:52:11:04:80:86:81:01:C0. Also in DRY mode
const uint8_t STATUS_DATA_FANVENT_BYTE = 3;
const uint8_t STATUS_DATA_FAN_MASK = 0b11100000;
const uint8_t STATUS_DATA_FAN_SHIFT_BITS = 5;
const uint8_t STATUS_DATA_VENT_MASK = 0b00000100;
const uint8_t STATUS_DATA_VENT_SHIFT_BITS = 2;
const uint8_t STATUS_DATA_TARGET_TEMP_BYTE = 6;

const uint8_t COMMAND_MODE_READ = 0x08;
const uint8_t COMMAND_MODE_WRITE = 0x80;

const uint8_t COMMAND_SET_TEMP = 0b00001000;
const uint8_t COMMAND_SET_FAN = 0b00010000;

const uint8_t COMMAND_MODE_FLAGS_TEMP = 0b00001000;
const uint8_t COMMAND_MODE_FLAGS_FAN = 0b00010000;

const uint8_t OPCODE2_READ_STATUS = 0x81;
const uint8_t OPCODE2_READ_ALIVE = 0x8A;
const uint8_t OPCODE2_PARAM_ACK = 0xA1;
const uint8_t OPCODE2_READ_MODE = 0x86;
const uint8_t OPCODE2_PING_PONG = 0x0C;
const uint8_t OPCODE2_SET_POWER = 0x41;
const uint8_t OPCODE2_SET_MODE = 0x42;
const uint8_t OPCODE2_SET_TEMP_WITH_FAN = 0x4C;
const uint8_t OPCODE2_SET_VENT = 0x52;
const uint8_t OPCODE2_SAVE = 0x54;
const uint8_t OPCODE2_SENSOR_QUERY = 0x80;
const uint8_t OPCODE2_SENSOR_ROOM_TEMP = 0x81;

const uint8_t TEMPERATURE_DATA_MASK = 0xFF; // Changed from 0b11111110 to keep the .5 degrees precision
const float TEMPERATURE_CONVERSION_RATIO = 2.0;
const float TEMPERATURE_CONVERSION_OFFSET = 35.0;

const uint8_t EMPTY_DATA = 0x00;

const uint8_t POWER_ON = 0x01;
const uint8_t POWER_OFF = 0x00;

const uint8_t MODE_MASK = 0x07;
const uint8_t MODE_HEAT = 0x01;
const uint8_t MODE_COOL = 0x02;
const uint8_t MODE_FAN_ONLY = 0x03;
const uint8_t MODE_DRY = 0x04;
const uint8_t MODE_AUTO = 0x05;

const uint8_t FAN_PEED_AUTO = 0x02;
const uint8_t FAN_PEED_LOW = 0x05;
const uint8_t FAN_PEED_MED = 0x04;
const uint8_t FAN_PEED_HIGH = 0x03;

const uint8_t SET_PARAMETER_PAYLOAD_HEADER_SIZE = 2;

const uint8_t DATA_FRAME_MAX_SIZE = 128;
const uint8_t DATA_MAX_SIZE = DATA_FRAME_MAX_SIZE - 4;  // exclude crc
const uint8_t DATA_MIN_SIZE = 2;

const uint8_t DATA_OFFSET_FROM_START = 4;

const uint8_t DATA_FRAME_SOURCE = 0;
const uint8_t DATA_FRAME_DEST = 1;
const uint8_t DATA_FRAME_OPCODE1 = 2;
const uint8_t DATA_FRAME_DATA_LENGTH = 3;
const uint8_t DATA_FRAME_READWRITE_FLAGS = 5;
const uint8_t DATA_FRAME_OPCODE2 = 2;

// Sensor addresses for central unit
constexpr uint8_t SENSOR_ID_CURRENT = 0x6A;  // Electrical current x10
constexpr float CURRENT_SCALE = 0.1f;        // 123 -> 12.3 A
const uint8_t SENSOR_ADDRESS_OUT_TEMP = 0x02;   // Outdoor temperature sensor
const uint8_t SENSOR_ADDRESS_IN_TEMP = 0x03;    // Indoor temperature sensor

// adds all the sensors configured in yaml to the component
struct PolledSensor {
  uint8_t id;                     // sensor ID (address) for 0x17 query
  float scale;                    // scale factor to apply to value
  uint32_t interval_ms;           // polling interval
  sensor::Sensor *sensor;         // target sensor to publish into
};


struct DataFrame {
  union {
    uint8_t raw[DATA_FRAME_MAX_SIZE];
    struct {
      uint8_t source;
      uint8_t dest;
      uint8_t opcode1;
      uint8_t data_length;
      uint8_t data[DATA_MAX_SIZE];
    };
  };

  /**
   * Get size of the raw data (needs data_length set).
   * Returns 0 if not present.
   */
  size_t size() const {
    if (!validate_bounds())
      return 0;

    return DATA_OFFSET_FROM_START + data_length + 1;  // 1 for CRC byte end the end
  }

  /**
   * Check if data frame looks good size wise
   */
  bool validate_bounds() const { return data_length >= DATA_MIN_SIZE && data_length <= DATA_MAX_SIZE; }

  /**
   * Check if CRC in data matches calcualted CRC.
   */
  bool validate_crc() const {
    if (!validate_bounds())
      return false;

    return crc() == calculate_crc();
  }

  /**
   * Get CRC byte at the end of the data.
   * Returns 0 if not yet available
   */
  uint8_t crc() const {
    if (!validate_bounds())
      return 0;

    return raw[size() - 1];
  }

  void set_tu2c(bool tu2c) { tu2c_ = tu2c; }
  bool is_tu2c() const { return tu2c_; }

  void set_estia(bool estia) { estia_ = estia; }
  bool is_estia() const { return estia_; }
  void set_estia_len(uint8_t len) { estia_len_ = len; }
  uint8_t estia_len() const { return estia_len_; }

  // Estia A0-protocol: raw[] stores bytes AFTER the A0:00 prefix
  // raw[0]=Type, raw[1]=Len(LL), raw[2]=0x00, raw[3:4]=Src, raw[5:6]=Dst, raw[7:8]=DataType, ..., CRC16
  // Bytes in raw[] = LL + 4 (type + len + 0x00 + src/dst/dtype header + payload + CRC)
  size_t estia_size() const {
    if (!estia_ || estia_len_ < 4) return 0;
    return estia_len_ + 4;  // TT + LL + LL_payload + CRC(2)
  }

  uint16_t estia_crc_received() const {
    size_t s = estia_size();
    if (s < 6) return 0;
    return (raw[s - 2] << 8) | raw[s - 1];
  }

  uint16_t calculate_estia_crc() const {
    size_t s = estia_size();
    if (s < 6) return 0;

    // CRC-16/MCRF4XX: poly 0x8408 (reflected 0x1021), init 0xFFFF
    auto crc_byte = [](uint16_t crc, uint8_t b) -> uint16_t {
      crc ^= b;
      for (uint8_t i = 0; i < 8; i++) {
        if (crc & 1)
          crc = (crc >> 1) ^ 0x8408;
        else
          crc >>= 1;
      }
      return crc;
    };

    uint16_t crc = 0xFFFF;
    // A0:00 prefix is NOT stored in raw[] but IS part of the CRC
    crc = crc_byte(crc, 0xA0);
    crc = crc_byte(crc, 0x00);
    // Then the frame data (excluding the 2-byte CRC at the end)
    size_t len = s - 2;
    for (size_t i = 0; i < len; i++) {
      crc = crc_byte(crc, raw[i]);
    }
    return crc;
  }

  bool validate_estia_crc() const {
    return estia_crc_received() == calculate_estia_crc();
  }

  /**
   * Calculates CRC on the current data by creating an XOR sum
   */
  uint8_t calculate_crc() const {
    if (!validate_bounds())
      return 0;

    uint8_t result = 0;
    size_t len = size() - 1;  // exclude CRC byte and the end
    for (size_t i = 0; i < len; i++) {
      result ^= raw[i];
    }
    return result;
  }

  void reset() {
    for (size_t i = 0; i < DATA_FRAME_MAX_SIZE; i++) {
      raw[i] = 0;
    }
  }

  std::vector<uint8_t> get_data() const { return std::vector<uint8_t>(raw, raw + size()); }

 private:
  bool tu2c_{false};
  bool estia_{false};
  uint8_t estia_len_{0};  // LL byte from A0-protocol (not in union, won't be clobbered)
};

enum class FrameFormat : uint8_t {
  NORMAL = 0,
  TU2C = 1,
  A0 = 2,
  // HM-range dialect (RAV-RM/HM indoors paired with TU2C-Link-capable
  // outdoors). On-wire layout for master broadcasts and remote→master
  // frames is:
  //
  //   A0 00 OPCODE LEN <00 [src_mode] SRC [dst_mode] DST 00> opcode2 ... CRC
  //
  // The 6-byte interlude `00 [src_mode] SRC [dst_mode] DST 00` lives at
  // wire offsets 4..9 and embeds the source and destination addresses
  // at offsets 6 and 8 respectively. SRC=0x00 master / 0x40 remote /
  // 0xFE broadcast — the same identifiers as classic TCC-Link, just at
  // different byte positions. The reader normalises HM frames to the
  // standard `src dst opcode len opcode2 ... crc` layout in put(), so
  // the rest of the pipeline (status decoding, ACK matching, master
  // detection) works unchanged.
  //
  // CRC is not validated for HM frames — the algorithm hasn't been
  // derived (see makusets/esphome-toshiba-ab#76). Frames are accepted
  // unconditionally and crc_valid is set to false so callers can
  // distinguish if needed.
  HM = 3,
  ESTIA = 4,
};
constexpr FrameFormat NORMAL = FrameFormat::NORMAL;
constexpr FrameFormat TU2C = FrameFormat::TU2C;
constexpr FrameFormat A0 = FrameFormat::A0;
constexpr FrameFormat HM = FrameFormat::HM;
constexpr FrameFormat ESTIA = FrameFormat::ESTIA;

struct DataFrameReader {
  // Reads a data frame byte by byte, accumulating bytes until a complete frame is received
  // Returns true when a complete frame is received, false otherwise
  // The frame is stored in the `frame` member variable
  DataFrame frame;

  bool crc_valid{false};
  bool complete{false};
  uint8_t  data_index_{0};
  uint16_t expected_total_{0};  // header(4) + payload(len) + crc(1)
  bool tu2c_{false};
  uint8_t prefix_match_{0};
  bool tu2c_len_pending_{false};
  uint16_t tu2c_expected_total_{0};  // total bytes after length byte
  uint16_t tu2c_bytes_seen_{0};
  bool filter_frames_{true};
  bool allow_same_source_dest_{false};
  FrameFormat frame_format_{FrameFormat::NORMAL};
  bool estia_{false};
  uint16_t estia_expected_total_{0};
  struct NormalRestartCandidate {
    uint8_t marker_index{0};
    std::array<uint8_t, 3> header{{0, 0, 0}};
    std::array<uint8_t, 4> follow{{0, 0, 0, 0}};
    uint8_t follow_count{0};
  };
  static const uint8_t NORMAL_RESTART_CANDIDATE_MAX = 4;
  std::array<NormalRestartCandidate, NORMAL_RESTART_CANDIDATE_MAX> normal_restart_candidates_{};
  uint8_t normal_restart_candidate_count_{0};
  std::array<uint8_t, 4> normal_restart_lookahead_{{0, 0, 0, 0}};
  uint8_t normal_restart_lookahead_count_{0};
  std::array<uint8_t, DATA_FRAME_MAX_SIZE> normal_replay_{{0}};
  uint8_t normal_replay_count_{0};
  uint32_t estia_last_byte_ms_{0};
  static const uint32_t ESTIA_BYTE_TIMEOUT_MS = 20;  // inter-byte timeout: ~4 byte-times at 2400 baud

  void reset() {
    reset_frame_state_();
    tu2c_      = false;
    prefix_match_ = 0;
    tu2c_len_pending_ = false;
    tu2c_expected_total_ = 0;
    tu2c_bytes_seen_ = 0;
    estia_ = false;
    estia_expected_total_ = 0;
    reset_normal_restart_state_();
    normal_replay_count_ = 0;
  }

  bool put(uint8_t byte) {
    if (normal_replay_count_ == 0) {
      return put_byte_(byte);
    }

    enqueue_normal_replay_(byte);
    while (normal_replay_count_ > 0) {
      const uint8_t replay_byte = normal_replay_[0];
      for (uint8_t i = 1; i < normal_replay_count_; i++) {
        normal_replay_[i - 1] = normal_replay_[i];
      }
      normal_replay_count_--;
      if (put_byte_(replay_byte)) {
        return true;
      }
    }
    return false;
  }

  bool put_byte_(uint8_t byte) {
    const bool use_estia = frame_format_ == FrameFormat::A0;

    // ── Estia A0-protocol: prefix A0:00, then Type:Len:...:CRC16 ──
    if (use_estia) {
      // Timeout: if collecting a frame takes too long, a byte was lost — resync
      if (estia_ && (millis() - estia_last_byte_ms_) > ESTIA_BYTE_TIMEOUT_MS) {
        ESP_LOGV("READER", "Estia inter-byte timeout after %ums gap (%u/%u bytes); resync",
                 (unsigned)(millis() - estia_last_byte_ms_), data_index_, estia_expected_total_);
        estia_ = false;
        data_index_ = 0;
        estia_expected_total_ = 0;
        prefix_match_ = 0;
        // Don't return — process this byte as potential new frame start below
      }
      // Wait for A0 prefix
      if (!estia_ && prefix_match_ == 0) {
        if (byte == 0xA0) {
          prefix_match_ = 1;
        }
        return false;
      }
      // Wait for 0x00 after A0
      if (!estia_ && prefix_match_ == 1) {
        if (byte == 0x00) {
          estia_ = true;
          prefix_match_ = 0;
          reset_frame_state_();
          frame.set_estia(true);
          data_index_ = 0;
          estia_expected_total_ = 0;
          estia_last_byte_ms_ = millis();
          return false;
        }
        prefix_match_ = 0;
        return false;
      }
      // Reading frame bytes (after A0:00 prefix)
      if (estia_) {
        estia_last_byte_ms_ = millis();  // update inter-byte timer
        if (data_index_ < DATA_FRAME_MAX_SIZE) {
          frame.raw[data_index_] = byte;
        } else {
          ESP_LOGV("READER", "Estia frame buffer overflow; resetting");
          reset();
          return false;
        }

        // Byte 1 = LL (length). After A0:00 prefix, frame = LL + 4 bytes
        if (data_index_ == 1) {
          estia_expected_total_ = byte + 4;
          frame.set_estia_len(byte);  // store outside union so raw[3] won't clobber it
          if (estia_expected_total_ < 6 || estia_expected_total_ > DATA_FRAME_MAX_SIZE) {
            ESP_LOGV("READER", "Invalid Estia length 0x%02X; resetting", byte);
            reset();
            return false;
          }
        }

        data_index_++;

        // Check if frame complete
        if (estia_expected_total_ > 0 && data_index_ >= estia_expected_total_) {
          crc_valid = frame.validate_estia_crc();
          complete = true;
          data_index_ = 0;
          estia_ = false;
          estia_expected_total_ = 0;
          return true;
        }
      }
      return false;
    }

    const bool use_tu2c = frame_format_ == FrameFormat::TU2C || frame_format_ == FrameFormat::ESTIA;
    if (data_index_ == 0 && use_tu2c && !tu2c_) {
      if (prefix_match_ == 1) {
        if (byte == 0xF0) {
          tu2c_ = true;
          prefix_match_ = 0;
          reset_frame_state_();
          frame.set_tu2c(true);
          tu2c_len_pending_ = true;
          return false;
        }
        prefix_match_ = 0;
        return false;
      }
      if (byte == 0xF0) {
        prefix_match_ = 1;
      }
      return false;
    }
    if (!use_tu2c) {
      tu2c_ = false;
      prefix_match_ = 0;
    }

    uint8_t current_byte = byte;
    if (use_tu2c && tu2c_) {
      if (tu2c_len_pending_) {
        ESP_LOGV("READER", "Receiving TU2C frame...");
        // TU2C frames: third byte is the length, and it includes the 3 wrapping bytes.
        // The total length includes the two 0xF0 prefix bytes, the length byte itself,
        // and the trailing 0xA0. We therefore expect (len - 3) bytes after consuming
        // the length byte here, with the final one being 0xA0.
        tu2c_expected_total_ = current_byte > 3 ? current_byte - 3 : 0;
        tu2c_bytes_seen_ = 0;
        tu2c_len_pending_ = false;
        if (tu2c_expected_total_ < 1 || (tu2c_expected_total_ - 1) > DATA_FRAME_MAX_SIZE) {
          ESP_LOGV("READER", "Invalid TU2C length 0x%02X; resetting reader", current_byte);
          reset();
          return false;
        }
        frame.raw[data_index_] = current_byte;
        data_index_++;
        return false;
      }

      tu2c_bytes_seen_++;

      if (tu2c_expected_total_ > 0 && tu2c_bytes_seen_ == tu2c_expected_total_) {
        if (current_byte != 0xA0) {
          ESP_LOGV("READER", "TU2C frame ended without 0xA0 (seen=%u expected=%u); resetting",
                   tu2c_bytes_seen_, tu2c_expected_total_);
          reset();
          return false;
        }
        if (data_index_ >= DATA_OFFSET_FROM_START + 1) {
          frame.data_length = data_index_ - DATA_OFFSET_FROM_START - 1;  // subtract CRC
        } else {
          frame.data_length = 0;
        }
        crc_valid = false;
        complete  = true;

        data_index_     = 0;
        expected_total_ = 0;
        tu2c_        = false;
        tu2c_expected_total_ = 0;
        tu2c_bytes_seen_ = 0;

        return true;
      }

      if (tu2c_expected_total_ > 0 && tu2c_bytes_seen_ > tu2c_expected_total_) {
        ESP_LOGV("READER", "TU2C frame exceeded expected length; resetting");
        reset();
        return false;
      }
    }

    // Optionally filter obvious invalid source values at frame start.
    if (!use_tu2c && data_index_ == 0 && filter_frames_ && current_byte > 0xA0) {
      ESP_LOGV("READER", "Ignoring packet with out-of-range source: 0x%02X", current_byte);
      return false;
    }
    if (frame_format_ == FrameFormat::NORMAL && handle_normal_restart_candidates_(current_byte)) {
      return complete;
    }

    // In classic TCC-Link, some units restart a frame in-place and mark the
    // abandoned frame by sending 0x00 from the 5th byte onward. A 0x00 can
    // also be valid payload or CRC data, so only treat it as a candidate when
    // the four-byte proof still fits inside this frame. Otherwise a legitimate
    // late-frame 0x00 (common in 0x1A sensor values/CRC bytes) would defer
    // completion until the next packet and then be reset as a timeout.
    const bool normal_restart_candidate = frame_format_ == FrameFormat::NORMAL && data_index_ >= 4 &&
                                          expected_total_ > 0 && (data_index_ + 4) < expected_total_ &&
                                          current_byte == 0x00;
    const bool normal_restart_lookahead_byte = frame_format_ == FrameFormat::NORMAL &&
                                               normal_restart_candidate_count_ > 0 && expected_total_ > 0 &&
                                               data_index_ >= expected_total_;

    // Store byte
    if (normal_restart_lookahead_byte) {
      enqueue_normal_restart_lookahead_(current_byte);
    } else {
      frame.raw[data_index_] = current_byte;
    }
    if (normal_restart_candidate) {
      add_normal_restart_candidate_();
    }

    if (!use_tu2c && data_index_ == 1) {
      // ignore frames where source == dest (likely noise or corrupted)
      if (!allow_same_source_dest_ && frame.raw[0] == frame.raw[1]) {
          ESP_LOGV("READER", "Ignoring packet where source == dest: 0x%02X", frame.raw[0]);
          // reset reader state so next byte is treated as new frame start
          reset();
          return false;
      }
    }

    // When the length byte arrives (raw[3]), compute the expected total size
    if (!use_tu2c && data_index_ == 3) {
      frame.data_length = frame.raw[3];  // keep DataFrame fields in sync
      if (!frame.validate_bounds()) {    // early length sanity check
        ESP_LOGV("READER", "Invalid length 0x%02X; resetting reader", frame.data_length);
        // Resync
        reset();
        return false;
      }
      expected_total_ = DATA_OFFSET_FROM_START + frame.data_length + 1;  // 4 + len + crc
    }

    if (!normal_restart_lookahead_byte) {
      data_index_++;
    }

    if (frame_format_ == FrameFormat::NORMAL && normal_restart_candidate_count_ == 0 && expected_total_ > 0 &&
        data_index_ >= expected_total_) {
      replay_normal_restart_lookahead_();
    }

    // If we know how many bytes we expect, finish only when we have them all
    if (expected_total_ > 0 && data_index_ >= expected_total_ && normal_restart_candidate_count_ == 0) {
      if (!use_tu2c) {
        // We have the whole frame (header + payload + CRC).
        //
        // HM dialect: master broadcasts and remote→master frames arrive
        // with an A0:00 sync delimiter and a 6-byte interlude that
        // embeds the real source/dest at wire offsets 6 and 8. Canonicalise
        // in place so the rest of the pipeline sees the standard layout
        // `src dst opcode len opcode2 PAD payload ... crc`. Sensor-reply
        // frames (e.g. 00:40:1A:...) lack the A0:00 prefix and are passed
        // through unchanged.
        //
        // We strip the 6-byte HM wrapper (wire offsets 4..9) and then
        // inject a single 0x00 padding byte right after opcode2 so the
        // mode/power, fan/vent, flags and target_temp bytes land at
        // exactly the same data[] offsets as classic TCC-Link
        // (STATUS_DATA_*_BYTE constants). Net change to the frame size
        // is therefore −5 bytes (strip 6 wrapper + insert 1 padding).
        if (frame_format_ == FrameFormat::HM
            && frame.raw[0] == 0xA0 && frame.raw[1] == 0x00
            && frame.data_length >= 7) {
          DataFrameReader::canonicalize_hm_frame(&frame, expected_total_);
          // CRC algorithm for HM hasn't been derived yet — accept the
          // frame but flag the CRC as unvalidated so callers can decide.
          crc_valid = false;
        } else {
          crc_valid = frame.validate_crc();
        }
        complete  = true;

        // Prepare reader for the next frame
        data_index_     = 0;
        expected_total_ = 0;

        return true;
      }
    }

    // Guard against runaway input
    if (data_index_ == DATA_FRAME_MAX_SIZE) {
      ESP_LOGW("READER", "Went over buffer; resetting");
      reset();
    }

    return false;
  }


  static bool canonicalize_hm_frame(DataFrame *hm_frame, uint16_t total) {
    if (hm_frame == nullptr || total < 12 || hm_frame->raw[0] != 0xA0 || hm_frame->raw[1] != 0x00 ||
        hm_frame->data_length < 7) {
      return false;
    }
    const uint8_t crc_byte = hm_frame->raw[total - 1];
    const uint8_t new_src = hm_frame->raw[6];
    const uint8_t new_dst = hm_frame->raw[8];
    const uint8_t new_data_length = hm_frame->data_length - 5;
    const uint8_t opcode2 = hm_frame->raw[10];
    const uint8_t tail_len = hm_frame->data_length - 7;
    for (uint8_t i = 0; i < tail_len; i++) {
      hm_frame->raw[DATA_OFFSET_FROM_START + 2 + i] = hm_frame->raw[11 + i];
    }
    hm_frame->raw[DATA_OFFSET_FROM_START + 0] = opcode2;
    hm_frame->raw[DATA_OFFSET_FROM_START + 1] = 0x00;
    hm_frame->raw[0] = new_src;
    hm_frame->raw[1] = new_dst;
    hm_frame->raw[3] = new_data_length;
    hm_frame->data_length = new_data_length;
    hm_frame->raw[DATA_OFFSET_FROM_START + new_data_length] = crc_byte;
    return true;
  }

  void set_filter_frames(bool filter_frames) { filter_frames_ = filter_frames; }
  bool filter_frames() const { return filter_frames_; }
  void set_allow_same_source_dest(bool allow_same) { allow_same_source_dest_ = allow_same; }
  void set_frame_format(FrameFormat format) {
    frame_format_ = format;
    reset();
  }
  FrameFormat frame_format() const { return frame_format_; }

private:
  void reset_normal_restart_state_() {
    for (auto &candidate : normal_restart_candidates_) {
      candidate.marker_index = 0;
      candidate.header = {{0, 0, 0}};
      candidate.follow = {{0, 0, 0, 0}};
      candidate.follow_count = 0;
    }
    normal_restart_candidate_count_ = 0;
    normal_restart_lookahead_ = {{0, 0, 0, 0}};
    normal_restart_lookahead_count_ = 0;
  }

  void enqueue_normal_replay_(uint8_t byte) {
    if (normal_replay_count_ >= normal_replay_.size()) {
      ESP_LOGW("READER", "Normal replay buffer overflow; dropping byte 0x%02X", byte);
      return;
    }
    normal_replay_[normal_replay_count_++] = byte;
  }

  void enqueue_normal_restart_lookahead_(uint8_t byte) {
    if (normal_restart_lookahead_count_ >= normal_restart_lookahead_.size()) {
      ESP_LOGW("READER", "Normal restart lookahead overflow; dropping byte 0x%02X", byte);
      return;
    }
    normal_restart_lookahead_[normal_restart_lookahead_count_++] = byte;
  }

  void replay_normal_restart_lookahead_() {
    for (uint8_t i = 0; i < normal_restart_lookahead_count_; i++) {
      enqueue_normal_replay_(normal_restart_lookahead_[i]);
    }
    normal_restart_lookahead_count_ = 0;
  }

  void remove_normal_restart_candidate_(uint8_t index) {
    if (index >= normal_restart_candidate_count_) {
      return;
    }
    for (uint8_t i = index + 1; i < normal_restart_candidate_count_; i++) {
      normal_restart_candidates_[i - 1] = normal_restart_candidates_[i];
    }
    normal_restart_candidate_count_--;
    normal_restart_candidates_[normal_restart_candidate_count_] = NormalRestartCandidate{};
  }

  void add_normal_restart_candidate_() {
    if (normal_restart_candidate_count_ >= normal_restart_candidates_.size()) {
      ESP_LOGW("READER", "Too many overlapping normal restart candidates; dropping marker at byte %u",
               static_cast<unsigned>(data_index_ + 1));
      return;
    }
    auto &candidate = normal_restart_candidates_[normal_restart_candidate_count_++];
    candidate.marker_index = data_index_;
    candidate.header[0] = frame.raw[0];
    candidate.header[1] = frame.raw[1];
    candidate.header[2] = frame.raw[2];
    candidate.follow = {{0, 0, 0, 0}};
    candidate.follow_count = 0;
  }

  bool handle_normal_restart_candidates_(uint8_t current_byte) {
    for (uint8_t i = 0; i < normal_restart_candidate_count_;) {
      auto &candidate = normal_restart_candidates_[i];
      candidate.follow[candidate.follow_count++] = current_byte;
      if (candidate.follow_count < candidate.follow.size()) {
        i++;
        continue;
      }

      const bool confirmed_restart = candidate.follow[0] == candidate.header[0] &&
                                     candidate.follow[1] == candidate.header[1] &&
                                     candidate.follow[2] == candidate.header[2];
      if (confirmed_restart) {
        ESP_LOGV("READER", "Normal TCC-Link frame restart marker at byte %u confirmed by header %02X:%02X:%02X",
                 static_cast<unsigned>(candidate.marker_index + 1), candidate.header[0], candidate.header[1],
                 candidate.header[2]);
        const std::array<uint8_t, 4> restarted_header = candidate.follow;
        reset_frame_state_();
        frame.raw[0] = restarted_header[0];
        frame.raw[1] = restarted_header[1];
        frame.raw[2] = restarted_header[2];
        frame.raw[3] = restarted_header[3];
        frame.data_length = restarted_header[3];
        data_index_ = 4;
        if (!frame.validate_bounds()) {
          ESP_LOGV("READER", "Invalid restarted frame length 0x%02X; resetting reader", frame.data_length);
          reset();
        } else {
          expected_total_ = DATA_OFFSET_FROM_START + frame.data_length + 1;
        }
        return true;
      }

      remove_normal_restart_candidate_(i);
    }

    return false;
  }

  void reset_frame_state_() {
    frame.reset();
    frame.set_tu2c(false);
    frame.set_estia(false);
    crc_valid       = false;
    complete        = false;
    data_index_     = 0;
    expected_total_ = 0;
    tu2c_len_pending_ = false;
    tu2c_expected_total_ = 0;
    tu2c_bytes_seen_ = 0;
    reset_normal_restart_state_();
  }
};

struct TccState {
  uint8_t mode;
  uint8_t fan;
  uint8_t vent;
  float room_temp = NAN;
  float target_temp = NAN;
  uint8_t power;
  uint8_t cooling;
  uint8_t heating;
  uint8_t preheating;
  bool filter_alert = false;


  TccState(){};

  TccState(const struct TccState *src) {
    mode = src->mode;
    fan = src->fan;
    vent = src->vent;
    room_temp = src->room_temp;
    target_temp = src->target_temp;
    power = src->power;
    cooling = src->cooling;
    heating = src->heating;
    preheating = src->preheating;
    filter_alert = src->filter_alert;
  };
};

class ToshibaAbClimate : public Component, public uart::UARTDevice, public climate::Climate {
 public:
  ToshibaAbClimate();

  void dump_config() override;
  void setup() override;
  void loop() override;

  uint8_t master_address_ = 0x00;
  bool master_address_confirmed_{false};
  uint8_t remote_address_{TOSHIBA_REMOTE_DEFAULT};
  bool remote_address_auto_{true};
  bool remote_address_confirmed_{false};
  bool master_address_auto_{true};
  uint8_t tu2c_remote_address_{TOSHIBA_TU2C_REMOTE_DEFAULT};
  uint8_t tu2c_master_address_{TOSHIBA_TU2C_MASTER_DEFAULT};
  bool filter_frames_{true};
  void set_master_address(uint8_t address);
  void set_remote_address(uint8_t address) {
    remote_address_ = std::min(address, static_cast<uint8_t>(TOSHIBA_ESTIA_REMOTE_MAX));
    remote_address_auto_ = false;
    remote_address_confirmed_ = true;
  }
  void set_master_address_auto(bool auto_detect) {
    master_address_auto_ = auto_detect;
  }
  bool get_master_address_auto() const { return master_address_auto_; }
  void set_filter_frames(bool filter_frames) {
    filter_frames_ = filter_frames;
    data_reader.set_filter_frames(filter_frames);
  }
  bool get_filter_frames() const { return filter_frames_; }
  void set_packet_min_wait(uint32_t millis) { packet_min_wait_millis_ = millis; }
  void set_command_mode_read(uint8_t value) { command_mode_read_ = value; }
  void set_command_mode_write(uint8_t value) { command_mode_write_ = value; }
  void set_filter_alert_sensor(binary_sensor::BinarySensor *sensor) { filter_alert_sensor_ = sensor; }
  void set_outdoor_temp_sensor(sensor::Sensor *sensor) { outdoor_temp_sensor_ = sensor; }
  void set_compressor_hours_sensor(sensor::Sensor *sensor) { compressor_hours_sensor_ = sensor; }
  void set_waterpump_hours_sensor(sensor::Sensor *sensor) { waterpump_hours_sensor_ = sensor; }
  void set_backup_heater_hours_sensor(sensor::Sensor *sensor) { backup_heater_hours_sensor_ = sensor; }
  void set_demand_sensor(sensor::Sensor *sensor) { demand_sensor_ = sensor; }
  void set_zone1_water_temp_sensor(sensor::Sensor *sensor) { zone1_water_temp_sensor_ = sensor; }
  void set_zone1_target_temperature_sensor(sensor::Sensor *sensor) { zone1_target_temperature_sensor_ = sensor; }
  void set_frame_format(FrameFormat format) {
    data_reader.set_frame_format(format);
    frame_format_auto_ = false;
    frame_format_confirmed_ = true;
    if (format == FrameFormat::ESTIA) {
      if (remote_address_auto_) remote_address_ = TOSHIBA_ESTIA_REMOTE_DEFAULT;
    }
  }
  void set_frame_format_auto() {
    data_reader.set_frame_format(FrameFormat::NORMAL);
    frame_format_auto_ = true;
    frame_format_confirmed_ = false;
  }
  bool is_hm_variant() const { return data_reader.frame_format() == FrameFormat::HM; }
  bool is_estia_first_gen() const { return data_reader.frame_format() == FrameFormat::ESTIA; }

  // ESP8266 only: receive on the hardware UART0 using `pin` (auto-detected for
  // GPIO13, or explicitly configured for GPIO3/GPIO13) instead of ESPHome's
  // software serial. The software-serial RX ISR
  // busy-waits ~1 char time per byte and starves Wi-Fi enough to trip the
  // watchdog on busy buses; the hardware UART has a FIFO and no busy-wait. TX
  // stays on the configured uart tx_pin (software serial). No effect elsewhere.
  void set_hardware_uart_rx_pin(uint8_t pin) {
    this->hw_uart_rx_pin_ = pin;
    this->hw_uart_rx_enabled_ = true;
  }

  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;

  void set_connected_binary_sensor(binary_sensor::BinarySensor *connected_binary_sensor) {
    connected_binary_sensor_ = connected_binary_sensor;
  }

  void set_vent_switch(switch_::Switch *vent_switch) { vent_switch_ = vent_switch; }

  void set_read_only_switch(switch_::Switch *read_only_switch) { read_only_switch_ = read_only_switch; }
  void set_zone1_switch(switch_::Switch *zone1_switch) { zone1_switch_ = zone1_switch; }
  void set_dhw_boost_switch(switch_::Switch *dhw_boost_switch) { dhw_boost_switch_ = dhw_boost_switch; }

  void set_failed_crcs_sensor(sensor::Sensor *failed_crcs_sensor) { this->failed_crcs_sensor_ = failed_crcs_sensor; }

  void send_command(struct DataFrame command);
  bool send_raw_frame_from_text(const std::string &frame_text);
  
  // Autonomous mode **********************************
  
  void set_autonomous(bool v) { autonomous_ = v; }
  void send_ping();
  void send_read_block(uint8_t opcode2, uint16_t start, uint16_t length);
  void remote_announce();
  void tu2c_remote_announce();
  void tu2c_send_ping();
  bool is_tu2c_registration_ack_(const DataFrame *frame) const;
  bool is_tu2c_registration_query_(const DataFrame &frame) const;
  void set_read_only(bool en) { read_only_ = en; }
  void set_ping_enabled(bool en) { ping_enabled_ = en; }
  void set_autoreset_errors(bool en) { autoreset_errors_ = en; }
  void set_estia_source_address(uint16_t addr) { estia_source_address_ = addr; }
  void send_estia_setpoint(float target_temp);
  void send_estia_power(bool on);
  void send_estia_mode(uint8_t mode_cmd);  // 0x02=heat, 0x01=cool
  void send_estia_demand(uint8_t demand);  // 0-10V demand (0..15)
  void send_estia_demand_heartbeat();      // periodic 0x55 as 0x0041
  void set_demand_enabled(bool en) { demand_enabled_ = en; }
  void send_estia_data_request(uint8_t subtype);
  void send_estia_first_gen_dhw_setpoint(float target_temp);
  void send_estia_first_gen_zone1(bool on);
  void send_estia_first_gen_dhw_boost(bool on);
  void send_estia_first_gen_request_data(uint8_t request_code);
  void send_estia_tracked_(const uint8_t *frame, size_t len, uint16_t ack_dtype);
  
  // Reporting external sensor temperature to AC *************************

  std::string ext_temp_sensor_name_;

  void set_ext_temp_source(sensor::Sensor *s) { ext_temp_sensor_ = s; }
  void set_ext_temp_interval(uint32_t ms) { ext_temp_interval_ms_ = ms; }
  void set_ext_temp_enabled(bool en) { ext_temp_enabled_ = en; }
  void send_remote_temp(float temp_c);  // builds & enqueues remote-temp frame
  void set_ext_temp_sensor_name(const std::string &name) { this->ext_temp_sensor_name_ = name; }
  
  // AC sensors polling ************************************

  void set_current_sensor(sensor::Sensor *s) { current_sensor_ = s; } // Sensor for current, x10 A
  void send_sensor_query(uint8_t sensor_id); // Send sensor query for a specific sensor ID
  void add_polled_sensor(uint8_t id, float scale, uint32_t interval_ms, sensor::Sensor *sensor);

//*************************************
  bool control_vent(bool state);

  bool receive_data(const std::vector<uint8_t> data);
  bool receive_data_frame(const struct DataFrame *frame);

  void add_on_data_received_callback(std::function<void(const struct DataFrame *frame)> &&callback) {
    this->set_data_received_callback_.add(std::move(callback));
  }

 protected:
  climate::ClimateTraits traits_;

  // Hardware-UART-RX mode (see set_hardware_uart_rx_pin). Disabled by default
  // unless final validation detects the ESP8266 GPIO13 RX/software-TX case.
  bool hw_uart_rx_enabled_{false};
  uint8_t hw_uart_rx_pin_{0};

  DataFrameReader data_reader;
  TccState tcc_state;

  void process_received_data(const struct DataFrame *frame);
  void process_received_data_tu2c_(const struct DataFrame *frame);
  void process_received_data_estia_first_gen_(const DataFrame *frame);
  bool is_ack_for_pending_command_(const DataFrame *frame) const;
  bool is_tu2c_command_ack_frame_(const DataFrame *frame) const;
  bool should_track_tu2c_command_ack_(const DataFrame &frame) const;
  bool should_track_command_ack_(const DataFrame &frame) const;
  void handle_pending_command_ack_(const DataFrame *frame);
  size_t send_new_state(const struct TccState *new_state);
  void sync_from_received_state();
  void autoreset_remote_error_();
  bool is_own_tx_echo_(const DataFrame *f) const; //used to filter echo after sending frame
  void remember_tx_frame_for_echo_(const uint8_t *bytes, size_t size, bool tu2c);
  void update_frame_validation_();
  bool has_bus_quiet_time_elapsed_(uint32_t now) const;
  bool has_tu2c_quiet_time_elapsed_(uint32_t now) const;
  bool should_auto_detect_frame_format_() const { return frame_format_auto_ && !frame_format_confirmed_; }
  void confirm_frame_format_(FrameFormat format, const char *reason);
  void watch_auto_frame_format_byte_(uint8_t byte);
  bool maybe_auto_detect_hm_frame_(DataFrame *frame);
  bool is_hm_wire_frame_from_master_(const DataFrame *frame) const;


  std::vector<DataFrame> create_commands(const struct TccState *new_state);

  // callbacks
  CallbackManager<void(const struct DataFrame *frame)> set_data_received_callback_{};

  // sensors  ******************************************
  binary_sensor::BinarySensor *filter_alert_sensor_{nullptr};
  bool filter_alert_state_{false}; 
  binary_sensor::BinarySensor *connected_binary_sensor_{nullptr};
  switch_::Switch *vent_switch_{nullptr};
  switch_::Switch *read_only_switch_{nullptr};
  switch_::Switch *zone1_switch_{nullptr};
  switch_::Switch *dhw_boost_switch_{nullptr};
  sensor::Sensor *failed_crcs_sensor_{nullptr};

  sensor::Sensor *current_sensor_{nullptr}; // Sensor for current, x10 A

  // Estia sensors
  sensor::Sensor *outdoor_temp_sensor_{nullptr};
  sensor::Sensor *compressor_hours_sensor_{nullptr};
  sensor::Sensor *waterpump_hours_sensor_{nullptr};
  sensor::Sensor *backup_heater_hours_sensor_{nullptr};
  sensor::Sensor *demand_sensor_{nullptr};  // 0-10V interface demand (0..15)
  sensor::Sensor *zone1_water_temp_sensor_{nullptr};
  sensor::Sensor *zone1_target_temperature_sensor_{nullptr};

  // rx handler for 0x1A (sensor) replies (called from process_received_data)
  void process_sensor_value_(const DataFrame *frame);
  std::vector<PolledSensor> polled_sensors_;



  // Tracks the last sensor ID we queried via 0x17 (for short 0x1A replies)
  uint8_t last_sensor_query_id_{0xFF};     // 0xFF = invalid / none
  bool    sensor_query_outstanding_{false}; // true after send, cleared on reply
  uint32_t last_sensor_query_ms_{0};
  uint32_t sensor_query_timeout_ms_{2500};  // Give busy HM buses time to answer before releasing the guard.
  uint32_t sensor_query_timeouts_{0};       // (optional) stats

  // Pending-query ring buffer. Periodic intervals push sensor IDs here;
  // the loop drains one at a time once the previous query has been
  // answered (or timed out). Strict serialisation prevents the response
  // misattribution that occurs when multiple short-form 0x1A replies
  // (which carry no sensor ID) collide with a back-to-back set of 0x17
  // queries. Capacity is a power of two so head advancement uses a
  // bitmask instead of modulo. Fixed std::array (vs std::deque) keeps
  // it on the object — zero heap, ~18 bytes total.
  static constexpr size_t MAX_PENDING_SENSOR_QUERIES = 16;
  static_assert((MAX_PENDING_SENSOR_QUERIES & (MAX_PENDING_SENSOR_QUERIES - 1)) == 0,
                "MAX_PENDING_SENSOR_QUERIES must be a power of two");
  std::array<uint8_t, MAX_PENDING_SENSOR_QUERIES> pending_sensor_queries_{};
  uint8_t pending_head_{0};
  uint8_t pending_count_{0};
  // TX back-pressure threshold reused from the original sensor-query
  // gating; sensor queries are throttled when the write queue is this
  // deep so we don't pile commands behind a slow bus.
  static constexpr size_t WRITE_QUEUE_THROTTLE = 3;
  bool enqueue_sensor_query_(uint8_t id);
  void drain_sensor_query_queue_();

  // room temperature sensor reporting to AC  ******************************
  sensor::Sensor *ext_temp_sensor_{nullptr};
  uint32_t ext_temp_interval_ms_{300000};  // 5 min default
  bool ext_temp_enabled_{false};
  // Flag set when a broadcast announce ACK containing 0x0D is received
  bool announce_ack_received_{false};
  // If true, component will not send any commands to the central unit
  // (useful for read-only deployments). Default: false
  bool read_only_{false};
  bool ping_enabled_{true};
  bool autoreset_errors_{false};

  //autonomous mode **********************************
  bool autonomous_ = false;
  uint32_t ping_interval_ms_ = 30000;  // default ping interval
  uint32_t read08_interval_ms = 60000;  // interval to send 40:00:15:06:08:E8:00:01:00:9E:2C, not sure what it does, but it is sent every minute by remote in the logs
  uint8_t command_mode_read_{COMMAND_MODE_READ};
  uint8_t command_mode_write_{COMMAND_MODE_WRITE};

 private:
  uint32_t loops_without_reads_ = 0;
  uint32_t loops_with_reads_ = 0;
  uint32_t last_read_millis_ = 0;
  uint32_t last_sent_millis_ = 0;
  uint32_t packet_min_wait_millis_{PACKET_MIN_WAIT_MILLIS};
  uint32_t last_received_millis_ = 0;
  bool can_read_packet = false;

  bool frame_format_auto_{true};
  bool frame_format_confirmed_{false};
  uint8_t auto_a0_prefix_match_{0};
  bool auto_a0_collecting_{false};
  uint8_t auto_a0_buffer_[DATA_FRAME_MAX_SIZE + 2]{};
  uint16_t auto_a0_index_{0};
  uint16_t auto_a0_expected_total_{0};
  uint32_t auto_a0_last_byte_ms_{0};

  uint32_t last_received_frame_millis_ = 0;
  uint32_t last_sent_frame_millis_ = 0;
  uint32_t last_tu2c_received_frame_millis_ = 0;
  uint32_t last_tu2c_sent_frame_millis_ = 0;
  std::queue<DataFrame> write_queue_;
  std::queue<std::vector<uint8_t>> raw_write_queue_;
  optional<DataFrame> last_unconfirmed_command_;
  uint8_t last_unconfirmed_command_attempts_{0};
  bool resend_last_unconfirmed_command_{false};
  uint32_t last_unconfirmed_command_sent_ms_{0};
  optional<DataFrame> last_tx_frame_for_echo_;
  uint32_t last_tx_frame_millis_{0};
  static const uint32_t ECHO_MATCH_WINDOW_MS = 1500;

  static const uint8_t MAX_COMMAND_SEND_ATTEMPTS = 5;
  static const uint8_t TU2C_MAX_COMMAND_SEND_ATTEMPTS = 3;
  static const uint32_t TU2C_ACK_TIMEOUT_MS = 3000;
  static const uint32_t TU2C_FRAME_SEND_MILLIS_FROM_LAST_RECEIVE = 500;
  static const uint32_t TU2C_FRAME_SEND_MILLIS_FROM_LAST_SEND = 500;

  // Estia raw command ACK tracking
  std::vector<uint8_t> estia_pending_cmd_;        // last sent raw command (for retry)
  uint16_t estia_pending_ack_dtype_{0};            // expected ACK dtype (e.g. 0x03C0, 0x03C1, 0x0041, 0x005F)
  uint8_t estia_cmd_attempts_{0};
  uint32_t estia_cmd_sent_ms_{0};
  static const uint8_t ESTIA_MAX_CMD_ATTEMPTS = 3;
  static const uint32_t ESTIA_CMD_ACK_TIMEOUT_MS = 3000;

  uint32_t last_master_alive_millis_ = 0;
  bool estia_was_connected_{false};
  uint16_t estia_source_address_{0x0040};  // default: mimic remote controller
  climate::ClimateMode estia_last_active_mode_{climate::CLIMATE_MODE_HEAT};  // last known mode while powered on
  bool estia_power_on_pending_{false};  // waiting for mode ACK before sending power on
  uint8_t estia_pending_mode_cmd_{0};   // mode command to retry (0x01=cool, 0x02=heat)
  uint8_t estia_mode_retries_{0};       // retry counter for mode command
  void estia_mode_retry_timeout_();
  uint32_t estia_last_e8c0_ms_{0};
  uint32_t estia_last_e8c1_ms_{0};
  static const uint32_t ESTIA_E8C0_INTERVAL_MS = 30000;
  static const uint32_t ESTIA_E8C1_INTERVAL_MS = 60000;
  static const uint32_t ESTIA_DEMAND_HEARTBEAT_MS = 5000;  // 0-10V heartbeat every 5s
  bool demand_enabled_{false};
  uint8_t estia_demand_value_{0};         // current demand value (0..15)
  uint32_t estia_last_demand_heartbeat_ms_{0};

  bool estia_first_gen_pump_state_known_{false};
  bool estia_first_gen_zone1_active_{false};
  bool estia_first_gen_dhw_active_{false};
  bool estia_first_gen_dhw_boost_{false};
  uint8_t estia_first_gen_dhw_encoded_{0};
  uint8_t estia_first_gen_zone1_encoded_{0};
  uint32_t last_temp_log_time_ = 0;  // Counter for BME280 temperature logging
  float last_sent_temp_ = 1; // Last sent room temperature to the unit


};

class ToshibaAbVentSwitch : public switch_::Switch, public Component {
 public:
  ToshibaAbVentSwitch(ToshibaAbClimate *climate) { climate_ = climate; }

  //   void setup() override;
  //   void dump_config() override;

  //   void loop() override;

  //   float get_setup_priority() const override;

 protected:
  //   bool assumed_state() override;

  void write_state(bool state) override;

  ToshibaAbClimate *climate_;
};

class ToshibaAbEstiaZone1Switch : public switch_::Switch, public Component {
 public:
  ToshibaAbEstiaZone1Switch(ToshibaAbClimate *climate) { climate_ = climate; }
 protected:
  void write_state(bool state) override;
  ToshibaAbClimate *climate_;
};

class ToshibaAbEstiaDhwBoostSwitch : public switch_::Switch, public Component {
 public:
  ToshibaAbEstiaDhwBoostSwitch(ToshibaAbClimate *climate) { climate_ = climate; }
 protected:
  void write_state(bool state) override;
  ToshibaAbClimate *climate_;
};

class ToshibaAbReadOnlySwitch : public switch_::Switch, public Component {
 public:
  ToshibaAbReadOnlySwitch(ToshibaAbClimate *climate) { climate_ = climate; }
 protected:
  void write_state(bool state) override;
  ToshibaAbClimate *climate_;
};

}  // namespace toshiba_ab


}  // namespace esphome
