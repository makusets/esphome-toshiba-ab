#pragma once

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/uart/uart.h"
#include <algorithm>
#include <bitset>
#include <queue>

namespace esphome {
namespace toshiba_ab {

const uint32_t ALIVE_MESSAGE_PERIOD_MILLIS = 5000;
const uint32_t LAST_ALIVE_TIMEOUT_MILLIS = ALIVE_MESSAGE_PERIOD_MILLIS * 3 +1000;

const uint32_t PACKET_MIN_WAIT_MILLIS = 200;
const uint32_t FRAME_SEND_MILLIS_FROM_LAST_RECEIVE = 500;
const uint32_t FRAME_SEND_MILLIS_FROM_LAST_SEND = 500;

// const uint8_t TOSHIBA_MASTER = 0x00;  replaced by master_address_ which is set up in yaml
const uint8_t TOSHIBA_REMOTE = 0x40;
const uint8_t TOSHIBA_TU2C_REMOTE_DEFAULT = 0x50;
const uint8_t TOSHIBA_TU2C_MASTER_DEFAULT = 0x90;
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
};

enum class FrameFormat : uint8_t {
  NORMAL = 0,
  TU2C = 1,
};
constexpr FrameFormat NORMAL = FrameFormat::NORMAL;
constexpr FrameFormat TU2C = FrameFormat::TU2C;

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

  void reset() {
    reset_frame_state_();
    tu2c_      = false;
    prefix_match_ = 0;
    tu2c_len_pending_ = false;
    tu2c_expected_total_ = 0;
    tu2c_bytes_seen_ = 0;
  }

  bool put(uint8_t byte) {
    const bool use_tu2c = frame_format_ == FrameFormat::TU2C;
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
    // Store byte
    frame.raw[data_index_] = current_byte;

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

    data_index_++;

    // If we know how many bytes we expect, finish only when we have them all
    if (expected_total_ > 0 && data_index_ >= expected_total_) {
      if (!use_tu2c) {
        // We have the whole frame (header + payload + CRC)
        crc_valid = frame.validate_crc();
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

  void set_filter_frames(bool filter_frames) { filter_frames_ = filter_frames; }
  bool filter_frames() const { return filter_frames_; }
  void set_allow_same_source_dest(bool allow_same) { allow_same_source_dest_ = allow_same; }
  void set_frame_format(FrameFormat format) {
    frame_format_ = format;
    reset();
  }
  FrameFormat frame_format() const { return frame_format_; }

private:
  void reset_frame_state_() {
    frame.reset();
    frame.set_tu2c(false);
    crc_valid       = false;
    complete        = false;
    data_index_     = 0;
    expected_total_ = 0;
    tu2c_len_pending_ = false;
    tu2c_expected_total_ = 0;
    tu2c_bytes_seen_ = 0;
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
  bool master_address_auto_{true};
  uint8_t tu2c_remote_address_{TOSHIBA_TU2C_REMOTE_DEFAULT};
  uint8_t tu2c_master_address_{TOSHIBA_TU2C_MASTER_DEFAULT};
  bool filter_frames_{true};
  void set_master_address(uint8_t address);
  void set_master_address_auto(bool auto_detect) {
    master_address_auto_ = auto_detect;
  }
  bool get_master_address_auto() const { return master_address_auto_; }
  void set_filter_frames(bool filter_frames) {
    filter_frames_ = filter_frames;
    data_reader.set_filter_frames(filter_frames);
  }
  bool get_filter_frames() const { return filter_frames_; }
  void set_command_mode_read(uint8_t value) { command_mode_read_ = value; }
  void set_command_mode_write(uint8_t value) { command_mode_write_ = value; }
  void set_filter_alert_sensor(binary_sensor::BinarySensor *sensor) { filter_alert_sensor_ = sensor; }
  void set_frame_format(FrameFormat format) { data_reader.set_frame_format(format); }



  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;

  void set_connected_binary_sensor(binary_sensor::BinarySensor *connected_binary_sensor) {
    connected_binary_sensor_ = connected_binary_sensor;
  }

  void set_vent_switch(switch_::Switch *vent_switch) { vent_switch_ = vent_switch; }

  void set_read_only_switch(switch_::Switch *read_only_switch) { read_only_switch_ = read_only_switch; }

  void set_failed_crcs_sensor(sensor::Sensor *failed_crcs_sensor) { this->failed_crcs_sensor_ = failed_crcs_sensor; }

  void send_command(struct DataFrame command);
  
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

  DataFrameReader data_reader;
  TccState tcc_state;

  void process_received_data(const struct DataFrame *frame);
  void process_received_data_tu2c_(const struct DataFrame *frame);
  bool is_ack_for_pending_command_(const DataFrame *frame) const;
  bool should_track_command_ack_(const DataFrame &frame) const;
  void handle_pending_command_ack_(const DataFrame *frame);
  size_t send_new_state(const struct TccState *new_state);
  void sync_from_received_state();
  bool is_own_tx_echo_(const DataFrame *f) const; //used to filter echo after sending frame
  void update_frame_validation_();


  std::vector<DataFrame> create_commands(const struct TccState *new_state);

  // callbacks
  CallbackManager<void(const struct DataFrame *frame)> set_data_received_callback_{};

  // sensors  ******************************************
  binary_sensor::BinarySensor *filter_alert_sensor_{nullptr};
  bool filter_alert_state_{false}; 
  binary_sensor::BinarySensor *connected_binary_sensor_{nullptr};
  switch_::Switch *vent_switch_{nullptr};
  switch_::Switch *read_only_switch_{nullptr};
  sensor::Sensor *failed_crcs_sensor_{nullptr};

  sensor::Sensor *current_sensor_{nullptr}; // Sensor for current, x10 A

  // rx handler for 0x1A (sensor) replies (called from process_received_data)
  void process_sensor_value_(const DataFrame *frame);
  std::vector<PolledSensor> polled_sensors_;



  // Tracks the last sensor ID we queried via 0x17 (for short 0x1A replies)
  uint8_t last_sensor_query_id_{0xFF};     // 0xFF = invalid / none
  bool    sensor_query_outstanding_{false}; // true after send, cleared on reply
  uint32_t last_sensor_query_ms_{0};
  uint32_t sensor_query_timeout_ms_{1000};  // 3s default; adjust if needed
  uint32_t sensor_query_timeouts_{0};       // (optional) stats

  // room temperature sensor reporting to AC  ******************************
  sensor::Sensor *ext_temp_sensor_{nullptr};
  uint32_t ext_temp_interval_ms_{300000};  // 5 min default
  bool ext_temp_enabled_{false};
  // Flag set when a broadcast announce ACK containing 0x0D is received
  bool announce_ack_received_{false};
  // If true, component will not send any commands to the central unit
  // (useful for read-only deployments). Default: false
  bool read_only_{false};

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
  uint32_t last_received_millis_ = 0;
  bool can_read_packet = false;

  uint32_t last_received_frame_millis_ = 0;
  uint32_t last_sent_frame_millis_ = 0;
  std::queue<DataFrame> write_queue_;
  optional<DataFrame> last_unconfirmed_command_;
  uint8_t last_unconfirmed_command_attempts_{0};
  bool resend_last_unconfirmed_command_{false};

  static const uint8_t MAX_COMMAND_SEND_ATTEMPTS = 5;

  uint32_t last_master_alive_millis_ = 0;

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

class ToshibaAbReadOnlySwitch : public switch_::Switch, public Component {
 public:
  ToshibaAbReadOnlySwitch(ToshibaAbClimate *climate) { climate_ = climate; }
 protected:
  void write_state(bool state) override;
  ToshibaAbClimate *climate_;
};

}  // namespace toshiba_ab


}  // namespace esphome
