#pragma once
// Auto-generated build_info declarations
#include <cstddef>
#include <cstdint>
#include <ctime>
#ifdef USE_ESP8266
#include <pgmspace.h>
#endif

namespace esphome {
extern const uint32_t ESPHOME_CONFIG_HASH;
extern const time_t ESPHOME_BUILD_TIME;
extern const size_t ESPHOME_COMMENT_SIZE;
#ifdef USE_ESP8266
extern const char ESPHOME_BUILD_TIME_STR[] PROGMEM;
extern const char ESPHOME_COMMENT_STR[] PROGMEM;
#else
extern const char ESPHOME_BUILD_TIME_STR[];
extern const char ESPHOME_COMMENT_STR[];
#endif
}  // namespace esphome
