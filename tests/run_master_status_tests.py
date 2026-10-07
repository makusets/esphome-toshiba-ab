#!/usr/bin/env python3
"""Exercise status recognition through the actual component's frame readers."""
from run_esp_address_tests import STUB, run_test

LOG_STUB = STUB.replace('#define ESP_LOGD(...) ((void) 0)', r'''
#include <cstdio>
extern std::vector<std::string> debug_logs;
template<typename... Args>
inline void capture_debug(const char *, const char *format, Args... args) {
  char buffer[2048];
  std::snprintf(buffer, sizeof(buffer), format, args...);
  debug_logs.emplace_back(buffer);
}
#define ESP_LOGD(...) capture_debug(__VA_ARGS__)
''')

if __name__ == "__main__":
    run_test("master_status_test.cpp", LOG_STUB)
