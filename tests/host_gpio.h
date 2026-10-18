#pragma once
constexpr int OUTPUT = 1;
constexpr int LOW = 0;
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}

// Match the ESP8266 Arduino function-like macro in host regression builds.
#define word(...) makeWord(__VA_ARGS__)
