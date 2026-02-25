#pragma once

#include <Arduino.h>

// Operations (byte 0 of the incoming packet)
constexpr uint8_t OP_BEGIN         = 0x1;
constexpr uint8_t OP_PRINT         = 0x2;
constexpr uint8_t OP_ARUCO         = 0x3;
constexpr uint8_t OP_MISSION       = 0x4;
constexpr uint8_t OP_ML_PREDICTION = 0x5;
constexpr uint8_t OP_ML_CAPTURE    = 0x6;
constexpr uint8_t OP_IS_CONNECTED  = 0x7;
constexpr uint8_t OP_PRED          = 0x8;

// Marks the end of a packet coming from the Arduino.
constexpr uint8_t FLUSH_SEQUENCE[4] = {0xFF, 0xFE, 0xFD, 0xFC};

inline bool endsWithFlushSequence(const char* buf, uint16_t len) {
  if (len < 4) return false;
  return (static_cast<uint8_t>(buf[len - 4]) == FLUSH_SEQUENCE[0] &&
          static_cast<uint8_t>(buf[len - 3]) == FLUSH_SEQUENCE[1] &&
          static_cast<uint8_t>(buf[len - 2]) == FLUSH_SEQUENCE[2] &&
          static_cast<uint8_t>(buf[len - 1]) == FLUSH_SEQUENCE[3]);
}
