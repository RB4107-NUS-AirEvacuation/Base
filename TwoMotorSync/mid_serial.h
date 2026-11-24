#pragma once
#include <Arduino.h>
#include "mid_command.h"

// MIDDLEWARE: parses one line from Serial into a command.
//   Tracks (counts/s):  F = +default speed, R = -default speed, S = stop, or a signed integer (clamped).
//   Hinge (PWM):        HF = +default, HR = -default, HS = stop, or H<integer>, e.g. H40, H-40 (clamped).
//   Descend one stair:  D
class MidSerial {
public:
  MidSerial(float defaultSpeed, float maxSpeed, int hingeDefault, int hingeMax);

  // Returns Command::NONE until a full line has been read.
  Command poll();

private:
  bool parseDrive(const char* s, float& out) const;
  bool parseHinge(const char* s, float& out) const;
  float _def;
  float _maxSpeed;
  int _hdef;
  int _hmax;
  char _buf[16];
  uint8_t _len;
};
