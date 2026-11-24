#pragma once
// MIDDLEWARE LAYER: one operator command. Every input (serial line, knob menu)
// produces these, and the app applies them in one place.
#include <stdint.h>

struct Command {
  enum Kind : uint8_t { NONE = 0, DRIVE = 1, HINGE = 2, DESCEND = 3 };

  Kind  kind;
  float value;    // DRIVE: track speed, counts/s (0 = stop everything)
                  // HINGE: hinge PWM, signed
                  // DESCEND: unused (start the go-down-a-stair sequence)

  static Command none() { return {NONE, 0.0f}; }
};
