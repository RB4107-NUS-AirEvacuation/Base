#pragma once
// DRIVER LAYER: push button wired to GND, using the internal pull-up.
// Raw level only, no debouncing (that is done in mid_knob).
#include <Arduino.h>

class DrvButton {
public:
  explicit DrvButton(uint8_t pin) : _pin(pin) {}
  void begin() { pinMode(_pin, INPUT_PULLUP); }
  bool down() const { return digitalRead(_pin) == LOW; }   // true while pressed

private:
  uint8_t _pin;
};
