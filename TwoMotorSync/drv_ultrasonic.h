#pragma once
// DRIVER LAYER: HC-SR04 style ultrasonic range sensor (TRIG + ECHO), read by polling.
// read() sends one ping and busy-waits for the echo with pulseIn(). No interrupts.
// It blocks for at most the echo time of maxCm (about 58 us per cm: ~7 ms at 100 cm).
#include <Arduino.h>

class DrvUltrasonic {
public:
  DrvUltrasonic(uint8_t trigPin, uint8_t echoPin, uint16_t maxCm);

  void begin();

  // Send one ping and wait for its echo. Returns true if an echo within maxCm came back.
  // Leave at least ~50 ms between calls so the echoes of the last ping have died away.
  bool read();

  bool ok() const;      // true: the last read() got an echo
  float cm() const;     // last good distance, cm (kept when a read fails)

private:
  uint8_t _trig;
  uint8_t _echo;
  uint16_t _maxCm;
  unsigned long _timeoutUs;
  float _cm;
  bool _ok;
};
