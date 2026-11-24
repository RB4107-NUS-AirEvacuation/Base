#pragma once
#include <Arduino.h>

// Quadrature encoder (A/B channels) with interrupt-based decoding.
class Encoder {
public:
  Encoder(uint8_t pinA, uint8_t pinB);
  void begin();
  long count() const;   // atomic read of the ISR-updated counter
  void reset();

private:
  void update();
  static void isrA();
  static void isrB();

  static Encoder* _instance;

  uint8_t _pinA;
  uint8_t _pinB;
  volatile long _count = 0;
  volatile uint8_t _prevAB = 0;
};
