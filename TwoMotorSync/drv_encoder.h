#pragma once
#include <Arduino.h>

// DRIVER LAYER: quadrature encoder.
// ESP32 (e.g. ESP32-C6): any GPIO can interrupt, so any two pins work, any number of encoders.
// Arduino Uno: pins 2/3 use external interrupts, A0..A5 use pin-change interrupts (PCINT1),
//              up to two of each.
#if defined(ESP32)
  #define DRV_ISR_ATTR IRAM_ATTR
#else
  #define DRV_ISR_ATTR
#endif

class DrvEncoder {
public:
  DrvEncoder(uint8_t pinA, uint8_t pinB);
  void begin();
  long count() const;     // atomic read of the signed count
  void onEdge();          // called from interrupt context

private:
  uint8_t _a;
  uint8_t _b;
  volatile long _count;
  volatile uint8_t _prev;
};
