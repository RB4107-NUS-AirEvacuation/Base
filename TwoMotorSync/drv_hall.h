#pragma once
// DRIVER LAYER: analog (linear) Hall sensor, e.g. SS49E / 49E.
// The output sits near half the supply with no magnet and moves up or down
// with field strength and polarity. Wire VCC -> 5V on the Uno, 3.3V on an ESP32
// (ESP32 pins are NOT 5 V tolerant), GND -> GND, OUT -> analog pin.
#include <Arduino.h>

class DrvHall {
public:
  // samples: ADC readings averaged per read (1..64). More = less noise, slower.
  DrvHall(uint8_t pin, uint8_t samples = 8);

  void begin();

  // Averaged ADC reading, 0..ADC_MAX.
  int raw() const;

  // Store the current reading as the "no magnet" level. Call with the magnet away.
  void calibrateZero();
  void setZero(int zeroRaw);
  int zero() const;

  // Signed field reading in ADC counts relative to the zero level.
  // Sign follows magnet polarity; magnitude grows as the magnet gets closer.
  int value() const;

  // Field strength regardless of polarity, in ADC counts.
  int strength() const;

  // Sensor output voltage, for checking against the datasheet.
  float volts(float vref = ADC_VREF) const;

#if defined(ESP32)
  static constexpr int ADC_MAX = 4095;    // 12-bit
  static constexpr float ADC_VREF = 3.3f;
#else
  static constexpr int ADC_MAX = 1023;    // 10-bit
  static constexpr float ADC_VREF = 5.0f;
#endif

private:
  uint8_t _pin;
  uint8_t _samples;
  int _zero;
};
