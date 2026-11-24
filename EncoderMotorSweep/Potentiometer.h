#pragma once
#include <Arduino.h>

// Analog potentiometer read with averaging and linear calibration to degrees.
class Potentiometer {
public:
  Potentiometer(uint8_t pin,
                uint16_t adcMin, uint16_t adcMax,
                float degMin, float degMax,
                uint8_t samples = 8);

  uint16_t readRaw();        // averaged ADC value, 0..1023
  float    readDegrees();    // calibrated angle

private:
  uint8_t  _pin;
  uint16_t _adcMin;
  uint16_t _adcMax;
  float    _degMin;
  float    _degMax;
  uint8_t  _samples;
};
