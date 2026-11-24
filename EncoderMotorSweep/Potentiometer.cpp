#include "Potentiometer.h"

Potentiometer::Potentiometer(uint8_t pin,
                             uint16_t adcMin, uint16_t adcMax,
                             float degMin, float degMax,
                             uint8_t samples)
  : _pin(pin), _adcMin(adcMin), _adcMax(adcMax),
    _degMin(degMin), _degMax(degMax), _samples(samples) {}

uint16_t Potentiometer::readRaw() {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < _samples; i++) {
    sum += analogRead(_pin);
  }
  return sum / _samples;
}

float Potentiometer::readDegrees() {
  float raw = readRaw();
  return _degMin + (raw - _adcMin) * (_degMax - _degMin) / (float)(_adcMax - _adcMin);
}
