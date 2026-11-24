#include "drv_hall.h"

DrvHall::DrvHall(uint8_t pin, uint8_t samples)
  : _pin(pin), _samples(samples), _zero(ADC_MAX / 2) {
  if (_samples < 1)  _samples = 1;
  if (_samples > 64) _samples = 64;     // 64 * 1023 fits the Uno's 16-bit sum
}

void DrvHall::begin() {
  pinMode(_pin, INPUT);
#if defined(ESP32)
  analogReadResolution(12);
  analogSetPinAttenuation(_pin, ADC_11db);   // full ~0..3.1 V input range
#endif
  analogRead(_pin);                     // discard the first conversion after power-up
}

int DrvHall::raw() const {
  unsigned long sum = 0;
  for (uint8_t i = 0; i < _samples; i++) {
    sum += (unsigned long)analogRead(_pin);
  }
  return (int)((sum + _samples / 2) / _samples);
}

void DrvHall::calibrateZero() {
  _zero = raw();
}

void DrvHall::setZero(int zeroRaw) {
  _zero = zeroRaw;
}

int DrvHall::zero() const {
  return _zero;
}

int DrvHall::value() const {
  return raw() - _zero;
}

int DrvHall::strength() const {
  int v = value();
  return v < 0 ? -v : v;
}

float DrvHall::volts(float vref) const {
  return (float)raw() * vref / (float)ADC_MAX;
}
