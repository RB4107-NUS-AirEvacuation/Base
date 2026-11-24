#include "drv_ultrasonic.h"

// Sound travels about 0.0343 cm/us, and the echo time covers the trip down and back.
static const float CM_PER_US = 0.0343f / 2.0f;
// The sensor raises ECHO a few hundred us after TRIG, once its 40 kHz burst is sent.
static const unsigned long ECHO_START_US = 1000;

DrvUltrasonic::DrvUltrasonic(uint8_t trigPin, uint8_t echoPin, uint16_t maxCm)
  : _trig(trigPin), _echo(echoPin), _maxCm(maxCm),
    _timeoutUs((unsigned long)(maxCm / CM_PER_US) + ECHO_START_US),
    _cm(0.0f), _ok(false) {}

void DrvUltrasonic::begin() {
  pinMode(_trig, OUTPUT);
  digitalWrite(_trig, LOW);
  pinMode(_echo, INPUT);
}

bool DrvUltrasonic::read() {
  // ECHO still high: the sensor is still timing an earlier out-of-range ping.
  // Skip this read rather than wait up to ~38 ms for it to finish.
  if (digitalRead(_echo) == HIGH) {
    _ok = false;
    return false;
  }

  digitalWrite(_trig, HIGH);               // a 10 us pulse starts one ping
  delayMicroseconds(10);
  digitalWrite(_trig, LOW);

  unsigned long us = pulseIn(_echo, HIGH, _timeoutUs);   // 0 = no echo in time
  float d = us * CM_PER_US;
  _ok = (us > 0) && (d <= _maxCm);
  if (_ok) _cm = d;
  return _ok;
}

bool DrvUltrasonic::ok() const {
  return _ok;
}

float DrvUltrasonic::cm() const {
  return _cm;
}
