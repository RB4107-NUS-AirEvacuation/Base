#include "mid_pi.h"

MidPI::MidPI(float kp, float ki, float outMin, float outMax)
  : _kp(kp), _ki(ki), _outMin(outMin), _outMax(outMax), _integral(0) {}

void MidPI::reset() {
  _integral = 0;
}

float MidPI::update(float error, float dtSec) {
  float trial = _integral + error * dtSec;
  float out = _kp * error + _ki * trial;
  if (out > _outMax) {
    out = _outMax;                       // saturated: do not integrate further
  } else if (out < _outMin) {
    out = _outMin;
  } else {
    _integral = trial;
  }
  return out;
}
