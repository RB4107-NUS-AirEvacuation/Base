#include "mid_speed.h"

MidSpeed::MidSpeed(unsigned long windowMs, float alpha)
  : _window(windowMs), _alpha(alpha), _lastCount(0), _lastMs(0), _speed(0), _primed(false) {}

void MidSpeed::reset(long count, unsigned long nowMs) {
  _lastCount = count;
  _lastMs = nowMs;
  _speed = 0;
  _primed = true;
}

float MidSpeed::update(long count, unsigned long nowMs) {
  if (!_primed) {
    reset(count, nowMs);
    return _speed;
  }
  unsigned long dt = nowMs - _lastMs;
  if (dt < _window) return _speed;     // only recompute once per window
  float raw = (float)(count - _lastCount) * 1000.0f / (float)dt;
  _speed = _alpha * raw + (1.0f - _alpha) * _speed;
  _lastCount = count;
  _lastMs = nowMs;
  return _speed;
}
