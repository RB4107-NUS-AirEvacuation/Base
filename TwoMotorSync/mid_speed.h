#pragma once
#include <Arduino.h>

// MIDDLEWARE: turns encoder counts into a filtered speed in counts per second.
class MidSpeed {
public:
  MidSpeed(unsigned long windowMs, float alpha);
  void reset(long count, unsigned long nowMs);
  float update(long count, unsigned long nowMs);   // returns filtered counts/s
  float value() const { return _speed; }

private:
  unsigned long _window;
  float _alpha;
  long _lastCount;
  unsigned long _lastMs;
  float _speed;
  bool _primed;
};
