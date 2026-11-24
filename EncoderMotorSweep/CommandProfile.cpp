#include "CommandProfile.h"

CommandProfile::CommandProfile(const int* waypoints, uint8_t count, unsigned long segmentMs)
  : _wp(waypoints), _n(count), _segMs(segmentMs) {}

void CommandProfile::start(unsigned long nowMs) {
  _startMs = nowMs;
}

int CommandProfile::valueAt(unsigned long nowMs) const {
  const unsigned long period = (unsigned long)(_n - 1) * _segMs;
  const unsigned long t = (nowMs - _startMs) % period;
  const uint8_t seg = t / _segMs;
  const unsigned long local = t - (unsigned long)seg * _segMs;

  const long a = _wp[seg];
  const long b = _wp[seg + 1];
  return (int)(a + (b - a) * (long)local / (long)_segMs);
}
