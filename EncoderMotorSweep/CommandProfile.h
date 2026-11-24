#pragma once
#include <Arduino.h>

// Piecewise-linear command that loops through its waypoints.
// {-max, 0, -max} gives -max -> 0 -> -max, then repeats.
class CommandProfile {
public:
  CommandProfile(const int* waypoints, uint8_t count, unsigned long segmentMs);
  void start(unsigned long nowMs);
  int  valueAt(unsigned long nowMs) const;

private:
  const int* _wp;
  uint8_t _n;
  unsigned long _segMs;
  unsigned long _startMs = 0;
};
