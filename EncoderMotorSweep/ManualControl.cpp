#include "ManualControl.h"

ManualControl::ManualControl(float degMin, float degMax, int maxCmd, float deadband)
  : _center((degMin + degMax) * 0.5f),
    _half((degMax - degMin) * 0.5f),
    _maxCmd(maxCmd),
    _deadband(deadband) {}

int ManualControl::command(float degrees) const {
  // Normalize to -1..+1 around center
  float u = (degrees - _center) / _half;
  if (u > 1.0f)  u = 1.0f;
  if (u < -1.0f) u = -1.0f;

  float mag = (u < 0.0f) ? -u : u;
  if (mag <= _deadband) return 0;

  // Rescale so the output starts at 0 just outside the deadband
  float scaled = (mag - _deadband) / (1.0f - _deadband);
  int cmd = (int)(scaled * _maxCmd + 0.5f);
  return (u < 0.0f) ? -cmd : cmd;
}
