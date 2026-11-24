#pragma once
#include <Arduino.h>

// Maps a potentiometer angle to a signed motor command.
// Center of travel = stop. Each side of center drives one direction,
// with speed proportional to how far the knob is turned.
class ManualControl {
public:
  ManualControl(float degMin, float degMax, int maxCmd, float deadband);
  int command(float degrees) const;   // -maxCmd .. +maxCmd

private:
  float _center;
  float _half;
  int   _maxCmd;
  float _deadband;
};
