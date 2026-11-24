#pragma once

// MIDDLEWARE: PI controller with output clamping and conditional integration (anti-windup).
class MidPI {
public:
  MidPI(float kp, float ki, float outMin, float outMax);
  void reset();
  float update(float error, float dtSec);

private:
  float _kp;
  float _ki;
  float _outMin;
  float _outMax;
  float _integral;
};
