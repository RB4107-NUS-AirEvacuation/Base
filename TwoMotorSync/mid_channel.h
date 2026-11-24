#pragma once
#include "drv_motor.h"
#include "drv_encoder.h"
#include "mid_speed.h"
#include "mid_pi.h"

struct ChannelParams {
  bool invert;
  float kp;
  float ki;
  int maxPwm;
  unsigned long windowMs;
  float alpha;
};

// MIDDLEWARE: one motor channel = motor driver + encoder + speed estimate + velocity loop.
class MidChannel {
public:
  MidChannel(DrvMotor& motor, DrvEncoder& enc, const ChannelParams& p);
  void begin();
  void measure(unsigned long nowMs);            // updates speed from the encoder
  void drive(float refSpeed, float dtSec);      // velocity loop -> PWM output
  void stop();
  float speed() const { return _speed; }        // counts/s in the logical (non-inverted) frame
  int output() const;                           // last PWM command, logical frame
  long position() const { return signedCount(); }   // encoder counts, logical frame

private:
  long signedCount() const;

  DrvMotor& _motor;
  DrvEncoder& _enc;
  bool _inv;
  MidSpeed _est;
  MidPI _pi;
  float _speed;
};
