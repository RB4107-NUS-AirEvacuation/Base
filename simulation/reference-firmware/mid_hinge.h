#pragma once
// MIDDLEWARE LAYER: hinge motor, open loop (no encoder).
// Command is a signed PWM. It is ramped (slew limited) and then passed through
// the Hall end stop, so the hinge can never be driven up past straight.
#include "drv_motor.h"
#include "mid_hall_stop.h"

struct HingeParams {
  bool  invert;          // flip motor polarity
  int   towardStopSign;  // +1 or -1: sign of a logical command that moves toward the Hall stop (hinge up)
  int   maxPwm;
  float slewPerSec;      // max PWM change per second
};

class MidHinge {
public:
  MidHinge(DrvMotor& motor, MidHallStop& stop, const HingeParams& p);

  void begin();
  void command(int pwm);        // logical command, clamped to +/- maxPwm
  void stop();                  // command 0 and ramp state cleared (immediate)
  void update(float dtSec);     // call once per control cycle, after the stop has been updated

  int command() const { return _target; }
  int output() const;           // logical PWM actually applied to the motor
  bool blocked() const { return _blocked; }

private:
  DrvMotor& _motor;
  MidHallStop& _stop;
  HingeParams _p;
  int _target;
  float _cur;
  int _out;
  bool _blocked;
};
