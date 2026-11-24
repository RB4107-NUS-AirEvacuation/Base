#include "mid_hinge.h"

MidHinge::MidHinge(DrvMotor& motor, MidHallStop& stop, const HingeParams& p)
  : _motor(motor), _stop(stop), _p(p), _target(0), _cur(0), _out(0), _blocked(false) {
  _p.towardStopSign = (_p.towardStopSign < 0) ? -1 : 1;
}

void MidHinge::begin() {
  _motor.begin();
}

void MidHinge::command(int pwm) {
  if (pwm > _p.maxPwm)  pwm = _p.maxPwm;
  if (pwm < -_p.maxPwm) pwm = -_p.maxPwm;
  _target = pwm;
}

void MidHinge::stop() {
  _target = 0;
  _cur = 0;
  _out = 0;
  _blocked = false;
  _motor.setCommand(0);
}

void MidHinge::update(float dtSec) {
  float step = _p.slewPerSec * dtSec;
  if (_cur < (float)_target) {
    _cur += step;
    if (_cur > (float)_target) _cur = (float)_target;
  } else if (_cur > (float)_target) {
    _cur -= step;
    if (_cur < (float)_target) _cur = (float)_target;
  }

  int cmd = (int)(_cur >= 0.0f ? _cur + 0.5f : _cur - 0.5f);
  int gated = _stop.gate(cmd, _p.towardStopSign);

  bool toward = (_p.towardStopSign > 0) ? (cmd > 0) : (cmd < 0);
  _blocked = toward && gated == 0;
  if (_blocked) {
    // At the stop: drop the command and the ramp so it cannot restart on its own.
    _target = 0;
    _cur = 0.0f;
  }

  _out = gated;
  _motor.setCommand(_p.invert ? -gated : gated);
}

int MidHinge::output() const {
  return _out;
}
