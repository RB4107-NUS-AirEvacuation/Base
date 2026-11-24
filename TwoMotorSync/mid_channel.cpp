#include "mid_channel.h"

MidChannel::MidChannel(DrvMotor& motor, DrvEncoder& enc, const ChannelParams& p)
  : _motor(motor), _enc(enc), _inv(p.invert),
    _est(p.windowMs, p.alpha), _pi(p.kp, p.ki, -p.maxPwm, p.maxPwm), _speed(0) {}

void MidChannel::begin() {
  _motor.begin();
  _enc.begin();
  _est.reset(signedCount(), millis());
}

long MidChannel::signedCount() const {
  long c = _enc.count();
  return _inv ? -c : c;
}

void MidChannel::measure(unsigned long nowMs) {
  _speed = _est.update(signedCount(), nowMs);
}

void MidChannel::drive(float refSpeed, float dtSec) {
  float u = _pi.update(refSpeed - _speed, dtSec);
  int out = (int)(u >= 0.0f ? u + 0.5f : u - 0.5f);
  _motor.setCommand(_inv ? -out : out);
}

void MidChannel::stop() {
  _pi.reset();
  _motor.setCommand(0);
}

int MidChannel::output() const {
  return _inv ? -_motor.command() : _motor.command();
}
