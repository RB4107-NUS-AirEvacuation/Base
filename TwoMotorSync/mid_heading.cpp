#include "mid_heading.h"
#include "mid_quat.h"
#include <math.h>

static const float RAD2DEG = 57.29578f;

static float clampf(float v, float lim) { return v > lim ? lim : (v < -lim ? -lim : v); }

static float wrap180(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

MidHeading::MidHeading(DrvBno08x& imu, const HeadingParams& p)
  : _imu(imu), _p(p), _set(0.0f), _yaw(0.0f), _err(0.0f), _integral(0.0f), _turn(0.0f) {}

// false when the forward axis points (nearly) straight up or down: no heading then.
bool MidHeading::readYaw(float& yawDeg, float& rateDps) const {
  const BnoQuat& q = _imu.rotation();
  BnoVec f = quatRotate(q, robotForwardAxis(_p.forwardIsY, _p.reversed));
  if (f.x * f.x + f.y * f.y < 0.01f) return false;
  yawDeg = atan2f(f.y, f.x) * RAD2DEG;
  rateDps = quatRotate(q, _imu.gyro()).z * RAD2DEG;    // rotation about world vertical
  return true;
}

void MidHeading::holdCurrent() {
  float rate;
  readYaw(_set, rate);
  _yaw = _set;
  _err = 0.0f;
  _integral = 0.0f;
}

void MidHeading::update(float dtSec, bool engaged, bool imuOk) {
  float rate = 0.0f;
  bool valid = imuOk && readYaw(_yaw, rate);
  _err = wrap180(_set - _yaw);

  if (!engaged || !valid) {
    _integral = 0.0f;
    _turn = 0.0f;
    return;
  }

  _integral = clampf(_integral + _p.ki * _err * dtSec, _p.maxTurn);   // anti-windup: I alone never exceeds the limit
  float p = _p.kp * _err;
  float d = -_p.kd * rate;                       // d(error)/dt = -yaw rate (setpoint is constant)
  _turn = clampf(p + _integral + d, _p.maxTurn);
}
