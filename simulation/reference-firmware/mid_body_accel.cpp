#include "mid_body_accel.h"
#include "mid_quat.h"
#include <math.h>

static const float TWO_PI_F = 6.2831853f;

MidBodyAccel::MidBodyAccel(DrvBno08x& imu, const BodyAccelParams& p)
  : _imu(imu), _p(p), _rc(0.0f), _up(0.0f), _fwd(0.0f) {
  _rc = (_p.cutoffHz > 0.0f) ? 1.0f / (TWO_PI_F * _p.cutoffHz) : 0.0f;
}

void MidBodyAccel::update(float dtSec) {
  const BnoQuat& q = _imu.rotation();

  // Acceleration in the world frame.
  BnoVec a = quatRotate(q, _imu.linearAccel());

  // The robot's forward direction, flattened onto the floor plane.
  BnoVec f = quatRotate(q, robotForwardAxis(_p.forwardIsY, _p.reversed));
  float len = sqrtf(f.x * f.x + f.y * f.y);

  float up = a.z;
  float fwd = (len > 0.1f) ? (a.x * f.x + a.y * f.y) / len : 0.0f;   // robot standing on end: no "forward"

  float k = dtSec / (_rc + dtSec);                 // first-order low-pass (k = 1 when off)
  _up += k * (up - _up);
  _fwd += k * (fwd - _fwd);
}
