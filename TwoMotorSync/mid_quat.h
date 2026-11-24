#pragma once
// MIDDLEWARE LAYER: small quaternion helpers shared by the IMU users.
#include "drv_bno08x.h"

// Rotate v from the board frame into the world frame (Z up) with unit quaternion q:
//   v' = v + 2w (u x v) + 2 u x (u x v),  u = (q.x, q.y, q.z)
inline BnoVec quatRotate(const BnoQuat& q, const BnoVec& v) {
  float tx = 2.0f * (q.y * v.z - q.z * v.y);       // t = 2 (u x v)
  float ty = 2.0f * (q.z * v.x - q.x * v.z);
  float tz = 2.0f * (q.x * v.y - q.y * v.x);
  return {
    v.x + q.w * tx + (q.y * tz - q.z * ty),
    v.y + q.w * ty + (q.z * tx - q.x * tz),
    v.z + q.w * tz + (q.x * ty - q.y * tx)
  };
}

// The robot's forward direction in the board frame: the board's X or Y arrow,
// or the opposite way if the arrow points to the back.
inline BnoVec robotForwardAxis(bool forwardIsY, bool reversed) {
  float s = reversed ? -1.0f : 1.0f;
  return forwardIsY ? BnoVec{0.0f, s, 0.0f} : BnoVec{s, 0.0f, 0.0f};
}
