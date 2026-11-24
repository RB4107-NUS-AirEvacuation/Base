#include "mid_pitch.h"
#include <math.h>

static const float RAD2DEG = 57.29578f;
static const float TWO_PI_F = 6.2831853f;
static const float QUAT_NORM_TOLERANCE = 0.1f;   // |q|^2 must be within 1 +/- this
static const unsigned long RETRY_MS = 500;       // while stale, ask for the reports this often

MidPitch::MidPitch(DrvBno08x& imu, const PitchParams& p)
  : _imu(imu), _p(p), _rateRc(0), _pitch(0), _rate(0), _lastRate(0), _rejects(0),
    _lastSampleMs(0), _lastRetryMs(0), _ok(false) {
  _rateRc = (_p.rateCutoffHz > 0.0f) ? 1.0f / (TWO_PI_F * _p.rateCutoffHz) : 0.0f;
}

bool MidPitch::begin() {
  bool found = _imu.begin(_p.reportMs);
  _lastSampleMs = millis();
  _lastRetryMs = _lastSampleMs;
  _ok = false;                   // becomes true with the first good sample
  return found;
}

// The quaternion rotates the device frame into the world frame (Z up).
// The world Z component of a device axis tells how far that axis points up;
// minus that, as an angle, is how far it points down = nose-down pitch.
float MidPitch::forwardTiltDeg(const BnoQuat& q) const {
  float up = _p.forwardIsY
               ? 2.0f * (q.y * q.z + q.w * q.x)    // world Z of the device Y axis
               : 2.0f * (q.x * q.z - q.w * q.y);   // world Z of the device X axis
  if (up > 1.0f) up = 1.0f;
  if (up < -1.0f) up = -1.0f;
  return asinf(-up) * RAD2DEG;
}

// Rotation about the robot's sideways axis, nose-down positive.
// X forward: sideways is +Y (to the left), and +Y rotation tips the nose down.
// Y forward: sideways is +X (to the right), and +X rotation lifts the nose.
float MidPitch::pitchRateDps(const BnoVec& g) const {
  float r = _p.forwardIsY ? -g.x : g.y;
  return r * RAD2DEG;
}

void MidPitch::update(unsigned long nowMs, float dtSec) {
  if (!_imu.poll()) {
    // Nothing new this cycle. Normal now and then (the chip has its own clock);
    // only a long silence means trouble.
    if (nowMs - _lastSampleMs > _p.staleMs) {
      _ok = false;
      if (nowMs - _lastRetryMs >= RETRY_MS) {
        _lastRetryMs = nowMs;
        _imu.enableReports();
      }
    }
    return;
  }

  const BnoQuat& q = _imu.rotation();
  float norm2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
  if (fabsf(norm2 - 1.0f) > QUAT_NORM_TOLERANCE) return;   // corrupted sample: ignore it

  _lastSampleMs = nowMs;
  _ok = true;

  float pitch = forwardTiltDeg(q);
  float rate = pitchRateDps(_imu.gyro());
  if (_p.invert) { pitch = -pitch; rate = -rate; }
  _pitch = pitch;

  // Gyro glitch: keep the last rate for this cycle.
  bool glitch = fabsf(rate - _lastRate) > _p.rateSpikeDps;
  if (glitch && _rejects < _p.maxRejects) {
    _rejects++;
    return;
  }
  _rejects = 0;
  _lastRate = rate;

  float k = dtSec / (_rateRc + dtSec);     // first-order low-pass
  _rate += k * (rate - _rate);
}
