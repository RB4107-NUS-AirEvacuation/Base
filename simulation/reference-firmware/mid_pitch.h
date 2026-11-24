#pragma once
// MIDDLEWARE LAYER: body pitch from the BNO08x.
// Convention: pitch > 0 means NOSE DOWN (front of the robot lower than the back).
//             rate  > 0 means pitching nose down, in deg/s.
//
// The BNO08x fuses gyro + accelerometer on the chip, so pitch is simply how far
// the robot's forward axis points below horizontal, taken from its quaternion.
// What is left to do here:
//   1. Reject corrupted samples (quaternion length far from 1).
//   2. Reject gyro glitches (rate jumping impossibly far in one sample) for rate().
//   3. Low-pass rate() for display.
//   4. Watchdog: no new orientation for staleMs -> ok() goes false (so the descend
//      sequence stops), and the reports are requested again until the chip answers.
#include "drv_bno08x.h"

struct PitchParams {
  bool    forwardIsY;     // false: the board's X arrow points to the robot's front; true: its Y arrow does
  bool    invert;         // flip if tilting the nose down by hand gives a negative pitch
  uint16_t reportMs;      // how often the chip sends orientation and gyro
  float   rateSpikeDps;   // a rate change bigger than this in one sample counts as a glitch, deg/s
  uint8_t maxRejects;     // glitches in a row before a sample is accepted anyway (the motion was real)
  float   rateCutoffHz;   // low-pass cutoff for rate(); 0 = no filtering
  unsigned long staleMs;  // no new orientation for this long = sensor lost
};

class MidPitch {
public:
  MidPitch(DrvBno08x& imu, const PitchParams& p);

  bool begin();                                  // false if the chip does not answer
  void update(unsigned long nowMs, float dtSec); // call once per control cycle

  float pitch() const { return _pitch; }   // deg, nose down positive
  float rate() const { return _rate; }     // deg/s, nose down positive, low-pass filtered
  bool ok() const { return _ok; }          // false while no fresh data arrives

private:
  float forwardTiltDeg(const BnoQuat& q) const;   // nose-down angle of the forward axis
  float pitchRateDps(const BnoVec& g) const;      // nose-down rate from the gyro

  DrvBno08x& _imu;
  PitchParams _p;
  float _rateRc;                 // rate low-pass time constant, s
  float _pitch;
  float _rate;                   // filtered rate (output)
  float _lastRate;               // last accepted raw rate (for glitch detection)
  uint8_t _rejects;              // glitches in a row
  unsigned long _lastSampleMs;   // time of the last good orientation
  unsigned long _lastRetryMs;    // last time the reports were requested again
  bool _ok;
};
