#pragma once
// MIDDLEWARE LAYER: acceleration of the base tray (where the IMU sits), gravity removed,
// split into the two directions the descend sequence cares about:
//   up()       straight up (world vertical), m/s^2
//   forward()  horizontal, toward the robot's front, m/s^2
// The chip's linear acceleration is in the board's own frame, so it is turned into
// the world frame with the orientation quaternion. That way it does not matter how
// the board is tilted on the robot.
//
// Uses the same DrvBno08x as MidPitch. MidPitch polls the chip, so call update()
// after pitch.update() in the same control cycle.
#include "drv_bno08x.h"

struct BodyAccelParams {
  bool  forwardIsY;     // same as PitchParams: which board arrow points to the robot front
  bool  reversed;       // same as PitchParams.invert: the arrow points to the BACK instead
  float cutoffHz;       // low-pass on both outputs (motor vibration); 0 = off
};

class MidBodyAccel {
public:
  MidBodyAccel(DrvBno08x& imu, const BodyAccelParams& p);

  void update(float dtSec);           // call once per control cycle, after pitch.update()

  float up() const { return _up; }            // m/s^2, + = upward
  float forward() const { return _fwd; }      // m/s^2, + = toward the robot front

private:
  DrvBno08x& _imu;
  BodyAccelParams _p;
  float _rc;           // low-pass time constant, s
  float _up;
  float _fwd;
};
