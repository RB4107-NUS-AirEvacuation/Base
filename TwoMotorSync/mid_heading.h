#pragma once
// MIDDLEWARE LAYER: heading hold. Keeps the robot driving straight using the IMU yaw.
//
// A PID on the heading error gives a TURN correction in counts/s. The app adds it to
// one track's speed and subtracts it from the other's, on top of the drive speed:
//   right track = speed + turn,   left track = speed - turn
// (turn > 0 means "turn left"). Track speed difference sets the turning rate in either
// driving direction, so the same sign works forward and in reverse.
//
// Yaw = compass direction of the robot's forward axis, flattened onto the floor, from the
// Game Rotation Vector (no magnetometer: it drifts slowly, which is fine for one run).
// D term uses the gyro's yaw rate directly instead of differentiating the error.
//
// Uses the same DrvBno08x as MidPitch: call update() after pitch.update().
#include "drv_bno08x.h"

struct HeadingParams {
  bool  forwardIsY;      // same as PitchParams
  bool  reversed;        // same as PitchParams.invert
  float kp;              // counts/s per degree of heading error
  float ki;              // counts/s per degree*second
  float kd;              // counts/s per deg/s of yaw rate
  float maxTurn;         // limit on the turn correction, counts/s
};

class MidHeading {
public:
  MidHeading(DrvBno08x& imu, const HeadingParams& p);

  // Hold the heading the robot points at now (clears the integral).
  void holdCurrent();

  // Call once per control cycle, after pitch.update().
  // engaged = tracks are driving; imuOk = the IMU data is fresh.
  // When not engaged (or no IMU) the output is 0 and the integral is cleared.
  void update(float dtSec, bool engaged, bool imuOk);

  float turn() const { return _turn; }          // counts/s, + = turn left
  float yaw() const { return _yaw; }            // deg, + = counter-clockwise seen from above
  float error() const { return _err; }          // deg, setpoint - yaw, wrapped to +/-180

private:
  bool readYaw(float& yawDeg, float& rateDps) const;

  DrvBno08x& _imu;
  HeadingParams _p;
  float _set;            // heading to hold, deg
  float _yaw;
  float _err;
  float _integral;       // the I term itself, counts/s
  float _turn;
};
