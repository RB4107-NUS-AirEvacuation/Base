#pragma once
// APP LAYER: drive DOWN one stair, front first. Started with serial "D" or the knob menu.
//
//   1 CALIBRATE   tracks stopped: average the down-facing range = normal floor height,
//                 and the body pitch = what "flat" looks like
//   2 APPROACH    drive forward until the range grows by more than edgeRiseCm
//                 (the floor has dropped away under the sensor: stair edge)
//     or BLIND    if step 1 got no range echo: drive forward for blindMs instead
//                 (the range sensor is not needed after this step)
//   3 CREEP       drive slower for creepCm more, measured with the track encoders
//   4 HINGE_DOWN  tracks stopped, hinge DOWN at full power until the base tray is pushed
//                 up or forward (the front segment has landed on the lower step and is
//                 now lifting the base)
//   5 STRAIGHTEN  hinge UP at full power + tracks at half speed, until the Hall end stop
//                 says the hinge is straight
//   6 CRUISE      drive down the flight, remembering the steepest pitch. Once the body has
//                 been on the stairs (pitch more than stairDeg from the calibrated flat
//                 pitch), it starts tilting back up when the front segment reaches the
//                 landing: pitch riseDeg back from the steepest, held for riseMs
//     SLOWING     ramp the track speed down to zero over stopRampMs
//   7 DONE        the drop-off point (front on the landing, rear still on the last steps):
//                 tracks stopped, until the next operator command.
//                 The casualty is handed over here; the base never leaves the stairs, so it
//                 can climb back up from here without lifting itself onto the first step.
//
// Nothing fails: a step that reaches its time limit just moves on to the next step
// (and says so on serial). Without the IMU, landing and the drop-off point cannot be felt,
// so those steps simply run to their time limits.
#include "mid_channel.h"
#include "mid_hinge.h"
#include "mid_hall_stop.h"
#include "mid_pitch.h"
#include "mid_body_accel.h"
#include "drv_ultrasonic.h"

struct DescendParams {
  // Geometry
  float countsPerCm;              // track encoder counts per cm of travel
  // Speeds, counts/s
  float approachSpeed;
  float creepSpeed;
  float straightenSpeed;
  float cruiseSpeed;
  // Hinge
  int   hingeUpSign;              // sign of a hinge command that moves UP (toward the Hall stop)
  int   hingePwm;                 // power for both hinge moves
  // Stair edge
  uint8_t calReadings;            // range readings averaged for the floor height
  float edgeRiseCm;               // range increase that means "edge"
  uint8_t edgeConfirm;            // readings in a row needed
  float creepCm;                  // distance to creep past the edge
  unsigned long blindMs;          // no range sensor: drive this long instead of finding the edge
  // Landing (hinge down)
  unsigned long spinupMs;         // ignore the accelerometer while the hinge motor starts
  float contactUpMs2;             // upward acceleration that means "landed"
  float contactFwdMs2;            // forward acceleration that means "landed"
  uint8_t contactConfirm;         // samples in a row needed
  // Drop-off point at the bottom of the flight
  float stairDeg;                 // pitch more than this from the calibrated flat pitch = on the stairs
  float riseDeg;                  // after that, tilting back up this much from the steepest = landing
  unsigned long riseMs;           // it must stay tilted back this long
  unsigned long stopRampMs;       // slow from cruise speed to a stop over this long
  // Time limits per step, ms
  unsigned long calTimeoutMs;
  unsigned long approachTimeoutMs;
  unsigned long creepTimeoutMs;
  unsigned long hingeDownTimeoutMs;
  unsigned long straightenTimeoutMs;
  unsigned long cruiseTimeoutMs;
};

class AppDescend {
public:
  enum State : uint8_t {
    IDLE = 0, CALIBRATE = 1, APPROACH = 2, CREEP = 3, HINGE_DOWN = 4,
    STRAIGHTEN = 5, CRUISE = 6, DONE = 7, BLIND = 8, SLOWING = 9
  };

  AppDescend(MidChannel& left, MidChannel& right, DrvUltrasonic& range, MidHinge& hinge,
             MidHallStop& stop, MidPitch& pitch, MidBodyAccel& accel, const DescendParams& p);

  void start(unsigned long nowMs);
  void abort();                   // operator took over: stop the hinge, go idle

  // Call each control cycle, BEFORE hinge.update().
  // newRange = a new range reading was taken since the last call.
  void update(unsigned long nowMs, bool newRange);

  // While active, the sequence owns the tracks: drive them at trackSpeed().
  // (DONE stays active, at speed 0, so the robot does not drive off.)
  bool active() const { return _state != IDLE; }
  bool running() const { return active() && _state != DONE; }
  float trackSpeed() const;

  State state() const { return _state; }
  static const char* name(State s);   // short upper-case name, for the screen and serial

private:
  void enter(State s, unsigned long nowMs);
  void timedOut(const __FlashStringHelper* what);   // serial note: step hit its time limit
  float travelledCm() const;
  void startCreep(unsigned long nowMs);
  void startHingeDown(unsigned long nowMs);
  void startStraighten(unsigned long nowMs);
  void startCruise(unsigned long nowMs);
  void startSlowing(unsigned long nowMs);
  bool pitchFromFlat(float& deg) const;   // |pitch - calibrated flat pitch|; false if unknown

  MidChannel& _left;
  MidChannel& _right;
  DrvUltrasonic& _range;
  MidHinge& _hinge;
  MidHallStop& _stop;
  MidPitch& _pitch;
  MidBodyAccel& _accel;
  DescendParams _p;

  State _state;
  unsigned long _t0;              // time the current step started
  uint8_t _hits;                  // confirm counter of the current step
  float _sum;                     // calibration sum
  uint8_t _n;                     // calibration readings
  float _floorCm;                 // calibrated floor height
  float _pitchSum;                // calibration: pitch sum
  long _pitchN;                   // calibration: pitch samples
  float _flatPitch;               // calibrated pitch on the flat, deg
  bool _flatKnown;                // the IMU gave a flat pitch during calibration
  bool _onStairs;                 // body seen at stair pitch since the hinge started straightening
  float _steepest;                // largest pitch from flat seen while cruising, deg
  unsigned long _riseSince;       // when the body last started tilting back up by riseDeg
  float _slowScale;               // SLOWING: fraction of cruise speed left, 1 -> 0
  long _left0;                    // encoder counts when CREEP started
  long _right0;
};
