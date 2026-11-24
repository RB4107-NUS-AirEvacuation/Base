#include "app_descend.h"

AppDescend::AppDescend(MidChannel& left, MidChannel& right, DrvUltrasonic& range, MidHinge& hinge,
                       MidHallStop& stop, MidPitch& pitch, MidBodyAccel& accel,
                       const DescendParams& p)
  : _left(left), _right(right), _range(range), _hinge(hinge), _stop(stop), _pitch(pitch),
    _accel(accel), _p(p), _state(IDLE), _t0(0), _hits(0), _sum(0.0f), _n(0), _floorCm(0.0f),
    _pitchSum(0.0f), _pitchN(0), _flatPitch(0.0f), _flatKnown(false), _onStairs(false),
    _steepest(0.0f), _riseSince(0), _slowScale(0.0f), _left0(0), _right0(0) {
  _p.hingeUpSign = (_p.hingeUpSign < 0) ? -1 : 1;
  if (_p.calReadings < 1) _p.calReadings = 1;
}

const char* AppDescend::name(State s) {
  switch (s) {
    case IDLE:       return "IDLE";
    case CALIBRATE:  return "CALIB";
    case APPROACH:   return "APPROACH";
    case BLIND:      return "BLIND";
    case CREEP:      return "CREEP";
    case HINGE_DOWN: return "H-DOWN";
    case STRAIGHTEN: return "H-UP";
    case CRUISE:     return "CRUISE";
    case SLOWING:    return "SLOWING";
    case DONE:       return "DONE";
  }
  return "?";
}

void AppDescend::start(unsigned long nowMs) {
  _hinge.stop();
  _sum = 0.0f;
  _n = 0;
  _pitchSum = 0.0f;
  _pitchN = 0;
  _onStairs = false;
  enter(CALIBRATE, nowMs);
}

void AppDescend::abort() {
  if (_state == IDLE) return;
  _hinge.stop();
  _state = IDLE;
  Serial.println(F("descend: stopped by operator"));
}

void AppDescend::enter(State s, unsigned long nowMs) {
  _state = s;
  _t0 = nowMs;
  _hits = 0;
  Serial.print(F("descend: "));
  Serial.println(name(s));
}

void AppDescend::timedOut(const __FlashStringHelper* what) {
  Serial.print(F("descend: time limit, "));
  Serial.print(what);
  Serial.println(F(" - moving on"));
}

float AppDescend::trackSpeed() const {
  switch (_state) {
    case APPROACH:
    case BLIND:      return _p.approachSpeed;
    case CREEP:      return _p.creepSpeed;
    case STRAIGHTEN: return _p.straightenSpeed;
    case CRUISE:     return _p.cruiseSpeed;
    case SLOWING:    return _p.cruiseSpeed * _slowScale;
    default:        return 0.0f;             // CALIBRATE, HINGE_DOWN, DONE: stand still
  }
}

// ---------------- Step transitions ----------------

void AppDescend::startCreep(unsigned long nowMs) {
  _left0 = _left.position();
  _right0 = _right.position();
  enter(CREEP, nowMs);
}

void AppDescend::startHingeDown(unsigned long nowMs) {
  _hinge.command(-_p.hingeUpSign * _p.hingePwm);
  enter(HINGE_DOWN, nowMs);
}

void AppDescend::startStraighten(unsigned long nowMs) {
  _hinge.stop();                                    // cut the down drive at once
  _hinge.command(_p.hingeUpSign * _p.hingePwm);
  enter(STRAIGHTEN, nowMs);
}

void AppDescend::startCruise(unsigned long nowMs) {
  _hinge.stop();
  _steepest = 0.0f;
  _riseSince = nowMs;
  enter(CRUISE, nowMs);
}

void AppDescend::startSlowing(unsigned long nowMs) {
  _slowScale = 1.0f;
  enter(SLOWING, nowMs);
}

// ---------------- Helpers ----------------

float AppDescend::travelledCm() const {
  long counts = ((_left.position() - _left0) + (_right.position() - _right0)) / 2;
  return counts / _p.countsPerCm;
}

// Without a flat reference or fresh IMU data the pitch offset is unknown.
bool AppDescend::pitchFromFlat(float& deg) const {
  if (!_flatKnown || !_pitch.ok()) return false;
  deg = fabsf(_pitch.pitch() - _flatPitch);
  return true;
}

// ---------------- The sequence ----------------

void AppDescend::update(unsigned long nowMs, bool newRange) {
  unsigned long inStep = nowMs - _t0;
  bool gotRange = newRange && _range.ok();

  switch (_state) {
    case IDLE:
    case DONE:
      break;

    // 1: average the floor height and the flat pitch while standing still.
    case CALIBRATE:
      if (gotRange) {
        _sum += _range.cm();
        _n++;
      }
      if (_pitch.ok()) {
        _pitchSum += _pitch.pitch();
        _pitchN++;
      }
      if ((_n >= _p.calReadings && _pitchN > 0) || inStep > _p.calTimeoutMs) {
        _flatKnown = (_pitchN > 0);
        if (_flatKnown) _flatPitch = _pitchSum / _pitchN;
        else Serial.println(F("descend: no IMU - landing and flat will run to their time limits"));

        if (_n >= _p.calReadings) {
          _floorCm = _sum / _n;
          Serial.print(F("descend: floor at "));
          Serial.print(_floorCm, 1);
          Serial.println(F(" cm"));
          enter(APPROACH, nowMs);
        } else {
          // No range sensor: skip the edge search and drive a fixed time instead.
          Serial.println(F("descend: no range echo, driving blind"));
          enter(BLIND, nowMs);
        }
      }
      break;

    // 2: drive until the floor drops away. A missing echo neither counts nor resets.
    case APPROACH:
      if (gotRange) {
        if (_range.cm() > _floorCm + _p.edgeRiseCm) _hits++;
        else _hits = 0;
      }
      if (_hits >= _p.edgeConfirm) {
        startCreep(nowMs);
      } else if (inStep > _p.approachTimeoutMs) {
        timedOut(F("no stair edge found"));
        startCreep(nowMs);
      }
      break;

    // 2 without the range sensor: drive a fixed time, then carry on as if at the edge.
    case BLIND:
      if (inStep >= _p.blindMs) startCreep(nowMs);
      break;

    // 3: creep a set distance past the edge, then bend down.
    case CREEP:
      if (travelledCm() >= _p.creepCm) {
        startHingeDown(nowMs);
      } else if (inStep > _p.creepTimeoutMs) {
        timedOut(F("creep distance not reached"));
        startHingeDown(nowMs);
      }
      break;

    // 4: hinge down until the base tray is pushed up or forward.
    case HINGE_DOWN:
      if (inStep >= _p.spinupMs && _pitch.ok()) {
        bool push = _accel.up() > _p.contactUpMs2 || _accel.forward() > _p.contactFwdMs2;
        _hits = push ? _hits + 1 : 0;
      }
      if (_hits >= _p.contactConfirm) {
        Serial.print(F("descend: landed, up "));
        Serial.print(_accel.up(), 2);
        Serial.print(F(" fwd "));
        Serial.println(_accel.forward(), 2);
        startStraighten(nowMs);
      } else if (inStep > _p.hingeDownTimeoutMs) {
        timedOut(F("no landing felt"));
        startStraighten(nowMs);
      }
      break;

    // 5: hinge up while rolling forward, until the Hall stop says straight.
    case STRAIGHTEN:
      {
        float d;
        if (pitchFromFlat(d) && d > _p.stairDeg) _onStairs = true;
      }
      if (_stop.atStop()) {                             // also true on a Hall fault: hinge up is blocked then
        if (_stop.fault()) Serial.println(F("descend: Hall sensor fault, hinge stopped"));
        startCruise(nowMs);
      } else if (inStep > _p.straightenTimeoutMs) {
        timedOut(F("hinge did not reach the Hall stop"));
        startCruise(nowMs);
      }
      break;

    // 6: drive down the flight. When the front segment reaches the landing the body starts
    //    tilting back up: once it is riseDeg back from the steepest pitch, slow to a stop.
    case CRUISE: {
      float d;
      bool known = pitchFromFlat(d);
      if (known) {
        if (d > _p.stairDeg) _onStairs = true;
        if (d > _steepest) _steepest = d;
      }
      bool tiltingUp = known && _onStairs && (_steepest - d) > _p.riseDeg;
      if (!tiltingUp) {
        _riseSince = nowMs;                             // not tilting back up yet: restart the timer
      } else if (nowMs - _riseSince >= _p.riseMs) {
        Serial.print(F("descend: tilting back up, pitch "));
        Serial.print(_pitch.pitch(), 1);
        Serial.print(F(", steepest "));
        Serial.println(_steepest, 1);
        startSlowing(nowMs);
        break;
      }
      if (inStep > _p.cruiseTimeoutMs) {
        timedOut(F("bottom of the flight not felt"));
        startSlowing(nowMs);
      }
      break;
    }

    // 6b: ramp down to a stop at the drop-off point (rear still on the last steps).
    case SLOWING:
      _slowScale = (_p.stopRampMs > 0) ? 1.0f - (float)inStep / _p.stopRampMs : 0.0f;
      if (_slowScale <= 0.0f) {
        _slowScale = 0.0f;
        Serial.println(F("descend: stopped at the drop-off point"));
        enter(DONE, nowMs);
      }
      break;
  }
}
