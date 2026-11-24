#include "mid_hall_stop.h"

// Readings this close to 0 or ADC_MAX mean the signal is not a working Hall sensor.
static const int RAIL_MARGIN = DrvHall::ADC_MAX / 50;

MidHallStop::MidHallStop(DrvHall& hall, const HallStopParams& p)
  : _hall(hall), _p(p), _strength(0), _hits(0), _stop(false), _fault(false) {
  if (_p.confirm < 1) _p.confirm = 1;
  // releaseAt and slowAt must not be nearer than stopAt.
  if (nearerThan(_p.releaseAt, _p.stopAt)) _p.releaseAt = _p.stopAt;
  if (nearerThan(_p.slowAt, _p.stopAt)) _p.slowAt = _p.stopAt;
  if (_p.crawl < 0.0f) _p.crawl = 0.0f;
  if (_p.crawl > 1.0f) _p.crawl = 1.0f;
}

bool MidHallStop::nearerThan(int s, int threshold) const {
  return _p.lowerIsNearer ? (s < threshold) : (s > threshold);
}

bool MidHallStop::fartherThan(int s, int threshold) const {
  return _p.lowerIsNearer ? (s > threshold) : (s < threshold);
}

void MidHallStop::update() {
  int raw = _hall.raw();
  _fault = (raw < RAIL_MARGIN) || (raw > DrvHall::ADC_MAX - RAIL_MARGIN);

  int s = raw - _hall.zero();
  _strength = s < 0 ? -s : s;

  if (!_stop) {
    if (nearerThan(_strength, _p.stopAt)) {
      if (_hits < 255) _hits++;
      if (_hits >= _p.confirm) _stop = true;
    } else {
      _hits = 0;
    }
  } else if (fartherThan(_strength, _p.releaseAt)) {
    _stop = false;
    _hits = 0;
  }
}

bool MidHallStop::atStop() const {
  return _stop || _fault;      // a broken sensor is treated as "at the stop"
}

bool MidHallStop::fault() const {
  return _fault;
}

int MidHallStop::strength() const {
  return _strength;
}

int MidHallStop::gate(int cmd, int towardStopSign) const {
  bool toward = (towardStopSign >= 0) ? (cmd > 0) : (cmd < 0);
  if (!toward) return cmd;                 // moving away is always allowed
  if (atStop()) return 0;
  if (nearerThan(_strength, _p.slowAt)) return (int)((float)cmd * _p.crawl);
  return cmd;
}
