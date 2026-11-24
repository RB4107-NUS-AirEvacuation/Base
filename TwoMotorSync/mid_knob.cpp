#include "mid_knob.h"

MidKnob::MidKnob(DrvEncoder& enc, DrvButton& btn, const KnobParams& p)
  : _enc(enc), _btn(btn), _p(p), _lastCount(0), _steps(0),
    _rawDown(false), _rawSince(0), _down(false), _downSince(0), _longFired(false),
    _click(false), _long(false) {
  if (_p.countsPerDetent < 1) _p.countsPerDetent = 1;
}

void MidKnob::begin() {
  _enc.begin();
  _btn.begin();
  _lastCount = _enc.count();
}

void MidKnob::update(unsigned long nowMs) {
  // Rotation: report whole detents only; a half-turned detent stays pending.
  long detents = (_enc.count() - _lastCount) / _p.countsPerDetent;
  if (detents != 0) {
    _lastCount += detents * _p.countsPerDetent;
    _steps += (int)(_p.reverse ? -detents : detents);
  }

  // Button: accept a new level only after it has been steady for debounceMs.
  bool raw = _btn.down();
  if (raw != _rawDown) {
    _rawDown = raw;
    _rawSince = nowMs;
  }
  if (raw != _down && nowMs - _rawSince >= _p.debounceMs) {
    _down = raw;
    if (_down) {
      _downSince = nowMs;
      _longFired = false;
    } else if (!_longFired) {
      _click = true;                       // released before the long-press time
    }
  }

  // Long press fires while the button is still held, so it acts straight away.
  if (_down && !_longFired && nowMs - _downSince >= _p.longPressMs) {
    _longFired = true;
    _long = true;
  }
}

int MidKnob::steps() {
  int s = _steps;
  _steps = 0;
  return s;
}

bool MidKnob::clicked() {
  bool c = _click;
  _click = false;
  return c;
}

bool MidKnob::longPressed() {
  bool l = _long;
  _long = false;
  return l;
}
