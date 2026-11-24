#pragma once
// MIDDLEWARE LAYER: rotary knob (e.g. KY-040) = quadrature encoder + push button.
// Turns raw counts into whole detent steps, and the raw button into
// debounced click / long-press events. Each event is reported once.
#include "drv_encoder.h"
#include "drv_button.h"

struct KnobParams {
  int  countsPerDetent;          // encoder counts per click of the knob (4 on most KY-040s, some are 2)
  bool reverse;                  // flip if turning clockwise moves the wrong way
  unsigned long debounceMs;      // button level must be steady this long to count
  unsigned long longPressMs;     // held this long = long press (fires while still held)
};

class MidKnob {
public:
  MidKnob(DrvEncoder& enc, DrvButton& btn, const KnobParams& p);

  void begin();
  void update(unsigned long nowMs);   // call every loop pass

  // Events since the last call (reading an event clears it).
  int  steps();          // detents turned: + clockwise, - counter-clockwise
  bool clicked();        // pressed and released before the long-press time
  bool longPressed();    // held for longPressMs

private:
  DrvEncoder& _enc;
  DrvButton& _btn;
  KnobParams _p;

  long _lastCount;       // encoder count already turned into steps
  int  _steps;

  bool _rawDown;         // last raw button level
  unsigned long _rawSince;
  bool _down;            // debounced button level
  unsigned long _downSince;
  bool _longFired;       // long press already reported for this press

  bool _click;
  bool _long;
};
