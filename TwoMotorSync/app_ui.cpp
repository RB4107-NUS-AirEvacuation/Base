#include "app_ui.h"
#include <stdio.h>

static const Command STOP_ALL = {Command::DRIVE, 0.0f};   // speed 0 = stop tracks, hinge and descend

AppUi::AppUi(MidKnob& knob, DrvOled& oled, MidPitch& pitch, MidHinge& hinge, MidHallStop& hall,
             DrvUltrasonic& range, AppDescend& descend, const UiParams& p)
  : _knob(knob), _oled(oled), _pitch(pitch), _hinge(hinge), _hall(hall), _range(range), _descend(descend), _p(p),
    _page(PAGE_MAIN), _sel(0), _top(0), _dirty(true), _lastDrawMs(0) {
  _p.hingeUpSign = (_p.hingeUpSign < 0) ? -1 : 1;
}

// ---------------- Input ----------------

Command AppUi::update(unsigned long nowMs, float driveTarget) {
  _knob.update(nowMs);

  if (_knob.longPressed()) {                 // emergency stop, from anywhere
    _dirty = true;
    return STOP_ALL;
  }
  if (_knob.clicked()) {
    _dirty = true;
    return onClick(driveTarget);
  }
  int steps = _knob.steps();
  if (steps != 0) {
    _dirty = true;
    moveSelection(steps);
  }
  return Command::none();
}

void AppUi::openPage(Page page) {
  _page = page;
  _sel = 0;
  _top = 0;
}

void AppUi::moveSelection(int steps) {
  int sel = (int)_sel + steps;
  if (sel < 0) sel = 0;
  if (sel > itemCount() - 1) sel = itemCount() - 1;
  _sel = (uint8_t)sel;
  // Scroll so the selected item stays on screen.
  if (_sel < _top) _top = _sel;
  if (_sel >= _top + MENU_ROWS) _top = _sel - MENU_ROWS + 1;
}

Command AppUi::onClick(float driveTarget) {
  if (_page == PAGE_DEBUG) return onClickDebug(driveTarget);

  switch (_sel) {
    case MAIN_DESCEND:
      if (_descend.running()) return STOP_ALL;
      return {Command::DESCEND, 0.0f};

    case MAIN_DEBUG:
      openPage(PAGE_DEBUG);
      return Command::none();
  }
  return Command::none();
}

// Click an action to start it; click it again while it runs to stop everything.
Command AppUi::onClickDebug(float driveTarget) {
  if (_sel == DBG_EXIT) {
    openPage(PAGE_MAIN);
    _sel = MAIN_DEBUG;
    return Command::none();
  }
  if (debugRunning(_sel, driveTarget)) return STOP_ALL;

  switch (_sel) {
    case DBG_FWD:        return {Command::DRIVE, _p.maxSpeed};
    case DBG_BACK:       return {Command::DRIVE, -_p.maxSpeed};
    case DBG_HINGE_UP:   return {Command::HINGE, (float)(_p.hingeUpSign * _p.hingeMax)};    // Hall stop ends it
    case DBG_HINGE_DOWN: return {Command::HINGE, (float)(-_p.hingeUpSign * _p.hingeMax)};   // no limit
  }
  return Command::none();
}

bool AppUi::debugRunning(uint8_t item, float driveTarget) const {
  int hinge = _hinge.command() * _p.hingeUpSign;   // > 0: commanded up, < 0: down
  switch (item) {
    case DBG_FWD:        return driveTarget > 0.0f;
    case DBG_BACK:       return driveTarget < 0.0f;
    case DBG_HINGE_UP:   return hinge > 0;          // goes back to 0 when the Hall stop is reached
    case DBG_HINGE_DOWN: return hinge < 0;
  }
  return false;
}

// ---------------- Display ----------------

void AppUi::service(unsigned long nowMs, float driveTarget) {
  bool due = _dirty || (nowMs - _lastDrawMs >= _p.refreshMs);
  if (due && !_oled.busy()) {                // never redraw half way through sending a frame
    render(driveTarget);
    _oled.startFlush();
    _lastDrawMs = nowMs;
    _dirty = false;
  }
  _oled.flushStep();
}

void AppUi::render(float driveTarget) {
  _oled.clear();
  drawStatus();
  for (uint8_t i = 0; i < MENU_ROWS; i++) {
    uint8_t item = _top + i;
    if (item >= itemCount()) break;
    uint8_t row = STATUS_ROWS + i;
    drawItem(row, item, driveTarget);
    if (item == _sel) _oled.invertRow(row);
  }
}

// Row 0: "PITCH -38.2  HG +255"
// Row 1: "HALL 360*  DIST 12.3"   (* = Hall stop latched: hinge straight, ! = sensor fault)
void AppUi::drawStatus() {
  char pitchText[8];
  if (_pitch.ok()) snprintf(pitchText, sizeof(pitchText), "%+5.1f", (double)_pitch.pitch());
  else             snprintf(pitchText, sizeof(pitchText), "  ---");

  char distText[8];
  if (_range.ok()) snprintf(distText, sizeof(distText), "%5.1f", (double)_range.cm());
  else             snprintf(distText, sizeof(distText), "  ---");

  const char* hallMark = _hall.fault() ? "!" : (_hall.atStop() ? "*" : " ");

  char line[32];                             // longer than a row; the display clips the rest
  snprintf(line, sizeof(line), "PITCH%s  HG %+4d", pitchText, _hinge.output());
  _oled.text(0, 0, line);
  snprintf(line, sizeof(line), "HALL %4d%s DIST%s", _hall.strength(), hallMark, distText);
  _oled.text(1, 0, line);
}

void AppUi::drawItem(uint8_t row, uint8_t item, float driveTarget) {
  if (_page == PAGE_MAIN) {
    switch (item) {
      case MAIN_DESCEND: drawLine(row, "DESCEND", _descend.active() ? AppDescend::name(_descend.state()) : "1 STAIR"); break;
      case MAIN_DEBUG:   drawLine(row, "DEBUG", ">"); break;
    }
    return;
  }

  const char* state = debugRunning(item, driveTarget) ? "ON" : "";
  switch (item) {
    case DBG_FWD:        drawLine(row, "FWD MAX", state); break;
    case DBG_BACK:       drawLine(row, "BACK MAX", state); break;
    case DBG_HINGE_UP:   drawLine(row, "HINGE UP", state); break;
    case DBG_HINGE_DOWN: drawLine(row, "HINGE DOWN", state); break;
    case DBG_EXIT:       drawLine(row, "< BACK", ""); break;
  }
}

// Label on the left, value right-aligned: "FWD MAX            ON"
void AppUi::drawLine(uint8_t row, const char* label, const char* value) {
  char line[DrvOled::COLS + 1];
  snprintf(line, sizeof(line), "%-10s%11s", label, value);
  _oled.text(row, 0, line);
}
