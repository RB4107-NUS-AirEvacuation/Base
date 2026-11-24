#pragma once
// APP LAYER: control panel = rotary knob + 0.91" OLED.
//
// Screen (21 x 4 characters): rows 0..1 = status, rows 2..3 = menu (scrolls, selection highlighted).
//   row 0     pitch, hinge PWM                          e.g.  PITCH -38.2  HG +255
//   row 1     Hall reading (* = at the stop, ! = fault),
//             down-facing distance in cm (--- = no echo) e.g.  HALL 360*  DIST 12.3
//
//   MAIN page
//     DESCEND      click: go down one stair (same as serial D); click again to stop.
//                  Shows the current step while it runs.
//     DEBUG    >   open the DEBUG page
//
//   DEBUG page     each one: click to start, click again to stop
//     FWD MAX      tracks forward at max speed (heading hold on)
//     BACK MAX     tracks backward at max speed (heading hold on)
//     HINGE UP     hinge up at full power, stops by itself at the Hall stop
//     HINGE DOWN   hinge down at full power, NO limit: click to stop
//     < BACK       back to MAIN
//
// Turn: move the highlight.   Long press: STOP EVERYTHING, from anywhere (same as serial S).
#include "mid_command.h"
#include "mid_knob.h"
#include "drv_oled.h"
#include "mid_pitch.h"
#include "mid_hinge.h"
#include "mid_hall_stop.h"
#include "app_descend.h"
#include "drv_ultrasonic.h"

struct UiParams {
  float maxSpeed;            // track speed for FWD MAX / BACK MAX, counts/s
  int   hingeMax;            // hinge power for HINGE UP / DOWN, PWM
  int   hingeUpSign;         // sign of a hinge command that moves UP
  unsigned long refreshMs;   // screen redraw period (also redrawn at once after any knob input)
};

class AppUi {
public:
  AppUi(MidKnob& knob, DrvOled& oled, MidPitch& pitch, MidHinge& hinge, MidHallStop& hall,
        DrvUltrasonic& range, AppDescend& descend, const UiParams& p);

  // Read the knob and return what the operator asked for (or Command::NONE).
  // Call every loop pass. driveTarget = the track speed currently commanded.
  Command update(unsigned long nowMs, float driveTarget);

  // Redraw when due and send one row of the screen. Call once per control cycle,
  // after the control work, so the display only uses spare time.
  void service(unsigned long nowMs, float driveTarget);

private:
  enum Page : uint8_t { PAGE_MAIN, PAGE_DEBUG };
  enum MainItem : uint8_t { MAIN_DESCEND, MAIN_DEBUG, MAIN_COUNT };
  enum DebugItem : uint8_t { DBG_FWD, DBG_BACK, DBG_HINGE_UP, DBG_HINGE_DOWN, DBG_EXIT, DBG_COUNT };
  static constexpr uint8_t STATUS_ROWS = 2;
  static constexpr uint8_t MENU_ROWS = DrvOled::ROWS - STATUS_ROWS;

  uint8_t itemCount() const { return _page == PAGE_MAIN ? (uint8_t)MAIN_COUNT : (uint8_t)DBG_COUNT; }
  void openPage(Page page);
  void moveSelection(int steps);
  Command onClick(float driveTarget);
  Command onClickDebug(float driveTarget);
  bool debugRunning(uint8_t item, float driveTarget) const;   // that debug action is on now

  void render(float driveTarget);
  void drawStatus();
  void drawItem(uint8_t row, uint8_t item, float driveTarget);
  void drawLine(uint8_t row, const char* label, const char* value);

  MidKnob& _knob;
  DrvOled& _oled;
  MidPitch& _pitch;
  MidHinge& _hinge;
  MidHallStop& _hall;
  DrvUltrasonic& _range;
  AppDescend& _descend;
  UiParams _p;

  Page    _page;
  uint8_t _sel;              // selected item on the current page
  uint8_t _top;              // first item shown (scroll position)
  bool    _dirty;            // redraw at the next chance
  unsigned long _lastDrawMs;
};
