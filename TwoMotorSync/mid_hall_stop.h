#pragma once
// MIDDLEWARE LAYER: end stop from an analog Hall sensor.
// A magnet on the moving part approaches the sensor as the hinge nears its stop
// (on this robot: hinge straight). Gives a slow-down zone and a stop point.
#include "drv_hall.h"

struct HallStopParams {
  bool lowerIsNearer; // false: strength rises as the magnet nears; true: it drops
  int slowAt;         // strength (ADC counts) where slowing starts
  int stopAt;         // strength where motion toward the stop is blocked
  int releaseAt;      // strength where the stop clears again (on the far side of stopAt, or equal)
  uint8_t confirm;    // consecutive readings needed to latch the stop
  float crawl;        // fraction of the command allowed inside the slow zone, 0..1
};

class MidHallStop {
public:
  MidHallStop(DrvHall& hall, const HallStopParams& p);

  // Read the sensor and update the stop state. Call once per control cycle.
  void update();

  bool atStop() const;      // true: do not move further toward the stop
  bool fault() const;       // true: reading is at a supply rail (sensor unplugged or shorted)
  int strength() const;     // last field strength, ADC counts

  // Limit a motor command. towardStopSign is +1 if positive commands move
  // toward the stop, -1 if negative commands do. Commands away from the stop
  // always pass unchanged, so the hinge can back off.
  int gate(int cmd, int towardStopSign) const;

private:
  // Compare a strength with a threshold in the magnet's direction.
  bool nearerThan(int s, int threshold) const;
  bool fartherThan(int s, int threshold) const;

  DrvHall& _hall;
  HallStopParams _p;
  int _strength;
  uint8_t _hits;
  bool _stop;
  bool _fault;
};
