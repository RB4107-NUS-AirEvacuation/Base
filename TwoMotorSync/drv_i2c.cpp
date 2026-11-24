#include "drv_i2c.h"
#include <Wire.h>

static bool gStarted = false;

void drvI2cBegin(int sda, int scl) {
  if (gStarted) return;
  gStarted = true;
#if defined(ESP32)
  Wire.begin(sda, scl);
  Wire.setTimeOut(5);                      // ms; a loose wire must not freeze the control loop
#else
  (void)sda;
  (void)scl;
  Wire.begin();
#endif
  Wire.setClock(400000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);         // us; a loose wire must not freeze the control loop
#endif
}
