#pragma once
// DRIVER LAYER: the shared I2C bus (IMU and OLED).
// Every I2C driver calls this from its begin(); only the first call does anything.
// Uno: SDA = A4, SCL = A5 (fixed). ESP32: any two pins (-1 = board default).
#include <Arduino.h>

void drvI2cBegin(int sda, int scl);
