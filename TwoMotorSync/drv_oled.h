#pragma once
// DRIVER LAYER: 0.91" OLED, SSD1306 controller, 128 x 32 pixels, I2C (shares the IMU's bus).
// Text only: 21 columns x 4 rows, 5x7 font. Lower case is shown as upper case.
//
// Drawing only changes a RAM buffer. The buffer is sent one text row at a time
// (128 bytes, about 3 ms at 400 kHz) per flushStep() call, so a screen refresh
// never holds up the control loop for long.
// Wiring: VCC -> 3.3V, GND -> GND, SDA/SCL -> the IMU's SDA/SCL.
#include <Arduino.h>

class DrvOled {
public:
  static constexpr uint8_t COLS = 21;
  static constexpr uint8_t ROWS = 4;
  static constexpr uint8_t WIDTH = 128;

  // addr is 0x3C on almost all 0.91" boards (0x3D on a few).
  explicit DrvOled(uint8_t addr = 0x3C, int sda = -1, int scl = -1);

  bool begin();                                         // false if the display does not answer
  bool ok() const { return _ok; }

  // Drawing (buffer only).
  void clear();
  void text(uint8_t row, uint8_t col, const char* s);   // clipped at the right edge
  void invertRow(uint8_t row);                          // highlight: white background, black text

  // Sending.
  void startFlush();                                    // queue the whole buffer for sending
  void flushStep();                                     // send the next row, if one is queued
  bool busy() const { return _nextRow < ROWS; }         // rows still waiting to be sent

private:
  bool sendCommands(const uint8_t* cmds, uint8_t n);
  bool sendRow(uint8_t row);

  uint8_t _addr;
  int _sda;
  int _scl;
  bool _ok;
  uint8_t _nextRow;
  uint8_t _buf[ROWS * WIDTH];   // one byte = 8 vertical pixels; row r starts at r * WIDTH
};
