#pragma once
#include <Arduino.h>

// Reads one line from Serial and turns it into a motor command.
// Accepted input:
//   F or f  -> forward at maxCmd
//   R or r  -> reverse at maxCmd
//   S or s  -> stop
//   integer -> signed command, e.g. 40 or -30 (clamped to +/- maxCmd)
class SerialCommand {
public:
  explicit SerialCommand(int maxCmd);

  // Call every loop. Returns true when a complete, valid line has been read.
  bool poll(int& cmdOut);

private:
  int _maxCmd;
  char _buf[16];
  uint8_t _len;
};
