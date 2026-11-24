#include "SerialCommand.h"
#include <stdlib.h>

// Parses one trimmed line. Returns false if it isn't a valid command.
static bool parseLine(const char* s, int maxCmd, int& out) {
  char c = s[0];
  if (c == 'F' || c == 'f') { out = maxCmd;  return true; }
  if (c == 'R' || c == 'r') { out = -maxCmd; return true; }
  if (c == 'S' || c == 's') { out = 0;       return true; }

  char* end = nullptr;
  long v = strtol(s, &end, 10);
  if (end == s || *end != '\0') return false;   // not a number

  if (v > maxCmd)  v = maxCmd;
  if (v < -maxCmd) v = -maxCmd;
  out = (int)v;
  return true;
}

SerialCommand::SerialCommand(int maxCmd)
  : _maxCmd(maxCmd), _len(0) {
  _buf[0] = '\0';
}

bool SerialCommand::poll(int& cmdOut) {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (_len == 0) continue;            // ignore empty lines (CRLF pairs)
      _buf[_len] = '\0';
      _len = 0;

      if (parseLine(_buf, _maxCmd, cmdOut)) {
        return true;
      }
      Serial.print(F("unknown command: "));
      Serial.println(_buf);
      return false;
    }

    if (_len < sizeof(_buf) - 1) {
      _buf[_len++] = c;
    }
  }
  return false;
}
