#include "mid_serial.h"
#include <stdlib.h>

MidSerial::MidSerial(float defaultSpeed, float maxSpeed, int hingeDefault, int hingeMax)
  : _def(defaultSpeed), _maxSpeed(maxSpeed), _hdef(hingeDefault), _hmax(hingeMax), _len(0) {
  _buf[0] = '\0';
}

bool MidSerial::parseDrive(const char* s, float& out) const {
  char c = s[0];
  if (c == 'F' || c == 'f') { out = _def;  return true; }
  if (c == 'R' || c == 'r') { out = -_def; return true; }
  if (c == 'S' || c == 's') { out = 0.0f;  return true; }

  char* end = nullptr;
  long v = strtol(s, &end, 10);
  if (end == s || *end != '\0') return false;
  long lim = (long)_maxSpeed;
  if (v > lim)  v = lim;
  if (v < -lim) v = -lim;
  out = (float)v;
  return true;
}

// s points just after the leading 'H'.
bool MidSerial::parseHinge(const char* s, float& out) const {
  char c = s[0];
  if (c == 'F' || c == 'f') { out = (float)_hdef;  return true; }
  if (c == 'R' || c == 'r') { out = (float)-_hdef; return true; }
  if (c == 'S' || c == 's') { out = 0.0f;          return true; }

  char* end = nullptr;
  long v = strtol(s, &end, 10);
  if (end == s || *end != '\0') return false;
  if (v > _hmax)  v = _hmax;
  if (v < -_hmax) v = -_hmax;
  out = (float)v;
  return true;
}

Command MidSerial::poll() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (_len == 0) continue;          // ignore blank lines (CRLF pairs)
      _buf[_len] = '\0';
      _len = 0;
      float v;
      if (_buf[0] == 'H' || _buf[0] == 'h') {
        if (parseHinge(_buf + 1, v)) return {Command::HINGE, v};
      } else if ((_buf[0] == 'D' || _buf[0] == 'd') && _buf[1] == '\0') {
        return {Command::DESCEND, 0.0f};
      } else {
        if (parseDrive(_buf, v)) return {Command::DRIVE, v};
      }
      Serial.print(F("unknown command: "));
      Serial.println(_buf);
      return Command::none();
    }
    if (_len < sizeof(_buf) - 1) _buf[_len++] = c;
  }
  return Command::none();
}
