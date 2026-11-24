#include "drv_bno08x.h"
#include "drv_i2c.h"
#include <Wire.h>

// SHTP channels
static const uint8_t CH_EXECUTABLE = 1;   // reset, reset-complete
static const uint8_t CH_CONTROL = 2;      // SH-2 commands (set feature, ...)
static const uint8_t CH_REPORTS = 3;      // sensor reports

// Messages
static const uint8_t EXEC_RESET = 1;           // host -> chip on CH_EXECUTABLE
static const uint8_t EXEC_RESET_COMPLETE = 1;  // chip -> host on CH_EXECUTABLE
static const uint8_t SET_FEATURE = 0xFD;

// Report IDs
static const uint8_t R_GYRO_CALIBRATED = 0x02;
static const uint8_t R_LINEAR_ACCEL = 0x04;
static const uint8_t R_GAME_ROTATION = 0x08;
static const uint8_t R_TIMESTAMP_BASE = 0xFB;
static const uint8_t R_TIMESTAMP_REBASE = 0xFA;

// Fixed-point scales of the report values
static const float Q14 = 1.0f / 16384.0f;   // quaternion
static const float Q9 = 1.0f / 512.0f;      // gyro, rad/s
static const float Q8 = 1.0f / 256.0f;      // linear acceleration, m/s^2

static const uint8_t HEADER = 4;
static const uint8_t MAX_PACKETS_PER_POLL = 6;   // bound the time spent in one poll()
#if defined(ESP32)
static const uint8_t READ_CHUNK = 124;           // payload per I2C read (Wire buffer 128 - header)
#else
static const uint8_t READ_CHUNK = 28;            // Uno Wire buffer is 32
#endif

static int16_t le16(const uint8_t* p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

// Total length (ID included) of each report we may see in a packet; 0 = unknown.
static uint8_t reportLength(uint8_t id) {
  switch (id) {
    case R_TIMESTAMP_BASE:   return 5;
    case R_TIMESTAMP_REBASE: return 5;
    case R_GYRO_CALIBRATED:  return 10;
    case R_LINEAR_ACCEL:     return 10;
    case R_GAME_ROTATION:    return 12;
    default:                 return 0;
  }
}

DrvBno08x::DrvBno08x(uint8_t addr, int sda, int scl)
  : _addr(addr), _sda(sda), _scl(scl), _intervalUs(10000), _seq{0, 0, 0, 0, 0, 0},
    _needEnable(false), _newRotation(false), _channel(0), _len(0),
    _rot{1.0f, 0.0f, 0.0f, 0.0f}, _gyro{0.0f, 0.0f, 0.0f}, _accel{0.0f, 0.0f, 0.0f} {}

// ---------------- Setup ----------------

bool DrvBno08x::begin(uint16_t reportMs) {
  drvI2cBegin(_sda, _scl);
  _intervalUs = (uint32_t)reportMs * 1000UL;

  Wire.beginTransmission(_addr);
  if (Wire.endTransmission() != 0) return false;      // nothing at this address

  // Soft reset, then throw away the startup messages (advertisement, reset complete, ...).
  const uint8_t reset = EXEC_RESET;
  sendPacket(CH_EXECUTABLE, &reset, 1);
  unsigned long t0 = millis();
  while (millis() - t0 < 300) {
    if (!readPacket()) delay(5);
  }
  _needEnable = false;
  return enableReports();
}

bool DrvBno08x::enableReports() {
  bool ok = setFeature(R_GAME_ROTATION, _intervalUs);
  ok = setFeature(R_GYRO_CALIBRATED, _intervalUs) && ok;
  ok = setFeature(R_LINEAR_ACCEL, _intervalUs) && ok;
  return ok;
}

bool DrvBno08x::setFeature(uint8_t reportId, uint32_t intervalUs) {
  uint8_t cmd[17] = {0};
  cmd[0] = SET_FEATURE;
  cmd[1] = reportId;
  // [2] flags, [3..4] change sensitivity: 0
  cmd[5] = (uint8_t)(intervalUs);              // report interval, microseconds, little endian
  cmd[6] = (uint8_t)(intervalUs >> 8);
  cmd[7] = (uint8_t)(intervalUs >> 16);
  cmd[8] = (uint8_t)(intervalUs >> 24);
  // [9..12] batch interval, [13..16] sensor-specific config: 0
  return sendPacket(CH_CONTROL, cmd, sizeof(cmd));
}

// ---------------- Reading ----------------

bool DrvBno08x::poll() {
  _newRotation = false;
  for (uint8_t n = 0; n < MAX_PACKETS_PER_POLL && readPacket(); n++) {
    handlePacket();
  }
  if (_needEnable) {                           // the chip reset itself: turn the reports back on
    _needEnable = false;
    enableReports();
  }
  return _newRotation;
}

bool DrvBno08x::readPacket() {
  // Read the header alone first to learn the length.
  if (Wire.requestFrom(_addr, HEADER) != HEADER) return false;
  uint8_t lsb = Wire.read();
  uint8_t msb = Wire.read();
  _channel = Wire.read();
  Wire.read();                                 // sequence number, unused
  uint16_t total = ((uint16_t)(msb & 0x7F) << 8) | lsb;   // top bit = continuation flag
  if (total <= HEADER) return false;           // nothing waiting

  // Then the payload. Every read starts with a fresh copy of the header, which we skip.
  uint16_t remaining = total - HEADER;
  _len = 0;
  while (remaining > 0) {
    uint8_t chunk = remaining > READ_CHUNK ? READ_CHUNK : (uint8_t)remaining;
    uint8_t want = chunk + HEADER;
    if (Wire.requestFrom(_addr, want) != want) return false;
    for (uint8_t i = 0; i < HEADER; i++) Wire.read();
    for (uint8_t i = 0; i < chunk; i++) {
      uint8_t b = Wire.read();
      if (_len < sizeof(_pkt)) _pkt[_len++] = b;
    }
    remaining -= chunk;
  }
  return true;
}

void DrvBno08x::handlePacket() {
  if (_channel == CH_EXECUTABLE) {
    if (_len >= 1 && _pkt[0] == EXEC_RESET_COMPLETE) _needEnable = true;
    return;
  }
  if (_channel != CH_REPORTS) return;          // command replies etc.: not needed

  // A report packet holds several reports back to back, e.g. [timestamp][rotation][gyro].
  // Each report: [ID][sequence][status][delay][data...]
  uint16_t i = 0;
  while (i < _len) {
    uint8_t n = reportLength(_pkt[i]);
    if (n == 0 || i + n > _len) return;        // unknown or cut-off report: skip the rest
    const uint8_t* r = _pkt + i;
    const uint8_t* data = r + 4;

    if (r[0] == R_GAME_ROTATION) {
      _rot.x = le16(data + 0) * Q14;
      _rot.y = le16(data + 2) * Q14;
      _rot.z = le16(data + 4) * Q14;
      _rot.w = le16(data + 6) * Q14;
      _newRotation = true;
    } else if (r[0] == R_GYRO_CALIBRATED) {
      _gyro.x = le16(data + 0) * Q9;
      _gyro.y = le16(data + 2) * Q9;
      _gyro.z = le16(data + 4) * Q9;
    } else if (r[0] == R_LINEAR_ACCEL) {
      _accel.x = le16(data + 0) * Q8;
      _accel.y = le16(data + 2) * Q8;
      _accel.z = le16(data + 4) * Q8;
    }
    i += n;
  }
}

// ---------------- Writing ----------------

bool DrvBno08x::sendPacket(uint8_t channel, const uint8_t* data, uint8_t len) {
  uint16_t total = len + HEADER;
  Wire.beginTransmission(_addr);
  Wire.write((uint8_t)(total & 0xFF));
  Wire.write((uint8_t)(total >> 8));
  Wire.write(channel);
  Wire.write(_seq[channel]++);
  Wire.write(data, len);
  return Wire.endTransmission() == 0;
}
