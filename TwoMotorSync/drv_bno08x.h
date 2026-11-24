#pragma once
// DRIVER LAYER: BNO080 / BNO085 IMU (e.g. GY-BNO08X board) over I2C.
// The chip fuses gyro + accelerometer itself and reports a finished orientation,
// so no filter or gyro calibration is needed on our side.
//
// Reports used:
//   Game Rotation Vector  orientation quaternion from gyro + accel (no magnetometer,
//                         so motors and steel steps cannot disturb it)
//   Calibrated Gyroscope  angular rate, rad/s
//   Linear Acceleration   acceleration with gravity removed, m/s^2
//
// Protocol (CEVA SHTP / SH-2): every message is a packet with a 4-byte header
//   [length LSB][length MSB][channel][sequence number], length includes the header.
// Wiring: VCC -> 3.3V, GND, SDA, SCL; PS0 + PS1 -> GND (I2C mode); AD0 -> GND = 0x4A, 3.3V = 0x4B.
#include <Arduino.h>

struct BnoQuat { float w, x, y, z; };   // unit quaternion, device frame -> world frame
struct BnoVec  { float x, y, z; };      // device frame (the X/Y/Z arrows on the board)

class DrvBno08x {
public:
  explicit DrvBno08x(uint8_t addr = 0x4A, int sda = -1, int scl = -1);

  // Reset the chip, clear its startup messages and turn on the reports.
  // false if the chip does not answer.
  bool begin(uint16_t reportMs);

  // Ask the chip for the reports again (after it reset itself or stopped sending).
  bool enableReports();

  // Read every packet waiting in the chip. Returns true if a new orientation arrived.
  bool poll();

  const BnoQuat& rotation() const { return _rot; }
  const BnoVec&  gyro() const { return _gyro; }    // rad/s
  const BnoVec&  linearAccel() const { return _accel; }   // m/s^2, gravity removed

private:
  bool readPacket();                                // one packet into _pkt; false if none waiting
  void handlePacket();
  bool sendPacket(uint8_t channel, const uint8_t* data, uint8_t len);
  bool setFeature(uint8_t reportId, uint32_t intervalUs);

  uint8_t _addr;
  int _sda;
  int _scl;
  uint32_t _intervalUs;
  uint8_t _seq[6];           // sequence number per channel
  bool _needEnable;          // chip reported a reset: reports must be turned on again
  bool _newRotation;

  uint8_t _channel;          // channel of the packet in _pkt
  uint16_t _len;             // payload bytes stored in _pkt (header removed)
  uint8_t _pkt[128];         // longer packets (startup messages) are cut, we never need their tail

  BnoQuat _rot;
  BnoVec _gyro;
  BnoVec _accel;
};
