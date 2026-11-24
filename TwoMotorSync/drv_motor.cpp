#include "drv_motor.h"

#if defined(ESP32)
// 20 kHz (inaudible, within the MDD10A's limit), 8-bit: same 0..255 scale as the Uno.
static const uint32_t PWM_FREQ_HZ = 20000;
static const uint8_t PWM_BITS = 8;
#endif

DrvMotor::DrvMotor(uint8_t dirPin, uint8_t pwmPin, int maxPwm)
  : _dir(dirPin), _pwm(pwmPin), _maxPwm(maxPwm), _cmd(0) {}

void DrvMotor::begin() {
  pinMode(_dir, OUTPUT);
#if defined(ESP32)
  ledcAttach(_pwm, PWM_FREQ_HZ, PWM_BITS);   // needs the ESP32 Arduino core 3.x
#else
  pinMode(_pwm, OUTPUT);
#endif
  setCommand(0);
}

// DIR LOW = forward (positive), DIR HIGH = reverse (negative).
void DrvMotor::setCommand(int cmd) {
  if (cmd > _maxPwm)  cmd = _maxPwm;
  if (cmd < -_maxPwm) cmd = -_maxPwm;
  _cmd = cmd;
  digitalWrite(_dir, cmd < 0 ? HIGH : LOW);
  int mag = cmd < 0 ? -cmd : cmd;
#if defined(ESP32)
  ledcWrite(_pwm, (uint32_t)mag);
#else
  analogWrite(_pwm, mag);
#endif
}
