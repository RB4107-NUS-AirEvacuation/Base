#pragma once
#include <Arduino.h>

// DRIVER LAYER: one brushed DC motor on a Cytron MDD10A channel (sign-magnitude PWM + DIR).
class DrvMotor {
public:
  DrvMotor(uint8_t dirPin, uint8_t pwmPin, int maxPwm);
  void begin();
  void setCommand(int cmd);          // -maxPwm..+maxPwm; sign selects direction
  int  command() const { return _cmd; }

private:
  uint8_t _dir;
  uint8_t _pwm;
  int _maxPwm;
  int _cmd;
};
