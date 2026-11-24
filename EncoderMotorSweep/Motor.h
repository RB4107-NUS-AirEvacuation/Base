#pragma once
#include <Arduino.h>
#include "Config.h"

// H-bridge style motor driver: PWM sets duty, DIR sets direction.
class Motor {
public:
  Motor(uint8_t pwmPin, uint8_t dirPin);
  void begin();
  void setCommand(int cmd);     // -MOTOR_MAX_PWM .. +MOTOR_MAX_PWM
  int  command() const { return _cmd; }

private:
  uint8_t _pwmPin;
  uint8_t _dirPin;
  int _cmd = 0;
};
