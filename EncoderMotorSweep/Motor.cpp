#include "Motor.h"

Motor::Motor(uint8_t pwmPin, uint8_t dirPin)
  : _pwmPin(pwmPin), _dirPin(dirPin) {}

void Motor::begin() {
  pinMode(_pwmPin, OUTPUT);
  pinMode(_dirPin, OUTPUT);
  setCommand(0);
}

// Sign selects direction, magnitude selects PWM duty.
// DIR LOW = forward (positive), DIR HIGH = reverse (negative).
void Motor::setCommand(int cmd) {
  cmd = constrain(cmd, -MOTOR_MAX_PWM, MOTOR_MAX_PWM);
  _cmd = cmd;
  digitalWrite(_dirPin, cmd < 0 ? HIGH : LOW);
  analogWrite(_pwmPin, abs(cmd));
}
