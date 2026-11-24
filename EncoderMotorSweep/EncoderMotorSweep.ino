#include "Config.h"
#include "Encoder.h"
#include "Motor.h"
#include "SerialCommand.h"

Encoder motorEncoder(PIN_ENC_A, PIN_ENC_B);
Motor motor(PIN_MOTOR_PWM, PIN_MOTOR_DIR);
SerialCommand input(MOTOR_MAX_PWM);

unsigned long lastPrintMs = 0;

void setup() {
  Serial.begin(115200);
  motor.begin();                 // starts stopped
  motorEncoder.begin();
  Serial.println(F("Commands: F=forward, R=reverse, S=stop, or a number -75..75"));
  Serial.println(F("count,revs"));
}

void loop() {
  int cmd;
  if (input.poll(cmd)) {
    motor.setCommand(cmd);
    Serial.print(F("cmd = "));
    Serial.println(cmd);
  }

  unsigned long now = millis();
  if (now - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = now;
    long c = motorEncoder.count();
    Serial.print(c);
    Serial.print(',');
    Serial.println(c / COUNTS_PER_REV, 3);
  }
}
