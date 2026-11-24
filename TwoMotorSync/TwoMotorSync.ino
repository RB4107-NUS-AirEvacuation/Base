// TwoMotorSync: app layer. Layers:
//   Driver      drv_motor, drv_encoder, drv_hall, drv_bno08x, drv_i2c, drv_button, drv_oled,
//               drv_ultrasonic
//   Middleware  mid_speed, mid_pi, mid_channel, mid_serial, mid_command, mid_hall_stop,
//               mid_hinge, mid_pitch, mid_body_accel, mid_heading, mid_quat, mid_knob
//   App         this file, app_descend, app_ui, app_config.h
//
// Modes:
//   manual    serial F / R / S / speed and H commands; knob DEBUG page (max fwd / back, hinge up / down)
//   descend   go down the flight front first and stop at the drop-off point, front segment on the
//             landing and rear still on the last steps (serial D, knob DESCEND).
//             See app_descend.h.
//   (climb up: to do)
#include "app_config.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "drv_hall.h"
#include "drv_bno08x.h"
#include "drv_ultrasonic.h"
#include "mid_command.h"
#include "mid_channel.h"
#include "mid_serial.h"
#include "mid_hall_stop.h"
#include "mid_hinge.h"
#include "mid_pitch.h"
#include "mid_body_accel.h"
#include "mid_heading.h"
#include "app_descend.h"
#if APP_HAS_PANEL
#include "drv_button.h"
#include "drv_oled.h"
#include "mid_knob.h"
#include "app_ui.h"
#endif

// ---- Driver layer ----
DrvMotor motor1(M1_DIR, M1_PWM, MOTOR_MAX_PWM);
DrvMotor motor2(M2_DIR, M2_PWM, MOTOR_MAX_PWM);
DrvEncoder enc1(M1_ENC_A, M1_ENC_B);
DrvEncoder enc2(M2_ENC_A, M2_ENC_B);
DrvMotor hingeMotor(HINGE_DIR, HINGE_PWM, HINGE_MAX_PWM);
DrvHall hall(HALL_PIN, HALL_SAMPLES);
DrvBno08x imu(IMU_ADDR, IMU_SDA, IMU_SCL);
DrvUltrasonic range(ULTRA_TRIG, ULTRA_ECHO, ULTRA_MAX_CM);
#if APP_HAS_PANEL
DrvEncoder knobEnc(KNOB_A, KNOB_B);
DrvButton knobBtn(KNOB_BTN);
DrvOled oled(OLED_ADDR, IMU_SDA, IMU_SCL);
#endif

// ---- Middleware layer ----
MidChannel ch1(motor1, enc1, {M1_INVERT, VEL_KP, VEL_KI, MOTOR_MAX_PWM, SPEED_WINDOW_MS, SPEED_ALPHA});
MidChannel ch2(motor2, enc2, {M2_INVERT, VEL_KP, VEL_KI, MOTOR_MAX_PWM, SPEED_WINDOW_MS, SPEED_ALPHA});
MidSerial input(DEFAULT_SPEED, MAX_SPEED, HINGE_DEFAULT_PWM, HINGE_MAX_PWM);
MidHallStop hingeStop(hall, {HALL_LOWER_IS_NEARER, HALL_SLOW_AT, HALL_STOP_AT, HALL_RELEASE_AT,
                             HALL_CONFIRM, HALL_CRAWL});
MidHinge hinge(hingeMotor, hingeStop, {HINGE_INVERT, HINGE_UP_SIGN, HINGE_MAX_PWM, HINGE_SLEW});
MidPitch pitch(imu, {IMU_FORWARD_IS_Y, IMU_PITCH_INVERT, IMU_REPORT_MS,
                     RATE_SPIKE_DPS, RATE_MAX_REJECTS, RATE_CUTOFF_HZ, IMU_STALE_MS});
MidBodyAccel accel(imu, {IMU_FORWARD_IS_Y, IMU_PITCH_INVERT, ACCEL_CUTOFF_HZ});
MidHeading heading(imu, {IMU_FORWARD_IS_Y, IMU_PITCH_INVERT, HEAD_KP, HEAD_KI, HEAD_KD, HEAD_MAX_TURN});
#if APP_HAS_PANEL
MidKnob knob(knobEnc, knobBtn, {KNOB_COUNTS_PER_DETENT, KNOB_REVERSE, KNOB_DEBOUNCE_MS, KNOB_LONG_PRESS_MS});
#endif

// ---- App layer ----
AppDescend descend(ch1, ch2, range, hinge, hingeStop, pitch, accel, {
  COUNTS_PER_CM,
  DESC_APPROACH_SPEED, DESC_CREEP_SPEED, DESC_STRAIGHTEN_SPEED, DESC_CRUISE_SPEED,
  HINGE_UP_SIGN, DESC_HINGE_PWM,
  DESC_CAL_READINGS, DESC_EDGE_RISE_CM, DESC_EDGE_CONFIRM, DESC_CREEP_CM, DESC_BLIND_MS,
  DESC_SPINUP_MS, DESC_CONTACT_UP_MS2, DESC_CONTACT_FWD_MS2, DESC_CONTACT_CONFIRM,
  DESC_STAIR_DEG, DESC_RISE_DEG, DESC_RISE_MS, DESC_STOP_RAMP_MS,
  DESC_CAL_TIMEOUT_MS, DESC_APPROACH_TIMEOUT_MS, DESC_CREEP_TIMEOUT_MS,
  DESC_HINGE_DOWN_TIMEOUT_MS, DESC_STRAIGHTEN_TIMEOUT_MS, DESC_CRUISE_TIMEOUT_MS});
#if APP_HAS_PANEL
AppUi ui(knob, oled, pitch, hinge, hingeStop, range, descend, {MAX_SPEED, HINGE_MAX_PWM, HINGE_UP_SIGN, UI_REFRESH_MS});
#endif

// ---- App state ----
float target = 0.0f;                       // track speed, counts/s (0 = stopped)
bool wasDriving = false;                   // tracks were driving in the last control cycle
bool newRange = false;                     // a range reading arrived since the last control cycle
unsigned long lastCtrlMs = 0;
unsigned long lastPrintMs = 0;
unsigned long lastRangeMs = 0;

// The one place where operator commands (serial or knob) change what the robot does.
// Any manual command ends a running descend: the operator always wins.
void applyCommand(const Command& cmd, unsigned long nowMs) {
  switch (cmd.kind) {
    case Command::DRIVE:
      descend.abort();
      target = cmd.value;
      if (target == 0.0f) hinge.stop();    // stop is an emergency stop for everything
      break;

    case Command::HINGE:
      descend.abort();
      hinge.command((int)cmd.value);
      break;

    case Command::DESCEND:
      target = 0.0f;
      heading.holdCurrent();               // hold this heading for the whole stair, through every stop
      descend.start(nowMs);
      break;

    case Command::NONE:
      break;
  }
}

void printStatusLine() {
  Serial.print(hall.raw());
  Serial.print(',');
  Serial.print(hingeStop.strength());
  Serial.print(',');
  Serial.print(hingeStop.fault() ? 2 : (hingeStop.atStop() ? 1 : 0));
  Serial.print(',');
  if (pitch.ok()) Serial.print(pitch.pitch(), 2); else Serial.print(F("nan"));
  Serial.print(',');
  Serial.print(accel.up(), 2);
  Serial.print(',');
  Serial.print(accel.forward(), 2);
  Serial.print(',');
  Serial.print(heading.yaw(), 1);
  Serial.print(',');
  Serial.print(heading.turn(), 0);
  Serial.print(',');
  Serial.print((int)descend.state());
  Serial.print(',');
  Serial.print(target, 0);
  Serial.print(',');
  Serial.print(ch1.speed(), 1);
  Serial.print(',');
  Serial.print(ch2.speed(), 1);
  Serial.print(',');
  Serial.print(ch1.output());
  Serial.print(',');
  Serial.print(ch2.output());
  Serial.print(',');
  Serial.print(hinge.output());
  Serial.print(',');
  if (range.ok()) Serial.println(range.cm(), 1); else Serial.println(F("nan"));
}

void setup() {
  Serial.begin(115200);
#if defined(ESP32)
  delay(1500);                             // give the USB serial monitor time to attach
#endif
  ch1.begin();
  ch2.begin();
  hall.begin();
  hinge.begin();
  range.begin();
  Serial.print(F("imu: "));
  Serial.println(pitch.begin() ? F("BNO08x ok")
                               : F("BNO08x NOT FOUND - check SDA/SCL, 3.3V, PS0/PS1 -> GND, IMU_ADDR"));
#if APP_HAS_PANEL
  knob.begin();
  Serial.print(F("oled: "));
  Serial.println(oled.begin() ? F("ok") : F("NOT FOUND - check SDA/SCL wiring and OLED_ADDR"));
#endif
  hall.setZero(HALL_ZERO_RAW);
  Serial.print(F("hall raw at startup: "));
  Serial.println(hall.raw());
  lastCtrlMs = millis();
  Serial.println(F("Commands: F = forward, R = reverse, S = stop everything, or counts/s (e.g. 300, -200)"));
  Serial.println(F("Hinge: HF, HR, HS, or H<pwm> (e.g. H40, H-40). D = descend one stair"));
#if APP_HAS_PANEL
  Serial.println(F("Knob: turn = move / change, click = select, long press = STOP EVERYTHING"));
#endif
  Serial.println(F("hall_raw,hall,stop,pitch,acc_up,acc_fwd,yaw,turn,mode,target,v1,v2,pwm1,pwm2,hinge,dist_cm"));
}

void loop() {
  unsigned long now = millis();

  // ---- Operator input ----
  applyCommand(input.poll(), now);
#if APP_HAS_PANEL
  applyCommand(ui.update(now, target), now);
#endif

  // ---- Control, every CONTROL_PERIOD_MS ----
  if (now - lastCtrlMs >= CONTROL_PERIOD_MS) {
    float dt = (now - lastCtrlMs) / 1000.0f;
    lastCtrlMs = now;

    // Sensors
    ch1.measure(now);
    ch2.measure(now);
    pitch.update(now, dt);                 // polls the IMU
    accel.update(dt);                      // uses the same IMU data
    hingeStop.update();

    // Descend sequence: sets the hinge command, and the track speed while it runs
    descend.update(now, newRange);
    newRange = false;
    if (descend.active()) target = descend.trackSpeed();

    // Heading hold: a manual drive holds the heading it starts at.
    // (A descend set its heading when it started, and keeps it through its stops.)
    bool driving = (target != 0.0f);
    if (driving && !wasDriving && !descend.running()) heading.holdCurrent();
    wasDriving = driving;
    heading.update(dt, driving, pitch.ok());

    // Actuators
    hinge.update(dt);
    if (!driving) {
      // Hard stop: PWM 0 (MDD10A brakes) and clear all integrators,
      // so nothing left over from the last run can keep the motors turning.
      ch1.stop();
      ch2.stop();
    } else {
      // Drive speed + heading correction: right = speed + turn, left = speed - turn.
      float left = target - heading.turn();
      float right = target + heading.turn();
      ch1.drive(M1_IS_LEFT ? left : right, dt);
      ch2.drive(M1_IS_LEFT ? right : left, dt);
    }

#if APP_HAS_PANEL
    ui.service(now, target);               // display last: it only uses the spare time
#endif
  }

  // ---- Down-facing range: one ping per ULTRA_PERIOD_MS ----
  // read() blocks until the echo returns (up to ~ULTRA_MAX_CM * 58 us), so it runs
  // right after a control cycle, where it has the most time before the next one.
  if (now - lastRangeMs >= ULTRA_PERIOD_MS) {
    lastRangeMs = now;
    range.read();
    newRange = true;
  }

  // ---- Telemetry ----
  if (now - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = now;
    printStatusLine();
  }
}
