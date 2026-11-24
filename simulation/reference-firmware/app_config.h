#pragma once
// APP LAYER: pins and tuning. Edit values here only.
#include <Arduino.h>

// ================= Board pins =================
#if defined(ESP32)
// ---- ESP32-C6-DevKitC-1. ALL PINS ARE 3.3 V ONLY (not 5 V tolerant). ----
// Avoided: 8, 15 (strapping; 8 = on-board RGB LED), 12, 13 (USB), 16, 17 (serial port).
// 4, 5, 9 are strapping pins too, but safe for the knob: 4 and 5 only choose SDIO timing,
// and 9 is the BOOT button (holding the knob button during reset enters download mode).
// 15 is a strapping pin, but only read at reset; the ESP drives it as TRIG after boot.
// No spare GPIOs left on the headers (the BNO08x INT pin stays unconnected).
constexpr uint8_t M1_DIR = 18;
constexpr uint8_t M1_PWM = 19;
constexpr uint8_t M2_ENC_A = 20;
constexpr uint8_t M2_ENC_B = 21;

constexpr uint8_t M2_DIR = 22;
constexpr uint8_t M2_PWM = 23;
constexpr uint8_t M1_ENC_A = 10;
constexpr uint8_t M1_ENC_B = 11;

constexpr uint8_t HINGE_DIR = 0;
constexpr uint8_t HINGE_PWM = 1;

constexpr uint8_t HALL_PIN = 2;        // ADC1 channel 2. Power the Hall sensor from 3.3 V
constexpr int IMU_SDA = 7;             // shared I2C bus: the OLED connects here too
constexpr int IMU_SCL = 6;             // (6/7 swapped to match the wiring; found with an I2C scan)

// Control panel knob (KY-040). Power the knob board from 3.3 V, not 5 V.
constexpr uint8_t KNOB_A = 4;          // CLK
constexpr uint8_t KNOB_B = 5;          // DT
constexpr uint8_t KNOB_BTN = 9;        // SW (to GND). Same pin as the BOOT button: either one clicks

// Down-facing ultrasonic sensor. A 5 V HC-SR04 drives ECHO at 5 V: use a divider
// (ECHO -> 1k -> GPIO 3 -> 2k -> GND). A 3.3 V-capable one (HC-SR04P, RCWL-1601) needs none.
constexpr uint8_t ULTRA_TRIG = 15;
constexpr uint8_t ULTRA_ECHO = 3;

#else
// ---- Arduino Uno ----
constexpr uint8_t M1_DIR = 4;
constexpr uint8_t M1_PWM = 5;
constexpr uint8_t M1_ENC_A = 2;          // external interrupts
constexpr uint8_t M1_ENC_B = 3;

constexpr uint8_t M2_DIR = 8;
constexpr uint8_t M2_PWM = 6;
constexpr uint8_t M2_ENC_A = A0;         // pin-change interrupts
constexpr uint8_t M2_ENC_B = A1;

constexpr uint8_t HINGE_DIR = 7;
constexpr uint8_t HINGE_PWM = 9;

constexpr uint8_t HALL_PIN = A2;
constexpr int IMU_SDA = -1;              // fixed on the Uno: SDA = A4, SCL = A5
constexpr int IMU_SCL = -1;

constexpr uint8_t ULTRA_TRIG = 12;
constexpr uint8_t ULTRA_ECHO = 11;
#endif

// ---- Down-facing ultrasonic range sensor (polled, no interrupts) ----
constexpr uint16_t ULTRA_MAX_CM = 100;          // farther = "no echo"; also sets the wait: ~58 us per cm
constexpr unsigned long ULTRA_PERIOD_MS = 60;   // time between pings (>= 50 ms so old echoes die away)

// ---- Analog Hall sensor ----
constexpr uint8_t HALL_SAMPLES = 8;    // ADC readings averaged per read

// ---- Hinge end stop from the Hall sensor (strength = ADC counts away from HALL_ZERO_RAW) ----
// PLACEHOLDERS: watch the "hall" column while moving the hinge by hand, then set these.
#if defined(ESP32)
// Measured on the robot: "hall" reads ~410-420 with no magnet and DROPS below ~350
// as the magnet comes near, so HALL_LOWER_IS_NEARER is true.
constexpr bool HALL_LOWER_IS_NEARER = true;
constexpr int HALL_ZERO_RAW = 2048;
constexpr int HALL_SLOW_AT = 400;      // crawl when nearer than this (= STOP_AT: no crawl zone)
constexpr int HALL_STOP_AT = 400;      // stop when hall < 400
constexpr int HALL_RELEASE_AT = 400;   // clear when hall > 400
#else
constexpr bool HALL_LOWER_IS_NEARER = false;
constexpr int HALL_ZERO_RAW = 500;
constexpr int HALL_SLOW_AT = 60;
constexpr int HALL_STOP_AT = 100;
constexpr int HALL_RELEASE_AT = 80;
#endif
constexpr uint8_t HALL_CONFIRM = 3;    // readings in a row needed to latch the stop
constexpr float HALL_CRAWL = 0.3f;     // fraction of hinge command allowed in the slow zone

// ---- Hinge motor: open loop (no encoder), on its own DIR + PWM driver channel ----
// (pins are in the board section at the top)
constexpr int  HINGE_MAX_PWM = 255;          // full power (no end stop on the opening side: watch it)
constexpr bool HINGE_INVERT = false;         // flip if "H+" moves the wrong way
constexpr int  HINGE_UP_SIGN = +1;           // +1: positive H commands move the hinge UP (toward straight,
                                             // the Hall stop side). Hinge down is never blocked.
constexpr float HINGE_SLEW = 250.0f;         // PWM per second, how fast the command may change (0 -> full in ~1 s)
constexpr int  HINGE_DEFAULT_PWM = 40;       // used by HF and HR

// ---- Body IMU (BNO080 / BNO085 on I2C; pins are in the board section at the top) ----
// Mount the board flat, chip side up, with its X (or Y) arrow pointing to the robot front.
constexpr uint8_t IMU_ADDR = 0x4A;         // AD0 -> GND: 0x4A, AD0 -> 3.3V: 0x4B
constexpr bool  IMU_FORWARD_IS_Y = false;  // false: the board's X arrow points to the robot front
constexpr bool  IMU_PITCH_INVERT = false;  // flip if tilting the nose down by hand gives a negative pitch
constexpr uint16_t IMU_REPORT_MS = 10;     // orientation + gyro every 10 ms (100 Hz, matches the control loop)
constexpr unsigned long IMU_STALE_MS = 100; // no new orientation this long = sensor lost (descend fails)
constexpr float RATE_SPIKE_DPS = 100.0f;   // gyro rate jump in one sample treated as a glitch
constexpr uint8_t RATE_MAX_REJECTS = 2;    // glitches in a row before accepting anyway (the motion was real)
constexpr float RATE_CUTOFF_HZ = 10.0f;    // low-pass on the pitch rate output (0 = off)

constexpr float ACCEL_CUTOFF_HZ = 15.0f;    // low-pass on the base tray acceleration (motor vibration)

// ---- Track geometry ----
constexpr float WHEEL_DIAMETER_CM = 3.6f;
constexpr float COUNTS_PER_WHEEL_REV = 64.0f * 150.0f;   // encoder 64 counts/rev x 150:1 gearbox
constexpr float COUNTS_PER_CM = COUNTS_PER_WHEEL_REV / (3.14159265f * WHEEL_DIAMETER_CM);   // ~849

// ---- Motor output limits and polarity ----
constexpr int  MOTOR_MAX_PWM = 255;     // raise to 255 after testing
constexpr bool M1_INVERT = true;       // true: swaps forward/backward (motor + encoder flip together)
constexpr bool M2_INVERT = true;

// ---- Speed estimate (middleware) ----
constexpr unsigned long SPEED_WINDOW_MS = 20;
constexpr float SPEED_ALPHA = 0.5f;    // 0..1, higher = less smoothing

// ---- Velocity loop, per motor (middleware) ----
constexpr float VEL_KP = 0.05f;        // PWM per (count/s) of speed error
constexpr float VEL_KI = 0.2f;         // PWM per (count/s * s)

// ---- Heading hold: PID on IMU yaw, keeps the robot driving straight (middleware) ----
// Its turn correction is added to one track and subtracted from the other, on top of
// the drive speed. Starting values for a slow tracked robot: tune on the floor.
// If the robot steers AWAY from its heading (wobble grows), flip M1_IS_LEFT.
constexpr bool  M1_IS_LEFT = false;    // false: motor 1 drives the RIGHT track (flipped after testing)
constexpr float HEAD_KP = 20.0f;       // counts/s per degree of heading error (5 deg -> 100)
constexpr float HEAD_KI = 2.0f;        // counts/s per degree*second (removes a steady pull to one side)
constexpr float HEAD_KD = 10.0f;       // counts/s per deg/s of yaw rate (damps overshoot)
constexpr float HEAD_MAX_TURN = 500.0f; // counts/s, limit on the correction

// ---- Control panel: knob + 0.91" OLED (ESP32 only; the Uno has no pins or RAM to spare) ----
#if defined(ESP32)
  #define APP_HAS_PANEL 1
#else
  #define APP_HAS_PANEL 0
#endif
constexpr uint8_t OLED_ADDR = 0x3C;               // 0x3D on a few boards
constexpr int   KNOB_COUNTS_PER_DETENT = 4;       // set to 2 if one click moves two lines
constexpr bool  KNOB_REVERSE = false;             // flip if clockwise moves the wrong way
constexpr unsigned long KNOB_DEBOUNCE_MS = 20;
constexpr unsigned long KNOB_LONG_PRESS_MS = 800; // long press = stop everything
constexpr unsigned long UI_REFRESH_MS = 200;

// ---- Commands and timing ----
constexpr float DEFAULT_SPEED = 1500.0f;   // used by F and R
constexpr float MAX_SPEED = 1500.0f;      // clamp for typed numbers
constexpr unsigned long CONTROL_PERIOD_MS = 10;
constexpr unsigned long PRINT_PERIOD_MS = 200;

// ---- Descend one stair (serial D / knob DESCEND). PLACEHOLDERS: tune on the robot ----
constexpr float DESC_APPROACH_SPEED = DEFAULT_SPEED;          // driving up to the edge
constexpr float DESC_CREEP_SPEED = 0.3f * MAX_SPEED;          // past the edge
constexpr float DESC_STRAIGHTEN_SPEED = 0.5f * MAX_SPEED;     // while the hinge straightens
constexpr float DESC_CRUISE_SPEED = 1000.0f;                  // after the stair (below max: leaves room for heading hold)
constexpr int   DESC_HINGE_PWM = HINGE_MAX_PWM;               // hinge down and up at full power
constexpr uint8_t DESC_CAL_READINGS = 10;                     // range readings for the floor height (~0.6 s)
constexpr float DESC_EDGE_RISE_CM = 2.0f;                     // range this much above the floor = edge
constexpr uint8_t DESC_EDGE_CONFIRM = 2;                      // readings in a row
constexpr float DESC_CREEP_CM = 9.0f;                         // drive this far past the edge
constexpr unsigned long DESC_BLIND_MS = 3000;                 // no range echo at the start: drive this long instead of finding the edge
constexpr unsigned long DESC_SPINUP_MS = 300;                 // ignore the IMU while the hinge motor starts
constexpr float DESC_CONTACT_UP_MS2 = 1.5f;                   // base tray pushed up this hard = landed
constexpr float DESC_CONTACT_FWD_MS2 = 1.5f;                  // or pushed forward this hard
constexpr uint8_t DESC_CONTACT_CONFIRM = 2;                   // samples in a row (10 ms each)
constexpr float DESC_FLAT_DEG = 3.0f;                         // pitch within this of the start pitch = flat
constexpr unsigned long DESC_FLAT_MS = 500;                   // flat this long = off the stair: stop
constexpr unsigned long DESC_CAL_TIMEOUT_MS = 2000;           // time limits per step: exceeded = move on to the next step
constexpr unsigned long DESC_APPROACH_TIMEOUT_MS = 60000;
constexpr unsigned long DESC_CREEP_TIMEOUT_MS = 30000;
constexpr unsigned long DESC_HINGE_DOWN_TIMEOUT_MS = 5000;
constexpr unsigned long DESC_STRAIGHTEN_TIMEOUT_MS = 8000;
constexpr unsigned long DESC_CRUISE_TIMEOUT_MS = 20000;
