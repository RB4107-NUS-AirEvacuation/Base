#pragma once
#include <Arduino.h>

// ---- Motor driver ----
constexpr uint8_t PIN_MOTOR_PWM = 5;
constexpr uint8_t PIN_MOTOR_DIR = 6;
constexpr int MOTOR_MAX_PWM = 75;          // raise to 255 after testing

// ---- Motor quadrature encoder ----
constexpr uint8_t PIN_ENC_A = 2;           // interrupt
constexpr uint8_t PIN_ENC_B = 3;           // interrupt
constexpr float COUNTS_PER_REV = 9600.0f;  // from original code

// ---- Potentiometer B10K (10 kOhm, linear taper) ----
// Wiring: outer legs to 5V and GND, wiper to A0.
constexpr uint8_t  PIN_POT     = A0;
constexpr uint16_t POT_ADC_MIN = 0;        // ADC reading at one end stop (calibrate)
constexpr uint16_t POT_ADC_MAX = 1023;     // ADC reading at the other end stop (calibrate)
constexpr float    POT_DEG_MIN = 0.0f;     // angle at POT_ADC_MIN
constexpr float    POT_DEG_MAX = 270.0f;   // angle at POT_ADC_MAX (measure real travel)
// 10k pots have high output impedance, so average more samples to reduce noise.
constexpr uint8_t  POT_SAMPLES = 16;

// ---- Manual control ----
// Fraction of half-travel around center treated as "stop".
constexpr float MANUAL_DEADBAND = 0.05f;

// ---- Telemetry ----
constexpr unsigned long PRINT_PERIOD_MS = 200;
