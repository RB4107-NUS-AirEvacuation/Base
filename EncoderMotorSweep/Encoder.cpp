#include "Encoder.h"

// Quadrature transition table, indexed by (previousAB << 2) | currentAB.
static const int8_t kTransitionTable[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

Encoder* Encoder::_instance = nullptr;

Encoder::Encoder(uint8_t pinA, uint8_t pinB)
  : _pinA(pinA), _pinB(pinB) {}

void Encoder::begin() {
  pinMode(_pinA, INPUT_PULLUP);
  pinMode(_pinB, INPUT_PULLUP);

  _prevAB = (digitalRead(_pinA) << 1) | digitalRead(_pinB);
  _instance = this;

  attachInterrupt(digitalPinToInterrupt(_pinA), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(_pinB), isrB, CHANGE);
}

long Encoder::count() const {
  noInterrupts();
  long c = _count;
  interrupts();
  return c;
}

void Encoder::reset() {
  noInterrupts();
  _count = 0;
  interrupts();
}

void Encoder::update() {
  uint8_t currentAB = (digitalRead(_pinA) << 1) | digitalRead(_pinB);
  uint8_t transition = (_prevAB << 2) | currentAB;
  _count += kTransitionTable[transition];
  _prevAB = currentAB;
}

void Encoder::isrA() { if (_instance) _instance->update(); }
void Encoder::isrB() { if (_instance) _instance->update(); }
