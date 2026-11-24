#include "drv_encoder.h"
#if defined(__AVR__)
  #include <avr/interrupt.h>
#endif

// Quadrature transition table, indexed by (previousAB << 2) | currentAB.
#if defined(ESP32)
static DRAM_ATTR const int8_t kTable[16] = {
#else
static const int8_t kTable[16] = {
#endif
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

#if defined(ESP32)

static void IRAM_ATTR isrEsp(void* arg) {
  static_cast<DrvEncoder*>(arg)->onEdge();
}

#elif defined(__AVR__)

static DrvEncoder* gIntEnc[2] = {nullptr, nullptr};
static uint8_t gIntN = 0;
static DrvEncoder* gPcEnc[2] = {nullptr, nullptr};
static uint8_t gPcN = 0;

static void dispatchInt() {
  for (uint8_t i = 0; i < gIntN; i++) gIntEnc[i]->onEdge();
}
static void isrInt0() { dispatchInt(); }
static void isrInt1() { dispatchInt(); }

// Calling onEdge on an encoder whose pins did not change is harmless:
// the table gives 0 when the previous and current states are equal.
ISR(PCINT1_vect) {
  for (uint8_t i = 0; i < gPcN; i++) gPcEnc[i]->onEdge();
}

#endif

DrvEncoder::DrvEncoder(uint8_t pinA, uint8_t pinB)
  : _a(pinA), _b(pinB), _count(0), _prev(0) {}

void DrvEncoder::begin() {
  pinMode(_a, INPUT_PULLUP);
  pinMode(_b, INPUT_PULLUP);
  _prev = (digitalRead(_a) << 1) | digitalRead(_b);

#if defined(ESP32)
  attachInterruptArg(digitalPinToInterrupt(_a), isrEsp, this, CHANGE);
  attachInterruptArg(digitalPinToInterrupt(_b), isrEsp, this, CHANGE);
#elif defined(__AVR__)
  bool intPins = (_a == 2 || _a == 3) && (_b == 2 || _b == 3);
  if (intPins) {
    if (gIntN < 2) gIntEnc[gIntN++] = this;
    attachInterrupt(digitalPinToInterrupt(_a), _a == 2 ? isrInt0 : isrInt1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(_b), _b == 2 ? isrInt0 : isrInt1, CHANGE);
  } else {
    if (gPcN < 2) gPcEnc[gPcN++] = this;
    PCICR |= _BV(PCIE1);
    PCMSK1 |= _BV(_a - A0) | _BV(_b - A0);
  }
#endif
}

long DrvEncoder::count() const {
#if defined(ESP32)
  return _count;          // 32-bit aligned read is a single instruction on the ESP32
#else
  noInterrupts();         // 32-bit read takes several instructions on the 8-bit AVR
  long c = _count;
  interrupts();
  return c;
#endif
}

void DRV_ISR_ATTR DrvEncoder::onEdge() {
  uint8_t cur = (digitalRead(_a) << 1) | digitalRead(_b);
  _count += kTable[(_prev << 2) | cur];
  _prev = cur;
}
