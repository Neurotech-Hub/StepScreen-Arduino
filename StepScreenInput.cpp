#include "StepScreenInput.h"

volatile int32_t StepScreenInput::_quarterSteps = 0;
volatile uint8_t StepScreenInput::_lastEncState = 0;
volatile uint8_t StepScreenInput::_pinEncA = PIN_ENC_A;
volatile uint8_t StepScreenInput::_pinEncB = PIN_ENC_B;

// Quadrature transition table, indexed by (prevState << 2) | newState
// where state = (A << 1) | B. Invalid transitions (bounce/skips) = 0.
static const int8_t QUAD_TABLE[16] = {
    0, -1, 1, 0,  //
    1, 0,  0, -1, //
    -1, 0, 0, 1,  //
    0, 1,  -1, 0  //
};

void StepScreenInput::begin(uint8_t encA, uint8_t encB, uint8_t btnBack,
                            uint8_t btnConfirm, uint8_t btnPush) {
  _pinEncA = encA;
  _pinEncB = encB;
  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);
  _lastEncState =
      (uint8_t)((digitalRead(encA) << 1) | digitalRead(encB));
  _quarterSteps = 0;
  _lastReadDetents = 0;

  initButton(_back, btnBack);
  initButton(_confirm, btnConfirm);
  initButton(_push, btnPush);
}

void StepScreenInput::handleEncoderISR() {
  const uint8_t state =
      (uint8_t)((digitalRead(_pinEncA) << 1) | digitalRead(_pinEncB));
  _quarterSteps += QUAD_TABLE[(_lastEncState << 2) | state];
  _lastEncState = state;
}

int32_t StepScreenInput::getEncoderCount() {
  noInterrupts();
  const int32_t quarters = _quarterSteps;
  interrupts();
  // One EC11 detent = 4 quadrature transitions
  return quarters / 4;
}

int32_t StepScreenInput::getEncoderDelta() {
  const int32_t count = getEncoderCount();
  const int32_t delta = count - _lastReadDetents;
  _lastReadDetents = count;
  return delta;
}

uint8_t StepScreenInput::pollButtons() {
  uint8_t events = STEPSCREEN_EVT_NONE;
  if (updateButton(_back) && _back.stable)
    events |= STEPSCREEN_EVT_BACK;
  if (updateButton(_confirm) && _confirm.stable)
    events |= STEPSCREEN_EVT_CONFIRM;
  if (updateButton(_push) && _push.stable)
    events |= STEPSCREEN_EVT_PUSH;
  return events;
}

void StepScreenInput::initButton(Button &b, uint8_t pin) {
  b.pin = pin;
  pinMode(pin, INPUT_PULLUP);
  b.stable = false;
  b.lastRaw = false;
  b.lastEdgeMs = millis();
}

bool StepScreenInput::updateButton(Button &b) {
  const bool raw = (digitalRead(b.pin) == LOW); // active LOW
  const uint32_t now = millis();
  if (raw != b.lastRaw) {
    b.lastRaw = raw;
    b.lastEdgeMs = now;
  }
  if ((now - b.lastEdgeMs) >= STEPSCREEN_DEBOUNCE_MS && b.stable != raw) {
    b.stable = raw;
    return true;
  }
  return false;
}
