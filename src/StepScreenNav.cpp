#include "StepScreenNav.h"

void StepScreenNav::begin(StepScreenInput *input) {
  _input = input;
  enterScreen(); // start settled: ignore any power-on button state
}

void StepScreenNav::enterScreen() {
  _settleUntilMs = millis() + STEPSCREEN_NAV_SETTLE_MS;
  _waitRelease = true;
  if (_input != nullptr) {
    _input->pollButtons();     // flush pending button edges
    _input->getEncoderDelta(); // flush pending encoder detents
  }
}

bool StepScreenNav::anyButtonHeld() {
  return _input->readBack() || _input->readConfirm() || _input->readPush();
}

uint8_t StepScreenNav::pollNavEvents() {
  // Always consume the underlying edge state so stale presses can't
  // accumulate while filtered.
  const uint8_t events = _input->pollButtons();

  if ((int32_t)(millis() - _settleUntilMs) < 0) {
    return STEPSCREEN_EVT_NONE; // still settling after a screen change
  }

  if (_waitRelease) {
    if (!anyButtonHeld()) {
      _waitRelease = false;
    }
    return STEPSCREEN_EVT_NONE;
  }

  if (events != STEPSCREEN_EVT_NONE) {
    _waitRelease = true; // require full release before the next press
  }
  return events;
}

int32_t StepScreenNav::consumeEncoderDelta() {
  return _input->getEncoderDelta();
}
