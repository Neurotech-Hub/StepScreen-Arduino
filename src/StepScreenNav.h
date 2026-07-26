/*!
 * @file StepScreenNav.h
 *
 * Menu-safe navigation layer on top of StepScreenInput. Fixes the two
 * classic sources of menu mis-fires that raw electrical debouncing
 * (StepScreenInput's 20 ms filter) cannot catch:
 *
 *  1. Transition settle: after switching screens, all button edges are
 *     discarded for STEPSCREEN_NAV_SETTLE_MS so a press that triggered
 *     the transition can't also act on the new screen.
 *  2. Release-before-accept: after a press is delivered, no further
 *     presses are accepted until every button has been released, so a
 *     held button can't repeat-fire or leak into the next screen.
 *
 * Usage pattern:
 *
 *   StepScreenNav nav;
 *   nav.begin(&screen.input());
 *
 *   // on EVERY screen/mode change:
 *   nav.enterScreen();
 *
 *   // once per loop():
 *   uint8_t ev = nav.pollNavEvents();       // STEPSCREEN_EVT_* bitmask
 *   int32_t enc = nav.consumeEncoderDelta();
 */

#ifndef STEPSCREEN_NAV_H
#define STEPSCREEN_NAV_H

#include <Arduino.h>

#include "StepScreenInput.h"

// How long to discard button edges after enterScreen()
#ifndef STEPSCREEN_NAV_SETTLE_MS
#define STEPSCREEN_NAV_SETTLE_MS 250
#endif

class StepScreenNav {
public:
  void begin(StepScreenInput *input);

  // Call on every screen/mode change: starts the settle window,
  // requires all buttons to be released, and flushes pending button
  // edges and encoder detents.
  void enterScreen();

  // Filtered press events (STEPSCREEN_EVT_* bitmask). Call exactly once
  // per loop() iteration; it consumes the underlying edge state.
  uint8_t pollNavEvents();

  // Encoder detents since the last call. Flushed by enterScreen().
  int32_t consumeEncoderDelta();

  // Debounced level state, for UI highlights only (not navigation)
  bool readBack() { return _input->readBack(); }
  bool readPush() { return _input->readPush(); }
  bool readConfirm() { return _input->readConfirm(); }

private:
  bool anyButtonHeld();

  StepScreenInput *_input = nullptr;
  uint32_t _settleUntilMs = 0;
  bool _waitRelease = false;
};

#endif // STEPSCREEN_NAV_H
