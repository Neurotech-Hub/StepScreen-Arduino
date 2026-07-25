/*!
 * @file StepScreenInput.h
 *
 * Buttons + EC11 encoder state for the StepScreen module.
 *
 * Buttons (Back, Confirm, encoder Push) are active LOW, polled, and
 * software-debounced -- no interrupts needed.
 *
 * The encoder IS interrupt-driven, but this library never calls
 * attachInterrupt() itself: paste the marked block from
 * StepScreenInterrupts.h into your sketch's setup(). The ISR body
 * (handleEncoderISR) lives here; sketches only wire it up.
 */

#ifndef STEPSCREEN_INPUT_H
#define STEPSCREEN_INPUT_H

#include <Arduino.h>

#include "StepScreenPins.h"

// Event bits returned by pollButtons()
#define STEPSCREEN_EVT_NONE 0x00
#define STEPSCREEN_EVT_BACK 0x01
#define STEPSCREEN_EVT_CONFIRM 0x02
#define STEPSCREEN_EVT_PUSH 0x04

#ifndef STEPSCREEN_DEBOUNCE_MS
#define STEPSCREEN_DEBOUNCE_MS 20
#endif

class StepScreenInput {
public:
  // Default pins come from StepScreenPins.h; defaults are resolved at
  // the call site, so sketch-level pin overrides apply automatically.
  void begin(uint8_t encA = PIN_ENC_A, uint8_t encB = PIN_ENC_B,
             uint8_t btnBack = PIN_BTN_BACK,
             uint8_t btnConfirm = PIN_BTN_CONFIRM,
             uint8_t btnPush = PIN_BTN_PUSH);

  // Debounced button state: true while held down
  bool readBack() { return readButton(_back); }
  bool readConfirm() { return readButton(_confirm); }
  bool readPush() { return readButton(_push); }

  // Edge-detected press events since the last call, as a bitmask of
  // STEPSCREEN_EVT_* values. Call once per loop() iteration.
  uint8_t pollButtons();

  // Encoder detents since the last call (positive = one direction,
  // negative = the other; swap PIN_ENC_A/B to flip the sign)
  int32_t getEncoderDelta();

  // Absolute detent count since begin()
  int32_t getEncoderCount();

  // Quadrature ISR body -- attach from the sketch (see
  // StepScreenInterrupts.h), never called directly.
  static void handleEncoderISR();

private:
  struct Button {
    uint8_t pin;
    bool stable;        // debounced state (true = pressed)
    bool lastRaw;       // last raw reading
    uint32_t lastEdgeMs;
  };

  void initButton(Button &b, uint8_t pin);
  bool updateButton(Button &b); // returns true when stable state changes
  bool readButton(Button &b) {
    updateButton(b);
    return b.stable;
  }

  Button _back, _confirm, _push;
  int32_t _lastReadDetents = 0;

  // Static so the ISR can reach them; the module has a single encoder,
  // so treat StepScreenInput as a singleton.
  static volatile int32_t _quarterSteps;
  static volatile uint8_t _lastEncState;
  static volatile uint8_t _pinEncA, _pinEncB;
};

#endif // STEPSCREEN_INPUT_H
