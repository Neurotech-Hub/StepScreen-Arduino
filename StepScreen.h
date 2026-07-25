/*!
 * @file StepScreen.h
 *
 * Main entry point for the StepScreen library: a UI layer for an
 * Adalogger M0-compatible board with a 1.3" SH1106 OLED + EC11 rotary
 * encoder module (I2C display, three active-LOW buttons).
 *
 * Quick start:
 *
 *   #include <StepScreen.h>
 *
 *   StepScreen screen;
 *
 *   void setup() {
 *     screen.begin();
 *     // --- STEPSCREEN ENCODER ISR SETUP (copy into setup()) ---
 *     attachInterrupt(digitalPinToInterrupt(PIN_ENC_A),
 *                     StepScreenInput::handleEncoderISR, CHANGE);
 *     attachInterrupt(digitalPinToInterrupt(PIN_ENC_B),
 *                     StepScreenInput::handleEncoderISR, CHANGE);
 *     // --- END STEPSCREEN ENCODER ISR SETUP ---
 *   }
 *
 *   void loop() {
 *     int32_t delta = screen.input().getEncoderDelta();
 *     screen.drawInfoBar("My App");
 *     screen.drawActionColumn(false, false, false);
 *     screen.display().display();
 *   }
 *
 * Facade methods are defined inline so that pin macro defaults are
 * resolved in the sketch's translation unit -- this is what makes
 * sketch-level pin overrides (see StepScreenPins.h) work.
 */

// Guard name avoids STEPSCREEN_H, which is the screen-height macro
#ifndef STEPSCREEN_MAIN_H
#define STEPSCREEN_MAIN_H

#include "StepScreenBoard.h"
#include "StepScreenDisplay.h"
#include "StepScreenInput.h"
#include "StepScreenInterrupts.h"
#include "StepScreenLayout.h"
#include "StepScreenPins.h"

class StepScreen {
public:
  // Initializes button/encoder pins and the display. Returns false if
  // the display does not respond on the I2C bus.
  bool begin(uint8_t i2cAddr = STEPSCREEN_I2C_ADDR, uint8_t encA = PIN_ENC_A,
             uint8_t encB = PIN_ENC_B, uint8_t btnBack = PIN_BTN_BACK,
             uint8_t btnConfirm = PIN_BTN_CONFIRM,
             uint8_t btnPush = PIN_BTN_PUSH) {
    _input.begin(encA, encB, btnBack, btnConfirm, btnPush);
    return _display.begin(i2cAddr);
  }

  // Full Adafruit_GFX/SH110X API plus layout helpers
  StepScreenDisplay &display() { return _display; }

  // Buttons + encoder
  StepScreenInput &input() { return _input; }

  // Convenience passthroughs for the most common layout calls
  void drawInfoBar(const char *title, bool highlightBack = false) {
    _display.drawInfoBar(title, highlightBack);
  }
  void drawActionColumn(bool backActive, bool pushActive,
                        bool confirmActive) {
    _display.drawActionColumn(backActive, pushActive, confirmActive);
  }
  void clearContentArea() { _display.clearContentArea(); }

private:
  StepScreenDisplay _display;
  StepScreenInput _input;
};

#endif // STEPSCREEN_MAIN_H
