/*!
 * @file StepScreenBoard.h
 *
 * Baseline pin map for the Adafruit Feather M0 Adalogger (and boards
 * modeled after it). Reference:
 * https://learn.adafruit.com/adafruit-feather-m0-adalogger/pinouts
 *
 * Every symbol is wrapped in #ifndef so a sketch (or a
 * StepScreenPins_override.h file) can redefine any pin before this
 * header is included.
 */

#ifndef STEPSCREEN_BOARD_H
#define STEPSCREEN_BOARD_H

#include <Arduino.h>

#ifndef PIN_SD_CS
#define PIN_SD_CS 4 // microSD chip select
#endif

#ifndef PIN_SD_CD
#define PIN_SD_CD 7 // microSD card detect (mechanical switch)
#endif

// CD pin level when a card is seated in the slot (INPUT_PULLUP on PIN_SD_CD).
// StepScreen board: LOW when inserted. Stock Adafruit Adalogger: HIGH — override
// in the sketch with #define STEPSCREEN_SD_CD_INSERTED HIGH if needed.
#ifndef STEPSCREEN_SD_CD_INSERTED
#define STEPSCREEN_SD_CD_INSERTED LOW
#endif

static inline bool StepScreenSdCardInserted(uint8_t cdPin = PIN_SD_CD) {
  return digitalRead(cdPin) == STEPSCREEN_SD_CD_INSERTED;
}

#ifndef PIN_LED_GREEN
#define PIN_LED_GREEN 8 // green LED next to the SD slot (Adalogger)
#endif

#ifndef PIN_LED_RED
#define PIN_LED_RED 13 // red LED next to the USB jack (Adalogger)
#endif

// --- User I/O ---

#ifndef PIN_USER_LED1
#define PIN_USER_LED1 13 // D13; same physical LED as PIN_LED_RED on this board
#endif

#ifndef PIN_USER_LED2
#define PIN_USER_LED2 A1 // user indicator LED
#endif

#ifndef PIN_EXT_LED
#define PIN_EXT_LED A2 // drives external LED via N-MOSFET (220 Ω gate series,
                       // 10 kΩ gate pulldown on hardware). HIGH = ON.
#endif

#ifndef PIN_BEAM_DET
#define PIN_BEAM_DET A4 // photodetector input; external pull-up on board.
                      // Use pinMode(PIN_BEAM_DET, INPUT) — no internal pullup.
#endif

#ifndef PIN_AUX_IN
#define PIN_AUX_IN A5 // general-purpose digital input
#endif

#ifndef PIN_VBAT
#define PIN_VBAT A7 // battery voltage divider (pin 9); reads VBAT/2
#endif

#ifndef PIN_I2C_SDA
#define PIN_I2C_SDA 20 // Wire SDA (no onboard pullup)
#endif

#ifndef PIN_I2C_SCL
#define PIN_I2C_SCL 21 // Wire SCL (no onboard pullup)
#endif

// TMC2209 stepper driver (BIGTREETECH module, standalone step/dir mode).
// MS1 + MS2 tied HIGH -> 1/16 microstepping. CLK tied GND -> internal clock.
// UART/INDEX not connected. ~EN is active LOW.
#ifndef PIN_MOTOR_STEP
#define PIN_MOTOR_STEP 5 // 2209_STEP
#endif

#ifndef PIN_MOTOR_DIR
#define PIN_MOTOR_DIR 6 // 2209_DIR
#endif

#ifndef PIN_MOTOR_EN
#define PIN_MOTOR_EN 9 // 2209_~EN (LOW = driver enabled)
#endif

// Hardware microstepping (MS1/MS2 strap pins, not GPIO)
#ifndef STEPSCREEN_MOTOR_MICROSTEPS
#define STEPSCREEN_MOTOR_MICROSTEPS 16
#endif

// Full steps per revolution for a typical 1.8 degree stepper
#ifndef STEPSCREEN_MOTOR_FULL_STEPS
#define STEPSCREEN_MOTOR_FULL_STEPS 200
#endif

#define STEPSCREEN_MOTOR_STEPS_PER_REV                                          \
  ((int32_t)STEPSCREEN_MOTOR_FULL_STEPS * (int32_t)STEPSCREEN_MOTOR_MICROSTEPS)

#endif // STEPSCREEN_BOARD_H
