/*!
 * @file StepScreenPins_override.example.h
 *
 * Template for overriding StepScreen pins without editing the library.
 *
 * Option A (recommended, works in the Arduino IDE):
 *   Copy the #defines you need into your sketch BEFORE the
 *   #include <StepScreen.h> line.
 *
 * Option B (PlatformIO or any build where your project folder is on the
 * include path):
 *   Copy this file into your project as "StepScreenPins_override.h" and
 *   uncomment/edit the pins you want to change. StepScreenPins.h picks
 *   it up automatically via __has_include.
 *
 * Only define the pins you want to change; everything else keeps its
 * default from StepScreenBoard.h / StepScreenPins.h.
 */

#ifndef STEPSCREEN_PINS_OVERRIDE_H
#define STEPSCREEN_PINS_OVERRIDE_H

// --- Screen module ---
// #define PIN_ENC_A 12
// #define PIN_ENC_B 10
// #define PIN_BTN_BACK 11
// #define PIN_BTN_CONFIRM A0
// #define PIN_BTN_PUSH A3
// #define STEPSCREEN_I2C_ADDR 0x3C
// #define STEPSCREEN_OLED_RESET -1

// --- Board baseline ---
// #define PIN_SD_CS 4
// #define PIN_SD_CD 7
// #define PIN_LED_GREEN 8
// #define PIN_LED_RED 13
// #define PIN_VBAT A7

// --- User I/O ---
// #define PIN_USER_LED1 13
// #define PIN_USER_LED2 A1
// #define PIN_EXT_LED A2
// #define PIN_BEAM_DET A4
// #define PIN_AUX_IN A5

// --- TMC2209 stepper ---
// #define PIN_MOTOR_STEP 5
// #define PIN_MOTOR_DIR 6
// #define PIN_MOTOR_EN 9
// #define STEPSCREEN_MOTOR_MICROSTEPS 16
// #define STEPSCREEN_MOTOR_FULL_STEPS 200

#endif // STEPSCREEN_PINS_OVERRIDE_H
