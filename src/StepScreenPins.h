/*!
 * @file StepScreenPins.h
 *
 * Pin map for the 1.3" SH1106 OLED + EC11 rotary encoder module.
 *
 * Overriding pins
 * ---------------
 * Two mechanisms are supported (both leave the library untouched):
 *
 * 1. Define pins in the sketch BEFORE including <StepScreen.h>
 *    (works in Arduino IDE and everywhere else):
 *
 *      #define PIN_ENC_A 6
 *      #include <StepScreen.h>
 *
 * 2. Place a StepScreenPins_override.h file on the include path
 *    (e.g. the src/ folder of a PlatformIO project). See
 *    StepScreenPins_override.example.h for a template.
 */

#ifndef STEPSCREEN_PINS_H
#define STEPSCREEN_PINS_H

#include <Arduino.h>

#include "StepScreenBoard.h"

#if defined(__has_include)
#if __has_include("StepScreenPins_override.h")
#include "StepScreenPins_override.h"
#endif
#endif

// EC11 rotary encoder (quadrature). Swap A/B to flip rotation direction.
#ifndef PIN_ENC_A
#define PIN_ENC_A 12 // SCREEN_TRA
#endif

#ifndef PIN_ENC_B
#define PIN_ENC_B 10 // SCREEN_TRB
#endif

// Push buttons (active LOW, internal pullups enabled by the library)
#ifndef PIN_BTN_BACK
#define PIN_BTN_BACK 11 // SCREEN_BAK
#endif

#ifndef PIN_BTN_CONFIRM
#define PIN_BTN_CONFIRM A0 // SCREEN_CONFIRM
#endif

#ifndef PIN_BTN_PUSH
#define PIN_BTN_PUSH A3 // SCREEN_PUSH (encoder shaft push)
#endif

// SH1106 OLED over I2C (Wire on pins 20/21)
#ifndef STEPSCREEN_I2C_ADDR
#define STEPSCREEN_I2C_ADDR 0x3C
#endif

#ifndef STEPSCREEN_OLED_RESET
#define STEPSCREEN_OLED_RESET -1 // no reset pin on the module
#endif

#endif // STEPSCREEN_PINS_H
