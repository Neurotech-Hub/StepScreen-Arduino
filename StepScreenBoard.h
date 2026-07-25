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
#define PIN_SD_CD 7 // microSD card detect (HIGH = card inserted)
#endif

#ifndef PIN_LED_GREEN
#define PIN_LED_GREEN 8 // green LED next to the SD slot
#endif

#ifndef PIN_LED_RED
#define PIN_LED_RED 13 // red LED next to the USB jack
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

#endif // STEPSCREEN_BOARD_H
