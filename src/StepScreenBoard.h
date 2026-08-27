/*!
 * @file StepScreenBoard.h
 *
 * Pin map for the StepScreen board (Feather M0–compatible SAMD21 carrier).
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
#define PIN_SD_CD 7 // SD card detect (D7 / SD_DET)
#endif

// CD pin level when a card is seated in the slot (INPUT_PULLUP on PIN_SD_CD).
#ifndef STEPSCREEN_SD_CD_INSERTED
#define STEPSCREEN_SD_CD_INSERTED LOW
#endif

static inline bool StepScreenSdCardInserted(uint8_t cdPin = PIN_SD_CD) {
  return digitalRead(cdPin) == STEPSCREEN_SD_CD_INSERTED;
}

#ifndef PIN_LED_GREEN
#define PIN_LED_GREEN 13 // D13 — green (USER_LED1)
#endif

#ifndef PIN_LED_RED
#define PIN_LED_RED A1 // A1 — red (USER_LED2)
#endif

#ifndef PIN_USER_LED1
#define PIN_USER_LED1 PIN_LED_GREEN
#endif

#ifndef PIN_USER_LED2
#define PIN_USER_LED2 PIN_LED_RED
#endif

#ifndef PIN_EXT_LED
#define PIN_EXT_LED A2 // external LED via N-MOSFET; HIGH = ON
#endif

#ifndef PIN_BEAM_DET
#define PIN_BEAM_DET A4 // photodetector; external pull-up — use INPUT
#endif

#ifndef PIN_AUX_IN
#define PIN_AUX_IN A5 // general-purpose digital input
#endif

#ifndef PIN_VBAT
#define PIN_VBAT A7 // battery voltage divider; reads VBAT/2
#endif

#ifndef PIN_I2C_SDA
#define PIN_I2C_SDA 20 // Wire SDA
#endif

#ifndef PIN_I2C_SCL
#define PIN_I2C_SCL 21 // Wire SCL
#endif

#ifndef STEPSCREEN_RTC_I2C_ADDR
#define STEPSCREEN_RTC_I2C_ADDR 0x68 // PCF8523 on Adalogger (optional)
#endif

// SWD pads repurposed as GPIO (silkscreen SWCLK / SWDIO). Not in the stock
// Feather M0 pin map — use StepScreenHX711 for GPIO access.
#ifndef PIN_SWCLK
#define PIN_SWCLK 30 // PA30 / SWCLK — HX711 SCK
#endif

#ifndef PIN_SWDIO
#define PIN_SWDIO 31 // PA31 / SWDIO — HX711 DT
#endif

#ifndef PIN_HX711_SCK
#define PIN_HX711_SCK PIN_SWCLK
#endif

#ifndef PIN_HX711_DT
#define PIN_HX711_DT PIN_SWDIO
#endif

// TMC2209 stepper (BIGTREETECH module): step/dir + half-duplex UART.
// MS1 + MS2 tied HIGH -> UART address 0b11 (not hardware microstep count
// once UART mstep_reg_select is enabled). ~EN active LOW.
#ifndef PIN_MOTOR_STEP
#define PIN_MOTOR_STEP 5 // D5 / 2209_STEP
#endif

#ifndef PIN_MOTOR_DIR
#define PIN_MOTOR_DIR 6 // D6 / 2209_DIR
#endif

#ifndef PIN_MOTOR_EN
#define PIN_MOTOR_EN 9 // D9 / 2209_~EN
#endif

#ifndef PIN_MOTOR_UART_RX
#define PIN_MOTOR_UART_RX 0 // D0 — half-duplex UART node with TX
#endif

#ifndef PIN_MOTOR_UART_TX
#define PIN_MOTOR_UART_TX 1 // D1 — 1kΩ in series to TMC PDN_UART
#endif

#ifndef STEPSCREEN_TMC2209_ADDRESS
#define STEPSCREEN_TMC2209_ADDRESS 0b11 // MS1 + MS2 high
#endif

#ifndef STEPSCREEN_TMC2209_R_SENSE
#define STEPSCREEN_TMC2209_R_SENSE 0.11f
#endif

#ifndef STEPSCREEN_TMC2209_BAUD
#define STEPSCREEN_TMC2209_BAUD 115200
#endif

#ifndef STEPSCREEN_TMC2209_RUN_CURRENT_MA
#define STEPSCREEN_TMC2209_RUN_CURRENT_MA 600
#endif

#ifndef STEPSCREEN_MOTOR_MICROSTEPS
#define STEPSCREEN_MOTOR_MICROSTEPS 128 // default; override in sketch if needed
#endif

#ifndef STEPSCREEN_MOTOR_FULL_STEPS
#define STEPSCREEN_MOTOR_FULL_STEPS 200
#endif

#define STEPSCREEN_MOTOR_STEPS_PER_REV                                          \
  ((int32_t)STEPSCREEN_MOTOR_FULL_STEPS * (int32_t)STEPSCREEN_MOTOR_MICROSTEPS)

#endif // STEPSCREEN_BOARD_H
