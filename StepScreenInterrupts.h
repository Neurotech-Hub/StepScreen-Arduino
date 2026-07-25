/*!
 * @file StepScreenInterrupts.h
 *
 * Encoder interrupt wiring lives at the SKETCH level, not inside the
 * library, so each sketch stays in control of its interrupt budget.
 * The ISR body (StepScreenInput::handleEncoderISR) is provided by the
 * library; sketches only attach it.
 *
 * ============================================================
 * COPY/PASTE BLOCK -- add to setup() in every new sketch, after
 * screen.begin():
 *
 *   // --- STEPSCREEN ENCODER ISR SETUP (copy into setup()) ---
 *   attachInterrupt(digitalPinToInterrupt(PIN_ENC_A),
 *                   StepScreenInput::handleEncoderISR, CHANGE);
 *   attachInterrupt(digitalPinToInterrupt(PIN_ENC_B),
 *                   StepScreenInput::handleEncoderISR, CHANGE);
 *   // --- END STEPSCREEN ENCODER ISR SETUP ---
 *
 * Or call the macro below, which expands to exactly that block:
 *
 *   STEPSCREEN_ATTACH_ENCODER_ISRS();
 * ============================================================
 *
 * SAMD21 note: each pin maps to one of 16 shared EXTINT lines. The
 * default encoder pins 12 (PA19/EXTINT3) and 10 (PA18/EXTINT2) do not
 * collide with each other or with the polled button pins. If you remap
 * the encoder, make sure the two pins land on different EXTINT lines.
 */

#ifndef STEPSCREEN_INTERRUPTS_H
#define STEPSCREEN_INTERRUPTS_H

#include <Arduino.h>

#include "StepScreenInput.h"
#include "StepScreenPins.h"

#define STEPSCREEN_ATTACH_ENCODER_ISRS()                                     \
  do {                                                                       \
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_A),                        \
                    StepScreenInput::handleEncoderISR, CHANGE);              \
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_B),                        \
                    StepScreenInput::handleEncoderISR, CHANGE);              \
  } while (0)

#endif // STEPSCREEN_INTERRUPTS_H
