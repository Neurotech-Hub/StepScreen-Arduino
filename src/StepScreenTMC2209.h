/*!
 * @file StepScreenTMC2209.h
 *
 * Half-duplex UART configuration for the BIGTREETECH TMC2209 on the
 * StepScreen board. Wraps TMCStepper + Serial1 (D0/D1).
 *
 * Wiring:
 *   D1 (TX) --[1kΩ]--+-- TMC PDN_UART
 *   D0 (RX) ---------+
 *   TMC TX pad: NC
 *
 * Call configureForStepDir() before enabling the driver (StepScreenMotor
 * or StepScreenStepper). Microsteps default to STEPSCREEN_MOTOR_MICROSTEPS.
 *
 *   StepScreenTMC2209 tmc;
 *   tmc.begin();
 *   if (tmc.verify()) {
 *     tmc.configureForStepDir();
 *   }
 */

#ifndef STEPSCREEN_TMC2209_H
#define STEPSCREEN_TMC2209_H

#include <Arduino.h>
#include <TMCStepper.h>

#include "StepScreenBoard.h"

class StepScreenTMC2209 {
public:
  StepScreenTMC2209()
      : _driver(&Serial1, STEPSCREEN_TMC2209_R_SENSE,
                STEPSCREEN_TMC2209_ADDRESS) {}

  bool begin(uint32_t baud = STEPSCREEN_TMC2209_BAUD);
  bool verify(); // version() == 0x21
  // Returns false if UART microstep write/read mismatch.
  bool configureForStepDir(
      uint16_t microsteps = STEPSCREEN_MOTOR_MICROSTEPS,
      uint16_t runCurrentMa = STEPSCREEN_TMC2209_RUN_CURRENT_MA);

  bool isReady() const { return _ready; }
  TMC2209Stepper &driver() { return _driver; }

  static void printWiringHelp();

private:
  TMC2209Stepper _driver;
  bool _ready = false;
  bool _configured = false;
};

#endif // STEPSCREEN_TMC2209_H
