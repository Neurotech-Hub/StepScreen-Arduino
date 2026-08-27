#include "StepScreenTMC2209.h"

bool StepScreenTMC2209::begin(uint32_t baud) {
  Serial1.begin(baud);
  _driver.begin();
  _ready = false;
  _configured = false;
  return true;
}

bool StepScreenTMC2209::verify() {
  const uint8_t ver = _driver.version();
  _ready = (ver == 0x21);
  return _ready;
}

bool StepScreenTMC2209::configureForStepDir(uint16_t microsteps,
                                            uint16_t runCurrentMa) {
  if (!_ready) {
    return false;
  }

  _driver.pdn_disable(true);
  _driver.mstep_reg_select(true);
  _driver.I_scale_analog(false);
  _driver.toff(5);
  _driver.rms_current(runCurrentMa);
  _driver.intpol(true);
  _driver.microsteps(microsteps);

  const uint16_t readback = _driver.microsteps();
  if (readback != microsteps) {
    return false;
  }

  _configured = true;
  return true;
}

void StepScreenTMC2209::printWiringHelp() {
  Serial.println(F("Check in order:"));
  Serial.println(
      F("  1. D1(TX)--[1k]--+--TMC RX(PDN_UART); D0(RX)--+ (same node)"));
  Serial.println(F("  2. TMC TX pad left NC (not wired to Feather)"));
  Serial.println(F("  3. 1kΩ is in the TX leg, not between TX and RX directly"));
  Serial.println(F("  4. GND common between Feather and driver"));
  Serial.println(F("  5. VIO = 3.3V, not 5V"));
  Serial.println(F("  6. VM (motor supply) is ON"));
  Serial.println(
      F("  7. STEPSCREEN_TMC2209_ADDRESS matches MS1/MS2 (0b11 here)"));
}
