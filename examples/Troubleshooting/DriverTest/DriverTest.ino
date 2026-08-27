/*
  StepScreen DriverTest

  UART bring-up test for the BIGTREETECH TMC2209, before trusting any
  microstep/current settings written over the wire. Confirms the SAMD21
  can actually talk to the driver, then does a short physical move so a
  UART-configured setting (microsteps) is visibly reflected in motion.

  Requires the TMCStepper library (teemuatlut) — listed in library.properties.

  Wiring (half-duplex single-wire UART on the StepScreen TMC2209 carrier):
    SAMD21 D1 (TX, Serial1) --[1kΩ]--+
    SAMD21 D0 (RX, Serial1) ---------+--- TMC2209 RX pad (PDN_UART on chip)
    SAMD21 GND               ------------------------ TMC2209 GND
    TMC2209 TX pad is NC on this board -- leave unconnected.
    TMC2209 VIO must be 3.3V (do not feed 5V logic)
    TMC2209 VM (motor supply) must be ON -- UART will not respond without it.

  The 1kΩ sits in the TX leg only; RX joins the same node directly. This
  matches the TMC2209 single-wire UART: one bus to PDN_UART, not crossed TX/RX.

  MS1 + MS2 tied HIGH on this board -> UART node address 0b11 (3).
  In UART mode those pins stop selecting microsteps and only set the
  address; microsteps are configured via StepScreenTMC2209.

  Sequence:
    1. StepScreenTMC2209::begin() + verify() — version() must read 0x21.
    2. configureForStepDir(), then write/read several microstep values.
    3. Enable the driver and issue a short blocking move via StepScreenMotor
       at STEPSCREEN_MOTOR_MICROSTEPS (board default).

  Board: Adafruit Feather M0 Adalogger (or compatible).
*/

#include <StepScreenMotor.h>
#include <StepScreenTMC2209.h>

// Valid TMC2209 MRES values exercised in the write/read sweep.
const uint16_t TEST_MICROSTEP_VALUES[] = {16, 32, 64, 128, 256};
const uint8_t TEST_MICROSTEP_COUNT =
    sizeof(TEST_MICROSTEP_VALUES) / sizeof(TEST_MICROSTEP_VALUES[0]);

StepScreenTMC2209 tmc;
StepScreenMotor motor;

void printHex8(uint8_t v) {
  if (v < 0x10) {
    Serial.print('0');
  }
  Serial.println(v, HEX);
}

void reportIOIN(TMC2209Stepper &driver) {
  Serial.println(F("--- IOIN (physical pin state as seen by the driver) ---"));
  Serial.print(F("  ms1      = "));
  Serial.println(driver.ms1());
  Serial.print(F("  ms2      = "));
  Serial.println(driver.ms2());
  Serial.print(F("  pdn_uart = "));
  Serial.println(driver.pdn_uart());
  Serial.print(F("  enn      = "));
  Serial.println(driver.enn());
}

bool testMicrostepWriteRead(TMC2209Stepper &driver, uint16_t requested) {
  driver.microsteps(requested);
  const uint16_t readback = driver.microsteps();

  Serial.print(F("  set "));
  Serial.print(requested);
  Serial.print(F(" -> read "));
  Serial.print(readback);
  Serial.print(F("  "));

  if (readback == requested) {
    Serial.println(F("PASS"));
    return true;
  }

  Serial.println(F("FAIL"));
  return false;
}

bool runMicrostepSweep(TMC2209Stepper &driver) {
  Serial.println(F("--- Microstep write/read sweep ---"));
  bool allOk = true;

  for (uint8_t i = 0; i < TEST_MICROSTEP_COUNT; i++) {
    if (!testMicrostepWriteRead(driver, TEST_MICROSTEP_VALUES[i])) {
      allOk = false;
    }
  }

  Serial.println();
  if (allOk) {
    Serial.println(F("PASS: all microstep values read back correctly."));
  } else {
    Serial.println(F("FAIL: one or more microstep values did not read back."));
  }
  Serial.println();
  return allOk;
}

void setup() {
  Serial.begin(115200);
  delay(2000); // let USB serial settle; no while(!Serial) so battery works

  Serial.println(F("StepScreen DriverTest"));
  Serial.println(F("Bring-up test for TMC2209 UART -- run before any motor test."));
  Serial.println(F("UART: D1--[1k]--+--TMC RX(PDN); D0--+ ; TMC TX = NC"));
  Serial.println();

  tmc.begin();
  TMC2209Stepper &driver = tmc.driver();

  reportIOIN(driver);

  const uint8_t ver = driver.version();
  Serial.print(F("driver.version() = 0x"));
  printHex8(ver);

  if (!tmc.verify()) {
    Serial.println();
    Serial.println(F("FAIL: expected 0x21. UART is not communicating."));
    StepScreenTMC2209::printWiringHelp();
    return;
  }

  Serial.println(F("PASS: UART link confirmed."));
  Serial.println();

  if (!tmc.configureForStepDir()) {
    Serial.println(F("FAIL: configureForStepDir()"));
    return;
  }

  if (!runMicrostepSweep(driver)) {
    return;
  }

  tmc.configureForStepDir(STEPSCREEN_MOTOR_MICROSTEPS);

  motor.begin();
  motor.enable();
  delay(5); // TMC2209 needs a short settle after ~EN goes low

  const uint32_t stepsForOneTenthRev = STEPSCREEN_MOTOR_STEPS_PER_REV / 10;

  Serial.print(F("Moving "));
  Serial.print(stepsForOneTenthRev);
  Serial.print(F(" microsteps forward at 1/"));
  Serial.print(STEPSCREEN_MOTOR_MICROSTEPS);
  Serial.println(F(" ..."));

  motor.setDirection(true);
  for (uint32_t i = 0; i < stepsForOneTenthRev; i++) {
    motor.step();
    delayMicroseconds(300);
  }

  delay(300);
  motor.setDirection(false);
  for (uint32_t i = 0; i < stepsForOneTenthRev; i++) {
    motor.step();
    delayMicroseconds(300);
  }

  motor.disable();
  Serial.println(F("Move complete. If the shaft visibly turned ~1/10 rev"));
  Serial.print(F("each way (smoothly, at 1/"));
  Serial.print(STEPSCREEN_MOTOR_MICROSTEPS);
  Serial.println(F("), UART + stepping both work."));
  Serial.println(F("Re-upload to repeat."));
}

void loop() {
  // One-shot test; everything runs in setup(). Nothing to do here.
}
