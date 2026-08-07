/*
  StepScreen DriverTest

  UART bring-up test for the BIGTREETECH TMC2209, before trusting any
  microstep/current settings written over the wire. Confirms the SAMD21
  can actually talk to the driver, then does a short physical move so a
  UART-configured setting (microsteps) is visibly reflected in motion.

  Requires the TMCStepper library (teemuatlut) -- install via Library
  Manager. Not a StepScreen library dependency; only needed for this test
  / future driver work.

  Wiring (half-duplex single-wire UART, resistor on the TX leg only):
    SAMD21 D1 (TX, Serial1) --[1k]-- TMC2209 TX pad --- PDN_UART
    SAMD21 D0 (RX, Serial1) ------------------------ TMC2209 RX pad --- PDN_UART
    SAMD21 GND               ------------------------ TMC2209 GND
    TMC2209 VIO must be 3.3V (do not feed 5V logic)

  MS1 + MS2 tied HIGH on this board -> UART node address 0b11 (3).
  In UART mode those pins stop selecting microsteps and only set the
  address; STEPSCREEN_MOTOR_MICROSTEPS (StepScreenBoard.h) still
  describes the hardware-strap default and is NOT updated by this test.

  Sequence:
    1. Serial1.begin() + driver.begin(), read IOIN and version().
       version() must read 0x21 -- anything else means wiring/address/
       baud is wrong and the rest of the test is skipped.
    2. Enable UART microstep control (pdn_disable, mstep_reg_select),
       set a known microstep value, read it back.
    3. Enable the driver and issue a short blocking move via
       StepScreenMotor so the configured microstep count is visibly
       one motor movement, not just a register readback.

  Board: Adafruit Feather M0 Adalogger (or compatible).
*/

#include <StepScreenMotor.h>
#include <TMCStepper.h>

// Confirm with a multimeter (driver powered off) before relying on this:
// R_SENSE is the driver's current-sense resistor value, not a wiring
// choice -- BIGTREETECH TMC2209 modules use 0.11 ohm.
#define R_SENSE 0.11f

// MS1 + MS2 tied HIGH -> UART address 0b11. Wrong address is the most
// common reason version() reads back 0x00 or 0xFF.
#define DRIVER_ADDRESS 0b11

#define TEST_MICROSTEPS 64
#define TEST_RUN_CURRENT_MA 600

TMC2209Stepper driver(&Serial1, R_SENSE, DRIVER_ADDRESS);
StepScreenMotor motor;

bool driverOk = false;

void printHex8(uint8_t v) {
  if (v < 0x10) {
    Serial.print('0');
  }
  Serial.println(v, HEX);
}

void reportIOIN() {
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

void setup() {
  Serial.begin(115200);
  delay(1000); // let USB serial settle; no while(!Serial) so battery works

  Serial.println(F("StepScreen DriverTest"));
  Serial.println(F("Bring-up test for TMC2209 UART -- run before any motor test."));
  Serial.println();

  Serial1.begin(115200);
  driver.begin();

  // ms1/ms2/pdn_uart read the driver's own pins over UART -- if these
  // come back wrong (or the read times out entirely), the problem is
  // wiring, not software.
  reportIOIN();

  const uint8_t ver = driver.version();
  Serial.print(F("driver.version() = 0x"));
  printHex8(ver);

  if (ver != 0x21) {
    Serial.println();
    Serial.println(F("FAIL: expected 0x21. UART is not communicating."));
    Serial.println(F("Check in order:"));
    Serial.println(F("  1. TX/RX not swapped (D1=TX->1k->PDN, D0=RX->PDN)"));
    Serial.println(F("  2. GND common between Feather and driver"));
    Serial.println(F("  3. VIO = 3.3V, not 5V"));
    Serial.println(F("  4. 1k resistor is in the TX leg, not RX"));
    Serial.println(F("  5. DRIVER_ADDRESS matches MS1/MS2 strapping (0b11 here)"));
    Serial.println(F("  6. No filter capacitor on PDN_UART blocking the signal"));
    return;
  }

  Serial.println(F("PASS: UART link confirmed."));
  Serial.println();

  // --- UART microstep control ---
  driver.pdn_disable(true);      // PDN_UART pin = UART, not power-down
  driver.mstep_reg_select(true); // microsteps come from MRES, not MS1/MS2
  driver.I_scale_analog(false);  // current fully software-controlled
  driver.toff(5);                // chopper on
  driver.rms_current(TEST_RUN_CURRENT_MA);
  driver.intpol(true);
  driver.microsteps(TEST_MICROSTEPS);

  const uint16_t msRead = driver.microsteps();
  Serial.print(F("Requested microsteps = "));
  Serial.println(TEST_MICROSTEPS);
  Serial.print(F("Readback microsteps  = "));
  Serial.println(msRead);

  if (msRead != TEST_MICROSTEPS) {
    Serial.println(F("FAIL: microstep readback does not match. UART writes"));
    Serial.println(F("are not landing (check RX leg / GND) even though reads work."));
    return;
  }
  Serial.println(F("PASS: microstep register set and confirmed."));
  Serial.println();

  driverOk = true;

  // --- Physical move: confirms the UART-configured microstep count is
  // really what the motor executes, not just what the register reports.
  motor.begin();
  motor.enable();
  delay(5); // TMC2209 needs a short settle after ~EN goes low

  const uint32_t stepsForOneTenthRev =
      (200UL * TEST_MICROSTEPS) / 10; // 200 full steps/rev, 1/10 rev

  Serial.print(F("Moving "));
  Serial.print(stepsForOneTenthRev);
  Serial.print(F(" microsteps forward at 1/"));
  Serial.print(TEST_MICROSTEPS);
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
  Serial.println(F("each way (smoothly, at 1/64), UART + stepping both work."));
  Serial.println(F("Re-upload to repeat."));
}

void loop() {
  // One-shot test; everything runs in setup(). Nothing to do here.
}
