/*
  StepScreen DriverTest

  UART bring-up test for the BIGTREETECH TMC2209, before trusting any
  microstep/current settings written over the wire. Confirms the SAMD21
  can actually talk to the driver, then does a short physical move so a
  UART-configured setting (microsteps) is visibly reflected in motion.

  Requires the TMCStepper library (teemuatlut) -- install via Library
  Manager. Not a StepScreen library dependency; only needed for this test
  / future driver work.

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
  address; STEPSCREEN_MOTOR_MICROSTEPS (StepScreenBoard.h) still
  describes the hardware-strap default and is NOT updated by this test.

  Sequence:
    1. Serial1.begin() + driver.begin(), read IOIN and version().
       version() must read 0x21 -- anything else means wiring/address/
       baud is wrong and the rest of the test is skipped.
    2. Enable UART microstep control (pdn_disable, mstep_reg_select),
       write/read several microstep values to confirm register access.
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

#define TEST_RUN_CURRENT_MA 600

// Valid TMC2209 MRES values exercised in the write/read sweep.
const uint16_t TEST_MICROSTEP_VALUES[] = {16, 32, 64, 128, 256};
const uint8_t TEST_MICROSTEP_COUNT =
    sizeof(TEST_MICROSTEP_VALUES) / sizeof(TEST_MICROSTEP_VALUES[0]);
#define MOVE_MICROSTEPS 64

TMC2209Stepper driver(&Serial1, R_SENSE, DRIVER_ADDRESS);
StepScreenMotor motor;

bool driverOk = false;

void printHex8(uint8_t v)
{
  if (v < 0x10)
  {
    Serial.print('0');
  }
  Serial.println(v, HEX);
}

void reportIOIN()
{
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

bool testMicrostepWriteRead(uint16_t requested)
{
  driver.microsteps(requested);
  const uint16_t readback = driver.microsteps();

  Serial.print(F("  set "));
  Serial.print(requested);
  Serial.print(F(" -> read "));
  Serial.print(readback);
  Serial.print(F("  "));

  if (readback == requested)
  {
    Serial.println(F("PASS"));
    return true;
  }

  Serial.println(F("FAIL"));
  return false;
}

bool runMicrostepSweep()
{
  Serial.println(F("--- Microstep write/read sweep ---"));
  bool allOk = true;

  for (uint8_t i = 0; i < TEST_MICROSTEP_COUNT; i++)
  {
    if (!testMicrostepWriteRead(TEST_MICROSTEP_VALUES[i]))
    {
      allOk = false;
    }
  }

  Serial.println();
  if (allOk)
  {
    Serial.println(F("PASS: all microstep values read back correctly."));
  }
  else
  {
    Serial.println(F("FAIL: one or more microstep values did not read back."));
  }
  Serial.println();
  return allOk;
}

void setup()
{
  Serial.begin(115200);
  delay(2000); // let USB serial settle; no while(!Serial) so battery works

  Serial.println(F("StepScreen DriverTest"));
  Serial.println(F("Bring-up test for TMC2209 UART -- run before any motor test."));
  Serial.println(F("UART: D1--[1k]--+--TMC RX(PDN); D0--+ ; TMC TX = NC"));
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

  if (ver != 0x21)
  {
    Serial.println();
    Serial.println(F("FAIL: expected 0x21. UART is not communicating."));
    Serial.println(F("Check in order:"));
    Serial.println(F("  1. D1(TX)--[1k]--+--TMC RX(PDN_UART); D0(RX)--+ (same node)"));
    Serial.println(F("  2. TMC TX pad left NC (not wired to Feather)"));
    Serial.println(F("  3. 1kΩ is in the TX leg, not between TX and RX directly"));
    Serial.println(F("  4. GND common between Feather and driver"));
    Serial.println(F("  5. VIO = 3.3V, not 5V"));
    Serial.println(F("  6. VM (motor supply) is ON"));
    Serial.println(F("  7. DRIVER_ADDRESS matches MS1/MS2 strapping (0b11 here)"));
    return;
  }

  Serial.println(F("PASS: UART link confirmed."));
  Serial.println();

  // --- UART microstep control ---
  driver.pdn_disable(true);      // GCONF: keep PDN_UART in UART mode (register bit)
  driver.mstep_reg_select(true); // microsteps come from MRES, not MS1/MS2
  driver.I_scale_analog(false);  // current fully software-controlled
  driver.toff(5);                // chopper on
  driver.rms_current(TEST_RUN_CURRENT_MA);
  driver.intpol(true);

  if (!runMicrostepSweep())
  {
    return;
  }

  driver.microsteps(MOVE_MICROSTEPS);
  driverOk = true;

  // --- Physical move: confirms the UART-configured microstep count is
  // really what the motor executes, not just what the register reports.
  motor.begin();
  motor.enable();
  delay(5); // TMC2209 needs a short settle after ~EN goes low

  const uint32_t stepsForOneTenthRev =
      (200UL * MOVE_MICROSTEPS) / 10; // 200 full steps/rev, 1/10 rev

  Serial.print(F("Moving "));
  Serial.print(stepsForOneTenthRev);
  Serial.print(F(" microsteps forward at 1/"));
  Serial.print(MOVE_MICROSTEPS);
  Serial.println(F(" ..."));

  motor.setDirection(true);
  for (uint32_t i = 0; i < stepsForOneTenthRev; i++)
  {
    motor.step();
    delayMicroseconds(300);
  }

  delay(300);
  motor.setDirection(false);
  for (uint32_t i = 0; i < stepsForOneTenthRev; i++)
  {
    motor.step();
    delayMicroseconds(300);
  }

  motor.disable();
  Serial.println(F("Move complete. If the shaft visibly turned ~1/10 rev"));
  Serial.println(F("each way (smoothly, at 1/64), UART + stepping both work."));
  Serial.println(F("Re-upload to repeat."));
}

void loop()
{
  // One-shot test; everything runs in setup(). Nothing to do here.
}
