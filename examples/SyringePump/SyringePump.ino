/*
  StepScreen SyringePump

  Four-screen pump controller demonstrating the StepScreenNav
  (menu-safe debouncing) and StepScreenStepper (AccelStepper/TMC2209)
  layers.

  Home   -- motor disabled.
            Back = Set, Sel = Dbg (jog), OK = Run (gated on motor 12V).
            Shows syringe_ml, reward_ul, and live VL53L4CD distance (mm).
            Polls TMC ~1 Hz for VM (GSTAT.uv_cp); without 12V shows
            "No 12V" and Confirm reads "XXX" instead of "Run".
  Set    -- edit syringe size (10/30/60 mL) or reward_ul with the encoder.
            Center press toggles which field is active. OK = Save writes
            /SYRING.CFG and shows a brief confirmation. Back discards.
  Dbg    -- motor enabled; encoder jogs the motor in both directions.
            OK cycles the speed preset (Low -> Med -> Fast).
            Back returns Home. Shows position, speed, encoder input,
            and live VL53L4CD distance. USB Serial accepts:
              Steps <n>   relative microsteps (+/-)
              Move F|B|S  continuous forward / backward / stop
              Dist <mm> [mm2]  drive until ToF reaches mm (noop if
                          within 2 mm of current, or ToF missing).
                          Optional second mm loops A↔B until stop.
  Run    -- waits for a rising edge on PIN_AUX_IN (HW pull-down; idle LOW)
            from Bpod (~50 ms pulse). Edge-only: a stuck-HIGH line does not
            re-fire until it returns LOW. Dispenses reward_ul via the 1/64
            µL→steps map. USB Serial still accepts signed steps + newline.
            Back returns Home (stops any move).

  Navigation debouncing (StepScreenNav): button edges are discarded for
  250 ms after each screen change, and a press is only accepted after
  all buttons have been released -- so one press never triggers actions
  on two screens.

  Stepping uses a TC3 ISR (same pattern as Truss) so OLED I2C and ToF
  reads cannot starve the pulse train. loop() only updates the target
  rate (accel/decel trapezoid); the timer keeps stepping through long
  Wire transfers. AccelStepper is used for enable/~EN and speed presets
  only — not for generating STEP pulses.

  ToF: optional Adafruit VL53L4CD on Wire (address 0x29). XSHUT is not
  wired. An I2C probe skips init if absent so boot does not hang; UI
  shows "No ToF" when missing. Hot-unplug is detected; re-plug re-inits
  after a ~1 s probe. Requires stm32duino VL53L4CD when used.

  Board: Adafruit Feather M0 Adalogger (or compatible).

  Motor: Stepperonline 17HE08-1004S. Coil wires to the driver:
    Red = B2, Blue = A2, Green = B1, Black = A1.

  Speed presets (µsteps/s) are raised above the library defaults so Run and
  Dbg feel usable at STEPSCREEN_MOTOR_MICROSTEPS. OK on Dbg still
  cycles Low -> Med -> Fast.

  Config (/SYRING.CFG on microSD), created with defaults if missing:
    syringe_ml=<10|30|60>
    reward_ul=<integer>
  At 1/64 microsteps: 10 mL → 36.5 steps/µL, 30 mL → 17, 60 mL → 9.5.
*/

// Match the µL calibration table (library default is 128).
#ifndef STEPSCREEN_MOTOR_MICROSTEPS
#define STEPSCREEN_MOTOR_MICROSTEPS 64
#endif

// Faster than library defaults (200 / 800 / 2000) — syringe travel felt slow.
#ifndef STEPSCREEN_SPEED_LOW_SPS
#define STEPSCREEN_SPEED_LOW_SPS 1000.0f
#endif
#ifndef STEPSCREEN_SPEED_MEDIUM_SPS
#define STEPSCREEN_SPEED_MEDIUM_SPS 3000.0f
#endif
#ifndef STEPSCREEN_SPEED_FAST_SPS
#define STEPSCREEN_SPEED_FAST_SPS 12000.0f
#endif
#ifndef STEPSCREEN_STEPPER_ACCEL
#define STEPSCREEN_STEPPER_ACCEL 15000.0f
#endif

#include <StepScreen.h>
#include <StepScreenNav.h>
#include <StepScreenRTC.h>
#include <StepScreenSD.h>
#include <StepScreenSplash.h>
#include <StepScreenStepper.h>
#include <StepScreenTMC2209.h>
#include <SD.h>
#include <Wire.h>
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <vl53l4cd_class.h>

StepScreen screen;
StepScreenRTC rtc;
StepScreenNav nav;
StepScreenSD sd;
StepScreenTMC2209 tmc;
StepScreenStepper stepper;

// Microsteps per encoder detent on the Dbg screen (1 full step per detent)
#define JOG_STEPS_PER_DETENT STEPSCREEN_MOTOR_MICROSTEPS

#define SYRING_CFG_PATH "/SYRING.CFG"
#define REWARD_UL_MIN 1
#define REWARD_UL_MAX 5000
#define REWARD_UL_DEFAULT 10
#define AUX_TRIG_DEBOUNCE_MS 3
#define SET_SAVED_FLASH_MS 1200
#define MOTOR_READY_POLL_MS 1000
#define DBG_DIST_TOL_MM 2
#define DBG_CONT_TARGET_STEPS 100000L
#define RX_BUF_SIZE 32

bool motorReady = false;
bool motorConfigured = false;
bool sdReady = false;
uint32_t lastMotorReadyPollMs = 0;
uint32_t setSavedUntilMs = 0;
bool setSaveOk = false;

// AUX_IN reward trigger (idle LOW via board pull-down).
bool auxRawLevel = false;
bool auxStableLevel = false;
uint32_t auxChangeMs = 0;
uint32_t auxLastFireMs = 0;
uint16_t auxFireCount = 0;

// VL53L4CD Time-of-Flight (shares Wire with the OLED and RTC)
#ifndef PIN_TOF_XSHUT
#define PIN_TOF_XSHUT -1
#endif

#define TOF_I2C_ADDR_7BIT 0x29
#define TOF_I2C_ADDR_8BIT 0x52 // VL53L4CD default (7-bit 0x29)
#define TOF_POLL_MS 50
#define TOF_TIMING_BUDGET_MS 200
#define TOF_INTERMEASUREMENT_MS 0 // 0 = back-to-back
#define TOF_OUT_OF_RANGE_MM 1300  // VL53L4CD rated max
#define TOF_I2C_PROBE_TIMEOUT_MS 50
#define TOF_REPROBE_MS 1000
#define TOF_I2C_FAIL_LIMIT 3

VL53L4CD tof(&Wire, PIN_TOF_XSHUT);
bool tofOk = false;
uint16_t lastRangeMm = 0;
bool lastRangeValid = false;
uint32_t lastTofMs = 0;
uint32_t lastTofReprobeMs = 0;
uint8_t tofI2cFailCount = 0;

#define DRAW_INTERVAL_MS 100
#define MOTOR_RAMP_MS 5

enum PumpScreen : uint8_t
{
  SCREEN_HOME,
  SCREEN_SET,
  SCREEN_ADJUST,
  SCREEN_RUN,
};

PumpScreen currentScreen = SCREEN_HOME;

enum SetField : uint8_t
{
  SET_FIELD_ML = 0,
  SET_FIELD_UL,
};

// Syringe size options (mL) and steps per µL at 1/64 microsteps.
static const uint8_t SYRINGE_ML_OPTIONS[] = {10, 30, 60};
static const float SYRINGE_STEPS_PER_UL[] = {36.5f, 17.0f, 9.5f};
static const uint8_t SYRINGE_ML_COUNT =
    sizeof(SYRINGE_ML_OPTIONS) / sizeof(SYRINGE_ML_OPTIONS[0]);

uint8_t syringeMlIndex = 1; // default 30 mL
uint16_t rewardUl = REWARD_UL_DEFAULT;
uint8_t setDraftMlIndex = 1;
uint16_t setDraftRewardUl = REWARD_UL_DEFAULT;
SetField setField = SET_FIELD_ML;

// Run-screen serial command state
enum RunStatus : uint8_t
{
  RUN_WAITING,
  RUN_MOVING,
  RUN_DONE,
  RUN_ERROR,
};

RunStatus runStatus = RUN_WAITING;
char rxBuf[RX_BUF_SIZE];
uint8_t rxLen = 0;
int32_t lastCommandSteps = 0;
int32_t lastEncoderDelta = 0;

// Dbg serial motion: continuous F/B and ToF Dist closed-loop.
int8_t dbgRunDir = 0; // +1 forward, -1 backward, 0 not continuous
bool dbgDistActive = false;
bool dbgDistLoop = false; // Dist A B: oscillate until Move S / jog / Back
uint16_t dbgDistTargetMm = 0;
uint16_t dbgDistAMm = 0;
uint16_t dbgDistBMm = 0;
int8_t dbgDistDir = 0; // sign of travel toward current Dist target

// ISR step generation (Truss-style). Position/target are volatile; the
// main loop only changes the commanded rate — TC3 keeps pulsing.
volatile int32_t motorPosition = 0;
volatile int32_t motorTarget = 0;
volatile int8_t motorDirSign = 1;
volatile bool motorTickActive = false;
volatile uint32_t stepPeriodTicks = 0;
float targetSpsCmd = 0.0f;
float currentSps = 0.0f;
uint32_t lastRampMs = 0;

#if defined(ARDUINO_ARCH_SAMD)
#define MOTOR_GCLK_HZ 48000000UL
#define MOTOR_TIMER_PRESCALE 2
#define MOTOR_TIMER_HZ (MOTOR_GCLK_HZ / MOTOR_TIMER_PRESCALE)
#define MOTOR_PERIOD_MAX 65536u
#define STEP_PULSE_US 2
#define MOTOR_PERIOD_UPDATE_PCT 2u

void motorTimerStop()
{
  TC3->COUNT16.CTRLA.bit.ENABLE = 0;
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY)
  {
  }
  TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;
}

void motorTimerApplyPeriod(uint32_t period)
{
  noInterrupts();
  stepPeriodTicks = period;

  if (!TC3->COUNT16.CTRLA.bit.ENABLE)
  {
    TC3->COUNT16.CC[0].reg = (uint16_t)(period - 1);
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY)
    {
    }
    TC3->COUNT16.COUNT.reg = 0;
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY)
    {
    }
    TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;
    TC3->COUNT16.CTRLA.bit.ENABLE = 1;
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY)
    {
    }
    interrupts();
    return;
  }

  TC3->COUNT16.CC[0].reg = (uint16_t)(period - 1);
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY)
  {
  }
  TC3->COUNT16.READREQ.reg = TC_READREQ_RREQ | TC_READREQ_ADDR(0x10);
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY)
  {
  }
  if (TC3->COUNT16.COUNT.reg >= (uint16_t)(period - 1))
  {
    TC3->COUNT16.CTRLBSET.reg = TC_CTRLBSET_CMD_RETRIGGER;
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY)
    {
    }
  }
  interrupts();
}

void initMotorTimer()
{
  GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 |
                                 GCLK_CLKCTRL_ID_TCC2_TC3);
  while (GCLK->STATUS.bit.SYNCBUSY)
  {
  }

  TC3->COUNT16.CTRLA.bit.ENABLE = 0;
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY)
  {
  }

  TC3->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_WAVEGEN_MFRQ |
                           TC_CTRLA_PRESCALER_DIV2;
  TC3->COUNT16.INTENSET.reg = TC_INTENSET_MC0;
  TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;
  NVIC_EnableIRQ(TC3_IRQn);
}

void isrStepPulse()
{
  digitalWrite(PIN_MOTOR_STEP, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(PIN_MOTOR_STEP, LOW);
}

void TC3_Handler()
{
  if (TC3->COUNT16.INTFLAG.bit.MC0 == 0)
  {
    return;
  }
  TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;
  if (!motorTickActive || stepPeriodTicks == 0)
  {
    return;
  }
  if (motorPosition == motorTarget)
  {
    return;
  }
  isrStepPulse();
  motorPosition += motorDirSign;
}
#endif

void applyStepRate(float sps, bool force = false);
void setMotorTickActive(bool active);
void motorStopHard();
bool motorIsRunning();
int32_t motorCurrentPosition();
void motorMoveRelative(int32_t steps);
void serviceMotor();
void dbgClearSerialMotion();
void dbgEnsureMotorLive();
void dbgSetContinuous(int8_t dir);
void dbgServiceSerialMotion();
void printDbgSerialHelp();
void handleDbgSerialCommand();

uint8_t syringeMl() { return SYRINGE_ML_OPTIONS[syringeMlIndex]; }

float stepsPerUl() { return SYRINGE_STEPS_PER_UL[syringeMlIndex]; }

int32_t rewardSteps() {
  return (int32_t)lroundf((float)rewardUl * stepsPerUl());
}

int8_t indexForSyringeMl(uint8_t ml) {
  for (uint8_t i = 0; i < SYRINGE_ML_COUNT; i++) {
    if (SYRINGE_ML_OPTIONS[i] == ml) {
      return (int8_t)i;
    }
  }
  return -1;
}

bool waitForSdReady(uint32_t timeoutMs) {
  const uint32_t deadline = millis() + timeoutMs;
  while ((int32_t)(millis() - deadline) < 0) {
    sd.update();
    if (sd.isReady()) {
      return true;
    }
    delay(50);
  }
  return sd.isReady();
}

bool saveSyringeConfig() {
  if (!sd.isReady()) {
    Serial.println(F("[Cfg] SD not ready — RAM only"));
    return false;
  }

  char body[64];
  const int n =
      snprintf(body, sizeof(body), "syringe_ml=%u\nreward_ul=%u\n",
               (unsigned)syringeMl(), (unsigned)rewardUl);
  if (n <= 0 || (size_t)n >= sizeof(body)) {
    return false;
  }

  const bool ok =
      sd.writeFile(SYRING_CFG_PATH, (const uint8_t *)body, (size_t)n, false);
  Serial.print(F("[Cfg] save "));
  Serial.print(SYRING_CFG_PATH);
  Serial.println(ok ? F(" ok") : F(" failed"));
  return ok;
}

bool parseSyringeConfigLine(const char *text, uint8_t *ml, uint16_t *ul,
                            bool *gotMl, bool *gotUl) {
  if (text == nullptr || text[0] == '\0') {
    return false;
  }
  unsigned v = 0;
  if (sscanf(text, "syringe_ml=%u", &v) == 1) {
    *ml = (uint8_t)v;
    *gotMl = true;
    return true;
  }
  if (sscanf(text, "reward_ul=%u", &v) == 1) {
    if (v < REWARD_UL_MIN) {
      v = REWARD_UL_MIN;
    } else if (v > REWARD_UL_MAX) {
      v = REWARD_UL_MAX;
    }
    *ul = (uint16_t)v;
    *gotUl = true;
    return true;
  }
  return false;
}

bool loadSyringeConfig() {
  if (!sd.isReady()) {
    return false;
  }

  File f = SD.open(SYRING_CFG_PATH, FILE_READ);
  if (!f) {
    return false;
  }

  uint8_t ml = syringeMl();
  uint16_t ul = rewardUl;
  bool gotMl = false;
  bool gotUl = false;
  char line[40];
  uint8_t len = 0;

  while (f.available()) {
    const char c = (char)f.read();
    if (c == '\r') {
      continue;
    }
    if (c == '\n' || len >= sizeof(line) - 1) {
      line[len] = '\0';
      parseSyringeConfigLine(line, &ml, &ul, &gotMl, &gotUl);
      len = 0;
      continue;
    }
    line[len++] = c;
  }
  if (len > 0) {
    line[len] = '\0';
    parseSyringeConfigLine(line, &ml, &ul, &gotMl, &gotUl);
  }
  f.close();

  if (!gotMl && !gotUl) {
    Serial.println(F("[Cfg] parse failed"));
    return false;
  }

  const int8_t idx = indexForSyringeMl(ml);
  if (idx >= 0) {
    syringeMlIndex = (uint8_t)idx;
  } else {
    Serial.print(F("[Cfg] unknown syringe_ml="));
    Serial.print(ml);
    Serial.println(F(" — keeping default"));
  }
  if (gotUl) {
    rewardUl = ul;
  }

  Serial.print(F("[Cfg] loaded syringe_ml="));
  Serial.print(syringeMl());
  Serial.print(F(" reward_ul="));
  Serial.println(rewardUl);
  return true;
}

void ensureSyringeConfig() {
  if (!sdReady) {
    return;
  }
  if (loadSyringeConfig()) {
    return;
  }
  Serial.println(F("[Cfg] creating defaults"));
  syringeMlIndex = 1;
  rewardUl = REWARD_UL_DEFAULT;
  saveSyringeConfig();
}

void pollSd() {
  sd.update();
  const bool ready = sd.isReady();
  if (ready == sdReady) {
    return;
  }
  sdReady = ready;
  Serial.print(F("[SD] "));
  Serial.println(sdReady ? F("ready") : F("not ready"));
  if (sdReady) {
    ensureSyringeConfig();
  }
}

// Re-check TMC UART + motor supply (GSTAT.uv_cp). Lets Home recover when
// the user plugs in 12V after booting from USB alone.
void refreshMotorReady() {
  const bool prev = motorReady;

  if (!tmc.isReady() && !tmc.verify()) {
    motorReady = false;
    motorConfigured = false;
    if (prev) {
      Serial.println(F("[TMC] UART not ready"));
    }
    return;
  }

  TMC2209Stepper &drv = tmc.driver();
  // GSTAT bits are sticky (W1C). Clear after read so late 12V / power
  // restore can recover, and so reset does not force reconfigure forever.
  const bool sawReset = drv.reset();
  const bool sawUv = drv.uv_cp();
  if (sawReset || sawUv) {
    drv.GSTAT(0b111);
  }
  if (sawReset) {
    Serial.println(F("[TMC] chip reset — will reconfigure"));
    motorConfigured = false;
  }
  if (sawUv && drv.uv_cp()) {
    motorReady = false;
    if (prev) {
      Serial.println(F("[TMC] no motor supply (uv_cp)"));
    }
    return;
  }

  if (!motorConfigured) {
    if (!tmc.configureForStepDir()) {
      motorReady = false;
      if (prev) {
        Serial.println(F("[TMC] configure failed"));
      }
      return;
    }
    motorConfigured = true;
  }

  motorReady = true;
  if (!prev) {
    Serial.println(F("[TMC] motor ready (12V ok)"));
  }
}

void pollMotorReady() {
  if (currentScreen != SCREEN_HOME) {
    return;
  }
  const uint32_t now = millis();
  if ((uint32_t)(now - lastMotorReadyPollMs) < MOTOR_READY_POLL_MS) {
    return;
  }
  lastMotorReadyPollMs = now;
  refreshMotorReady();
}

void setMotorTickActive(bool active)
{
  motorTickActive = active;
#if defined(ARDUINO_ARCH_SAMD)
  noInterrupts();
  motorTimerStop();
  if (!active)
  {
    stepPeriodTicks = 0;
  }
  interrupts();
#endif
}

void applyStepRate(float sps, bool force)
{
  if (!motorReady || !motorTickActive)
  {
    setMotorTickActive(false);
    return;
  }

  const float absSps = fabsf(sps);

#if defined(ARDUINO_ARCH_SAMD)
  if (absSps >= 1.0f)
  {
    motorDirSign = (sps >= 0.0f) ? 1 : -1;
    digitalWrite(PIN_MOTOR_DIR, (sps >= 0.0f) ? HIGH : LOW);
  }

  if (absSps < 1.0f)
  {
    noInterrupts();
    motorTimerStop();
    stepPeriodTicks = 0;
    interrupts();
    return;
  }

  uint32_t period = (uint32_t)(MOTOR_TIMER_HZ / absSps);
  if (period < 2)
  {
    period = 2;
  }
  else if (period > MOTOR_PERIOD_MAX)
  {
    period = MOTOR_PERIOD_MAX;
  }

  if (!force && stepPeriodTicks > 0 && TC3->COUNT16.CTRLA.bit.ENABLE)
  {
    if (period == stepPeriodTicks)
    {
      return;
    }
    const uint32_t diff =
        (period > stepPeriodTicks) ? (period - stepPeriodTicks)
                                   : (stepPeriodTicks - period);
    if (diff * 100u < stepPeriodTicks * MOTOR_PERIOD_UPDATE_PCT)
    {
      return;
    }
  }

  motorTimerApplyPeriod(period);
#else
  // Non-SAMD: fall back to AccelStepper constant-speed polling.
  if (absSps >= 1.0f)
  {
    stepper.setSpeed(sps);
  }
  else
  {
    stepper.setSpeed(0.0f);
  }
#endif
}

bool motorIsRunning()
{
  noInterrupts();
  const int32_t pos = motorPosition;
  const int32_t tgt = motorTarget;
  interrupts();
  return pos != tgt || currentSps != 0.0f || targetSpsCmd != 0.0f;
}

int32_t motorCurrentPosition()
{
  noInterrupts();
  const int32_t pos = motorPosition;
  interrupts();
  return pos;
}

void motorStopHard()
{
  targetSpsCmd = 0.0f;
  currentSps = 0.0f;
  noInterrupts();
  motorTarget = motorPosition;
  interrupts();
  applyStepRate(0.0f, true);
}

void motorMoveRelative(int32_t steps)
{
  if (steps == 0)
  {
    return;
  }
  noInterrupts();
  motorTarget += steps;
  interrupts();
}

// Trapezoid: accel to preset max, cruise, decelerate into the target.
// Only the rate is computed here; TC3 keeps issuing steps while we block.
void serviceMotor()
{
  if (!motorReady || !motorTickActive)
  {
    return;
  }

  noInterrupts();
  const int32_t pos = motorPosition;
  const int32_t tgt = motorTarget;
  interrupts();
  const int32_t dist = tgt - pos;

  // Continuous Move/Dist: cruise at the current speed preset (no target
  // trapezoid). Target is only kept ahead so the ISR keeps pulsing.
  if (dbgRunDir != 0)
  {
    targetSpsCmd = (float)dbgRunDir * stepper.speedPresetSps();
  }
  else if (dist == 0)
  {
    // Arrived: drop the pulse train immediately (ISR already stopped
    // advancing position when pos == target).
    targetSpsCmd = 0.0f;
    currentSps = 0.0f;
    applyStepRate(0.0f, true);
    return;
  }
  else
  {
    const float maxSps = stepper.speedPresetSps();
    const float accel = STEPSCREEN_STEPPER_ACCEL;
    const float remaining = (float)((dist > 0) ? dist : -dist);
    const float absV = fabsf(currentSps);
    const float decelSteps =
        (accel > 0.0f) ? (absV * absV) / (2.0f * accel) : remaining;
    float desired = maxSps;
    if (remaining <= decelSteps)
    {
      desired = sqrtf(2.0f * accel * remaining);
      if (desired > maxSps)
      {
        desired = maxSps;
      }
    }
    // Keep a small floor so the last few steps still arrive.
    if (desired < 1.0f && remaining >= 1.0f)
    {
      desired = 1.0f;
    }
    targetSpsCmd = (dist > 0) ? desired : -desired;
  }

  const uint32_t now = millis();
  const uint32_t dtMs = now - lastRampMs;
  if (dtMs < MOTOR_RAMP_MS)
  {
    return;
  }
  lastRampMs = now;

  if (currentSps != targetSpsCmd)
  {
    const float step = STEPSCREEN_STEPPER_ACCEL * (float)dtMs / 1000.0f;
    if (targetSpsCmd > currentSps)
    {
      currentSps += step;
      if (currentSps > targetSpsCmd)
      {
        currentSps = targetSpsCmd;
      }
    }
    else
    {
      currentSps -= step;
      if (currentSps < targetSpsCmd)
      {
        currentSps = targetSpsCmd;
      }
    }
  }

  applyStepRate(currentSps);

#if !defined(ARDUINO_ARCH_SAMD)
  // Host / non-SAMD: AccelStepper polls one step per call (no TC3).
  if (fabsf(currentSps) >= 1.0f)
  {
    if (stepper.runSpeed())
    {
      noInterrupts();
      motorPosition += motorDirSign;
      if (motorPosition == motorTarget)
      {
        interrupts();
        currentSps = 0.0f;
        targetSpsCmd = 0.0f;
        applyStepRate(0.0f, true);
      }
      else
      {
        interrupts();
      }
    }
  }
#endif
}

// --- Screen transitions (always via these, so nav debounce applies) ---

void enterHome()
{
  currentScreen = SCREEN_HOME;
  dbgClearSerialMotion();
  motorStopHard();
  setMotorTickActive(false);
  stepper.disable();
  nav.enterScreen();
  Serial.println(F("[Home]"));
}

void enterSet()
{
  currentScreen = SCREEN_SET;
  setDraftMlIndex = syringeMlIndex;
  setDraftRewardUl = rewardUl;
  setField = SET_FIELD_ML;
  setSavedUntilMs = 0;
  setSaveOk = false;
  nav.enterScreen();
  Serial.print(F("[Set] syringe_ml="));
  Serial.print(syringeMl());
  Serial.print(F(" reward_ul="));
  Serial.println(rewardUl);
}

void enterAdjust()
{
  currentScreen = SCREEN_ADJUST;
  lastEncoderDelta = 0;
  dbgClearSerialMotion();
  rxLen = 0;
  while (Serial.available())
  {
    Serial.read();
  }
  if (motorReady)
  {
    stepper.enable();
    setMotorTickActive(true);
    lastRampMs = millis();
  }
  nav.enterScreen();
  printDbgSerialHelp();
}

bool startRewardMove(int32_t steps, const __FlashStringHelper *source)
{
  if (steps == 0)
  {
    Serial.println(F("[Run] skip 0 steps"));
    return false;
  }
  if (motorIsRunning())
  {
    Serial.println(F("[Run] busy"));
    return false;
  }
  if (!motorReady)
  {
    runStatus = RUN_ERROR;
    Serial.println(F("[Run] motor not ready"));
    return false;
  }

  lastCommandSteps = steps;
  stepper.enable();
  setMotorTickActive(true);
  lastRampMs = millis();
  motorMoveRelative(steps);
  runStatus = RUN_MOVING;
  Serial.print(F("[Run] "));
  Serial.print(source);
  Serial.print(F(" moving "));
  Serial.println(steps);
  return true;
}

void fireAuxReward()
{
  auxFireCount++;
  auxLastFireMs = millis();
  const int32_t steps = rewardSteps();
  Serial.print(F("[Run] AUX rise #"));
  Serial.print(auxFireCount);
  Serial.print(F(" reward_ul="));
  Serial.print(rewardUl);
  Serial.print(F(" -> "));
  Serial.print(steps);
  Serial.println(F(" steps"));
  startRewardMove(steps, F("aux"));
}

void beginAuxTrigger()
{
  pinMode(PIN_AUX_IN, INPUT); // board has HW pull-down; idle LOW
  auxRawLevel = digitalRead(PIN_AUX_IN) == HIGH;
  auxStableLevel = auxRawLevel;
  auxChangeMs = millis();
  // If already HIGH on entry, wait for LOW before the next rise counts.
  Serial.print(F("[Run] AUX_IN pin "));
  Serial.print(PIN_AUX_IN);
  Serial.print(F(" level="));
  Serial.println(auxRawLevel ? 'H' : 'L');
}

void pollAuxTrigger()
{
  const bool raw = digitalRead(PIN_AUX_IN) == HIGH;
  const uint32_t now = millis();
  if (raw != auxRawLevel)
  {
    auxRawLevel = raw;
    auxChangeMs = now;
    return;
  }
  if ((uint32_t)(now - auxChangeMs) < AUX_TRIG_DEBOUNCE_MS)
  {
    return;
  }
  if (raw == auxStableLevel)
  {
    return;
  }
  auxStableLevel = raw;
  if (auxStableLevel)
  {
    // Debounced rising edge only — stuck HIGH cannot re-fire.
    fireAuxReward();
  }
}

void enterRun()
{
  currentScreen = SCREEN_RUN;
  motorStopHard();
  setMotorTickActive(false);
  stepper.disable();
  runStatus = RUN_WAITING;
  rxLen = 0;
  auxFireCount = 0;
  while (Serial.available())
  {
    Serial.read(); // flush stale input
  }
  beginAuxTrigger();
  nav.enterScreen();
  Serial.print(F("[Run] AUX rise = reward ("));
  Serial.print(rewardUl);
  Serial.print(F(" uL / "));
  Serial.print(rewardSteps());
  Serial.println(F(" steps); Serial steps+\\n still works"));
}

// --- Serial line reader (shared rxBuf; Run vs Dbg parsers differ) ---

bool readSerialLine()
{
  while (Serial.available())
  {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r')
    {
      if (rxLen == 0)
      {
        continue;
      }
      rxBuf[rxLen] = '\0';
      rxLen = 0;
      return true;
    }
    if (rxLen < sizeof(rxBuf) - 1)
    {
      rxBuf[rxLen++] = c;
    }
    else
    {
      rxLen = 0;
      Serial.println(F("error: command too long"));
    }
  }
  return false;
}

bool cmdTokenIs(const char *s, const char *word)
{
  size_t i = 0;
  while (word[i] != '\0')
  {
    if (s[i] == '\0' ||
        tolower((unsigned char)s[i]) != tolower((unsigned char)word[i]))
    {
      return false;
    }
    i++;
  }
  return s[i] == '\0' || s[i] == ' ' || s[i] == '\t';
}

void printDbgSerialHelp()
{
  Serial.println(F("[Dbg] serial commands:"));
  Serial.println(F("  Steps <n>   relative microsteps (+/-)"));
  Serial.println(F("  Move F|B|S  forward / backward / stop"));
  Serial.println(F("  Dist <mm>       drive until ToF reaches mm"));
  Serial.println(F("  Dist <mm> <mm2> loop between those ToF distances"));
  Serial.println(F("             (noop if within 2mm or no ToF)"));
  Serial.println(F("encoder jogs; OK cycles speed"));
}

void dbgClearSerialMotion()
{
  dbgRunDir = 0;
  dbgDistActive = false;
  dbgDistLoop = false;
  dbgDistDir = 0;
}

bool dbgDistNear(uint16_t mm)
{
  if (!lastRangeValid)
  {
    return false;
  }
  const int16_t err = (int16_t)mm - (int16_t)lastRangeMm;
  return err <= DBG_DIST_TOL_MM && err >= -DBG_DIST_TOL_MM;
}

// Aim continuous travel at a ToF target (positive steps raise ToF).
void dbgAimDistTarget(uint16_t mm)
{
  dbgDistTargetMm = mm;
  const int16_t err = (int16_t)mm - (int16_t)lastRangeMm;
  dbgDistDir = (err >= 0) ? 1 : -1;
  dbgDistActive = true;
  dbgSetContinuous(dbgDistDir);
}

void dbgEnsureMotorLive()
{
  if (!motorReady)
  {
    return;
  }
  stepper.enable();
  // Only reset the ramp clock when starting from a dead stop — refreshing
  // it every Move/Dist service tick starved accel and crawled at ~0 sps.
  if (!motorTickActive)
  {
    setMotorTickActive(true);
    lastRampMs = millis();
  }
}

// Keep ISR target ahead so continuous F/B does not decelerate into a stop.
void dbgSetContinuous(int8_t dir)
{
  dbgRunDir = dir;
  if (dir == 0)
  {
    return;
  }
  dbgEnsureMotorLive();
  noInterrupts();
  motorTarget = motorPosition + (int32_t)dir * DBG_CONT_TARGET_STEPS;
  interrupts();
}

void dbgServiceSerialMotion()
{
  if (dbgDistActive)
  {
    if (!tofOk || !lastRangeValid)
    {
      motorStopHard();
      dbgClearSerialMotion();
      Serial.println(F("[Dbg] Dist aborted — ToF lost"));
      return;
    }
    const int16_t err =
        (int16_t)dbgDistTargetMm - (int16_t)lastRangeMm;
    // Stop in the ±2 mm band, or once we have reached/passed the target
    // along the original travel direction (avoids overshoot hunting).
    const bool arrived =
        (err <= DBG_DIST_TOL_MM && err >= -DBG_DIST_TOL_MM) ||
        (dbgDistDir > 0 && err <= DBG_DIST_TOL_MM) ||
        (dbgDistDir < 0 && err >= -DBG_DIST_TOL_MM);
    if (arrived)
    {
      if (dbgDistLoop)
      {
        const uint16_t nextMm =
            (dbgDistTargetMm == dbgDistAMm) ? dbgDistBMm : dbgDistAMm;
        Serial.print(F("[Dbg] Dist cycle ToF="));
        Serial.print(lastRangeMm);
        Serial.print(F(" -> "));
        Serial.println(nextMm);
        dbgAimDistTarget(nextMm);
        return;
      }
      motorStopHard();
      dbgClearSerialMotion();
      Serial.print(F("[Dbg] Dist done ToF="));
      Serial.print(lastRangeMm);
      Serial.println(F(" mm"));
      return;
    }
    dbgSetContinuous(dbgDistDir);
    return;
  }

  if (dbgRunDir != 0)
  {
    dbgSetContinuous(dbgRunDir);
  }
}

void handleDbgSerialCommand()
{
  while (readSerialLine())
  {
    // Skip leading spaces.
    char *p = rxBuf;
    while (*p == ' ' || *p == '\t')
    {
      p++;
    }

    if (cmdTokenIs(p, "Steps"))
    {
      p += 5;
      while (*p == ' ' || *p == '\t')
      {
        p++;
      }
      char *end = nullptr;
      const long steps = strtol(p, &end, 10);
      if (end == p || *end != '\0')
      {
        Serial.print(F("error: Steps needs an integer: "));
        Serial.println(rxBuf);
        continue;
      }
      if (!motorReady)
      {
        Serial.println(F("[Dbg] motor not ready"));
        continue;
      }
      dbgClearSerialMotion();
      dbgEnsureMotorLive();
      motorMoveRelative((int32_t)steps);
      Serial.print(F("[Dbg] Steps "));
      Serial.println((int32_t)steps);
      continue;
    }

    if (cmdTokenIs(p, "Move"))
    {
      p += 4;
      while (*p == ' ' || *p == '\t')
      {
        p++;
      }
      const char cmd = (char)toupper((unsigned char)p[0]);
      if ((cmd != 'F' && cmd != 'B' && cmd != 'S') || p[1] != '\0')
      {
        Serial.println(F("error: Move F|B|S"));
        continue;
      }
      if (cmd == 'S')
      {
        motorStopHard();
        dbgClearSerialMotion();
        Serial.println(F("[Dbg] Move S"));
        continue;
      }
      if (!motorReady)
      {
        Serial.println(F("[Dbg] motor not ready"));
        continue;
      }
      dbgClearSerialMotion();
      dbgSetContinuous(cmd == 'F' ? 1 : -1);
      Serial.print(F("[Dbg] Move "));
      Serial.println(cmd);
      continue;
    }

    if (cmdTokenIs(p, "Dist"))
    {
      p += 4;
      while (*p == ' ' || *p == '\t')
      {
        p++;
      }
      char *end = nullptr;
      const long mm = strtol(p, &end, 10);
      if (end == p || mm < 0 || mm > TOF_OUT_OF_RANGE_MM)
      {
        Serial.print(F("error: Dist <mm> [mm2] (0.."));
        Serial.print(TOF_OUT_OF_RANGE_MM);
        Serial.println(F(")"));
        continue;
      }
      while (*end == ' ' || *end == '\t')
      {
        end++;
      }
      bool loop = false;
      long mm2 = 0;
      if (*end != '\0')
      {
        char *end2 = nullptr;
        mm2 = strtol(end, &end2, 10);
        if (end2 == end || *end2 != '\0' || mm2 < 0 ||
            mm2 > TOF_OUT_OF_RANGE_MM)
        {
          Serial.print(F("error: Dist <mm> [mm2] (0.."));
          Serial.print(TOF_OUT_OF_RANGE_MM);
          Serial.println(F(")"));
          continue;
        }
        loop = true;
      }
      if (!tofOk || !lastRangeValid)
      {
        Serial.println(F("[Dbg] Dist ignored — ToF not available"));
        continue;
      }
      if (!motorReady)
      {
        Serial.println(F("[Dbg] motor not ready"));
        continue;
      }

      if (loop)
      {
        const int16_t span = (int16_t)mm - (int16_t)mm2;
        if (span <= DBG_DIST_TOL_MM && span >= -DBG_DIST_TOL_MM)
        {
          Serial.println(F("[Dbg] Dist loop ignored — ends within 2 mm"));
          continue;
        }
        dbgDistAMm = (uint16_t)mm;
        dbgDistBMm = (uint16_t)mm2;
        dbgDistLoop = true;
        // Already at A → head to B first so the cycle starts immediately.
        const uint16_t firstMm =
            dbgDistNear(dbgDistAMm) ? dbgDistBMm : dbgDistAMm;
        dbgAimDistTarget(firstMm);
        Serial.print(F("[Dbg] Dist loop "));
        Serial.print(dbgDistAMm);
        Serial.print(F(" <-> "));
        Serial.print(dbgDistBMm);
        Serial.print(F(" mm (first "));
        Serial.print(firstMm);
        Serial.print(F(", from "));
        Serial.print(lastRangeMm);
        Serial.println(F(")"));
        continue;
      }

      if (dbgDistNear((uint16_t)mm))
      {
        Serial.print(F("[Dbg] Dist ignored — within "));
        Serial.print(DBG_DIST_TOL_MM);
        Serial.print(F(" mm (ToF="));
        Serial.print(lastRangeMm);
        Serial.println(F(")"));
        continue;
      }
      dbgDistLoop = false;
      dbgAimDistTarget((uint16_t)mm);
      Serial.print(F("[Dbg] Dist "));
      Serial.print(dbgDistTargetMm);
      Serial.print(F(" mm (from "));
      Serial.print(lastRangeMm);
      Serial.println(F(")"));
      continue;
    }

    Serial.print(F("error: unknown command: "));
    Serial.println(rxBuf);
  }

  dbgServiceSerialMotion();
}

// --- Run-screen serial protocol ---

void handleSerialCommand()
{
  while (readSerialLine())
  {
    char *end = nullptr;
    const long steps = strtol(rxBuf, &end, 10);
    if (end == rxBuf || *end != '\0')
    {
      runStatus = RUN_ERROR;
      Serial.print(F("error: not a number: "));
      Serial.println(rxBuf);
      continue;
    }

    startRewardMove((int32_t)steps, F("serial"));
  }

  // Move finished: release the motor and report
  if (runStatus == RUN_MOVING && !motorIsRunning())
  {
    motorStopHard();
    setMotorTickActive(false);
    stepper.disable();
    runStatus = RUN_DONE;
    Serial.print(F("done, position "));
    Serial.println(motorCurrentPosition());
  }
}

// --- Per-screen input handling ---

void handleHome(uint8_t ev)
{
  if (ev & STEPSCREEN_EVT_BACK)
  {
    enterSet();
  }
  else if (ev & STEPSCREEN_EVT_PUSH)
  {
    enterAdjust();
  }
  else if (ev & STEPSCREEN_EVT_CONFIRM)
  {
    if (!motorReady)
    {
      Serial.println(F("[Home] Run blocked — need 12V"));
      return;
    }
    enterRun();
  }
}

void handleSet(uint8_t ev, int32_t enc)
{
  if (setSavedUntilMs != 0)
  {
    if ((int32_t)(millis() - setSavedUntilMs) >= 0)
    {
      setSavedUntilMs = 0;
      enterHome();
    }
    return;
  }

  if (ev & STEPSCREEN_EVT_BACK)
  {
    Serial.println(F("[Set] discarded"));
    enterHome();
    return;
  }
  if (ev & STEPSCREEN_EVT_CONFIRM)
  {
    syringeMlIndex = setDraftMlIndex;
    rewardUl = setDraftRewardUl;
    setSaveOk = saveSyringeConfig();
    setSavedUntilMs = millis() + SET_SAVED_FLASH_MS;
    Serial.print(F("[Set] saved syringe_ml="));
    Serial.print(syringeMl());
    Serial.print(F(" reward_ul="));
    Serial.print(rewardUl);
    Serial.print(F(" steps="));
    Serial.print(rewardSteps());
    Serial.println(setSaveOk ? F(" (SD ok)") : F(" (RAM only)"));
    return;
  }
  if (ev & STEPSCREEN_EVT_PUSH)
  {
    setField = (setField == SET_FIELD_ML) ? SET_FIELD_UL : SET_FIELD_ML;
    return;
  }
  if (enc == 0)
  {
    return;
  }
  if (setField == SET_FIELD_ML)
  {
    int16_t next = (int16_t)setDraftMlIndex + (int16_t)enc;
    while (next < 0)
    {
      next += SYRINGE_ML_COUNT;
    }
    setDraftMlIndex = (uint8_t)(next % SYRINGE_ML_COUNT);
  }
  else
  {
    int32_t next = (int32_t)setDraftRewardUl + enc;
    if (next < REWARD_UL_MIN)
    {
      next = REWARD_UL_MIN;
    }
    else if (next > REWARD_UL_MAX)
    {
      next = REWARD_UL_MAX;
    }
    setDraftRewardUl = (uint16_t)next;
  }
}

void handleAdjust(uint8_t ev, int32_t enc)
{
  if (ev & STEPSCREEN_EVT_BACK)
  {
    enterHome();
    return;
  }
  if (ev & STEPSCREEN_EVT_CONFIRM)
  {
    stepper.nextSpeedPreset();
    Serial.print(F("speed: "));
    Serial.println(stepper.speedPresetName());
  }
  if (enc != 0 && motorReady)
  {
    // Encoder jog cancels continuous / Dist closed-loop.
    dbgClearSerialMotion();
    lastEncoderDelta = enc;
    dbgEnsureMotorLive();
    motorMoveRelative(enc * JOG_STEPS_PER_DETENT);
  }
  handleDbgSerialCommand();
}

void handleRun(uint8_t ev)
{
  if (ev & STEPSCREEN_EVT_BACK)
  {
    enterHome();
    return;
  }
  pollAuxTrigger();
  handleSerialCommand();
}

// --- VL53L4CD ranging (Wire already started by the OLED) ---

bool i2cProbe(uint8_t addr7)
{
  Wire.beginTransmission(addr7);
  return Wire.endTransmission() == 0;
}

void markTofLost(const __FlashStringHelper *why)
{
  if (tofOk)
  {
    Serial.print(F("ToF: lost — "));
    Serial.println(why);
  }
  tofOk = false;
  lastRangeValid = false;
  lastRangeMm = 0;
  tofI2cFailCount = 0;
  lastTofReprobeMs = millis();
}

bool initTof()
{
  // Probe first: InitSensor can block for a long time with no ACK.
  Wire.setClock(100000);
#if defined(WIRE_HAS_TIMEOUT) || defined(ARDUINO_ARCH_SAMD)
  Wire.setTimeout(TOF_I2C_PROBE_TIMEOUT_MS);
#endif
  if (!i2cProbe(TOF_I2C_ADDR_7BIT))
  {
    Serial.println(F("ToF: no device at 0x29 — continuing without distance"));
    return false;
  }

  Serial.println(F("ToF: starting VL53L4CD init..."));
  tof.begin();
  tof.VL53L4CD_Off();

  const VL53L4CD_ERROR status = tof.InitSensor(TOF_I2C_ADDR_8BIT);
  if (status != VL53L4CD_ERROR_NONE)
  {
    Serial.print(F("ToF: InitSensor failed status="));
    Serial.println(status);
    return false;
  }

  if (tof.VL53L4CD_SetRangeTiming(TOF_TIMING_BUDGET_MS, TOF_INTERMEASUREMENT_MS) !=
      VL53L4CD_ERROR_NONE)
  {
    Serial.println(F("ToF: SetRangeTiming failed"));
    return false;
  }

  // Default min-count threshold (~1000 kcps) rejects weak returns and
  // reports them as status 2 / distance 0.
  if (tof.VL53L4CD_SetSignalThreshold(0) != VL53L4CD_ERROR_NONE)
  {
    Serial.println(F("ToF: SetSignalThreshold failed"));
    return false;
  }

  if (tof.VL53L4CD_StartRanging() != VL53L4CD_ERROR_NONE)
  {
    Serial.println(F("ToF: StartRanging failed"));
    return false;
  }

  tofI2cFailCount = 0;
  lastRangeValid = false;
  Serial.println(F("SyringePump: VL53L4CD ready @ 0x29"));
  return true;
}

void pollTof()
{
  const uint32_t now = millis();

  // Absent / previously lost: re-probe ~1 Hz and re-init on plug-in.
  if (!tofOk)
  {
    if ((uint32_t)(now - lastTofReprobeMs) < TOF_REPROBE_MS)
    {
      return;
    }
    lastTofReprobeMs = now;
    Wire.setClock(100000);
#if defined(WIRE_HAS_TIMEOUT) || defined(ARDUINO_ARCH_SAMD)
    Wire.setTimeout(TOF_I2C_PROBE_TIMEOUT_MS);
#endif
    if (!i2cProbe(TOF_I2C_ADDR_7BIT))
    {
      return;
    }
    Serial.println(F("ToF: device detected — re-init"));
    tofOk = initTof();
    if (!tofOk)
    {
      Serial.println(F("ToF: re-init failed — will retry"));
    }
    return;
  }

  if ((uint32_t)(now - lastTofMs) < TOF_POLL_MS)
  {
    return;
  }
  lastTofMs = now;

  // Periodic presence check so unplug is noticed even if API stays quiet.
  if ((uint32_t)(now - lastTofReprobeMs) >= TOF_REPROBE_MS)
  {
    lastTofReprobeMs = now;
    if (!i2cProbe(TOF_I2C_ADDR_7BIT))
    {
      markTofLost(F("unplugged"));
      return;
    }
  }

  uint8_t ready = 0;
  if (tof.VL53L4CD_CheckForDataReady(&ready) != VL53L4CD_ERROR_NONE)
  {
    if (++tofI2cFailCount >= TOF_I2C_FAIL_LIMIT)
    {
      markTofLost(F("I2C error"));
    }
    return;
  }
  if (!ready)
  {
    return;
  }

  tof.VL53L4CD_ClearInterrupt();

  VL53L4CD_Result_t result;
  if (tof.VL53L4CD_GetResult(&result) != VL53L4CD_ERROR_NONE)
  {
    if (++tofI2cFailCount >= TOF_I2C_FAIL_LIMIT)
    {
      markTofLost(F("I2C error"));
    }
    return;
  }

  tofI2cFailCount = 0;
  lastRangeMm = result.distance_mm;
  // Status 0 is "range valid". Status 2 is signal-fail (no return);
  // the distance register is then 0 and must not be shown as a range.
  lastRangeValid = (result.range_status == 0 && result.distance_mm > 0 &&
                    result.distance_mm <= TOF_OUT_OF_RANGE_MM);
}

// --- Drawing ---

void drawDistance(StepScreenDisplay &d)
{
  d.print("Dist: ");
  if (!tofOk)
  {
    d.println("No ToF");
  }
  else if (!lastRangeValid)
  {
    d.println("--");
  }
  else
  {
    d.print(lastRangeMm);
    d.println(" mm");
  }
}

void drawHome(StepScreenDisplay &d)
{
  char infoBar[12];
  rtc.formatInfoBar(infoBar, sizeof(infoBar));
  d.drawInfoBar(infoBar);
  const StepScreenActionLabels labels = {"Set", "Dbg",
                                         motorReady ? "Run" : "XXX"};
  d.drawActionColumn(labels, nav.readBack(), nav.readPush(),
                     nav.readConfirm());

  d.setTextSize(2);
  d.setCursor(STEPSCREEN_CONTENT_X, STEPSCREEN_CONTENT_Y + 2);
  d.print(syringeMl());
  d.println(" mL");

  d.setTextSize(1);
  d.setCursor(STEPSCREEN_CONTENT_X, STEPSCREEN_CONTENT_Y + 22);
  d.print("Rwd ");
  d.print(rewardUl);
  d.println(" uL");
  d.setCursor(STEPSCREEN_CONTENT_X, STEPSCREEN_CONTENT_Y + 34);
  drawDistance(d);
  if (!motorReady)
  {
    d.setCursor(STEPSCREEN_CONTENT_X, STEPSCREEN_CONTENT_Y + 46);
    d.println("No 12V");
  }
}

void drawSet(StepScreenDisplay &d)
{
  d.drawInfoBar("Set");
  if (setSavedUntilMs != 0)
  {
    const StepScreenActionLabels labels = {nullptr, nullptr, nullptr};
    d.drawActionColumn(labels, false, false, false);
    d.setCursor(STEPSCREEN_CONTENT_X, STEPSCREEN_CONTENT_Y + 16);
    d.setTextSize(2);
    d.println(setSaveOk ? "Saved" : "No SD");
    d.setTextSize(1);
    d.println();
    d.print(syringeMl());
    d.print(" mL  ");
    d.print(rewardUl);
    d.println(" uL");
    return;
  }

  const char *fieldLabel = (setField == SET_FIELD_ML) ? "mL" : "uL";
  const StepScreenActionLabels labels = {"Back", fieldLabel, "Save"};
  d.drawActionColumn(labels, nav.readBack(), nav.readPush(),
                     nav.readConfirm());

  d.setCursor(STEPSCREEN_CONTENT_X, STEPSCREEN_CONTENT_Y + 4);
  if (setField == SET_FIELD_ML)
  {
    d.println("Syringe:");
    d.println();
    if (setDraftMlIndex > 0)
    {
      d.print("  ");
      d.print(SYRINGE_ML_OPTIONS[setDraftMlIndex - 1]);
      d.println(" mL");
    }
    else
    {
      d.println();
    }
    d.print("> ");
    d.print(SYRINGE_ML_OPTIONS[setDraftMlIndex]);
    d.println(" mL");
    if (setDraftMlIndex + 1 < SYRINGE_ML_COUNT)
    {
      d.print("  ");
      d.print(SYRINGE_ML_OPTIONS[setDraftMlIndex + 1]);
      d.println(" mL");
    }
  }
  else
  {
    d.println("Reward:");
    d.println();
    d.setTextSize(2);
    d.print(setDraftRewardUl);
    d.println(" uL");
    d.setTextSize(1);
    d.println();
    d.print("-> ");
    d.print((long)lroundf((float)setDraftRewardUl *
                          SYRINGE_STEPS_PER_UL[setDraftMlIndex]));
    d.println(" stp");
  }
}

void drawAdjust(StepScreenDisplay &d)
{
  d.drawInfoBar("Dbg");
  const StepScreenActionLabels labels = {"Back", nullptr, "Spd"};
  d.drawActionColumn(labels, nav.readBack(), false, nav.readConfirm());

  d.setCursor(STEPSCREEN_CONTENT_X, STEPSCREEN_CONTENT_Y + 4);
  d.print("Pos: ");
  d.println(motorCurrentPosition());
  d.print("Spd: ");
  d.print(stepper.speedPresetName());
  d.print(" ");
  d.print((int32_t)stepper.speedPresetSps());
  d.println("/s");
  d.print("Enc: ");
  d.println(lastEncoderDelta);
  drawDistance(d);
  if (!motorReady)
  {
    d.println("No 12V");
  }
}

void drawRun(StepScreenDisplay &d)
{
  d.drawInfoBar("Run");
  const StepScreenActionLabels labels = {"Back", nullptr, nullptr};
  d.drawActionColumn(labels, nav.readBack(), false, false);

  const uint32_t totalUl = (uint32_t)auxFireCount * (uint32_t)rewardUl;

  d.setCursor(STEPSCREEN_CONTENT_X, STEPSCREEN_CONTENT_Y + 2);
  d.print("Syr ");
  d.print(syringeMl());
  d.print("mL, Rwd ");
  d.print(rewardUl);
  d.println("uL");

  d.print("Trig #");
  d.print(auxFireCount);
  d.print(" = ");
  d.print(totalUl);
  d.println("uL");

  d.print("Position ");
  d.println(motorCurrentPosition());

  d.print("Status ");
  switch (runStatus)
  {
  case RUN_MOVING:
    d.println("DISPENSE");
    break;
  case RUN_ERROR:
    d.println("ERROR");
    break;
  case RUN_WAITING:
  case RUN_DONE:
  default:
    d.println("READY");
    break;
  }
}

void drawScreen()
{
  static uint32_t lastDrawMs = 0;
  if (millis() - lastDrawMs < DRAW_INTERVAL_MS)
  {
    return;
  }
  lastDrawMs = millis();

  StepScreenDisplay &d = screen.display();
  d.clearDisplay();
  switch (currentScreen)
  {
  case SCREEN_HOME:
    drawHome(d);
    break;
  case SCREEN_SET:
    drawSet(d);
    break;
  case SCREEN_ADJUST:
    drawAdjust(d);
    break;
  case SCREEN_RUN:
    drawRun(d);
    break;
  }
  d.display();
}

void setup()
{
  Serial.begin(115200);
  delay(1000); // let USB serial settle; no while(!Serial) so battery works

  if (!screen.begin())
  {
    Serial.println(F("SyringePump: display not found"));
    while (1)
    {
      delay(100);
    }
  }

  rtc.begin();
  rtc.syncFromBuildTimeIfNeeded();

  sd.begin();
  sd.setActivityLed(false);
  sdReady = waitForSdReady(2000);
  Serial.print(F("[SD] "));
  Serial.println(sdReady ? F("ready") : F("not ready"));
  if (sdReady)
  {
    ensureSyringeConfig();
  }

  tofOk = initTof();
  if (!tofOk)
  {
    Serial.println(F("ToF unavailable — UI continues, Dist: --"));
  }

  nav.begin(&screen.input());

  tmc.begin();
  motorConfigured = false;
  if (stepper.beginWithDriver(tmc))
  {
    motorConfigured = true;
    refreshMotorReady(); // apply uv_cp check after configure
  }
  else
  {
    motorReady = false;
    Serial.println(F("TMC2209 UART failed — UI only, no motion."));
    StepScreenTMC2209::printWiringHelp();
    stepper.begin();
  }
  // Boot on Fast; OK on Dbg still cycles Low -> Med -> Fast.
  stepper.setSpeedPreset(STEPSCREEN_SPEED_FAST);
  lastMotorReadyPollMs = millis();

#if defined(ARDUINO_ARCH_SAMD)
  pinMode(PIN_MOTOR_STEP, OUTPUT);
  pinMode(PIN_MOTOR_DIR, OUTPUT);
  digitalWrite(PIN_MOTOR_STEP, LOW);
  digitalWrite(PIN_MOTOR_DIR, LOW);
  initMotorTimer();
#else
  Serial.println(F("Warning: TC3 step ISR requires SAMD21"));
#endif

  // --- STEPSCREEN ENCODER ISR SETUP (copy into setup()) ---
  STEPSCREEN_ATTACH_ENCODER_ISRS();
  // --- END STEPSCREEN ENCODER ISR SETUP ---

  StepScreenSplashConfig splash;
  splash.title = "Syringe Pump v1.1";
  StepScreenSplash::play(screen.display(), splash, &screen.input());

  enterHome();
}

void loop()
{
  serviceMotor();
  pollTof();
  pollSd();
  pollMotorReady();

  const uint8_t ev = nav.pollNavEvents();
  const int32_t enc = nav.consumeEncoderDelta();

  switch (currentScreen)
  {
  case SCREEN_HOME:
    handleHome(ev);
    break;
  case SCREEN_SET:
    handleSet(ev, enc);
    break;
  case SCREEN_ADJUST:
    handleAdjust(ev, enc);
    break;
  case SCREEN_RUN:
    handleRun(ev);
    break;
  }

  drawScreen();
}
