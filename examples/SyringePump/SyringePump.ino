/*
  StepScreen SyringePump

  Four-screen pump controller demonstrating the StepScreenNav
  (menu-safe debouncing) and StepScreenStepper (AccelStepper/TMC2209)
  layers.

  Home   -- motor disabled.
            Back = Set, Sel = Dbg (jog), OK = Run.
            Shows syringe_ml, reward_ul, and live VL53L4CD distance (mm).
  Set    -- edit syringe size (10/30/60 mL) or reward_ul with the encoder.
            Center press toggles which field is active. OK = Save writes
            /SYRING.CFG and shows a brief confirmation. Back discards.
  Dbg    -- motor enabled; encoder jogs the motor in both directions.
            OK cycles the speed preset (Low -> Med -> Fast).
            Back returns Home. Shows position, speed, encoder input,
            and live VL53L4CD distance.
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
  continues and distance shows "--". Requires stm32duino VL53L4CD when used.

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
#define STEPSCREEN_SPEED_FAST_SPS 6000.0f
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

bool motorReady = false;
bool sdReady = false;
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

VL53L4CD tof(&Wire, PIN_TOF_XSHUT);
bool tofOk = false;
uint16_t lastRangeMm = 0;
bool lastRangeValid = false;
uint32_t lastTofMs = 0;

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
char rxBuf[16];
uint8_t rxLen = 0;
int32_t lastCommandSteps = 0;
int32_t lastEncoderDelta = 0;

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

  if (dist == 0)
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
  if (motorReady)
  {
    stepper.enable();
    setMotorTickActive(true);
    lastRampMs = millis();
  }
  nav.enterScreen();
  Serial.println(F("[Dbg] encoder jogs motor"));
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

// --- Run-screen serial protocol ---

void handleSerialCommand()
{
  while (Serial.available())
  {
    const char c = (char)Serial.read();

    if (c == '\n' || c == '\r')
    {
      if (rxLen == 0)
      {
        continue; // ignore blank lines / CRLF pairs
      }
      rxBuf[rxLen] = '\0';
      rxLen = 0;

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
    else if (rxLen < sizeof(rxBuf) - 1)
    {
      rxBuf[rxLen++] = c;
    }
    else
    {
      rxLen = 0; // overflow: drop the line
      runStatus = RUN_ERROR;
      Serial.println(F("error: command too long"));
    }
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
    lastEncoderDelta = enc;
    motorMoveRelative(enc * JOG_STEPS_PER_DETENT);
  }
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

  Serial.println(F("SyringePump: VL53L4CD ready @ 0x29"));
  return true;
}

void pollTof()
{
  if (!tofOk)
  {
    return;
  }
  if (millis() - lastTofMs < TOF_POLL_MS)
  {
    return;
  }
  lastTofMs = millis();

  uint8_t ready = 0;
  if (tof.VL53L4CD_CheckForDataReady(&ready) != VL53L4CD_ERROR_NONE || !ready)
  {
    return;
  }

  tof.VL53L4CD_ClearInterrupt();

  VL53L4CD_Result_t result;
  if (tof.VL53L4CD_GetResult(&result) != VL53L4CD_ERROR_NONE)
  {
    return;
  }

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
  if (!tofOk || !lastRangeValid)
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
  const StepScreenActionLabels labels = {"Set", "Dbg", "Run"};
  d.drawActionColumn(labels, nav.readBack(), nav.readPush(),
                     nav.readConfirm());

  d.setTextSize(2);
  d.setCursor(STEPSCREEN_CONTENT_X, STEPSCREEN_CONTENT_Y + 2);
  d.print(syringeMl());
  d.println(" mL");

  d.setTextSize(1);
  d.setCursor(STEPSCREEN_CONTENT_X, STEPSCREEN_CONTENT_Y + 24);
  d.print("Rwd ");
  d.print(rewardUl);
  d.println(" uL");
  d.setCursor(STEPSCREEN_CONTENT_X, STEPSCREEN_CONTENT_Y + 36);
  drawDistance(d);
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
}

void drawRun(StepScreenDisplay &d)
{
  d.drawInfoBar("Run");
  const StepScreenActionLabels labels = {"Back", nullptr, nullptr};
  d.drawActionColumn(labels, nav.readBack(), false, false);

  d.setCursor(STEPSCREEN_CONTENT_X, STEPSCREEN_CONTENT_Y + 2);
  d.print(syringeMl());
  d.print("mL ");
  d.print(rewardUl);
  d.println("uL");
  d.print("Rwd ");
  d.print(rewardSteps());
  d.println(" stp");
  d.print("AUX ");
  d.print(auxStableLevel ? 'H' : 'L');
  d.print(" #");
  d.println(auxFireCount);

  d.print("St: ");
  switch (runStatus)
  {
  case RUN_WAITING:
    d.println("wait");
    break;
  case RUN_MOVING:
    d.print("mv ");
    d.println(lastCommandSteps);
    break;
  case RUN_DONE:
    d.print("ok ");
    d.println(lastCommandSteps);
    break;
  case RUN_ERROR:
    d.println("err");
    break;
  }
  d.print("Pos ");
  d.println(motorCurrentPosition());
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
  motorReady = stepper.beginWithDriver(tmc);
  if (!motorReady)
  {
    Serial.println(F("TMC2209 UART failed — UI only, no motion."));
    StepScreenTMC2209::printWiringHelp();
    stepper.begin();
  }
  // Boot on Fast; OK on Dbg still cycles Low -> Med -> Fast.
  stepper.setSpeedPreset(STEPSCREEN_SPEED_FAST);

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

  StepScreenSplash::play(screen.display(), StepScreenSplashConfig{},
                         &screen.input());

  enterHome();
}

void loop()
{
  serviceMotor();
  pollTof();
  pollSd();

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
