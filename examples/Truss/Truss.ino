/*
  StepScreen TRUSS

  Closed-loop cable tension controller: HX711 load cell inline with a
  cable wrapped on a stepper spool. Wind until a target load (grams) is
  reached, then maintain it.

  Wiring:
    HX711 SCK -> CLK  (PA30 / SWCLK)
    HX711 DT  -> DIO  (PA31 / SWDIO)
    HX711 VCC -> 3.3V, GND -> GND
    TMC2209 step/dir as on the StepScreen board

  HX711 sample rate:
    Many off-the-shelf modules ship with RATE soldered for 10 Hz (~100 ms
    per sample). That is too slow for this closed loop and causes overshoot.
    Prefer 80 Hz: move the RATE jumper/solder blob from the 10 Hz pad to
    the 80 Hz pad (RATE high / tied toward VCC; 10 Hz is RATE low / GND).
    After that change, a new conversion is ready about every 12.5 ms.

  Calibration (Cal screen):
    1. Remove weight; RUN tares (0g TARE step).
    2. Hang a known mass; set grams with encoder (50-500); RUN calibrates.
    3. SAVED flashes, then step 1 repeats. BACK returns Home.

  Control: hysteresis + near-target nudge. Stop when |error| < TRUSS_STOP_G;
  do not start again until |error| > TRUSS_START_G. Inside TRUSS_NUDGE_G,
  each HX711 sample issues TRUSS_NUDGE_STEPS then waits. Between that and
  TRUSS_APPROACH_G, creep at TRUSS_APPROACH_SPS. Farther out, speed is
  proportional (TRUSS_GAIN_SPS_PER_G, capped at TRUSS_SPS).

  Control tuning (sketch #defines below):
    TRUSS_SPS, TRUSS_GAIN_SPS_PER_G, TRUSS_STOP_G, TRUSS_START_G,
    TRUSS_APPROACH_G, TRUSS_APPROACH_SPS, TRUSS_NUDGE_G, TRUSS_NUDGE_STEPS,
    TRUSS_ACCEL_SPS2,
    TRUSS_RAMP_MS, TRUSS_FILTER_ALPHA,
    TRUSS_WIND_SIGN (+1 if winding increases load),
    TRUSS_DIR_TIMEOUT_MS (continuous same-direction limit -> fault),
    TRUSS_OVERLOAD_G, TRUSS_MAX_STEPS_UNSETTLED

  Stepping uses a TC3 ISR (one interrupt per pulse) so HX711 reads and the
  OLED cannot starve the motor. Load samples are non-blocking (one conversion
  when DOUT is ready).

  Session logging (SD + RTC): each Run creates /YYYYMMDD/HHMMSS.CSV (same
  FAT 8.3 layout as Treadmill). Columns: datetime,event,target_load_g,
  actual_load_g. Events: begin, measure (~1 Hz), end, fault. Samples buffer
  for ~2 minutes in RAM; a full buffer briefly holds the motor (enabled,
  zero speed — not released) while flushing, then closed-loop resumes.
  Remaining rows flush when the session ends.

  Board: Adafruit Feather M0 Adalogger (or compatible SAMD21).

  Calibration is saved to /TRUSS.CAL on the microSD card (required for Cal).
 */

#ifndef STEPSCREEN_NAV_SETTLE_MS
#define STEPSCREEN_NAV_SETTLE_MS 100
#endif

#ifndef STEPSCREEN_MOTOR_MICROSTEPS
#define STEPSCREEN_MOTOR_MICROSTEPS 32
#endif

#ifndef STEPSCREEN_TMC2209_RUN_CURRENT_MA
#define STEPSCREEN_TMC2209_RUN_CURRENT_MA 800
#endif

// --- Control tuning ---
// Top speed when the load is far from target (steps per second). Raise for
// faster spool-up; lower if the cable jerks hard when taking up slack.
#ifndef TRUSS_SPS
#define TRUSS_SPS 2000.0f
#endif

// How strongly the motor reacts to each gram of error. Higher = snappier
// and more likely to overshoot; lower = gentler and slower to settle.
#ifndef TRUSS_GAIN_SPS_PER_G
#define TRUSS_GAIN_SPS_PER_G 12.0f
#endif

// Inner band: stop when |error| is inside this (grams). This is how close
// we try to match the target.
#ifndef TRUSS_STOP_G
#define TRUSS_STOP_G 4.0f
#endif

// Outer band: after a stop, do not start again until |error| exceeds this.
// Keeps HX711 noise from chattering around the inner band.
#ifndef TRUSS_START_G
#define TRUSS_START_G 8.0f
#endif

// Mid band: continuous creep at TRUSS_APPROACH_SPS when |error| is below
// this but still above the nudge window (recover after a lift/drop without
// blasting through the stop band).
#ifndef TRUSS_APPROACH_G
#define TRUSS_APPROACH_G 40.0f
#endif

#ifndef TRUSS_APPROACH_SPS
#define TRUSS_APPROACH_SPS 80.0f
#endif

// Very near target: a few steps per HX711 sample, then wait (stiff cable).
#ifndef TRUSS_NUDGE_G
#define TRUSS_NUDGE_G 12.0f
#endif

#ifndef TRUSS_NUDGE_STEPS
#define TRUSS_NUDGE_STEPS 4
#endif

// How quickly speed can change (steps/s^2). Raise for punchier starts/stops;
// lower if the spool sounds rough when speeding up or reversing.
#ifndef TRUSS_ACCEL_SPS2
#define TRUSS_ACCEL_SPS2 4000.0f
#endif

// How often the speed ramp updates (ms). Smaller = smoother accel; leave
// alone unless you are debugging motor feel.
#ifndef TRUSS_RAMP_MS
#define TRUSS_RAMP_MS 5
#endif

// How often Run prints a status line on Serial (ms). Set to 0 to turn off.
#ifndef TRUSS_DEBUG_MS
#define TRUSS_DEBUG_MS 200
#endif

// Load-cell smoothing (0–1). 1.0 = use each reading as-is (fastest, noisier).
// Lower = more filtering (calmer, but slower to notice a real load change).
#ifndef TRUSS_FILTER_ALPHA
#define TRUSS_FILTER_ALPHA 0.75f
#endif

// Flip wind/unwind if the spool moves the wrong way (+1 or -1).
#ifndef TRUSS_WIND_SIGN
#define TRUSS_WIND_SIGN -1
#endif

// Stop and fault if the motor keeps going one way this long FAR from
// target without progress (slack cable, wrong TRUSS_WIND_SIGN, bad cal).
// The clock resets on any direction change, on progress, and whenever the
// load is back within TRUSS_APPROACH_G of target.
#ifndef TRUSS_DIR_TIMEOUT_MS
#define TRUSS_DIR_TIMEOUT_MS 5000
#endif

// Hard stop if live load exceeds this (grams).
#ifndef TRUSS_OVERLOAD_G
#define TRUSS_OVERLOAD_G 800.0f
#endif

// Hard stop if the motor takes more than this many steps without the load
// getting back near target (runaway). The count resets whenever the load
// is within TRUSS_APPROACH_G, settles in the stop band, or HOLD is used.
#ifndef TRUSS_MAX_STEPS_UNSETTLED
#define TRUSS_MAX_STEPS_UNSETTLED 200000L
#endif

// How many HX711 samples to average when taring / calibrating.
#ifndef TRUSS_CAL_SAMPLES
#define TRUSS_CAL_SAMPLES 20
#endif

// Session log: one "measure" row this often (ms). Requires SD + RTC.
#ifndef TRUSS_LOG_MS
#define TRUSS_LOG_MS 1000
#endif

// RAM buffer depth (~2 min at 1 Hz). Full buffer -> hold motor, flush, resume.
#ifndef TRUSS_LOG_BUFFER
#define TRUSS_LOG_BUFFER 120
#endif

#define TARGET_MIN_G 0
#define TARGET_MAX_G 500
#define CAL_MIN_G 50
#define CAL_MAX_G 500

#define DRAW_INTERVAL_MS 50
#define DRAW_INTERVAL_RUNNING_MS 100
#define SAVED_FLASH_MS 1000
#define NOCAL_FLASH_MS 800

#define TRUSS_SD_CAL_PATH "/TRUSS.CAL"

#include <StepScreen.h>
#include <StepScreenHX711.h>
#include <StepScreenNav.h>
#include <StepScreenRTC.h>
#include <StepScreenSD.h>
#include <StepScreenSplash.h>
#include <StepScreenStepper.h>
#include <StepScreenTMC2209.h>

#include <RTClib.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct TrussCalData {
  int32_t offset;
  float scale;
  uint8_t calibrated;
  uint8_t reserved[3];
};

StepScreen screen;
StepScreenRTC rtc;
StepScreenSD sd;
StepScreenNav nav;
StepScreenTMC2209 tmc;
StepScreenStepper stepper;
StepScreenHX711 scale;

bool motorReady = false;
bool loadCellReady = false;
bool sdReady = false;
bool loadCalibrated = false;

enum TrussScreen : uint8_t {
  SCREEN_HOME,
  SCREEN_CAL,
  SCREEN_RUN,
};

enum RunPhase : uint8_t {
  PHASE_RUNNING,
  PHASE_HELD,
  PHASE_FAULT,
};

enum CalPhase : uint8_t {
  CAL_TARE,
  CAL_WEIGHT,
  CAL_SAVED,
};

TrussScreen currentScreen = SCREEN_HOME;
RunPhase runPhase = PHASE_RUNNING;
CalPhase calPhase = CAL_TARE;

int16_t targetGrams = 100;
int16_t calGrams = 200;
float liveGrams = 0.0f;

char pushLabel[6] = "100g";

uint32_t savedFlashUntilMs = 0;
uint32_t noCalFlashUntilMs = 0;

bool displayDirty = true;

// Closed-loop state
enum TensionState : uint8_t {
  TENSION_IDLE,
  TENSION_WIND,
  TENSION_UNWIND,
};
TensionState tensionState = TENSION_IDLE;
uint32_t tensionStateSinceMs = 0;
float lastAbsErrForTimeout = 0.0f;
uint32_t stepsAtSettle = 0; // sessionStepCount when last near target
float targetSpsCmd = 0.0f; // set by control loop, chased by the ramp
float currentSps = 0.0f;   // ramped speed actually applied to the timer
bool loadFilterSeeded = false;
bool loadSampleFresh = false;
uint32_t lastRampMs = 0;
volatile uint32_t sessionStepCount = 0;

volatile bool motorTickActive = false;
volatile uint32_t stepPeriodTicks = 0; // timer ticks between steps; 0 = hold

#if defined(ARDUINO_ARCH_SAMD)
#define MOTOR_GCLK_HZ 48000000UL
#define MOTOR_TIMER_PRESCALE 2
#define MOTOR_TIMER_HZ (MOTOR_GCLK_HZ / MOTOR_TIMER_PRESCALE)
#define MOTOR_PERIOD_MAX 65536u
#define STEP_PULSE_US 2
#define MOTOR_PERIOD_UPDATE_PCT 2u

void motorTimerStop() {
  TC3->COUNT16.CTRLA.bit.ENABLE = 0;
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY) {
  }
  TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;
}

void motorTimerApplyPeriod(uint32_t period) {
  noInterrupts();
  stepPeriodTicks = period;

  if (!TC3->COUNT16.CTRLA.bit.ENABLE) {
    // Starting from stopped: load period, zero the count, enable.
    TC3->COUNT16.CC[0].reg = (uint16_t)(period - 1);
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY) {
    }
    TC3->COUNT16.COUNT.reg = 0;
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY) {
    }
    TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;
    TC3->COUNT16.CTRLA.bit.ENABLE = 1;
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY) {
    }
    interrupts();
    return;
  }

  // Already running: update the period on the fly so the pulse train is
  // never interrupted (stop/restart here caused audible stutter on ramps).
  TC3->COUNT16.CC[0].reg = (uint16_t)(period - 1);
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY) {
  }

  // If the counter already passed the new top it would have to wrap through
  // 0xFFFF once; retrigger instead so the next step lands on schedule.
  TC3->COUNT16.READREQ.reg = TC_READREQ_RREQ | TC_READREQ_ADDR(0x10);
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY) {
  }
  if (TC3->COUNT16.COUNT.reg >= (uint16_t)(period - 1)) {
    TC3->COUNT16.CTRLBSET.reg = TC_CTRLBSET_CMD_RETRIGGER;
    while (TC3->COUNT16.STATUS.bit.SYNCBUSY) {
    }
  }
  interrupts();
}

void initMotorTimer() {
  GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 |
                                 GCLK_CLKCTRL_ID_TCC2_TC3);
  while (GCLK->STATUS.bit.SYNCBUSY) {
  }

  TC3->COUNT16.CTRLA.bit.ENABLE = 0;
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY) {
  }

  TC3->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_WAVEGEN_MFRQ |
                             TC_CTRLA_PRESCALER_DIV2;
  TC3->COUNT16.INTENSET.reg = TC_INTENSET_MC0;
  TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;
  NVIC_EnableIRQ(TC3_IRQn);
}

void isrStepPulse() {
  digitalWrite(PIN_MOTOR_STEP, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(PIN_MOTOR_STEP, LOW);
}

void TC3_Handler() {
  if (TC3->COUNT16.INTFLAG.bit.MC0 == 0) {
    return;
  }
  TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;
  if (!motorTickActive || stepPeriodTicks == 0) {
    return;
  }
  isrStepPulse();
  sessionStepCount++;
}
#endif

void applyStepRate(float sps, bool force = false);

void setMotorTickActive(bool active) {
  motorTickActive = active;
#if defined(ARDUINO_ARCH_SAMD)
  noInterrupts();
  motorTimerStop();
  if (!active) {
    stepPeriodTicks = 0;
  }
  interrupts();
#endif
}

// Session timer (pauses while held)
uint32_t sessionElapsedMs = 0;
uint32_t sessionTickMs = 0;

// Session CSV logging (Treadmill-style path; buffered measure rows)
char sessionLogPath[24] = {};
uint32_t logSessionStartMs = 0;
DateTime logSessionStartDt;
struct TrussLogSample {
  uint32_t ms;
  int16_t targetG;
  float actualG;
};
TrussLogSample logBuffer[TRUSS_LOG_BUFFER];
uint16_t logCount = 0;
uint32_t lastLogMs = 0;
bool logFlushActive = false;

void drawHomeSdStatus(StepScreenDisplay &d);
void drawHomeHardwareFooter(StepScreenDisplay &d);
void drawLargeGrams(StepScreenDisplay &d, int16_t y, float grams,
                    bool showNoCal);
void updatePushLabel();
bool loadCalibration();
bool saveCalibration();
void enterHome();
void enterCal();
void enterRun();
void serviceRun();
void serviceMotorRamp();
void serviceRunDebug();
void serviceSessionLog();
void serviceCal();
void updateLoadCellDetect();
void updateSdStatus();
void readLiveLoad();
bool formatSessionPath(char *filePath, size_t fileLen, char *dirPath,
                       size_t dirLen);
void formatSampleTimestamp(char *buf, size_t len, uint32_t sampleMs);
void holdMotorForLogFlush();
bool flushLogBuffer();
void startSessionLog();
void finishSessionLog(const char *event);

bool calAllowed() { return loadCellReady && sdReady; }

bool runAllowed() { return motorReady && loadCellReady && sdReady; }

// A few STEP pulses, then stop. Used near target so one HX711 sample cannot
// wind far enough to overshoot. DIR follows the same sign as applyStepRate.
void nudgeSteps(int8_t sign) {
  applyStepRate(0.0f, true);
  targetSpsCmd = 0.0f;
  currentSps = 0.0f;

  digitalWrite(PIN_MOTOR_DIR, (sign >= 0) ? HIGH : LOW);
  delayMicroseconds(5);

  for (uint8_t i = 0; i < (uint8_t)TRUSS_NUDGE_STEPS; i++) {
    digitalWrite(PIN_MOTOR_STEP, HIGH);
    delayMicroseconds(2);
    digitalWrite(PIN_MOTOR_STEP, LOW);
    delayMicroseconds(2);
    sessionStepCount++;
  }
}

void applyStepRate(float sps, bool force) {
  if (!motorReady || runPhase == PHASE_FAULT) {
    setMotorTickActive(false);
    return;
  }

  const float absSps = fabsf(sps);

#if defined(ARDUINO_ARCH_SAMD)
  digitalWrite(PIN_MOTOR_DIR, (sps >= 0.0f) ? HIGH : LOW);

  if (!motorTickActive || absSps < 1.0f) {
    noInterrupts();
    motorTimerStop();
    stepPeriodTicks = 0;
    interrupts();
    return;
  }

  uint32_t period = (uint32_t)(MOTOR_TIMER_HZ / absSps);
  if (period < 2) {
    period = 2;
  } else if (period > MOTOR_PERIOD_MAX) {
    period = MOTOR_PERIOD_MAX;
  }

  if (!force && stepPeriodTicks > 0 && TC3->COUNT16.CTRLA.bit.ENABLE) {
    if (period == stepPeriodTicks) {
      return;
    }
    const uint32_t diff =
        (period > stepPeriodTicks) ? (period - stepPeriodTicks)
                                   : (stepPeriodTicks - period);
    if (diff * 100u < stepPeriodTicks * MOTOR_PERIOD_UPDATE_PCT) {
      return;
    }
  }

  motorTimerApplyPeriod(period);
#else
  if (motorTickActive && absSps >= 1.0f) {
    stepper.setSpeed(sps);
  } else {
    stepper.setSpeed(0.0f);
  }
#endif
}

void drawHomeSdStatus(StepScreenDisplay &d) {
  const bool ready = sd.isReady();
  const int16_t sdTextW = 12;
  const int16_t checkW = 7;
  const int16_t gap = 2;
  const int16_t pad = 2;
  const int16_t groupW = (ready ? checkW + gap : 0) + sdTextW;
  const int16_t x = STEPSCREEN_CONTENT_W - pad - groupW;

  d.setTextSize(1);
  d.setTextColor(SH110X_BLACK);
  if (ready) {
    d.drawLine(x, 3, x + 2, 6, SH110X_BLACK);
    d.drawLine(x, 2, x + 2, 5, SH110X_BLACK);
    d.drawLine(x + 2, 6, x + 6, 1, SH110X_BLACK);
    d.drawLine(x + 2, 5, x + 6, 0, SH110X_BLACK);
    d.setCursor(x + checkW + gap, 0);
  } else {
    d.setCursor(x, 0);
  }
  d.print(F("SD"));
  d.setTextColor(SH110X_WHITE);
}

void drawHomeHardwareFooter(StepScreenDisplay &d) {
  if (motorReady && loadCellReady && sdReady) {
    return;
  }

  // Bottom-align so all lines fit when everything is missing.
  const int16_t missing = (int16_t)!motorReady + (int16_t)!loadCellReady +
                          (int16_t)!sdReady;
  d.setTextSize(1);
  int16_t y = min((int16_t)48, (int16_t)(STEPSCREEN_H - 8 * missing));
  if (!motorReady) {
    d.setCursor(2, y);
    d.print(F("xStepper"));
    y += 8;
  }
  if (!loadCellReady) {
    d.setCursor(2, y);
    d.print(F("xLoad Cell"));
    y += 8;
  }
  if (!sdReady) {
    d.setCursor(2, y);
    d.print(F("xSD Card"));
  }
}

void drawLargeGrams(StepScreenDisplay &d, int16_t y, float grams,
                    bool showNoCal) {
  if (showNoCal) {
    const bool flash = noCalFlashUntilMs != 0 &&
                       (int32_t)(millis() - noCalFlashUntilMs) < 0 &&
                       ((millis() / 200) & 1);
    d.setTextSize(2);
    d.setTextColor(flash ? SH110X_BLACK : SH110X_WHITE);
    if (flash) {
      d.fillRect(0, y - 2, STEPSCREEN_CONTENT_W - 4, 19, SH110X_WHITE);
    }
    d.setCursor(2, y);
    d.print(F("NO CAL"));
    d.setTextColor(SH110X_WHITE);
    return;
  }

  char buf[8];
  snprintf(buf, sizeof(buf), "%d", (int)lroundf(grams));
  d.setTextSize(2);
  d.setTextColor(SH110X_WHITE);
  d.setCursor(2, y);
  d.print(buf);
  d.setTextSize(1);
  d.setCursor(d.getCursorX() + 3, y + 7);
  d.print(F("g"));
}

void updatePushLabel() {
  snprintf(pushLabel, sizeof(pushLabel), "%dg", targetGrams);
}

bool parseCalLine(const char *line, TrussCalData &data) {
  if (line == nullptr) {
    return false;
  }
  const char *o = strstr(line, "O=");
  const char *s = strstr(line, "S=");
  if (o == nullptr || s == nullptr) {
    return false;
  }
  data.offset = (int32_t)strtol(o + 2, nullptr, 10);
  data.scale = (float)atof(s + 2);
  data.calibrated = 1;
  return data.scale != 0.0f;
}

bool loadCalFromSd(TrussCalData &data) {
  if (!sd.isReady()) {
    Serial.println(F("[Cal] SD not mounted"));
    return false;
  }

  static const char *const kPaths[] = {TRUSS_SD_CAL_PATH, "TRUSS.CAL"};
  char line[64];
  for (uint8_t i = 0; i < 2; i++) {
    if (!sd.readLine(kPaths[i], line, sizeof(line))) {
      Serial.print(F("[Cal] "));
      Serial.print(kPaths[i]);
      Serial.print(F(": "));
      Serial.println(sd.lastError());
      continue;
    }
    Serial.print(F("[Cal] "));
    Serial.print(kPaths[i]);
    Serial.print(F(" '"));
    Serial.print(line);
    Serial.println(F("'"));
    if (parseCalLine(line, data)) {
      return true;
    }
    Serial.println(F("[Cal] parse failed"));
  }
  return false;
}

bool saveCalToSd(const TrussCalData &data) {
  if (!sd.isReady()) {
    return false;
  }

  char line[48];
  const int n = snprintf(line, sizeof(line), "O=%ld,S=%.6f\n",
                         (long)data.offset, (double)data.scale);
  if (n <= 0 || (size_t)n >= sizeof(line)) {
    return false;
  }

  return sd.writeFile(TRUSS_SD_CAL_PATH, (const uint8_t *)line, (size_t)n,
                      false);
}

void applyCalibration(const TrussCalData &data) {
  scale.set_offset(data.offset);
  scale.set_scale(data.scale);
  loadCalibrated = data.calibrated != 0;
}

bool loadCalibration() {
  TrussCalData data = {};
  if (loadCalFromSd(data)) {
    applyCalibration(data);
    return true;
  }
  loadCalibrated = false;
  return false;
}

bool saveCalibration() {
  TrussCalData data = {};
  data.offset = scale.get_offset();
  data.scale = scale.get_scale();
  data.calibrated = 1;

  const bool ok = saveCalToSd(data);
  loadCalibrated = ok;
  return ok;
}

void initStatusLeds() {
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_RED, LOW);
}

void playBootLedDance() {
  initStatusLeds();
  for (uint8_t i = 0; i < 3; i++) {
    digitalWrite(PIN_LED_GREEN, HIGH);
    delay(70);
    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_LED_RED, HIGH);
    delay(70);
    digitalWrite(PIN_LED_RED, LOW);
    delay(50);
  }
  digitalWrite(PIN_LED_GREEN, HIGH);
  digitalWrite(PIN_LED_RED, HIGH);
  delay(100);
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_RED, LOW);
}

void updateStatusLeds() {
  if (currentScreen == SCREEN_HOME || currentScreen == SCREEN_CAL) {
    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_LED_RED, LOW);
    return;
  }

  if (runPhase == PHASE_FAULT) {
    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_LED_RED, (millis() / 500) & 1 ? HIGH : LOW);
    return;
  }

  if (runPhase == PHASE_HELD) {
    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_LED_RED, (millis() / 400) & 1 ? HIGH : LOW);
    return;
  }

  digitalWrite(PIN_LED_GREEN, HIGH);
  digitalWrite(PIN_LED_RED, LOW);
}

void readLiveLoad() {
  if (!scale.is_ready()) {
    return;
  }
  const float raw = scale.get_units(1);
  if (!loadFilterSeeded) {
    liveGrams = raw;
    loadFilterSeeded = true;
  } else {
    liveGrams += TRUSS_FILTER_ALPHA * (raw - liveGrams);
  }
  loadSampleFresh = true;
  if (!loadCellReady) {
    loadCellReady = true;
    displayDirty = true;
    Serial.println(F("[Load Cell] online"));
  }
}

void updateLoadCellDetect() {
  if (loadCellReady) {
    return;
  }
  if (scale.is_ready()) {
    loadCellReady = true;
    displayDirty = true;
    Serial.println(F("[Load Cell] online"));
  }
}

void updateSdStatus() {
  sd.update();
  const bool ready = sd.isReady();
  if (ready == sdReady) {
    return;
  }
  sdReady = ready;
  displayDirty = true;
  Serial.print(F("[SD] "));
  Serial.println(sdReady ? F("ready") : F("not ready"));
  if (sdReady && !loadCalibrated) {
    if (loadCalibration()) {
      Serial.println(F("[Cal] loaded from SD"));
    }
  }
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

// FAT 8.3: /YYYYMMDD/HHMMSS.CSV (same layout as Treadmill).
bool formatSessionPath(char *filePath, size_t fileLen, char *dirPath,
                       size_t dirLen) {
  char ts[24];
  rtc.formatLogTimestamp(ts, sizeof(ts));
  if (ts[0] == '\0') {
    filePath[0] = '\0';
    return false;
  }
  unsigned y, mo, d, h, mi, s;
  if (sscanf(ts, "%u-%u-%u %u:%u:%u", &y, &mo, &d, &h, &mi, &s) != 6) {
    filePath[0] = '\0';
    return false;
  }
  snprintf(dirPath, dirLen, "/%04u%02u%02u", y, mo, d);
  snprintf(filePath, fileLen, "%s/%02u%02u%02u.CSV", dirPath, h, mi, s);
  return true;
}

void formatSampleTimestamp(char *buf, size_t len, uint32_t sampleMs) {
  if (buf == nullptr || len == 0) {
    return;
  }
  const uint32_t elapsedSec = (sampleMs - logSessionStartMs) / 1000UL;
  const DateTime at = logSessionStartDt + TimeSpan((int32_t)elapsedSec);
  snprintf(buf, len, "%04u-%02u-%02u %02u:%02u:%02u", at.year(), at.month(),
           at.day(), at.hour(), at.minute(), at.second());
}

void holdMotorForLogFlush() {
  // Hold tension: stop pulses but keep the driver enabled (not released).
  targetSpsCmd = 0.0f;
  currentSps = 0.0f;
  applyStepRate(0.0f, true);
}

bool flushLogBuffer() {
  if (sessionLogPath[0] == '\0' || logCount == 0) {
    return true;
  }
  if (!sd.isReady()) {
    Serial.println(F("[Log] flush skipped (SD not ready)"));
    logCount = 0;
    return false;
  }

  const bool shouldHold =
      motorReady && currentScreen == SCREEN_RUN && runPhase != PHASE_FAULT &&
      stepper.isEnabled();

  logFlushActive = true;
  if (shouldHold) {
    holdMotorForLogFlush();
  }

  bool ok = true;
  for (uint16_t i = 0; i < logCount; i++) {
    char ts[24];
    formatSampleTimestamp(ts, sizeof(ts), logBuffer[i].ms);
    char line[72];
    snprintf(line, sizeof(line), "%s,measure,%d,%.1f", ts, logBuffer[i].targetG,
             (double)logBuffer[i].actualG);
    if (!sd.appendLine(sessionLogPath, line)) {
      ok = false;
      Serial.print(F("[Log] measure flush failed: "));
      Serial.println(sd.lastError());
      break;
    }
  }
  logCount = 0;

  logFlushActive = false;
  lastRampMs = millis();
  return ok;
}

void startSessionLog() {
  sessionLogPath[0] = '\0';
  logCount = 0;
  lastLogMs = 0;

  if (!sd.isReady() || !rtc.isReady()) {
    Serial.println(F("[Log] skipped (need SD + RTC)"));
    return;
  }

  char dirPath[12];
  if (!formatSessionPath(sessionLogPath, sizeof(sessionLogPath), dirPath,
                         sizeof(dirPath))) {
    Serial.println(F("[Log] begin failed (RTC timestamp)"));
    return;
  }

  char ts[24];
  rtc.formatLogTimestamp(ts, sizeof(ts));
  unsigned y, mo, d, h, mi, s;
  if (sscanf(ts, "%u-%u-%u %u:%u:%u", &y, &mo, &d, &h, &mi, &s) != 6) {
    sessionLogPath[0] = '\0';
    return;
  }
  logSessionStartDt = DateTime((uint16_t)y, (uint8_t)mo, (uint8_t)d, (uint8_t)h,
                               (uint8_t)mi, (uint8_t)s);
  logSessionStartMs = millis();

  char payload[128];
  const int n = snprintf(
      payload, sizeof(payload),
      "datetime,event,target_load_g,actual_load_g\n%s,begin,%d,%.1f\n", ts,
      targetGrams, (double)liveGrams);
  if (n <= 0 || (size_t)n >= sizeof(payload)) {
    sessionLogPath[0] = '\0';
    return;
  }

  bool ok = sd.ensureDirectory(dirPath);
  if (ok) {
    ok = sd.writeFile(sessionLogPath, (const uint8_t *)payload, (size_t)n,
                      false);
  }

  if (!ok) {
    Serial.print(F("[Log] begin failed ("));
    Serial.print(sessionLogPath);
    Serial.print(F("): "));
    Serial.println(sd.lastError());
    sessionLogPath[0] = '\0';
    return;
  }

  lastLogMs = logSessionStartMs;
  Serial.print(F("[Log] begin "));
  Serial.println(sessionLogPath);
}

void finishSessionLog(const char *event) {
  if (sessionLogPath[0] == '\0') {
    return;
  }

  (void)flushLogBuffer();

  if (!sd.isReady()) {
    sessionLogPath[0] = '\0';
    return;
  }

  char ts[24];
  rtc.formatLogTimestamp(ts, sizeof(ts));
  if (ts[0] == '\0') {
    formatSampleTimestamp(ts, sizeof(ts), millis());
  }

  char line[72];
  snprintf(line, sizeof(line), "%s,%s,%d,%.1f", ts, event, targetGrams,
           (double)liveGrams);

  const bool shouldHold =
      motorReady && currentScreen == SCREEN_RUN && runPhase != PHASE_FAULT &&
      stepper.isEnabled();
  logFlushActive = true;
  if (shouldHold) {
    holdMotorForLogFlush();
  }
  const bool ok = sd.appendLine(sessionLogPath, line);
  logFlushActive = false;

  if (!ok) {
    Serial.print(F("[Log] "));
    Serial.print(event);
    Serial.print(F(" failed: "));
    Serial.println(sd.lastError());
  } else {
    Serial.print(F("[Log] "));
    Serial.println(event);
  }

  sessionLogPath[0] = '\0';
  logCount = 0;
}

void serviceSessionLog() {
  if (currentScreen != SCREEN_RUN || sessionLogPath[0] == '\0' ||
      logFlushActive || runPhase == PHASE_FAULT) {
    return;
  }

  const uint32_t now = millis();
  if (lastLogMs != 0 && now - lastLogMs < (uint32_t)TRUSS_LOG_MS) {
    return;
  }
  lastLogMs = now;

  if (logCount >= TRUSS_LOG_BUFFER) {
    (void)flushLogBuffer();
  }

  if (logCount >= TRUSS_LOG_BUFFER) {
    return;
  }

  logBuffer[logCount].ms = now;
  logBuffer[logCount].targetG = targetGrams;
  logBuffer[logCount].actualG = liveGrams;
  logCount++;

  if (logCount >= TRUSS_LOG_BUFFER) {
    (void)flushLogBuffer();
  }
}

void enterHome() {
  if (currentScreen == SCREEN_RUN && runPhase != PHASE_FAULT) {
    finishSessionLog("end");
  }

  setMotorTickActive(false);
  applyStepRate(0.0f);
  stepper.disable();
  currentScreen = SCREEN_HOME;
  runPhase = PHASE_RUNNING;
  tensionState = TENSION_IDLE;
  targetSpsCmd = 0.0f;
  currentSps = 0.0f;
  displayDirty = true;
  updatePushLabel();
  if (!loadCellReady) {
    loadCellReady = scale.verify(500);
  }
  if (sd.isReady() && !loadCalibrated) {
    if (loadCalibration()) {
      Serial.println(F("[Cal] loaded from SD"));
    }
  }
  nav.enterScreen();
  Serial.println(F("[Home]"));
}

void enterCal() {
  if (!calAllowed()) {
    displayDirty = true;
    return;
  }

  currentScreen = SCREEN_CAL;
  calPhase = CAL_TARE;
  savedFlashUntilMs = 0;
  displayDirty = true;
  nav.enterScreen();
  Serial.println(F("[Cal] step 1: tare"));
}

void enterRun() {
  if (!runAllowed()) {
    displayDirty = true;
    return;
  }
  if (!loadCalibrated) {
    noCalFlashUntilMs = millis() + NOCAL_FLASH_MS;
    displayDirty = true;
    return;
  }

  // SD before motor timer — keep SPI free of step ISR contention.
  startSessionLog();

  currentScreen = SCREEN_RUN;
  runPhase = PHASE_RUNNING;
  tensionState = TENSION_IDLE;
  tensionStateSinceMs = millis();
  lastAbsErrForTimeout = 0.0f;
  stepsAtSettle = 0;
  targetSpsCmd = 0.0f;
  currentSps = 0.0f;
  loadSampleFresh = false;
  lastRampMs = millis();
  sessionElapsedMs = 0;
  sessionTickMs = millis();
  sessionStepCount = 0;
  displayDirty = true;
  updatePushLabel();

  if (motorReady) {
    stepper.enable();
    stepper.setMaxSpeed(TRUSS_SPS);
    stepper.setAcceleration(TRUSS_ACCEL_SPS2);
    setMotorTickActive(true);
    applyStepRate(0.0f);
  }

  nav.enterScreen();
  Serial.print(F("[Run] target "));
  Serial.print(targetGrams);
  Serial.println(F(" g"));
}

void tripFault(const char *reason) {
  // Hold, flush log as fault, then release the driver.
  targetSpsCmd = 0.0f;
  currentSps = 0.0f;
  applyStepRate(0.0f, true);
  finishSessionLog("fault");

  setMotorTickActive(false);
  applyStepRate(0.0f);
  stepper.disable();
  runPhase = PHASE_FAULT;
  displayDirty = true;
  Serial.print(F("[Fault] "));
  Serial.println(reason);
}

void serviceRun() {
  if (currentScreen != SCREEN_RUN) {
    return;
  }

  const uint32_t now = millis();

  if (runPhase != PHASE_HELD && runPhase != PHASE_FAULT) {
    sessionElapsedMs += now - sessionTickMs;
  }
  sessionTickMs = now;

  if (runPhase == PHASE_FAULT || logFlushActive) {
    return;
  }

  if (liveGrams > TRUSS_OVERLOAD_G) {
    tripFault("overload");
    return;
  }

  if (sessionStepCount - stepsAtSettle > (uint32_t)TRUSS_MAX_STEPS_UNSETTLED) {
    tripFault("step limit");
    return;
  }

  if (runPhase == PHASE_HELD) {
    tensionState = TENSION_IDLE;
    tensionStateSinceMs = now;
    stepsAtSettle = sessionStepCount;
    targetSpsCmd = 0.0f; // ramp decelerates to a stop
    return;
  }

  // Recompute motor command on each new load sample.
  if (!loadSampleFresh) {
    return;
  }
  loadSampleFresh = false;

  if (!motorReady) {
    return;
  }

  const float error = (float)targetGrams - liveGrams;
  const float absErr = fabsf(error);

  // Near target is the healthy regime: clear both fault budgets so a later
  // disturbance (animal lift) starts from a fresh timeout and step count.
  if (absErr < TRUSS_APPROACH_G) {
    tensionStateSinceMs = now;
    lastAbsErrForTimeout = absErr;
    stepsAtSettle = sessionStepCount;
  }

  // Hysteresis: stop inside the inner band; stay idle until the outer band.
  if (tensionState == TENSION_IDLE) {
    if (absErr <= TRUSS_START_G) {
      targetSpsCmd = 0.0f;
      return;
    }
  } else if (absErr < TRUSS_STOP_G) {
    tensionState = TENSION_IDLE;
    tensionStateSinceMs = now;
    targetSpsCmd = 0.0f;
    applyStepRate(0.0f, true);
    currentSps = 0.0f;
    return;
  }

  TensionState desired = (error > 0.0f) ? TENSION_WIND : TENSION_UNWIND;
  if (desired != tensionState) {
    tensionState = desired;
    tensionStateSinceMs = now;
    lastAbsErrForTimeout = absErr;
  } else if (absErr + 1.0f < lastAbsErrForTimeout) {
    // Closing on the target — keep the direction-timeout clock fresh.
    tensionStateSinceMs = now;
    lastAbsErrForTimeout = absErr;
  } else if (now - tensionStateSinceMs > (uint32_t)TRUSS_DIR_TIMEOUT_MS) {
    // Far from target and not making progress (slack / wrong sign).
    tripFault("direction timeout");
    return;
  }

  const int8_t windSign = (int8_t)TRUSS_WIND_SIGN;
  const int8_t dirSign = (error > 0.0f) ? windSign : (int8_t)(-windSign);

  // Very near target: a few steps per sample, then wait for the next reading.
  if (absErr < TRUSS_NUDGE_G) {
    nudgeSteps(dirSign);
    return;
  }

  // Mid band: continuous creep (faster recovery after a lift/drop than nudges).
  if (absErr < TRUSS_APPROACH_G) {
    targetSpsCmd = TRUSS_APPROACH_SPS * (float)dirSign;
    return;
  }

  float sps = TRUSS_GAIN_SPS_PER_G * absErr;
  if (sps > TRUSS_SPS) {
    sps = TRUSS_SPS;
  }
  targetSpsCmd = sps * (float)dirSign;
}

// Serial telemetry: one line every TRUSS_DEBUG_MS while on the Run screen.
// Example: [Run] load 84.2g tgt 200g err 115.8 WIND sps 2000/1360 steps 5120
//   load = filtered live load, err = target - load,
//   sps  = commanded/actual (commanded is the control output, actual is the
//          ramped speed driving the step timer; sign = direction after
//          TRUSS_WIND_SIGN), steps = pulses this session.
void serviceRunDebug() {
#if TRUSS_DEBUG_MS > 0
  static uint32_t lastDebugMs = 0;
  if (currentScreen != SCREEN_RUN) {
    return;
  }
  const uint32_t now = millis();
  if (now - lastDebugMs < (uint32_t)TRUSS_DEBUG_MS) {
    return;
  }
  lastDebugMs = now;

  Serial.print(F("[Run] load "));
  Serial.print(liveGrams, 1);
  Serial.print(F("g tgt "));
  Serial.print(targetGrams);
  Serial.print(F("g err "));
  Serial.print((float)targetGrams - liveGrams, 1);

  if (runPhase == PHASE_FAULT) {
    Serial.print(F(" FAULT"));
  } else if (runPhase == PHASE_HELD) {
    Serial.print(F(" HELD"));
  } else if (tensionState == TENSION_WIND) {
    Serial.print(F(" WIND"));
  } else if (tensionState == TENSION_UNWIND) {
    Serial.print(F(" UNWIND"));
  } else {
    Serial.print(F(" IDLE"));
  }

  Serial.print(F(" sps "));
  Serial.print(targetSpsCmd, 0);
  Serial.print(F("/"));
  Serial.print(currentSps, 0);
  Serial.print(F(" steps "));
  Serial.println(sessionStepCount);
#endif
}

// Chases targetSpsCmd with the accel ramp. Called every loop() pass so the
// speed changes in small, frequent increments (smooth), independent of the
// 10 Hz load-cell control update.
void serviceMotorRamp() {
  if (currentScreen != SCREEN_RUN || !motorReady || runPhase == PHASE_FAULT ||
      logFlushActive) {
    return;
  }

  const uint32_t now = millis();
  const uint32_t dtMs = now - lastRampMs;
  if (dtMs < TRUSS_RAMP_MS) {
    return;
  }
  lastRampMs = now;

  if (currentSps == targetSpsCmd) {
    return;
  }

  const float step = TRUSS_ACCEL_SPS2 * (float)dtMs / 1000.0f;
  if (targetSpsCmd > currentSps) {
    currentSps += step;
    if (currentSps > targetSpsCmd) {
      currentSps = targetSpsCmd;
    }
  } else {
    currentSps -= step;
    if (currentSps < targetSpsCmd) {
      currentSps = targetSpsCmd;
    }
  }

  applyStepRate(currentSps);
}

void runTareProcedure() {
  Serial.println(F("[Cal] taring..."));
  scale.tare(TRUSS_CAL_SAMPLES);
  calPhase = CAL_WEIGHT;
  displayDirty = true;
  Serial.print(F("[Cal] offset "));
  Serial.println(scale.get_offset());
}

void runCalibrationProcedure() {
  Serial.print(F("[Cal] calibrating with "));
  Serial.print(calGrams);
  Serial.println(F(" g"));

  scale.calibrate_scale((float)calGrams, TRUSS_CAL_SAMPLES);

  if (saveCalibration()) {
    calPhase = CAL_SAVED;
    savedFlashUntilMs = millis() + SAVED_FLASH_MS;
    Serial.println(F("[Cal] saved"));
  } else {
    Serial.println(F("[Cal] save failed"));
  }
  displayDirty = true;
}

void serviceCal() {
  if (currentScreen != SCREEN_CAL) {
    return;
  }
  if (calPhase == CAL_SAVED && savedFlashUntilMs != 0 &&
      (int32_t)(millis() - savedFlashUntilMs) >= 0) {
    calPhase = CAL_TARE;
    savedFlashUntilMs = 0;
    displayDirty = true;
  }
}

void handleHome(uint8_t ev, int32_t enc) {
  if (ev & STEPSCREEN_EVT_BACK) {
    if (calAllowed()) {
      enterCal();
    } else {
      displayDirty = true;
    }
    return;
  }
  if (ev & STEPSCREEN_EVT_CONFIRM) {
    enterRun();
    return;
  }
  if (enc != 0) {
    targetGrams += (int16_t)enc;
    targetGrams = constrain(targetGrams, TARGET_MIN_G, TARGET_MAX_G);
    updatePushLabel();
    displayDirty = true;
  }
}

void handleCal(uint8_t ev, int32_t enc) {
  if (ev & STEPSCREEN_EVT_BACK) {
    enterHome();
    return;
  }
  if (ev & STEPSCREEN_EVT_CONFIRM) {
    if (calPhase == CAL_TARE) {
      runTareProcedure();
    } else if (calPhase == CAL_WEIGHT) {
      runCalibrationProcedure();
    }
    return;
  }
  if (enc != 0 && calPhase == CAL_WEIGHT) {
    calGrams += (int16_t)enc;
    calGrams = constrain(calGrams, CAL_MIN_G, CAL_MAX_G);
    displayDirty = true;
  }
}

void handleRun(uint8_t ev, int32_t enc) {
  if (ev & STEPSCREEN_EVT_BACK) {
    enterHome();
    return;
  }
  if (ev & STEPSCREEN_EVT_PUSH && runPhase != PHASE_FAULT) {
    if (runPhase == PHASE_HELD) {
      runPhase = PHASE_RUNNING;
      sessionTickMs = millis();
      lastRampMs = millis();
      if (motorReady) {
        stepper.enable();
      }
    } else {
      runPhase = PHASE_HELD;
      applyStepRate(0.0f);
      currentSps = 0.0f;
    }
    displayDirty = true;
    return;
  }
  if (enc != 0 && runPhase != PHASE_FAULT) {
    targetGrams += (int16_t)enc;
    targetGrams = constrain(targetGrams, TARGET_MIN_G, TARGET_MAX_G);
    updatePushLabel();
    displayDirty = true;
  }
}

void drawHome(StepScreenDisplay &d) {
  char infoBar[12];
  rtc.formatInfoBar(infoBar, sizeof(infoBar));
  d.drawInfoBar(infoBar);
  drawHomeSdStatus(d);

  const StepScreenActionLabels labels = {
      calAllowed() ? "CAL" : nullptr, pushLabel,
      runAllowed() ? "RUN" : nullptr};
  d.drawActionColumn(labels, nav.readBack(), nav.readPush(),
                     nav.readConfirm());

  if (loadCellReady) {
    drawLargeGrams(d, 13, liveGrams, !loadCalibrated);
  } else {
    d.setTextSize(2);
    d.setCursor(2, 13);
    d.print(F("---"));
  }
  drawHomeHardwareFooter(d);
}

void drawCal(StepScreenDisplay &d) {
  d.drawInfoBar("Cal");
  const StepScreenActionLabels labels = {"BACK", nullptr,
                                         calPhase == CAL_SAVED ? nullptr : "RUN"};
  d.drawActionColumn(labels, nav.readBack(), false, nav.readConfirm());

  if (calPhase == CAL_SAVED) {
    const bool inv = (millis() / 200) & 1;
    if (inv) {
      d.fillRect(0, 13, STEPSCREEN_CONTENT_W - 4, 19, SH110X_WHITE);
      d.setTextColor(SH110X_BLACK);
    }
    d.setTextSize(2);
    d.setCursor(2, 15);
    d.print(F("SAVED"));
    d.setTextColor(SH110X_WHITE);
    return;
  }

  if (calPhase == CAL_TARE) {
    d.setTextSize(2);
    d.setCursor(2, 13);
    d.print(F("0g"));
    d.setTextSize(1);
    d.setCursor(2, 28);
    d.print(F("TARE"));
    d.setCursor(2, 40);
    d.print(F("remove weight"));
    return;
  }

  char buf[8];
  snprintf(buf, sizeof(buf), "%d", calGrams);
  d.setTextSize(2);
  d.setCursor(2, 13);
  d.print(buf);
  d.setTextSize(1);
  d.setCursor(d.getCursorX() + 3, 20);
  d.print(F("g"));
  d.setCursor(2, 40);
  d.print(F("hang weight"));
}

void drawRun(StepScreenDisplay &d) {
  d.drawInfoBar("Run");

  const char *holdLabel = (runPhase == PHASE_HELD) ? "GO" : "HOLD";
  const StepScreenActionLabels labels = {"END", holdLabel, nullptr};
  d.drawActionColumn(labels, nav.readBack(), nav.readPush(), false);

  drawLargeGrams(d, 13, liveGrams, false);

  char buf[16];
  snprintf(buf, sizeof(buf), "tgt %dg", targetGrams);
  d.setTextSize(1);
  d.setCursor(2, 34);
  d.print(buf);

  if (runPhase == PHASE_FAULT) {
    const bool inv = (millis() / 500) & 1;
    if (inv) {
      d.fillRect(0, 42, STEPSCREEN_CONTENT_W - 4, 18, SH110X_WHITE);
      d.setTextColor(SH110X_BLACK);
    }
    d.setTextSize(2);
    d.setCursor(2, 43);
    d.print(F("FAULT"));
    d.setTextColor(SH110X_WHITE);
  } else {
    const uint32_t totalSec = sessionElapsedMs / 1000UL;
    const uint16_t mm = totalSec / 60U;
    const uint8_t ss = totalSec % 60U;
    snprintf(buf, sizeof(buf), "%u:%02u", mm, ss);
    d.setTextSize(2);
    d.setCursor(2, 43);
    d.print(buf);
  }
}

void drawScreen() {
  static uint32_t lastDrawMs = 0;
  const bool activeLoop =
      currentScreen == SCREEN_RUN && runPhase == PHASE_RUNNING;
  const uint32_t interval =
      activeLoop ? DRAW_INTERVAL_RUNNING_MS : DRAW_INTERVAL_MS;
  const uint32_t now = millis();

  if (!displayDirty && (now - lastDrawMs < interval)) {
    return;
  }
  lastDrawMs = now;
  displayDirty = false;

  StepScreenDisplay &d = screen.display();
  d.clearDisplay();
  switch (currentScreen) {
  case SCREEN_HOME:
    drawHome(d);
    break;
  case SCREEN_CAL:
    drawCal(d);
    break;
  case SCREEN_RUN:
    drawRun(d);
    break;
  }
  d.display();
}

void haltHardwareWarning() {
  StepScreenDisplay &d = screen.display();
  d.clearDisplay();
  d.drawInfoBar("Error");
  const StepScreenActionLabels noActions = {nullptr, nullptr, nullptr};
  d.drawActionColumn(noActions, false, false, false);
  d.setTextSize(1);
  d.setCursor(STEPSCREEN_CONTENT_X, STEPSCREEN_CONTENT_Y + 4);
  d.println(F("Missing:"));
  if (!motorReady) {
    d.println(F(" 12V power"));
  }
  if (!loadCellReady) {
    d.println(F(" Load Cell"));
  }
  if (!sdReady) {
    d.println(F(" SD card"));
  }
  d.setCursor(STEPSCREEN_CONTENT_X, STEPSCREEN_CONTENT_Y + 40);
  d.println(F("Any btn skip"));
  d.display();

  Serial.println(F("Hardware check failed"));
  if (!motorReady) {
    Serial.println(F("  Stepper: no 12V / TMC"));
  }
  if (!loadCellReady) {
    Serial.println(F("  Load Cell: not responding"));
  }
  if (!sdReady) {
    Serial.println(F("  SD card: not ready"));
  }
  Serial.println(F("Any button to skip"));

  StepScreenInput &in = screen.input();
  while (true) {
    in.pollButtons();
    if (in.readBack() || in.readConfirm() || in.readPush()) {
      break;
    }
    delay(20);
  }
  while (in.readBack() || in.readConfirm() || in.readPush()) {
    in.pollButtons();
    delay(20);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (!screen.begin()) {
    Serial.println(F("TRUSS: display not found"));
    while (1) {
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

  scale.begin();
  loadCellReady = scale.verify();
  Serial.print(F("[Load Cell] "));
  Serial.println(loadCellReady ? F("online") : F("not detected"));

  nav.begin(&screen.input());

  tmc.begin();
  motorReady = tmc.verify() && tmc.configureForStepDir(
                                   STEPSCREEN_MOTOR_MICROSTEPS,
                                   STEPSCREEN_TMC2209_RUN_CURRENT_MA);
  if (motorReady) {
    tmc.driver().intpol(false);
  }
  stepper.begin();

#if defined(ARDUINO_ARCH_SAMD)
  initMotorTimer();
#else
  Serial.println(F("Warning: step-scheduled ISR requires SAMD21"));
#endif

  if (!motorReady || !loadCellReady || !sdReady) {
    haltHardwareWarning();
  }

  STEPSCREEN_ATTACH_ENCODER_ISRS();

  playBootLedDance();

  StepScreenSplashConfig splash;
  splash.theme = STEPSCREEN_SPLASH_TRUSS;
  splash.title = "TRUSS v1.0";
  splash.credit = "by the Neurotech Hub";
  StepScreenSplash::play(screen.display(), splash, &screen.input());

  sd.update();
  sdReady = sd.isReady();
  if (sdReady && loadCalibration()) {
    Serial.println(F("Loaded cal from SD"));
  } else if (sdReady) {
    Serial.println(F("No stored calibration"));
  }

  updatePushLabel();
  enterHome();
}

void loop() {
  if (currentScreen == SCREEN_HOME) {
    updateSdStatus();
    updateLoadCellDetect();
  }

  if (currentScreen == SCREEN_HOME || currentScreen == SCREEN_RUN) {
    readLiveLoad();
  }

  serviceRun();
  serviceMotorRamp();
  serviceSessionLog();
  serviceRunDebug();
  serviceCal();

#if !defined(ARDUINO_ARCH_SAMD)
  if (motorTickActive) {
    stepper.runSpeed();
  }
#endif

  const uint8_t ev = nav.pollNavEvents();
  const int32_t enc = nav.consumeEncoderDelta();

  switch (currentScreen) {
  case SCREEN_HOME:
    handleHome(ev, enc);
    break;
  case SCREEN_CAL:
    handleCal(ev, enc);
    break;
  case SCREEN_RUN:
    handleRun(ev, enc);
    break;
  }

  drawScreen();
  updateStatusLeds();
}
