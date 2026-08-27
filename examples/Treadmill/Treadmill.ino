/*
  StepScreen Treadmill

  Two-screen rodent treadmill controller built on StepScreenNav
  (menu-safe debouncing) and StepScreenStepper (AccelStepper/TMC2209).

  Home -- motor disabled. Shows speed (cm/s) and timeout (minutes);
          the metric being edited is drawn inverted. Info bar shows
          date/time plus SD (mounted) or xSD (absent/unready).
          Back = toggle edit target (label shows "Spd" or "Time"),
          Sel = toggle direction (Fwd/Rev), OK = Run.
          Encoder adjusts the selected metric.
          SD card detect/mount runs on Home only.
  Run  -- ramps the motor to the target speed, then counts down the
          timeout. Encoder adjusts speed live (smooth ramp). When the
          timeout expires the motor is disabled and DONE blinks
          (inverted) in the countdown area.
          Back = stop/disable motor, return Home.

  Session logging (SD + RTC): each Run creates /YYYYMMDD/HHMMSS.CSV using
  FAT 8.3-safe names (day folder + time file). Full timestamp is in CSV rows.

  Speed mapping: microsteps/sec = cm/s * STEPSCREEN_TREADMILL_SPS_PER_CMS
  (calibrated from belt measurement — see constant below).

  Stepping uses a step-scheduled TC3 ISR (one interrupt per pulse, minimal
  GPIO) so high belt speeds do not starve the UI loop.

  Board: Adafruit Feather M0 Adalogger (or compatible).

  Status LEDs: boot dance; off on Home; green on during Run; red blinks on DONE.
 */

// Shorter nav settle than library default (250 ms) — OK-to-Run feels snappier.
#ifndef STEPSCREEN_NAV_SETTLE_MS
#define STEPSCREEN_NAV_SETTLE_MS 100
#endif

#ifndef STEPSCREEN_MOTOR_MICROSTEPS
#define STEPSCREEN_MOTOR_MICROSTEPS 64
#endif

#ifndef STEPSCREEN_TMC2209_RUN_CURRENT_MA
#define STEPSCREEN_TMC2209_RUN_CURRENT_MA 800
#endif

#include <StepScreen.h>
#include <StepScreenNav.h>
#include <StepScreenRTC.h>
#include <StepScreenSD.h>
#include <StepScreenSplash.h>
#include <StepScreenStepper.h>
#include <StepScreenTMC2209.h>
#include <StepScreenBoard.h>

#include <stdio.h>

StepScreen screen;
StepScreenRTC rtc;
StepScreenSD sd;
StepScreenNav nav;
StepScreenTMC2209 tmc;
StepScreenStepper stepper;

bool motorReady = false;

char sessionLogPath[24] = {}; // "/YYYYMMDD/HHMMSS.CSV"
char runSdFooter[24] = {};    // bottom line on Run screen
int16_t sessionSpeedCms = 0;
int16_t sessionDurationMin = 0;

// microsteps/sec per cm/s of belt speed.
// Calibrated at 128 µsteps (1092.3); halved for 64 µsteps to preserve cm/s.
#ifndef STEPSCREEN_TREADMILL_SPS_PER_CMS
#define STEPSCREEN_TREADMILL_SPS_PER_CMS 546.15f
#endif

// microsteps/sec^2 — higher = snappier start and encoder response
#define TREADMILL_ACCEL_SPS2 20000.0f

// Adjustment ranges (encoder = +/-1 per detent)
#define SPEED_MIN_CMS 1
#define SPEED_MAX_CMS 50
#define TIME_MIN_MIN 1
#define TIME_MAX_MIN 180

#define DRAW_INTERVAL_MS 50
#define DRAW_INTERVAL_RUNNING_MS 100

enum TmScreen : uint8_t {
  SCREEN_HOME,
  SCREEN_RUN,
};

enum EditTarget : uint8_t {
  EDIT_SPEED,
  EDIT_TIME,
};

enum RunPhase : uint8_t {
  PHASE_RUNNING,
  PHASE_DONE,
};

TmScreen currentScreen = SCREEN_HOME;
EditTarget editTarget = EDIT_SPEED;
bool directionFwd = true;

int16_t speedCms = 10;   // belt speed setting
int16_t timeoutMin = 30; // treadmill timeout

RunPhase runPhase = PHASE_RUNNING;
uint32_t runEndMs = 0;
uint32_t lastRampMs = 0;
float currentSps = 0.0f; // ramped motor speed (signed microsteps/sec)
float targetSps = 0.0f;

volatile bool motorTickActive = false;
volatile uint32_t stepPeriodTicks = 0; // 48 MHz ticks between steps; 0 = hold
bool displayDirty = true;

#if defined(ARDUINO_ARCH_SAMD)
#define MOTOR_GCLK_HZ 48000000UL
#define STEP_PULSE_US 2 // TMC2209 STEP high time; avoid zero-width toggles
// Ignore tiny period tweaks during ramp (avoids CC glitches mid-run).
#define MOTOR_PERIOD_UPDATE_PCT 2u

void motorTimerStop() {
  TC3->COUNT32.CTRLA.bit.ENABLE = 0;
  while (TC3->COUNT32.STATUS.bit.SYNCBUSY) {
  }
  TC3->COUNT32.INTFLAG.reg = TC_INTFLAG_MC0;
}

void motorTimerApplyPeriod(uint32_t period) {
  noInterrupts();
  motorTimerStop();
  stepPeriodTicks = period;
  TC3->COUNT32.CC[0].reg = period - 1;
  while (TC3->COUNT32.STATUS.bit.SYNCBUSY) {
  }
  TC3->COUNT32.COUNT.reg = 0;
  while (TC3->COUNT32.STATUS.bit.SYNCBUSY) {
  }
  TC3->COUNT32.INTFLAG.reg = TC_INTFLAG_MC0;
  TC3->COUNT32.CTRLA.bit.ENABLE = 1;
  while (TC3->COUNT32.STATUS.bit.SYNCBUSY) {
  }
  interrupts();
}

void initMotorTimer() {
  GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 |
                                 GCLK_CLKCTRL_ID_TCC2_TC3);
  while (GCLK->STATUS.bit.SYNCBUSY) {
  }

  TC3->COUNT32.CTRLA.bit.ENABLE = 0;
  while (TC3->COUNT32.STATUS.bit.SYNCBUSY) {
  }

  TC3->COUNT32.CTRLA.reg = TC_CTRLA_MODE_COUNT32 | TC_CTRLA_WAVEGEN_MFRQ |
                             TC_CTRLA_PRESCALER_DIV1;
  TC3->COUNT32.INTENSET.reg = TC_INTENSET_MC0;
  TC3->COUNT32.INTFLAG.reg = TC_INTFLAG_MC0;
  NVIC_EnableIRQ(TC3_IRQn);
}

void isrStepPulse() {
  digitalWrite(PIN_MOTOR_STEP, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(PIN_MOTOR_STEP, LOW);
}

void TC3_Handler() {
  if (TC3->COUNT32.INTFLAG.bit.MC0 == 0) {
    return;
  }
  TC3->COUNT32.INTFLAG.reg = TC_INTFLAG_MC0;
  if (!motorTickActive || stepPeriodTicks == 0) {
    return;
  }
  isrStepPulse();
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

void applyStepRate(float sps, bool force) {
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

  uint32_t period = (uint32_t)(MOTOR_GCLK_HZ / absSps);
  if (period < 2) {
    period = 2;
  }

  if (!force && stepPeriodTicks > 0 && TC3->COUNT32.CTRLA.bit.ENABLE) {
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

void updateTargetSps() {
  targetSps = (float)speedCms * STEPSCREEN_TREADMILL_SPS_PER_CMS;
  if (!directionFwd) {
    targetSps = -targetSps;
  }
}

// --- Status LEDs (PIN_LED_GREEN / PIN_LED_RED) ---

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
  if (currentScreen == SCREEN_HOME) {
    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_LED_RED, LOW);
    return;
  }

  if (runPhase == PHASE_RUNNING) {
    digitalWrite(PIN_LED_GREEN, HIGH);
    digitalWrite(PIN_LED_RED, LOW);
    return;
  }

  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_RED, (millis() / 500) & 1 ? HIGH : LOW);
}

// --- SD session logging ---

void formatHomeInfoBar(char *buf, size_t len) {
  char timePart[12];
  rtc.formatInfoBar(timePart, sizeof(timePart));
  snprintf(buf, len, "%s%s", timePart, sd.isReady() ? " SD" : " xSD");
}

// FAT 8.3: /YYYYMMDD/HHMMSS.CSV (8-char dir + 6-char file + .CSV).
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

void startSessionLog() {
  if (!sd.isReady() || !rtc.isReady()) {
    return;
  }

  char dirPath[12];
  if (!formatSessionPath(sessionLogPath, sizeof(sessionLogPath), dirPath,
                         sizeof(dirPath))) {
    return;
  }

  sessionSpeedCms = speedCms;
  sessionDurationMin = timeoutMin;

  char ts[24];
  rtc.formatLogTimestamp(ts, sizeof(ts));
  char payload[80];
  const int n = snprintf(payload, sizeof(payload),
                         "datetime,event,speed,duration\n%s,begin,%d,%d\n", ts,
                         sessionSpeedCms, sessionDurationMin);
  if (n <= 0 || (size_t)n >= sizeof(payload)) {
    sessionLogPath[0] = '\0';
    return;
  }

  noInterrupts();
  bool ok = sd.ensureDirectory(dirPath);
  if (ok) {
    ok = sd.writeFile(sessionLogPath, (const uint8_t *)payload, (size_t)n,
                      false);
  }
  interrupts();

  if (!ok) {
    Serial.print(F("[Log] begin failed ("));
    Serial.print(sessionLogPath);
    Serial.print(F("): "));
    Serial.println(sd.lastError());
    sessionLogPath[0] = '\0';
    return;
  }

  Serial.print(F("[Log] begin "));
  Serial.println(sessionLogPath);
}

void finishSessionLog(const char *event) {
  if (sessionLogPath[0] == '\0') {
    return;
  }

  if (!sd.isReady() || !rtc.isReady()) {
    sessionLogPath[0] = '\0';
    return;
  }

  char ts[24];
  rtc.formatLogTimestamp(ts, sizeof(ts));
  char line[64];
  snprintf(line, sizeof(line), "%s,%s,%d,%d", ts, event, sessionSpeedCms,
           sessionDurationMin);

  noInterrupts();
  const bool ok = sd.appendLine(sessionLogPath, line);
  interrupts();

  if (!ok) {
    Serial.print(F("[Log] "));
    Serial.print(event);
    Serial.print(F(" failed ("));
    Serial.print(sessionLogPath);
    Serial.print(F("): "));
    Serial.println(sd.lastError());
  } else {
    Serial.print(F("[Log] "));
    Serial.println(event);
  }

  sessionLogPath[0] = '\0';
}

void updateRunSdFooter() {
  if (sessionLogPath[0] != '\0') {
    snprintf(runSdFooter, sizeof(runSdFooter), "%s", sessionLogPath + 1);
    return;
  }
  if (!sd.cardPresent()) {
    snprintf(runSdFooter, sizeof(runSdFooter), "No SD card");
    return;
  }
  if (sd.mountState() == STEPSCREEN_SD_ERROR) {
    snprintf(runSdFooter, sizeof(runSdFooter), "SD mount failed");
    return;
  }
  if (!sd.isReady()) {
    snprintf(runSdFooter, sizeof(runSdFooter), "SD not ready");
    return;
  }
  snprintf(runSdFooter, sizeof(runSdFooter), "Log write failed");
}

// --- Screen transitions (always via these, so nav debounce applies) ---

void enterHome() {
  setMotorTickActive(false);
  stepper.disable();
  finishSessionLog("cancelled");
  currentScreen = SCREEN_HOME;
  displayDirty = true;
  nav.enterScreen();
  Serial.println(F("[Home]"));
}

void enterRun() {
  startSessionLog(); // SD before motor timer — TC3 ISR starves SPI
  updateRunSdFooter();

  currentScreen = SCREEN_RUN;
  runPhase = PHASE_RUNNING;

  updateTargetSps();
  currentSps = 0.0f;
  lastRampMs = millis();
  runEndMs = millis() + (uint32_t)timeoutMin * 60000UL;
  displayDirty = true;

  stepper.enable();
  setMotorTickActive(true);
  applyStepRate(0.0f);
  nav.enterScreen();

  Serial.print(F("[Run] "));
  Serial.print(speedCms);
  Serial.print(F(" cm/s "));
  Serial.print(directionFwd ? F("Fwd") : F("Rev"));
  Serial.print(F(" for "));
  Serial.print(timeoutMin);
  Serial.println(F(" min"));
}

// --- Run screen motor service (ramp + countdown; stepping is in ISR) ---

void serviceRun() {
  if (currentScreen != SCREEN_RUN || runPhase != PHASE_RUNNING || !motorReady) {
    return;
  }

  const uint32_t now = millis();
  const float prevSps = currentSps;

  if (currentSps != targetSps) {
    const float step =
        TREADMILL_ACCEL_SPS2 * (float)(now - lastRampMs) / 1000.0f;
    if (targetSps > currentSps) {
      currentSps = currentSps + step;
      if (currentSps > targetSps) {
        currentSps = targetSps;
      }
    } else {
      currentSps = currentSps - step;
      if (currentSps < targetSps) {
        currentSps = targetSps;
      }
    }
  }
  lastRampMs = now;

  if (currentSps != prevSps) {
    applyStepRate(currentSps);
  }
  // Exact period when ramp lands on target (threshold may have skipped last tick).
  if (currentSps == targetSps && prevSps != targetSps) {
    applyStepRate(currentSps, true);
  }

  if ((int32_t)(now - runEndMs) >= 0) {
    setMotorTickActive(false);
    stepper.disable();
    finishSessionLog("end");
    runPhase = PHASE_DONE;
    displayDirty = true;
    Serial.println(F("[Run] done"));
  }
}

// --- Per-screen input handling ---

void handleHome(uint8_t ev, int32_t enc) {
  if (ev & STEPSCREEN_EVT_BACK) {
    editTarget = (editTarget == EDIT_SPEED) ? EDIT_TIME : EDIT_SPEED;
    displayDirty = true;
  }
  if (ev & STEPSCREEN_EVT_PUSH) {
    directionFwd = !directionFwd;
    displayDirty = true;
  }
  if (ev & STEPSCREEN_EVT_CONFIRM) {
    enterRun();
    return;
  }
  if (enc != 0) {
    if (editTarget == EDIT_SPEED) {
      speedCms += (int16_t)enc;
      speedCms = constrain(speedCms, SPEED_MIN_CMS, SPEED_MAX_CMS);
    } else {
      timeoutMin += (int16_t)enc;
      timeoutMin = constrain(timeoutMin, TIME_MIN_MIN, TIME_MAX_MIN);
    }
    displayDirty = true;
  }
}

void handleRun(uint8_t ev, int32_t enc) {
  if (ev & STEPSCREEN_EVT_BACK) {
    enterHome();
    return;
  }
  if (enc != 0 && runPhase == PHASE_RUNNING) {
    speedCms += (int16_t)enc;
    speedCms = constrain(speedCms, SPEED_MIN_CMS, SPEED_MAX_CMS);
    updateTargetSps();
    displayDirty = true;
    Serial.print(F("[Run] speed -> "));
    Serial.print(speedCms);
    Serial.println(F(" cm/s"));
  }
}

// --- Drawing ---

void drawMetric(StepScreenDisplay &d, int16_t y, const char *value,
                const char *unit, bool selected) {
  const int16_t rowW = STEPSCREEN_CONTENT_W - 4;
  if (selected) {
    d.fillRect(0, y - 2, rowW, 19, SH110X_WHITE);
  }
  const uint16_t fg = selected ? SH110X_BLACK : SH110X_WHITE;

  d.setTextSize(2);
  d.setTextColor(fg);
  d.setCursor(2, y);
  d.print(value);

  d.setTextSize(1);
  d.setCursor(d.getCursorX() + 3, y + 7);
  d.print(unit);
  d.setTextColor(SH110X_WHITE);
}

void drawHome(StepScreenDisplay &d) {
  char infoBar[16];
  formatHomeInfoBar(infoBar, sizeof(infoBar));
  d.drawInfoBar(infoBar);
  const StepScreenActionLabels labels = {
      editTarget == EDIT_SPEED ? "Spd" : "Time", "Dir", "Run"};
  d.drawActionColumn(labels, nav.readBack(), nav.readPush(),
                     nav.readConfirm());

  char buf[8];
  snprintf(buf, sizeof(buf), "%d", speedCms);
  drawMetric(d, 13, buf, "cm/s", editTarget == EDIT_SPEED);

  snprintf(buf, sizeof(buf), "%d", timeoutMin);
  drawMetric(d, 33, buf, "minutes", editTarget == EDIT_TIME);

  d.setTextSize(1);
  d.setCursor(2, 54);
  d.print("Dir: ");
  d.print(directionFwd ? "Fwd" : "Rev");
}

void drawRun(StepScreenDisplay &d) {
  d.drawInfoBar("Run");
  const StepScreenActionLabels labels = {"Back", "Spd", nullptr};
  d.drawActionColumn(labels, nav.readBack(), false, false);

  const int16_t liveCms = speedCms;
  char buf[12];
  snprintf(buf, sizeof(buf), "%d", liveCms);
  d.setTextSize(2);
  d.setCursor(2, 13);
  d.print(buf);
  d.setTextSize(1);
  d.setCursor(d.getCursorX() + 3, 20);
  d.print("cm/s ");
  d.print(directionFwd ? F("Fwd") : F("Rev"));

  if (runPhase == PHASE_DONE) {
    const bool inv = (millis() / 500) & 1;
    if (inv) {
      d.fillRect(0, 34, STEPSCREEN_CONTENT_W - 4, 19, SH110X_WHITE);
      d.setTextColor(SH110X_BLACK);
    }
    d.setTextSize(2);
    d.setCursor(2, 36);
    d.print("DONE");
    d.setTextColor(SH110X_WHITE);
  } else {
    const uint32_t now = millis();
    const uint32_t remMs = (int32_t)(runEndMs - now) > 0 ? runEndMs - now : 0;
    const uint16_t mm = remMs / 60000UL;
    const uint8_t ss = (remMs % 60000UL) / 1000UL;
    snprintf(buf, sizeof(buf), "%u:%02u", mm, ss);
    d.setTextSize(2);
    d.setCursor(2, 36);
    d.print(buf);
  }

  d.setTextSize(1);
  d.setCursor(2, 56);
  d.print(runSdFooter);
}

void drawScreen() {
  static uint32_t lastDrawMs = 0;
  const bool beltRunning =
      currentScreen == SCREEN_RUN && runPhase == PHASE_RUNNING;
  const uint32_t interval =
      beltRunning ? DRAW_INTERVAL_RUNNING_MS : DRAW_INTERVAL_MS;
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
  case SCREEN_RUN:
    drawRun(d);
    break;
  }
  d.display();
}

void haltMotorDriverOffline() {
  StepScreenDisplay &d = screen.display();
  d.clearDisplay();
  d.drawInfoBar("Error");
  d.drawActionColumn(false, false, false);
  d.setTextSize(1);
  d.setCursor(STEPSCREEN_CONTENT_X, STEPSCREEN_CONTENT_Y + 8);
  d.println(F("Motor Driver Offline"));
  d.println();
  d.println(F("Apply 12V and Reset"));
  d.display();

  Serial.println(F("Motor Driver Offline"));
  Serial.println(F("Apply 12V and Reset"));

  while (true) {
    delay(250);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000); // let USB serial settle; no while(!Serial) so battery works

  if (!screen.begin()) {
    Serial.println(F("Treadmill: display not found"));
    while (1) {
      delay(100);
    }
  }

  rtc.begin();
  rtc.syncFromBuildTimeIfNeeded();

  sd.begin();
  sd.setActivityLed(false);

  nav.begin(&screen.input());

  tmc.begin();
  if (!tmc.verify()) {
    haltMotorDriverOffline();
  }
  // Pass microsteps explicitly — sketch #defines are not visible inside
  // StepScreenStepper.cpp's beginWithDriver() default argument.
  if (!tmc.configureForStepDir(STEPSCREEN_MOTOR_MICROSTEPS,
                               STEPSCREEN_TMC2209_RUN_CURRENT_MA)) {
    haltMotorDriverOffline();
  }
  tmc.driver().intpol(false);
  stepper.begin();
  motorReady = true;

#if defined(ARDUINO_ARCH_SAMD)
  initMotorTimer();
#else
  Serial.println(F("Warning: step-scheduled ISR requires SAMD21"));
#endif

  // --- STEPSCREEN ENCODER ISR SETUP (copy into setup()) ---
  STEPSCREEN_ATTACH_ENCODER_ISRS();
  // --- END STEPSCREEN ENCODER ISR SETUP ---

  playBootLedDance();

  StepScreenSplashConfig splash;
  splash.theme = STEPSCREEN_SPLASH_TREADMILL;
  splash.title = "Treadmill v1.0";
  splash.credit = "by the Neurotech Hub";
  StepScreenSplash::play(screen.display(), splash, &screen.input());

  enterHome();
}

void loop() {
  if (currentScreen == SCREEN_HOME) {
    sd.update();
  }

  serviceRun(); // speed ramp + timeout (stepping in TC3 ISR)

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
  case SCREEN_RUN:
    handleRun(ev, enc);
    break;
  }

  drawScreen();
  updateStatusLeds();
}
