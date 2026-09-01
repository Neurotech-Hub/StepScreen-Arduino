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

  Calibration (Cal screen):
    1. Remove hanging mass; enter Cal (auto-tare).
    2. Hang a known mass; set grams with encoder (50-500, default 200).
    3. Press RUN to calibrate and save (EEPROM, SD /TRUSS.CAL fallback).
    4. Repeat or BACK to Home.

  Control tuning (sketch #defines below):
    TRUSS_KP, TRUSS_KI, TRUSS_DEADBAND_G, TRUSS_MAX_SPS, TRUSS_ACCEL_SPS2,
    TRUSS_LOOP_MS, TRUSS_WIND_SIGN (+1 if winding increases load),
    TRUSS_OVERLOAD_G, TRUSS_MAX_STEPS_SESSION

  Board: Adafruit Feather M0 Adalogger (or compatible SAMD21).

  Requires FlashStorage_SAMD (Library Manager). StepScreenStore includes it
  internally — do not #include <FlashStorage_SAMD.h> in this sketch.
  Without flash storage, calibration falls back to /TRUSS.CAL on the SD card.
 */

#ifndef STEPSCREEN_NAV_SETTLE_MS
#define STEPSCREEN_NAV_SETTLE_MS 100
#endif

#ifndef STEPSCREEN_MOTOR_MICROSTEPS
#define STEPSCREEN_MOTOR_MICROSTEPS 32
#endif

#ifndef STEPSCREEN_TMC2209_RUN_CURRENT_MA
#define STEPSCREEN_TMC2209_RUN_CURRENT_MA 500
#endif

// --- Control tuning ---
#ifndef TRUSS_KP
#define TRUSS_KP 8.0f
#endif

#ifndef TRUSS_KI
#define TRUSS_KI 0.4f
#endif

#ifndef TRUSS_DEADBAND_G
#define TRUSS_DEADBAND_G 2.0f
#endif

#ifndef TRUSS_MAX_SPS
#define TRUSS_MAX_SPS 400.0f
#endif

#ifndef TRUSS_ACCEL_SPS2
#define TRUSS_ACCEL_SPS2 800.0f
#endif

#ifndef TRUSS_LOOP_MS
#define TRUSS_LOOP_MS 100
#endif

#ifndef TRUSS_WIND_SIGN
#define TRUSS_WIND_SIGN 1
#endif

#ifndef TRUSS_OVERLOAD_G
#define TRUSS_OVERLOAD_G 600.0f
#endif

#ifndef TRUSS_MAX_STEPS_SESSION
#define TRUSS_MAX_STEPS_SESSION 50000L
#endif

#ifndef TRUSS_CAL_SAMPLES
#define TRUSS_CAL_SAMPLES 20
#endif

#define TARGET_MIN_G 0
#define TARGET_MAX_G 500
#define CAL_MIN_G 50
#define CAL_MAX_G 500

#define DRAW_INTERVAL_MS 50
#define DRAW_INTERVAL_RUNNING_MS 100
#define SAVED_FLASH_MS 1000
#define NOCAL_FLASH_MS 800

#define TRUSS_CAL_MAGIC 0x54525353UL // 'TRSS'
#define TRUSS_SD_CAL_PATH "/TRUSS.CAL"

#include <StepScreen.h>
#include <StepScreenHX711.h>
#include <StepScreenNav.h>
#include <StepScreenRTC.h>
#include <StepScreenSD.h>
#include <StepScreenSplash.h>
#include <StepScreenStepper.h>
#include <StepScreenStore.h>
#include <StepScreenTMC2209.h>

#include <math.h>
#include <stdio.h>

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
StepScreenStore store;

bool motorReady = false;
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

TrussScreen currentScreen = SCREEN_HOME;
RunPhase runPhase = PHASE_RUNNING;

int16_t targetGrams = 200;
int16_t calGrams = 200;
float liveGrams = 0.0f;

char pushLabel[6] = "200g";

uint32_t savedFlashUntilMs = 0;
uint32_t noCalFlashUntilMs = 0;

bool displayDirty = true;

// Closed-loop state
float currentSps = 0.0f;
float integralTerm = 0.0f;
uint32_t lastControlMs = 0;
uint32_t lastRampMs = 0;
int32_t sessionStartPos = 0;

// Session timer (pauses while held)
uint32_t sessionElapsedMs = 0;
uint32_t sessionTickMs = 0;

void applyStepRate(float sps);
void drawHomeSdStatus(StepScreenDisplay &d);
void drawLargeGrams(StepScreenDisplay &d, int16_t y, float grams,
                    bool showNoCal);
void updatePushLabel();
bool loadCalibration();
bool saveCalibration();
void enterHome();
void enterCal();
void enterRun();
void serviceRun();

void applyStepRate(float sps) {
  if (!motorReady || runPhase == PHASE_FAULT) {
    stepper.setSpeed(0.0f);
    return;
  }
  stepper.setSpeed(sps);
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

bool loadCalFromSd(TrussCalData &data) {
  if (!sd.isReady()) {
    return false;
  }

  char line[64];
  if (!sd.readLine(TRUSS_SD_CAL_PATH, line, sizeof(line))) {
    return false;
  }

  int32_t offset = 0;
  float scaleVal = 0.0f;
  if (sscanf(line, "O=%ld,S=%f", (long *)&offset, &scaleVal) != 2) {
    return false;
  }

  data.offset = offset;
  data.scale = scaleVal;
  data.calibrated = 1;
  return true;
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
  if (store.available() && store.load(TRUSS_CAL_MAGIC, &data, sizeof(data))) {
    applyCalibration(data);
    return loadCalibrated;
  }
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

  bool ok = false;
  if (store.available()) {
    ok = store.save(TRUSS_CAL_MAGIC, &data, sizeof(data));
  }
  if (!ok) {
    ok = saveCalToSd(data);
  } else {
    saveCalToSd(data);
  }

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
  if (scale.is_ready()) {
    liveGrams = scale.get_units(3);
  }
}

void enterHome() {
  stepper.setSpeed(0.0f);
  stepper.disable();
  currentScreen = SCREEN_HOME;
  runPhase = PHASE_RUNNING;
  integralTerm = 0.0f;
  currentSps = 0.0f;
  displayDirty = true;
  updatePushLabel();
  nav.enterScreen();
  Serial.println(F("[Home]"));
}

void enterCal() {
  currentScreen = SCREEN_CAL;
  displayDirty = true;
  nav.enterScreen();

  Serial.println(F("[Cal] taring..."));
  scale.tare(TRUSS_CAL_SAMPLES);
  Serial.print(F("[Cal] offset "));
  Serial.println(scale.get_offset());
}

void enterRun() {
  if (!loadCalibrated) {
    noCalFlashUntilMs = millis() + NOCAL_FLASH_MS;
    displayDirty = true;
    return;
  }

  currentScreen = SCREEN_RUN;
  runPhase = PHASE_RUNNING;
  integralTerm = 0.0f;
  currentSps = 0.0f;
  lastControlMs = 0;
  lastRampMs = millis();
  sessionElapsedMs = 0;
  sessionTickMs = millis();
  sessionStartPos = stepper.currentPosition();
  displayDirty = true;
  updatePushLabel();

  if (motorReady) {
    stepper.enable();
    stepper.setMaxSpeed(TRUSS_MAX_SPS);
    stepper.setAcceleration(TRUSS_ACCEL_SPS2);
    applyStepRate(0.0f);
  }

  nav.enterScreen();
  Serial.print(F("[Run] target "));
  Serial.print(targetGrams);
  Serial.println(F(" g"));
}

void tripFault(const char *reason) {
  stepper.setSpeed(0.0f);
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

  if (now - lastControlMs < TRUSS_LOOP_MS) {
    return;
  }
  lastControlMs = now;

  readLiveLoad();

  if (runPhase == PHASE_FAULT) {
    return;
  }

  if (liveGrams > TRUSS_OVERLOAD_G) {
    tripFault("overload");
    return;
  }

  int32_t stepsMoved = stepper.currentPosition() - sessionStartPos;
  if (stepsMoved < 0) {
    stepsMoved = -stepsMoved;
  }
  if (stepsMoved > TRUSS_MAX_STEPS_SESSION) {
    tripFault("step limit");
    return;
  }

  if (runPhase == PHASE_HELD) {
    applyStepRate(0.0f);
    return;
  }

  if (!motorReady) {
    return;
  }

  const float error = (float)targetGrams - liveGrams;
  float targetSps = 0.0f;

  if (fabsf(error) >= TRUSS_DEADBAND_G) {
    integralTerm += error * ((float)TRUSS_LOOP_MS / 1000.0f);
    const float iMax = TRUSS_MAX_SPS / fmaxf(TRUSS_KI, 0.001f);
    integralTerm = constrain(integralTerm, -iMax, iMax);
    targetSps = TRUSS_KP * error + TRUSS_KI * integralTerm;
    targetSps = constrain(targetSps, -TRUSS_MAX_SPS, TRUSS_MAX_SPS);
  } else {
    integralTerm = 0.0f;
  }

  targetSps *= (float)TRUSS_WIND_SIGN;

  const float step =
      TRUSS_ACCEL_SPS2 * (float)(now - lastRampMs) / 1000.0f;
  lastRampMs = now;

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

  applyStepRate(currentSps);
  displayDirty = true;
}

void runCalibrationProcedure() {
  Serial.print(F("[Cal] calibrating with "));
  Serial.print(calGrams);
  Serial.println(F(" g"));

  scale.calibrate_scale((float)calGrams, TRUSS_CAL_SAMPLES);

  if (saveCalibration()) {
    savedFlashUntilMs = millis() + SAVED_FLASH_MS;
    Serial.println(F("[Cal] saved"));
  } else {
    Serial.println(F("[Cal] save failed"));
  }
  displayDirty = true;
}

void handleHome(uint8_t ev, int32_t enc) {
  if (ev & STEPSCREEN_EVT_BACK) {
    enterCal();
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
    runCalibrationProcedure();
    return;
  }
  if (enc != 0) {
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

  const StepScreenActionLabels labels = {"CAL", pushLabel, "RUN"};
  d.drawActionColumn(labels, nav.readBack(), nav.readPush(),
                     nav.readConfirm());

  drawLargeGrams(d, 13, liveGrams, !loadCalibrated);
}

void drawCal(StepScreenDisplay &d) {
  d.drawInfoBar("Cal");
  const StepScreenActionLabels labels = {"BACK", nullptr, "RUN"};
  d.drawActionColumn(labels, nav.readBack(), false, nav.readConfirm());

  char buf[8];
  snprintf(buf, sizeof(buf), "%d", calGrams);
  d.setTextSize(2);
  d.setCursor(2, 13);
  d.print(buf);
  d.setTextSize(1);
  d.setCursor(d.getCursorX() + 3, 20);
  d.print(F("g"));

  d.setTextSize(1);
  d.setCursor(2, 40);
  d.print(F("hang weight"));

  if (savedFlashUntilMs != 0 && (int32_t)(millis() - savedFlashUntilMs) < 0) {
    const bool inv = (millis() / 200) & 1;
    if (inv) {
      d.fillRect(0, 48, STEPSCREEN_CONTENT_W - 4, 11, SH110X_WHITE);
      d.setTextColor(SH110X_BLACK);
    }
    d.setTextSize(1);
    d.setCursor(2, 50);
    d.print(F("SAVED"));
    d.setTextColor(SH110X_WHITE);
  }
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
      d.fillRect(0, 40, STEPSCREEN_CONTENT_W - 4, 19, SH110X_WHITE);
      d.setTextColor(SH110X_BLACK);
    }
    d.setTextSize(2);
    d.setCursor(2, 42);
    d.print(F("FAULT"));
    d.setTextColor(SH110X_WHITE);
  } else {
    const uint32_t totalSec = sessionElapsedMs / 1000UL;
    const uint16_t mm = totalSec / 60U;
    const uint8_t ss = totalSec % 60U;
    snprintf(buf, sizeof(buf), "%u:%02u", mm, ss);
    d.setTextSize(2);
    d.setCursor(2, 42);
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

  if (currentScreen == SCREEN_HOME || currentScreen == SCREEN_RUN) {
    readLiveLoad();
  }

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

void haltMotorDriverOffline() {
  StepScreenDisplay &d = screen.display();
  d.clearDisplay();
  d.drawInfoBar("Error");
  const StepScreenActionLabels noActions = {nullptr, nullptr, nullptr};
  d.drawActionColumn(noActions, false, false, false);
  d.setTextSize(1);
  d.setCursor(STEPSCREEN_CONTENT_X, STEPSCREEN_CONTENT_Y + 8);
  d.println(F("Motor Driver Offline"));
  d.println();
  d.println(F("Apply 12V and Reset"));
  d.println();
  d.println(F("Any button to skip"));
  d.display();

  Serial.println(F("Motor Driver Offline"));
  Serial.println(F("Apply 12V and Reset"));
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

  store.begin();
  scale.begin();

  nav.begin(&screen.input());

  tmc.begin();
  motorReady = tmc.verify() && tmc.configureForStepDir(
                                   STEPSCREEN_MOTOR_MICROSTEPS,
                                   STEPSCREEN_TMC2209_RUN_CURRENT_MA);
  if (motorReady) {
    tmc.driver().intpol(false);
  } else {
    haltMotorDriverOffline();
  }
  stepper.begin();

  STEPSCREEN_ATTACH_ENCODER_ISRS();

  playBootLedDance();

  if (!loadCalibration()) {
    Serial.println(F("No stored calibration"));
  } else {
    Serial.print(F("Loaded cal offset="));
    Serial.print(scale.get_offset());
    Serial.print(F(" scale="));
    Serial.println(scale.get_scale(), 6);
  }

  StepScreenSplashConfig splash;
  splash.theme = STEPSCREEN_SPLASH_TRUSS;
  splash.title = "TRUSS v1.0";
  splash.credit = "by the Neurotech Hub";
  StepScreenSplash::play(screen.display(), splash, &screen.input());

  updatePushLabel();
  enterHome();
}

void loop() {
  if (currentScreen == SCREEN_HOME) {
    sd.update();
  }

  serviceRun();

  if (motorReady && currentScreen == SCREEN_RUN && runPhase == PHASE_RUNNING) {
    stepper.runSpeed();
  }

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
