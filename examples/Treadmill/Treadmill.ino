/*
  StepScreen Treadmill

  Two-screen rodent treadmill controller built on StepScreenNav
  (menu-safe debouncing) and StepScreenStepper (AccelStepper/TMC2209).

  Home -- motor disabled. Shows speed (cm/s) and timeout (minutes);
          the metric being edited is drawn inverted.
          Back = toggle edit target (label shows "Spd" or "Time"),
          Sel = toggle direction (Fwd/Rev), OK = Run.
          Encoder adjusts the selected metric.
  Run  -- ramps the motor to the target speed, then counts down the
          timeout. When it expires the motor is disabled and DONE
          blinks (inverted) in the countdown area.
          Back = stop/disable motor, return Home.

  Speed mapping is a PLACEHOLDER: microsteps/sec = cm/s *
  STEPSCREEN_TREADMILL_SPS_PER_CMS until a calibrated lookup table
  exists. Direction is applied to the motor when Run starts.

  Note: the OLED frame transfer blocks step generation for ~20 ms, so
  the display refresh is slowed while the belt is running to keep
  motion smooth.

  Board: Adafruit Feather M0 Adalogger (or compatible).
*/

#include <StepScreen.h>
#include <StepScreenNav.h>
#include <StepScreenSplash.h>
#include <StepScreenStepper.h>

StepScreen screen;
StepScreenNav nav;
StepScreenStepper stepper;

// Placeholder speed map until a calibrated lookup table exists:
// microsteps/sec per cm/s of belt speed.
#ifndef STEPSCREEN_TREADMILL_SPS_PER_CMS
#define STEPSCREEN_TREADMILL_SPS_PER_CMS 50.0f
#endif

// Ramp rate for gradual acceleration (microsteps/sec^2)
#define TREADMILL_ACCEL_SPS2 500.0f

// Adjustment ranges (encoder = +/-1 per detent)
#define SPEED_MIN_CMS 1
#define SPEED_MAX_CMS 100
#define TIME_MIN_MIN 1
#define TIME_MAX_MIN 180

// Display refresh throttling (OLED I2C transfer blocks stepping)
#define DRAW_INTERVAL_MS 100
#define DRAW_INTERVAL_RUNNING_MS 500

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

// --- Screen transitions (always via these, so nav debounce applies) ---

void enterHome() {
  currentScreen = SCREEN_HOME;
  stepper.disable(); // also cancels motion
  nav.enterScreen();
  Serial.println(F("[Home]"));
}

void enterRun() {
  currentScreen = SCREEN_RUN;
  runPhase = PHASE_RUNNING;

  targetSps = (float)speedCms * STEPSCREEN_TREADMILL_SPS_PER_CMS;
  if (!directionFwd) {
    targetSps = -targetSps;
  }
  currentSps = 0.0f;
  lastRampMs = millis();
  runEndMs = millis() + (uint32_t)timeoutMin * 60000UL;

  stepper.setMaxSpeed(fabsf(targetSps)); // setSpeed clamps to max
  stepper.enable();
  nav.enterScreen();

  Serial.print(F("[Run] "));
  Serial.print(speedCms);
  Serial.print(F(" cm/s "));
  Serial.print(directionFwd ? F("Fwd") : F("Rev"));
  Serial.print(F(" for "));
  Serial.print(timeoutMin);
  Serial.println(F(" min"));
}

// --- Run screen motor service (ramp + countdown) ---

void serviceRun() {
  if (currentScreen != SCREEN_RUN || runPhase != PHASE_RUNNING) {
    return;
  }

  const uint32_t now = millis();

  // Ramp currentSps toward targetSps (AccelStepper's runSpeed applies
  // no acceleration on its own)
  if (currentSps != targetSps) {
    const float step = TREADMILL_ACCEL_SPS2 * (float)(now - lastRampMs) / 1000.0f;
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
    stepper.setSpeed(currentSps);
  }
  lastRampMs = now;

  stepper.runSpeed();

  // Timeout: stop and blink DONE
  if ((int32_t)(now - runEndMs) >= 0) {
    stepper.disable();
    runPhase = PHASE_DONE;
    Serial.println(F("[Run] done"));
  }
}

// --- Per-screen input handling ---

void handleHome(uint8_t ev, int32_t enc) {
  if (ev & STEPSCREEN_EVT_BACK) {
    editTarget = (editTarget == EDIT_SPEED) ? EDIT_TIME : EDIT_SPEED;
  }
  if (ev & STEPSCREEN_EVT_PUSH) {
    directionFwd = !directionFwd;
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
  }
}

void handleRun(uint8_t ev) {
  if (ev & STEPSCREEN_EVT_BACK) {
    enterHome();
    return;
  }
}

// --- Drawing ---

// One metric row: size-2 value + size-1 unit. Inverted when selected.
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
  d.drawInfoBar("-- --:--"); // datetime placeholder until RTC
  // Back label shows which metric the encoder is editing
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
  const StepScreenActionLabels labels = {"Back", nullptr, nullptr};
  d.drawActionColumn(labels, nav.readBack(), false, false);

  // Live speed (ramps up), so acceleration is visible
  const int16_t liveCms = (int16_t)(fabsf(currentSps) /
                                    STEPSCREEN_TREADMILL_SPS_PER_CMS +
                                    0.5f);
  char buf[12];
  snprintf(buf, sizeof(buf), "%d", liveCms);
  d.setTextSize(2);
  d.setCursor(2, 13);
  d.print(buf);
  d.setTextSize(1);
  d.setCursor(d.getCursorX() + 3, 20);
  d.print("cm/s ");
  d.print(directionFwd ? "Fwd" : "Rev");

  // Countdown area (blinks DONE inverted after timeout)
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
}

void drawScreen() {
  static uint32_t lastDrawMs = 0;
  const bool beltRunning =
      currentScreen == SCREEN_RUN && runPhase == PHASE_RUNNING;
  const uint32_t interval =
      beltRunning ? DRAW_INTERVAL_RUNNING_MS : DRAW_INTERVAL_MS;
  if (millis() - lastDrawMs < interval) {
    return;
  }
  lastDrawMs = millis();

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

void setup() {
  Serial.begin(115200);
  delay(1000); // let USB serial settle; no while(!Serial) so battery works

  if (!screen.begin()) {
    Serial.println(F("Treadmill: display not found"));
    while (1) {
      delay(100);
    }
  }

  nav.begin(&screen.input());
  stepper.begin(); // driver starts disabled

  // --- STEPSCREEN ENCODER ISR SETUP (copy into setup()) ---
  STEPSCREEN_ATTACH_ENCODER_ISRS();
  // --- END STEPSCREEN ENCODER ISR SETUP ---

  StepScreenSplashConfig splash;
  splash.theme = STEPSCREEN_SPLASH_TREADMILL;
  splash.title = "Treadmill v1.0";
  splash.credit = "by the Neurotech Hub";
  StepScreenSplash::play(screen.display(), splash, &screen.input());

  enterHome();
}

void loop() {
  serviceRun(); // ramp + continuous belt motion (Run screen only)

  const uint8_t ev = nav.pollNavEvents();
  const int32_t enc = nav.consumeEncoderDelta();

  switch (currentScreen) {
  case SCREEN_HOME:
    handleHome(ev, enc);
    break;
  case SCREEN_RUN:
    handleRun(ev);
    break;
  }

  drawScreen();
}
