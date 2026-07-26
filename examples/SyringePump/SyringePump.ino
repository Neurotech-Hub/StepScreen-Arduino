/*
  StepScreen SyringePump

  Three-screen pump controller demonstrating the StepScreenNav
  (menu-safe debouncing) and StepScreenStepper (AccelStepper/TMC2209)
  layers.

  Home   -- motor disabled, welcome message.
            Sel = Adjust screen, OK = Run screen. Back is n/a.
  Adjust -- motor enabled; encoder jogs the motor in both directions.
            OK cycles the speed preset (Low -> Med -> Fast).
            Back returns Home. Shows position, speed, encoder input.
  Run    -- motor disabled until a serial command arrives. Send a signed
            integer + newline (e.g. "3200" or "-500") at 115200 baud to
            move that many microsteps (1/16 microstepping: 3200 = 1 rev).
            Back returns Home (stops any move). Shows command status.

  Navigation debouncing (StepScreenNav): button edges are discarded for
  250 ms after each screen change, and a press is only accepted after
  all buttons have been released -- so one press never triggers actions
  on two screens.

  Note: the OLED frame transfer blocks step generation for ~20 ms, so
  the display refresh is throttled (and slowed further while a Run move
  is in progress) to keep motion smooth.

  The info bar shows a datetime placeholder ("--:--") until an RTC is
  integrated.

  Board: Adafruit Feather M0 Adalogger (or compatible).
*/

#include <StepScreen.h>
#include <StepScreenNav.h>
#include <StepScreenSplash.h>
#include <StepScreenStepper.h>

StepScreen screen;
StepScreenNav nav;
StepScreenStepper stepper;

// Microsteps per encoder detent on the Adjust screen (16 = 1 full step)
#define JOG_STEPS_PER_DETENT 16

// Display refresh throttling (OLED I2C transfer blocks stepping)
#define DRAW_INTERVAL_MS 100
#define DRAW_INTERVAL_MOVING_MS 400

enum PumpScreen : uint8_t {
  SCREEN_HOME,
  SCREEN_ADJUST,
  SCREEN_RUN,
};

PumpScreen currentScreen = SCREEN_HOME;

// Run-screen serial command state
enum RunStatus : uint8_t {
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

// --- Screen transitions (always via these, so nav debounce applies) ---

void enterHome() {
  currentScreen = SCREEN_HOME;
  stepper.stop();
  stepper.disable();
  nav.enterScreen();
  Serial.println(F("[Home]"));
}

void enterAdjust() {
  currentScreen = SCREEN_ADJUST;
  lastEncoderDelta = 0;
  stepper.enable();
  nav.enterScreen();
  Serial.println(F("[Adjust] encoder jogs motor"));
}

void enterRun() {
  currentScreen = SCREEN_RUN;
  stepper.disable();
  runStatus = RUN_WAITING;
  rxLen = 0;
  while (Serial.available()) {
    Serial.read(); // flush stale input
  }
  nav.enterScreen();
  Serial.println(F("[Run] send signed steps + newline, e.g. 3200"));
}

// --- Run-screen serial protocol ---

void handleSerialCommand() {
  while (Serial.available()) {
    const char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (rxLen == 0) {
        continue; // ignore blank lines / CRLF pairs
      }
      rxBuf[rxLen] = '\0';
      rxLen = 0;

      if (stepper.isRunning()) {
        Serial.println(F("busy"));
        continue;
      }

      char *end = nullptr;
      const long steps = strtol(rxBuf, &end, 10);
      if (end == rxBuf || *end != '\0') {
        runStatus = RUN_ERROR;
        Serial.print(F("error: not a number: "));
        Serial.println(rxBuf);
        continue;
      }

      lastCommandSteps = steps;
      stepper.enable();
      stepper.moveSigned(steps);
      runStatus = RUN_MOVING;
      Serial.print(F("moving "));
      Serial.println(steps);
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = c;
    } else {
      rxLen = 0; // overflow: drop the line
      runStatus = RUN_ERROR;
      Serial.println(F("error: command too long"));
    }
  }

  // Move finished: release the motor and report
  if (runStatus == RUN_MOVING && !stepper.isRunning()) {
    stepper.disable();
    runStatus = RUN_DONE;
    Serial.print(F("done, position "));
    Serial.println(stepper.currentPosition());
  }
}

// --- Per-screen input handling ---

void handleHome(uint8_t ev) {
  if (ev & STEPSCREEN_EVT_PUSH) {
    enterAdjust();
  } else if (ev & STEPSCREEN_EVT_CONFIRM) {
    enterRun();
  }
}

void handleAdjust(uint8_t ev, int32_t enc) {
  if (ev & STEPSCREEN_EVT_BACK) {
    enterHome();
    return;
  }
  if (ev & STEPSCREEN_EVT_CONFIRM) {
    stepper.nextSpeedPreset();
    Serial.print(F("speed: "));
    Serial.println(stepper.speedPresetName());
  }
  if (enc != 0) {
    lastEncoderDelta = enc;
    stepper.jogSteps(enc * JOG_STEPS_PER_DETENT);
  }
}

void handleRun(uint8_t ev) {
  if (ev & STEPSCREEN_EVT_BACK) {
    enterHome();
    return;
  }
  handleSerialCommand();
}

// --- Drawing ---

void drawHome(StepScreenDisplay &d) {
  d.drawInfoBar("Ready");
  const StepScreenActionLabels labels = {nullptr, "Adj", "Run"};
  d.drawActionColumn(labels, false, nav.readPush(), nav.readConfirm());

  d.setCursor(STEPSCREEN_CONTENT_X, STEPSCREEN_CONTENT_Y + 10);
  d.println("Select an option");
}

void drawAdjust(StepScreenDisplay &d) {
  d.drawInfoBar("Adjust");
  const StepScreenActionLabels labels = {"Back", nullptr, "Spd"};
  d.drawActionColumn(labels, nav.readBack(), false, nav.readConfirm());

  d.setCursor(STEPSCREEN_CONTENT_X, STEPSCREEN_CONTENT_Y + 4);
  d.print("Pos: ");
  d.println(stepper.currentPosition());
  d.print("Spd: ");
  d.print(stepper.speedPresetName());
  d.print(" ");
  d.print((int32_t)stepper.speedPresetSps());
  d.println("/s");
  d.print("Enc: ");
  d.println(lastEncoderDelta);
}

void drawRun(StepScreenDisplay &d) {
  d.drawInfoBar("Run");
  const StepScreenActionLabels labels = {"Back", nullptr, nullptr};
  d.drawActionColumn(labels, nav.readBack(), false, false);

  d.setCursor(STEPSCREEN_CONTENT_X, STEPSCREEN_CONTENT_Y + 4);
  d.println("Serial 115200:");
  d.println("send steps + \\n");
  d.println();

  d.print("Status: ");
  switch (runStatus) {
  case RUN_WAITING:
    d.println("waiting");
    break;
  case RUN_MOVING:
    d.print("move ");
    d.println(lastCommandSteps);
    break;
  case RUN_DONE:
    d.print("done ");
    d.println(lastCommandSteps);
    break;
  case RUN_ERROR:
    d.println("error");
    break;
  }
  d.print("Pos: ");
  d.println(stepper.currentPosition());
}

void drawScreen() {
  static uint32_t lastDrawMs = 0;
  const uint32_t interval =
      stepper.isRunning() ? DRAW_INTERVAL_MOVING_MS : DRAW_INTERVAL_MS;
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
  case SCREEN_ADJUST:
    drawAdjust(d);
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
    Serial.println(F("SyringePump: display not found"));
    while (1) {
      delay(100);
    }
  }

  nav.begin(&screen.input());
  stepper.begin(); // driver starts disabled

  // --- STEPSCREEN ENCODER ISR SETUP (copy into setup()) ---
  STEPSCREEN_ATTACH_ENCODER_ISRS();
  // --- END STEPSCREEN ENCODER ISR SETUP ---

  StepScreenSplash::play(screen.display(), StepScreenSplashConfig{},
                         &screen.input());

  enterHome();
}

void loop() {
  stepper.run(); // non-blocking; must run every loop

  const uint8_t ev = nav.pollNavEvents();
  const int32_t enc = nav.consumeEncoderDelta();

  switch (currentScreen) {
  case SCREEN_HOME:
    handleHome(ev);
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
