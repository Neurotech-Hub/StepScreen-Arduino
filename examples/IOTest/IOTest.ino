/*
  StepScreen IOTest

  Automatic board bring-up test — no menu navigation required.

  On boot:
    - Walks each output solo for 1.5 s (LED1, LED2, EXT, GRN, MEN, MDIR, STEP…)
    - Shows live inputs on one dashboard (beam, aux, buttons, encoder, VBAT, SD)
    - Mounts SD when inserted; runs a write/read self-test once per mount

  Watch the hardware and Serial (115200). Re-insert an SD card to re-test hot-plug.

  Board: Adafruit Feather M0 Adalogger (or compatible).
*/

#include <StepScreen.h>
#include <StepScreenIO.h>
#include <StepScreenSD.h>

#include <stdio.h>

StepScreen screen;
StepScreenIO io;
StepScreenSD sd;

uint8_t soloIndex = 0;
uint32_t lastWalkMs = 0;
uint32_t lastDrawMs = 0;
int32_t lastDrawnEncoder = 0;
bool lastCardPresent = false;
StepScreenSDMountState lastMountState = STEPSCREEN_SD_ABSENT;
bool sdTestedThisMount = false;

const uint32_t WALK_INTERVAL_MS = 1500;
const uint32_t DRAW_INTERVAL_MS = 250;

void logSdEvents() {
  if (sd.cardPresent() != lastCardPresent) {
    lastCardPresent = sd.cardPresent();
    sdTestedThisMount = false;
    Serial.print(F("SD card "));
    Serial.println(lastCardPresent ? F("inserted") : F("removed"));
  }

  if (sd.mountState() != lastMountState) {
    lastMountState = sd.mountState();
    Serial.print(F("SD mount: "));
    Serial.println(sd.mountStateLabel());
  }
}

void trySdSelfTest() {
  if (!sd.isReady() || sdTestedThisMount) {
    return;
  }

  Serial.println(F("SD self-test..."));
  if (sd.runSelfTest()) {
    Serial.println(F("SD self-test PASS"));
  } else {
    Serial.print(F("SD self-test FAIL: "));
    Serial.println(sd.lastError());
  }
  sdTestedThisMount = true;
}

void advanceOutputWalk() {
  soloIndex = (soloIndex + 1) % io.outputCount();
  io.soloOutput(soloIndex);
  io.apply();
  Serial.print(F("Output: "));
  Serial.println(io.outputLabel(soloIndex));
  lastWalkMs = millis();
  lastDrawMs = 0; // refresh OLED immediately
}

void drawDashboard(const StepScreenInputs &inputs) {
  StepScreenDisplay &d = screen.display();
  char buf[16];

  d.clearDisplay();
  d.drawInfoBar("IOTest");

  d.setTextSize(1);
  int16_t y = STEPSCREEN_CONTENT_Y;

  d.setCursor(STEPSCREEN_CONTENT_X + 2, y);
  d.print(F("Out "));
  d.println(io.outputLabel(soloIndex));
  y += 8;

  d.setCursor(STEPSCREEN_CONTENT_X + 2, y);
  d.print(F("Beam "));
  d.print(inputs.beamDet ? F("HI") : F("LO"));
  d.print(F("  Aux "));
  d.println(inputs.auxIn ? F("HI") : F("LO"));
  y += 8;

  d.setCursor(STEPSCREEN_CONTENT_X + 2, y);
  d.print(F("SD "));
  if (!sd.cardPresent()) {
    d.print(F("out"));
  } else {
    d.print(sd.mountStateLabel());
    if (sd.lastSelfTestPassed()) {
      d.print(F(" OK"));
    }
  }
  d.println();
  y += 8;

  d.setCursor(STEPSCREEN_CONTENT_X + 2, y);
  snprintf(buf, sizeof(buf), "Enc %ld", (long)inputs.encoder);
  d.print(buf);
  snprintf(buf, sizeof(buf), "  V %.1f", inputs.vbatVolts);
  d.println(buf);
  y += 8;

  d.setCursor(STEPSCREEN_CONTENT_X + 2, y);
  d.print(inputs.btnBack ? F("B") : F("-"));
  d.print(inputs.btnPush ? F("S") : F("-"));
  d.print(inputs.btnConfirm ? F("K") : F("-"));
  d.println(F("  auto walk"));

  d.display();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (!screen.begin()) {
    Serial.println(F("IOTest: display not found"));
    while (1) {
      delay(100);
    }
  }

  io.begin();
  sd.begin();

  STEPSCREEN_ATTACH_ENCODER_ISRS();

  soloIndex = io.soloOutput(0);
  io.apply();
  lastWalkMs = millis();
  lastDrawMs = millis();

  lastCardPresent = sd.cardPresent();
  lastMountState = sd.mountState();

  Serial.println(F("IOTest — auto output walk + live dashboard"));
  Serial.print(F("Output: "));
  Serial.println(io.outputLabel(soloIndex));
}

void loop() {
  sd.update();
  logSdEvents();
  trySdSelfTest();

  if (millis() - lastWalkMs >= WALK_INTERVAL_MS) {
    advanceOutputWalk();
  }

  io.serviceStepOutput();

  StepScreenInput &in = screen.input();
  in.pollButtons(); // keep debounce state fresh for dashboard

  StepScreenInputs inputs;
  io.readInputs(inputs, &in);

  const bool needsDraw = (millis() - lastDrawMs >= DRAW_INTERVAL_MS) ||
                         (inputs.encoder != lastDrawnEncoder);

  if (needsDraw) {
    lastDrawMs = millis();
    lastDrawnEncoder = inputs.encoder;
    drawDashboard(inputs);
  }
}
