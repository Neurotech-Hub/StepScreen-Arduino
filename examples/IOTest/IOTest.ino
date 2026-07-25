/*
  StepScreen IOTest

  Exercises all board outputs and displays live input states on the OLED.

  Modes (encoder push / Sel toggles page; footer shows controls):
    OUT page — auto-walks outputs one-at-a-time (1.5 s each), then repeats.
               Encoder: select output manually. Confirm/OK: toggle selected.
               Back: all off.  (Auto-walk pauses until all-off, then resumes.)
    IN page  — live digital/analog inputs + encoder count.

  Outputs under test (StepScreenIO registry):
    LED1, LED2, EXT, GRN, [RED], MEN (~EN), MDIR, STEP (pulses when active)

  Board: Adafruit Feather M0 Adalogger (or compatible).
*/

#include <StepScreen.h>
#include <StepScreenIO.h>
#include <StepScreenIODisplay.h>

StepScreen screen;
StepScreenIO io;

enum RunMode : uint8_t {
  MODE_AUTO_WALK,
  MODE_MANUAL,
};

RunMode runMode = MODE_AUTO_WALK;
StepScreenIOPage page = STEPSCREEN_IO_PAGE_OUTPUTS;

uint8_t soloIndex = 0;
int8_t selectedOutput = 0;
uint32_t lastWalkMs = 0;

const uint32_t WALK_INTERVAL_MS = 1500;

void applyAndReport(const char *action) {
  io.apply();
  Serial.println(action);
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

  // --- STEPSCREEN ENCODER ISR SETUP (copy into setup()) ---
  STEPSCREEN_ATTACH_ENCODER_ISRS();
  // --- END STEPSCREEN ENCODER ISR SETUP ---

  runMode = MODE_AUTO_WALK;
  soloIndex = io.soloOutput(0);
  io.apply();
  lastWalkMs = millis();

  Serial.println(F("IOTest started — auto-walking outputs"));
}

void loop() {
  StepScreenInput &in = screen.input();
  StepScreenInputs inputs;
  io.readInputs(inputs, &in);

  // --- Output walk / manual control ---
  if (page == STEPSCREEN_IO_PAGE_OUTPUTS) {
    if (runMode == MODE_AUTO_WALK) {
      if (millis() - lastWalkMs >= WALK_INTERVAL_MS) {
        soloIndex = (soloIndex + 1) % io.outputCount();
        io.soloOutput(soloIndex);
        io.apply();
        Serial.print(F("Auto solo: "));
        Serial.println(io.outputLabel(soloIndex));
        lastWalkMs = millis();
      }
    }

    int32_t encDelta = in.getEncoderDelta();
    if (encDelta != 0) {
      runMode = MODE_MANUAL;
      selectedOutput =
          (selectedOutput + encDelta) % (int8_t)io.outputCount();
      if (selectedOutput < 0) {
        selectedOutput += io.outputCount();
      }
    }

    uint8_t events = in.pollButtons();
    if (events & STEPSCREEN_EVT_BACK) {
      io.setAllOutputs(false);
      io.apply();
      runMode = MODE_MANUAL;
      Serial.println(F("All outputs OFF"));
    }
    if (events & STEPSCREEN_EVT_CONFIRM) {
      runMode = MODE_MANUAL;
      io.setOutput((uint8_t)selectedOutput,
                   !io.getOutput((uint8_t)selectedOutput));
      io.apply();
      Serial.print(F("Toggle "));
      Serial.println(io.outputLabel((uint8_t)selectedOutput));
    }
    if (events & STEPSCREEN_EVT_PUSH) {
      page = (page == STEPSCREEN_IO_PAGE_OUTPUTS)
                 ? STEPSCREEN_IO_PAGE_INPUTS
                 : STEPSCREEN_IO_PAGE_OUTPUTS;
    }

    io.serviceStepOutput();
  } else {
    uint8_t events = in.pollButtons();
    if (events & STEPSCREEN_EVT_PUSH) {
      page = STEPSCREEN_IO_PAGE_OUTPUTS;
      runMode = MODE_AUTO_WALK;
      lastWalkMs = millis();
    }
    if (events & STEPSCREEN_EVT_BACK) {
      io.setAllOutputs(true);
      io.apply();
      Serial.println(F("All outputs ON"));
    }
    if (events & STEPSCREEN_EVT_CONFIRM) {
      io.setAllOutputs(false);
      io.apply();
      Serial.println(F("All outputs OFF"));
    }
  }

  // --- Display ---
  StepScreenDisplay &d = screen.display();
  d.clearDisplay();

  const char *title = (page == STEPSCREEN_IO_PAGE_OUTPUTS) ? "IO Out" : "IO In";
  d.drawInfoBar(title, in.readBack());
  d.drawActionColumn(in.readBack(), in.readPush(), in.readConfirm());

  int8_t highlight = -1;
  const char *footer = nullptr;
  if (page == STEPSCREEN_IO_PAGE_OUTPUTS) {
    highlight = (runMode == MODE_MANUAL) ? selectedOutput : (int8_t)soloIndex;
    footer = (runMode == MODE_AUTO_WALK) ? "Sel:inputs" : "Enc:sel OK:tog";
  } else {
    footer = "Sel:outputs";
  }

  StepScreenIODisplay::drawPanel(d, io, inputs, page, highlight, footer);
  d.display();

  delay(16);
}
