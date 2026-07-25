/*
  StepScreen ScreenTest

  Two-phase hardware check for the 1.3" SH1106 OLED (128x64, I2C) and
  the EC11 encoder + three buttons.

  Phase 1 -- display validation (~2s per step, runs in setup):
    1. Full 128x64 border (catches SH1106 column-offset clipping)
    2. Corner markers at (0,0), (127,0), (0,63), (127,63)
    3. Layout guides with zone labels
    4. Fill patterns in each zone (info bar, content, action column)
    5. Display inversion check

  Phase 2 -- interactive control check (runs in loop):
    - Rotate encoder: updates the counter and a horizontal bar
    - Hold any button: highlights its label in the action column
    - Press any button: shows a one-shot message in the content area
    - All activity is also reported over Serial

  Board: Adafruit Feather M0 Adalogger (or compatible).
  Wiring: OLED SDA->20, SCL->21, 3.3V, GND.
*/

#include <StepScreen.h>

StepScreen screen;

#define STEP_MS 2000
#define EVENT_HOLD_MS 1500

int32_t encoderValue = 0;
int8_t lastEncoderDir = 0; // -1, 0, +1 (shown for one frame after rotation)
char lastEvent[20] = "";
uint32_t lastEventMs = 0;

void runDisplayTests(StepScreenDisplay &d) {
  // 1. Full-screen border: all four edges should be fully visible
  Serial.println("Step 1: full 128x64 border");
  d.clearDisplay();
  d.drawRect(0, 0, STEPSCREEN_W, STEPSCREEN_H, SH110X_WHITE);
  d.setCursor(34, 28);
  d.print("128 x 64");
  d.display();
  delay(STEP_MS);

  // 2. Corner markers: a lit pixel in each extreme corner plus ticks
  Serial.println("Step 2: corner markers");
  d.clearDisplay();
  for (uint8_t i = 0; i < 6; i++) {
    d.drawPixel(i, 0, SH110X_WHITE);
    d.drawPixel(0, i, SH110X_WHITE);
    d.drawPixel(STEPSCREEN_W - 1 - i, 0, SH110X_WHITE);
    d.drawPixel(STEPSCREEN_W - 1, i, SH110X_WHITE);
    d.drawPixel(i, STEPSCREEN_H - 1, SH110X_WHITE);
    d.drawPixel(0, STEPSCREEN_H - 1 - i, SH110X_WHITE);
    d.drawPixel(STEPSCREEN_W - 1 - i, STEPSCREEN_H - 1, SH110X_WHITE);
    d.drawPixel(STEPSCREEN_W - 1, STEPSCREEN_H - 1 - i, SH110X_WHITE);
  }
  d.setCursor(28, 28);
  d.print("4 corners lit");
  d.display();
  delay(STEP_MS);

  // 3. Layout guides + zone labels
  Serial.println("Step 3: layout guides");
  d.clearDisplay();
  d.drawLayoutGuides();
  d.setCursor(4, 0);
  d.print("INFO");
  d.setCursor(30, 30);
  d.print("CONTENT");
  d.drawActionColumn(false, false, false);
  d.display();
  delay(STEP_MS);

  // 4. Zone fills: each region painted solid, separated by 1px gaps
  Serial.println("Step 4: zone fills");
  d.clearDisplay();
  d.fillRect(0, 0, STEPSCREEN_CONTENT_W, STEPSCREEN_INFO_BAR_H,
             SH110X_WHITE);
  d.fillRect(STEPSCREEN_CONTENT_X, STEPSCREEN_CONTENT_Y + 1,
             STEPSCREEN_CONTENT_W, STEPSCREEN_CONTENT_H - 1, SH110X_WHITE);
  d.fillRect(STEPSCREEN_ACTION_COL_X + 1, 0, STEPSCREEN_ACTION_COL_W - 1,
             STEPSCREEN_H, SH110X_WHITE);
  d.display();
  delay(STEP_MS);

  // 5. Inversion check
  Serial.println("Step 5: invert check");
  d.invertDisplay(true);
  delay(1000);
  d.invertDisplay(false);
  delay(500);
}

void noteEvent(const char *label) {
  strncpy(lastEvent, label, sizeof(lastEvent) - 1);
  lastEvent[sizeof(lastEvent) - 1] = '\0';
  lastEventMs = millis();
  Serial.println(label);
}

void drawInteractiveUI(StepScreenDisplay &d, bool back, bool push, bool confirm) {
  d.clearDisplay();
  d.drawInfoBar("Controls", back);
  d.drawActionColumn(back, push, confirm);

  // Encoder value (large)
  d.setTextSize(1);
  d.setCursor(STEPSCREEN_CONTENT_X, STEPSCREEN_CONTENT_Y + 2);
  d.print("Encoder:");
  if (lastEncoderDir < 0) {
    d.print(" <<");
  } else if (lastEncoderDir > 0) {
    d.print(" >>");
  }

  d.setTextSize(2);
  d.setCursor(STEPSCREEN_CONTENT_X + 4, STEPSCREEN_CONTENT_Y + 12);
  d.print(encoderValue);
  d.setTextSize(1);

  // Bar tracks encoder position (wraps 0..99)
  const int16_t barY = STEPSCREEN_CONTENT_Y + 30;
  const int16_t barH = 8;
  int32_t wrapped = encoderValue % 100;
  if (wrapped < 0)
    wrapped += 100;
  const int16_t fillW =
      map((int16_t)wrapped, 0, 99, 0, STEPSCREEN_CONTENT_W - 2);

  d.drawRect(STEPSCREEN_CONTENT_X, barY, STEPSCREEN_CONTENT_W, barH,
             SH110X_WHITE);
  if (fillW > 0) {
    d.fillRect(STEPSCREEN_CONTENT_X + 1, barY + 1, fillW, barH - 2,
               SH110X_WHITE);
  }

  // One-shot press feedback
  d.setCursor(STEPSCREEN_CONTENT_X, STEPSCREEN_CONTENT_Y + 42);
  if (lastEvent[0] != '\0' && (millis() - lastEventMs) < EVENT_HOLD_MS) {
    d.print(lastEvent);
  } else {
    d.print("Press a button...");
  }

  d.display();
}

void setup() {
  Serial.begin(115200);
  delay(1000); // give USB serial a moment (no while(!Serial) so it runs on battery)

  if (!screen.begin()) {
    Serial.println("StepScreen: display not found at I2C 0x3C!");
    pinMode(PIN_LED_RED, OUTPUT);
    while (1) { // fast-blink the red LED on failure
      digitalWrite(PIN_LED_RED, HIGH);
      delay(100);
      digitalWrite(PIN_LED_RED, LOW);
      delay(100);
    }
  }

  Serial.println("StepScreen: display OK, starting test sequence");
  runDisplayTests(screen.display());

  // --- STEPSCREEN ENCODER ISR SETUP (copy into setup()) ---
  STEPSCREEN_ATTACH_ENCODER_ISRS();
  // --- END STEPSCREEN ENCODER ISR SETUP ---

  noteEvent("Display OK");
  Serial.println("ScreenTest: interactive control check -- rotate encoder, press buttons");
}

void loop() {
  StepScreenInput &in = screen.input();

  int32_t delta = in.getEncoderDelta();
  if (delta != 0) {
    encoderValue += delta;
    lastEncoderDir = (delta > 0) ? 1 : -1;
  } else {
    lastEncoderDir = 0;
  }

  uint8_t events = in.pollButtons();
  if (events & STEPSCREEN_EVT_BACK)
    noteEvent("Back pressed");
  if (events & STEPSCREEN_EVT_PUSH)
    noteEvent("Sel pressed");
  if (events & STEPSCREEN_EVT_CONFIRM)
    noteEvent("OK pressed");

  drawInteractiveUI(screen.display(), in.readBack(), in.readPush(),
                    in.readConfirm());

  delay(16); // ~60 fps refresh cap
}
