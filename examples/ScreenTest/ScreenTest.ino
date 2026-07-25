/*
  StepScreen ScreenTest

  Display-only validation for the 1.3" SH1106 OLED (128x64, I2C).
  Steps through a sequence (~2s each) to verify the full panel is
  usable and the StepScreen layout zones land where expected:

    1. Full 128x64 border (catches SH1106 column-offset clipping)
    2. Corner markers at (0,0), (127,0), (0,63), (127,63)
    3. Layout guides with zone labels
    4. Fill patterns in each zone (info bar, content, action column)
    5. Display inversion check, then a final composed UI frame

  No encoder ISR is required for this example. Board: Adafruit Feather
  M0 Adalogger (or compatible). Wiring: OLED SDA->20, SCL->21, 3.3V, GND.
*/

#include <StepScreen.h>

StepScreen screen;

#define STEP_MS 2000

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

  StepScreenDisplay &d = screen.display();

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

  // 5. Inversion check, then hold a final composed UI frame
  Serial.println("Step 5: invert + final frame");
  d.invertDisplay(true);
  delay(1000);
  d.invertDisplay(false);
  delay(500);

  d.clearDisplay();
  d.drawInfoBar("ScreenTest");
  d.drawActionColumn(false, false, false);
  d.setCursor(STEPSCREEN_CONTENT_X, STEPSCREEN_CONTENT_Y + 8);
  d.println("All zones drawn.");
  d.println("If every edge and");
  d.println("corner was visible,");
  d.println("the panel is good.");
  d.display();
  Serial.println("ScreenTest complete");
}

void loop() {
  // Hold the final frame
}
