/*
  StepScreen BasicUI

  Minimal interactive demo:
    - Rotating the encoder changes the counter in the content area
    - Holding a button highlights its label in the action column
      (Back = top right, Sel = encoder push, OK = confirm)
    - Press events are also reported over Serial

  This sketch includes the encoder ISR setup block that every
  StepScreen sketch needs -- copy it into new sketches as-is.
*/

#include <StepScreen.h>

StepScreen screen;

int32_t counter = 0;

void setup() {
  Serial.begin(115200);

  if (!screen.begin()) {
    Serial.println("StepScreen: display not found!");
    while (1)
      delay(10);
  }

  // --- STEPSCREEN ENCODER ISR SETUP (copy into setup()) ---
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A),
                  StepScreenInput::handleEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B),
                  StepScreenInput::handleEncoderISR, CHANGE);
  // --- END STEPSCREEN ENCODER ISR SETUP ---
}

void loop() {
  StepScreenInput &in = screen.input();

  counter += in.getEncoderDelta();

  // Edge-detected press events (fires once per press)
  uint8_t events = in.pollButtons();
  if (events & STEPSCREEN_EVT_BACK)
    Serial.println("Back pressed");
  if (events & STEPSCREEN_EVT_PUSH)
    Serial.println("Encoder pushed");
  if (events & STEPSCREEN_EVT_CONFIRM)
    Serial.println("Confirm pressed");

  // Level state (true while held) drives the label highlights
  bool back = in.readBack();
  bool push = in.readPush();
  bool confirm = in.readConfirm();

  StepScreenDisplay &d = screen.display();
  d.clearDisplay();
  d.drawInfoBar("BasicUI", back);
  d.drawActionColumn(back, push, confirm);

  d.setCursor(STEPSCREEN_CONTENT_X, STEPSCREEN_CONTENT_Y + 8);
  d.print("Turn encoder:");
  d.setTextSize(2);
  d.setCursor(STEPSCREEN_CONTENT_X + 8, STEPSCREEN_CONTENT_Y + 22);
  d.print(counter);
  d.setTextSize(1);

  d.display();
  delay(16); // ~60 fps refresh cap
}
