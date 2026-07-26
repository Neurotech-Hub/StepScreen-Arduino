#include "StepScreenDisplay.h"

bool StepScreenDisplay::begin(uint8_t i2cAddr) {
  if (!Adafruit_SH1106G::begin(i2cAddr, true)) {
    return false;
  }
  clearDisplay(); // drop the Adafruit splash from the buffer
  setTextSize(1);
  setTextColor(SH110X_WHITE);
  setTextWrap(false);
  display();
  return true;
}

void StepScreenDisplay::drawInfoBar(const char *title, bool highlightBack) {
  // Clear the whole top row, then invert only the content-width bar so
  // the action column keeps its black background for the Back label.
  fillRect(0, 0, STEPSCREEN_W, STEPSCREEN_INFO_BAR_H, SH110X_BLACK);
  fillRect(0, 0, STEPSCREEN_CONTENT_W, STEPSCREEN_INFO_BAR_H, SH110X_WHITE);
  setTextSize(1);
  setTextColor(SH110X_BLACK);
  setCursor(2, 0);
  print(title);
  setTextColor(SH110X_WHITE);
  drawActionLabel("Back", STEPSCREEN_BTN_BACK_Y, highlightBack);
}

void StepScreenDisplay::drawActionColumn(bool backActive, bool pushActive,
                                         bool confirmActive) {
  const StepScreenActionLabels defaults = {"Back", "Sel", "OK"};
  drawActionColumn(defaults, backActive, pushActive, confirmActive);
}

void StepScreenDisplay::drawActionColumn(const StepScreenActionLabels &labels,
                                         bool backActive, bool pushActive,
                                         bool confirmActive) {
  drawActionLabel(labels.back ? labels.back : "----", STEPSCREEN_BTN_BACK_Y,
                  labels.back != nullptr && backActive);
  drawActionLabel(labels.push ? labels.push : "----", STEPSCREEN_BTN_PUSH_Y,
                  labels.push != nullptr && pushActive);
  drawActionLabel(labels.confirm ? labels.confirm : "----",
                  STEPSCREEN_BTN_CONFIRM_Y,
                  labels.confirm != nullptr && confirmActive);
}

void StepScreenDisplay::clearContentArea() {
  fillRect(STEPSCREEN_CONTENT_X, STEPSCREEN_CONTENT_Y, STEPSCREEN_CONTENT_W,
           STEPSCREEN_CONTENT_H, SH110X_BLACK);
  setCursor(STEPSCREEN_CONTENT_X, STEPSCREEN_CONTENT_Y);
}

void StepScreenDisplay::drawLayoutGuides() {
  // Outer border: confirms the full 128x64 area is visible with no
  // SH1106 column-offset clipping.
  drawRect(0, 0, STEPSCREEN_W, STEPSCREEN_H, SH110X_WHITE);

  // Zone separators
  drawFastHLine(0, STEPSCREEN_INFO_BAR_H, STEPSCREEN_W, SH110X_WHITE);
  drawFastVLine(STEPSCREEN_ACTION_COL_X, 0, STEPSCREEN_H, SH110X_WHITE);

  // Corner markers (3px ticks pointing inward)
  for (uint8_t i = 0; i < 3; i++) {
    drawPixel(i, 0, SH110X_WHITE);
    drawPixel(0, i, SH110X_WHITE);
    drawPixel(STEPSCREEN_W - 1 - i, 0, SH110X_WHITE);
    drawPixel(STEPSCREEN_W - 1, i, SH110X_WHITE);
    drawPixel(i, STEPSCREEN_H - 1, SH110X_WHITE);
    drawPixel(0, STEPSCREEN_H - 1 - i, SH110X_WHITE);
    drawPixel(STEPSCREEN_W - 1 - i, STEPSCREEN_H - 1, SH110X_WHITE);
    drawPixel(STEPSCREEN_W - 1, STEPSCREEN_H - 1 - i, SH110X_WHITE);
  }
}

void StepScreenDisplay::drawActionLabel(const char *label, int16_t y,
                                        bool active) {
  const int16_t w = 6 * (int16_t)strlen(label); // 6px advance per char
  const int16_t x = STEPSCREEN_W - w - 1;       // right-justified
  const int16_t boxY = (y > 0) ? y - 1 : 0;

  // Clear this label's strip inside the action column so labels can be
  // redrawn (e.g. the labeled column replacing the info bar's "Back").
  fillRect(STEPSCREEN_ACTION_COL_X + 1, boxY,
           STEPSCREEN_W - STEPSCREEN_ACTION_COL_X - 1, 9, SH110X_BLACK);

  setTextSize(1);
  if (active) {
    fillRect(x - 2, boxY, w + 3, 9, SH110X_WHITE);
    setTextColor(SH110X_BLACK);
  } else {
    setTextColor(SH110X_WHITE);
  }
  setCursor(x, y);
  print(label);
  setTextColor(SH110X_WHITE);
}
