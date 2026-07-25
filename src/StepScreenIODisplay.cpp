#include "StepScreenIODisplay.h"

#include <stdio.h>
#include <string.h>

namespace {

void formatOutputValue(StepScreenIO &io, uint8_t index, char *buf, size_t len) {
  if (io.isStepOutput(index)) {
    strncpy(buf, io.getOutput(index) ? "PULSE" : "idle", len);
  } else {
    strncpy(buf, io.getOutput(index) ? "ON" : "off", len);
  }
  buf[len - 1] = '\0';
}

void drawHalfRow(StepScreenDisplay &d, int16_t y, int16_t x, int16_t w,
                 const char *label, const char *value, bool highlight) {
  if (highlight) {
    d.fillRect(x, y, w, 8, SH110X_WHITE);
    d.setTextColor(SH110X_BLACK);
  } else {
    d.setTextColor(SH110X_WHITE);
  }
  d.setTextSize(1);
  d.setCursor(x + 2, y);
  d.print(label);
  d.print(':');
  d.print(value);
  d.setTextColor(SH110X_WHITE);
}

} // namespace

void StepScreenIODisplay::drawStatusLine(StepScreenDisplay &d, int16_t y,
                                         const char *label, const char *value,
                                         bool highlight) {
  const int16_t rowH = 8;
  if (highlight) {
    d.fillRect(STEPSCREEN_CONTENT_X, y, STEPSCREEN_CONTENT_W, rowH,
               SH110X_WHITE);
    d.setTextColor(SH110X_BLACK);
  } else {
    d.setTextColor(SH110X_WHITE);
  }

  d.setTextSize(1);
  d.setCursor(STEPSCREEN_CONTENT_X + 2, y);
  d.print(label);

  const int16_t valueW = 6 * (int16_t)strlen(value);
  d.setCursor(STEPSCREEN_CONTENT_X + STEPSCREEN_CONTENT_W - valueW - 2, y);
  d.print(value);

  d.setTextColor(SH110X_WHITE);
}

void StepScreenIODisplay::drawPanel(StepScreenDisplay &d, StepScreenIO &io,
                                    const StepScreenInputs &inputs,
                                    StepScreenIOPage page,
                                    int8_t highlightOutput,
                                    const char *footer) {
  d.clearContentArea();
  d.setTextSize(1);

  char bufL[8];
  char bufR[8];
  int16_t y = STEPSCREEN_CONTENT_Y;
  const int16_t halfW = STEPSCREEN_CONTENT_W / 2;

  if (page == STEPSCREEN_IO_PAGE_OUTPUTS) {
    for (uint8_t i = 0; i < io.outputCount(); i += 2) {
      formatOutputValue(io, i, bufL, sizeof(bufL));
      drawHalfRow(d, y, STEPSCREEN_CONTENT_X, halfW, io.outputLabel(i), bufL,
                  (int8_t)i == highlightOutput);

      if (i + 1 < io.outputCount()) {
        formatOutputValue(io, i + 1, bufR, sizeof(bufR));
        drawHalfRow(d, y, STEPSCREEN_CONTENT_X + halfW, halfW,
                    io.outputLabel(i + 1), bufR,
                    (int8_t)(i + 1) == highlightOutput);
      }
      y += 8;
    }
  } else {
    drawStatusLine(d, y, "Beam", inputs.beamDet ? "HIGH" : "LOW");
    y += 8;
    drawStatusLine(d, y, "Aux", inputs.auxIn ? "HIGH" : "LOW");
    y += 8;
    drawStatusLine(d, y, "SD card", inputs.sdCard ? "IN" : "out");
    y += 8;
    drawStatusLine(d, y, "Encoder", itoa(inputs.encoder, bufL, 10));
    y += 8;
    snprintf(bufL, sizeof(bufL), "%.1f", inputs.vbatVolts);
    drawStatusLine(d, y, "VBAT", bufL);
  }

  if (footer != nullptr && y <= STEPSCREEN_CONTENT_Y + STEPSCREEN_CONTENT_H - 8) {
    d.setTextColor(SH110X_WHITE);
    d.setCursor(STEPSCREEN_CONTENT_X + 2, STEPSCREEN_CONTENT_Y +
                                              STEPSCREEN_CONTENT_H - 8);
    d.print(footer);
  }
}
