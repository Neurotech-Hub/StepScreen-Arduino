#include "StepScreenSplash.h"

#include "StepScreenInput.h"

#include <string.h>

// Fixed vertical bands — graphic above, text below, no overlap.
static const int16_t NEURON_CY = 13;
static const int16_t TITLE_Y = 30;
static const int16_t CREDIT_Y = 48;

// Axon path: hillock → gentle curve → terminal arbor (coords relative to soma center)
static const int16_t AXON_PT[][2] = {
    {5, 0}, {14, -1}, {26, 1}, {38, 0}, {50, -1}, {62, 0}, {72, 1}, {80, 0},
};
static const uint8_t AXON_PT_COUNT = 8;

// Terminal branches (from last axon point)
static const int16_t TERMINAL_PT[][2] = {
    {84, -4}, {88, -6}, {86, 4},  {90, 6},  {84, 5},
};
static const uint8_t TERMINAL_PT_COUNT = 5;

static bool skipRequested(StepScreenInput *input, bool skippable) {
  if (!skippable || input == nullptr) {
    return false;
  }
  input->pollButtons();
  return input->readBack() || input->readConfirm() || input->readPush();
}

static void drawCentered(StepScreenDisplay &d, const char *text, int16_t y) {
  if (text == nullptr || text[0] == '\0') {
    return;
  }
  const int16_t w = 6 * (int16_t)strlen(text);
  d.setTextSize(1);
  d.setTextColor(SH110X_WHITE);
  d.setCursor((STEPSCREEN_W - w) / 2, y);
  d.print(text);
}

static int16_t axonPathLength() {
  int16_t len = 0;
  for (uint8_t i = 1; i < AXON_PT_COUNT; i++) {
    const int16_t dx = AXON_PT[i][0] - AXON_PT[i - 1][0];
    const int16_t dy = AXON_PT[i][1] - AXON_PT[i - 1][1];
    len += (int16_t)sqrt((float)(dx * dx + dy * dy));
  }
  return len;
}

static void axonPointAt(int16_t somaX, int16_t somaY, uint8_t travel,
                        int16_t &outX, int16_t &outY) {
  const int16_t total = axonPathLength();
  int16_t target = (int16_t)((int32_t)total * travel / 255);
  int16_t segStart = 0;

  for (uint8_t i = 1; i < AXON_PT_COUNT; i++) {
    const int16_t x0 = somaX + AXON_PT[i - 1][0];
    const int16_t y0 = somaY + AXON_PT[i - 1][1];
    const int16_t x1 = somaX + AXON_PT[i][0];
    const int16_t y1 = somaY + AXON_PT[i][1];
    const int16_t dx = x1 - x0;
    const int16_t dy = y1 - y0;
    const int16_t segLen = (int16_t)sqrt((float)(dx * dx + dy * dy));

    if (target <= segStart + segLen || i == AXON_PT_COUNT - 1) {
      const int16_t t = segLen > 0 ? target - segStart : 0;
      outX = x0 + (int16_t)((int32_t)dx * t / segLen);
      outY = y0 + (int16_t)((int32_t)dy * t / segLen);
      return;
    }
    segStart += segLen;
  }
  outX = somaX + AXON_PT[AXON_PT_COUNT - 1][0];
  outY = somaY + AXON_PT[AXON_PT_COUNT - 1][1];
}

// Myelinated axon: dashed sheath segments with gaps at nodes of Ranvier
static void drawMyelinatedAxon(StepScreenDisplay &d, int16_t somaX, int16_t somaY,
                               uint8_t progress) {
  int16_t px = somaX + AXON_PT[0][0];
  int16_t py = somaY + AXON_PT[0][1];
  int16_t drawn = 0;
  const int16_t total = axonPathLength();
  const int16_t limit = (int16_t)((int32_t)total * progress / 255);

  for (uint8_t i = 1; i < AXON_PT_COUNT; i++) {
    const int16_t x1 = somaX + AXON_PT[i][0];
    const int16_t y1 = somaY + AXON_PT[i][1];
    const int16_t dx = x1 - px;
    const int16_t dy = y1 - py;
    const int16_t segLen = (int16_t)sqrt((float)(dx * dx + dy * dy));
    if (segLen == 0) {
      px = x1;
      py = y1;
      continue;
    }

    for (int16_t s = 0; s < segLen; s++) {
      if (drawn >= limit) {
        return;
      }
      const int16_t x = px + (int16_t)((int32_t)dx * s / segLen);
      const int16_t y = py + (int16_t)((int32_t)dy * s / segLen);
      // 5 px myelin, 2 px node gap
      if ((drawn % 7) < 5) {
        d.drawPixel(x, y, SH110X_WHITE);
        if (dy == 0) {
          d.drawPixel(x, y - 1, SH110X_WHITE);
        } else {
          d.drawPixel(x, y + 1, SH110X_WHITE);
        }
      }
      drawn++;
    }
    px = x1;
    py = y1;
  }
}

static void drawNeuron(StepScreenDisplay &d, int16_t somaX, int16_t somaY,
                       uint8_t detail) {
  if (detail >= 20) {
    d.drawCircle(somaX, somaY, 4, SH110X_WHITE);
  }
  if (detail >= 50) {
    d.drawLine(somaX - 1, somaY - 3, somaX - 8, somaY - 10, SH110X_WHITE);
    d.drawLine(somaX + 2, somaY - 4, somaX + 9, somaY - 10, SH110X_WHITE);
    d.drawLine(somaX - 2, somaY + 2, somaX - 7, somaY + 8, SH110X_WHITE);
  }
  if (detail >= 70) {
    const int16_t hx = somaX + 4;
    d.drawLine(somaX + 3, somaY - 1, hx, somaY, SH110X_WHITE);
    d.drawLine(somaX + 3, somaY + 1, hx, somaY, SH110X_WHITE);
    d.fillCircle(hx, somaY, 1, SH110X_WHITE);
  }
  if (detail >= 90) {
    const uint8_t axonProgress =
        detail < 220 ? (uint8_t)((detail - 90) * 255UL / 130) : 255;
    drawMyelinatedAxon(d, somaX, somaY, axonProgress);
  }
  if (detail >= 200) {
    const int16_t tx = somaX + AXON_PT[AXON_PT_COUNT - 1][0];
    const int16_t ty = somaY + AXON_PT[AXON_PT_COUNT - 1][1];
    for (uint8_t i = 0; i < TERMINAL_PT_COUNT; i++) {
      const int16_t bx = somaX + TERMINAL_PT[i][0];
      const int16_t by = somaY + TERMINAL_PT[i][1];
      d.drawLine(tx, ty, bx, by, SH110X_WHITE);
      d.fillCircle(bx, by, 1, SH110X_WHITE);
    }
  }
}

static void drawAxonPulse(StepScreenDisplay &d, int16_t somaX, int16_t somaY,
                          uint8_t travel) {
  int16_t x, y;
  axonPointAt(somaX, somaY, travel, x, y);
  d.fillCircle(x, y, 2, SH110X_WHITE);
}

bool StepScreenSplash::play(StepScreenDisplay &display, const char *title,
                            const char *subtitle, StepScreenInput *input) {
  StepScreenSplashConfig cfg;
  cfg.title = title;
  cfg.credit = subtitle;
  return play(display, cfg, input);
}

bool StepScreenSplash::play(StepScreenDisplay &display,
                            const StepScreenSplashConfig &config,
                            StepScreenInput *input) {
  const char *title = config.title ? config.title : "Syringe Pump v1.0";
  const char *credit =
      config.credit ? config.credit : "by the Neurotech Hub";
  const uint16_t totalMs = config.durationMs ? config.durationMs : 3200;
  const uint32_t startMs = millis();
  const int16_t somaX = 22;

  while (true) {
    const uint32_t elapsed = millis() - startMs;
    if (elapsed >= totalMs) {
      break;
    }
    if (skipRequested(input, config.skippable)) {
      display.clearDisplay();
      display.display();
      return false;
    }

    display.clearDisplay();

    uint8_t structure = elapsed < 700 ? (uint8_t)(elapsed * 255UL / 700) : 255;
    drawNeuron(display, somaX, NEURON_CY, structure);

    if (elapsed >= 550) {
      const uint32_t pulseElapsed = elapsed - 550;
      const uint8_t travel =
          (uint8_t)((pulseElapsed % 380) * 255UL / 380);
      drawAxonPulse(display, somaX, NEURON_CY, travel);
    }

    if (elapsed >= 700) {
      drawCentered(display, title, TITLE_Y);
    }
    if (elapsed >= 1000) {
      drawCentered(display, credit, CREDIT_Y);
    }

    display.display();
    delay(45);
  }

  display.clearDisplay();
  display.display();
  return true;
}
