#include "StepScreenSplash.h"

#include "StepScreenInput.h"

#include <string.h>

// Fixed vertical bands — graphic above, text below, no overlap.
static const int16_t TITLE_Y = 30;
static const int16_t CREDIT_Y = 48;

// Syringe geometry (horizontal, upper band y=0..24, needle to the right)
static const int16_t SYR_CY = 12;      // centerline
static const int16_t BARREL_X = 40;    // barrel mouth (left edge)
static const int16_t BARREL_W = 40;
static const int16_t BARREL_H = 12;    // y 6..17
static const int16_t TAPER_W = 6;      // barrel-to-needle taper
static const int16_t NEEDLE_LEN = 12;
static const int16_t ROD_LEN = 34;     // plunger rod (handle stops at mouth)
static const int16_t PLUNGER_MAX = 30; // seal travel in px
static const int16_t DROP_SPAN = 18;   // droplet flight distance

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

// detail 0..255 reveals the syringe body; plunger 0..PLUNGER_MAX is how
// far the seal has been depressed into the barrel.
static void drawSyringe(StepScreenDisplay &d, uint8_t detail,
                        int16_t plunger) {
  const int16_t top = SYR_CY - BARREL_H / 2;
  const int16_t bottom = top + BARREL_H - 1;
  const int16_t barrelRight = BARREL_X + BARREL_W - 1;
  const int16_t needleBase = barrelRight + TAPER_W;

  // Barrel walls sweep in left-to-right
  const int16_t reveal =
      detail < 120 ? (int16_t)((int32_t)BARREL_W * detail / 120) : BARREL_W;
  if (reveal > 0) {
    d.drawFastHLine(BARREL_X, top, reveal, SH110X_WHITE);
    d.drawFastHLine(BARREL_X, bottom, reveal, SH110X_WHITE);
  }

  // Open-backed mouth with finger flanges (central gap lets the rod
  // slide through)
  if (detail >= 120) {
    d.drawFastVLine(BARREL_X, top - 2, 5, SH110X_WHITE);
    d.drawFastVLine(BARREL_X, SYR_CY + 3, 5, SH110X_WHITE);
  }

  // Taper down to the needle
  if (detail >= 160) {
    d.drawLine(barrelRight, top, needleBase, SYR_CY - 1, SH110X_WHITE);
    d.drawLine(barrelRight, bottom, needleBase, SYR_CY + 1, SH110X_WHITE);
    d.drawFastHLine(needleBase, SYR_CY, NEEDLE_LEN, SH110X_WHITE);
  }

  // Plunger (thumb pad + rod + seal) and remaining fluid
  if (detail >= 200) {
    const int16_t sealX = BARREL_X + 2 + plunger;
    d.fillRect(sealX, top + 2, 2, BARREL_H - 4, SH110X_WHITE);
    d.drawFastHLine(sealX - ROD_LEN, SYR_CY, ROD_LEN, SH110X_WHITE);
    d.fillRect(sealX - ROD_LEN - 2, SYR_CY - 5, 2, 11, SH110X_WHITE);

    // Fluid ahead of the seal, dithered so it reads as liquid (1px gap
    // keeps it distinct from the solid seal)
    const int16_t fluidX = sealX + 3;
    const int16_t fluidEnd = barrelRight - 1;
    for (int16_t x = fluidX; x < fluidEnd; x++) {
      for (int16_t y = top + 2; y <= bottom - 2; y++) {
        if (((x + y) & 1) == 0) {
          d.drawPixel(x, y, SH110X_WHITE);
        }
      }
    }
  }
}

// Droplets leaving the needle tip; phase 0..255 loops.
static void drawDrops(StepScreenDisplay &d, uint8_t phase) {
  const int16_t tipX = BARREL_X + BARREL_W - 1 + TAPER_W + NEEDLE_LEN + 2;
  for (uint8_t i = 0; i < 2; i++) {
    const uint8_t p = (uint8_t)(phase + i * 128);
    const int16_t x = tipX + (int16_t)((int32_t)DROP_SPAN * p / 255);
    const int16_t y = SYR_CY + (((p >> 6) & 1) ? 1 : -1); // slight wobble
    d.fillCircle(x, y, 1, SH110X_WHITE);
  }
}

// Treadmill geometry (horizontal side view, upper band y=0..24)
static const int16_t TM_LEFT = 34;     // left roller center x
static const int16_t TM_RIGHT = 94;    // right roller center x
static const int16_t TM_BELT_TOP = 16; // belt top surface
static const int16_t TM_BELT_BOT = 22; // belt bottom
static const int16_t TM_ROLLER_R = 3;

// detail 0..255 reveals the frame; beltPhase 0..7 scrolls the belt
// dashes leftward; gait alternates the rodent's leg pairs.
static void drawTreadmill(StepScreenDisplay &d, uint8_t detail,
                          uint8_t beltPhase, bool gait) {
  const int16_t rollerCy = (TM_BELT_TOP + TM_BELT_BOT) / 2;

  // Belt frame sweeps in left-to-right
  const int16_t span = TM_RIGHT - TM_LEFT;
  const int16_t reveal =
      detail < 120 ? (int16_t)((int32_t)span * detail / 120) : span;
  if (reveal > 0) {
    d.drawFastHLine(TM_LEFT, TM_BELT_TOP, reveal, SH110X_WHITE);
    d.drawFastHLine(TM_LEFT, TM_BELT_BOT, reveal, SH110X_WHITE);
  }

  // Rollers at both ends
  if (detail >= 60) {
    d.drawCircle(TM_LEFT, rollerCy, TM_ROLLER_R, SH110X_WHITE);
  }
  if (detail >= 120) {
    d.drawCircle(TM_RIGHT, rollerCy, TM_ROLLER_R, SH110X_WHITE);
  }

  // Belt dashes scrolling left (belt surface moves under the rodent)
  if (detail >= 160) {
    for (int16_t x = TM_LEFT + 3 + ((8 - beltPhase) & 7); x < TM_RIGHT - 3;
         x += 8) {
      d.drawFastVLine(x, TM_BELT_TOP + 2, 3, SH110X_WHITE);
    }
  }

  // Rodent running in place, facing right
  if (detail >= 200) {
    // Body + head + nose
    d.fillRoundRect(56, 7, 16, 7, 3, SH110X_WHITE); // body y 7..13
    d.fillCircle(73, 8, 3, SH110X_WHITE);           // head
    d.drawPixel(77, 9, SH110X_WHITE);               // nose
    d.drawCircle(72, 4, 1, SH110X_WHITE);           // ear
    d.drawPixel(74, 7, SH110X_BLACK);               // eye

    // Tail curving up and back to the left
    d.drawLine(56, 10, 48, 6, SH110X_WHITE);
    d.drawLine(48, 6, 42, 8, SH110X_WHITE);

    // Four stick legs, two-phase gait (diagonal pairs swap)
    const int16_t footY = TM_BELT_TOP - 1;
    const int8_t offA = gait ? 2 : -1;
    const int8_t offB = gait ? -1 : 2;
    d.drawLine(58, 13, 58 + offA, footY, SH110X_WHITE); // rear left
    d.drawLine(62, 13, 62 + offB, footY, SH110X_WHITE); // rear right
    d.drawLine(66, 13, 66 + offA, footY, SH110X_WHITE); // front left
    d.drawLine(70, 13, 70 + offB, footY, SH110X_WHITE); // front right
  }
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

    const uint8_t detail =
        elapsed < 500 ? (uint8_t)(elapsed * 255UL / 500) : 255;

    if (config.theme == STEPSCREEN_SPLASH_TREADMILL) {
      // Belt scroll + gait start once the frame is complete
      const uint8_t beltPhase =
          elapsed >= 600 ? (uint8_t)((elapsed / 60) & 7) : 0;
      const bool gait = elapsed >= 600 && ((elapsed / 120) & 1);
      drawTreadmill(display, detail, beltPhase, gait);
    } else {
      // Plunger depression: ease-out over 500-1400 ms, then hold
      int16_t plunger = 0;
      if (elapsed >= 1400) {
        plunger = PLUNGER_MAX;
      } else if (elapsed > 500) {
        const int32_t inv = 900 - (int32_t)(elapsed - 500);
        plunger =
            (int16_t)(PLUNGER_MAX - PLUNGER_MAX * inv * inv / (900L * 900L));
      }

      drawSyringe(display, detail, plunger);

      // Droplets exit the needle while the plunger is being depressed
      if (elapsed >= 900 && elapsed < 2400) {
        const uint8_t phase =
            (uint8_t)(((elapsed - 900) % 380) * 255UL / 380);
        drawDrops(display, phase);
      }
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
