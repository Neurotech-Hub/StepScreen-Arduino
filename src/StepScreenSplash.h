/*!
 * @file StepScreenSplash.h
 *
 * Boot splash for the 128x64 SH1106 panel. Uses only Adafruit_GFX.
 * Two themes share the same layout (animated graphic in the upper band,
 * title and credit in separate vertical bands below):
 *
 *   - SYRINGE: a horizontal syringe is depressed (plunger travels,
 *     fluid empties, droplets exit the needle)
 *   - TREADMILL: a side-view treadmill with a rodent running on the
 *     belt (scrolling belt dashes, two-phase leg gait)
 *
 *   #include <StepScreenSplash.h>
 *
 *   StepScreenSplash::play(screen.display());
 *
 * Optional theme/title/credit overrides and button-skip (pass
 * StepScreenInput*).
 */

#ifndef STEPSCREEN_SPLASH_H
#define STEPSCREEN_SPLASH_H

#include "StepScreenDisplay.h"

class StepScreenInput;

enum StepScreenSplashTheme : uint8_t {
  STEPSCREEN_SPLASH_SYRINGE = 0,
  STEPSCREEN_SPLASH_TREADMILL = 1,
};

struct StepScreenSplashConfig {
  StepScreenSplashTheme theme = STEPSCREEN_SPLASH_SYRINGE;
  const char *title = "Syringe Pump v1.0";
  const char *credit = "by the Neurotech Hub";
  uint16_t durationMs = 3200;
  bool skippable = true;
};

class StepScreenSplash {
public:
  // Blocking animation; returns true if played to completion, false if
  // skipped via button press (when skippable and input is non-null).
  static bool play(StepScreenDisplay &display,
                   const StepScreenSplashConfig &config,
                   StepScreenInput *input = nullptr);

  static bool play(StepScreenDisplay &display, const char *title,
                   const char *subtitle = nullptr,
                   StepScreenInput *input = nullptr);
};

#endif // STEPSCREEN_SPLASH_H
