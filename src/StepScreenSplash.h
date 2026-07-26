/*!
 * @file StepScreenSplash.h
 *
 * Neuroscience-themed boot splash for the 128x64 SH1106 panel. Uses only
 * Adafruit_GFX — a minimal neuron with an action-potential pulse, then
 * title and status in separate vertical bands (no overlapping layers).
 *
 *   #include <StepScreenSplash.h>
 *
 *   StepScreenSplash::play(screen.display());
 *
 * Optional title/credit overrides and button-skip (pass StepScreenInput*).
 */

#ifndef STEPSCREEN_SPLASH_H
#define STEPSCREEN_SPLASH_H

#include "StepScreenDisplay.h"

class StepScreenInput;

struct StepScreenSplashConfig {
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
