/*!
 * @file StepScreenDisplay.h
 *
 * SH1106 display wrapper with StepScreen layout helpers. Subclasses
 * Adafruit_SH1106G, so the full Adafruit_GFX API (print, drawLine,
 * fillRect, ...) is available directly on this object.
 *
 * Layout helpers use the zones defined in StepScreenLayout.h and always
 * draw at text size 1; call setTextSize() again afterwards if your
 * content uses a larger font.
 */

#ifndef STEPSCREEN_DISPLAY_H
#define STEPSCREEN_DISPLAY_H

#include <Adafruit_SH110X.h>
#include <Wire.h>

#include "StepScreenLayout.h"
#include "StepScreenPins.h"

class StepScreenDisplay : public Adafruit_SH1106G {
public:
  StepScreenDisplay(TwoWire *wire = &Wire)
      : Adafruit_SH1106G(STEPSCREEN_W, STEPSCREEN_H, wire,
                         STEPSCREEN_OLED_RESET) {}

  // Initializes Wire + the panel, clears the splash buffer, and sets
  // white size-1 text. Returns false if the display is not responding.
  bool begin(uint8_t i2cAddr = STEPSCREEN_I2C_ADDR);

  // Draws the inverted top bar with a left-aligned title (max ~16
  // chars) and the Back label in the action column.
  void drawInfoBar(const char *title, bool highlightBack = false);

  // Draws the three right-justified button labels: Back (top),
  // Sel (encoder push, middle), OK (confirm, bottom). Active labels are
  // drawn inverted (white box, black text).
  void drawActionColumn(bool backActive, bool pushActive,
                        bool confirmActive);

  // Clears only the content zone and parks the cursor at its origin.
  void clearContentArea();

  // Draws zone borders and corner markers for visual verification
  // (used by the ScreenTest example).
  void drawLayoutGuides();

private:
  void drawActionLabel(const char *label, int16_t y, bool active);
};

#endif // STEPSCREEN_DISPLAY_H
