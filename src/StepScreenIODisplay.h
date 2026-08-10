/*!
 * @file StepScreenIODisplay.h
 *
 * On-screen rendering helpers for board I/O status. Uses the StepScreen
 * content zone layout from StepScreenLayout.h.
 */

#ifndef STEPSCREEN_IODISPLAY_H
#define STEPSCREEN_IODISPLAY_H

#include "StepScreenDisplay.h"
#include "StepScreenIO.h"

enum StepScreenIOPage : uint8_t {
  STEPSCREEN_IO_PAGE_OUTPUTS = 0,
  STEPSCREEN_IO_PAGE_INPUTS = 1,
  STEPSCREEN_IO_PAGE_SD = 2,
};

class StepScreenIODisplay {
public:
  // Draw one label/value row inside the content zone (8 px tall).
  static void drawStatusLine(StepScreenDisplay &d, int16_t y,
                             const char *label, const char *value,
                             bool highlight = false);

  // Full I/O panel for the IOTest sketch and similar tools.
  static void drawPanel(StepScreenDisplay &d, StepScreenIO &io,
                        const StepScreenInputs &inputs,
                        StepScreenIOPage page, int8_t highlightOutput = -1,
                        const char *footer = nullptr);

  // SD card status panel (IOTest SD page).
  static void drawSdPanel(StepScreenDisplay &d, class StepScreenSD &sd,
                          const char *footer = nullptr);
};

#endif // STEPSCREEN_IODISPLAY_H
