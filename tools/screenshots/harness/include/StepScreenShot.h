/*!
 * Screenshot capture for host scenes. Each scene file defines
 *
 *   void stepscreenScenes();
 *
 * which the harness calls once. save() writes the most recently displayed
 * frame (what the OLED would show right now) to <out>/<name>.png.
 */

#ifndef STEPSCREEN_SHOT_H
#define STEPSCREEN_SHOT_H

#include <stdint.h>

#include <string>

namespace StepScreenShot {

struct Frame {
  static const int W = 128;
  static const int H = 64;
  uint8_t pixels[W * H / 8] = {}; // SH110X page layout
  bool inverted = false;
  uint32_t ms = 0; // virtual time the frame was displayed

  bool pixel(int x, int y) const {
    const bool on = (pixels[x + (y / 8) * W] >> (y & 7)) & 1;
    return on != inverted;
  }
  bool blank() const;
};

// Last frame sent to the panel with display().
const Frame &lastFrame();

void save(const char *name);
void save(const char *name, const Frame &frame);

// Abort the run with a message if a scene step did not reach the expected
// state (e.g. a button press did not change screens).
void expect(bool condition, const char *what);

// Output settings (set from the command line).
void setOutputDir(const std::string &dir);
void setScale(int scale);

} // namespace StepScreenShot

// Defined by sketches/<Sketch>/scenes.cpp
void stepscreenScenes();

#endif // STEPSCREEN_SHOT_H
