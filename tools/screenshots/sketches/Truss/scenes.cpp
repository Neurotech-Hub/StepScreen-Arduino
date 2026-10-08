// Screenshot scenes for examples/Truss.
//
//   tools/screenshots/render.sh Truss
//
// The sketch is compiled unmodified; scenes drive it with simulated buttons,
// encoder, and load, and may read or set its globals directly.

#include <StepScreenShot.h>
#include <StepScreenSim.h>

#include "../../../../examples/Truss/Truss.ino"

namespace sim = StepScreenSim;
using StepScreenShot::expect;
using StepScreenShot::save;

namespace {

// Press and release, then give debounce + StepScreenNav time to act.
void pressButton(uint8_t pin) {
  sim::press(pin);
  sim::run(loop, 400);
}

// Blinking states (FAULT, SAVED) toggle every periodMs; run until the
// requested phase has been on screen long enough to be redrawn.
void runToBlinkPhase(uint32_t periodMs, bool inverted) {
  const bool ok = sim::runUntil(
      loop,
      [&] {
        const uint32_t t = millis() % (2 * periodMs);
        const uint32_t start = inverted ? periodMs : 0;
        return t >= start + DRAW_INTERVAL_MS + 10 && t < start + periodMs - 10;
      },
      4 * periodMs);
  expect(ok, "blink phase reached");
}

} // namespace

void stepscreenScenes() {
  sim::sdWrite(TRUSS_SD_CAL_PATH, "O=8000,S=420.000000\n");
  sim::hw().loadGrams = 2.0f;

  // Boot. Keep the last splash frame (title and credit fully drawn).
  StepScreenShot::Frame splash;
  sim::onDisplay([&] {
    if (!StepScreenShot::lastFrame().blank()) {
      splash = StepScreenShot::lastFrame();
    }
  });
  setup();
  sim::onDisplay(nullptr);
  save("01-splash", splash);

  sim::run(loop, 500);
  expect(currentScreen == SCREEN_HOME && loadCalibrated,
         "boots to calibrated Home");
  save("02-home");

  sim::turnEncoder(50);
  sim::run(loop, 300);
  expect(targetGrams == 150, "encoder raises target to 150 g");
  save("03-home-target");
  sim::turnEncoder(-50);
  sim::run(loop, 300);

  // No calibration on the card
  sim::sdFiles().erase(sim::sdKey(TRUSS_SD_CAL_PATH));
  loadCalibrated = false;
  displayDirty = true;
  sim::run(loop, 300);
  save("04-home-no-cal");

  // Missing hardware while on Home
  sim::hw().sdCard = false;
  sim::hw().loadCell = false;
  loadCellReady = false;
  motorReady = false;
  sim::run(loop, 500);
  expect(!sdReady, "SD removal detected");
  save("05-home-missing-hw");

  // Boot-time warning (setup() shows this when hardware is missing; any
  // button continues)
  sim::pressAt(PIN_BTN_BACK, millis() + 200);
  haltHardwareWarning();
  save("06-boot-missing-hw");

  sim::hw().sdCard = true;
  sim::hw().loadCell = true;
  motorReady = true;
  sim::run(loop, 500);
  expect(sdReady && loadCellReady, "hardware restored");

  // Calibration: tare empty, then hang the default 200 g weight
  sim::hw().loadGrams = 0.0f;
  pressButton(PIN_BTN_BACK);
  expect(currentScreen == SCREEN_CAL && calPhase == CAL_TARE,
         "CAL opens the tare step");
  save("07-cal-tare");

  pressButton(PIN_BTN_CONFIRM);
  expect(calPhase == CAL_WEIGHT, "tare advances to the weight step");
  save("08-cal-weight");

  sim::hw().loadGrams = (float)calGrams;
  sim::press(PIN_BTN_CONFIRM);
  expect(sim::runUntil(loop, [] { return calPhase == CAL_SAVED; }, 1000),
         "calibration saved");
  runToBlinkPhase(200, true);
  save("09-cal-saved");

  sim::run(loop, SAVED_FLASH_MS + 100);
  pressButton(PIN_BTN_BACK);
  expect(currentScreen == SCREEN_HOME && loadCalibrated,
         "back on Home, calibrated");

  // Run session holding at target
  sim::hw().loadGrams = (float)targetGrams;
  pressButton(PIN_BTN_CONFIRM);
  expect(currentScreen == SCREEN_RUN && runPhase == PHASE_RUNNING,
         "RUN starts a session");
  sim::runUntil(loop, [] { return sessionElapsedMs >= 83200; }, 90000);
  save("10-run");

  pressButton(PIN_BTN_PUSH);
  expect(runPhase == PHASE_HELD, "HOLD pauses the session");
  save("11-run-held");

  // Resume, then the load drops far below target and winding makes no
  // progress, so the direction timeout trips.
  pressButton(PIN_BTN_PUSH);
  expect(runPhase == PHASE_RUNNING, "GO resumes the session");
  sim::hw().loadGrams = 40.0f;
  expect(sim::runUntil(loop, [] { return runPhase == PHASE_FAULT; },
                       TRUSS_DIR_TIMEOUT_MS + 5000),
         "direction timeout faults the session");
  runToBlinkPhase(500, false);
  save("12-run-fault");
  runToBlinkPhase(500, true);
  save("13-run-fault-flash");
}
