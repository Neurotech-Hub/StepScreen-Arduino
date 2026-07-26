#include "StepScreenStepper.h"

void StepScreenStepper::begin() {
  // Invert only the enable pin: ~EN is active LOW on the TMC2209.
  // setPinsInverted must precede setEnablePin so the pin is driven with
  // the correct polarity when it is first configured.
  _stepper.setPinsInverted(false, false, true);
  _stepper.setEnablePin(_enPin);
  _stepper.setMinPulseWidth(3); // us; TMC2209 needs >= ~100 ns
  _stepper.setAcceleration(_accel);
  setSpeedPreset(_preset);
  disable(); // start with the driver off
}

void StepScreenStepper::enable() {
  _stepper.enableOutputs();
  _enabled = true;
}

void StepScreenStepper::disable() {
  // Cancel any pending motion so a later enable() doesn't resume an
  // old target (setCurrentPosition zeroes speed and target).
  _stepper.setCurrentPosition(_stepper.currentPosition());
  _stepper.disableOutputs();
  _enabled = false;
}

void StepScreenStepper::setSpeedPreset(StepScreenSpeedPreset preset) {
  _preset = preset;
  _stepper.setMaxSpeed(_presetSps[preset]);
}

StepScreenSpeedPreset StepScreenStepper::nextSpeedPreset() {
  setSpeedPreset((StepScreenSpeedPreset)((_preset + 1) % 3));
  return _preset;
}

const char *StepScreenStepper::speedPresetName() const {
  switch (_preset) {
  case STEPSCREEN_SPEED_LOW:
    return "Low";
  case STEPSCREEN_SPEED_MEDIUM:
    return "Med";
  default:
    return "Fast";
  }
}
