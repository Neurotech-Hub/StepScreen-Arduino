/*!
 * @file StepScreenStepper.h
 *
 * AccelStepper-based motion control for the TMC2209 (standalone
 * step/dir mode). Complements the low-level StepScreenMotor class:
 * use StepScreenStepper for application motion (acceleration,
 * non-blocking moves, speed presets) and StepScreenMotor for raw
 * pin-level testing.
 *
 * Requires the AccelStepper library (Library Manager).
 *
 * Include independently of StepScreen.h:
 *   #include <StepScreenStepper.h>
 *
 * Call run() from every loop() iteration; all moves are non-blocking.
 */

#ifndef STEPSCREEN_STEPPER_H
#define STEPSCREEN_STEPPER_H

#include <AccelStepper.h>
#include <Arduino.h>

#include "StepScreenBoard.h"

// Speed presets in microsteps/sec (1/16 microstepping -> 3200/rev).
// Override any of these before including the library.
#ifndef STEPSCREEN_SPEED_LOW_SPS
#define STEPSCREEN_SPEED_LOW_SPS 200.0f
#endif

#ifndef STEPSCREEN_SPEED_MEDIUM_SPS
#define STEPSCREEN_SPEED_MEDIUM_SPS 800.0f
#endif

#ifndef STEPSCREEN_SPEED_FAST_SPS
#define STEPSCREEN_SPEED_FAST_SPS 2000.0f
#endif

// Acceleration in microsteps/sec^2
#ifndef STEPSCREEN_STEPPER_ACCEL
#define STEPSCREEN_STEPPER_ACCEL 4000.0f
#endif

enum StepScreenSpeedPreset : uint8_t {
  STEPSCREEN_SPEED_LOW = 0,
  STEPSCREEN_SPEED_MEDIUM = 1,
  STEPSCREEN_SPEED_FAST = 2,
};

class StepScreenStepper {
public:
  // Constructor is inline so pin/speed macro defaults resolve in the
  // sketch's translation unit (sketch-level overrides apply).
  StepScreenStepper(uint8_t stepPin = PIN_MOTOR_STEP,
                    uint8_t dirPin = PIN_MOTOR_DIR,
                    uint8_t enPin = PIN_MOTOR_EN)
      : _stepper(AccelStepper::DRIVER, stepPin, dirPin, 4, 5,
                 /*enable=*/false),
        _enPin(enPin) {
    _presetSps[STEPSCREEN_SPEED_LOW] = STEPSCREEN_SPEED_LOW_SPS;
    _presetSps[STEPSCREEN_SPEED_MEDIUM] = STEPSCREEN_SPEED_MEDIUM_SPS;
    _presetSps[STEPSCREEN_SPEED_FAST] = STEPSCREEN_SPEED_FAST_SPS;
    _accel = STEPSCREEN_STEPPER_ACCEL;
  }

  // Configures pins (driver starts DISABLED) and applies the default
  // speed preset + acceleration.
  void begin();

  // ~EN is active LOW; handled via AccelStepper pin inversion.
  // disable() also cancels any pending motion.
  void enable();
  void disable();
  bool isEnabled() const { return _enabled; }

  // Direct speed control (presets are usually enough)
  void setMaxSpeed(float stepsPerSec) { _stepper.setMaxSpeed(stepsPerSec); }
  void setAcceleration(float stepsPerSec2) {
    _stepper.setAcceleration(stepsPerSec2);
  }

  // Speed presets (Low / Med / Fast)
  void setSpeedPreset(StepScreenSpeedPreset preset);
  StepScreenSpeedPreset nextSpeedPreset(); // cycles Low -> Med -> Fast -> Low
  StepScreenSpeedPreset speedPreset() const { return _preset; }
  const char *speedPresetName() const;
  float speedPresetSps() const { return _presetSps[_preset]; }

  // Relative moves (non-blocking; poll run() until done). jogSteps is
  // an alias intended for encoder-driven adjustment.
  void jogSteps(int32_t steps) { _stepper.move(steps); }
  void moveSigned(int32_t steps) { _stepper.move(steps); }

  // Call every loop() iteration; returns true while a move is pending
  bool run() { return _stepper.run(); }
  bool isRunning() { return _stepper.isRunning(); }

  // Decelerate to a stop as fast as the acceleration setting allows
  void stop() { _stepper.stop(); }

  int32_t currentPosition() { return _stepper.currentPosition(); }
  void resetPosition() { _stepper.setCurrentPosition(0); }
  int32_t distanceToGo() { return _stepper.distanceToGo(); }

  // Full AccelStepper API for advanced use
  AccelStepper &raw() { return _stepper; }

private:
  AccelStepper _stepper;
  uint8_t _enPin;
  bool _enabled = false;
  StepScreenSpeedPreset _preset = STEPSCREEN_SPEED_MEDIUM;
  float _presetSps[3];
  float _accel;
};

#endif // STEPSCREEN_STEPPER_H
