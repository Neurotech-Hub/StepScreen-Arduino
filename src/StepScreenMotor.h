/*!
 * @file StepScreenMotor.h
 *
 * Step/dir driver for a BIGTREETECH TMC2209 module in standalone mode.
 *
 * Hardware assumptions (configured by strap pins, not software):
 *   MS1 + MS2 tied HIGH  -> 1/16 microstepping
 *   CLK tied GND         -> internal oscillator
 *   UART / DIAG          -> not connected
 *
 * The driver exposes STEP, DIR, and ~EN (active LOW). No UART configuration
 * is performed; this class generates step pulses only.
 *
 * Include independently of StepScreen.h:
 *   #include <StepScreenMotor.h>
 */

#ifndef STEPSCREEN_MOTOR_H
#define STEPSCREEN_MOTOR_H

#include <Arduino.h>

#include "StepScreenBoard.h"

class StepScreenMotor {
public:
  void begin(uint8_t stepPin = PIN_MOTOR_STEP, uint8_t dirPin = PIN_MOTOR_DIR,
             uint8_t enPin = PIN_MOTOR_EN);

  // ~EN is active LOW
  void enable();
  void disable();
  bool isEnabled() const { return _enabled; }

  // true = forward (positive position), false = reverse
  void setDirection(bool forward);
  bool getDirection() const { return _directionForward; }

  // Issue one STEP pulse (respects current direction for position tracking)
  void step();

  // Blocking move: steps sign follows direction set by setDirection()
  void moveSteps(uint32_t steps, uint32_t stepDelayUs = 500);

  // Signed blocking move: positive = forward, negative = reverse
  void moveStepsSigned(int32_t steps, uint32_t stepDelayUs = 500);

  // Non-blocking speed control (call runSpeed() from loop())
  void setSpeed(float stepsPerSecond);
  float getSpeed() const { return _speedStepsPerSec; }
  bool runSpeed(); // returns true if a step was issued this call

  int32_t getPosition() const { return _position; }
  void setPosition(int32_t position) { _position = position; }
  void resetPosition() { _position = 0; }

  // Total STEP pulses issued since begin() (always increases)
  uint32_t getStepCount() const { return _stepCount; }

  // Minimum HIGH/LOW time on STEP in microseconds (TMC2209 needs ~100 ns)
  void setMinPulseWidthUs(uint16_t us) { _minPulseWidthUs = us; }

  uint8_t stepPin() const { return _stepPin; }
  uint8_t dirPin() const { return _dirPin; }
  uint8_t enPin() const { return _enPin; }

private:
  void pulseStep();

  uint8_t _stepPin = PIN_MOTOR_STEP;
  uint8_t _dirPin = PIN_MOTOR_DIR;
  uint8_t _enPin = PIN_MOTOR_EN;

  bool _enabled = false;
  bool _directionForward = true;

  int32_t _position = 0;
  uint32_t _stepCount = 0;

  float _speedStepsPerSec = 0.0f;
  uint32_t _lastStepUs = 0;

  uint16_t _minPulseWidthUs = 2;
};

#endif // STEPSCREEN_MOTOR_H
