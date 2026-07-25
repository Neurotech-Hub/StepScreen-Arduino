#include "StepScreenMotor.h"

void StepScreenMotor::begin(uint8_t stepPin, uint8_t dirPin, uint8_t enPin) {
  _stepPin = stepPin;
  _dirPin = dirPin;
  _enPin = enPin;

  pinMode(_stepPin, OUTPUT);
  pinMode(_dirPin, OUTPUT);
  pinMode(_enPin, OUTPUT);

  digitalWrite(_stepPin, LOW);
  digitalWrite(_dirPin, LOW);

  disable(); // start with driver disabled (~EN HIGH)

  _position = 0;
  _stepCount = 0;
  _speedStepsPerSec = 0.0f;
  _lastStepUs = micros();
}

void StepScreenMotor::enable() {
  digitalWrite(_enPin, LOW);
  _enabled = true;
}

void StepScreenMotor::disable() {
  digitalWrite(_enPin, HIGH);
  _enabled = false;
}

void StepScreenMotor::setDirection(bool forward) {
  _directionForward = forward;
  digitalWrite(_dirPin, forward ? HIGH : LOW);
}

void StepScreenMotor::pulseStep() {
  digitalWrite(_stepPin, HIGH);
  delayMicroseconds(_minPulseWidthUs);
  digitalWrite(_stepPin, LOW);
  delayMicroseconds(_minPulseWidthUs);
  _stepCount++;
}

void StepScreenMotor::step() {
  pulseStep();
  _position += _directionForward ? 1 : -1;
}

void StepScreenMotor::moveSteps(uint32_t steps, uint32_t stepDelayUs) {
  if (steps == 0 || !_enabled) {
    return;
  }
  for (uint32_t i = 0; i < steps; i++) {
    step();
    if (stepDelayUs > 0) {
      delayMicroseconds(stepDelayUs);
    }
  }
}

void StepScreenMotor::moveStepsSigned(int32_t steps, uint32_t stepDelayUs) {
  if (steps == 0) {
    return;
  }
  setDirection(steps > 0);
  moveSteps((uint32_t)abs(steps), stepDelayUs);
}

void StepScreenMotor::setSpeed(float stepsPerSecond) {
  _speedStepsPerSec = stepsPerSecond;
  _lastStepUs = micros();
}

bool StepScreenMotor::runSpeed() {
  if (!_enabled || _speedStepsPerSec == 0.0f) {
    return false;
  }

  const float absSpeed = (_speedStepsPerSec >= 0.0f) ? _speedStepsPerSec
                                                     : -_speedStepsPerSec;
  if (absSpeed <= 0.0f) {
    return false;
  }

  setDirection(_speedStepsPerSec >= 0.0f);

  const uint32_t intervalUs = (uint32_t)(1000000.0f / absSpeed);
  const uint32_t now = micros();
  if ((uint32_t)(now - _lastStepUs) >= intervalUs) {
    _lastStepUs = now;
    step();
    return true;
  }
  return false;
}
