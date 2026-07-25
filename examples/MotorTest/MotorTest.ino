/*
  StepScreen MotorTest

  Exercises the TMC2209 step/dir interface without a motor connected.
  Toggles STEP, DIR, and ~EN on the configured pins and reports activity
  over Serial. The green LED blinks on each STEP pulse so you can verify
  output even without a scope.

  Hardware (BIGTREETECH TMC2209, standalone mode):
    STEP  -> D5  (PIN_MOTOR_STEP)
    DIR   -> D6  (PIN_MOTOR_DIR)
    ~EN   -> D9  (PIN_MOTOR_EN, active LOW)
    MS1/MS2 tied HIGH (1/16 microstepping, 3200 steps/rev for 200-step motor)
    CLK tied GND (internal clock)

  Sequence:
    1. Enable driver, move forward 320 microsteps (1/10 rev) blocking
    2. Pause, move reverse 320 microsteps blocking
    3. Run 640 microsteps at 800 steps/sec using non-blocking runSpeed()
    4. Disable driver, idle

  Board: Adafruit Feather M0 Adalogger (or compatible).
*/

#include <StepScreenMotor.h>

StepScreenMotor motor;

enum TestPhase {
  PHASE_FORWARD,
  PHASE_PAUSE_FWD,
  PHASE_REVERSE,
  PHASE_PAUSE_REV,
  PHASE_RUN_SPEED,
  PHASE_DONE
};

TestPhase phase = PHASE_FORWARD;
uint32_t phaseStartMs = 0;

// 320 microsteps = 1/10 revolution at 1/16 microstepping
const uint32_t TEST_STEPS = STEPSCREEN_MOTOR_STEPS_PER_REV / 10;
const uint32_t STEP_DELAY_US = 800;

void blinkStepLed() {
  digitalWrite(PIN_LED_GREEN, !digitalRead(PIN_LED_GREEN));
}

void reportState(const __FlashStringHelper *label) {
  Serial.print(label);
  Serial.print("  pos=");
  Serial.print(motor.getPosition());
  Serial.print("  steps=");
  Serial.print(motor.getStepCount());
  Serial.print("  dir=");
  Serial.print(motor.getDirection() ? "FWD" : "REV");
  Serial.print("  en=");
  Serial.println(motor.isEnabled() ? "ON" : "OFF");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_LED_GREEN, OUTPUT);
  digitalWrite(PIN_LED_GREEN, LOW);

  motor.begin();
  Serial.println(F("TMC2209 MotorTest (no motor required)"));
  Serial.print(F("  STEP pin "));
  Serial.println(motor.stepPin());
  Serial.print(F("  DIR  pin "));
  Serial.println(motor.dirPin());
  Serial.print(F("  ~EN  pin "));
  Serial.println(motor.enPin());
  Serial.print(F("  microsteps/rev "));
  Serial.println(STEPSCREEN_MOTOR_STEPS_PER_REV);

  motor.enable();
  reportState(F("Driver enabled"));
  phaseStartMs = millis();
}

void loop() {
  switch (phase) {
  case PHASE_FORWARD:
    Serial.print(F("Blocking forward "));
    Serial.print(TEST_STEPS);
    Serial.println(F(" steps..."));
    motor.setDirection(true);
    for (uint32_t i = 0; i < TEST_STEPS; i++) {
      motor.step();
      blinkStepLed();
      delayMicroseconds(STEP_DELAY_US);
    }
    reportState(F("Forward done"));
    phase = PHASE_PAUSE_FWD;
    phaseStartMs = millis();
    break;

  case PHASE_PAUSE_FWD:
    if (millis() - phaseStartMs >= 1500) {
      phase = PHASE_REVERSE;
    }
    break;

  case PHASE_REVERSE:
    Serial.print(F("Blocking reverse "));
    Serial.print(TEST_STEPS);
    Serial.println(F(" steps..."));
    motor.setDirection(false);
    for (uint32_t i = 0; i < TEST_STEPS; i++) {
      motor.step();
      blinkStepLed();
      delayMicroseconds(STEP_DELAY_US);
    }
    reportState(F("Reverse done"));
    phase = PHASE_PAUSE_REV;
    phaseStartMs = millis();
    break;

  case PHASE_PAUSE_REV:
    if (millis() - phaseStartMs >= 1500) {
      Serial.println(F("Non-blocking runSpeed() forward..."));
      motor.resetPosition();
      motor.setSpeed(800.0f);
      phase = PHASE_RUN_SPEED;
      phaseStartMs = millis();
    }
    break;

  case PHASE_RUN_SPEED: {
    const int32_t target = (int32_t)TEST_STEPS * 2;
    if (motor.runSpeed()) {
      blinkStepLed();
    }
    if (motor.getPosition() >= target) {
      motor.setSpeed(0.0f);
      reportState(F("runSpeed done"));
      phase = PHASE_DONE;
      phaseStartMs = millis();
    }
    break;
  }

  case PHASE_DONE:
    if (millis() - phaseStartMs < 100) {
      motor.disable();
      digitalWrite(PIN_LED_GREEN, LOW);
      reportState(F("Driver disabled, test complete"));
      Serial.println(F("Re-upload to repeat."));
    }
    break;
  }
}
