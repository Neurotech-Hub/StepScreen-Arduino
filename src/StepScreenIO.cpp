#include "StepScreenIO.h"

#include "StepScreenInput.h"

// PIN_LED_RED omitted when it shares a pin with PIN_USER_LED1
#if PIN_LED_RED == PIN_USER_LED1
#define STEPSCREEN_IO_HAS_SEPARATE_RED 0
#else
#define STEPSCREEN_IO_HAS_SEPARATE_RED 1
#endif

const StepScreenIO::OutputChannel StepScreenIO::_channels[] = {
    {PIN_USER_LED1, "LED1", false, false},
    {PIN_USER_LED2, "LED2", false, false},
    {PIN_EXT_LED, "EXT", false, false},
    {PIN_LED_GREEN, "GRN", false, false},
#if STEPSCREEN_IO_HAS_SEPARATE_RED
    {PIN_LED_RED, "RED", false, false},
#endif
    {PIN_MOTOR_EN, "MEN", true, false},  // ~EN active LOW
    {PIN_MOTOR_DIR, "MDIR", false, false},
    {PIN_MOTOR_STEP, "STEP", false, true},
};

const uint8_t StepScreenIO::_channelCount =
    sizeof(_channels) / sizeof(_channels[0]);

void StepScreenIO::begin() {
  for (uint8_t i = 0; i < _channelCount; i++) {
    pinMode(_channels[i].pin, OUTPUT);
    _logicalOn[i] = false;
    writePin(_channels[i], false);
  }

  pinMode(PIN_BEAM_DET, INPUT); // external pull-up
  pinMode(PIN_AUX_IN, INPUT);
  pinMode(PIN_SD_CD, INPUT_PULLUP);
}

uint8_t StepScreenIO::outputCount() const { return _channelCount; }

const char *StepScreenIO::outputLabel(uint8_t index) const {
  if (index >= _channelCount) {
    return "?";
  }
  return _channels[index].label;
}

bool StepScreenIO::getOutput(uint8_t index) const {
  if (index >= _channelCount) {
    return false;
  }
  return _logicalOn[index];
}

void StepScreenIO::setOutput(uint8_t index, bool on) {
  if (index >= _channelCount) {
    return;
  }
  _logicalOn[index] = on;
}

void StepScreenIO::setAllOutputs(bool on) {
  for (uint8_t i = 0; i < _channelCount; i++) {
    _logicalOn[i] = on;
  }
}

bool StepScreenIO::isStepOutput(uint8_t index) const {
  if (index >= _channelCount) {
    return false;
  }
  return _channels[index].isStepPin;
}

uint8_t StepScreenIO::soloOutput(uint8_t index) {
  index = index % _channelCount;
  for (uint8_t i = 0; i < _channelCount; i++) {
    _logicalOn[i] = (i == index);
  }
  return index;
}

void StepScreenIO::writePin(const OutputChannel &ch, bool logicalOn) const {
  if (ch.isStepPin) {
    digitalWrite(ch.pin, LOW);
    return;
  }
  const bool level = logicalOn ? !ch.activeLow : ch.activeLow;
  digitalWrite(ch.pin, level ? HIGH : LOW);
}

void StepScreenIO::apply() {
  for (uint8_t i = 0; i < _channelCount; i++) {
    if (!_channels[i].isStepPin) {
      writePin(_channels[i], _logicalOn[i]);
    }
  }
}

void StepScreenIO::serviceStepOutput() {
  for (uint8_t i = 0; i < _channelCount; i++) {
    if (_channels[i].isStepPin && _logicalOn[i]) {
      digitalWrite(_channels[i].pin, HIGH);
      delayMicroseconds(2);
      digitalWrite(_channels[i].pin, LOW);
      return;
    }
  }
}

float StepScreenIO::readVbatVolts() {
  const int raw = analogRead(PIN_VBAT);
  return raw * (2.0f * 3.3f / 1024.0f); // divider is VBAT/2 on Adalogger
}

void StepScreenIO::readInputs(StepScreenInputs &inputs,
                              StepScreenInput *screenInput) {
  inputs.beamDet = digitalRead(PIN_BEAM_DET) == HIGH;
  inputs.auxIn = digitalRead(PIN_AUX_IN) == HIGH;
  inputs.sdCard = digitalRead(PIN_SD_CD) == HIGH;
  inputs.vbatVolts = readVbatVolts();

  if (screenInput != nullptr) {
    inputs.btnBack = screenInput->readBack();
    inputs.btnConfirm = screenInput->readConfirm();
    inputs.btnPush = screenInput->readPush();
    inputs.encoder = screenInput->getEncoderCount();
  }
}
