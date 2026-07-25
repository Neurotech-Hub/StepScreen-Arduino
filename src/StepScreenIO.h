/*!
 * @file StepScreenIO.h
 *
 * Board I/O manager: output registry, input sampling, and apply/read helpers.
 * Works with StepScreenInput for screen-module buttons and encoder.
 *
 *   StepScreenIO io;
 *   io.begin();
 *   io.setAllOutputs(true);
 *   io.apply();
 *
 *   StepScreenInputs inputs;
 *   io.readInputs(inputs, &screen.input());
 */

#ifndef STEPSCREEN_IO_H
#define STEPSCREEN_IO_H

#include <Arduino.h>

#include "StepScreenBoard.h"

class StepScreenInput;

struct StepScreenInputs {
  bool beamDet = false;
  bool auxIn = false;
  bool sdCard = false;
  bool btnBack = false;
  bool btnConfirm = false;
  bool btnPush = false;
  int32_t encoder = 0;
  float vbatVolts = 0.0f;
};

class StepScreenIO {
public:
  void begin();

  uint8_t outputCount() const;
  const char *outputLabel(uint8_t index) const;

  bool getOutput(uint8_t index) const;
  void setOutput(uint8_t index, bool on);
  void setAllOutputs(bool on);
  bool isStepOutput(uint8_t index) const;

  // Turn on only the selected output; all others off. Returns index.
  uint8_t soloOutput(uint8_t index);

  // Write logical output states to GPIO (call after changing outputs)
  void apply();

  // Issue one STEP pulse when the STEP output channel is logically on
  void serviceStepOutput();

  void readInputs(StepScreenInputs &inputs,
                  StepScreenInput *screenInput = nullptr);

  static float readVbatVolts();

private:
  struct OutputChannel {
    uint8_t pin;
    const char *label;
    bool activeLow;
    bool isStepPin;
  };

  void writePin(const OutputChannel &ch, bool logicalOn) const;

  static const OutputChannel _channels[];
  static const uint8_t _channelCount;

  bool _logicalOn[8] = {}; // max channel count
};

#endif // STEPSCREEN_IO_H
