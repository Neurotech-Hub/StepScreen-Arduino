/*!
 * @file StepScreenHX711.h
 *
 * HX711 load cell driver for the StepScreen board, using SWCLK/SWDIO
 * (PA30/PA31). These pads are not mapped in the stock Feather M0 variant,
 * so this class bit-bangs the HX711 protocol via SAMD PORT registers.
 *
 * API mirrors Rob Tillaart's HX711 library (tare, set_scale, get_units).
 *
 *   StepScreenHX711 scale;
 *   scale.begin();
 *   scale.set_scale(420.0f);  // raw counts per gram — calibrate once
 *   scale.tare(10);
 *   float grams = scale.get_units(3);
 */

#ifndef STEPSCREEN_HX711_H
#define STEPSCREEN_HX711_H

#include <Arduino.h>

#include "StepScreenBoard.h"

class StepScreenHX711 {
public:
  void begin();

  bool is_ready();
  float read();
  float read_average(uint8_t times = 10);

  void tare(uint8_t times = 10);
  bool set_scale(float scale);
  float get_scale() const;
  void set_offset(int32_t offset);
  int32_t get_offset() const;

  float get_value(uint8_t times = 1);
  float get_units(uint8_t times = 1);

  // Assumes tare() was called. Sets scale from a known weight in grams.
  void calibrate_scale(float weight, uint8_t times = 10);

  void power_down();
  void power_up();

private:
  void configureGpio();
  uint8_t shiftIn();

  int32_t _offset = 0;
  float _scale = 1.0f;
  uint8_t _gain = 128;
  bool _fastProcessor = true;
};

#endif // STEPSCREEN_HX711_H
