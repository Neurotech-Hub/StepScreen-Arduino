/*!
 * @file StepScreenRTC.h
 *
 * PCF8523 real-time clock helper for the Adafruit Feather M0 Adalogger
 * (or compatible). Shares Wire with the OLED at STEPSCREEN_RTC_I2C_ADDR
 * (default 0x68).
 *
 * Call begin() after the display (or any other I2C device) has started
 * Wire. syncFromBuildTimeIfNeeded() sets the clock from __DATE__/__TIME__
 * when the RTC is unset, lost power, or behind the firmware build time.
 *
 *   StepScreenRTC rtc;
 *   rtc.begin();
 *   rtc.syncFromBuildTimeIfNeeded();
 *
 *   char header[12];
 *   rtc.formatInfoBar(header, sizeof(header));
 *   screen.drawInfoBar(header);
 */

#ifndef STEPSCREEN_RTC_H
#define STEPSCREEN_RTC_H

#include <Arduino.h>
#include <Wire.h>

#include "StepScreenBoard.h"

class StepScreenRTC {
public:
  bool begin(TwoWire *wire = &Wire);

  bool isReady() const { return _ready; }

  // Sets time from firmware build stamp when RTC is unset or stale.
  // The defaults expand in the calling sketch, which is recompiled on every
  // upload; library objects are cached, so __DATE__/__TIME__ there go stale.
  bool syncFromBuildTimeIfNeeded(const char *buildDate = __DATE__,
                                 const char *buildTime = __TIME__);

  // "MM-DD HH:MM" or "-- --:--" when not ready.
  void formatInfoBar(char *buf, size_t len) const;

  // "YYYY-MM-DD HH:MM:SS" for log lines; empty string when not ready.
  void formatLogTimestamp(char *buf, size_t len) const;

private:
  bool _ready = false;
};

#endif // STEPSCREEN_RTC_H
