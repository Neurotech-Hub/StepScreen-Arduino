#include "StepScreenRTC.h"

#include <RTClib.h>
#include <stdio.h>

namespace {

RTC_PCF8523 rtc;

DateTime buildDateTime() { return DateTime(F(__DATE__), F(__TIME__)); }

void printDateTime(const DateTime &dt) {
  char buf[] = "YYYY-MM-DD hh:mm:ss";
  dt.toString(buf);
  Serial.print(buf);
}

} // namespace

bool StepScreenRTC::begin(TwoWire *wire) {
  _ready = rtc.begin(wire);
  return _ready;
}

bool StepScreenRTC::syncFromBuildTimeIfNeeded() {
  if (!_ready) {
    return false;
  }

  const DateTime buildTime = buildDateTime();
  const DateTime now = rtc.now();
  const bool needsSync = !rtc.initialized() || rtc.lostPower() ||
                         now.year() < 2020 ||
                         now.unixtime() < buildTime.unixtime();

  if (!needsSync) {
    return false;
  }

  rtc.adjust(buildTime);

  Serial.print(F("RTC synced to build time: "));
  printDateTime(buildTime);
  Serial.println();

  return true;
}

void StepScreenRTC::formatInfoBar(char *buf, size_t len) const {
  if (buf == nullptr || len == 0) {
    return;
  }

  if (!_ready) {
    snprintf(buf, len, "-- --:--");
    return;
  }

  const DateTime now = rtc.now();
  snprintf(buf, len, "%02u-%02u %02u:%02u", now.month(), now.day(),
           now.hour(), now.minute());
}

void StepScreenRTC::formatLogTimestamp(char *buf, size_t len) const {
  if (buf == nullptr || len == 0) {
    return;
  }

  if (!_ready) {
    buf[0] = '\0';
    return;
  }

  const DateTime now = rtc.now();
  snprintf(buf, len, "%04u-%02u-%02u %02u:%02u:%02u", now.year(), now.month(),
           now.day(), now.hour(), now.minute(), now.second());
}
