// Host fake of the RTClib subset StepScreen uses: DateTime, TimeSpan and
// RTC_PCF8523. The clock reads StepScreenSim::hw().rtc* plus virtual
// millis(), so screenshots show a fixed, reproducible date and time.

#ifndef _RTCLIB_H_
#define _RTCLIB_H_

#include <Arduino.h>
#include <Wire.h>

#define SECONDS_FROM_1970_TO_2000 946684800UL

class TimeSpan {
public:
  TimeSpan(int32_t seconds = 0) : _seconds(seconds) {}
  TimeSpan(int16_t days, int8_t hours, int8_t minutes, int8_t seconds)
      : _seconds((int32_t)days * 86400L + (int32_t)hours * 3600 +
                 (int32_t)minutes * 60 + seconds) {}

  int16_t days() const { return _seconds / 86400L; }
  int8_t hours() const { return _seconds / 3600 % 24; }
  int8_t minutes() const { return _seconds / 60 % 60; }
  int8_t seconds() const { return _seconds % 60; }
  int32_t totalseconds() const { return _seconds; }

  TimeSpan operator+(const TimeSpan &right) const {
    return TimeSpan(_seconds + right._seconds);
  }
  TimeSpan operator-(const TimeSpan &right) const {
    return TimeSpan(_seconds - right._seconds);
  }

private:
  int32_t _seconds;
};

class DateTime {
public:
  DateTime(uint32_t t = SECONDS_FROM_1970_TO_2000);
  DateTime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour = 0,
           uint8_t min = 0, uint8_t sec = 0);
  // __DATE__ ("Oct  8 2026") and __TIME__ ("09:41:00") formats
  DateTime(const char *date, const char *time);

  bool isValid() const;
  char *toString(char *buffer) const;

  uint16_t year() const { return 2000U + _yOff; }
  uint8_t month() const { return _m; }
  uint8_t day() const { return _d; }
  uint8_t hour() const { return _hh; }
  uint8_t twelveHour() const;
  uint8_t isPM() const { return _hh >= 12; }
  uint8_t minute() const { return _mm; }
  uint8_t second() const { return _ss; }
  uint8_t dayOfTheWeek() const;

  uint32_t secondstime() const;
  uint32_t unixtime() const;

  DateTime operator+(const TimeSpan &span) const {
    return DateTime(unixtime() + span.totalseconds());
  }
  DateTime operator-(const TimeSpan &span) const {
    return DateTime(unixtime() - span.totalseconds());
  }
  TimeSpan operator-(const DateTime &right) const {
    return TimeSpan((int32_t)(unixtime() - right.unixtime()));
  }
  bool operator<(const DateTime &right) const {
    return unixtime() < right.unixtime();
  }
  bool operator>(const DateTime &right) const { return right < *this; }
  bool operator<=(const DateTime &right) const { return !(*this > right); }
  bool operator>=(const DateTime &right) const { return !(*this < right); }
  bool operator==(const DateTime &right) const {
    return unixtime() == right.unixtime();
  }
  bool operator!=(const DateTime &right) const { return !(*this == right); }

private:
  uint8_t _yOff = 0, _m = 1, _d = 1, _hh = 0, _mm = 0, _ss = 0;
};

class RTC_PCF8523 {
public:
  bool begin(TwoWire *wire = &Wire);
  bool initialized();
  bool lostPower();
  void adjust(const DateTime &dt);
  DateTime now();
  void start() {}
  void stop() {}
  uint8_t isrunning() { return 1; }
};

#endif // _RTCLIB_H_
