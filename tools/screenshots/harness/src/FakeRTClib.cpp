#include <RTClib.h>

#include "SimInternal.h"
#include "StepScreenSim.h"

namespace {

// Days from 1970-01-01 to y-m-d (proleptic Gregorian).
int64_t daysFromCivil(int64_t y, unsigned m, unsigned d) {
  y -= m <= 2;
  const int64_t era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = (unsigned)(y - era * 400);
  const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + (int64_t)doe - 719468;
}

void civilFromDays(int64_t z, int64_t &y, unsigned &m, unsigned &d) {
  z += 719468;
  const int64_t era = (z >= 0 ? z : z - 146096) / 146097;
  const unsigned doe = (unsigned)(z - era * 146097);
  const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  y = (int64_t)yoe + era * 400;
  const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const unsigned mp = (5 * doy + 2) / 153;
  d = doy - (153 * mp + 2) / 5 + 1;
  m = mp < 10 ? mp + 3 : mp - 9;
  y += m <= 2;
}

uint8_t conv2d(const char *p) {
  uint8_t v = 0;
  if ('0' <= *p && *p <= '9') {
    v = (uint8_t)(*p - '0');
  }
  return (uint8_t)(10 * v + *++p - '0');
}

int64_t g_adjustSeconds = 0;

} // namespace

DateTime::DateTime(uint32_t t) {
  const int64_t days = t / 86400;
  const uint32_t rem = t % 86400;
  int64_t y;
  unsigned m, d;
  civilFromDays(days, y, m, d);
  _yOff = (uint8_t)(y - 2000);
  _m = (uint8_t)m;
  _d = (uint8_t)d;
  _hh = (uint8_t)(rem / 3600);
  _mm = (uint8_t)(rem / 60 % 60);
  _ss = (uint8_t)(rem % 60);
}

DateTime::DateTime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour,
                   uint8_t min, uint8_t sec) {
  if (year >= 2000U) {
    year -= 2000U;
  }
  _yOff = (uint8_t)year;
  _m = month;
  _d = day;
  _hh = hour;
  _mm = min;
  _ss = sec;
}

DateTime::DateTime(const char *date, const char *time) {
  _yOff = conv2d(date + 9);
  switch (date[0]) {
  case 'J':
    _m = (date[1] == 'a') ? 1 : ((date[2] == 'n') ? 6 : 7);
    break;
  case 'F':
    _m = 2;
    break;
  case 'A':
    _m = date[2] == 'r' ? 4 : 8;
    break;
  case 'M':
    _m = date[2] == 'r' ? 3 : 5;
    break;
  case 'S':
    _m = 9;
    break;
  case 'O':
    _m = 10;
    break;
  case 'N':
    _m = 11;
    break;
  case 'D':
    _m = 12;
    break;
  }
  _d = conv2d(date + 4);
  _hh = conv2d(time);
  _mm = conv2d(time + 3);
  _ss = conv2d(time + 6);
}

bool DateTime::isValid() const {
  if (_yOff >= 100 || _m < 1 || _m > 12 || _d < 1 || _d > 31 || _hh > 23 ||
      _mm > 59 || _ss > 59) {
    return false;
  }
  const DateTime same(unixtime());
  return same._m == _m && same._d == _d;
}

uint8_t DateTime::twelveHour() const {
  if (_hh == 0 || _hh == 12) {
    return 12;
  }
  return _hh > 12 ? _hh - 12 : _hh;
}

uint8_t DateTime::dayOfTheWeek() const {
  // 1970-01-01 was a Thursday (4); Sunday = 0
  return (uint8_t)((unixtime() / 86400 + 4) % 7);
}

uint32_t DateTime::unixtime() const {
  const int64_t days = daysFromCivil(2000 + _yOff, _m, _d);
  return (uint32_t)(days * 86400 + _hh * 3600 + _mm * 60 + _ss);
}

uint32_t DateTime::secondstime() const {
  return unixtime() - SECONDS_FROM_1970_TO_2000;
}

char *DateTime::toString(char *buffer) const {
  for (char *p = buffer; *p; p++) {
    char tmp[5];
    if (strncmp(p, "YYYY", 4) == 0) {
      snprintf(tmp, sizeof(tmp), "%04u", year());
      memcpy(p, tmp, 4);
      p += 3;
    } else if (strncmp(p, "YY", 2) == 0) {
      snprintf(tmp, sizeof(tmp), "%02u", _yOff);
      memcpy(p, tmp, 2);
      p += 1;
    } else if (strncmp(p, "MM", 2) == 0) {
      snprintf(tmp, sizeof(tmp), "%02u", _m);
      memcpy(p, tmp, 2);
      p += 1;
    } else if (strncmp(p, "DD", 2) == 0) {
      snprintf(tmp, sizeof(tmp), "%02u", _d);
      memcpy(p, tmp, 2);
      p += 1;
    } else if (strncmp(p, "hh", 2) == 0) {
      snprintf(tmp, sizeof(tmp), "%02u", _hh);
      memcpy(p, tmp, 2);
      p += 1;
    } else if (strncmp(p, "mm", 2) == 0) {
      snprintf(tmp, sizeof(tmp), "%02u", _mm);
      memcpy(p, tmp, 2);
      p += 1;
    } else if (strncmp(p, "ss", 2) == 0) {
      snprintf(tmp, sizeof(tmp), "%02u", _ss);
      memcpy(p, tmp, 2);
      p += 1;
    }
  }
  return buffer;
}

bool RTC_PCF8523::begin(TwoWire *) { return StepScreenSim::hw().rtc; }

bool RTC_PCF8523::initialized() { return true; }

bool RTC_PCF8523::lostPower() { return false; }

DateTime RTC_PCF8523::now() {
  const StepScreenSim::Hardware &hw = StepScreenSim::hw();
  const DateTime start(hw.year, hw.month, hw.day, hw.hour, hw.minute,
                       hw.second);
  const int64_t elapsed = (int64_t)(StepScreenSimInternal::nowUs() / 1000000ULL);
  return DateTime((uint32_t)(start.unixtime() + elapsed + g_adjustSeconds));
}

void RTC_PCF8523::adjust(const DateTime &dt) {
  if (!StepScreenSim::hw().rtcAllowAdjust) {
    return;
  }
  g_adjustSeconds += (int64_t)dt.unixtime() - (int64_t)now().unixtime();
}
