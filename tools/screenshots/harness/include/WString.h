// Minimal Arduino String for host builds (enough for Adafruit_GFX and
// typical sketch formatting).

#ifndef STEPSCREEN_HOST_WSTRING_H
#define STEPSCREEN_HOST_WSTRING_H

#include <stdint.h>
#include <stdlib.h>
#include <string>

class __FlashStringHelper;

class String {
public:
  String(const char *s = "") : _s(s ? s : "") {}
  String(const __FlashStringHelper *s)
      : _s(s ? reinterpret_cast<const char *>(s) : "") {}
  String(const std::string &s) : _s(s) {}
  explicit String(char c) : _s(1, c) {}
  explicit String(int v) : _s(std::to_string(v)) {}
  explicit String(unsigned int v) : _s(std::to_string(v)) {}
  explicit String(long v) : _s(std::to_string(v)) {}
  explicit String(unsigned long v) : _s(std::to_string(v)) {}
  String(float v, unsigned char decimals = 2) { setFloat(v, decimals); }
  String(double v, unsigned char decimals = 2) { setFloat(v, decimals); }

  unsigned int length() const { return (unsigned int)_s.size(); }
  const char *c_str() const { return _s.c_str(); }
  char operator[](unsigned int i) const { return i < _s.size() ? _s[i] : 0; }
  char charAt(unsigned int i) const { return (*this)[i]; }

  String &operator+=(const String &rhs) {
    _s += rhs._s;
    return *this;
  }
  String &operator+=(const char *rhs) {
    _s += rhs ? rhs : "";
    return *this;
  }
  String &operator+=(char c) {
    _s += c;
    return *this;
  }
  bool concat(const String &rhs) {
    _s += rhs._s;
    return true;
  }

  bool operator==(const String &rhs) const { return _s == rhs._s; }
  bool operator!=(const String &rhs) const { return _s != rhs._s; }
  bool equals(const String &rhs) const { return _s == rhs._s; }

  int indexOf(char c, unsigned int from = 0) const {
    const size_t i = _s.find(c, from);
    return i == std::string::npos ? -1 : (int)i;
  }
  String substring(unsigned int from) const { return String(_s.substr(from)); }
  String substring(unsigned int from, unsigned int to) const {
    return String(_s.substr(from, to > from ? to - from : 0));
  }
  long toInt() const { return strtol(_s.c_str(), nullptr, 10); }
  float toFloat() const { return strtof(_s.c_str(), nullptr); }
  void trim();
  void toUpperCase();
  void toLowerCase();

  friend String operator+(String lhs, const String &rhs) {
    lhs += rhs;
    return lhs;
  }

private:
  void setFloat(double v, unsigned char decimals);
  std::string _s;
};

#endif // STEPSCREEN_HOST_WSTRING_H
