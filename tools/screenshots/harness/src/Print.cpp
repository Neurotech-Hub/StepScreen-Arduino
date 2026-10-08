// Print, Stream and String implementations for host builds.

#include <Arduino.h>

#include <ctype.h>
#include <stdarg.h>

size_t Print::write(const uint8_t *buffer, size_t size) {
  size_t n = 0;
  while (size--) {
    n += write(*buffer++);
  }
  return n;
}

size_t Print::write(const char *str) {
  return str ? write((const uint8_t *)str, strlen(str)) : 0;
}

size_t Print::printUnsigned(unsigned long long n, int base) {
  if (base < 2) {
    base = 10;
  }
  char buf[8 * sizeof(n) + 1];
  char *p = &buf[sizeof(buf) - 1];
  *p = '\0';
  do {
    const int digit = (int)(n % (unsigned)base);
    *--p = (char)(digit < 10 ? '0' + digit : 'A' + digit - 10);
    n /= (unsigned)base;
  } while (n);
  return write(p);
}

size_t Print::printSigned(long long n, int base) {
  if (base == 10 && n < 0) {
    return print('-') + printUnsigned((unsigned long long)(-n), 10);
  }
  return printUnsigned((unsigned long long)n, base);
}

size_t Print::print(const __FlashStringHelper *s) {
  return write(reinterpret_cast<const char *>(s));
}
size_t Print::print(const String &s) { return write(s.c_str()); }
size_t Print::print(const char s[]) { return write(s); }
size_t Print::print(char c) { return write((uint8_t)c); }
size_t Print::print(unsigned char n, int base) { return printUnsigned(n, base); }
size_t Print::print(int n, int base) { return printSigned(n, base); }
size_t Print::print(unsigned int n, int base) { return printUnsigned(n, base); }
size_t Print::print(long n, int base) { return printSigned(n, base); }
size_t Print::print(unsigned long n, int base) {
  return printUnsigned(n, base);
}
size_t Print::print(long long n, int base) { return printSigned(n, base); }
size_t Print::print(unsigned long long n, int base) {
  return printUnsigned(n, base);
}
size_t Print::print(double n, int digits) {
  char buf[64];
  snprintf(buf, sizeof(buf), "%.*f", digits < 0 ? 0 : digits, n);
  return write(buf);
}

size_t Print::println() { return write("\r\n"); }
size_t Print::println(const __FlashStringHelper *s) {
  return print(s) + println();
}
size_t Print::println(const String &s) { return print(s) + println(); }
size_t Print::println(const char s[]) { return print(s) + println(); }
size_t Print::println(char c) { return print(c) + println(); }
size_t Print::println(unsigned char n, int base) {
  return print(n, base) + println();
}
size_t Print::println(int n, int base) { return print(n, base) + println(); }
size_t Print::println(unsigned int n, int base) {
  return print(n, base) + println();
}
size_t Print::println(long n, int base) { return print(n, base) + println(); }
size_t Print::println(unsigned long n, int base) {
  return print(n, base) + println();
}
size_t Print::println(long long n, int base) {
  return print(n, base) + println();
}
size_t Print::println(unsigned long long n, int base) {
  return print(n, base) + println();
}
size_t Print::println(double n, int digits) {
  return print(n, digits) + println();
}

size_t Print::printf(const char *format, ...) {
  char buf[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);
  return write(buf);
}

size_t Stream::readBytes(char *buffer, size_t length) {
  size_t n = 0;
  while (n < length) {
    const int c = read();
    if (c < 0) {
      break;
    }
    buffer[n++] = (char)c;
  }
  return n;
}

size_t Stream::readBytesUntil(char terminator, char *buffer, size_t length) {
  size_t n = 0;
  while (n < length) {
    const int c = read();
    if (c < 0 || c == terminator) {
      break;
    }
    buffer[n++] = (char)c;
  }
  return n;
}

String Stream::readStringUntil(char terminator) {
  String s;
  while (true) {
    const int c = read();
    if (c < 0 || c == terminator) {
      break;
    }
    s += (char)c;
  }
  return s;
}

void String::setFloat(double v, unsigned char decimals) {
  char buf[64];
  snprintf(buf, sizeof(buf), "%.*f", decimals, v);
  _s = buf;
}

void String::trim() {
  size_t a = 0;
  while (a < _s.size() && isspace((unsigned char)_s[a])) {
    a++;
  }
  size_t b = _s.size();
  while (b > a && isspace((unsigned char)_s[b - 1])) {
    b--;
  }
  _s = _s.substr(a, b - a);
}

void String::toUpperCase() {
  for (char &c : _s) {
    c = (char)toupper((unsigned char)c);
  }
}

void String::toLowerCase() {
  for (char &c : _s) {
    c = (char)tolower((unsigned char)c);
  }
}
