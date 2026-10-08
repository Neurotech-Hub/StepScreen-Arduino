// Arduino Print base class for host builds.

#ifndef STEPSCREEN_HOST_PRINT_H
#define STEPSCREEN_HOST_PRINT_H

#include <stddef.h>
#include <stdint.h>

#include "WString.h"

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

class Print {
public:
  virtual ~Print() {}

  virtual size_t write(uint8_t c) = 0;
  virtual size_t write(const uint8_t *buffer, size_t size);
  size_t write(const char *str);
  size_t write(const char *buffer, size_t size) {
    return write((const uint8_t *)buffer, size);
  }
  virtual void flush() {}

  size_t print(const __FlashStringHelper *s);
  size_t print(const String &s);
  size_t print(const char s[]);
  size_t print(char c);
  size_t print(unsigned char n, int base = DEC);
  size_t print(int n, int base = DEC);
  size_t print(unsigned int n, int base = DEC);
  size_t print(long n, int base = DEC);
  size_t print(unsigned long n, int base = DEC);
  size_t print(long long n, int base = DEC);
  size_t print(unsigned long long n, int base = DEC);
  size_t print(double n, int digits = 2);

  size_t println(const __FlashStringHelper *s);
  size_t println(const String &s);
  size_t println(const char s[]);
  size_t println(char c);
  size_t println(unsigned char n, int base = DEC);
  size_t println(int n, int base = DEC);
  size_t println(unsigned int n, int base = DEC);
  size_t println(long n, int base = DEC);
  size_t println(unsigned long n, int base = DEC);
  size_t println(long long n, int base = DEC);
  size_t println(unsigned long long n, int base = DEC);
  size_t println(double n, int digits = 2);
  size_t println();

  size_t printf(const char *format, ...);

private:
  size_t printUnsigned(unsigned long long n, int base);
  size_t printSigned(long long n, int base);
};

#endif // STEPSCREEN_HOST_PRINT_H
