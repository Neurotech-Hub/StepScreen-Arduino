/*!
 * Host (macOS/Linux) stand-in for the Arduino core, used only by the
 * screenshot tool. Time is virtual: millis()/micros() advance only through
 * delay(), delayMicroseconds(), StepScreenSim::advance*(), and a 1 us tick
 * per clock read (so busy-wait loops always terminate).
 */

#ifndef STEPSCREEN_HOST_ARDUINO_H
#define STEPSCREEN_HOST_ARDUINO_H

#include <algorithm>
#include <cmath>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using std::abs;
using std::max;
using std::min;

typedef uint8_t byte;
typedef bool boolean;
typedef uint16_t word;

#define HIGH 0x1
#define LOW 0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2
#define INPUT_PULLDOWN 0x3

#define CHANGE 2
#define FALLING 3
#define RISING 4

#define LSBFIRST 0
#define MSBFIRST 1

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

// Feather M0 analog pin numbers (A7 shares D9 with the VBAT divider).
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19
#define A6 20
#define A7 9

#define PROGMEM
#define PSTR(s) (s)
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_word(addr) (*(const unsigned short *)(addr))

#define bit(b) (1UL << (b))
#define bitRead(value, b) (((value) >> (b)) & 0x01)
#define bitSet(value, b) ((value) |= (1UL << (b)))
#define bitClear(value, b) ((value) &= ~(1UL << (b)))
#define bitWrite(value, b, v) ((v) ? bitSet(value, b) : bitClear(value, b))
#define lowByte(w) ((uint8_t)((w) & 0xff))
#define highByte(w) ((uint8_t)((w) >> 8))
#define sq(x) ((x) * (x))
#define radians(deg) ((deg) * DEG_TO_RAD)
#define degrees(rad) ((rad) * RAD_TO_DEG)

template <typename T, typename L, typename H>
inline T constrain(T x, L lo, H hi) {
  return x < (T)lo ? (T)lo : (x > (T)hi ? (T)hi : x);
}

inline long map(long x, long inMin, long inMax, long outMin, long outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t level);
int digitalRead(uint8_t pin);
int analogRead(uint8_t pin);
void analogWrite(uint8_t pin, int value);
void analogReadResolution(int bits);
void analogWriteResolution(int bits);

unsigned long millis();
unsigned long micros();
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
void yield();

void noInterrupts();
void interrupts();

typedef void (*voidFuncPtr)(void);
#define digitalPinToInterrupt(p) (p)
void attachInterrupt(uint8_t interruptNum, voidFuncPtr fn, int mode);
void detachInterrupt(uint8_t interruptNum);

char *itoa(int value, char *str, int base);
char *ltoa(long value, char *str, int base);
char *utoa(unsigned value, char *str, int base);
char *ultoa(unsigned long value, char *str, int base);
char *dtostrf(double value, signed char width, unsigned char prec, char *buf);

long random(long howBig);
long random(long howSmall, long howBig);
void randomSeed(unsigned long seed);

class __FlashStringHelper;
#define F(s) (reinterpret_cast<const __FlashStringHelper *>(s))

#include "WString.h"
#include "Print.h"
#include "Stream.h"
#include "HardwareSerial.h"

#endif // STEPSCREEN_HOST_ARDUINO_H
