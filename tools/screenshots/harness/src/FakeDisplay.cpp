#include <Adafruit_SH110X.h>

#include "SimInternal.h"
#include "StepScreenShot.h"

Adafruit_SH110X::Adafruit_SH110X(uint16_t w, uint16_t h, TwoWire *, int16_t,
                                 uint32_t, uint32_t)
    : Adafruit_GFX(w, h) {
  _buffer = new uint8_t[w * ((h + 7) / 8)]();
}

Adafruit_SH110X::~Adafruit_SH110X() { delete[] _buffer; }

bool Adafruit_SH110X::begin(uint8_t, bool) {
  clearDisplay();
  return true;
}

void Adafruit_SH110X::clearDisplay() {
  memset(_buffer, 0, WIDTH * ((HEIGHT + 7) / 8));
}

void Adafruit_SH110X::drawPixel(int16_t x, int16_t y, uint16_t color) {
  if (x < 0 || x >= width() || y < 0 || y >= height()) {
    return;
  }
  switch (getRotation()) {
  case 1: {
    const int16_t t = x;
    x = WIDTH - y - 1;
    y = t;
    break;
  }
  case 2:
    x = WIDTH - x - 1;
    y = HEIGHT - y - 1;
    break;
  case 3: {
    const int16_t t = x;
    x = y;
    y = HEIGHT - t - 1;
    break;
  }
  }
  uint8_t &b = _buffer[x + (y / 8) * WIDTH];
  const uint8_t mask = (uint8_t)(1 << (y & 7));
  switch (color) {
  case SH110X_WHITE:
    b |= mask;
    break;
  case SH110X_BLACK:
    b &= (uint8_t)~mask;
    break;
  case SH110X_INVERSE:
    b ^= mask;
    break;
  }
}

bool Adafruit_SH110X::getPixel(int16_t x, int16_t y) {
  if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) {
    return false;
  }
  return (_buffer[x + (y / 8) * WIDTH] >> (y & 7)) & 1;
}

void Adafruit_SH110X::invertDisplay(bool inv) { _inverted = inv; }

void Adafruit_SH110X::display() {
  if (WIDTH == StepScreenShot::Frame::W && HEIGHT == StepScreenShot::Frame::H) {
    StepScreenSimInternal::publishFrame(_buffer, _inverted);
  }
  StepScreenSimInternal::notifyDisplay();
}
