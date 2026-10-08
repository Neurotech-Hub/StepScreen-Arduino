// Host fake of Adafruit_SH110X: a 1-bit framebuffer in the same page layout
// as the real driver. display() publishes the buffer to the screenshot tool
// (StepScreenShot::lastFrame()) instead of sending it over I2C.

#ifndef _Adafruit_SH110X_H_
#define _Adafruit_SH110X_H_

#include <Adafruit_GFX.h>
#include <Wire.h>

#define SH110X_BLACK 0
#define SH110X_WHITE 1
#define SH110X_INVERSE 2

class Adafruit_SH110X : public Adafruit_GFX {
public:
  Adafruit_SH110X(uint16_t w, uint16_t h, TwoWire *twi = &Wire,
                  int16_t rst_pin = -1, uint32_t preclk = 400000,
                  uint32_t postclk = 100000);
  virtual ~Adafruit_SH110X();

  bool begin(uint8_t i2caddr = 0x3C, bool reset = true);
  void display();
  void clearDisplay();
  void drawPixel(int16_t x, int16_t y, uint16_t color) override;
  void invertDisplay(bool inv) override;
  bool getPixel(int16_t x, int16_t y);
  uint8_t *getBuffer() { return _buffer; }
  void setContrast(uint8_t contrast) { _contrast = contrast; }
  void oled_command(uint8_t) {}
  bool oled_commandList(const uint8_t *, uint8_t) { return true; }

private:
  uint8_t *_buffer = nullptr;
  bool _inverted = false;
  uint8_t _contrast = 0x2F;
};

class Adafruit_SH1106G : public Adafruit_SH110X {
public:
  Adafruit_SH1106G(uint16_t w, uint16_t h, TwoWire *twi = &Wire,
                   int16_t rst_pin = -1, uint32_t preclk = 400000,
                   uint32_t postclk = 100000)
      : Adafruit_SH110X(w, h, twi, rst_pin, preclk, postclk) {}
};

class Adafruit_SH1107 : public Adafruit_SH110X {
public:
  Adafruit_SH1107(uint16_t w, uint16_t h, TwoWire *twi = &Wire,
                  int16_t rst_pin = -1, uint32_t preclk = 400000,
                  uint32_t postclk = 100000)
      : Adafruit_SH110X(w, h, twi, rst_pin, preclk, postclk) {}
};

#endif // _Adafruit_SH110X_H_
