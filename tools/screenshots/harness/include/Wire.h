// I2C stub for host builds: every transaction "succeeds" with no data.
// Devices on the bus (OLED, RTC) are faked at the driver level instead.

#ifndef STEPSCREEN_HOST_WIRE_H
#define STEPSCREEN_HOST_WIRE_H

#include "Arduino.h"

class TwoWire : public Stream {
public:
  void begin() {}
  void begin(uint8_t) {}
  void end() {}
  void setClock(uint32_t) {}
  void beginTransmission(uint8_t) {}
  uint8_t endTransmission(bool = true) { return 0; }
  uint8_t requestFrom(uint8_t, size_t, bool = true) { return 0; }

  size_t write(uint8_t) override { return 1; }
  using Print::write;
  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
};

extern TwoWire Wire;

#endif // STEPSCREEN_HOST_WIRE_H
