// SPI stub for host builds.

#ifndef STEPSCREEN_HOST_SPI_H
#define STEPSCREEN_HOST_SPI_H

#include "Arduino.h"

#define SPI_MODE0 0x00
#define SPI_MODE1 0x01
#define SPI_MODE2 0x02
#define SPI_MODE3 0x03

class SPISettings {
public:
  SPISettings() {}
  SPISettings(uint32_t, uint8_t, uint8_t) {}
};

class SPIClass {
public:
  void begin() {}
  void end() {}
  void beginTransaction(SPISettings) {}
  void endTransaction() {}
  uint8_t transfer(uint8_t) { return 0xFF; }
};

extern SPIClass SPI;

#endif // STEPSCREEN_HOST_SPI_H
