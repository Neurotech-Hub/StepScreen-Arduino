// Serial / Serial1 for host builds. Serial output goes to the sim's serial
// log (and stdout with --serial); nothing is ever received.

#ifndef STEPSCREEN_HOST_HARDWARESERIAL_H
#define STEPSCREEN_HOST_HARDWARESERIAL_H

#include "Stream.h"

class HardwareSerial : public Stream {
public:
  explicit HardwareSerial(bool console) : _console(console) {}

  void begin(unsigned long) {}
  void begin(unsigned long, uint16_t) {}
  void end() {}
  operator bool() const { return true; }

  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }

  size_t write(uint8_t c) override;
  using Print::write;

private:
  bool _console;
};

extern HardwareSerial Serial;
extern HardwareSerial Serial1;

#endif // STEPSCREEN_HOST_HARDWARESERIAL_H
