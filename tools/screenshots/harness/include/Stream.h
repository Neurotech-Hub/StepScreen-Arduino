// Arduino Stream base class for host builds.

#ifndef STEPSCREEN_HOST_STREAM_H
#define STEPSCREEN_HOST_STREAM_H

#include "Print.h"

class Stream : public Print {
public:
  virtual int available() = 0;
  virtual int read() = 0;
  virtual int peek() = 0;

  void setTimeout(unsigned long ms) { _timeoutMs = ms; }
  size_t readBytes(char *buffer, size_t length);
  size_t readBytesUntil(char terminator, char *buffer, size_t length);
  String readStringUntil(char terminator);

protected:
  unsigned long _timeoutMs = 1000;
};

#endif // STEPSCREEN_HOST_STREAM_H
