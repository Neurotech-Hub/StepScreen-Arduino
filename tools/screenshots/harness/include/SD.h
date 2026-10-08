// Host fake of the Arduino SD library backed by an in-memory file system
// (StepScreenSim::sdFiles()). Paths are case-insensitive like FAT and a
// leading '/' is optional. The card mounts only when
// StepScreenSim::hw().sdCard is set.

#ifndef __SD_H__
#define __SD_H__

#include <Arduino.h>

#include <string>

#define FILE_READ 0x01
#define FILE_WRITE 0x13

class File : public Stream {
public:
  File() {}
  File(const std::string &key, uint8_t mode);

  operator bool() const { return !_key.empty(); }
  const char *name() const { return _key.c_str(); }

  size_t write(uint8_t c) override;
  size_t write(const uint8_t *buf, size_t size) override;
  using Print::write;
  int available() override;
  int read() override;
  int peek() override;

  bool seek(uint32_t pos);
  uint32_t position() const { return _pos; }
  uint32_t size() const;
  void close() { _key.clear(); }

private:
  std::string _key;
  uint32_t _pos = 0;
};

class SDClass {
public:
  bool begin(uint8_t csPin = 4);
  void end() {}
  File open(const char *path, uint8_t mode = FILE_READ);
  bool exists(const char *path);
  bool mkdir(const char *path);
  bool remove(const char *path);
  bool rmdir(const char *path);
};

extern SDClass SD;

#endif // __SD_H__
