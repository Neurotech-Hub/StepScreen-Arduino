#include <SD.h>

#include "StepScreenSim.h"

#include <set>

namespace {

std::set<std::string> &directories() {
  static std::set<std::string> dirs;
  return dirs;
}

std::string &contents(const std::string &key) {
  return StepScreenSim::sdFiles()[key];
}

} // namespace

SDClass SD;

File::File(const std::string &key, uint8_t mode) : _key(key) {
  _pos = (mode == FILE_WRITE) ? (uint32_t)contents(key).size() : 0;
}

size_t File::write(uint8_t c) { return write(&c, 1); }

size_t File::write(const uint8_t *buf, size_t size) {
  if (_key.empty()) {
    return 0;
  }
  std::string &data = contents(_key);
  if (_pos > data.size()) {
    _pos = (uint32_t)data.size();
  }
  data.replace(_pos, std::min(size, data.size() - _pos), (const char *)buf,
               size);
  _pos += (uint32_t)size;
  return size;
}

int File::available() {
  return _key.empty() ? 0 : (int)(contents(_key).size() - _pos);
}

int File::read() {
  if (available() <= 0) {
    return -1;
  }
  return (uint8_t)contents(_key)[_pos++];
}

int File::peek() {
  if (available() <= 0) {
    return -1;
  }
  return (uint8_t)contents(_key)[_pos];
}

bool File::seek(uint32_t pos) {
  if (_key.empty() || pos > size()) {
    return false;
  }
  _pos = pos;
  return true;
}

uint32_t File::size() const {
  if (_key.empty()) {
    return 0;
  }
  const auto it = StepScreenSim::sdFiles().find(_key);
  return it == StepScreenSim::sdFiles().end() ? 0 : (uint32_t)it->second.size();
}

bool SDClass::begin(uint8_t) { return StepScreenSim::hw().sdCard; }

File SDClass::open(const char *path, uint8_t mode) {
  if (!StepScreenSim::hw().sdCard) {
    return File();
  }
  const std::string key = StepScreenSim::sdKey(path);
  auto &files = StepScreenSim::sdFiles();
  if (mode != FILE_WRITE && files.find(key) == files.end()) {
    return File();
  }
  return File(key, mode);
}

bool SDClass::exists(const char *path) {
  const std::string key = StepScreenSim::sdKey(path);
  return StepScreenSim::sdFiles().count(key) > 0 ||
         directories().count(key) > 0;
}

bool SDClass::mkdir(const char *path) {
  directories().insert(StepScreenSim::sdKey(path));
  return StepScreenSim::hw().sdCard;
}

bool SDClass::remove(const char *path) {
  return StepScreenSim::sdFiles().erase(StepScreenSim::sdKey(path)) > 0;
}

bool SDClass::rmdir(const char *path) {
  return directories().erase(StepScreenSim::sdKey(path)) > 0;
}
