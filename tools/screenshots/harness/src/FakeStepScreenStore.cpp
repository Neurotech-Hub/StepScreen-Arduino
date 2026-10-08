// Replaces src/StepScreenStore.cpp (FlashStorage_SAMD) with an in-memory
// store that starts empty on every run.

#include <StepScreenStore.h>

#include <map>
#include <vector>

namespace {

std::map<uint32_t, std::vector<uint8_t>> &blobs() {
  static std::map<uint32_t, std::vector<uint8_t>> b;
  return b;
}

} // namespace

bool StepScreenStore::begin() {
  _available = true;
  return true;
}

bool StepScreenStore::load(uint32_t magic, void *dst, size_t len) {
  const auto it = blobs().find(magic);
  if (it == blobs().end() || it->second.size() != len) {
    return false;
  }
  memcpy(dst, it->second.data(), len);
  return true;
}

bool StepScreenStore::save(uint32_t magic, const void *src, size_t len) {
  const uint8_t *p = (const uint8_t *)src;
  blobs()[magic].assign(p, p + len);
  return true;
}

bool StepScreenStore::clear(uint32_t magic) {
  blobs().erase(magic);
  return true;
}

uint16_t StepScreenStore::checksum(const uint8_t *data, size_t len) {
  uint16_t sum = 0;
  for (size_t i = 0; i < len; i++) {
    sum = (uint16_t)(sum + data[i]);
  }
  return sum;
}
