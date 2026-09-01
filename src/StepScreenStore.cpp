#include "StepScreenStore.h"

// FlashStorage_SAMD.h pulls in implementations and must appear in exactly ONE
// translation unit for the whole build. Keep it here — do not include it from
// sketches that also link StepScreenStore.
#if defined(ARDUINO_ARCH_SAMD)
#include <FlashStorage_SAMD.h>
#define STEPSCREEN_STORE_EEPROM 1
#endif

namespace {

constexpr size_t kHeaderSize = 8; // magic(4) + len(2) + checksum(2)
constexpr size_t kMaxPayload = 64;

struct StoreHeader {
  uint32_t magic;
  uint16_t length;
  uint16_t checksum;
};

static_assert(sizeof(StoreHeader) == kHeaderSize, "store header size");

bool readHeader(StoreHeader &hdr) {
#if defined(STEPSCREEN_STORE_EEPROM)
  EEPROM.get(0, hdr);
  return true;
#else
  (void)hdr;
  return false;
#endif
}

bool writeHeader(const StoreHeader &hdr) {
#if defined(STEPSCREEN_STORE_EEPROM)
  EEPROM.put(0, hdr);
  return true;
#else
  (void)hdr;
  return false;
#endif
}

#if defined(STEPSCREEN_STORE_EEPROM)
void commitStore() { EEPROM.commit(); }
#endif

} // namespace

bool StepScreenStore::begin() {
#if defined(STEPSCREEN_STORE_EEPROM)
  _available = true;
  return true;
#else
  _available = false;
  return false;
#endif
}

uint16_t StepScreenStore::checksum(const uint8_t *data, size_t len) {
  uint16_t sum = 0;
  for (size_t i = 0; i < len; i++) {
    sum = (uint16_t)(sum + data[i]);
  }
  return sum;
}

bool StepScreenStore::load(uint32_t magic, void *dst, size_t len) {
  if (!_available || dst == nullptr || len == 0 || len > kMaxPayload) {
    return false;
  }

  StoreHeader hdr;
  if (!readHeader(hdr)) {
    return false;
  }
  if (hdr.magic != magic || hdr.length != (uint16_t)len) {
    return false;
  }

  uint8_t buf[kMaxPayload];
#if defined(STEPSCREEN_STORE_EEPROM)
  for (size_t i = 0; i < len; i++) {
    buf[i] = EEPROM.read(kHeaderSize + i);
  }
#else
  return false;
#endif

  if (checksum(buf, len) != hdr.checksum) {
    return false;
  }

  memcpy(dst, buf, len);
  return true;
}

bool StepScreenStore::save(uint32_t magic, const void *src, size_t len) {
  if (!_available || src == nullptr || len == 0 || len > kMaxPayload) {
    return false;
  }

  const uint8_t *bytes = (const uint8_t *)src;
  StoreHeader hdr;
  hdr.magic = magic;
  hdr.length = (uint16_t)len;
  hdr.checksum = checksum(bytes, len);

  if (!writeHeader(hdr)) {
    return false;
  }

#if defined(STEPSCREEN_STORE_EEPROM)
  for (size_t i = 0; i < len; i++) {
    EEPROM.write(kHeaderSize + i, bytes[i]);
  }
  commitStore();
  return true;
#else
  return false;
#endif
}

bool StepScreenStore::clear(uint32_t magic) {
  if (!_available) {
    return false;
  }
  StoreHeader hdr;
  if (!readHeader(hdr) || hdr.magic != magic) {
    return false;
  }
  hdr.magic = 0;
  if (!writeHeader(hdr)) {
    return false;
  }
#if defined(STEPSCREEN_STORE_EEPROM)
  commitStore();
  return true;
#else
  return false;
#endif
}
