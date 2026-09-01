/*!
 * @file StepScreenStore.h
 *
 * Small blob persistence on SAMD21 via FlashStorage_SAMD emulated EEPROM.
 * Install FlashStorage_SAMD from Library Manager; StepScreenStore.cpp includes
 * FlashStorage_SAMD.h internally — do NOT also include it from your sketch.
 * If flash storage is unavailable, use SD fallback (see Truss example).
 * Each blob is stored with a header: magic (uint32_t), length (uint16_t),
 * checksum (uint16_t), then payload bytes.
 *
 *   StepScreenStore store;
 *   store.begin();
 *   if (store.load(0x54525353UL, &data, sizeof(data))) { ... }
 *   store.save(0x54525353UL, &data, sizeof(data));
 */

#ifndef STEPSCREEN_STORE_H
#define STEPSCREEN_STORE_H

#include <Arduino.h>

class StepScreenStore {
public:
  // Returns false if EEPROM is unavailable on this platform.
  bool begin();

  bool available() const { return _available; }

  // Load/save a blob identified by magic. Returns false if missing or corrupt.
  bool load(uint32_t magic, void *dst, size_t len);
  bool save(uint32_t magic, const void *src, size_t len);

  // Erase stored blob (writes zero magic).
  bool clear(uint32_t magic);

private:
  static uint16_t checksum(const uint8_t *data, size_t len);

  bool _available = false;
};

#endif // STEPSCREEN_STORE_H
