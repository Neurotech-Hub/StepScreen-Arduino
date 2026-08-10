/*!
 * @file StepScreenSD.h
 *
 * microSD card helper for the Adafruit Feather M0 Adalogger (or compatible).
 * Uses SPI + PIN_SD_CS (default 4) and card-detect PIN_SD_CD (default 7).
 * Card present when the CD pin reads STEPSCREEN_SD_CD_INSERTED (default LOW).
 *
 * Call update() from loop() so card removal and re-insertion are handled.
 *
 *   StepScreenSD sd;
 *   sd.begin();
 *
 *   void loop() {
 *     sd.update();
 *     if (sd.isReady()) {
 *       sd.appendLine("/log.txt", "hello");
 *     }
 *   }
 */

#ifndef STEPSCREEN_SD_H
#define STEPSCREEN_SD_H

#include <Arduino.h>

#include "StepScreenBoard.h"

enum StepScreenSDMountState : uint8_t {
  STEPSCREEN_SD_ABSENT = 0, // no card in the slot
  STEPSCREEN_SD_MOUNTING,   // card present, mount in progress / retrying
  STEPSCREEN_SD_READY,      // mounted and usable
  STEPSCREEN_SD_ERROR,      // card present but mount failed
};

class StepScreenSD {
public:
  void begin(uint8_t csPin = PIN_SD_CS, uint8_t cdPin = PIN_SD_CD);

  // Poll card detect and mount/unmount as needed. Call every loop().
  void update();

  bool cardPresent() const { return _cardPresent; }
  bool isReady() const { return _mountState == STEPSCREEN_SD_READY; }
  StepScreenSDMountState mountState() const { return _mountState; }
  const char *mountStateLabel() const;
  const char *lastError() const { return _lastError; }

  bool lastSelfTestPassed() const { return _lastSelfTestPassed; }

  // Write/read helpers (no-op when not mounted)
  bool writeFile(const char *path, const uint8_t *data, size_t len,
                 bool append = false);
  bool appendLine(const char *path, const char *line);
  bool readLine(const char *path, char *buf, size_t bufLen);

  // Write IOTEST.TXT, read it back, verify payload. Leaves the file on card.
  bool runSelfTest();

  // Blink the Adalogger green LED (PIN_LED_GREEN) on successful writes
  void setActivityLed(bool enable) { _activityLed = enable; }

private:
  bool readCardDetect() const;
  void handleCardChange(bool present);
  void unmount();
  bool tryMount();
  void setError(const char *msg);
  void pulseActivityLed();

  uint8_t _csPin = PIN_SD_CS;
  uint8_t _cdPin = PIN_SD_CD;
  bool _cardPresent = false;
  bool _cdDebounced = false;
  uint32_t _cdLastChangeMs = 0;
  uint8_t _mountAttempts = 0;
  uint32_t _nextMountTryMs = 0;
  StepScreenSDMountState _mountState = STEPSCREEN_SD_ABSENT;
  bool _activityLed = true;
  bool _lastSelfTestPassed = false;
  char _lastError[48] = {};

  static const uint32_t CD_DEBOUNCE_MS = 100;
  static const uint32_t INSERT_SETTLE_MS = 150;
  static const uint32_t MOUNT_RETRY_MS = 500;
  static const uint8_t MAX_MOUNT_ATTEMPTS = 8;
};

#endif // STEPSCREEN_SD_H
