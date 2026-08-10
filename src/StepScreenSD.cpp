#include "StepScreenSD.h"

#include <SD.h>
#include <SPI.h>
#include <stdio.h>
#include <string.h>

namespace {

const char *selfTestPath = "/IOTEST.TXT";

} // namespace

void StepScreenSD::begin(uint8_t csPin, uint8_t cdPin) {
  _csPin = csPin;
  _cdPin = cdPin;

  pinMode(_cdPin, INPUT_PULLUP);

  _cdDebounced = readCardDetect();
  _cardPresent = _cdDebounced;
  _cdLastChangeMs = millis();
  _mountAttempts = 0;
  _lastError[0] = '\0';
  _lastSelfTestPassed = false;

  if (_cardPresent) {
    _mountState = STEPSCREEN_SD_MOUNTING;
    _nextMountTryMs = millis() + INSERT_SETTLE_MS;
  } else {
    _mountState = STEPSCREEN_SD_ABSENT;
  }
}

bool StepScreenSD::readCardDetect() const {
  return StepScreenSdCardInserted(_cdPin);
}

const char *StepScreenSD::mountStateLabel() const {
  switch (_mountState) {
  case STEPSCREEN_SD_ABSENT:
    return "no card";
  case STEPSCREEN_SD_MOUNTING:
    return "mounting";
  case STEPSCREEN_SD_READY:
    return "ready";
  case STEPSCREEN_SD_ERROR:
    return "error";
  default:
    return "?";
  }
}

void StepScreenSD::setError(const char *msg) {
  if (msg == nullptr) {
    _lastError[0] = '\0';
    return;
  }
  strncpy(_lastError, msg, sizeof(_lastError) - 1);
  _lastError[sizeof(_lastError) - 1] = '\0';
}

void StepScreenSD::pulseActivityLed() {
  if (!_activityLed) {
    return;
  }
  digitalWrite(PIN_LED_GREEN, HIGH);
  delay(40);
  digitalWrite(PIN_LED_GREEN, LOW);
}

void StepScreenSD::unmount() {
#if defined(SD_HAS_END) || defined(ARDUINO_ARCH_SAMD)
  SD.end();
#endif
  _mountState = STEPSCREEN_SD_ABSENT;
  _mountAttempts = 0;
  _nextMountTryMs = 0;
  setError("card removed");
}

void StepScreenSD::handleCardChange(bool present) {
  if (present) {
    _mountState = STEPSCREEN_SD_MOUNTING;
    _mountAttempts = 0;
    _nextMountTryMs = millis() + INSERT_SETTLE_MS;
    setError(nullptr);
  } else {
    unmount();
  }
}

bool StepScreenSD::tryMount() {
  if (!SD.begin(_csPin)) {
    setError("SD.begin failed");
    return false;
  }

  _mountState = STEPSCREEN_SD_READY;
  _mountAttempts = 0;
  setError(nullptr);
  return true;
}

void StepScreenSD::update() {
  const bool rawCd = readCardDetect();
  if (rawCd != _cdDebounced) {
    _cdDebounced = rawCd;
    _cdLastChangeMs = millis();
  }

  if (millis() - _cdLastChangeMs >= CD_DEBOUNCE_MS &&
      _cdDebounced != _cardPresent) {
    _cardPresent = _cdDebounced;
    handleCardChange(_cardPresent);
  }

  if (!_cardPresent || _mountState == STEPSCREEN_SD_READY) {
    return;
  }

  if (millis() < _nextMountTryMs) {
    return;
  }

  _mountAttempts++;
  if (tryMount()) {
    return;
  }

  if (_mountAttempts >= MAX_MOUNT_ATTEMPTS) {
    _mountState = STEPSCREEN_SD_ERROR;
    _mountAttempts = 0;
    _nextMountTryMs = millis() + 2000;
    return;
  }

  _nextMountTryMs = millis() + MOUNT_RETRY_MS;
}

bool StepScreenSD::writeFile(const char *path, const uint8_t *data, size_t len,
                             bool append) {
  if (!isReady() || path == nullptr || data == nullptr || len == 0) {
    setError("write unavailable");
    return false;
  }

  if (!append) {
    SD.remove(path);
  }

  File file = SD.open(path, FILE_WRITE);
  if (!file) {
    setError("open write failed");
    return false;
  }

  if (append) {
    file.seek(file.size());
  }

  const size_t written = file.write(data, len);
  file.close();

  if (written != len) {
    setError("write incomplete");
    return false;
  }

  pulseActivityLed();
  setError(nullptr);
  return true;
}

bool StepScreenSD::appendLine(const char *path, const char *line) {
  if (!isReady() || path == nullptr || line == nullptr) {
    setError("append unavailable");
    return false;
  }

  File file = SD.open(path, FILE_WRITE);
  if (!file) {
    setError("open append failed");
    return false;
  }
  file.seek(file.size());
  file.println(line);
  file.close();

  pulseActivityLed();
  setError(nullptr);
  return true;
}

bool StepScreenSD::readLine(const char *path, char *buf, size_t bufLen) {
  if (!isReady() || path == nullptr || buf == nullptr || bufLen == 0) {
    setError("read unavailable");
    return false;
  }

  File file = SD.open(path, FILE_READ);
  if (!file) {
    setError("open read failed");
    return false;
  }

  int n = file.readBytesUntil('\n', buf, bufLen - 1);
  file.close();
  if (n < 0) {
    setError("read failed");
    return false;
  }

  buf[n] = '\0';
  while (n > 0 && (buf[n - 1] == '\r' || buf[n - 1] == '\n')) {
    buf[--n] = '\0';
  }

  setError(nullptr);
  return true;
}

bool StepScreenSD::runSelfTest() {
  _lastSelfTestPassed = false;

  if (!isReady()) {
    setError("not mounted");
    return false;
  }

  char payload[32];
  snprintf(payload, sizeof(payload), "IOTEST %lu", millis());

  SD.remove(selfTestPath);
  if (!appendLine(selfTestPath, payload)) {
    return false;
  }

  char readBack[32];
  if (!readLine(selfTestPath, readBack, sizeof(readBack))) {
    return false;
  }

  if (strstr(readBack, payload) == nullptr) {
    setError("verify failed");
    return false;
  }

  _lastSelfTestPassed = true;
  setError(nullptr);
  return true;
}
