// Virtual clock, simulated pins/interrupts, Serial, and the StepScreenSim
// control API.

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include <StepScreenBoard.h>
#include <StepScreenPins.h>

#include "SimInternal.h"
#include "StepScreenSim.h"

#include <algorithm>
#include <vector>

namespace {

uint64_t g_us = 0;
bool g_inEvents = false;

const int kPinCount = 256;
int g_level[kPinCount];
bool g_levelSet[kPinCount] = {};

struct Isr {
  voidFuncPtr fn = nullptr;
  int mode = CHANGE;
};
Isr g_isr[kPinCount];

struct PinEvent {
  uint64_t atUs;
  uint64_t seq;
  uint8_t pin;
  int level;
};
std::vector<PinEvent> g_events;
uint64_t g_eventSeq = 0;

FILE *g_serialLog = nullptr;
bool g_serialEcho = false;

std::function<void()> g_onDisplay;

int defaultLevel(uint8_t pin) {
  if (pin == PIN_SD_CD) {
    return StepScreenSim::hw().sdCard ? STEPSCREEN_SD_CD_INSERTED
                                      : !STEPSCREEN_SD_CD_INSERTED;
  }
  return HIGH; // pull-ups
}

int readLevel(uint8_t pin) {
  return g_levelSet[pin] ? g_level[pin] : defaultLevel(pin);
}

void writeLevel(uint8_t pin, int level) {
  const int prev = readLevel(pin);
  g_level[pin] = level ? HIGH : LOW;
  g_levelSet[pin] = true;
  const Isr &isr = g_isr[pin];
  if (isr.fn == nullptr || prev == g_level[pin]) {
    return;
  }
  const bool rising = g_level[pin] == HIGH;
  if (isr.mode == CHANGE || (isr.mode == RISING && rising) ||
      (isr.mode == FALLING && !rising)) {
    isr.fn();
  }
}

void applyDueEvents() {
  if (g_inEvents) {
    return;
  }
  g_inEvents = true;
  while (!g_events.empty()) {
    auto next = std::min_element(
        g_events.begin(), g_events.end(),
        [](const PinEvent &a, const PinEvent &b) {
          return a.atUs != b.atUs ? a.atUs < b.atUs : a.seq < b.seq;
        });
    if (next->atUs > g_us) {
      break;
    }
    const PinEvent ev = *next;
    g_events.erase(next);
    writeLevel(ev.pin, ev.level);
  }
  g_inEvents = false;
}

void schedule(uint8_t pin, uint64_t atUs, int level) {
  g_events.push_back({atUs, g_eventSeq++, pin, level});
}

uint64_t tick() {
  g_us += 1;
  applyDueEvents();
  return g_us;
}

} // namespace

HardwareSerial Serial(true);
HardwareSerial Serial1(false);
TwoWire Wire;
SPIClass SPI;

size_t HardwareSerial::write(uint8_t c) {
  if (_console) {
    StepScreenSimInternal::writeSerial(c);
  }
  return 1;
}

void pinMode(uint8_t, uint8_t) {}

void digitalWrite(uint8_t pin, uint8_t level) { writeLevel(pin, level); }

int digitalRead(uint8_t pin) {
  tick();
  return readLevel(pin);
}

int analogRead(uint8_t pin) {
  tick();
  return readLevel(pin) ? 1023 : 0;
}

void analogWrite(uint8_t pin, int value) { writeLevel(pin, value > 0); }
void analogReadResolution(int) {}
void analogWriteResolution(int) {}

unsigned long millis() { return (uint32_t)(tick() / 1000ULL); }
unsigned long micros() { return (uint32_t)tick(); }

void delay(unsigned long ms) { StepScreenSim::advanceUs((uint64_t)ms * 1000ULL); }
void delayMicroseconds(unsigned int us) { StepScreenSim::advanceUs(us); }
void yield() {}

void noInterrupts() {}
void interrupts() {}

void attachInterrupt(uint8_t pin, voidFuncPtr fn, int mode) {
  g_isr[pin].fn = fn;
  g_isr[pin].mode = mode;
}

void detachInterrupt(uint8_t pin) { g_isr[pin].fn = nullptr; }

char *ultoa(unsigned long value, char *str, int base) {
  if (base < 2 || base > 36) {
    base = 10;
  }
  char tmp[8 * sizeof(value) + 1];
  int n = 0;
  do {
    const int digit = (int)(value % (unsigned long)base);
    tmp[n++] = (char)(digit < 10 ? '0' + digit : 'a' + digit - 10);
    value /= (unsigned long)base;
  } while (value);
  for (int i = 0; i < n; i++) {
    str[i] = tmp[n - 1 - i];
  }
  str[n] = '\0';
  return str;
}

char *ltoa(long value, char *str, int base) {
  if (base == 10 && value < 0) {
    str[0] = '-';
    ultoa((unsigned long)(-value), str + 1, base);
    return str;
  }
  return ultoa((unsigned long)value, str, base);
}

char *itoa(int value, char *str, int base) { return ltoa(value, str, base); }

char *utoa(unsigned value, char *str, int base) {
  return ultoa(value, str, base);
}

char *dtostrf(double value, signed char width, unsigned char prec, char *buf) {
  sprintf(buf, "%*.*f", width, prec, value);
  return buf;
}

long random(long howBig) { return howBig > 0 ? ::random() % howBig : 0; }
long random(long howSmall, long howBig) {
  return howBig > howSmall ? howSmall + random(howBig - howSmall) : howSmall;
}
void randomSeed(unsigned long seed) { srandom((unsigned)seed); }

namespace StepScreenSimInternal {

uint64_t nowUs() { return g_us; }

void setSerialSink(FILE *log, bool echo) {
  g_serialLog = log;
  g_serialEcho = echo;
}

void writeSerial(uint8_t c) {
  if (g_serialLog != nullptr) {
    fputc(c, g_serialLog);
  }
  if (g_serialEcho) {
    fputc(c, stdout);
  }
}

void notifyDisplay() {
  if (g_onDisplay) {
    g_onDisplay();
  }
}

} // namespace StepScreenSimInternal

namespace StepScreenSim {

Hardware &hw() {
  static Hardware h;
  return h;
}

void advanceUs(uint64_t us) {
  const uint64_t target = g_us + us;
  while (true) {
    uint64_t nextAt = target;
    for (const PinEvent &ev : g_events) {
      nextAt = std::min(nextAt, ev.atUs);
    }
    if (nextAt >= target) {
      break;
    }
    g_us = std::max(g_us, nextAt);
    applyDueEvents();
  }
  g_us = target;
  applyDueEvents();
}

void advanceMs(uint32_t ms) { advanceUs((uint64_t)ms * 1000ULL); }

void run(void (*loopFn)(), uint32_t ms, uint32_t tickUs) {
  const uint64_t end = g_us + (uint64_t)ms * 1000ULL;
  while (g_us < end) {
    loopFn();
    advanceUs(tickUs);
  }
}

bool runUntil(void (*loopFn)(), const std::function<bool()> &pred,
              uint32_t timeoutMs, uint32_t tickUs) {
  const uint64_t end = g_us + (uint64_t)timeoutMs * 1000ULL;
  while (!pred()) {
    if (g_us >= end) {
      return false;
    }
    loopFn();
    advanceUs(tickUs);
  }
  return true;
}

void setPin(uint8_t pin, int level) { writeLevel(pin, level); }

int pinLevel(uint8_t pin) { return readLevel(pin); }

void press(uint8_t pin, uint32_t holdMs) {
  writeLevel(pin, LOW);
  schedule(pin, g_us + (uint64_t)holdMs * 1000ULL, HIGH);
}

void pressAt(uint8_t pin, uint32_t atMs, uint32_t holdMs) {
  const uint64_t at = (uint64_t)atMs * 1000ULL;
  schedule(pin, at, LOW);
  schedule(pin, at + (uint64_t)holdMs * 1000ULL, HIGH);
}

void turnEncoder(int32_t detents) {
  // From rest (A=B=HIGH), A leading B is +1 per StepScreenInput's table.
  const uint8_t lead = detents >= 0 ? PIN_ENC_A : PIN_ENC_B;
  const uint8_t lag = detents >= 0 ? PIN_ENC_B : PIN_ENC_A;
  const int32_t n = detents >= 0 ? detents : -detents;
  for (int32_t i = 0; i < n; i++) {
    writeLevel(lead, LOW);
    advanceUs(500);
    writeLevel(lag, LOW);
    advanceUs(500);
    writeLevel(lead, HIGH);
    advanceUs(500);
    writeLevel(lag, HIGH);
    advanceUs(2000);
  }
}

std::string sdKey(const char *path) {
  std::string key = path ? path : "";
  while (!key.empty() && key[0] == '/') {
    key.erase(0, 1);
  }
  for (char &c : key) {
    c = (char)toupper((unsigned char)c);
  }
  return key;
}

std::map<std::string, std::string> &sdFiles() {
  static std::map<std::string, std::string> files;
  return files;
}

void sdWrite(const char *path, const char *contents) {
  sdFiles()[sdKey(path)] = contents ? contents : "";
}

bool sdRead(const char *path, std::string &contents) {
  const auto it = sdFiles().find(sdKey(path));
  if (it == sdFiles().end()) {
    return false;
  }
  contents = it->second;
  return true;
}

void onDisplay(std::function<void()> fn) { g_onDisplay = std::move(fn); }

} // namespace StepScreenSim
