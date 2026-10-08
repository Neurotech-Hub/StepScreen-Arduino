/*!
 * Simulated StepScreen board for host screenshot scenes.
 *
 * Set hw() before calling the sketch's setup(), then drive the sketch with
 * run(loop, ms) and simulated input (press, turnEncoder). Everything runs
 * on a virtual clock, so a 90 s session renders in well under a second.
 */

#ifndef STEPSCREEN_SIM_H
#define STEPSCREEN_SIM_H

#include <Arduino.h>

#include <functional>
#include <map>
#include <string>

namespace StepScreenSim {

struct Hardware {
  // PCF8523 RTC. The clock starts at this date/time when the sketch boots
  // and advances with virtual millis(). rtcAllowAdjust lets the sketch set
  // the clock (e.g. syncFromBuildTimeIfNeeded); off keeps output stable.
  bool rtc = true;
  bool rtcAllowAdjust = false;
  uint16_t year = 2026;
  uint8_t month = 10, day = 8, hour = 9, minute = 41, second = 0;

  // microSD card in the slot (drives the card-detect pin).
  bool sdCard = true;

  // HX711 load cell. Raw counts = loadGrams * countsPerGram + offsetCounts.
  bool loadCell = true;
  float loadGrams = 0.0f;
  float countsPerGram = 420.0f;
  int32_t offsetCounts = 8000;
  float sampleHz = 80.0f;

  // TMC2209 driver answering on UART (motor power present).
  bool motorDriver = true;
};

Hardware &hw();

// Virtual time
void advanceMs(uint32_t ms);
void advanceUs(uint64_t us);

// Call loopFn repeatedly for ms of virtual time; each call advances the
// clock by tickUs (plus whatever the loop itself delays).
void run(void (*loopFn)(), uint32_t ms, uint32_t tickUs = 1000);

// Run until pred() is true or timeoutMs elapses; returns pred().
bool runUntil(void (*loopFn)(), const std::function<bool()> &pred,
              uint32_t timeoutMs, uint32_t tickUs = 1000);

// Pin levels. Unset pins read HIGH (pull-ups), except card detect, which
// follows hw().sdCard.
void setPin(uint8_t pin, int level);
int pinLevel(uint8_t pin);

// Buttons are active LOW: press now, release after holdMs (applied as the
// clock advances). pressAt schedules the press for a later virtual time.
void press(uint8_t pin, uint32_t holdMs = 80);
void pressAt(uint8_t pin, uint32_t atMs, uint32_t holdMs = 80);

// Quadrature encoder on PIN_ENC_A/PIN_ENC_B; positive detents match the
// direction StepScreenInput counts as positive.
void turnEncoder(int32_t detents);

// In-memory SD card contents, keyed by normalized path ("TRUSS.CAL",
// "20261008/094100.CSV"). Survives sketch-side SD.begin/end.
std::map<std::string, std::string> &sdFiles();
void sdWrite(const char *path, const char *contents);
bool sdRead(const char *path, std::string &contents);
std::string sdKey(const char *path);

// Called after every display() (e.g. to keep a mid-animation frame).
void onDisplay(std::function<void()> fn);

} // namespace StepScreenSim

#endif // STEPSCREEN_SIM_H
