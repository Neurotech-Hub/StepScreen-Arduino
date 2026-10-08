// Replaces src/StepScreenHX711.cpp (SAMD PORT bit-banging). Conversions
// arrive at hw().sampleHz; read() blocks (in virtual time) like the real
// chip. Math helpers match the real implementation.

#include <StepScreenHX711.h>

#include "SimInternal.h"
#include "StepScreenSim.h"

namespace {

uint64_t g_nextSampleUs = 0;

uint64_t samplePeriodUs() {
  const float hz = StepScreenSim::hw().sampleHz;
  return hz > 0.0f ? (uint64_t)(1000000.0f / hz) : 12500;
}

} // namespace

void StepScreenHX711::configureGpio() {}

uint8_t StepScreenHX711::shiftIn() { return 0; }

void StepScreenHX711::begin() {
  delay(400);
  _offset = 0;
  _scale = 1.0f;
  _gain = 128;
  g_nextSampleUs = StepScreenSimInternal::nowUs();
}

bool StepScreenHX711::verify(uint32_t timeoutMs) {
  if (!StepScreenSim::hw().loadCell) {
    delay(timeoutMs);
    return false;
  }
  (void)read();
  return true;
}

bool StepScreenHX711::is_ready() {
  return StepScreenSim::hw().loadCell &&
         StepScreenSimInternal::nowUs() >= g_nextSampleUs;
}

float StepScreenHX711::read() {
  const uint64_t now = StepScreenSimInternal::nowUs();
  if (now < g_nextSampleUs) {
    StepScreenSim::advanceUs(g_nextSampleUs - now);
  }
  g_nextSampleUs = StepScreenSimInternal::nowUs() + samplePeriodUs();

  const StepScreenSim::Hardware &hw = StepScreenSim::hw();
  if (!hw.loadCell) {
    return 0.0f;
  }
  return (float)lroundf(hw.loadGrams * hw.countsPerGram +
                        (float)hw.offsetCounts);
}

float StepScreenHX711::read_average(uint8_t times) {
  if (times < 1) {
    times = 1;
  }
  float sum = 0.0f;
  for (uint8_t i = 0; i < times; i++) {
    sum += read();
  }
  return sum / times;
}

void StepScreenHX711::tare(uint8_t times) {
  _offset = (int32_t)read_average(times);
}

bool StepScreenHX711::set_scale(float scale) {
  if (scale == 0.0f) {
    return false;
  }
  _scale = 1.0f / scale;
  return true;
}

float StepScreenHX711::get_scale() const { return 1.0f / _scale; }

void StepScreenHX711::set_offset(int32_t offset) { _offset = offset; }

int32_t StepScreenHX711::get_offset() const { return _offset; }

float StepScreenHX711::get_value(uint8_t times) {
  return read_average(times) - (float)_offset;
}

float StepScreenHX711::get_units(uint8_t times) {
  return get_value(times) * _scale;
}

void StepScreenHX711::calibrate_scale(float weight, uint8_t times) {
  const float counts = read_average(times) - (float)_offset;
  if (counts != 0.0f && weight != 0.0f) {
    _scale = weight / counts;
  }
}

void StepScreenHX711::power_down() {}

void StepScreenHX711::power_up() {}
