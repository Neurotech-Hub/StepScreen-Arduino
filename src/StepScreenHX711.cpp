#include "StepScreenHX711.h"

#if !defined(ARDUINO_ARCH_SAMD)
#error StepScreenHX711 requires SAMD21 (PA30/PA31 SWD GPIO)
#endif

namespace {

constexpr uint8_t kGain128 = 128;

PortGroup *hx711Port() { return &PORT->Group[0]; }

inline uint32_t sckMask() { return (1ul << PIN_SWCLK); }

inline uint32_t dtMask() { return (1ul << PIN_SWDIO); }

} // namespace

void StepScreenHX711::configureGpio() {
  PortGroup *port = hx711Port();

  port->PINCFG[PIN_SWCLK].reg = PORT_PINCFG_INEN | PORT_PINCFG_DRVSTR;
  port->DIRSET.reg = sckMask();
  port->OUTCLR.reg = sckMask();

  port->PINCFG[PIN_SWDIO].reg = PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;
  port->DIRCLR.reg = dtMask();
  port->OUTSET.reg = dtMask();
}

void StepScreenHX711::begin() {
  configureGpio();
  power_down();
  power_up();
  _offset = 0;
  _scale = 1.0f;
  _gain = kGain128;
}

bool StepScreenHX711::is_ready() {
  return (hx711Port()->IN.reg & dtMask()) == 0;
}

void StepScreenHX711::power_down() {
  hx711Port()->OUTSET.reg = sckMask();
  delayMicroseconds(64);
}

void StepScreenHX711::power_up() {
  hx711Port()->OUTCLR.reg = sckMask();
}

uint8_t StepScreenHX711::shiftIn() {
  PortGroup *port = hx711Port();
  const uint32_t sck = sckMask();
  const uint32_t dt = dtMask();
  uint8_t value = 0;
  uint8_t mask = 0x80;

  while (mask > 0) {
    port->OUTSET.reg = sck;
    if (_fastProcessor) {
      delayMicroseconds(1);
    }
    if (port->IN.reg & dt) {
      value |= mask;
    }
    port->OUTCLR.reg = sck;
    if (_fastProcessor) {
      delayMicroseconds(1);
    }
    mask >>= 1;
  }
  return value;
}

float StepScreenHX711::read() {
  PortGroup *port = hx711Port();
  const uint32_t sck = sckMask();
  const uint32_t dt = dtMask();

  while (port->IN.reg & dt) {
    yield();
  }

  union {
    int32_t value = 0;
    uint8_t data[4];
  } v;

  noInterrupts();

  v.data[2] = shiftIn();
  v.data[1] = shiftIn();
  v.data[0] = shiftIn();

  uint8_t pulses = 1;
  if (_gain == 64) {
    pulses = 3;
  } else if (_gain == 32) {
    pulses = 2;
  }

  while (pulses > 0) {
    port->OUTSET.reg = sck;
    if (_fastProcessor) {
      delayMicroseconds(1);
    }
    port->OUTCLR.reg = sck;
    if (_fastProcessor) {
      delayMicroseconds(1);
    }
    pulses--;
  }

  interrupts();

  if (v.data[2] & 0x80) {
    v.data[3] = 0xFF;
  }

  return (float)v.value;
}

float StepScreenHX711::read_average(uint8_t times) {
  if (times < 1) {
    times = 1;
  }
  float sum = 0.0f;
  for (uint8_t i = 0; i < times; i++) {
    sum += read();
    yield();
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
