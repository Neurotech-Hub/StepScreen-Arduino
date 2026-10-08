// Host fake of the TMCStepper subset StepScreenTMC2209 uses. The driver
// answers version() == 0x21 only when StepScreenSim::hw().motorDriver is set.

#ifndef TMCStepper_h
#define TMCStepper_h

#include <Arduino.h>

class TMC2209Stepper {
public:
  TMC2209Stepper(Stream *, float, uint8_t) {}

  void begin() {}
  uint8_t version();
  bool test_connection() { return version() == 0x21 ? 0 : 1; }

  void pdn_disable(bool) {}
  void mstep_reg_select(bool) {}
  void I_scale_analog(bool) {}
  void toff(uint8_t) {}
  void rms_current(uint16_t ma) { _rmsMa = ma; }
  uint16_t rms_current() { return _rmsMa; }
  void intpol(bool on) { _intpol = on; }
  bool intpol() { return _intpol; }
  void microsteps(uint16_t ms) { _microsteps = ms; }
  uint16_t microsteps() { return _microsteps; }
  void en_spreadCycle(bool) {}
  void pwm_autoscale(bool) {}
  void blank_time(uint8_t) {}
  void TCOOLTHRS(uint32_t) {}
  void SGTHRS(uint8_t) {}
  uint16_t SG_RESULT() { return 0; }

private:
  uint16_t _rmsMa = 0;
  uint16_t _microsteps = 0;
  bool _intpol = true;
};

#endif // TMCStepper_h
