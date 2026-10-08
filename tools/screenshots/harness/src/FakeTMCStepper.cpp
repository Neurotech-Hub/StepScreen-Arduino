#include <TMCStepper.h>

#include "StepScreenSim.h"

uint8_t TMC2209Stepper::version() {
  return StepScreenSim::hw().motorDriver ? 0x21 : 0x00;
}
