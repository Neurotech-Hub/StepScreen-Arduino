// Harness-internal hooks shared between the core, fakes and capture code.

#ifndef STEPSCREEN_SIM_INTERNAL_H
#define STEPSCREEN_SIM_INTERNAL_H

#include <stdint.h>
#include <stdio.h>

namespace StepScreenSimInternal {

uint64_t nowUs(); // virtual clock without the per-read tick
void setSerialSink(FILE *log, bool echo);
void writeSerial(uint8_t c);

// Called by the fake SH110X on display(): store the frame, then run the
// scene's onDisplay hook.
void publishFrame(const uint8_t *pages, bool inverted);
void notifyDisplay();

} // namespace StepScreenSimInternal

#endif // STEPSCREEN_SIM_INTERNAL_H
