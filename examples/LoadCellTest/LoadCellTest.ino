/*
  StepScreen LoadCellTest

  HX711 load cell on the StepScreen SWD pads. Serial at 115200 baud.

  Wiring (StepScreen board silkscreen):
    HX711 SCK -> CLK  (PA30 / SWCLK)
    HX711 DT  -> DIO  (PA31 / SWDIO)
    HX711 VCC -> 3.3V, GND -> GND

  Serial commands:
    MEASURE | M     Continuous weight readout (default, every 200 ms)
    CAL | CALIBRATE Run interactive calibration (tare + known weight)
    TARE | T        Re-zero in measure mode
    HELP | ?        Command list

  On boot applies LOADCELL_OFFSET and LOADCELL_SCALE (from CAL, or defaults below).

  Board: Adafruit Feather M0 Adalogger (or compatible SAMD21).
*/

#include <StepScreenHX711.h>

#include <ctype.h>
#include <string.h>

#ifndef LOADCELL_OFFSET
#define LOADCELL_OFFSET -53731
#endif

#ifndef LOADCELL_SCALE
#define LOADCELL_SCALE -4286.284180f
#endif

const uint32_t READ_INTERVAL_MS = 200;
const uint8_t CAL_SAMPLES = 20;

enum RunMode : uint8_t {
  MODE_MEASURE,
  MODE_CALIBRATE,
};

RunMode runMode = MODE_MEASURE;
StepScreenHX711 scale;
uint32_t lastReadMs = 0;

#define MAX_CMD_LEN 16
char cmdBuf[MAX_CMD_LEN];
uint8_t cmdIdx = 0;

void flushSerialInput() {
  while (Serial.available() > 0) {
    Serial.read();
  }
}

void waitForEnter() {
  flushSerialInput();
  Serial.println(F("Press Enter to continue."));
  while (Serial.available() == 0) {
    delay(10);
  }
  while (Serial.available() > 0) {
    const char c = Serial.read();
    if (c == '\n' || c == '\r') {
      break;
    }
  }
}

uint32_t readGramsFromSerial() {
  flushSerialInput();
  Serial.println(F("Enter weight in whole grams, then Enter:"));

  uint32_t grams = 0;
  bool gotDigit = false;
  while (true) {
    if (Serial.available() > 0) {
      const char ch = Serial.read();
      if (ch == '\n' || ch == '\r') {
        if (gotDigit) {
          break;
        }
        continue;
      }
      if (isdigit(ch)) {
        gotDigit = true;
        grams = grams * 10u + (uint32_t)(ch - '0');
      }
    }
    delay(10);
  }
  return grams;
}

void printScaleConstants() {
  const int32_t offset = scale.get_offset();
  const float scaleVal = scale.get_scale();

  Serial.println();
  Serial.print(F("OFFSET: "));
  Serial.println(offset);
  Serial.print(F("SCALE:  "));
  Serial.println(scaleVal, 6);
  Serial.println();
  Serial.println(F("Paste into your sketch setup:"));
  Serial.print(F("  scale.set_offset("));
  Serial.print(offset);
  Serial.println(F(");"));
  Serial.print(F("  scale.set_scale("));
  Serial.print(scaleVal, 6);
  Serial.println(F(");"));
  Serial.println();
}

void runCalibration() {
  runMode = MODE_CALIBRATE;
  Serial.println();
  Serial.println(F("CALIBRATION"));
  Serial.println(F("==========="));

  Serial.println(F("Remove all weight from the load cell."));
  waitForEnter();

  Serial.println(F("Taring..."));
  scale.tare(CAL_SAMPLES);
  Serial.print(F("OFFSET: "));
  Serial.println(scale.get_offset());
  Serial.println();

  Serial.println(F("Place a known weight on the load cell."));
  waitForEnter();

  const uint32_t grams = readGramsFromSerial();
  Serial.print(F("WEIGHT: "));
  Serial.print(grams);
  Serial.println(F(" g"));

  scale.calibrate_scale((float)grams, CAL_SAMPLES);
  printScaleConstants();

  Serial.println(F("Calibration done. Returning to measure mode."));
  runMode = MODE_MEASURE;
  lastReadMs = 0;
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  MEASURE | M       Weight readout every 200 ms"));
  Serial.println(F("  CAL | CALIBRATE   Interactive tare + scale calibration"));
  Serial.println(F("  TARE | T          Re-zero (measure mode)"));
  Serial.println(F("  HELP | ?          This list"));
}

void printLoad() {
  const float grams = scale.get_units(3);
  Serial.print(F("load: "));
  Serial.print(grams, 2);
  Serial.println(F(" g"));
}

void processCommand(const char *cmd) {
  if (strcmp(cmd, "M") == 0 || strcmp(cmd, "MEASURE") == 0) {
    runMode = MODE_MEASURE;
    lastReadMs = 0;
    Serial.println(F("Measure mode."));
  } else if (strcmp(cmd, "CAL") == 0 || strcmp(cmd, "CALIBRATE") == 0) {
    runCalibration();
  } else if (strcmp(cmd, "T") == 0 || strcmp(cmd, "TARE") == 0) {
    Serial.println(F("Taring..."));
    scale.tare(10);
    Serial.println(F("Tare done."));
  } else if (strcmp(cmd, "?") == 0 || strcmp(cmd, "HELP") == 0) {
    printHelp();
  } else if (cmd[0] != '\0') {
    Serial.println(F("Unknown command. Type HELP."));
  }
}

void serialUpdate() {
  while (Serial.available() > 0) {
    const char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdIdx > 0) {
        cmdBuf[cmdIdx] = '\0';
        for (char *p = cmdBuf; *p; p++) {
          *p = (char)toupper(*p);
        }
        processCommand(cmdBuf);
        cmdIdx = 0;
      }
    } else if (cmdIdx < MAX_CMD_LEN - 1) {
      cmdBuf[cmdIdx++] = c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  scale.begin();
  scale.set_offset(LOADCELL_OFFSET);
  scale.set_scale(LOADCELL_SCALE);

  Serial.println(F("LoadCellTest"));
  Serial.print(F("Offset: "));
  Serial.println(LOADCELL_OFFSET);
  Serial.print(F("Scale:  "));
  Serial.println(LOADCELL_SCALE, 6);
  Serial.println(F("Type CAL to calibrate or HELP for commands."));
  Serial.println(F("Measure mode — readings every 200 ms."));
  printHelp();
}

void loop() {
  serialUpdate();

  if (runMode != MODE_MEASURE) {
    return;
  }

  if (millis() - lastReadMs >= READ_INTERVAL_MS && scale.is_ready()) {
    lastReadMs = millis();
    printLoad();
  }
}
