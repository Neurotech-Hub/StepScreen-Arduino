/*
  StepScreen ToFTest

  Adafruit VL53L4CD Time-of-Flight ranging on I2C (default 0x29) plus TMC2209
  motor control. Distance is printed over Serial and shown on the OLED.

  Hardware:
    ToF   Adafruit VL53L4CD on Wire (SDA 20 / SCL 21), I2C address 0x29
          XSHUT optional -- set PIN_TOF_XSHUT if wired to the Feather
    RTC   optional at 0x68 (shares the bus; not used by this sketch)
    OLED  optional at 0x3C -- skipped automatically if not present
    Motor BIGTREETECH TMC2209 standalone step/dir (MotorTest pins):
            STEP -> D5  (PIN_MOTOR_STEP)
            DIR  -> D6  (PIN_MOTOR_DIR)
            ~EN  -> D9  (PIN_MOTOR_EN, active LOW) — held enabled
            MS1/MS2 tied HIGH (1/16 microstepping, 3200 steps/rev)

  Serial commands (115200 baud, newline-terminated):
    MOVE <steps> [speed_sps]   Fixed move. Sign = direction.
                               Optional speed in microsteps/sec.
    CONT <speed_sps>           Continuous move. Sign = direction.
    DIR F|R | FWD | REV        Set direction (applies to next CONT
                               if you use unsigned CONT SPEED + DIR).
    SPEED <sps>                Default max / continuous speed (abs).
    STOP                       Halt motion
    TOF                        Print last range (also streams continuously)
    STATUS                     Motor + ToF summary
    RESET POS                  Zero motor position
    INIT                       Re-init ToF
    SCAN                       I2C bus scan
    HELP

  ToF readings print continuously as  tof=123 mm  (including during CONT).

  Dependencies (Library Manager):
    stm32duino VL53L4CD, AccelStepper, Adafruit SH110X (+ GFX, BusIO)

  Board: Adafruit Feather M0 Adalogger (or compatible).
*/

#include <StepScreen.h>
#include <StepScreenBoard.h>
#include <StepScreenStepper.h>
#include <Wire.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <vl53l4cd_class.h>

// Set to a GPIO if XSHUT is wired to the MCU; -1 if tied high on-module.
#ifndef PIN_TOF_XSHUT
#define PIN_TOF_XSHUT -1
#endif

#define TOF_I2C_ADDR_8BIT 0x52 // VL53L4CD default (7-bit 0x29)
#define TOF_POLL_MS 100
#define TOF_TIMING_BUDGET_MS 50
#define TOF_OUT_OF_RANGE_MM 4000
#define DRAW_INTERVAL_MS 150
#define DRAW_INTERVAL_MOVING_MS 400

#define DEFAULT_SPEED_SPS 800.0f
#define MAX_SPEED_SPS 2000.0f

StepScreen screen;
StepScreenStepper stepper;
VL53L4CD tof(&Wire, PIN_TOF_XSHUT);

enum MotionMode : uint8_t
{
  MODE_IDLE,
  MODE_FIXED,
  MODE_CONTINUOUS,
};

MotionMode motionMode = MODE_IDLE;
float defaultSpeedSps = DEFAULT_SPEED_SPS;
bool directionForward = true;
bool tofOk = false;
bool displayOk = false;

uint16_t lastRangeMm = 0;
bool lastRangeValid = false;
uint32_t lastTofMs = 0;
uint32_t lastDrawMs = 0;

#define MAX_CMD_LEN 64
char cmdBuf[MAX_CMD_LEN];
uint8_t cmdIdx = 0;

static void i2cInit()
{
  Wire.begin();
  Wire.setClock(100000);
#if defined(WIRE_HAS_TIMEOUT) || defined(ARDUINO_ARCH_SAMD)
  Wire.setTimeout(1000);
#endif
}

static bool i2cProbe(uint8_t addr7)
{
  Wire.beginTransmission(addr7);
  return Wire.endTransmission() == 0;
}

float clampSpeed(float sps)
{
  if (sps > MAX_SPEED_SPS)
  {
    return MAX_SPEED_SPS;
  }
  if (sps < -MAX_SPEED_SPS)
  {
    return -MAX_SPEED_SPS;
  }
  return sps;
}

void motionStop()
{
  stepper.stop();
  stepper.setSpeed(0.0f);
  motionMode = MODE_IDLE;
}

void startFixedMove(int32_t steps, float speedSps)
{
  if (steps == 0)
  {
    return;
  }

  const float absSpeed = fabsf(clampSpeed(speedSps));
  if (absSpeed <= 0.0f)
  {
    return;
  }

  directionForward = steps > 0;
  stepper.setMaxSpeed(absSpeed);
  stepper.setAcceleration(STEPSCREEN_STEPPER_ACCEL);
  stepper.moveSigned(steps);
  motionMode = MODE_FIXED;

  Serial.print(F("MOVE "));
  Serial.print(steps);
  Serial.print(F(" @ "));
  Serial.print(absSpeed, 1);
  Serial.println(F(" sps"));
}

void startContinuous(float speedSps)
{
  float sps = clampSpeed(speedSps);
  if (sps == 0.0f)
  {
    Serial.println(F("CONT speed must be non-zero"));
    return;
  }

  if (sps > 0.0f)
  {
    sps = directionForward ? sps : -sps;
  }
  else
  {
    directionForward = false;
  }

  const float absSpeed = fabsf(sps);
  stepper.setMaxSpeed(absSpeed);
  stepper.setSpeed(sps);
  motionMode = MODE_CONTINUOUS;

  Serial.print(F("CONT "));
  Serial.print(directionForward ? F("FWD") : F("REV"));
  Serial.print(F(" @ "));
  Serial.print(absSpeed, 1);
  Serial.println(F(" sps"));
}

void i2cScanBus()
{
  Serial.print(F("I2C scan:"));
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++)
  {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0)
    {
      Serial.print(F(" 0x"));
      if (addr < 16)
      {
        Serial.print('0');
      }
      Serial.print(addr, HEX);
      found++;
    }
  }
  if (found == 0)
  {
    Serial.print(F(" (none)"));
  }
  Serial.println();
}

bool initTof()
{
  i2cInit();
  delay(100);
  i2cScanBus();

  Serial.println(F("ToF: starting VL53L4CD init..."));
  tof.begin();
  tof.VL53L4CD_Off();

  const VL53L4CD_ERROR status = tof.InitSensor(TOF_I2C_ADDR_8BIT);
  if (status != VL53L4CD_ERROR_NONE)
  {
    Serial.print(F("ToF: InitSensor failed status="));
    Serial.println(status);
    return false;
  }

  uint16_t id = 0;
  if (tof.VL53L4CD_GetSensorId(&id) == VL53L4CD_ERROR_NONE)
  {
    Serial.print(F("ToF: sensor ID 0x"));
    Serial.println(id, HEX);
  }

  if (tof.VL53L4CD_SetRangeTiming(TOF_TIMING_BUDGET_MS, TOF_POLL_MS) !=
      VL53L4CD_ERROR_NONE)
  {
    Serial.println(F("ToF: SetRangeTiming failed"));
    return false;
  }

  if (tof.VL53L4CD_StartRanging() != VL53L4CD_ERROR_NONE)
  {
    Serial.println(F("ToF: StartRanging failed"));
    return false;
  }

  Serial.println(F("ToFTest: VL53L4CD ready @ 0x29"));
  return true;
}

void printTofLine()
{
  Serial.print(F("tof="));
  if (!tofOk)
  {
    Serial.println(F("N/A"));
  }
  else if (lastRangeValid)
  {
    Serial.print(lastRangeMm);
    Serial.println(F(" mm"));
  }
  else
  {
    Serial.println(F("out of range"));
  }
}

void printStatus()
{
  Serial.print(F("mode="));
  switch (motionMode)
  {
  case MODE_IDLE:
    Serial.print(F("IDLE"));
    break;
  case MODE_FIXED:
    Serial.print(F("FIXED"));
    break;
  case MODE_CONTINUOUS:
    Serial.print(F("CONT"));
    break;
  }
  Serial.print(F("  dir="));
  Serial.print(directionForward ? F("FWD") : F("REV"));
  Serial.print(F("  pos="));
  Serial.print(stepper.currentPosition());
  Serial.print(F("  speed="));
  Serial.print(defaultSpeedSps, 1);
  Serial.print(F("  "));
  printTofLine();
}

void pollTof()
{
  if (!tofOk)
  {
    return;
  }
  if (millis() - lastTofMs < TOF_POLL_MS)
  {
    return;
  }
  lastTofMs = millis();

  uint8_t ready = 0;
  if (tof.VL53L4CD_CheckForDataReady(&ready) != VL53L4CD_ERROR_NONE || !ready)
  {
    return;
  }

  tof.VL53L4CD_ClearInterrupt();

  VL53L4CD_Result_t result;
  if (tof.VL53L4CD_GetResult(&result) != VL53L4CD_ERROR_NONE)
  {
    return;
  }

  if (result.range_status == 0 && result.distance_mm > 0 &&
      result.distance_mm < TOF_OUT_OF_RANGE_MM)
  {
    lastRangeMm = result.distance_mm;
    lastRangeValid = true;
  }
  else
  {
    lastRangeValid = false;
  }

  printTofLine();
}

void processCommand()
{
  char *token = strtok(cmdBuf, " ");
  if (token == nullptr)
  {
    return;
  }
  for (char *p = token; *p; p++)
  {
    *p = (char)toupper(*p);
  }

  if (strcmp(token, "MOVE") == 0)
  {
    char *stepsStr = strtok(nullptr, " ");
    if (!stepsStr)
    {
      Serial.println(F("MOVE <steps> [speed_sps]"));
      return;
    }
    const long steps = atol(stepsStr);
    char *speedStr = strtok(nullptr, " ");
    float speed = speedStr ? atof(speedStr) : defaultSpeedSps;
    if (speed < 0.0f)
    {
      speed = -speed;
    }
    if (speed == 0.0f)
    {
      speed = defaultSpeedSps;
    }
    startFixedMove((int32_t)steps, speed);
  }
  else if (strcmp(token, "CONT") == 0)
  {
    char *speedStr = strtok(nullptr, " ");
    if (!speedStr)
    {
      Serial.println(F("CONT <speed_sps>"));
      return;
    }
    startContinuous(atof(speedStr));
  }
  else if (strcmp(token, "DIR") == 0)
  {
    char *dirStr = strtok(nullptr, " ");
    if (!dirStr)
    {
      Serial.println(F("DIR F|R"));
      return;
    }
    for (char *p = dirStr; *p; p++)
    {
      *p = (char)toupper(*p);
    }
    if (dirStr[0] == 'F')
    {
      directionForward = true;
    }
    else if (dirStr[0] == 'R' || dirStr[0] == 'B')
    {
      directionForward = false;
    }
    else
    {
      Serial.println(F("DIR F|R"));
      return;
    }
    if (motionMode == MODE_CONTINUOUS)
    {
      const float absSpeed = fabsf(stepper.speed());
      stepper.setSpeed(directionForward ? absSpeed : -absSpeed);
    }
    Serial.print(F("DIR "));
    Serial.println(directionForward ? F("FWD") : F("REV"));
  }
  else if (strcmp(token, "FWD") == 0)
  {
    directionForward = true;
    if (motionMode == MODE_CONTINUOUS)
    {
      stepper.setSpeed(fabsf(stepper.speed()));
    }
    Serial.println(F("DIR FWD"));
  }
  else if (strcmp(token, "REV") == 0)
  {
    directionForward = false;
    if (motionMode == MODE_CONTINUOUS)
    {
      stepper.setSpeed(-fabsf(stepper.speed()));
    }
    Serial.println(F("DIR REV"));
  }
  else if (strcmp(token, "SPEED") == 0)
  {
    char *speedStr = strtok(nullptr, " ");
    if (!speedStr)
    {
      Serial.println(F("SPEED <sps>"));
      return;
    }
    defaultSpeedSps = fabsf(clampSpeed(atof(speedStr)));
    if (defaultSpeedSps <= 0.0f)
    {
      defaultSpeedSps = DEFAULT_SPEED_SPS;
    }
    stepper.setMaxSpeed(defaultSpeedSps);
    if (motionMode == MODE_CONTINUOUS)
    {
      stepper.setSpeed(directionForward ? defaultSpeedSps : -defaultSpeedSps);
    }
    Serial.print(F("SPEED "));
    Serial.println(defaultSpeedSps, 1);
  }
  else if (strcmp(token, "STOP") == 0)
  {
    motionStop();
    Serial.println(F("STOP"));
  }
  else if (strcmp(token, "TOF") == 0 || strcmp(token, "RANGE") == 0)
  {
    printTofLine();
  }
  else if (strcmp(token, "SCAN") == 0)
  {
    i2cInit();
    delay(50);
    i2cScanBus();
  }
  else if (strcmp(token, "INIT") == 0)
  {
    if (tofOk)
    {
      tof.VL53L4CD_StopRanging();
    }
    tofOk = initTof();
    Serial.println(tofOk ? F("INIT done") : F("INIT failed"));
  }
  else if (strcmp(token, "STATUS") == 0)
  {
    printStatus();
  }
  else if (strcmp(token, "RESET") == 0)
  {
    char *sub = strtok(nullptr, " ");
    if (sub)
    {
      for (char *p = sub; *p; p++)
      {
        *p = (char)toupper(*p);
      }
    }
    if (sub && strcmp(sub, "POS") == 0)
    {
      stepper.resetPosition();
      Serial.println(F("RESET POS"));
    }
    else
    {
      Serial.println(F("RESET POS"));
    }
  }
  else if (strcmp(token, "HELP") == 0)
  {
    Serial.println(F("Commands:"));
    Serial.println(F("  MOVE <steps> [speed_sps]"));
    Serial.println(F("  CONT <speed_sps>"));
    Serial.println(F("  DIR F|R | FWD | REV"));
    Serial.println(F("  SPEED <sps>"));
    Serial.println(F("  STOP"));
    Serial.println(F("  TOF | SCAN | INIT | STATUS | RESET POS"));
  }
  else
  {
    Serial.println(F("Unknown command (HELP)"));
  }
}

void serialCmdUpdate()
{
  while (Serial.available() > 0)
  {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r')
    {
      if (cmdIdx > 0)
      {
        cmdBuf[cmdIdx] = '\0';
        processCommand();
        cmdIdx = 0;
      }
    }
    else if (cmdIdx < MAX_CMD_LEN - 1)
    {
      cmdBuf[cmdIdx++] = c;
    }
  }
}

void drawUi()
{
  if (!displayOk)
  {
    return;
  }

  StepScreenDisplay &d = screen.display();
  d.clearDisplay();
  d.drawInfoBar("VL53L4CD");
  d.drawActionColumn(false, false, false);

  d.setTextSize(1);
  d.setCursor(STEPSCREEN_CONTENT_X, STEPSCREEN_CONTENT_Y);
  d.print(F("Range "));
  if (!tofOk)
  {
    d.println(F("--"));
  }
  else if (lastRangeValid)
  {
    d.print(lastRangeMm);
    d.println(F(" mm"));
  }
  else
  {
    d.println(F("OOR"));
  }

  d.setCursor(STEPSCREEN_CONTENT_X, STEPSCREEN_CONTENT_Y + 12);
  d.print(F("Motor "));
  if (motionMode == MODE_CONTINUOUS)
  {
    d.print(F("CONT "));
  }
  else if (motionMode == MODE_FIXED)
  {
    d.print(F("MOVE "));
  }
  else
  {
    d.print(F("IDLE "));
  }
  d.println(directionForward ? F("FWD") : F("REV"));

  d.setCursor(STEPSCREEN_CONTENT_X, STEPSCREEN_CONTENT_Y + 24);
  d.print(F("Pos "));
  d.println(stepper.currentPosition());

  d.setCursor(STEPSCREEN_CONTENT_X, STEPSCREEN_CONTENT_Y + 36);
  d.print(F("Spd "));
  d.print(defaultSpeedSps, 0);
  d.println(F("  EN"));

  d.display();
}

void setup()
{
  Serial.begin(115200);
  delay(2000);
  Serial.println(F("Serial ready."));

  tofOk = initTof();
  if (!tofOk)
  {
    Serial.println(F("ToFTest: VL53L4CD init failed (motor still available)"));
  }

  if (i2cProbe(STEPSCREEN_I2C_ADDR))
  {
    displayOk = screen.display().begin(STEPSCREEN_I2C_ADDR);
    if (!displayOk)
    {
      Serial.println(F("ToFTest: OLED at 0x3C did not initialize"));
    }
  }
  else
  {
    Serial.println(F("ToFTest: OLED not present, skipping display"));
    displayOk = false;
  }

  stepper.begin();
  stepper.setMaxSpeed(defaultSpeedSps);
  stepper.enable();

  Serial.println(F("StepScreen ToFTest (VL53L4CD)"));
  Serial.print(F("  STEP pin "));
  Serial.println(PIN_MOTOR_STEP);
  Serial.print(F("  DIR  pin "));
  Serial.println(PIN_MOTOR_DIR);
  Serial.print(F("  ~EN  pin "));
  Serial.println(PIN_MOTOR_EN);
  Serial.print(F("  microsteps/rev "));
  Serial.println(STEPSCREEN_MOTOR_STEPS_PER_REV);
  Serial.println(F("Ready. Type HELP for commands."));
  if (tofOk)
  {
    Serial.println(F("ToF streams continuously (tof=... mm)."));
  }
  else
  {
    Serial.println(F("ToF offline. Type INIT to retry."));
  }

  lastDrawMs = millis();
  drawUi();
}

void loop()
{
  serialCmdUpdate();
  pollTof();

  if (motionMode == MODE_CONTINUOUS)
  {
    stepper.runSpeed();
  }
  else if (motionMode == MODE_FIXED)
  {
    stepper.run();
    if (!stepper.isRunning())
    {
      motionMode = MODE_IDLE;
      Serial.print(F("DONE  pos="));
      Serial.println(stepper.currentPosition());
    }
  }

  const uint32_t drawInterval =
      (motionMode != MODE_IDLE) ? DRAW_INTERVAL_MOVING_MS : DRAW_INTERVAL_MS;
  if (millis() - lastDrawMs >= drawInterval)
  {
    lastDrawMs = millis();
    drawUi();
  }
}
