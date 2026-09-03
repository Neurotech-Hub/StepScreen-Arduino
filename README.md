# StepScreen

**StepScreen** is the control platform behind Neurotech Hub instruments — a compact board and software stack that pairs a sharp OLED display and rotary encoder with precision stepper motion, onboard storage, and lab-ready I/O.

Built for hands-on neuroscience and behavioral setups, StepScreen powers applications like syringe pumps and rodent treadmills: set parameters on-device, start a run with a single button, and keep the experiment moving without a laptop in the cage room. The same hardware also supports load cells, beam detectors, indicator lights, and battery-aware field use.

What you get:

- **On-device UI** — 1.3" OLED with encoder and dedicated Back / Select / Confirm controls for clear, gloved-hand menus
- **Motion control** — quiet, accurate stepper driving for pumps, belts, and custom actuators
- **Experiment I/O** — sensors, LEDs, external triggers, and microSD logging on a familiar Feather-class footprint
- **Ready-to-run apps** — example firmware for syringe delivery and treadmill sessions you can deploy or adapt

Open-source firmware and pin maps live below for builders who want to extend or rebrand the platform.

## Library structure

Source lives under `src/` (Arduino's modern layout). Only files in `src/` are compiled as library code, which keeps the root clean as the library grows:

```
StepScreen/
├── library.properties
├── README.md
├── StepScreenPins_override.example.h   # pin override template (not compiled)
├── src/                                # library source
│   ├── StepScreen.h                    # main include
│   └── ...
└── examples/
    ├── ScreenTest/
    └── BasicUI/
```

Sketches still use `#include <StepScreen.h>` — the IDE adds `src/` to the include path automatically.

## Hardware

- **Board**: ATSAMD21G18 @ 48 MHz, 3.3V logic (select "Adafruit Feather M0" in the Arduino IDE)
- **Display**: 1.3" 128x64 monochrome OLED, SH1106 driver, I2C at `0x3C` on `Wire` (pins 20/21)
- **RTC**: PCF8523 on the same I2C bus at `0x68` (Adalogger onboard RTC; optional)
- **Encoder**: EC11, 20 pulses / 20 detents, with push switch
- **Buttons**: independent back and confirm buttons on the module

## Dependencies

Install via the Arduino Library Manager:

- Adafruit SH110X
- Adafruit GFX Library
- Adafruit BusIO
- AccelStepper (used by `StepScreenStepper`)
- TMCStepper (used by `StepScreenTMC2209`)
- RTClib (used by `StepScreenRTC`)

## Pin map

Screen module defaults (from `StepScreenPins.h`):

| Symbol | Default pin | Module signal |
|---|---|---|
| `PIN_ENC_A` | 12 | SCREEN_TRA (encoder quadrature A) |
| `PIN_ENC_B` | 10 | SCREEN_TRB (encoder quadrature B) |
| `PIN_BTN_BACK` | 11 | SCREEN_BAK (back button) |
| `PIN_BTN_CONFIRM` | A0 | SCREEN_CONFIRM (confirm button) |
| `PIN_BTN_PUSH` | A3 | SCREEN_PUSH (encoder shaft push) |

Board I/O (from `StepScreenBoard.h`):

| Symbol | Default pin | Notes |
|---|---|---|
| `PIN_SD_CS` | 4 | microSD chip select |
| `PIN_SD_CD` | 7 | card detect (`STEPSCREEN_SD_CD_INSERTED`, default LOW) |
| `PIN_LED_GREEN` / `PIN_USER_LED1` | 13 | green user LED (D13) |
| `PIN_LED_RED` / `PIN_USER_LED2` | A1 | red user LED |
| `PIN_EXT_LED` | A2 | external LED via N-MOSFET (220 Ω gate, 10 kΩ pulldown); HIGH = ON |
| `PIN_BEAM_DET` | A4 | photodetector; **external pull-up** — use `INPUT`, no internal pullup |
| `PIN_AUX_IN` | A5 | general-purpose digital input |
| `PIN_VBAT` | A7 (pin 9) | battery voltage divider (reads VBAT/2) |
| `PIN_I2C_SDA` / `PIN_I2C_SCL` | 20 / 21 | Wire |
| `STEPSCREEN_RTC_I2C_ADDR` | 0x68 | PCF8523 RTC (optional) |
| `PIN_SWCLK` / `PIN_SWDIO` | PA30 (30) / PA31 (31) | silkscreen SWCLK / SWDIO — HX711 SCK/DT |
| `PIN_HX711_SCK` / `PIN_HX711_DT` | PA30 / PA31 | aliases of `PIN_SWCLK` / `PIN_SWDIO` |

Optional helpers: `#include <StepScreenIO.h>` for output registry and input sampling; `#include <StepScreenIODisplay.h>` to render I/O status on the OLED; `#include <StepScreenSD.h>` for microSD mount/read/write with card-detect hot-plug; `#include <StepScreenHX711.h>` for HX711 load cell on SWD pads; `#include <StepScreenStore.h>` for small EEPROM blob persistence on SAMD21; `#include <StepScreenRTC.h>` for PCF8523 date/time. See the **IOTest**, **LoadCellTest**, **Treadmill**, **Truss**, and **SyringePump** examples.

TMC2209 stepper (step/dir + half-duplex UART, from `StepScreenBoard.h`):

| Symbol | Default pin | Notes |
|---|---|---|
| `PIN_MOTOR_STEP` | 5 | STEP pulse |
| `PIN_MOTOR_DIR` | 6 | direction |
| `PIN_MOTOR_EN` | 9 | ~EN, active LOW (LOW = enabled) |
| `PIN_MOTOR_UART_RX` | 0 | D0 — half-duplex UART node with TX |
| `PIN_MOTOR_UART_TX` | 1 | D1 — 1kΩ in series to TMC PDN_UART |

MS1 and MS2 are tied HIGH on the module (UART address `0b11`). Runtime microstep count is configured over UART via `StepScreenTMC2209`, not the strap pins. `STEPSCREEN_MOTOR_MICROSTEPS` defaults to **128**; `STEPSCREEN_MOTOR_STEPS_PER_REV` is derived (200 full steps × microsteps). Override in a sketch before includes:

```cpp
#define STEPSCREEN_MOTOR_MICROSTEPS 64
#include <StepScreenStepper.h>
```

### microSD (`StepScreenSD`)

The Adalogger exposes SPI chip select on pin 4 and a mechanical card-detect switch on pin 7. On the StepScreen board the CD pin reads **LOW** when a card is inserted (`INPUT_PULLUP`). Stock [Adalogger pinouts](https://learn.adafruit.com/adafruit-feather-m0-adalogger/pinouts) use the opposite polarity — add `#define STEPSCREEN_SD_CD_INSERTED HIGH` before `#include <StepScreenSD.h>` if needed.

```cpp
#include <StepScreenSD.h>

StepScreenSD sd;
sd.begin();          // optional cs/cd pins; defaults PIN_SD_CS / PIN_SD_CD

void loop() {
  sd.update();       // required for hot-plug
  if (sd.isReady()) {
    sd.appendLine("/log.txt", "sample");
  }
}
```

See **IOTest** for automatic output walk, live dashboard, and SD self-test on mount.

**SD timestamps:** Arduino `SD.h` does not wire the RTC into FAT directory entry timestamps, so files may show incorrect dates in the OS. For correct FAT times, a future change would need SdFat with `FsDateTime::setCallback`. Until then, prefix log lines with `StepScreenRTC::formatLogTimestamp()` in the file content itself.

### RTC (`StepScreenRTC`)

The Adalogger PCF8523 shares `Wire` with the OLED at `STEPSCREEN_RTC_I2C_ADDR` (default `0x68`). Call `begin()` after the display (or any other code that starts I2C). `syncFromBuildTimeIfNeeded()` sets the clock from the firmware build stamp (`__DATE__` / `__TIME__`) when the RTC is unset, lost power, or behind the compile time.

```cpp
#include <StepScreenRTC.h>

StepScreenRTC rtc;
rtc.begin();                        // after screen.begin()
rtc.syncFromBuildTimeIfNeeded();    // sets from __DATE__/__TIME__ when needed

char header[12];
rtc.formatInfoBar(header, sizeof(header));
screen.drawInfoBar(header);
```

If the RTC is absent, `formatInfoBar()` returns `"-- --:--"` and the sketch continues normally. See **Treadmill** and **SyringePump** for live date/time in the info bar.

### HX711 load cell (`StepScreenHX711`)

The StepScreen board routes the HX711 to the SWD pads (PA30/PA31), silkscreened **CLK** and **DIO**. These pins are not in the stock Feather M0 Arduino pin map, so `StepScreenHX711` bit-bangs the HX711 protocol via SAMD PORT registers. SWD hardware debug is unavailable while the load cell is connected.

```cpp
#include <StepScreenHX711.h>

StepScreenHX711 scale;
scale.begin();
if (scale.verify()) {
  scale.set_scale(420.0f);   // raw counts per gram — calibrate once
  scale.tare(10);
  float grams = scale.get_units(3);
}
```

See **LoadCellTest** for serial measure mode (200 ms) and **CAL** interactive calibration. See **Truss** for on-device calibration saved to SD (`/TRUSS.CAL`) and closed-loop cable tension control.

Non-blocking reads: poll `is_ready()` and call `get_units(1)` when DOUT is low — one conversion per call, typically ~10 Hz (80 Hz if the HX711 module **RATE** pin is tied high).

All buttons are read active LOW with internal pullups; no external resistors are needed. Swap `PIN_ENC_A`/`PIN_ENC_B` to flip the encoder's rotation sign.

### Overriding pins

Nothing in the library needs to be edited. Define the pins you want to change in your sketch **before** including the library:

```cpp
#define PIN_ENC_A 6
#define PIN_BTN_BACK 5
#include <StepScreen.h>
```

Alternatively (PlatformIO or any build where your project folder is on the include path), copy `StepScreenPins_override.example.h` into your project as `StepScreenPins_override.h` and edit it there -- `StepScreenPins.h` picks it up automatically.

## Screen layout

The 128x64 panel is divided into fixed zones (constants in `StepScreenLayout.h`):

```text
x=0                  x=100    x=127
+---------------------+--------+  y=0
| Info bar (title)    |   Back |  y=0..7
+---------------------+--------+  y=8
|                     |        |
| Content (100x56)    |    Sel |  <- encoder push
|                     |        |
|                     |     OK |  <- confirm
+---------------------+--------+  y=63
```

The action column labels mirror the physical button positions: back at the top right, encoder push in the middle, confirm at the lower right. Active labels are drawn inverted.

## Quick start

```cpp
#include <StepScreen.h>

StepScreen screen;
int32_t counter = 0;

void setup() {
  screen.begin();

  // --- STEPSCREEN ENCODER ISR SETUP (copy into setup()) ---
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A),
                  StepScreenInput::handleEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B),
                  StepScreenInput::handleEncoderISR, CHANGE);
  // --- END STEPSCREEN ENCODER ISR SETUP ---
}

void loop() {
  counter += screen.input().getEncoderDelta();

  StepScreenDisplay &d = screen.display();
  d.clearDisplay();
  d.drawInfoBar("My App");
  d.drawActionColumn(screen.input().readBack(),
                     screen.input().readPush(),
                     screen.input().readConfirm());
  d.setCursor(0, 16);
  d.print(counter);
  d.display();
}
```

## Encoder interrupts (must live in the sketch)

The library provides the ISR body (`StepScreenInput::handleEncoderISR`) but never calls `attachInterrupt()` itself, so each sketch stays in control of its interrupt configuration. **Every new sketch must copy the marked block above into `setup()`**, after `screen.begin()`. The macro `STEPSCREEN_ATTACH_ENCODER_ISRS();` from `StepScreenInterrupts.h` expands to the same block if you prefer a one-liner.

Rotation is then consumed in the loop:

- `input().getEncoderDelta()` -- detents since the last call (signed)
- `input().getEncoderCount()` -- absolute detent count since `begin()`

Buttons are polled and debounced (no interrupts needed): `readBack()` / `readPush()` / `readConfirm()` return the held state, and `pollButtons()` returns edge-detected press events (`STEPSCREEN_EVT_BACK`, `STEPSCREEN_EVT_PUSH`, `STEPSCREEN_EVT_CONFIRM`).

SAMD21 note: pins 12 and 10 use separate EXTINT lines, so the default mapping is safe. If you remap the encoder, the two pins must not share an EXTINT line.

## TMC2209 UART: StepScreenTMC2209

Half-duplex UART on D0/D1 configures the BIGTREETECH TMC2209 before motion. Wiring:

```text
D1 (TX) --[1kΩ]--+-- TMC PDN_UART
D0 (RX) ---------+
TMC TX pad: NC
```

Call `configureForStepDir()` before `enable()` on `StepScreenMotor` or `StepScreenStepper`:

```cpp
#include <StepScreenTMC2209.h>
#include <StepScreenStepper.h>

StepScreenTMC2209 tmc;
StepScreenStepper stepper;

void setup() {
  tmc.begin();
  if (!tmc.verify()) {
    StepScreenTMC2209::printWiringHelp();
    return;
  }
  tmc.configureForStepDir();           // uses STEPSCREEN_MOTOR_MICROSTEPS
  // tmc.configureForStepDir(64);      // optional per-call override
  stepper.begin();                     // or stepper.beginWithDriver(tmc);
  stepper.enable();
}
```

| Call | Purpose |
|---|---|
| `begin()` | Start Serial1 + driver |
| `verify()` | Confirm UART link (`version()` == 0x21) |
| `configureForStepDir(microsteps, mA)` | UART mode, chopper, current, microsteps |
| `driver()` | Raw `TMC2209Stepper` for advanced register access |
| `printWiringHelp()` | Serial wiring checklist |

See **DriverTest** (Troubleshooting) for a full UART bring-up sequence.

## TMC2209 step/dir: StepScreenMotor

`StepScreenMotor` generates STEP/DIR/~EN pulses only — no UART. Use after `StepScreenTMC2209::configureForStepDir()`:

```cpp
#include <StepScreenMotor.h>

StepScreenMotor motor;

void setup() {
  motor.begin();
  motor.enable();
  motor.moveStepsSigned(STEPSCREEN_MOTOR_STEPS_PER_REV / 10);
}

void loop() {
  motor.setSpeed(400.0f);  // steps/sec; sign sets direction
  motor.runSpeed();        // call every loop() iteration
}
```

| Call | Purpose |
|---|---|
| `begin()` | Configure STEP/DIR/~EN pins; driver starts disabled |
| `enable()` / `disable()` | Toggle ~EN (active LOW) |
| `setDirection(fwd)` / `step()` | Set DIR and issue one microstep pulse |
| `moveSteps(n, delayUs)` | Blocking move in current direction |
| `moveStepsSigned(n, delayUs)` | Blocking move; sign sets direction |
| `setSpeed(stepsPerSec)` / `runSpeed()` | Non-blocking continuous motion |
| `getPosition()` / `resetPosition()` | Signed microstep counter |
| `getStepCount()` | Total pulses issued since `begin()` |

## Application motion: StepScreenStepper (AccelStepper)

For application motion — acceleration, non-blocking moves, speed presets — use `StepScreenStepper`, a thin wrapper around [AccelStepper](https://www.airspayce.com/mikem/arduino/AccelStepper/) in DRIVER mode with the board's pins and ~EN polarity preconfigured. Configure UART first via `StepScreenTMC2209`; `StepScreenMotor` remains the raw pin-level option for hardware bring-up.

```cpp
#include <StepScreenTMC2209.h>
#include <StepScreenStepper.h>

StepScreenTMC2209 tmc;
StepScreenStepper stepper;

void setup() {
  tmc.begin();
  tmc.verify();
  tmc.configureForStepDir();
  stepper.begin();          // driver starts disabled
  stepper.enable();
  stepper.moveSigned(STEPSCREEN_MOTOR_STEPS_PER_REV);
}

void loop() {
  stepper.run(); // non-blocking; call every loop() iteration
}
```

| Call | Purpose |
|---|---|
| `begin()` | Configure pins + defaults; UART must be configured first |
| `beginWithDriver(tmc)` | `verify()` + `configureForStepDir()` + `begin()` |
| `enable()` / `disable()` | Toggle ~EN; `disable()` also cancels pending motion |
| `jogSteps(n)` / `moveSigned(n)` | Relative moves (poll `run()` until done) |
| `run()` | Step generator; returns `true` while a move is pending |
| `setSpeedPreset(p)` / `nextSpeedPreset()` | Low / Med / Fast presets (`STEPSCREEN_SPEED_*_SPS`, overridable) |
| `setMaxSpeed(sps)` / `setAcceleration(sps2)` | Direct control |
| `isRunning()` / `stop()` / `currentPosition()` | Motion state |
| `raw()` | Underlying `AccelStepper` for advanced use |

Preset defaults: Low 200, Med 800, Fast 2000 microsteps/sec; acceleration 4000 microsteps/sec². Override any of them before including the header (e.g. `#define STEPSCREEN_SPEED_FAST_SPS 4000.0f`).

## Menu navigation: StepScreenNav

`StepScreenInput`'s 20 ms debounce handles switch bounce, but menus need more: a press that triggers a screen change must not also fire on the new screen, and a held button must not repeat. `StepScreenNav` layers both guarantees on top of the raw input:

- **Transition settle** — all button edges are discarded for `STEPSCREEN_NAV_SETTLE_MS` (default 250 ms) after `enterScreen()`.
- **Release-before-accept** — after a press is delivered, nothing more is accepted until every button has been released.

```cpp
#include <StepScreenNav.h>

StepScreenNav nav;

void setup() {
  // after screen.begin() ...
  nav.begin(&screen.input());
}

void changeScreen() {
  nav.enterScreen(); // call on EVERY screen/mode change
}

void loop() {
  uint8_t ev = nav.pollNavEvents();       // filtered STEPSCREEN_EVT_* bitmask
  int32_t enc = nav.consumeEncoderDelta(); // flushed by enterScreen()
  // nav.readBack()/readPush()/readConfirm() for UI highlights only
}
```

Call `pollNavEvents()` exactly once per loop — it consumes the underlying edge state.

## Examples

- **ScreenTest** -- display validation followed by an interactive control check (encoder + all three buttons). Run this first on new hardware.
- **BasicUI** -- interactive demo: the encoder drives a counter, and each button highlights its action-column label. Includes the encoder ISR block.
- **MotorTest** -- TMC2209 UART verify + step/dir exercise without a motor connected. Blocking and non-blocking moves with Serial reporting; green LED blinks on each STEP pulse.
- **DriverTest** (Troubleshooting) -- UART bring-up: version check, microstep register sweep, physical move at `STEPSCREEN_MOTOR_MICROSTEPS`.
- **IOTest** -- auto-walks each board output solo while a single dashboard shows inputs, encoder, VBAT, and SD status. SD self-test runs automatically when a card mounts.
- **LoadCellTest** -- HX711 load cell on SWCLK/SWDIO (CLK/DIO). Measure mode prints grams every 200 ms; type **CAL** for interactive tare/scale calibration over Serial.
- **SyringePump** -- three-screen pump controller built on `StepScreenNav` + `StepScreenStepper`. Home (motor off), Adjust (encoder jogs the motor; OK cycles Low/Med/Fast speed), Run (send a signed step count over Serial at 115200 — one revolution equals `STEPSCREEN_MOTOR_STEPS_PER_REV`). Demonstrates the menu debounce pattern.
- **Treadmill** -- rodent treadmill controller. Home shows speed (cm/s) and timeout (minutes) with the edited metric inverted (Back toggles Spd/Time, Sel toggles Fwd/Rev, encoder adjusts). Run ramps the belt to speed via `setSpeed()`/`runSpeed()`, counts down, then disables the motor and blinks DONE. Speed→steps mapping is a placeholder (`STEPSCREEN_TREADMILL_SPS_PER_CMS`).
- **Truss** -- closed-loop cable tension controller for a load-cell inline with a stepper spool. Home shows live load (grams) or `NO CAL`; encoder sets target load (0–500 g). Startup warns if stepper (12 V/TMC), load cell, or SD card is missing; Home footer lists anything still offline. **Cal** requires load cell + SD: two-step tare then known-weight calibration saved to `/TRUSS.CAL`. **Run** requires stepper, load cell, SD, and valid cal. Control uses hysteresis (`TRUSS_STOP_G` / `TRUSS_START_G`), continuous creep at `TRUSS_APPROACH_SPS` in the mid band, and a few-step nudge (`TRUSS_NUDGE_STEPS` inside `TRUSS_NUDGE_G`); farther out, speed scales with load error (`TRUSS_GAIN_SPS_PER_G`, capped at `TRUSS_SPS`). Commands update on each HX711 sample; an EMA filter (`TRUSS_FILTER_ALPHA`) smooths readings. Prefer an 80 Hz HX711 RATE jumper (many modules ship at 10 Hz). Direction-timeout faults only apply far from target when error is not improving. A TC3 step ISR drives pulses while a 5 ms ramp (`TRUSS_RAMP_MS`) smooths speed changes. Set `TRUSS_WIND_SIGN` if wind/unwind is reversed. Serial telemetry every `TRUSS_DEBUG_MS` (0 to disable). Session logging (SD + RTC) writes `/YYYYMMDD/HHMMSS.CSV` with columns `datetime,event,target_load_g,actual_load_g` and events `begin` / `measure` (~1 Hz, 2-minute RAM buffer) / `end` / `fault`; a full buffer briefly holds the motor (enabled, zero speed) while flushing. **HOLD** pauses the loop and timer; overload, step-limit, and direction-timeout faults stop the motor.

## API summary

| Call | Purpose |
|---|---|
| `screen.begin()` | Initialize display + inputs (returns `false` if the OLED is missing) |
| `screen.display()` | The `StepScreenDisplay` (full GFX API + helpers below) |
| `screen.input()` | Buttons + encoder |
| `drawInfoBar(title, highlightBack)` | Inverted top bar with left-aligned title |
| `drawActionColumn(back, push, confirm)` | Right-justified Back/Sel/OK labels |
| `drawActionColumn(labels, back, push, confirm)` | Per-screen labels via `StepScreenActionLabels`; `nullptr` leaves that slot blank (unavailable). Labels longer than 4 chars are truncated |
| `clearContentArea()` | Clear only the content zone, cursor to its origin |
| `drawLayoutGuides()` | Zone borders + corner markers (for testing) |

Layout helpers draw at text size 1; set your own size afterwards for content.
