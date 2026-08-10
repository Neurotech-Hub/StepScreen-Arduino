# StepScreen

Arduino UI library for a custom board modeled after the [Adafruit Feather M0 Adalogger](https://learn.adafruit.com/adafruit-feather-m0-adalogger/pinouts), paired with a 1.3" SH1106 OLED + EC11 rotary encoder module (I2C display, three active-LOW buttons: back, confirm, and encoder push).

Built on the [Adafruit SH110X](https://github.com/adafruit/Adafruit_SH110X) driver, so the full Adafruit_GFX API is available alongside the StepScreen layout helpers.

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
- **Encoder**: EC11, 20 pulses / 20 detents, with push switch
- **Buttons**: independent back and confirm buttons on the module

## Dependencies

Install via the Arduino Library Manager:

- Adafruit SH110X
- Adafruit GFX Library
- Adafruit BusIO
- AccelStepper (used by `StepScreenStepper`)

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
| `PIN_LED_GREEN` | 8 | Adalogger green LED (SD area) |
| `PIN_LED_RED` / `PIN_USER_LED1` | 13 | Adalogger red LED; same pin on this board |
| `PIN_USER_LED2` | A1 | user indicator LED |
| `PIN_EXT_LED` | A2 | external LED via N-MOSFET (220 Ω gate, 10 kΩ pulldown); HIGH = ON |
| `PIN_BEAM_DET` | A4 | photodetector; **external pull-up** — use `INPUT`, no internal pullup |
| `PIN_AUX_IN` | A5 | general-purpose digital input |
| `PIN_VBAT` | A7 (pin 9) | battery voltage divider (reads VBAT/2) |
| `PIN_I2C_SDA` / `PIN_I2C_SCL` | 20 / 21 | Wire |

Optional helpers: `#include <StepScreenIO.h>` for output registry and input sampling; `#include <StepScreenIODisplay.h>` to render I/O status on the OLED; `#include <StepScreenSD.h>` for microSD mount/read/write with card-detect hot-plug. See the **IOTest** example.

TMC2209 stepper (standalone step/dir, from `StepScreenBoard.h`):

| Symbol | Default pin | Notes |
|---|---|---|
| `PIN_MOTOR_STEP` | 5 | STEP pulse |
| `PIN_MOTOR_DIR` | 6 | direction |
| `PIN_MOTOR_EN` | 9 | ~EN, active LOW (LOW = enabled) |

MS1 and MS2 are tied HIGH on the module (1/16 microstepping). CLK is tied GND (internal clock). UART/DIAG are not used. `STEPSCREEN_MOTOR_STEPS_PER_REV` defaults to 3200 (200 full steps × 16 microsteps).

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

## TMC2209 motor driver

`StepScreenMotor` drives a BIGTREETECH TMC2209 in standalone step/dir mode (no UART). Include it independently of the display stack:

```cpp
#include <StepScreenMotor.h>

StepScreenMotor motor;

void setup() {
  motor.begin();
  motor.enable();
  motor.moveStepsSigned(320); // 1/10 rev forward at 1/16 microstepping
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

For application motion — acceleration, non-blocking moves, speed presets — use `StepScreenStepper`, a thin wrapper around [AccelStepper](https://www.airspayce.com/mikem/arduino/AccelStepper/) in DRIVER mode with the board's pins and ~EN polarity preconfigured. `StepScreenMotor` remains the raw pin-level option for hardware bring-up.

```cpp
#include <StepScreenStepper.h>

StepScreenStepper stepper;

void setup() {
  stepper.begin();          // driver starts disabled
  stepper.enable();
  stepper.moveSigned(3200); // one revolution at 1/16 microstepping
}

void loop() {
  stepper.run(); // non-blocking; call every loop() iteration
}
```

| Call | Purpose |
|---|---|
| `begin()` | Configure pins + defaults; driver starts disabled |
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
- **MotorTest** -- TMC2209 step/dir exercise without a motor connected. Blocking and non-blocking moves with Serial reporting; green LED blinks on each STEP pulse.
- **IOTest** -- auto-walks each board output solo while a single dashboard shows inputs, encoder, VBAT, and SD status. SD self-test runs automatically when a card mounts.
- **SyringePump** -- three-screen pump controller built on `StepScreenNav` + `StepScreenStepper`. Home (motor off), Adjust (encoder jogs the motor; OK cycles Low/Med/Fast speed), Run (send a signed step count over Serial at 115200, e.g. `3200` + newline). Demonstrates the menu debounce pattern.
- **Treadmill** -- rodent treadmill controller. Home shows speed (cm/s) and timeout (minutes) with the edited metric inverted (Back toggles Spd/Time, Sel toggles Fwd/Rev, encoder adjusts). Run ramps the belt to speed via `setSpeed()`/`runSpeed()`, counts down, then disables the motor and blinks DONE. Speed→steps mapping is a placeholder (`STEPSCREEN_TREADMILL_SPS_PER_CMS`).

## API summary

| Call | Purpose |
|---|---|
| `screen.begin()` | Initialize display + inputs (returns `false` if the OLED is missing) |
| `screen.display()` | The `StepScreenDisplay` (full GFX API + helpers below) |
| `screen.input()` | Buttons + encoder |
| `drawInfoBar(title, highlightBack)` | Inverted top bar with left-aligned title |
| `drawActionColumn(back, push, confirm)` | Right-justified Back/Sel/OK labels |
| `drawActionColumn(labels, back, push, confirm)` | Per-screen labels via `StepScreenActionLabels`; `nullptr` renders as `----` (unavailable). Labels longer than 4 chars are truncated |
| `clearContentArea()` | Clear only the content zone, cursor to its origin |
| `drawLayoutGuides()` | Zone borders + corner markers (for testing) |

Layout helpers draw at text size 1; set your own size afterwards for content.
