# StepScreen

Arduino UI library for a custom board modeled after the [Adafruit Feather M0 Adalogger](https://learn.adafruit.com/adafruit-feather-m0-adalogger/pinouts), paired with a 1.3" SH1106 OLED + EC11 rotary encoder module (I2C display, three active-LOW buttons: back, confirm, and encoder push).

Built on the [Adafruit SH110X](https://github.com/adafruit/Adafruit_SH110X) driver, so the full Adafruit_GFX API is available alongside the StepScreen layout helpers.

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

## Pin map

Screen module defaults (from `StepScreenPins.h`):

| Symbol | Default pin | Module signal |
|---|---|---|
| `PIN_ENC_A` | 12 | SCREEN_TRA (encoder quadrature A) |
| `PIN_ENC_B` | 10 | SCREEN_TRB (encoder quadrature B) |
| `PIN_BTN_BACK` | 11 | SCREEN_BAK (back button) |
| `PIN_BTN_CONFIRM` | A0 | SCREEN_CONFIRM (confirm button) |
| `PIN_BTN_PUSH` | A3 | SCREEN_PUSH (encoder shaft push) |

Board baseline (from `StepScreenBoard.h`, following the Adalogger M0): `PIN_SD_CS` 4, `PIN_SD_CD` 7, `PIN_LED_GREEN` 8, `PIN_LED_RED` 13, `PIN_VBAT` A7, I2C on 20/21.

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

## Examples

- **ScreenTest** -- display-only validation. Steps through a full-screen border, corner markers, layout guides, and zone fills so you can confirm the entire 128x64 area is visible with no SH1106 offset clipping. Run this first on new hardware.
- **BasicUI** -- interactive demo: the encoder drives a counter, and each button highlights its action-column label. Includes the encoder ISR block.

## API summary

| Call | Purpose |
|---|---|
| `screen.begin()` | Initialize display + inputs (returns `false` if the OLED is missing) |
| `screen.display()` | The `StepScreenDisplay` (full GFX API + helpers below) |
| `screen.input()` | Buttons + encoder |
| `drawInfoBar(title, highlightBack)` | Inverted top bar with left-aligned title |
| `drawActionColumn(back, push, confirm)` | Right-justified Back/Sel/OK labels |
| `clearContentArea()` | Clear only the content zone, cursor to its origin |
| `drawLayoutGuides()` | Zone borders + corner markers (for testing) |

Layout helpers draw at text size 1; set your own size afterwards for content.
