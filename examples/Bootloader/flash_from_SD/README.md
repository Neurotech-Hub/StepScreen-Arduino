# Bootloader flash from SD

Host sketch for a Feather M0 Adalogger that writes a SAMD21 bootloader into a blank target over SWD. The image is `fw.bin` on the host's microSD card. After a good run, the target enumerates over its own USB port and can take Arduino uploads.

This is the sketch referenced by [Emulating the Adafruit Adalogger using the SAMD21](https://sites.wustl.edu/neurotechhub/emulating-the-adafruit-adalogger-using-the-samd21/).

It starts from [`examples/samd21/flash_from_SD`](https://github.com/adafruit/Adafruit_DAP/blob/master/examples/samd21/flash_from_SD/flash_from_SD.ino) in the Adafruit DAP library. The stock example programs the file and stops. This copy adds an SD clock fallback, a full flash read-back, and bootloader protection before it releases the target.

## Libraries

Install these from the Arduino Library Manager:

- [Adafruit DAP](https://github.com/adafruit/Adafruit_DAP) — SWD connection, chip erase, page programming, and the `protectBoot()` fuse write. The sketch includes `Adafruit_DAP.h`.
- [Adafruit ImageReader](https://github.com/adafruit/Adafruit_ImageReader) — this sketch reads the card with the same library ImageReader is built on, **SdFat – Adafruit Fork** (`SdFat_Adafruit_Fork.h`). ImageReader's M0 examples open the card at 12 MHz. This sketch uses that stack, tries the default speed first, then falls back to 12 MHz and 4 MHz.

Also install the **Adafruit SAMD** board package. In the Arduino IDE, select **Adafruit Feather M0 Adalogger**. If that name is not in the list, select **Adafruit Feather M0**.

## Hardware

The host is a spare Feather M0 Adalogger. Its USB port stays connected to the PC that is running the Serial Monitor. The target is a SAMD21 that does not yet have a USB bootloader, so the first program uses the SWD pads.

| Feather M0 Adalogger | Signal | SAMD21 target |
| --- | --- | --- |
| Pin 12 | SWDIO | SWDIO |
| Pin 11 | SWCLK | SWCLK |
| Pin 9 | RESET | RESET |
| GND | GND | GND |
| 3.3V | 3.3V | 3.3V |

Pin 4 is the host's onboard microSD chip select. Pin 13 is the host LED; it blinks only after the run finishes.

Use a current-limited 3.3 V supply on the target before the first attempt.

The host cannot see the target appear on USB. The target needs its own USB cable to the PC. When programming finishes, the sketch pulses the target reset and prints a reminder to look for that device.

## fw.bin

Copy [`../fw.bin`](../fw.bin) to the **root** of a FAT-formatted microSD card. The sketch opens the fixed name `fw.bin`. Leave the name as it is.

That file is the Feather M0 USB bootloader from [ArduinoCore-samd](https://github.com/adafruit/ArduinoCore-samd/tree/master/bootloaders/featherM0) `bootloaders/featherM0`, [`bootloader-feather_m0-v2.0.0-adafruit.5.bin`](https://github.com/adafruit/ArduinoCore-samd/blob/master/bootloaders/featherM0/bootloader-feather_m0-v2.0.0-adafruit.5.bin), renamed to `fw.bin`. The rename keeps the bootloader step to one filename: the sketch does not have to be edited when the Adafruit filename changes, and the card always carries the same 8 KB image.

The file is 8192 bytes. That is the SAMD21 boot region this sketch locks after verify. Chip erase wipes the whole target flash, including any bootloader already there. The target only shows up as a USB device if `fw.bin` is a USB bootloader image.

## What the sketch does

Open the Serial Monitor at **115200** baud before expecting any output. `while (!Serial)` waits until the monitor is open, then:

1. Mount the card. `SD.begin(SD_CS)` with no speed argument asks SdFat for **50 MHz**. On some boards, including the Feather M0 Adalogger, that becomes a clock the card will not answer. The sketch then calls `SD.end()` and retries at **12 MHz**, then **4 MHz**.
2. Open `fw.bin`.
3. Connect over SWD, print the debugger name, and identify the target (name, flash size, page count).
4. Chip-erase the target.
5. Program `fw.bin` one page at a time. The number printed after `Programming...` is the write time in milliseconds.
6. **Flash verification.** See checkpoints below.
7. **Bootloader protection.** See checkpoints below.
8. Drop the SWD connection and pulse RESET. The host LED blinks. Check the PC for the target's USB device.

`program_start()` switches SWD to its fastest clock for the write. Before the read-back, the sketch calls `dap_set_clock(50)` so a fast SWD clock is not a second source of mismatches.

## Checkpoints

The stock DAP example has neither of these. This sketch will not release the target until both succeed. A failure prints the reason and stops in `error()`; the LED does not blink.

### Flash verification

The file is rewound to the start. Each page is read back with `readBlock()` and compared to the bytes just written. Serial prints `Verifying... OK` when every page matches.

A mismatch prints `mismatch at 0x........` and `Verify failed`, and the sketch halts. The boot region is left unprotected and the target is not released.

### Bootloader protection

`program_start()` sets `BOOTPROT` to 7, which protects nothing, and leaves it there. After verify succeeds, `protectBoot()` writes `BOOTPROT` to `0x02`. That value protects the first 8 KB, the SAMD21 bootloader region, so later uploads do not overwrite it.

Serial prints `Protecting boot region... 8 KB`. If the fuse write fails, the sketch prints the DAP error and halts instead of releasing the target.

## Serial output

A clean run looks like this. If the card rejects 50 MHz, the fallback lines appear between `Initializing SD card...` and `Card initialized`.

```
Initializing SD card...
Card initialized
Connecting...
Found Target: SAM D21G18A    Flash size: 262144    Flash pages: 4096
Chip erase destroys all flash, including the bootloader.
The target enumerates over USB only if fw.bin contains a USB image.
Erasing...  done.
Programming... 850
Verifying... OK
Protecting boot region... 8 KB
Target released.
Check the PC for the target's USB device.

Done!
```

| Message | What to check |
| --- | --- |
| Nothing prints | Open the Serial Monitor. The host waits for that connection before it touches the card or the target. |
| `Default speed failed, retrying at 12 MHz` | Expected on some Adalogger boards. The sketch continues at 12 MHz. |
| `12 MHz init failed, retrying at 4 MHz` | The card did not answer at 12 MHz. The sketch continues at 4 MHz. |
| `Card failed, or not present` | Re-seat the card. It must be FAT, and all three clock attempts failed. |
| `Couldn't open file` | The card mounted, and `fw.bin` is missing from the root or named differently. |
| `Connecting...` then an error | SWDIO, SWCLK, RESET, and GND. Pin 12 is SWDIO, pin 11 is SWCLK, pin 9 is RESET. The target needs 3.3 V. |
| `Unknown device found` | SWD came up, and the chip ID is not a SAMD part this example knows. Confirm the target is a SAMD21. |
| `Verify failed` | The read-back did not match `fw.bin`. Do not treat the target as bootloaded. Check wiring and run again. |
| `Protecting boot region...` then an error | The image may be written, but the 8 KB region is not locked. Fix the reported DAP error and run again. |
