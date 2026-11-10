# SyringePump

Four-screen pump UI on Feather M0 Adalogger + StepScreen (OLED, encoder, TMC2209, optional VL53L4CD).

| Screen | Role |
|--------|------|
| **Home** | Motor off; size + ToF; OK → Run, Sel → Adjust, Back → Size |
| **Size** | Syringe mL picker (10/20/30/50/60) |
| **Adjust** | Jog motor; OK cycles Low/Med/Fast SPS |
| **Run** | USB Serial step commands; Bpod arm/trigger debug on SWD pads |

USB Serial (115200): send a signed microstep count + newline (e.g. `25600`) while on Run.

---

## Bpod RX debug (paused)

Work in progress — **paused**. Goal: treat the pump as a Bpod end-device: arm from a short UART frame on **SWDIO (PA31)**, fire on a rising edge on **SWCLK (PA30)**. No physical dispense yet.

### Wiring / pins

| Signal | MCU pad | Notes |
|--------|---------|--------|
| Bpod TX → device RX | **PA31 / SWDIO** (silkscreen DIO) | Not an Arduino `D31` pin — raw PORT bit 31 |
| Bpod trigger | **PA30 / SWCLK** | GPIO input, rising edge |
| Frame | `[0xA5, device_id, reward_amt]` | Arm if `device_id == BPOD_MY_ID` (default 1) |

Do not confuse with Feather variant **D30/D31** (`PB22`/`PB23`, SERCOM5 EDBG UART). Those are unrelated.

### What the logic analyzer showed

- Idle **HIGH**, 8N1.
- Bytes `0xA5`, `0x01`, `0x05` with **~2.5 ms** gaps (not a problem; assembler gap timeout is 10 ms).
- Character duration **82.464 µs** / 10 bits ≈ **8.25 µs/bit** (~121 kbaud), close to 115200.

Probe at the **Feather DIO/SWDIO pad** when debugging the MCU path; probing only at the Bpod/RJ45 side can look perfect while PA31 is quiet or different.

### What we tried

1. **Soft UART (bit-bang) on PA31** — decoded correctly in a blocking “sniff” window (`A5 01 05` → `arm id=1 amt=5`) using rise-sync when the start-bit rise landed near mid-bit0.
2. **SERCOM1 USART RX** (PA31 = SERCOM1/PAD3, mux C, `RXPO=3`, baud ~121200, TXEN off) — **never got `RXC`**. Diag showed `gpioFalls` / pad activity with `bytes=0`, `STATUS=0x0`. Config looked sane (`ENABLE=1`, `RXPO=3`, `PMUXO=2`).
3. **SERCOM left off** (`BPOD_USE_SERCOM 0`); soft RX wired into `bpodPoll()` with short blocking listens so Run mode could arm without a manual sniff prompt.

### Soft-UART failure modes we hit

| Symptom | Likely cause |
|---------|----------------|
| `gpioFalls=0` while LA sees UART | Probe/net not on PA31, or listen window missed the burst |
| `0xFD` / `0xF0` / `0xF8`, `us≈200` | `micros()` (~4 µs) used for bit timing; IRQ stretch |
| `A5→D2`, `01→00`, `05→82` | One bit **late** (waited for start rise + ½ bit on a slow edge) |
| `A5→4B`, `01→03`, `05→0B` | One bit **early** / left-shift (rise-immediate + short bit time in poll) |
| Fall + 1.5-bit mid-sample → no Serial / no locks | Current paused state — needs re-tune or edge IRQ |

Inter-byte spacing (~2.5 ms) is fine. TX going high-Z between bursts is fine if idle is pulled/driven HIGH; power-cycling the TX driver per packet glitches the line.

### Code knobs (as left)

```cpp
BPOD_MY_ID          // default 1
BPOD_UART_BAUD      // 121200 (LA-derived; for SERCOM path)
BPOD_UART_BIT_US    // 9 (soft sample period)
BPOD_USE_SERCOM     // 0 = soft RX (default), 1 = try SERCOM1 again
BPOD_RX_DEBUG       // 1 = USB diag / rx logs
BPOD_SOFT_SNIFF     // 0; set 1 for old blocking GPIO sniff helper
```

Arm timeout **2 s**; fire on SWCLK rising edge only while armed. Last arm wins.

### Open questions / next steps (when resumed)

1. Stabilize soft RX: edge-triggered start (EIC on PA31 / EXTINT[11]) so sampling is not late relative to OLED/`loop` latency; re-validate mid-bit timing against LA bit time (~8.25 µs).
2. Re-test SERCOM with soft path disabled and a known-good send while muxed — if still `RXC=0` with clean edges at the pad, treat SWDIO as soft-only or move RX to a header SERCOM pin.
3. Optional: checksum, real dispense on fire, drop debug noise.

---

## Dependencies

- StepScreen (+ Nav, Stepper, TMC2209, RTC, Splash)
- stm32duino **VL53L4CD** (optional; UI runs without ToF)
- Board: **Adafruit SAMD** Feather M0 (or compatible)
