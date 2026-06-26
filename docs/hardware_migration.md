# Hardware Migration Notes

## Final Hardware

- Controller: Arduino Uno
- Sensing: 3 servo + ultrasonic heads (round-robin scan)
  - 2x HC-SR04 ultrasonic distance sensors
  - 1x HY-SRF05 (mode-select pin left unconnected → trig/echo mode, identical
    timing to HC-SR04, so one read routine drives all three)
- Servos: 3x MG90S micro servos
- Edge node: Jetson Nano (or another Linux/Windows host) running the backend
- Display: HDMI display attached to the edge node (optional; backend also runs
  headless and writes PBM frames)

## Wiring

| Device            | Signal | Arduino pin |
| ----------------- | ------ | ----------- |
| Servo 0 (main)    | Signal | D10         |
| Servo 1 (left)    | Signal | D11         |
| Servo 2 (right)   | Signal | D12         |
| Sensor 0 (HC-SR04)| TRIG   | D2          |
| Sensor 0 (HC-SR04)| ECHO   | D3          |
| Sensor 1 (HC-SR04)| TRIG   | D4          |
| Sensor 1 (HC-SR04)| ECHO   | D5          |
| Sensor 2 (HY-SRF05)| TRIG  | D6          |
| Sensor 2 (HY-SRF05)| ECHO  | D7          |

Power notes:

- Drive the three MG90S servos from an **external regulated 5 V / 3 A supply**
  (breadboard rail), not the Arduino 5 V pin, and tie that supply's ground to the
  Arduino ground. Three servos moving together will brown out the Uno's onboard
  regulator.
- The Arduino Uno itself is powered from a 9 V battery (barrel jack), and the
  Jetson Nano from its own 5 V DC adapter.
- Sensors run on the 5 V breadboard rail with a shared ground.
- The HY-SRF05 mode pin is left unconnected (trig/echo mode).

## Firmware

The active sketch is `Arduino/radar_firmware/radar_firmware.ino` (Arduino Uno,
FreeRTOS). It runs three FreeRTOS tasks:

1. **Scan coordinator** — round-robin reads sensor 0 → 1 → 2 (with a short
   settle gap between triggers to avoid acoustic crosstalk), records each
   channel's angle/distance/timeout, then advances all three servos by 1°.
2. **Security/framing** — packs the three readings, encrypts, frames.
3. **UART dispatch** — writes the 20-byte frame over the hardware UART.

All three heads sweep the same **0–120°** arc (reversing independently at their
own bounds). Distance `0` is a valid **timeout** marker (echo timed out); the
backend counts it but excludes it from MTI matching.

HC-SR04 / HY-SRF05 trigger/echo timing:

1. Drive TRIG low for 2 µs.
2. Drive TRIG high for 10 µs.
3. Drive TRIG low.
4. Measure ECHO high duration with a 30000 µs timeout.
5. Convert echo duration to millimeters with `duration_us * 343 / 2000`.
6. Emit distance `0` when the echo times out.

## Protocol (20-byte frame)

Matches `backend/src/protocol.cpp` (`decode_frame` / `FrameParser`):

- Bytes `0..1`: sync bytes `0xAA 0x55`
- Bytes `2..17`: XTEA-encrypted 16-byte payload (two consecutive 8-byte XTEA
  blocks, ECB, same 128-bit key)
- Bytes `18..19`: CRC-16 CCITT over bytes `0..17`

Plaintext payload (16 bytes, before encryption):

| Byte    | Field                                                   |
| ------- | ------------------------------------------------------- |
| `0`     | sensor id 0 (main)                                      |
| `1`     | angle 0 (deg)                                           |
| `2..3`  | distance 0 (hi, lo)                                     |
| `4`     | sensor id 1 (left)                                      |
| `5`     | angle 1 (deg)                                           |
| `6..7`  | distance 1 (hi, lo)                                     |
| `8`     | sensor id 2 (right)                                     |
| `9`     | angle 2 (deg)                                           |
| `10..11`| distance 2 (hi, lo)                                     |
| `12`    | cycle counter (rolling)                                 |
| `13`    | status flags — bit0 = ch0 timeout, bit1 = ch1, bit2 = ch2 |
| `14..15`| reserved (zero)                                         |

## UART link to Jetson (vs USB)

The firmware streams over the Arduino Uno's **hardware UART** (`Serial`, pins
D0/D1), so it works either over USB (`/dev/ttyACM0`) or as a bare UART wired
directly to the Jetson's 40-pin header. No firmware change is needed for either.

Bare UART wiring (Arduino Uno → Jetson Nano):

- The firmware is **transmit-only**, so only one signal line is needed:
  **Arduino D1 (TX) → Jetson pin 10 (RXD, `/dev/ttyTHS1`)**, plus a common
  ground.
- The Jetson's UART pins are **3.3 V and not 5 V tolerant**, so the Arduino's
  5 V TX must be level-shifted down. A resistor divider works:
  **Arduino D1 → 500 Ω (e.g. 2×1 kΩ in parallel) → node → 1 kΩ → GND**, with the
  node going to the Jetson RX. That yields `5 V × 1000/(500+1000) = 3.33 V`.
  (Putting the resistors the other way around gives ~1.67 V — too low.)
- **Common ground is mandatory:** Arduino GND ↔ a Jetson header GND (e.g. pin 6).
- The Uno's D0/D1 UART is shared with the USB programmer — **disconnect D0/D1
  while uploading sketches**, then reconnect to run. There is no DTR auto-reset
  over a bare UART (harmless; the firmware streams continuously).

Jetson Nano setup:

- The 40-pin header UART (pins 8 = TXD, 10 = RXD) is `/dev/ttyTHS1`.
- If the port is held by a serial-console getty: `sudo fuser /dev/ttyTHS1`; if
  busy, `sudo systemctl disable --now nvgetty` and reboot.
- Grant access: `sudo usermod -aG dialout $USER` (re-login) or run with `sudo`.
- Electrical smoke test (independent of the backend):
  ```sh
  stty -F /dev/ttyTHS1 115200 raw -echo
  timeout 2 xxd /dev/ttyTHS1 | head      # expect repeating "aa 55 ..." frames
  ```
- Run the backend against the UART device:
  ```sh
  backend/build/radar_app --serial /dev/ttyTHS1 --baud 115200 --no-render
  ```
  Expect `valid_frames` climbing with `crc_drops`/`sync_drops` ≈ 0. Many CRC
  drops point to a noise / ground / baud problem.
- **Live display:** with an HDMI display attached and the backend built with
  `-DRADAR_ENABLE_VULKAN=ON`, drop `--no-render` to get the live Vulkan "Radar
  PPI" window (the sweep animates as each degree-step frame arrives). Headless
  builds/runs fall back to per-cycle PBM files automatically.

## Calibration

The default sweep is `0..120` degrees with `15 ms` per degree, per channel. If
the physical assembly binds near an endpoint, change that channel's `minAngle` /
`maxAngle` in `radar_firmware.ino` to a calibrated range and record it here.

Recommended hardware checks:

- Confirm each sensor reads correctly at 100 mm, 200 mm, and 400 mm.
- Confirm timeout readings produce distance `0`.
- Confirm the three servos sweep smoothly and do not brown out the Arduino.
- Confirm the backend receives valid encrypted frames at 115200 baud
  (`valid_frames` climbing, `crc_drops`/`sync_drops` ≈ 0), with three distinct
  sensor ids in the decoded stream.
