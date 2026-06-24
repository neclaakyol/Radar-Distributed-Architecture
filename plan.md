# Arduino: 3-Motor / 3-Sensor Round-Robin Radar Scan

## Context

The project currently runs a single SG90 servo sweeping 0-180° with a single HC-SR04
ultrasonic sensor (`Arduino/sketch_may7d/sketch_may7d.ino`), sending 12-byte
XTEA-encrypted + CRC16-protected frames to a Jetson Nano backend that decrypts, builds
sweeps, runs MTI motion detection, and writes a PBM render per scan cycle.

The system is being upgraded to 3 independent servo+sensor heads mounted together:
- **Main head**: scans 0-120°.
- **Left/right heads**: mounted at the boundaries of the main head's 120° arc, each
  scanning 0-180° back toward the area the main head covers (flank coverage with
  overlap into the center).

Data acquisition is round-robin: read main → left → right, and once all three have
reported a reading at the current step, advance all three servos by 1° simultaneously
(each within its own range, reversing direction independently at its own bounds).

Confirmed hardware: 2x HC-SR04 + 1x HY-SRF05 (the HY-SRF05's mode-select pin is left
unconnected, which puts it in separate trig/echo pin mode — electrically and timing-wise
compatible with the existing HC-SR04 trigger/echo routine, so no new sensor driver is
needed).

Board stays Arduino Uno. This requires keeping the FreeRTOS task count low (2KB SRAM)
rather than naively tripling tasks per channel.

The frame format will be enlarged to batch all 3 readings from one round into a single
encrypted frame (user's explicit choice, accepting that a minimal backend parser update
is required now as a consequence — see Companion Backend Patch below). Per-sensor
zone-aware processing (sweep building, MTI, rendering) on the Jetson side stays out of
scope for this phase.

Out of scope for this phase (explicitly deferred): adaptive scan speed (dropped),
LED/buzzer proximity alert (later), EEPROM calibration offsets (after physical
mounting, when real misalignment can be measured). Documentation updates (README,
docs/, report.tex) wait until the Arduino and Jetson work is enhanced and stable.

## Firmware Changes — `Arduino/sketch_may7d/sketch_may7d.ino`

### 1. Per-channel data model

Replace the single `currentAngle` / `radarServo` globals with a 3-element channel
array:

```cpp
struct ScanChannel {
  uint8_t  id;           // 0 = main, 1 = left, 2 = right
  Servo    servo;
  uint8_t  servoPin;
  uint8_t  trigPin;
  uint8_t  echoPin;
  uint8_t  angle;
  uint8_t  minAngle;
  uint8_t  maxAngle;      // main: 120, left/right: 180
  bool     sweepingForward;
  uint16_t lastDistance;
  bool     lastTimeout;
};
ScanChannel channels[3];
```

Pin assignment (free pins after Serial D0/D1): 3 servo pins + 3 trig/echo pairs = 9
pins total, comfortably within D2-D13. Reuse D10 for main servo (no wiring change for
the existing unit) and pick two more free PWM-capable-not-required pins for the new
servos (Servo.h drives via Timer1 compare interrupts regardless of PWM hardware
support, so any digital pin works).

### 2. Task restructuring (RAM-conscious — 3 tasks total, not 6+)

Collapse `TaskServoActuation` + `TaskSensorPolling` into a single
**`TaskScanCoordinator`** that round-robins the 3 channels:

```
for each round:
  for ch in channels (id 0,1,2 order):
    trigger + read distance on ch (reuse existing read_*_distance_mm logic for all 3 —
      HC-SR04 and HY-SRF05 share the same trig/echo timing)
    record ch.lastDistance / ch.lastTimeout
    enforce a settle gap (e.g. vTaskDelay) before triggering the next channel,
      to avoid acoustic crosstalk between sensors firing close in time
  // all 3 channels have now reported for this step
  push a RawData3{channels} struct to rawDataQueue
  for each channel: advance angle by 1 in its current direction,
      reverse direction independently at its own minAngle/maxAngle
  for each channel: channel.servo.write(channel.angle)
```

Keep `TaskSecurity` (crypto/framing) and `TaskUARTDispatch` as separate tasks, unchanged
in role — just consuming the new larger struct/frame. Net: 3 FreeRTOS tasks instead of
4, despite 3x the sensing channels, keeping stack/heap usage flat.

No Timer1 conflict risk: this FreeRTOS port ticks off the Watchdog Timer
(`Arduino/libraries/FreeRTOS/src/FreeRTOSVariant.h:62`), and the AVR Servo library
supports up to 12 servos on the single available Timer1
(`Arduino/libraries/Servo/src/avr/ServoTimers.h:43`) — confirmed safe for 3 servos.

### 3. New frame format (12 bytes → 20 bytes)

```
Byte 0-1:    Sync bytes (0xAA 0x55)                      [unchanged]
Byte 2-17:   XTEA-encrypted 16-byte payload (2 blocks, ECB, same key)
Byte 18-19:  CRC-16 CCITT over bytes 0-17                 [range extended]
```

Plaintext payload (16 bytes, encrypted as two consecutive 8-byte XTEA blocks):

```
Byte 0:   sensor id 0 (0=main)      Byte 4: sensor id 1 (1=left)
Byte 1:   angle 0                   Byte 5: angle 1
Byte 2-3: distance 0 (hi,lo)        Byte 6-7: distance 1 (hi,lo)
Byte 8:   sensor id 2 (2=right)
Byte 9:   angle 2
Byte 10-11: distance 2 (hi,lo)
Byte 12:  cycle counter (rolling)
Byte 13:  status flags — bit0=main timeout, bit1=left timeout, bit2=right timeout
Byte 14-15: reserved (zero)
```

`xtea_encrypt`/CRC helpers stay the same algorithms — just called twice (once per
8-byte block) and CRC computed over the longer prefix.

### 4. Anti-crosstalk

Add an explicit minimum settle delay between consecutive sensor triggers in the
round-robin loop (in addition to the round-robin's inherent sequencing). Never trigger
two sensors back-to-back without this gap.

## Companion Backend Patch (minimal, protocol-level only)

`backend/include/radar/protocol.hpp` + `backend/src/protocol.cpp`:
- Update frame size constant 12 → 20, CRC range 10 → 18 bytes, decrypt 2 XTEA blocks
  instead of 1.
- `FrameParser` emits 3 `TelemetryPoint`s per frame (extend the struct with a
  `sensor_id` field), reusing the existing per-point pipeline — i.e. just feed all 3
  into the existing `SweepBuilder` as before. Status flags byte parsed into existing
  `ProtocolCounters` (timeout counts) rather than a new mechanism.
- Explicitly **not** changing: `sweep.cpp`, `mti_cpu.cpp`, `software_renderer.cpp` —
  zone-aware (main/left/right) sweep building and rendering stays deferred to the
  Jetson-focused phase. This patch only keeps the pipe from breaking.
- `backend/tools/make_replay.cpp` and `backend/tests/radar_tests.cpp` need their
  fixture frames regenerated/updated to the new 20-byte format so tests keep passing.

## Out of Scope This Phase

- Adaptive scan speed — dropped per discussion (complexity not justified yet).
- LED/buzzer proximity alert — deferred, cheap to add later.
- EEPROM calibration offsets — fast-follow once the 3 units are physically mounted and
  real misalignment can be measured.
- README/docs/report.tex updates — wait until Arduino + Jetson enhancements are done,
  per explicit sequencing request.

## Verification

1. Compile the updated sketch (Arduino IDE or `arduino-cli compile`) for Uno target —
   confirm it fits flash/SRAM (watch FreeRTOS heap config if task stacks change).
2. Bench-test with Serial Monitor / a small scratch script: verify each of the 3
   channels reports plausible distances and that angle bounds (120 / 180 / 180) and
   direction reversal behave independently per channel.
3. Update `backend/tools/make_replay.cpp` to emit the new 20-byte frame format, run
   `radar_app --replay <file> --no-render` to confirm `valid_frames` increments and
   `crc_drops`/`sync_drops` stay at 0.
4. Run `backend/tests/radar_tests.cpp` (CTest) after updating fixtures — protocol
   round-trip and sync-recovery tests must still pass against the new frame size.
5. Once backend parses cleanly, do a live serial run end-to-end (Arduino → Jetson/host)
   and confirm 3 distinct sensor ids show up in the decoded stream.