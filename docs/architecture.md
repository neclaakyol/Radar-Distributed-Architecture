# System Architecture

A distributed ultrasonic radar: an **Arduino Uno** sweeps three servo-mounted
ultrasonic heads, encrypts each round of readings, and streams them over UART to
a **Jetson Nano** that decrypts, fuses, runs motion detection, and renders a live
top-down coverage map.

```
  Arduino Uno (FreeRTOS)                         Jetson Nano (C++20 backend)
 ┌───────────────────────────┐                 ┌────────────────────────────────────┐
 │ 3x servo + ultrasonic head │                 │ ingest thread          main thread  │
 │   sensor 0  HC-SR04        │                 │ ┌───────────┐   points ┌──────────┐ │
 │   sensor 1  HC-SR04        │  20-byte frame  │ │FrameParser│ ───────▶ │ per-point│ │
 │   sensor 2  HY-SRF05       │  XTEA + CRC16   │ │ sync→CRC  │  (queue) │ pipeline │ │
 │                            │ ══════════════▶ │ │ →decrypt  │          │          │ │
 │ TaskScanCoordinator        │   UART 115200   │ │ →3 points │          │ corrob.  │ │
 │ TaskSecurity (encrypt)     │  (USB or TTL)   │ └───────────┘          │ sweeps   │ │
 │ TaskUARTDispatch           │                 │                        │ MTI      │ │
 └───────────────────────────┘                 │                        │ render   │ │
                                                │                        └────┬─────┘ │
                                                │            ┌────────────────┴─────┐ │
                                                │            │ Vulkan live window    │ │
                                                │            │ + PBM file per cycle  │ │
                                                │            └───────────────────────┘ │
                                                └────────────────────────────────────┘
```

## 1. Firmware (Arduino Uno)

`Arduino/radar_firmware/radar_firmware.ino` — FreeRTOS, three tasks:

| Task | Role |
|---|---|
| `TaskScanCoordinator` | Round-robin reads sensor 0 → 1 → 2 at the current angle (short settle delay between triggers avoids acoustic crosstalk), batches one reading from each into a `RawData3`, then advances all three servos by 1°. |
| `TaskSecurity` | Packs the three readings into a 16-byte payload, encrypts it as two XTEA blocks, frames it with sync bytes + CRC-16. |
| `TaskUARTDispatch` | Writes the 20-byte frame to the hardware UART (`Serial`). |

- **Sensors:** 2× HC-SR04 + 1× HY-SRF05 (mode pin unconnected → trig/echo mode,
  same timing as HC-SR04, so one read routine drives all three).
- **Sweep:** all three heads sweep the same **0–120°** arc, reversing
  independently at their own bounds.
- **Cadence:** one frame is emitted **per degree-step** (~25 ms: 15 ms step +
  2×5 ms inter-sensor settle), continuously — not once per full sweep. Distance
  `0` is a valid **timeout** marker (echo timed out).

## 2. Wire protocol (20-byte frame)

Encoder/decoder: `backend/src/protocol.cpp` (`make_frame` / `decode_frame` /
`FrameParser`); mirror of the firmware's `TaskSecurity`.

```
Byte 0-1    sync 0xAA 0x55
Byte 2-17   XTEA-encrypted 16-byte payload (two 8-byte ECB blocks, shared 128-bit key)
Byte 18-19  CRC-16/CCITT over bytes 0-17
```

Plaintext payload (before encryption):

```
[0] sensor_id 0   [1] angle 0   [2-3] dist 0 (hi,lo)
[4] sensor_id 1   [5] angle 1   [6-7] dist 1 (hi,lo)
[8] sensor_id 2   [9] angle 2   [10-11] dist 2 (hi,lo)
[12] cycle counter  [13] status flags (bit0/1/2 = ch0/1/2 timeout)  [14-15] reserved
```

XTEA (32 rounds) and CRC-16/CCITT are byte-for-byte identical on both sides. The
shared key is hard-coded for the prototype (`kXteaKey`). ECB over two fixed
blocks is a known limitation (pattern leakage); fine for this demo, not for
production secrecy.

## 3. Transport (UART)

The firmware streams over the Uno's hardware UART, so the same frames flow over
either:

- **USB** — Arduino enumerates as `/dev/ttyACM0`.
- **Bare TTL UART** to the Jetson 40-pin header (pins 8/10 = `/dev/ttyTHS1`).
  The Jetson UART is 3.3 V and not 5 V tolerant, so the Arduino's 5 V TX is
  level-shifted down (resistor divider) and grounds are shared. See
  `docs/hardware_migration.md` for the wiring + `nvgetty` setup.

`backend/src/serial_source.cpp` (`SerialByteSource`) opens either at 115200 via
termios/Win32; `replay_source.cpp` (`ReplayByteSource`) replays a captured
`.bin` of frames for deterministic, hardware-free testing.

## 4. Backend pipeline

Entry point: `backend/src/main.cpp`. Two threads share state through a
mutex + condition variable (`SharedIngestState`):

**Ingest thread** (`start_ingest_thread`) reads bytes from the `ByteSource` and
feeds `FrameParser::ingest`, a small state machine: it resynchronizes on the
sync bytes, buffers 20 bytes, checks CRC, decrypts both XTEA blocks, and emits
**three `TelemetryPoint`s** (one per sensor, with running sequence numbers).
Counters track `valid_frames`, `crc_drops`, `sync_drops`, `timeout_readings`.

**Main thread** drains the points each loop tick (~16 ms) and, per point with a
non-zero distance and a valid `sensor_id`:

1. **Corroboration** (`corroboration.cpp` + `geometry.cpp`) — a cross-sensor
   "double check". In the wedge-boundary overlap bands it looks for a recent,
   geometrically consistent reading from the paired sensor (transformed into a
   shared world frame via `SensorGeometry` mounts) and tags the point
   `Confirmed` / `Rejected` / `Unchecked`.
2. **Sweep building** (`sweep.cpp`, one `SweepBuilder` per sensor) — accumulates
   points into directional sweeps, detects forward/return reversals, and emits a
   `CompletedSweepEvent` when a sensor finishes a bidirectional cycle. Routing
   per `sensor_id` keeps the three independently-paced heads from corrupting each
   other's direction detection.
3. **MTI motion detection** (`mti_cpu.cpp` via `GpuMti`) — on sweep completion,
   nearest-neighbor matches the new sweep against the opposite-direction sweep
   (within `--tau-mm`) to produce `MotionVector`s (object displacement/speed).
4. **Render** — on a completed bidirectional cycle, render the current state and
   write a PBM; every loop tick, present the current state to the live window.

## 5. Rendering & live display

`RenderState` (`render.hpp`) is the renderer's input: current points, per-sensor
sweep angles, MTI vectors, protocol stats, geometry.

- **`SoftwareRenderer`** (`software_renderer.cpp`) rasterizes `RenderState` into
  an RGBA framebuffer — the **multi-sensor coverage map**: dark field, a light
  "shield" border through the three sensors, **red** sensor markers (HC-SR04
  flanks + HY-SRF05 base, each labeled), a **white** live scan ray per sensor at
  its current angle, **green** dots for detected objects (corroborated ones
  ringed white), **green** MTI motion arrows, and a compact HUD. The three
  sensors' on-screen positions and 120° fan headings are a fixed *display
  layout* inside the renderer, independent of the backend's corroboration
  geometry.
- **`VulkanRenderer`** (`vulkan_renderer.cpp`) opens a GLFW window, creates a
  Vulkan surface + swapchain, and each loop tick **presents the RGBA frame live**
  by copying it into the acquired swapchain image (`vkCmdCopyBufferToImage`) and
  presenting — no graphics pipeline or shaders. It handles swapchain recreation
  on resize/out-of-date. Any setup failure (no display, no GPU, no Vulkan) leaves
  it unavailable and the backend silently uses the software path.
- **`write_pbm`** (`raster.cpp`) exports a 1-bit PBM per completed cycle to
  `--log-dir`, used for headless runs and deterministic inspection.

The checked-in `shaders/mti.comp` is a compute-shader *contract* for a future GPU
MTI path; the current executable resolves MTI on the CPU so replay and tests stay
deterministic.

## 6. Build & run

Build (Vulkan optional — auto-disabled if Vulkan/GLFW/GLM are absent):

```sh
cmake -S backend -B backend/build -DRADAR_ENABLE_VULKAN=ON   # or OFF for headless
cmake --build backend/build
ctest --test-dir backend/build --output-on-failure
```

Run modes:

```sh
# Live serial (Jetson UART) with the live Vulkan window — needs a display attached
backend/build/radar_app --serial /dev/ttyTHS1 --baud 115200

# Headless / no display — same data path, writes PBM frames to logs/
backend/build/radar_app --serial /dev/ttyTHS1 --baud 115200 --no-render

# Hardware-free replay (also opens the window if built with Vulkan + a display)
backend/build/radar_make_replay frames.bin 2
backend/build/radar_app --replay frames.bin --log-dir logs
```

**Live Vulkan window notes.** Build with `-DRADAR_ENABLE_VULKAN=ON`, run *without*
`--no-render`, from a terminal on the Jetson's own HDMI desktop (over SSH, first
`export DISPLAY=:0`). On startup the program prints the renderer status —
`Vulkan swapchain live present active` means the GPU window is up; any other
message means it fell back to PBM files. See `backend/README.md` for the build
matrix and `docs/hardware_migration.md` for wiring.

## 7. Source map

| Area | Files |
|---|---|
| Firmware | `Arduino/radar_firmware/radar_firmware.ino` |
| Protocol (frame/XTEA/CRC) | `backend/src/protocol.cpp`, `include/radar/protocol.hpp` |
| I/O sources | `backend/src/serial_source.cpp`, `replay_source.cpp`, `include/radar/io.hpp` |
| Orchestration / threading | `backend/src/main.cpp` |
| Geometry & corroboration | `backend/src/geometry.cpp`, `corroboration.cpp` |
| Sweep building | `backend/src/sweep.cpp` |
| MTI motion detection | `backend/src/mti_cpu.cpp`, `mti_gpu.cpp`, `shaders/mti.comp` |
| Rendering | `backend/src/software_renderer.cpp`, `vulkan_renderer.cpp`, `raster.cpp` |
| Tests / fixtures | `backend/tests/radar_tests.cpp`, `backend/tools/make_replay.cpp` |

See also: `backend/README.md` (build/deps), `docs/hardware_migration.md`
(wiring/UART), `docs/multi-sensor-plan.md` (project build plan).
