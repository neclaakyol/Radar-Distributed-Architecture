# MATLAB And Simulink Notes

This folder is a MATLAB-side test harness for the radar protocol. It does not replace the Arduino firmware or the C++ backend.

## What The Folders Mean

- `Arduino/`: active Arduino sketches and vendored Arduino libraries. The current firmware is `Arduino/sketch_may7d/sketch_may7d.ino`; it runs FreeRTOS tasks on the Arduino and emits encrypted 12-byte UART frames.
- `backend/`: active portable C++20 edge backend. It has protocol parsing, replay input, serial input, sweep buffering, CPU MTI, raster PBM export, and optional Vulkan window support. It is not only Vulkan.
- `FreeRTOS/`: legacy edge-node C++ experiments from earlier project stages. `main_v1.cpp` decodes serial frames, and `main_v2.cpp` adds early sweep buffering. The files named `radar_backend_v1` and `radar_backend_v2` appear to be compiled binaries. This folder is useful as historical reference, but the active backend is now `backend/`.

## What MATLAB Is Good For Here

Use MATLAB/Simulink to:

- Generate synthetic encrypted replay frames without compiling C++.
- Decode replay files created by either MATLAB or `backend/tools/make_replay.cpp`.
- Read live Arduino serial frames from `COM3`, `/dev/ttyACM0`, or similar.
- Plot a simple half-PPI radar view.
- Prototype signal-processing ideas before moving them to the C++ backend.

Avoid using Simulink deployment for the final Arduino firmware unless you intentionally want to replace `sketch_may7d.ino`. Deploying a Simulink model to the Arduino overwrites the existing sketch.

## Quick Start Without Hardware

From MATLAB, run:

```matlab
cd("C:\Users\voyi0\Projects\Radar-Distributed-Architecture")
addpath("matlab")

radar_demo_simulated
```

This creates `matlab/generated_frames.bin`, decodes it, and plots the synthetic sweep.

You can also do the steps manually:

```matlab
addpath("matlab")
radar_generate_replay("matlab/generated_frames.bin", 2)
points = radar_read_replay("matlab/generated_frames.bin")
radar_plot_replay("matlab/generated_frames.bin")
```

## Live Arduino Serial

Upload `Arduino/sketch_may7d/sketch_may7d.ino` to the Arduino first. Then connect the Arduino over USB and run one of:

```matlab
addpath("matlab")
radar_live_serial("COM3", 115200)
radar_live_serial("/dev/ttyACM0", 115200)
```

To run for a fixed duration:

```matlab
radar_live_serial("COM3", 115200, 60)
```

Only one program can own the serial port at a time. Close Arduino Serial Monitor, MATLAB serial sessions, and `radar_app` before opening the same port elsewhere.

## File List

- `radar_demo_simulated.m`: end-to-end no-hardware demo.
- `radar_generate_replay.m`: writes encrypted raw UART frames.
- `radar_read_replay.m`: decodes a replay file into a MATLAB table.
- `radar_plot_replay.m`: plots decoded replay points.
- `radar_live_serial.m`: reads and plots live serial frames.
- `radar_parse_bytes.m`: streaming frame parser.
- `radar_decode_frame.m`: validates sync/CRC and decrypts one 12-byte frame.
- `radar_make_frame.m`: creates one encrypted 12-byte frame.
- `radar_crc16_ccitt.m`: protocol CRC.
- `radar_xtea_encrypt_block.m`, `radar_xtea_decrypt_block.m`: protocol crypto helpers.

## Simulink Option

Simulink can model the sensor and servo path with Arduino support blocks, but it is best used here for experiments:

1. Use a Signal Generator or Counter block for angle `0..180..0`.
2. Use an Ultrasonic Sensor block or simulated distance source.
3. Send angle/distance to MATLAB for plotting, or compare it with this folder's replay format.

The project protocol uses binary encrypted frames. A pure Simulink block diagram for XTEA + CRC is possible, but it is more work than using the MATLAB helper functions here.

## Troubleshooting

- If MATLAB says the serial port is busy, close Arduino Serial Monitor and any running backend process.
- If decoded frames show many CRC drops, confirm the baud rate is `115200`.
- If live plots show no points but counters increase, the sensor may be timing out and emitting distance `0`.
- If plots look mirrored, confirm angles are interpreted as `0` degrees to the right and `180` degrees to the left.
