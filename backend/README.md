# Radar Backend

Portable C++20 backend for the HC-SR04/SG90 distributed radar system.

## Build

```sh
cmake -S backend -B backend/build -DRADAR_ENABLE_VULKAN=OFF
cmake --build backend/build
ctest --test-dir backend/build --output-on-failure
```

Enable Vulkan when the host has the Vulkan SDK, GLFW, and GLM:

```sh
cmake -S backend -B backend/build -DRADAR_ENABLE_VULKAN=ON
cmake --build backend/build
```

## Replay Test

```sh
backend/build/radar_make_replay backend/build/frames.bin 2
backend/build/radar_app --replay backend/build/frames.bin --log-dir backend/build/logs
```

Use `--no-render` to validate parser, counters, sweeps, and MTI without PBM output:

```sh
backend/build/radar_app --replay backend/build/frames.bin --no-render
```

## Live Serial

```sh
backend/build/radar_app --serial /dev/ttyACM0 --baud 115200
backend/build/radar_app --serial COM3 --baud 115200
```

Options:

- `--tau-mm 80`: nearest-neighbor MTI threshold.
- `--log-dir logs`: PBM output directory.
- `--threshold 128`: luminance threshold for 1-bit PBM conversion.
- `--no-render`: skip framebuffer generation and PBM export.

The portable core does not require Vulkan. When Vulkan dependencies are present, the optional renderer initializes a Vulkan-capable GLFW window while keeping the software framebuffer readback path available for deterministic PBM export. The checked-in `shaders/mti.comp` is the compute-shader contract for GPU candidate generation; the current executable still resolves MTI through the CPU reference path so replay and tests remain deterministic on machines without a Vulkan toolchain.
