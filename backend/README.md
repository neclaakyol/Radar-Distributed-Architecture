# Radar Backend

Portable C++20 backend for the HC-SR04/SG90 distributed radar system.

## Dependencies

### Required

| Dependency | Minimum version | Notes |
|---|---|---|
| C++ compiler | GCC 10 / Clang 14 | C++20 required (`std::filesystem`, concepts). Ubuntu 18.04 ships GCC 7 — see below. |
| CMake | 3.22 | Ubuntu 18.04 apt provides 3.10 — see below. |
| POSIX threads | — | Part of glibc; no extra package needed. |
| POSIX termios | — | Used for serial I/O; part of libc. No libserial dependency. |

### Optional (Vulkan renderer)

| Dependency | Package (Ubuntu/Debian) | Purpose |
|---|---|---|
| Vulkan loader + headers | `libvulkan1 libvulkan-dev` | Runtime loader and build headers |
| GLFW 3 | `libglfw3-dev` | Window and Vulkan surface creation |
| GLM | `libglm-dev` | Math types used by the GPU renderer |

When any of the three optional packages are absent, CMake sets `RADAR_HAS_VULKAN=0`
and the build falls back to the software renderer transparently.

## Setup on Jetson Nano (Ubuntu 18.04 / JetPack 4.x)

Run the provided script to install every dependency automatically:

```sh
bash backend/install_deps.sh
```

The script performs the following steps (all idempotent — safe to re-run):

1. Installs `build-essential`, `wget`, `gnupg`, and related base tools via apt.
2. Adds the **`ubuntu-toolchain-r/test` PPA** and installs **GCC 10** (the minimum
   compiler that supports C++20). The system default compiler is not changed;
   CMake is invoked explicitly with `-DCMAKE_CXX_COMPILER=g++-10`.
3. Adds the **Kitware apt repository** and installs **CMake 3.22+** (Ubuntu 18's
   default CMake 3.10 does not satisfy the project's `cmake_minimum_required(3.22)`).
4. Installs `libvulkan1 libvulkan-dev`, `libglfw3-dev`, and `libglm-dev` for the
   optional Vulkan renderer.

### Vulkan on Jetson Nano

The Jetson Nano's Maxwell GPU supports **Vulkan 1.1** starting with **JetPack 4.4**.
The Vulkan ICD (the actual GPU driver) is shipped as part of JetPack's NVIDIA
driver stack — it is already present if JetPack 4.4 or later is installed.
`install_deps.sh` adds only the Vulkan loader (`libvulkan1`) and C headers
(`libvulkan-dev`); no LunarG SDK is required and there is no x86-only SDK to install.

**Headless / no-display deployments:** the Vulkan renderer opens a GLFW window.
If no display is attached, `glfwCreateWindow` fails, the renderer marks itself
unavailable, and the backend silently uses the software framebuffer. Build with
`-DRADAR_ENABLE_VULKAN=ON` is still safe in this scenario.

## Build

### Headless Jetson Nano (recommended)

```sh
cmake -S backend -B backend/build \
      -DCMAKE_CXX_COMPILER=g++-10 \
      -DRADAR_ENABLE_VULKAN=OFF
cmake --build backend/build
ctest --test-dir backend/build --output-on-failure
```

### With Vulkan renderer (Jetson Nano + display, or desktop with Vulkan SDK)

```sh
cmake -S backend -B backend/build \
      -DCMAKE_CXX_COMPILER=g++-10 \
      -DRADAR_ENABLE_VULKAN=ON
cmake --build backend/build
```

### Desktop (Linux/Windows/macOS, system compiler, Vulkan off)

```sh
cmake -S backend -B backend/build -DRADAR_ENABLE_VULKAN=OFF
cmake --build backend/build
ctest --test-dir backend/build --output-on-failure
```

## Replay Test

```sh
backend/build/radar_make_replay backend/build/frames.bin 2
backend/build/radar_app --replay backend/build/frames.bin --log-dir backend/build/logs
```

Use `--no-render` to validate the parser, counters, sweeps, and MTI without PBM output:

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

The portable core does not require Vulkan. When Vulkan dependencies are present,
the optional renderer initializes a Vulkan-capable GLFW window while keeping the
software framebuffer readback path available for deterministic PBM export. The
checked-in `shaders/mti.comp` is the compute-shader contract for GPU candidate
generation; the current executable resolves MTI through the CPU reference path so
replay and tests remain deterministic on machines without a Vulkan toolchain.
