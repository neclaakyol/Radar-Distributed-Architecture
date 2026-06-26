#!/usr/bin/env bash
# Install build dependencies for the radar backend on Ubuntu 18.04 (Bionic).
# Tested against Jetson Nano running JetPack 4.x.
# Requires a user with sudo privileges.

set -euo pipefail

CODENAME=$(lsb_release -cs 2>/dev/null || echo "bionic")

echo "=== Radar Backend: dependency installer (Ubuntu 18.04 / Jetson Nano) ==="
echo "    Codename: ${CODENAME}"
echo ""

# ─── Base build tools ────────────────────────────────────────────────────────
echo "--- Base build tools ---"
sudo apt-get update -qq
sudo apt-get install -y \
    build-essential \
    wget \
    ca-certificates \
    gnupg \
    lsb-release \
    software-properties-common

# ─── GCC 10  (Ubuntu 18 ships GCC 7; C++20 requires GCC 10+) ────────────────
echo ""
echo "--- GCC 10 (C++20 support) ---"
sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
sudo apt-get update -qq
sudo apt-get install -y gcc-10 g++-10

# Register as an alternative so cmake -DCMAKE_CXX_COMPILER=g++-10 always works.
# This does NOT change the system default compiler.
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 10 \
    --slave /usr/bin/g++ g++ /usr/bin/g++-10 \
    --slave /usr/bin/gcov gcov /usr/bin/gcov-10

echo "    $(g++-10 --version | head -1)"
echo "    To make GCC 10 the system default: sudo update-alternatives --config gcc"

# ─── CMake 3.22+  (Ubuntu 18 apt provides 3.10 — too old) ───────────────────
echo ""
echo "--- CMake 3.22+ (via Kitware apt) ---"
wget -qO- https://apt.kitware.com/keys/kitware-archive-latest.asc \
    | sudo gpg --dearmor -o /usr/share/keyrings/kitware-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] \
https://apt.kitware.com/ubuntu/ ${CODENAME} main" \
    | sudo tee /etc/apt/sources.list.d/kitware.list > /dev/null
sudo apt-get update -qq
sudo apt-get install -y cmake

echo "    $(cmake --version | head -1)"

# ─── Vulkan loader + headers  (optional renderer) ────────────────────────────
echo ""
echo "--- Vulkan loader + dev headers (optional renderer) ---"
echo "    On Jetson Nano the Vulkan ICD is provided by the JetPack NVIDIA GPU"
echo "    drivers that come pre-installed with JetPack 4.4+. This step installs"
echo "    only the Vulkan loader (libvulkan1) and C headers (libvulkan-dev)."
sudo apt-get install -y libvulkan1 libvulkan-dev

# ─── GLFW3 + GLM  (optional, required for the Vulkan renderer window) ────────
echo ""
echo "--- GLFW3 and GLM (Vulkan window dependencies) ---"
echo "    GLFW window creation will silently fail on headless Jetson deployments;"
echo "    the backend automatically falls back to the software renderer."
sudo apt-get install -y libglfw3-dev libglm-dev

# ─── Done ─────────────────────────────────────────────────────────────────────
echo ""
echo "=== All dependencies installed ==="
echo ""
echo "Build (software renderer only — recommended for headless Jetson Nano):"
echo ""
echo "    cmake -S backend -B backend/build \\"
echo "          -DCMAKE_CXX_COMPILER=g++-10 \\"
echo "          -DRADAR_ENABLE_VULKAN=OFF"
echo "    cmake --build backend/build"
echo ""
echo "Build with Vulkan renderer (requires JetPack 4.4+ GPU drivers and a display):"
echo ""
echo "    cmake -S backend -B backend/build \\"
echo "          -DCMAKE_CXX_COMPILER=g++-10 \\"
echo "          -DRADAR_ENABLE_VULKAN=ON"
echo "    cmake --build backend/build"
echo ""
echo "Run tests:"
echo "    ctest --test-dir backend/build --output-on-failure"
