#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"

echo "Configuring..."
cmake -B "${SCRIPT_DIR}/build" -S "${SCRIPT_DIR}"

echo "Building..."
cmake --build "${SCRIPT_DIR}/build" -j"$(nproc)"

echo "Installing to /usr/local..."
sudo cmake --install "${SCRIPT_DIR}/build"
sudo ldconfig

echo "Build and install complete."
