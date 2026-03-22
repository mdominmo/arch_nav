#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"

echo "Removing build artifacts..."
rm -rf "${SCRIPT_DIR}/build"

echo "Removing system-wide installation..."
sudo rm -rf /usr/local/lib/libarch_nav.so \
            /usr/local/lib/arch_nav \
            /usr/local/lib/cmake/arch_nav \
            /usr/local/include/arch_nav*.hpp \
            /usr/local/include/core \
            /usr/local/include/config \
            /usr/local/include/dispatchers \
            /usr/local/include/platform \
            /usr/local/include/utils \
            /usr/local/share/arch-nav \
            /usr/local/share/arch_nav_mavsdk_driver
sudo ldconfig

echo "Clean complete."
