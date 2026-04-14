#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
INSTALL_PREFIX="/usr/local"
BUILD_TESTS=OFF

print_usage() {
  echo "Usage: $(basename "$0") [OPTIONS]"
  echo ""
  echo "Options:"
  echo "  --prefix <path>   Installation prefix (default: /usr/local)"
  echo "  --tests           Build and run tests"
  echo "  -h, --help        Show this help"
  echo ""
  echo "Note: If --prefix is not specified, /usr/local is used and installation will typically require sudo."
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --prefix) INSTALL_PREFIX="$2"; shift 2 ;;
    --tests)  BUILD_TESTS=ON; shift ;;
    -h|--help) print_usage; exit 0 ;;
    *) echo "Error: unknown option '$1'"; print_usage; exit 1 ;;
  esac
done

if [[ "${INSTALL_PREFIX}" == "/usr/local" ]]; then
  echo "==> Note: no --prefix specified; using /usr/local usually requires sudo for installation."
fi

# Returns true if the install prefix requires elevated permissions
install_needs_sudo() {
  local target="${1}"
  while [[ ! -d "${target}" ]]; do
    target="$(dirname "${target}")"
  done
  [[ ! -w "${target}" ]]
}

echo "==> Configuring (prefix: ${INSTALL_PREFIX})"
cmake -B "${BUILD_DIR}" -S "${SCRIPT_DIR}" \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DBUILD_TESTING="${BUILD_TESTS}"

echo "==> Building"
cmake --build "${BUILD_DIR}" -j"$(nproc)"

if [[ "${BUILD_TESTS}" == "ON" ]]; then
  echo "==> Running tests"
  ctest --test-dir "${BUILD_DIR}" --output-on-failure
fi

echo "==> Installing to ${INSTALL_PREFIX}"
if install_needs_sudo "${INSTALL_PREFIX}"; then
  sudo cmake --install "${BUILD_DIR}"
else
  cmake --install "${BUILD_DIR}"
fi

# Refresh the dynamic linker cache only when installing to a system-wide path
if [[ "${INSTALL_PREFIX}" == /usr* || "${INSTALL_PREFIX}" == /lib* ]]; then
  sudo ldconfig
fi

echo "==> Done. arch_nav installed to ${INSTALL_PREFIX}"
