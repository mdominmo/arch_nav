#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
MANIFEST="${BUILD_DIR}/install_manifest.txt"

# Returns true if the path requires elevated permissions to write
path_needs_sudo() {
  local target="${1}"
  while [[ ! -d "${target}" ]]; do
    target="$(dirname "${target}")"
  done
  [[ ! -w "${target}" ]]
}

if [[ -f "${MANIFEST}" ]]; then
  echo "==> Removing installed files"
  FIRST_INSTALLED="$(head -1 "${MANIFEST}")"
  if path_needs_sudo "$(dirname "${FIRST_INSTALLED}")"; then
    sudo xargs -r rm -f < "${MANIFEST}"
    # Refresh linker cache if files were in a system-wide path
    if [[ "${FIRST_INSTALLED}" == /usr* || "${FIRST_INSTALLED}" == /lib* ]]; then
      sudo ldconfig
    fi
  else
    xargs -r rm -f < "${MANIFEST}"
  fi
else
  echo "==> No install manifest found — skipping uninstall"
fi

echo "==> Removing build directory"
rm -rf "${BUILD_DIR}"

echo "==> Done"
