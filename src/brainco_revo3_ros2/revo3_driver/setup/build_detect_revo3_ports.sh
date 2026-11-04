#!/usr/bin/env bash
# Build setup-only tool detect_revo3_ports (independent of colcon / revo3_driver).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
BIN="${SCRIPT_DIR}/bin/detect_revo3_ports"

if [[ ! -f "${PKG_ROOT}/vendor/dist/include/stark-sdk.h" ]]; then
  echo "[ERROR] Stark SDK missing. Run first:" >&2
  echo "  cd ${PKG_ROOT} && bash scripts/download_sdk.sh" >&2
  exit 1
fi

mkdir -p "${BUILD_DIR}"
cmake -S "${SCRIPT_DIR}" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=Release
cmake --build "${BUILD_DIR}" --target detect_revo3_ports -j"$(nproc 2>/dev/null || echo 4)"

if [[ ! -x "${BIN}" ]]; then
  echo "[ERROR] expected binary at ${BIN}" >&2
  exit 1
fi

echo "[OK] ${BIN}"
