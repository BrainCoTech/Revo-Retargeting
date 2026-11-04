#!/usr/bin/env bash
# Scan serial ports for Revo3 hands (slave_id 126=left, 127=right) via Stark SDK.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DETECTOR_BIN="${SCRIPT_DIR}/bin/detect_revo3_ports"

find_detector_bin() {
  if [[ -n "${REVO3_DETECT_PORTS_BIN:-}" && -x "${REVO3_DETECT_PORTS_BIN}" ]]; then
    echo "${REVO3_DETECT_PORTS_BIN}"
    return 0
  fi

  if [[ -x "${DETECTOR_BIN}" ]]; then
    echo "${DETECTOR_BIN}"
    return 0
  fi

  return 1
}

ensure_detector() {
  if find_detector_bin >/dev/null; then
    return 0
  fi
  echo "[INFO] detect_revo3_ports not built; running setup/build_detect_revo3_ports.sh ..."
  bash "${SCRIPT_DIR}/build_detect_revo3_ports.sh"
}

ensure_detector
DETECTOR="$(find_detector_bin)"

vendor_lib="${PKG_ROOT}/vendor/dist/shared/linux"
if [[ ! -f "${vendor_lib}/libbc_revo3_sdk.so" ]]; then
  echo "[ERROR] Revo3 SDK not found: ${vendor_lib}/libbc_revo3_sdk.so" >&2
  echo "  Run: cd ${PKG_ROOT} && bash scripts/download_sdk.sh" >&2
  exit 1
fi

export LD_LIBRARY_PATH="${vendor_lib}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

exec timeout 30 "${DETECTOR}" "$@"
