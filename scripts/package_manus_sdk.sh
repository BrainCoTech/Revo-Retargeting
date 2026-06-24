#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"
SDK_ROOT="${MANUS_SDK_SOURCE:-${WORKSPACE}/src/manus_ros2/ManusSDK}"
DIST_DIR="${MANUS_SDK_DIST:-${WORKSPACE}/dist}"
PACKAGE_NAME="${MANUS_SDK_PACKAGE_NAME:-manus-sdk-linux-x86_64}"
TMP_DIR="$(mktemp -d)"

cleanup() {
  rm -rf "${TMP_DIR}"
}
trap cleanup EXIT

is_lfs_pointer() {
  local file="$1"
  [[ -f "${file}" ]] && head -c 128 "${file}" | grep -q "version https://git-lfs.github.com/spec/v1"
}

require_real_file() {
  local path="$1"
  if [[ ! -f "${path}" ]]; then
    echo "[manus_sdk] Missing required file: ${path}" >&2
    exit 1
  fi
  if is_lfs_pointer "${path}"; then
    echo "[manus_sdk] File is a Git LFS pointer, not a real SDK file: ${path}" >&2
    exit 1
  fi
}

require_real_file "${SDK_ROOT}/include/ManusSDK.h"
require_real_file "${SDK_ROOT}/include/ManusSDKTypeInitializers.h"
require_real_file "${SDK_ROOT}/include/ManusSDKTypes.h"
require_real_file "${SDK_ROOT}/lib/libManusSDK.so"
require_real_file "${SDK_ROOT}/lib/libManusSDK_Integrated.so"

mkdir -p "${TMP_DIR}/ManusSDK/include" "${TMP_DIR}/ManusSDK/lib" "${DIST_DIR}"
cp -a "${SDK_ROOT}/include/ManusSDK.h" "${TMP_DIR}/ManusSDK/include/"
cp -a "${SDK_ROOT}/include/ManusSDKTypeInitializers.h" "${TMP_DIR}/ManusSDK/include/"
cp -a "${SDK_ROOT}/include/ManusSDKTypes.h" "${TMP_DIR}/ManusSDK/include/"
cp -a "${SDK_ROOT}/lib/libManusSDK.so" "${TMP_DIR}/ManusSDK/lib/"
cp -a "${SDK_ROOT}/lib/libManusSDK_Integrated.so" "${TMP_DIR}/ManusSDK/lib/"

cat >"${TMP_DIR}/README.txt" <<'EOF'
MANUS SDK package for Revo-Retargeting.

Install from the Revo-Retargeting workspace root:

  MANUS_SDK_ARCHIVE=/path/to/this/archive.tar.gz ./scripts/install_manus_sdk.sh

Expected destination:

  src/manus_ros2/ManusSDK/include/ManusSDK.h
  src/manus_ros2/ManusSDK/lib/libManusSDK.so
  src/manus_ros2/ManusSDK/lib/libManusSDK_Integrated.so

Then rebuild:

  source /opt/ros/humble/setup.bash
  python -m colcon build --symlink-install --packages-select manus_ros2
EOF

ARCHIVE="${DIST_DIR}/${PACKAGE_NAME}.tar.gz"
tar -C "${TMP_DIR}" -czf "${ARCHIVE}" ManusSDK README.txt
sha256sum "${ARCHIVE}" >"${ARCHIVE}.sha256"

echo "[manus_sdk] Created ${ARCHIVE}"
echo "[manus_sdk] Created ${ARCHIVE}.sha256"
