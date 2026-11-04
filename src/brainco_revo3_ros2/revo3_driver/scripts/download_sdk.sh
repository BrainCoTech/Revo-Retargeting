#!/bin/bash
set -e

# BrainCo Revo3 SDK downloader for revo3_driver.
# Downloads the SDK binary package into this driver's vendor directory.
# Usage: bash scripts/download_sdk.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
VENDOR_DIR="${PKG_DIR}/vendor"
DIST_DIR="${VENDOR_DIR}/dist"
VERSION_FILE="${VENDOR_DIR}/VERSION"

SDK_VERSION="${BRAINCO_REVO3_SDK_VERSION:-v1.0.4}"
BASE_URL="${BRAINCO_REVO3_SDK_BASE_URL:-https://app.brainco.cn/universal/bc-revo3-sdk/libs/${SDK_VERSION}}"

echo_y() { echo -e "\033[1;33m$*\033[0m"; }
echo_g() { echo -e "\033[0;32m$*\033[0m"; }
echo_r() { echo -e "\033[0;31m$*\033[0m"; }

# Detect platform.
OS_TYPE="$(uname -s)"
ARCH="$(uname -m)"
echo_y "Detected: OS=${OS_TYPE}, ARCH=${ARCH}"

case "$OS_TYPE" in
  Linux)
    if [[ "$ARCH" == "aarch64" || "$ARCH" == "arm64" ]]; then
      PLATFORM="linux-arm64"
    else
      PLATFORM="linux"
    fi
    LIB_PATH="${DIST_DIR}/shared/linux/libbc_revo3_sdk.so"
    ;;
  Darwin)
    PLATFORM="mac"
    LIB_PATH="${DIST_DIR}/shared/mac/libbc_revo3_sdk.dylib"
    ;;
  MSYS_NT*|MINGW*)
    PLATFORM="win"
    LIB_PATH="${DIST_DIR}/shared/win/bc_revo3_sdk.dll"
    ;;
  *)
    echo_r "Unsupported OS: ${OS_TYPE}"
    exit 1
    ;;
esac

# Check if already installed and complete. The public release includes
# vendor/dist, so this script is only needed when refreshing the SDK.
if [ -f "$VERSION_FILE" ] &&
  grep -F --quiet "$SDK_VERSION" "$VERSION_FILE" &&
  [ -f "${DIST_DIR}/include/stark-sdk.h" ] &&
  [ -f "$LIB_PATH" ]; then
  echo_y "[revo3_driver SDK] ${SDK_VERSION} already installed."
  cat "$VERSION_FILE"
  exit 0
fi

if [ -f "$VERSION_FILE" ] && grep -F --quiet "$SDK_VERSION" "$VERSION_FILE"; then
  echo_y "[revo3_driver SDK] ${SDK_VERSION} marker found, but files are incomplete. Re-downloading."
fi

ZIP_NAME="bc_revo3_sdk_${SDK_VERSION}_${PLATFORM}.zip"
ZIP_PATH="${SCRIPT_DIR}/${ZIP_NAME}"
DOWNLOAD_URL="${BASE_URL}/${PLATFORM}.zip"

mkdir -p "$VENDOR_DIR"

echo_y "[revo3_driver SDK] Downloading ${SDK_VERSION} for ${PLATFORM} from: ${DOWNLOAD_URL}"
if command -v curl >/dev/null 2>&1; then
  curl -L -# -o "$ZIP_PATH" "$DOWNLOAD_URL" || {
    echo_r "Download failed. Check network or URL."
    exit 1
  }
elif command -v wget >/dev/null 2>&1; then
  wget -q --show-progress -O "$ZIP_PATH" "$DOWNLOAD_URL" || {
    echo_r "Download failed. Check network or URL."
    exit 1
  }
else
  echo_r "Error: Neither curl nor wget is installed. Please install one and try again."
  exit 1
fi

echo_y "[revo3_driver SDK] Extracting to vendor/ ..."
rm -rf "$DIST_DIR" "${VENDOR_DIR}/__MACOSX"
unzip -o -q "$ZIP_PATH" -d "$VENDOR_DIR" || {
  echo_r "Error: Failed to unzip ${ZIP_NAME}"
  exit 1
}
rm -f "$ZIP_PATH"
rm -rf "${VENDOR_DIR}/__MACOSX" "${DIST_DIR}/__MACOSX"

if [ ! -f "${DIST_DIR}/include/stark-sdk.h" ]; then
  echo_r "ERROR: stark-sdk.h not found after extraction. Check zip layout."
  find "$VENDOR_DIR" -maxdepth 3 -type f | head -30
  exit 1
fi

if [ ! -f "$LIB_PATH" ]; then
  echo_r "ERROR: SDK library not found at ${LIB_PATH}"
  find "${DIST_DIR}/shared" -maxdepth 3 -type f | head -30
  exit 1
fi

echo "${SDK_VERSION} ($(date '+%Y-%m-%d'))" > "$VERSION_FILE"
echo_g "[revo3_driver SDK] ${SDK_VERSION} installed successfully."
echo_g "  Header : ${DIST_DIR}/include/stark-sdk.h"
echo_g "  Library: ${LIB_PATH}"
