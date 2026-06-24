#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"
SDK_ROOT="${MANUS_SDK_DEST:-${WORKSPACE}/src/manus_ros2/ManusSDK}"
TMP_DIR="$(mktemp -d)"

cleanup() {
  rm -rf "${TMP_DIR}"
}
trap cleanup EXIT

is_lfs_pointer() {
  local file="$1"
  [[ -f "${file}" ]] && head -c 128 "${file}" | grep -q "version https://git-lfs.github.com/spec/v1"
}

has_real_manus_libs() {
  local lib_dir="${SDK_ROOT}/lib"
  local lib
  for lib in "${lib_dir}/libManusSDK.so" "${lib_dir}/libManusSDK_Integrated.so"; do
    if [[ -f "${lib}" ]] && ! is_lfs_pointer "${lib}"; then
      return 0
    fi
  done
  return 1
}

find_sdk_dir() {
  local root="$1"
  local candidate
  for candidate in \
    "${root}/ManusSDK" \
    "${root}/manus_sdk/ManusSDK" \
    "${root}/MANUS_SDK/ManusSDK" \
    "${root}"; do
    if [[ -d "${candidate}/lib" ]] && find "${candidate}/lib" -maxdepth 1 -type f -name 'libManusSDK*.so*' -print -quit | grep -q .; then
      printf '%s\n' "${candidate}"
      return 0
    fi
    if find "${candidate}" -maxdepth 1 -type f -name 'libManusSDK*.so*' -print -quit | grep -q .; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done

  candidate="$(find "${root}" -type f -path '*/include/ManusSDK.h' -print -quit | sed 's#/include/ManusSDK.h$##')"
  if [[ -n "${candidate}" && -d "${candidate}/lib" ]]; then
    printf '%s\n' "${candidate}"
    return 0
  fi

  return 1
}

extract_archive() {
  local archive="$1"
  local out_dir="$2"
  case "${archive}" in
    *.zip)
      unzip -q "${archive}" -d "${out_dir}"
      ;;
    *.tar|*.tar.gz|*.tgz|*.tar.xz|*.txz)
      tar -xf "${archive}" -C "${out_dir}"
      ;;
    *)
      echo "[manus_sdk] Unsupported archive type: ${archive}" >&2
      echo "[manus_sdk] Supported: .zip, .tar, .tar.gz, .tgz, .tar.xz, .txz" >&2
      return 2
      ;;
  esac
}

download_archive() {
  local url="$1"
  local out="$2"
  if command -v curl >/dev/null 2>&1; then
    curl -L --fail --retry 3 -o "${out}" "${url}"
  elif command -v wget >/dev/null 2>&1; then
    wget -O "${out}" "${url}"
  else
    echo "[manus_sdk] Missing curl or wget for MANUS_SDK_URL downloads." >&2
    return 2
  fi
}

install_from_dir() {
  local src="$1"
  mkdir -p "${SDK_ROOT}/include" "${SDK_ROOT}/lib"

  if [[ -d "${src}/include" ]]; then
    cp -a "${src}/include/." "${SDK_ROOT}/include/"
  fi

  if [[ -d "${src}/lib" ]]; then
    find "${src}/lib" -maxdepth 1 -type f -name 'libManusSDK*.so*' -exec cp -a {} "${SDK_ROOT}/lib/" \;
  else
    find "${src}" -maxdepth 1 -type f -name 'libManusSDK*.so*' -exec cp -a {} "${SDK_ROOT}/lib/" \;
  fi
}

if has_real_manus_libs; then
  echo "[manus_sdk] MANUS SDK libraries already installed under ${SDK_ROOT}/lib."
  exit 0
fi

SOURCE_DIR=""
if [[ -n "${MANUS_SDK_DIR:-}" ]]; then
  SOURCE_DIR="$(find_sdk_dir "${MANUS_SDK_DIR}" || true)"
elif [[ -n "${MANUS_SDK_ARCHIVE:-}" ]]; then
  extract_archive "${MANUS_SDK_ARCHIVE}" "${TMP_DIR}"
  SOURCE_DIR="$(find_sdk_dir "${TMP_DIR}" || true)"
elif [[ -n "${MANUS_SDK_URL:-}" ]]; then
  ARCHIVE_NAME="$(basename "${MANUS_SDK_URL%%\?*}")"
  if [[ "${ARCHIVE_NAME}" != *.* ]]; then
    ARCHIVE_NAME="manus-sdk-download.zip"
  fi
  ARCHIVE="${TMP_DIR}/${ARCHIVE_NAME}"
  download_archive "${MANUS_SDK_URL}" "${ARCHIVE}"
  extract_archive "${ARCHIVE}" "${TMP_DIR}/extracted"
  SOURCE_DIR="$(find_sdk_dir "${TMP_DIR}/extracted" || true)"
else
  cat >&2 <<EOF
[manus_sdk] MANUS SDK shared libraries are not included in this repository.

Download the official MANUS SDK from MANUS, then rerun one of:

  MANUS_SDK_ARCHIVE=/path/to/MANUS_SDK.zip ./scripts/install_manus_sdk.sh
  MANUS_SDK_DIR=/path/to/unpacked/ManusSDK ./scripts/install_manus_sdk.sh
  MANUS_SDK_URL=https://.../MANUS_SDK.zip ./scripts/install_manus_sdk.sh

Expected SDK layout after install:
  ${SDK_ROOT}/include/ManusSDK.h
  ${SDK_ROOT}/lib/libManusSDK.so
  ${SDK_ROOT}/lib/libManusSDK_Integrated.so
EOF
  exit 1
fi

if [[ -z "${SOURCE_DIR}" ]]; then
  echo "[manus_sdk] Could not find a ManusSDK directory in the provided source." >&2
  exit 1
fi

install_from_dir "${SOURCE_DIR}"

if ! has_real_manus_libs; then
  echo "[manus_sdk] Install finished, but no real libManusSDK*.so was found under ${SDK_ROOT}/lib." >&2
  exit 1
fi

echo "[manus_sdk] Installed MANUS SDK libraries to ${SDK_ROOT}/lib."
