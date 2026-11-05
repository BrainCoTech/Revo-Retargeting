#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"
VENV="${ANYTELEOP_VENV:-${WORKSPACE}/.venv-anyteleop}"
PYTHON_BIN="${PYTHON:-python3.10}"

if [[ "${ROS_DISTRO:-humble}" != "humble" ]]; then
  echo "[anyteleop] This workspace requires ROS 2 Humble." >&2
  exit 1
fi
"${PYTHON_BIN}" -c 'import sys; assert sys.version_info[:2] == (3, 10), "Python 3.10 is required for ROS 2 Humble"'

if [[ ! -x "${VENV}/bin/python" ]]; then
  "${PYTHON_BIN}" -m venv --system-site-packages "${VENV}"
fi
if ! "${VENV}/bin/python" -m pip --version >/dev/null 2>&1; then
  "${VENV}/bin/python" -m ensurepip
fi
"${VENV}/bin/python" -c 'import sys; assert sys.version_info[:2] == (3, 10), "Recreate this venv with Python 3.10"'

"${VENV}/bin/python" -m pip install --upgrade pip
"${VENV}/bin/python" -m pip install \
  --extra-index-url https://download.pytorch.org/whl/cpu \
  -r "${WORKSPACE}/requirements-anyteleop.txt"

echo "[anyteleop] ready: ${VENV}/bin/python"
echo "[anyteleop] ./scripts/teleop.sh right retarget_method:=anyteleop anyteleop_python:=${VENV}/bin/python"
