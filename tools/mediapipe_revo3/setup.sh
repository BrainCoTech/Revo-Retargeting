#!/usr/bin/env bash
set -euo pipefail
REPO=$(cd "$(dirname "$0")/../.." && pwd)
cd "$REPO"
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 PIP_REQUIRE_VIRTUALENV=true
export MPLCONFIGDIR="$REPO/artifacts/mediapipe_revo3/cache/matplotlib"
export PIP_CACHE_DIR="$REPO/artifacts/mediapipe_revo3/cache/pip"
mkdir -p "$MPLCONFIGDIR"
if [[ ! -x .venv/bin/python ]]; then
  PY=${PYTHON_BIN:-python3.11}
  "$PY" -c 'import sys; assert sys.version_info[:2] == (3,11), "Use Python 3.11"'
  "$PY" -m venv .venv
fi
.venv/bin/python -c 'import sys; assert sys.version_info[:2] == (3,11) and sys.prefix != sys.base_prefix, "Expected project Python 3.11 venv"'
.venv/bin/python -m pip install --disable-pip-version-check --no-cache-dir -r tools/mediapipe_revo3/requirements.lock
.venv/bin/python -m pip check
.venv/bin/python tools/mediapipe_revo3/assets.py
