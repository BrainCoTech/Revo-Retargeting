#!/usr/bin/env bash
set -euo pipefail
REPO=$(cd "$(dirname "$0")/../.." && pwd)
cd "$REPO"
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
export MPLCONFIGDIR="$REPO/artifacts/mediapipe_revo3/cache/matplotlib"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
if [[ ! -x .venv/bin/python ]]; then
  echo 'Run: bash tools/mediapipe_revo3/setup.sh' >&2
  exit 1
fi
exec .venv/bin/python tools/mediapipe_revo3/app.py "$@"
