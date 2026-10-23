#!/usr/bin/env bash
set -euo pipefail

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VECTOR_WORKSPACE="${WORKSPACE_ROOT:-}"
if [[ -z "${VECTOR_WORKSPACE}" ]]; then
    candidate="${PACKAGE_DIR}"
    while [[ "${candidate}" != / ]]; do
        if [[ -f "${candidate}/install/local_setup.bash" ]]; then
            VECTOR_WORKSPACE="${candidate}"
            break
        fi
        candidate="$(dirname "${candidate}")"
    done
fi
if [[ -z "${VECTOR_WORKSPACE}" ]]; then
    echo "Build Revo-Retargeting first, or set WORKSPACE_ROOT to that workspace." >&2
    exit 2
fi
set +u
source /opt/ros/humble/setup.bash
source "${VECTOR_WORKSPACE}/install/local_setup.bash"
set -u
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
if [[ $# -eq 0 ]]; then
    set -- hardware
fi
exec ros2 run manus_revo3_vector_retarget retarget "$@"
