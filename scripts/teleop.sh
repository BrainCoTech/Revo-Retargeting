#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" == "revo3" || "${1:-}" == "3" ]]; then
  shift
fi

exec "$(dirname "$0")/teleop_revo3.sh" "$@"
