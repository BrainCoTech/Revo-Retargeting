#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" == "revo3" || "${1:-}" == "3" ]]; then
  shift
fi

exec "$(dirname "$0")/start_revo3_driver.sh" "$@"
