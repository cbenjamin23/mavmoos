#!/bin/bash
#------------------------------------------------------------
#   Script: launch.sh
#  Mission: autonomy_sitl
#------------------------------------------------------------
#  Part 1: Local inputs
#------------------------------------------------------------
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SITL_PYTHON="${MAVMOOS_SITL_PYTHON:-python3}"

#------------------------------------------------------------
#  Part 2: Interpreter override
#------------------------------------------------------------
if [[ "${1:-}" == "--python" ]]; then
  if [[ $# -lt 2 ]]; then
    printf 'Missing interpreter for --python\n' >&2
    exit 2
  fi
  SITL_PYTHON="$2"
  shift 2
fi

#------------------------------------------------------------
#  Part 3: Foreground owned simulator and mission supervisor
#------------------------------------------------------------
exec "$SITL_PYTHON" -B "$ROOT/tests/autonomy_missions/run_native.py" "$@"
