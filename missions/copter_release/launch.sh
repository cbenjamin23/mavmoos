#!/bin/bash
#------------------------------------------------------------
#   Script: launch.sh
#  Mission: copter_release
#------------------------------------------------------------
#  Part 1: Local inputs
#------------------------------------------------------------
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SITL_PYTHON="${MAVMOOS_SITL_PYTHON:-python3}"
ARGS=()

#------------------------------------------------------------
#  Part 2: Operator interface
#------------------------------------------------------------
for_help() {
  cat <<'HELP'
Run the owned Copter 4.6.3 SITL operator demo.

launch.sh [--python PATH] --build DIR --moos-root DIR --ardupilot-root DIR
          --firmware-attestation FILE --behavior-dir DIR --output FRESH_DIR
          [--expected-app-revision REV]

Use a pymavlink venv for --python or MAVMOOS_SITL_PYTHON. The demo allocates
local ports, starts the pinned simulator and stock helm, and runs in the
foreground. It writes result.json and profile-result.json under FRESH_DIR.
HELP
}
while [ "$#" -gt 0 ]; do
  case "$1" in
    --help|-h) for_help; exit 0 ;;
    --python)
      if [ "$#" -lt 2 ] || [ -z "$2" ]; then
        printf 'Missing interpreter for --python\n' >&2
        exit 2
      fi
      SITL_PYTHON="$2"
      shift 2 ;;
    *) ARGS+=("$1"); shift ;;
  esac
done

#------------------------------------------------------------
#  Part 3: Foreground owned simulator and mission supervisor
#------------------------------------------------------------
exec "$SITL_PYTHON" -B "$ROOT/tests/run_sitl.py" --release-demo "${ARGS[@]}"
