#!/bin/bash
# One local MOOS community and a caller-selected, already running endpoint.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
exec python3 -B "$here/mission.py" "$@"
