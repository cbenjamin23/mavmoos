#!/bin/bash
#------------------------------------------------------------
#   Script: launch.sh
#  Mission: external_observer (one receive-only community)
#   Author: Charles Benjamin
#------------------------------------------------------------
#  Part 1: The foreground supervisor owns scoped exit handling.
#------------------------------------------------------------
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"

#------------------------------------------------------------
#  Part 2: Defaults, arguments, strict generation and one uMAC
#          session are handled without shell interpolation.
#------------------------------------------------------------
exec python3 -B "$HERE/supervise.py" "$@"
