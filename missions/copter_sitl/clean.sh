#!/bin/bash
# Clean generated mission artifacts only; no process killing.
set -euo pipefail
cd "$(dirname "$0")"
rm -f targ_copter.moos targ_copter.bhv .checkvars
rm -rf logs
