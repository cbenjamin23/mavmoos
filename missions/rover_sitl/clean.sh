#!/bin/bash
# Clean generated mission artifacts only; no process killing.
set -euo pipefail
cd "$(dirname "$0")"
rm -f targ_rover.moos targ_rover.bhv .checkvars
rm -rf logs
