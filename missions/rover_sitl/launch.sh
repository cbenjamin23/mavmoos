#!/bin/bash
#------------------------------------------------------------
#   Script: launch.sh
#  Mission: rover_sitl
#   Author: Charles Benjamin
#------------------------------------------------------------
#  Part 1: Scoped exit handling
#------------------------------------------------------------
set -euo pipefail
cd "$(dirname "$0")"
CHILD_PID=""
on_exit() { if [ -n "$CHILD_PID" ]; then kill "$CHILD_PID" 2>/dev/null || true; wait "$CHILD_PID" 2>/dev/null || true; fi; }
trap on_exit EXIT INT TERM

#------------------------------------------------------------
#  Part 2: Defaults
#------------------------------------------------------------
JUST_MAKE=no
XLAUNCHED=no
ARGS=()

#------------------------------------------------------------
#  Part 3: Arguments, forwarded to the one vehicle community
#------------------------------------------------------------
for arg in "$@"; do
  case "$arg" in
    --help|-h)
      echo "launch.sh [--just_make] [--nogui] [--xlaunched] [1]"
      echo "  --veh_mport=19021 --bind_port=14621 --peer_port=14620"
      echo "  --simulation-only --sitl_pid=PID --native_replay_dir=PATH (owned native ArduRover only)"
      echo "  --app=PATH --plugin_dir=DIR"
      echo "  --lat_origin=DEG --long_origin=DEG --speed=1.2"
      echo "  --waypoints='0,8:8,8:8,0:0,0'"
      echo "Default shadow mode. One local community; no shoreside or pShare ports."
      exit 0;;
    --just_make|-j) JUST_MAKE=yes; ARGS+=(--just_make);;
    --xlaunched|-x) XLAUNCHED=yes;;
    --nogui|-ng) :;;
    --veh_mport=*) ARGS+=("--mport=${arg#*=}");;
    *) ARGS+=("$arg");;
  esac
done

#------------------------------------------------------------
#  Part 4: Generate before launching; top-level owns uMAC
#------------------------------------------------------------
if [ "$JUST_MAKE" = yes ]; then exec ./launch_vehicle.sh --auto "${ARGS[@]}"; fi
./launch_vehicle.sh --auto "${ARGS[@]}" &
CHILD_PID=$!

#------------------------------------------------------------
#  Part 5: Foreground interactive or owned automation session
#------------------------------------------------------------
if [ "$XLAUNCHED" = yes ]; then wait "$CHILD_PID"; else uMAC targ_rover.moos; fi
