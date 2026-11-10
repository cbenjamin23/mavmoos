#!/bin/bash
#------------------------------------------------------------
#   Script: launch_vehicle.sh
#  Mission: copter_operations (single community, stock helm)
#   Author: Charles Benjamin
#------------------------------------------------------------
#  Part 1: Convenience functions and scoped cleanup
#------------------------------------------------------------
set -euo pipefail
cd "$(dirname "$0")"
ANTLER_PID=""
on_exit() { if [ -n "$ANTLER_PID" ]; then kill "$ANTLER_PID" 2>/dev/null || true; wait "$ANTLER_PID" 2>/dev/null || true; fi; }
trap on_exit EXIT INT TERM

#------------------------------------------------------------
#  Part 2: Defaults; effectful simulation is explicitly selected
#------------------------------------------------------------
JUST_MAKE=no
AUTO_LAUNCHED=no
SIMULATION_ONLY=false
SHADOW_MODE=true
MOOS_PORT=19001
BIND_PORT=14561
PEER_PORT=14560
SITL_PID=0
PROJECT_ROOT="$(cd ../.. && pwd)"
APP_PATH="$PROJECT_ROOT/build-parallel/bin/iMAVLink"
PLUGIN_DIRECTORY="$PROJECT_ROOT/build-parallel/plugins"
BEHAVIOR_DIRECTORY="${MAVMOOS_BEHAVIOR_DIR:-$HOME/moos-ivp-uav-base/lib}"
LAT_ORIGIN=-35.363261
LONG_ORIGIN=149.165230
SPEED=1.5
HEIGHT=10
FACING_POLICY=course
FACING_HEADING=0

#------------------------------------------------------------
#  Part 3: Arguments
#------------------------------------------------------------
for arg in "$@"; do
  case "$arg" in
    --help|-h)
      echo "launch_vehicle.sh [--just_make] [--auto] [--simulation-only --sitl_pid=PID]"
      echo "  --mport=19001 --bind_port=14561 --peer_port=14560"
      echo "  --app=PATH --plugin_dir=DIR --behavior_dir=DIR"
      echo "  --lat_origin=DEG --long_origin=DEG --speed=1.5 --height=10"
      echo "Default: shadow mode, no effectful MAVLink transmission. Time warp fixed at 1."
      exit 0;;
    --just_make|-j) JUST_MAKE=yes;;
    --auto|-a|--nogui|-ng) AUTO_LAUNCHED=yes;;
    --simulation-only) SIMULATION_ONLY=true; SHADOW_MODE=false;;
    --mport=*) MOOS_PORT="${arg#*=}";;
    --bind_port=*) BIND_PORT="${arg#*=}";;
    --peer_port=*) PEER_PORT="${arg#*=}";;
    --sitl_pid=*) SITL_PID="${arg#*=}";;
    --app=*) APP_PATH="${arg#*=}";;
    --plugin_dir=*) PLUGIN_DIRECTORY="${arg#*=}";;
    --behavior_dir=*) BEHAVIOR_DIRECTORY="${arg#*=}";;
    --lat_origin=*) LAT_ORIGIN="${arg#*=}";;
    --long_origin=*) LONG_ORIGIN="${arg#*=}";;
    --speed=*) SPEED="${arg#*=}";;
    --height=*) HEIGHT="${arg#*=}";;
    --facing_policy=*) FACING_POLICY="${arg#*=}";;
    --facing_heading=*) FACING_HEADING="${arg#*=}";;
    1) :;;
    *) echo "Unknown argument: $arg" >&2; exit 1;;
  esac
done
for port in "$MOOS_PORT" "$BIND_PORT" "$PEER_PORT"; do
  case "$port" in ''|*[!0-9]*) echo "Invalid port: $port" >&2; exit 1;; esac
  if [ "$port" -lt 1 ] || [ "$port" -gt 65535 ]; then echo "Port out of range" >&2; exit 1; fi
done
if [ "$SIMULATION_ONLY" = true ] && ! [[ "$SITL_PID" =~ ^[1-9][0-9]*$ ]]; then
  echo "--simulation-only requires an owned --sitl_pid=PID" >&2; exit 1
fi

#------------------------------------------------------------
#  Part 4: Generate targets with strict substitution
#------------------------------------------------------------
nsplug meta_vehicle.moos targ_copter.moos --strict --force -x \
  "MOOS_PORT=$MOOS_PORT" "BIND_PORT=$BIND_PORT" "PEER_PORT=$PEER_PORT" \
  "APP_PATH=$APP_PATH" "PLUGIN_DIRECTORY=$PLUGIN_DIRECTORY" \
  "BEHAVIOR_DIRECTORY=$BEHAVIOR_DIRECTORY" "SIMULATION_ONLY=$SIMULATION_ONLY" \
  "SHADOW_MODE=$SHADOW_MODE" "SITL_PID=$SITL_PID" \
  "LAT_ORIGIN=$LAT_ORIGIN" "LONG_ORIGIN=$LONG_ORIGIN" "SPEED=$SPEED" \
  "FACING_POLICY=$FACING_POLICY" "FACING_HEADING=$FACING_HEADING"
nsplug meta_vehicle.bhv targ_copter.bhv --strict --force -x \
  "SPEED=$SPEED" "HEIGHT=$HEIGHT"
[ "$JUST_MAKE" = yes ] && exit 0

#------------------------------------------------------------
#  Part 5: Launch this owned community; wrapper owns uMAC
#------------------------------------------------------------
export IVP_BEHAVIOR_DIRS="$BEHAVIOR_DIRECTORY${IVP_BEHAVIOR_DIRS:+:$IVP_BEHAVIOR_DIRS}"
mkdir -p logs
pAntler targ_copter.moos &
ANTLER_PID=$!
if [ "$AUTO_LAUNCHED" = no ]; then uMAC targ_copter.moos; else wait "$ANTLER_PID"; fi
