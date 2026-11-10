#!/bin/bash
#------------------------------------------------------------
#   Script: clean.sh
#  Mission: external_observer
#------------------------------------------------------------
# Generated files only. Running processes are owned by launch.sh.
set -euo pipefail
cd "$(dirname "$0")"
rm -f targ_external_observer.moos targ_external_observer.moosx
rm -rf logs
