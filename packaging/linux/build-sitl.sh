#!/bin/sh
set -eu
JOBS=${JOBS:-4}
mkdir -p /work/ardupilot /work/evidence
tar -xf /inputs/ardupilot.tar -C /work/ardupilot
# Git rejects host-owned .git metadata when the build runs as container root.
# This changes only the owned writable container copy.
chown -R "$(id -u):$(id -g)" /work/ardupilot
python3 -m venv /work/ap-venv
setuptools_pin=83.0.0
if [ -f /inputs/ci-config.json ]; then
  # Old stable DroneCAN generators still import pkg_resources.
  setuptools_pin=80.9.0
fi
/work/ap-venv/bin/pip install empy==3.3.4 fastcrc==0.3.5 future==1.0.0 \
  lxml==6.0.2 numpy==2.4.1 pexpect==4.9.0 ptyprocess==0.7.0 \
  pymavlink==2.4.49 "setuptools==$setuptools_pin"
/work/ap-venv/bin/pip freeze > /work/evidence/sitl-python-packages.txt
export PATH=/work/ap-venv/bin:$PATH
cd /work/ardupilot
if [ -f /inputs/ci-config.json ] && [ -f /inputs/firmware-manifest.json ]; then
  mkdir -p /work/mavmoos
  tar -xf /inputs/mavmoos.tar -C /work/mavmoos
  /work/ap-venv/bin/python -B /work/mavmoos/scripts/ci/container.py --stage build-native
else
  ./waf configure --board sitl --no-submodule-update
  ./waf copter -j "$JOBS"
fi
git rev-parse HEAD > /work/evidence/ardupilot-revision.txt
python3 /inputs/record_elf.py /work/ardupilot/build/sitl/bin/arducopter > /work/evidence/linux-sitl-artifact.json
