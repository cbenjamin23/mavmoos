#!/bin/sh
set -eu
JOBS=${JOBS:-4}
mkdir -p /work/mavmoos /work/downloads /work/evidence
tar -xf /inputs/mavmoos.tar -C /work/mavmoos
if [ -f /inputs/mavmoos-git.tar ]; then
  mkdir -p /work/mavmoos/.git
  tar -xf /inputs/mavmoos-git.tar -C /work/mavmoos/.git
  git --git-dir=/work/mavmoos/.git config core.bare false
  git --git-dir=/work/mavmoos/.git config core.worktree /work/mavmoos
  git -C /work/mavmoos read-tree HEAD
fi
if [ -f /inputs/mavlink.tar.gz ]; then
  cp /inputs/mavlink.tar.gz /work/downloads/mavlink.tar.gz
fi
if [ ! -f /work/downloads/mavlink.tar.gz ]; then
  curl --fail --location --retry 2 \
    https://codeload.github.com/mavlink/c_library_v2/tar.gz/a2d0ecf011a4aaa7840480e43f4e38d8c9e79d29 \
    --output /work/downloads/mavlink.tar.gz
fi
printf '%s\n' '3fc41a3cd46717b1aa226b958aed0829fbe7b7c9793bc8ffdcca76345a9dfde3  /work/downloads/mavlink.tar.gz' | sha256sum --check
set --
if [ -x /work/ardupilot/build/sitl/bin/arducopter ]; then
  set -- -DMAVMOOS_ARDUPILOT_ROOT=/work/ardupilot -DMAVMOOS_SITL_PYTHON=/work/ap-venv/bin/python
fi
if [ -f /inputs/ci-config.json ] && [ -x /work/ardupilot/build/sitl/bin/ardurover ]; then
  set -- "$@" -DMAVMOOS_ROVER_ARDUPILOT_ROOT=/work/ardupilot
fi
cmake -G Ninja -S /work/mavmoos -B /work/mavmoos-build \
  -DMOOS_IVP_ROOT=/work/moos-ivp \
  -DMAVMOOS_BEHAVIOR_DIR=/work/altitude/build/lib \
  -DMAVMOOS_MAVLINK_ARCHIVE=/work/downloads/mavlink.tar.gz \
  -DMAVMOOS_EXTERNAL_OBSERVER_EVIDENCE_DIR=/work/evidence/external-observer-evidence \
  -DCMAKE_BUILD_TYPE=Debug "$@"
cmake --build /work/mavmoos-build -j "$JOBS"
if [ -f /inputs/ci-config.json ]; then
  python3 -B /work/mavmoos/scripts/ci/container.py --stage run
  cp /work/evidence/ci/ctest.log /work/evidence/ctest-linux.log
  cp /work/evidence/ci/plan/Testing/Temporary/LastTest.log /work/evidence/ctest-linux-detail.log
else
  ctest --test-dir /work/mavmoos-build --output-on-failure --output-log /work/evidence/ctest-linux.log
  cp /work/mavmoos-build/Testing/Temporary/LastTest.log /work/evidence/ctest-linux-detail.log
fi
python3 /inputs/check_sources.py --mavmoos > /work/evidence/final-source-check.json
python3 /inputs/record_elf.py /work/mavmoos-build/bin/iMAVLink \
  /work/mavmoos-build/plugins/mavmoos_*.so > /work/evidence/linux-app-artifacts.json
sha256sum /work/mavmoos-build/bin/iMAVLink /work/moos-ivp/bin/pHelmIvP \
  /work/altitude/build/lib/libBHV_ConstantAltitude.so /work/mavmoos-build/plugins/mavmoos_*.so > /work/evidence/binary-sha256.txt
