#!/bin/sh
set -eu
JOBS=${JOBS:-4}
if [ "${MAVMOOS_V1_RUNTIME:-0}" = 1 ]; then
  set -- -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON '-DCMAKE_INSTALL_RPATH=$ORIGIN/../lib'
else
  set --
fi
mkdir -p /work/evidence /work/moos-ivp /work/altitude
tar -xf /inputs/moos-ivp.tar -C /work/moos-ivp
tar -xf /inputs/altitude.tar -C /work/altitude
if [ -f /inputs/moos-git.tar ]; then
  mkdir -p /work/moos-ivp/.git
  tar -xf /inputs/moos-git.tar -C /work/moos-ivp/.git
  chown -R "$(id -u):$(id -g)" /work/moos-ivp/.git
  git --git-dir=/work/moos-ivp/.git config core.bare false
  git --git-dir=/work/moos-ivp/.git config core.worktree /work/moos-ivp
  git -C /work/moos-ivp read-tree HEAD
fi
cp /inputs/altitude-CMakeLists.txt /work/altitude/CMakeLists.txt
cmake -G Ninja -S /work/moos-ivp/MOOS/MOOSCore -B /work/moos-ivp/build/MOOS/MOOSCore \
  -DENABLE_EXPORT=ON -DUSE_ASYNC_COMMS=ON -DTIME_WARP_AGGLOMERATION_CONSTANT=0.4 \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_RUNTIME_OUTPUT_DIRECTORY=/work/moos-ivp/bin "$@"
cmake --build /work/moos-ivp/build/MOOS/MOOSCore -j "$JOBS"
cmake -G Ninja -S /work/moos-ivp/MOOS/MOOSEssentials -B /work/moos-ivp/build/MOOS/MOOSEssentials \
  -DMOOS_DIR=/work/moos-ivp/build/MOOS/MOOSCore -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_RUNTIME_OUTPUT_DIRECTORY=/work/moos-ivp/bin "$@"
cmake --build /work/moos-ivp/build/MOOS/MOOSEssentials -j "$JOBS"
cmake -G Ninja -S /work/moos-ivp/MOOS/MOOSGeodesy -B /work/moos-ivp/build/MOOS/MOOSGeodesy \
  -DMOOS_DIR=/work/moos-ivp/build/MOOS/MOOSCore \
  -DMOOSGEODESY_USE_PROJ=ON -DMOOSGEODESY_FETCH_PROJ=auto -DCMAKE_BUILD_TYPE=Release "$@"
cmake --build /work/moos-ivp/build/MOOS/MOOSGeodesy -j "$JOBS"
cmake -G Ninja -S /work/moos-ivp/ivp/src -B /work/moos-ivp/build/ivp \
  -DIVP_BUILD_GUI_CODE=OFF -DIVP_BUILD_BOT_CODE_ONLY=OFF -DFLTK_SKIP_FLUID=ON \
  -DIVP_LIB_DIRECTORY=/work/moos-ivp/lib -DIVP_BIN_DIRECTORY=/work/moos-ivp/bin \
  -DMOOS_DIR=/work/moos-ivp/build/MOOS/MOOSCore \
  -DMOOSGeodesy_DIR=/work/moos-ivp/build/MOOS/MOOSGeodesy \
  -DUSE_UTM=ON -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_BUILD_TYPE=Release "$@"
cmake --build /work/moos-ivp/build/ivp -j "$JOBS"
cmake -G Ninja -S /work/altitude -B /work/altitude/build -DMOOS_IVP_ROOT=/work/moos-ivp "$@"
cmake --build /work/altitude/build -j "$JOBS"
dpkg-query -W > /work/evidence/packages.tsv
uname -a > /work/evidence/uname.txt
cmake --version > /work/evidence/cmake-version.txt
c++ --version > /work/evidence/compiler-version.txt
python3 /inputs/record_elf.py /work/moos-ivp/bin/pHelmIvP \
  /work/altitude/build/lib/libBHV_ConstantAltitude.so > /work/evidence/linux-stock-artifacts.json
python3 /inputs/check_sources.py > /work/evidence/stock-source-check.json
