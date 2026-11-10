#!/bin/sh
set -eu
test "$(uname -m)" = aarch64
mkdir -p /work/mavmoos /work/evidence /work/downloads
tar -xf /inputs/mavmoos.tar -C /work/mavmoos
printf '%s\n' '3fc41a3cd46717b1aa226b958aed0829fbe7b7c9793bc8ffdcca76345a9dfde3  /inputs/mavlink.tar.gz' | sha256sum -c
cp /inputs/mavlink.tar.gz /work/downloads/mavlink.tar.gz
cmake -G Ninja -S /work/mavmoos -B /work/mavmoos-build \
    -DCMAKE_BUILD_TYPE=Release -DMOOS_IVP_ROOT=/work/moos-ivp \
    -DMAVMOOS_MAVLINK_ARCHIVE=/work/downloads/mavlink.tar.gz \
    -DMAVMOOS_BEHAVIOR_DIR=/work/altitude/build/lib \
    -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON '-DCMAKE_INSTALL_RPATH=$ORIGIN/../lib'
cmake --build /work/mavmoos-build --parallel "$JOBS" --target \
    iMAVLink observer_matrix_probe observer_matrix_frames px4_offboard_moos_probe
mkdir -p /work/mavmoos-build/lib /work/mavmoos-build/licenses
cp /work/moos-ivp/build/MOOS/MOOSGeodesy/lib/libMOOSGeodesy.so /work/mavmoos-build/lib/
cp /work/moos-ivp/COPYING.md /work/mavmoos-build/licenses/MOOS-IvP-COPYING.md
cp /work/proj-5.2.0/COPYING /work/mavmoos-build/licenses/PROJ-COPYING
python3 -B /inputs/check_sources.py --mavmoos > /work/evidence/final-source-check.json
revision=$1
shift
if [ "$#" -gt 0 ]; then
    cp /inputs/UAV-BASE-LICENSE /work/mavmoos-build/licenses/
fi
python3 -B /work/mavmoos/packaging/release/kit.py pack "$revision" /work/mavmoos-build /work/evidence "$@"
python3 -B /inputs/record_elf.py /work/moos-ivp/bin/pHelmIvP /work/moos-ivp/bin/pAntler \
    /work/moos-ivp/bin/MOOSDB /work/moos-ivp/bin/pLogger /work/moos-ivp/bin/nsplug \
    /work/altitude/build/lib/libBHV_ConstantAltitude.so \
    /work/mavmoos-build/bin/px4_offboard_moos_probe \
    /work/mavmoos-build/bin/observer_matrix_probe /work/mavmoos-build/bin/observer_matrix_frames \
    > /work/evidence/build-witness-artifacts.json
