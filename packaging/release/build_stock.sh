#!/bin/sh
set -eu
cd /inputs
sha256sum -c <<'HASHES'
1c40fc102ce9625d7de7eb14f541cab30cc3138dca627f0b0ec40293ce6c2934  CPM_0.43.1.cmake
11958654f92314ba4fd72fb697d975d2a9c22cd7eb3545798019b099f8285d91  proj-5.2.0.tar.gz
HASHES
mkdir -p /work/cpm/cpm /work/proj-5.2.0
cp CPM_0.43.1.cmake /work/cpm/cpm
tar -xzf proj-5.2.0.tar.gz -C /work/proj-5.2.0 --strip-components=1
for file in ApplyPatch.cmake proj-5.2.0.patch; do
    tar -xOf moos-ivp.tar "MOOS_Jul2724/MOOSGeodesy/cmake/$file" > "/work/$file"
done
cmake -DPATCH_FILE=/work/proj-5.2.0.patch -DPATCH_WORKING_DIRECTORY=/work/proj-5.2.0 -P /work/ApplyPatch.cmake
export CPM_SOURCE_CACHE=/work/cpm CPM_proj_SOURCE=/work/proj-5.2.0
exec sh /inputs/build-stock.sh
