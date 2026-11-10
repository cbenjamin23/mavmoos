# Owned RC observer evidence

This directory adds a read-only MOOS subscriber, strict offline checker and owned native simulator wrappers. It does not add a production feature, upstream helm change, physical route, RC override or servo command.

Build only the subscriber without changing shared build wiring:

```sh
cmake -G Ninja -S tests/rc_io_native -B build-rc-native-probe-clt \
  -DMOOS_IVP_ROOT=~/moos-ivp \
  -DCMAKE_CXX_COMPILER=/Library/Developer/CommandLineTools/usr/bin/clang++ \
  -DCMAKE_OSX_SYSROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk
cmake --build build-rc-native-probe-clt
python3 tests/rc_io_native/test_rc_native_audit.py -v
```

Root build integration can instead define `rc_io_moos_probe` with this C++ source and link `${MOOS_LIBRARIES}`. The subscriber takes `DB_PORT DURATION_SECONDS CONTROL_DIRECTORY`, prints JSONL to stdout, reads `rc.phase`, and stops early if `observer.stop` exists. Phase markers and observations share its elapsed clock; no command is sent through MOOS.

The Copter wrapper requires `--ardupilot-root`, `--moos-root`, `--source-root` (actual build source), `--build`, a new `--output`, frozen `--expected-app-sha256` and `--expected-rc-sha256`; `--probe` can select the standalone subscriber. Use the pinned ArduPilot virtualenv Python. Native execution must first be allocated by the root integrator, serial under the common lock. Raw/decoded FC, MOOS, ownership, DataFlash, hashes and cleanup evidence go under the fresh output directory even on a failed run. Never reuse a previous output directory or a fixture connected to physical hardware.

`run_rc_rover.py` takes the same required flags; its optional separate subscriber is `--rc-probe`. It uses the unchanged built `rover_workflow_probe` from `--build`, renders the unchanged stock Rover mission from `--source-root`, and adds only the optional observer/configuration to the rendered app block. The existing200s wrapper covers GPS/EKF startup, native Guided/arm, stock2D helm motion, native Hold, explicit helm reacquisition, owned RC8 Manual takeover, explicit Guided/helm return, native RTL/Hold and disarm. A separate RC subscriber is stopped by a local marker after the workflow. Passive FC state, native DataFlash RC-reason1, neutral RC1/RC3 and switch values still qualify the existing takeover path; RC observations independently report the values without granting authority.
