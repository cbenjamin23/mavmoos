# Native mission cancellation and recovery

These tests use the actual `missions` shared module through MOOS in a data-only
`iMAVLink` process. The owned native Copter remains disarmed; no helm, vehicle
control provider, arming, mode change or mission execution is requested.

The two cases deliberately stop a 64-row mission upload after the provider has
reported `SENT` for row 4 or later. `cancel` submits the public cancellation
request. `reset` pauses only the exact owned bridge process for 2.6 seconds to
cross its heartbeat continuity boundary. Both require a terminal `CANCELLED`
outcome preserving `mutation_may_have_applied=true`, reject a controlled repost
of the old request ID, and observe no further successful local sends for it.
The ordering check compares producer `MAVMOOS_MISSION_TX_RESULT.at` with the
terminal result's `at` in the same host clock. An earlier receipt delivered late
through MOOS is allowed; acceptance at or after the terminal time is rejected.
The probe's own elapsed clock is used separately for the settlement wait.

A new immediate download must report remote mission denial, demonstrating that
the FC can still be receiving after the bridge has retired the operation. After
10.5 seconds from retirement, a different request ID must download an exact
partial prefix of the submitted waypoints. HOME is retained as a raw dynamic
observation. A final explicit clear must complete with an empty verified readback
and no local mission work remaining. Cancellation does not promise rollback.

The fixed wait is derived from the pinned FC's 8-second receive timeout. It is
not an observed timeout acknowledgement: mission replies normally use the
primary link, while the independent monitor is passive on the second FC link.
The monitor archives raw MAVLink and decoded telemetry; MOOS records archive
provider requests, results, snapshots and host send receipts. These are distinct
evidence layers, not a packet capture of every primary-link datagram.

## Prepare without sockets

```sh
cmake -S tests/mission_faults -B build-mission-faults-private -G Ninja \
  -DMOOS_IVP_ROOT=/path/to/moos-ivp
cmake --build build-mission-faults-private
python3 -m unittest discover -s tests/mission_faults -p '*Tests.py'
```

`MissionFaultEvidenceTests.py` uses synthetic records solely to test rejection of
false or malformed evidence. It does not count as a native protocol test.

## Run an owned native fixture

After the integration build is frozen, use the Python environment containing
`pymavlink` and a new output directory for each attempt:

```sh
/path/to/ardupilot/.venv-ardupilot/bin/python \
  tests/mission_faults/run_mission_faults.py \
  --ardupilot-root /path/to/ardupilot --moos-root /path/to/moos-ivp \
  --source-root /path/to/mavmoos --build /path/to/mavmoos/build \
  --probe /path/to/build-mission-faults-private/mission_fault_probe \
  --scenario cancel --output /new/absolute/output/directory
```

Repeat with `--scenario reset` only after inspecting the first case. All native
harnesses share `/tmp/mavmoos-native-sitl.lock`; do not run them concurrently.
The runner verifies the native PID's exact loopback endpoints before launching
the bridge and never inserts a relay. It checks loaded module hashes, source and
binary stability, independent disarmed/ground state, DataFlash arming records,
and scoped process/port cleanup. Failed attempts remain evidence.

The 150-second native supervision bound and 125-second probe bound cover home
initialization, the explicit settlement wait, bounded download/clear and cleanup
at time warp 1. Waiting for HOME before clearing is fixture stabilization, not a
new mission-service prerequisite.

The provider request still has its original `published_at + timeout` deadline;
neither retries nor this test's settlement wait renew it. Recovery uses new IDs
and new explicit requests. This does not prove survival across a bridge process
restart, remote storage rollback, radio behavior or field readiness.
