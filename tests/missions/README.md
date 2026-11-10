# Mission provider tests

These tests exercise the actual startup-loadable mission module. They do not modify pHelmIvP, and the mission interface exposes no execution operation.

- `MissionPluginTests.cpp`: 42 groups against fake host services, run by direct factory and real MODULE loading. Covers typed request/reply semantics, supported profile, exact send receipts, failed sends, original/phase deadlines, phase/global retry budgets, duplicate/reordered/incorrectly addressed messages, reset/replay protection, final-ACK loss, readback differences, dynamic HOME/current metadata, exact pinned LAND/RTL storage defaults with unrelated changes rejected, external arming and observation-only progress.
- `MissionRequestTests.py`: 11 standard-library request-generation tests; no MOOS/network activity.
- `MissionEvidenceTests.py`: 11 offline native-evidence-validator tests, including version-bound capability catalogs.
- `ConcurrentReadEvidenceTests.py`: ten stdlib groups for the final HOME-only mission/parameter read acceptance auditor, including bound-home, mutation-message and continuity negative controls. The pure validator is loaded from the actual concurrent wrapper without native imports.
- `run_private.py`: reproducible direct+MODULE compiler/test runner, optionally ASan/UBSan. Outputs commands, logs and source/artifact hashes to a new ignored build directory. Requires generated MAVLink C headers, not a vehicle.
- `MissionMoosProbe.cpp`: test-only MOOS client for the owned disarmed native bench. Effects go through `MAVMOOS_MISSION_REQUEST`; it opens no FC connection.
- `run_missions.py`: serialized owned-Copter bench wrapper. Requires the ArduPilot Python environment with pymavlink, the native Copter binary, MOOS, app/module/probe builds, and a fresh output directory. It owns its FC process and loopback endpoints, checks the observed plugin catalog, transfers only stored mission data, captures FC observations/DataFlash and checks scoped cleanup. It never arms or requests execution.

Private example:

```sh
python3 -B tests/missions/run_private.py --mavlink-dir /absolute/path/to/generated/mavlink --output build-missions-private/run-01
python3 -B tests/missions/run_private.py --mavlink-dir /absolute/path/to/generated/mavlink --output build-missions-private/run-02 --sanitize
python3 -B tests/missions/MissionRequestTests.py
python3 -B tests/missions/MissionEvidenceTests.py
```

Native bench, only while the integrator owns the serialized native test window:

```sh
/path/to/ardupilot/.venv-ardupilot/bin/python -B tests/missions/run_missions.py --ardupilot-root /path/to/ardupilot --moos-root /path/to/moos-ivp --build /path/to/mavmoos/build --output /new/owned/result-directory
```

The wrapper uses the project's existing `mavmoos-native-sitl.lock`, a fresh wiped FC state, no helm/control plugin, 1x time and bounded startup/phase/supervisor deadlines. The >30-second supervisor limit covers FC ground readiness plus upload, mandatory readback, independent download, clear/readback and final download with bounded retry margins. No two native tests should be run concurrently without separately qualified isolation.

Acceptance requires snapshots 0/3/3/0/0, two exact supported waypoint semantics, explicit upload/clear readback confirmation, rejected mission-start action, fresh disarmed/on-ground admission, no observed arming/NAV/helm, corroborating FC CMD log records, and clear owned processes/ports after teardown. The owned disarmed native bench passed with HOME and two WAYPOINT items; the implementation ledger preserves both its successful native evidence and the initial sandbox first-bind denial separately. The final corrected HOME-only mission/parameter read flight also passed with continuous stock-helm navigation and normal landing/disarm. Fake-Service fault tests, item-type normalization, nominal disarmed transfer and nominal combined-flight acceptance remain separate evidence layers.

## Flight, fence and rally inspection

The real module and host tests exercise wrong type and recipient replies, late replies, MAVLink 1 framing, empty and unsupported replies, bounded retry, typed cancellation, epoch reset, conflicting flight proof and same-ID read-to-write escalation. The `mission_inspection_loopback` CTest loads the actual app and MOOSDB with generated frames. It requires explicit read refusal in receive-only mode, zero captured egress, session invalidation during selected-peer heartbeat loss and old-ID rejection after recovery.

The `mission_native_inspection` CTest loads the actual app and MOOSDB with an attested native Copter on owned loopback. It observes empty flight, fence and rally plans first. A separate PID-verified fixture GCS then seeds two flight waypoints plus the HOME placeholder, a polygon and circle, and two rally points with different altitude frames. It accepts the pinned FC's addressed MAVLink 2 upload requests in either message form and supplies INT rows. MAVMOOS inspection uses the separate app route and reads back 3, 4 and 2 rows respectively. The audit matches snapshots to each exact operation, verifies typed list, indexed item and final ACK receipts, checks the stored coordinates and parameters, and independently confirms disarmed ground state and scoped cleanup.

Configure the native build with `MAVMOOS_ARDUPILOT_ROOT` and `MAVMOOS_SITL_PYTHON`. For a direct CTest invocation, export both `MAVMOOS_FIRMWARE_MANIFEST` and `MAVMOOS_FIRMWARE_ATTESTATION` for that build. Local CI injects the selected pair into each native case. Use a new evidence directory for each attempt and the common native lock.
