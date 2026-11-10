# PX4 Ackermann qualification

## Direct firmware witness

This witness runs the locally retained PX4 v1.17.0 Linux ARM64 SIH Ackermann simulator in a network-disabled container. It uses stock firmware and the ordinary MAVLink routes. The test drives the autopilot protocol directly. It provides no MAVMOOS app or stock helm qualification. [Interface study](../../docs/research/2026-11-06-px4-rover-interface.md)

The host needs the retained firmware volume and a local generated MAVLink 2 common dialect from pymavlink. The runner mounts just the dialect module read-only and verifies its digest. This lets the pinned Linux tools image use the module without installing dependencies. It also verifies the exact simulator binary, Ackermann startup files and owned socket, and takes the common Linux native lease.

```sh
codec_path=$(/path/to/native-test-venv/bin/python -B -c 'from pymavlink.dialects.v20 import common; print(common.__file__)')
python3 -B tests/px4_rover/run_ackermann.py \
  --px4-package-volume mavmoos-stable-px4-4b2eac609afc45619159fc755701a8a3-package \
  --dialect "$codec_path" --case both --output build-px4-rover-evidence/new-witness
```

The default `mission` case transfers two waypoints, verifies a disarmed readback, and issues a separate native Mission start. The independent GCS link must observe Mission, motion, Hold, Mission after resume, final waypoint reached and disarm. The Hold dwell combines fresh GCS mode and position with advancing simulator ground truth. It preserves the navigation estimator's velocity report separately.

The `offboard` case arms in native Hold and tests a bounded local NED velocity stream with a dedicated observed mode entry and withdrawal. The `both` case runs the probe after the mission completes and gives each its own result. Read the `offboard_probe` verdict in the native receipt when both are selected. Direct MAVLink setpoints are scoped to that velocity mask and pinned Ackermann model.

Evidence includes directed sent and received MAVLink captures, independent GCS bytes, decoded events, native CLI, onboard log, firmware and fixture digests, and the container and native process cleanup receipts. Use a new output directory for every attempt.

## MAVMOOS app and stock helm

`qualify_ackermann.py` builds the real Linux ARM64 app, selected modules and independent MOOS probe from a clean revision. It uses the locally retained exact stock MOOS-IvP and PX4 volumes and the pinned offline MAVLink archive. Read the [Linux/PX4 setup](../px4_offboard/FIXED_WING.md) for the shared prerequisites. The runner starts an owned loopback-only container with scoped process, port and lease cleanup. The active fixture sets only the pinned test parameters on the simulator side.

```sh
python3 -B tests/px4_rover/qualify_ackermann.py --case full \
  --output build/px4-ackermann/full \
  --mavlink-archive /absolute/path/to/mavlink-a2d0ecf.tar.gz
python3 -B tests/px4_rover/qualify_ackermann.py --case restart \
  --output build/px4-ackermann/restart \
  --built build/px4-ackermann/full
python3 -B tests/px4_rover/qualify_ackermann.py --case abort \
  --output build/px4-ackermann/abort \
  --built build/px4-ackermann/full
python3 -B tests/px4_rover/qualify_ackermann.py --audit build/px4-ackermann/full
python3 -B tests/px4_rover/qualify_ackermann.py --audit build/px4-ackermann/restart
python3 -B tests/px4_rover/qualify_ackermann.py --audit build/px4-ackermann/abort
```

`full` verifies the plan readback, native Mission, paused Hold and stop, resume, final waypoint and explicit release. Stock helm then drives two different courses, exercises a short normal handoff, loses its stream into native Hold, and reacquires after explicit handback. `restart` interrupts only the owned bridge during a moving native Mission, verifies continued FC execution, and then checks the retained fence, stale session rejection and fresh handback. `abort` stops an actively moving native Mission, verifies released authority and rejects resume. The fixture verifies native Hold before final disarm. `--audit` checks archived receipts and hashes without starting PX4.

The three required software checks are `ctest.px4_rover_backend`, `ctest.px4_rover_mission` and `ctest.px4_rover_runtime`. The last loads the actual control and shared missions DSOs. See the qualification archive for exact app/firmware pins and the distinction between native and injected fault evidence.
