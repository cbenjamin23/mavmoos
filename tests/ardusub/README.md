# ArduSub SITL qualification

The [provider documentation](../../docs/plugins/ardusub_control.md) describes the operator interface. The native workflow uses one vectored ArduSub simulator, one unmodified pHelmIvP, and a private MOOSDB. The independent operator and observation channels connect to exact loopback sockets of the simulator. The monitor sends zero packets. The runner holds the shared native SITL lease until scoped cleanup. It checks the selected binaries and dependencies before and after each run.

## Run

Use a built stock MOOS-IvP checkout and the same generated MAVLink dependency as the ordinary MAVMOOS build.

```sh
cmake -S . -B "$build" -DCMAKE_BUILD_TYPE=Debug -DMOOS_IVP_ROOT="$moos_ivp"
cmake --build "$build" --target iMAVLink mavmoos_telemetry mavmoos_ardusub_control ardusub_control_tests ardusub_moos_probe
ctest --test-dir "$build" --output-on-failure -R '^ardusub_control$'
```

The exact native runner uses macOS ARM64, Python 3.14.7, pymavlink 2.4.49, the checked-in mission templates, and the binaries identified in [the qualification receipt](qualification.json). The firmware came from upstream commit `92b0cd788ec29406f26c6f9c31d5ceedbd1cc538`, configured for the native `sitl` board and compiled with `waf sub` using Apple clang 21.0.0. This shared source pin produces **ArduSub V4.6.0-dev**. The MOOS-IvP dependency is `023fb54e00692611ecee2cd6f1138d10eec2127e`. The runner checks the published SHA-256 of the selected simulator, stock defaults and stock helm. For another host build, record and review its digests separately before qualifying it.

```sh
"$fixture_python" -B tests/ardusub/run_native.py \
  --ardupilot-root "$ardusub_source" \
  --moos-root "$moos_ivp" \
  --build "$build" \
  --app-revision "$(git rev-parse HEAD)" \
  --output "$new_receipt_directory"
```

For isolated boot, mode selection, ordinary arm and disarm checks, use `--preflight` and omit `--build` and `--app-revision`. Select a new output directory on each run. Review `result.json`, `ownership.jsonl`, `monitor-rx.mavlink`, `monitor-rx.jsonl`, `operator-tx.jsonl`, `moos.jsonl`, and `logs/00000001.BIN`. The receiver preserves the exact MAVLink datagrams. The receipt encodes absent nonfinite extension fields as explicit markers and hashes the raw captures. The tested fixture uses stock `BHV_ConstantHeading`, `BHV_ConstantSpeed`, and `BHV_ConstantDepth`.

## Repeat against stable ArduSub

The stable comparator pins official ArduSub 4.7.2 and the exact macOS ARM64 MAVMOOS app, two modules, probe, MOOS utilities and mission templates from the `4eb4e29` release campaign. Build [the upstream Sub-4.7.2 tag](https://github.com/ArduPilot/ardupilot/tree/Sub-4.7.2) at `fb773567896014da007bca58358704d4a0d8516f` with its recursive submodules, the native `sitl` board and `waf sub`. The runner verifies this build's simulator and stock defaults digests. [ArduPilot's versioned SITL metadata](https://firmware.ardupilot.org/Sub/stable-4.7.2/sitl/git-version.txt) identifies the same source.

```sh
"$fixture_python" -B tests/ardusub/run_native.py \
  --firmware-release stable-4.7.2 \
  --ardupilot-root "$stable_ardusub_source" \
  --moos-root "$moos_ivp" \
  --build "$baseline_build" \
  --app-revision 4eb4e295acdbfd7f119040da49e4847d6e8dc337 \
  --output "$new_receipt_directory"
```

The stable fixture sets `MAV_SYSID=1` and `MAV_GCS_SYSID=245` under the new names. It sets `ARMING_SKIPCHK=0` to preserve the baseline's all-checks arming policy. The accepted Guided position target is logged in metres in 4.7.2 and centimetres in the baseline. The readback normalizes that unit before applying the same capture threshold. Both variants use the same mission, operator steps, independent observations, failsafe settings and acceptance thresholds.

## Acceptance checks

- Read back the vectored frame, GCS failsafe settings and standard arming checks. Confirm fresh pressure, absolute position, and the surface home before ordinary arming in Position Hold.
- Before each explicit acquire, observe a fresh armed Position Hold heartbeat from the independent monitor. Wait for the bridge to report that mode and then report fresh Sub readiness. Verify that the navigation send count stays fixed while waiting. After the request, wait for an independent Guided heartbeat and check new course 90°, speed 0.7 m/s, and depth 2 m from stock pHelmIvP. Measure at least 3 m east, up to 2 m north or south, and 1.65–2.35 m below home from independent FC telemetry.
- Pause only the exact owned stock helm process. Require the grant to retire, the FC to accept exactly one Guided position target, and its onboard `GUIP` record to agree within 0.25 m with independent local position. Limit subsequent horizontal drift to 0.65 m and depth drift to 0.30 m. Restore the helm and confirm its resumed decisions cannot reuse the retired grant.
- Request native Position Hold and confirm fresh FC mode after a separate reacquisition. Reacquire again, pause the exact bridge process, and observe native Depth Hold on the FC's GCS watchdog. Restoring the bridge leaves the prior grant retired.
- Re-enter Position Hold and acquire explicitly. Command native Manual from the independent simulator operator and observe retirement with no further movement sends. Disarm normally, confirm clean process and port ownership, and verify input hashes.

The deterministic CTest covers the bounded normal no-goal handoff, expired/missing/wrong-source data, the post-Guided fresh-tuple requirement, pressure failure, handback, takeover, final send cancellation and shadow output. Native qualification covers a real stock-helm producer and an upstream software FC with synthetic horizontal navigation.

## Compound depth transect

Build `depth_autonomy_probe` and `mavmoos_plane_fault_tap` alongside the app and plugins. Use the stable fixture arguments with `--app-revision "$(git rev-parse HEAD)" --surface --stress` on the default vectored frame. Stock waypoint behaviors advance through three surveyed gates. Bridge-requested native Position Hold and pilot Manual interrupt the first handoff. The flight controller's watchdog selects native Depth Hold during a later pause of the owned bridge. Each interruption requires a separate acquire from Position Hold before the next leg. The final native Surface, shallow Depth Hold and disarm close the mission.

Replay a passing run with `"$fixture_python" -B tests/ardusub/stress_transect.py "$new_receipt_directory"`. The independent monitor verifies physical travel and depth. The private bridge capture verifies the exact mode commands and fences navigation until each new acquire. Stock `pMissionEval` writes `results.txt`.
