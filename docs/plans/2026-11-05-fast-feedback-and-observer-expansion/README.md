# Fast feedback and parallel observer expansion

November 5, 2026. The focused software lane and tier guards are implemented. The observer packages and native work below are the next proposed batch. The baseline is `6f53589`, with exact native evidence at `119f723`. Use the local repository, stock `pHelmIvP`, receive-only defaults and owned loopback simulation.

## What is growing

| Archived measurement | CTest time | Scope |
| --- | ---: | --- |
| Normal software | 3m 41s | 103 cases with four jobs |
| ASan/UBSan software | 3m 44s | 103 cases with four jobs |
| Full pinned macOS ARM64 | 44m 54s | 133 cases, including 29 Copter 4.6.3 and one separately pinned Rover |
| Native portion of full | 40m 36s | 30 serial cases, 90.4% of total time |

These numbers come from the CTest logs and JUnit rows in the software and stable full archives. The ten slowest native cases account for 20m 28s. The longest is the repeated wind cycle at 177s. The nine software external-observer app cases also account for 141s of aggregate execution and run serially. Measure compile, CTest and setup time separately before optimizing the smaller costs.

## Which gate to run

| Gate | Trigger | Test scope |
| --- | --- | --- |
| Focused | During a feature edit or review | Explicit registered software IDs on a warm private build. Aim for feedback within 60s including a focused app witness. |
| Software | Before integrating the feature commits | All configured software cases, with the required software inventory intact. Run normal and sanitizer gates on the integrated freeze. Aim for 4m 30s per warm gate. |
| Smoke | When validating the integrated simulator path after shared control, host or native fixture changes | Software plus the single existing Copter lifecycle case. About 5m 09s estimated from separate archived timings. |
| Full | Integrated qualification checkpoint or promotion of an exact firmware/platform cell | Entire registry tier, including all 30 existing native cases. Review projected additions or repeated timings that take this past 50 minutes on the same host. |

The 60s, 4m 30s and 50m thresholds are development budgets for review. Individual test deadlines and flight evidence limits retain their existing values. Each added native case must name the firmware behavior it uniquely proves, record its wall time and explain why a fixture or an existing scenario cannot prove it. In this batch, one new disarmed Copter witness can prove delivery for three observers in one simulator boot. Check all other message variants and faults through generated-wire fixtures and the real app on loopback. Review whether the shared witness belongs in full after measuring it, with a provisional 90s wall target. Keep smoke fixed at one native lifecycle case.

The [local runner](../../../scripts/ci/run.py) builds normally, validates the configured CTest graph against the registry and creates a private plan. Focused results identify the exact requested, executed and excluded IDs. Unknown, duplicate, absent and native case requests fail closed. Scope and `sitl` labels must agree, including on configured cases outside the selection. Required software rows stay in all qualification tiers. Smoke fails if its Copter lifecycle is omitted or replaced. Full retains the baseline native identities and original evidence audits. This avoids a false pass by editing a tier or using a loose name filter.

```sh
python3 -B scripts/ci/run.py --build /path/to/private-build --tier focused \
  --case ctest.imu_provider_observations --case ctest.external_observer_moos_normal \
  --output /path/to/fresh-feedback --jobs 2
```

Focused runs still build the configured targets. Reuse each worker's own incremental build and the pinned dependencies as read-only inputs. Separate worktrees, build directories, outputs and generated missions. Begin with two compile jobs per worker and parallel fake-I/O or decoder cases. Run timing-sensitive app witnesses and native qualification in centrally assigned windows. CTest `RUN_SERIAL` covers one plan, and the simulator lease covers separate processes in an execution domain. [CTest `RUN_SERIAL`](https://cmake.org/cmake/help/v3.24/prop_test/RUN_SERIAL.html)

### Scaling the checkpoint

The host and one verified isolated Linux domain can overlap separate platform cells. The earlier F6 campaign demonstrated overlapping native execution and cleanup, with an independent Linux AUTO assertion failure retained. Its observer load cases passed on both lanes during the overlap. For a single platform cell, an offline split by measured case duration estimates 20m 24s native wall time on two equivalent workers or 13m 34s on three. These are scheduling estimates. Implement sharding after the serial lane warrants it and equivalent isolated workers are available. Each shard must bind identical source, app/modules, simulator build and profile, stock dependency and platform. A merger must check that every required ID appears exactly once, every original criterion passes, and all raw evidence and cleanup proofs are present. Compare platform cells separately. [F6 record](../../implementation/2026-11-04-expansion/final/f6/matrix.json) · Linux isolation

## Next batch and file ownership

Use three agents with inherited settings and disjoint worktrees. Root owns host APIs, shared `src` and `tests` CMake wiring, the test registry, capability documentation, and `tests/observer_matrix/`. Agents own their plugin, plugin-local decoders, test subdirectory and operator guide. They can build and validate their component concurrently while root prepares the shared app and firmware witnesses.

| Worker | Owned files | First delivery and its evidence |
| --- | --- | --- |
| Local kinematics and odometry | `src/plugins/OdometryPlugin.cpp`, `tests/odometry/`, `docs/plugins/odometry.md` | Publish `LOCAL_POSITION_NED` as local position and velocity with the FC's NED reference. Keep MAVLink `ODOMETRY` separate, preserving pose/twist frame IDs, covariance unknowns, reset counter and quality. Require a native witness for `LOCAL_POSITION_NED` and an actual app loopback witness for `ODOMETRY`. The two Copter pins handle `ODOMETRY` as an input to the FC. [MAVROS local position](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/local_position.cpp) · [MAVROS odometry](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros_extras/src/plugins/odom.cpp) |
| Air data | `src/plugins/AirDataPlugin.cpp`, `tests/air_data/`, `docs/plugins/air_data.md` | Expose `VFR_HUD` as FC-reported values. Copter can put groundspeed in its airspeed field. Keep ArduPilot `WIND` and common `WIND_COV` distinct and preserve their provenance. Exercise wind families on loopback and require actual HUD on Copter. Leave normalized `AIRSPEED` pressure out of this delivery until sender units are resolved. [MAVROS HUD](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros_extras/src/plugins/vfr_hud.cpp) · [MAVROS wind](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/wind_estimation.cpp) |
| ESC observations | `src/plugins/EscStatusPlugin.cpp`, `tests/esc_status/`, `docs/plugins/esc_status.md` | Decode common `ESC_INFO` and `ESC_STATUS` and ArduPilot `ESC_TELEMETRY_1_TO_4` as distinct families. Preserve sparse group validity and age. Exercise common messages on loopback and require the ArduPilot dialect from the pinned Copter simulator. Simulated electrical fields qualify transport and interpretation only. [MAVROS ESC status](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros_extras/src/plugins/esc_status.cpp) · [MAVROS ESC telemetry](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros_extras/src/plugins/esc_telemetry.cpp) |

### Shared interface and integration order

1. Freeze ABI 9, startup names `odometry`, `air_data` and `esc_status`, distinct MOOS output families and module-local `plugin.<name>.sample_timeout` and `publish_interval`. Follow the existing config validation and observation contracts. Bind observations to the selected source and session. Carry original effective/read time, validity and unknowns. Bound storage, preserve age when republishing and reset on host or heartbeat loss. The modules publish observations through the existing host and have no command or navigation request surface. [External module configuration](../../development/external-plugins.md) · [Existing IMU contract](../../plugins/imu.md)
2. Each agent demonstrates a startup-loaded module through focused tests. Cover valid frames, malformed or wrong-source data, stale/reset behavior, config rejection and zero outgoing MAVLink in an actual app/owned peer path. Supply one candidate native stream and expected units, frame and rate based on the exact sender. Root reviews the contracts and integrates one agent's changes at a time before freezing the combined source and running the software gates.
3. In one serial, disarmed Copter 4.6.3 app witness, preconfigure the needed streams at simulator boot. Require independent wire evidence for `LOCAL_POSITION_NED` (32) and `VFR_HUD` (74), plus `ESC_TELEMETRY_1_TO_4` (11030) once a simulated ESC producer proves it has fresh data on the pinned build. A stream rate alone cannot produce ESC telemetry. If the producer is unavailable, close the two native observations and report the ESC cell pending beside its app loopback result. Use the pinned generated C headers for common ESC test frames, as the existing SITL Python dialect lacks `ESC_INFO` and `ESC_STATUS`. Require corresponding MOOS observations with original ages, an empty bridge egress capture and proven scoped teardown. Reuse the recipe in a separate cell for the development Copter pin if needed to resolve a version difference. Earlier passive captures did not request these streams, so boot-time delivery remains to be demonstrated. [Stable profile](../../../config/firmware/arducopter-4.6.3-quad.json) · [Development profile](../../../config/firmware/arducopter-dev-04cf6486-quad.json) · [Stable ESC sender](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/AP_ESC_Telem/AP_ESC_Telem.cpp#L407)
4. Preflight a narrow second-family `LOCAL_POSITION_NED` RX-only cell on a pinned PX4 simulator, with identity, sender, endpoint and output expectations recorded. Start with wire fixtures for its advertised frame and identity. Keep native PX4 status pending until a separate owned simulator proves the emitted stream and the same module's MOOS output. This cell is independent of primary Copter acceptance. Use the native Copter evidence and reviewed replay to close the first delivery, then run the full pinned checkpoint once on the integrated freeze.

## Local verification of the feedback lane

The fresh macOS ARM64 Debug build used the existing Command Line Tools, stock MOOS-IvP `023fb54e00692611ecee2cd6f1138d10eec2127e`, and the pinned offline MAVLink archive. The build took 33.97s after configuration. Verification used local process inventory and owned loopback, with both simulator roots absent from the build configuration. An initial default compiler attempt encountered the Xcode license check and the default sandbox blocked `ps`. The successful runs used the existing explicit compiler and SDK paths and process access for the local tests.

| Execution or inspection | Result |
| --- | --- |
| Exact-case CLI | 3/3 in 20.53s wall, including the IMU loaded module, its wire codec and the external observer with the real app and MOOSDB |
| Full software CLI | 103/103 in 221.96s wall, including the controller tests and stock helm cases |
| Read-only native-enabled CTest inventory copy | Software 103 cases with zero native, smoke 104 with one native, full 133 with 30 native |

Both execution receipts had exact matching JUnit IDs, zero reported source or artifact drift, and proven command cleanup. Inert CTest controls exercised duplicate, unknown, absent and native focused IDs, dependency rejection, scope-label drift and a missing smoke case. The software runs exercised the working-tree source over `6f53589`, with `run.py` SHA-256 `a491ed2630f18092059a9f54a11ed82663f4941ae9538378868a9a09eaf3be7a` and `RunnerTests.py` SHA-256 `c390521409c72381bee5368c3543d06c50929965f3489f5abdf2172accec9187`. Raw local receipts are under `build-feedback-RP6HAn/focused-01`, `build-feedback-RP6HAn/software-01` and `build-feedback-RP6HAn/native-metadata-only`. The native timing and qualification cited above come from the earlier exact archives.
