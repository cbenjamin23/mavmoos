# Attitude observations and concurrent feature qualification

November 4, 2026. Baseline `82eb16a`. The user again authorized clear further plugin/improvement work while asleep and encouraged parallel agents. The previous overnight archive and frozen builds remain unchanged. This is a bounded simulator-only continuation, not permission for a new physical route or control mode.

## Intended increment

Add one optional `attitude` capability provider for fused FC Euler/quaternion orientation and body angular rates. It consumes ATTITUDE and ATTITUDE_QUATERNION with ABI7 acquisition/read metadata; it does not own NAV variables, a control backend, permission or effects. Raw accelerometer/gyro/magnetometer hardware, calibration, sensor selection and covariance inference are outside this increment. MAVROS's IMU implementation informs frame/representation handling; pinned MAVLink and ArduPilot source determine actual semantics. Preserve original wire representations, unmapped remote time and source-quality unknowns. Any derived representation must document its convention and input validity. Do not silently fuse independent messages, apply unsupported offsets or manufacture a noise model.

Prove the configured observation providers coexist with stock pHelmIvP control and independent protocol reads. The native fixture should include telemetry, system status, attitude, parameters, missions and ArduCopter control; successful reads and observed movement must coexist without observer-triggered acquisition/revocation. Capture advancing late observations, frame/voltage consistency, actual configured catalog, interval requests, independent FC telemetry, bounded target/update gaps, DataFlash and exact owned cleanup. Record measured behavior rather than claim throughput or worst-case timing from one flight. A status-only composition can serve as a separate controlled comparison if useful.

## Parallel ownership

1. Attitude agent: only `src/plugins/AttitudePlugin.cpp`, new `tests/attitude/*`, `docs/plugins/attitude.md`, and its dated feature evidence. Study source, implement/test actual MODULE with fake Services under normal/sanitizer builds; no native execution without root window.
2. Observation-load agent: new `tests/observation_load/*` and its dated evidence. Reuse proven stock-helm/native orchestration while keeping old fixtures immutable. Prepare the owned flight and strict independent evidence checker/negative controls. Root provides final compiled composition and serial native window.
3. Observer-review agent: independent source/interface review and new combined-host observer tests under `tests/observer_review/*`, plus its review archive. Focus on metadata/session/frames, invalid and cached values, identity, zero effect ownership and unchanged authority; no production edits. Coordinate narrow publication/schema details with the attitude agent.

Root owns shared/app/CMake integration, docs/decision ledger, any needed shared correction, qualification source/artifact freeze and cleanup. All three agents inherit the parent model/reasoning settings. No changes to stock MOOS-IvP, existing native archives or the prior frozen binaries. Use separate worktrees and fresh evidence directories.

## Acceptance and evidence

Real compiled-module tests must cover finite/unknown values, quaternion norm/sign/equivalence, frame conventions at simple known rotations, angular-rate units, extension support, original ages and acquisition bounds, per-family expiry, duplicates/order, heartbeat/restart/clock faults, malformed wire shape and no effect calls. Synthetic invalid data must not become a fresh valid pose. Numeric defaults are explicit local policy, not vehicle guarantees. Source/representation disagreements remain visible.

The combined-host test must bind both observer modules to the host's accepted vehicle and demonstrate that publication/flood/reset behavior does not grant or resurrect navigation or interfere with independent protocol correlation. Native fault/control boundaries stay host-owned. Qualification includes the full software suite, sanitizers, clean source-built Linux, the new owned flight(s), and relevant existing simulator regressions. Record exact revisions/hashes, all material failed attempts, final accepted results and excluded claims. Do not equate local send acceptance with FC execution or a single native run with hardware readiness.
