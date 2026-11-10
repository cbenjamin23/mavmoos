# RC observations and bounded stream requests

November 4, 2026. Baseline `a9d386e`. The user again authorized clear continuation while asleep and encouraged parallel work. Stock pHelmIvP, default receive-only and no real vehicle/hardware control persist. Prior archives and compiled evidence builds remain unchanged.

## Two useful changes

Add optional `rc_io`, grouping observed RC_CHANNELS input and per-port SERVO_OUTPUT_RAW command output. Preserve channel/port identity, raw PWM units, count/RSSI/extension unknowns, source/session binding and original acquisition ages. Channel roles and actuator mapping/calibration are not inferred. Servo reports are FC command observations, not physical actuator feedback. No override, servo command, manual takeover inference, control backend, NAV projection or readiness gate is added. Use pinned MAVROS rc_io, generated MAVLink and actual ArduPilot sender source to establish semantics.

Replace the current one-shot stream-request burst with a small host service. Each canonical descriptor demand retains an original batch deadline, a bounded local handoff-attempt count and fixed pacing/retry bounds. Proposed internal profile: at most32 demands,10s original batch lifetime,3 handoff attempts per demand,0.1s global spacing and0.2s rejected-send retry delay. These are local resource policies, not guaranteed update rates. Retry only false local handoff results; never retransmit accepted datagrams or infer remote application from an uncorrelated COMMAND_ACK511. Reset/configuration changes invalidate old generations. Recheck heartbeat/session/work/expiry at the final socket predicate. No new permission or control-command lease is attached to observation housekeeping.

NAV/native control retains priority. Stream requests join mission/parameter work in the existing single bounded protocol slot with fair arbitration; adding a service must not double the bulk budget. Diagnostic events distinguish queued, local accepted, locally rejected, exhausted, expired and cancelled work. RX_ONLY/shadow produce zero MAVLink and do not admit a send plan. A failed stream setup remains explicit diagnostic state, not an implicit vehicle-health verdict.

## Work ownership

1. RC agent owns `src/plugins/RcIoPlugin.cpp`, `tests/rc_io/*`, `docs/plugins/rc_io.md` and its feature archive. Normal/sanitizer actual-MODULE tests, pinned source study and bounded unknown/port/reset cases; no native run without root window.
2. Stream agent owns new private `src/libmavmoos_runtime/ObservationRequests.h/.cpp`, `tests/stream_requests/*` and its feature archive. A pure scheduling/state helper and meaningful fake-clock tests; root owns all existing Runtime/app/build edits. Report API early. No socket/native work without root assignment.
3. Native/review agent owns new `tests/rc_io_native/*` and its dated archive. Prepare strict owned Copter receive-only RC-input/servo observation and existing Rover pilot-takeover workflow with the observer. Use the already-qualified native RC fixture semantics; no physical signal/device, override protocol or new public motion command. Independently review RC/source and root stream integration, with negative evidence controls.

Root integrates shared state/effect scheduling, final eligibility, module/config/help/build, public documentation and complete qualification. All agents inherit parent model/reasoning settings. Default ABI7 and provider boundaries remain intact. No upstream source changes, remote publication, physical link or general calibration/parameter-write scope.

## Acceptance

The observer must cover zero/unused/count/RSSI meanings, MAVLink1 versus truncated MAVLink2 extension ambiguity, per-port cache bounds, no false global actuator mapping, original timestamps, same-datagram coalescing, nonrenewal/reset/identity/clock boundaries, and zero effect calls. Native comparisons require actual source values and DataFlash/independent telemetry, not merely plausible ranges or early samples.

The stream service must reproduce the current lost-local-send defect, retry it within fixed bounds, stop after local acceptance, never renew deadlines, reject old epochs/clock faults, preserve queued/dequeued cancellation, and bound work/memory. Combined host tests must prove control priority, one shared protocol slot, mission/parameter fairness, reset/reconfigure final fencing, no RX/shadow output and no ACK511 authority inference. Distinguish pre-send cancellation from already-transmitted residuals.

Run full software/sanitizer/clean source-built Linux and relevant native regressions for the changed host scheduling, serial under the common lock. Record exact code/artifact/profile identity, actual outcomes, failures, closed processes/ports and explicit simulation/clock/remote-effect limits. Native runtime above30s is justified only for bounded FC initialization, existing motion/RC workflow and protocol checks.
