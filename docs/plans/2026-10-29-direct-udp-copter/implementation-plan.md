# Proposed phases and acceptance criteria

October 29, 2026. **All gates unpassed; no code, build scaffold, runnable mission or simulator run was produced.** Criteria describe future observable results. Acceptance is scoped to the selected version/configuration/test layer; a passed lower-level test does not establish FC behavior. [Direction review](README.md), [decisions/evidence](decisions-and-evidence.md).

## P0 — Establish staged implementation readiness

Deliver a short readiness/configuration record, not another general architecture study. It should name assumptions, selected values, evidence and unresolved items. Use the [six decision groups](decisions-and-evidence.md#1-decisions-at-the-next-needed-boundary).

| ID | Acceptance criterion | Needed before |
| --- | --- | --- |
| P0-A | Name one build/test platform, exact stock MOOS/core revision, candidate codec/I/O pins, app/config boundary and isolated configured-peer fixture endpoints. Record the real SITL/link topology when available; do not assume a UDP port number proves the peer/route. | P1 fixture/build work; actual topology before P4 |
| P0-B | Specify the minimum MOOS telemetry/validity and acquire/release/status map: types, units, mission origin, heading versus course, altitude references, source/as-of meaning, stale handling and request/result identity. Identify the ordinary stock behaviors/domains that produce all three required inputs. | P3; P1 may expose clearly labelled raw diagnostics first |
| P0-C | Record a concrete SITL firmware/frame/parameter snapshot and execution-profile revision: speed/height envelope, required estimator/home/IN_AIR evidence, acquisition/retry/all-stop/recovery policy, gains/masks, native failsafes and numerical motion/hold tolerances. All required values must be concrete before control tests can pass. | P4 |
| P0-D | Define monotonic clock ownership, original expiry calculation, exact local attempt/acceptance/fence events, bounded admission/results and overload policy. Record initial test budgets using the draft values provisionally, plus 1× qualification timebase and evidence format. Any later policy/budget change gets a versioned rationale. | P2; completed platform/timebase/load specification before P4–P5 |

P0-A unblocks P1 without waiting for every P0-C vehicle decision. Do not implement the full historical strict navigation contract or require new helm publications as a readiness condition.

## P1 — Deliver receive-only telemetry as the first useful increment

Future work: minimal app/build wiring on the selected platform; small built-in status/telemetry features; configured-peer UDP receive and generated decoding; observation records and a narrow MOOS projection. Default operation is RX_ONLY, with helm support disabled and no hidden SDK/helper sender.

| ID | Acceptance criterion |
| --- | --- |
| P1-A | A clean build/start works on the declared platform, registers/publishes its documented MOOS interface and remains useful with no helm running. Missing/invalid target or configuration leaves it visibly inhibited. |
| P1-B | Known peer fixtures decode mode/armed, exact landed state, global/relative altitude, velocity and required home/estimator reports with correct units/reference. Wrong peer/target/framing, unknown values and zero coordinates have explicit outcomes; no invented zero makes an absent observation valid. |
| P1-C | Endpoint capture and the bridge ledger show **zero outbound MAVLink** in RX_ONLY, including while MOOS requests arrive. Status/telemetry publication alone cannot start heartbeats, rate requests, mode changes or setpoints. |
| P1-D | Original datagram/report acquisition evidence survives deferred host publication. Stopping one report family expires that family even while other traffic continues. A later Notify/status update or heartbeat cannot renew cached estimator/pose age. Queue/batch limits and source-age unknowns are visible. |

Review artifact: a short receive-only demonstration, fixture/packet records and MOOS values/age transitions. Add real SITL receive evidence when its topology is available; keep fixture and SITL results separately labelled.

## P2 — Prove bounded egress and control logic against fake I/O/UDP

Future work: host-owned latest waiting NAV slot, separate bounded response/discrete work, codec preparation, immediate nonblocking UDP attempt/results, scoped fences and a narrow supervisor/Copter transformation. Use an injectable clock/I/O seam for faults and an isolated loopback fake peer for real socket/framing evidence. Command-protocol retries remain host policy; transport adds none.

| ID | Acceptance criterion |
| --- | --- |
| P2-A | Deterministic inputs reproduce the profile's course/speed-to-NED mapping, home-relative altitude feedback/clip and moving/near-zero yaw masks, plus conditional hold encoding. Missing/negative/non-finite/out-of-envelope inputs and unknown references are rejected. Original field/permission/observation deadlines never renew through packing, selection or sending. |
| P2-B | Injected would-block/error/late-return/queue pressure produces exact-work outcomes without an inline resend loop or false handoff. Latest waiting NAV replaces older unsent work. Eligible response priority applies only to waiting work; any unavailable bounded response is reported honestly. |
| P2-C | After FENCE_APPLIED for an old scope, injected delayed submissions/results and a later enable cannot initiate old-scope writes. Admission/application of the fence, local quiescence and already accepted residuals remain distinct. Race/lifetime checks cover RX/close and any owner/worker handoff; supported sanitizer findings are resolved, not waived as passed tests. |
| P2-D | The real fake-peer socket observes exact addressing/bytes/masks and explicit full-acceptance/error accounting through the managed path. A different sender cannot redirect TX. Legacy/raw bypass entry points cannot evade the gate. Deterministic EAGAIN tests are not represented as proof that real UDP buffer pressure was reproduced. |

Review artifact: deterministic state/effect trace, packet comparison and bounded-resource/fence summary. These establish local software behavior; they do not establish radio, FC receipt or physical capture.

## P3 — Integrate stock pHelmIvP in shadow mode

Future work: ordinary `.moos`/`.bhv` configuration for the selected stock behaviors, the stock scalar/state/all-stop adapter, minimal acquire/release/status request path and operator diagnostics. Shadow mode computes/explains intent and eligibility while the control transmit capability remains disabled.

| ID | Acceptance criterion |
| --- | --- |
| P3-A | Record the running stock helm's source/build provenance. The mission produces DESIRED_HEADING/SPEED/ALTITUDE with ordinary configuration, without a producer hook, modified behavior output contract or upstream source change. Exactly the declared telemetry/source chain supplies the selected NAV quantities. |
| P3-B | Required post-boundary fields, source/type/finite checks, publication/silence age, selected DRIVE evidence and latched all-stop work as documented. A legitimately old `clear` timestamp alone does not fail a healthy stream; heartbeat/clear cannot refresh expired scalars. New unchanged-value publications and duplicate notifications are distinguished. |
| P3-C | Real endpoint capture shows zero control/mode/setpoint transmissions even with valid scalars and enable requests. Shadow eligibility is labelled as such, never reported as observed Guided control or physical execution. Configuration-disabled control cannot be enabled solely by a MOOS request. |
| P3-D | Helm silence/all-stop, stale FC data, retained registration data, MOOS clock discontinuity and source mismatch give explicit blocked/loss reasons. The input-set label remains ARRIVAL_GROUPED; no test claims one solver generation. |

Review artifact: stock-helm/MOOS log plus matched shadow decisions and zero-control-output capture. Compare ordinary behavior transitions with the conservative all-stop latch; record the availability/re-enable impact instead of silently relaxing it.

## P4 — Demonstrate the basic Copter SITL workflow

Prerequisites: P0-C complete and P1–P3 accepted at their stated layers. Use a separate declared simulator/GCS fixture to establish already-armed, IN_AIR state; arming/takeoff are not bridge capabilities. Record all MAVProxy/GCS hops/writers, source IDs and parameters. Enable only the reviewed managed effects for this isolated scenario.

| ID | Acceptance criterion |
| --- | --- |
| P4-A | The actual route delivers selected FC reports and routes a declared allowed non-navigation probe/support message with meaningful far-hop evidence. The running firmware/frame/params, 1× clocks and motion tolerances match the record. Packet audit shows no bridge arm/takeoff/mission/parameter-write side effect. Any heartbeat/rate request is separately allowlisted/logged. |
| P4-B | Explicit acquisition from native LOITER or GUIDED binds fresh inputs/current target. The original admission deadline governs attempts; two distinct current Guided/armed reports meet the configured confirmation span. A cached mode or ACK alone cannot activate streaming. Unsupported entry modes and duplicate/stale requests cannot cause an extra unapproved mode attempt. |
| P4-C | A stock-helm scenario exercises horizontal direction/speed, zero horizontal speed and altitude changes inside the frozen envelope. Captured targets match the candidate law/masks and FC-reported motion reaches the predeclared tolerances/settling windows. Target submission/ACK is reported separately from those observations. There is no parallel pArduBridge/marine-PID/NAV writer driving the same quantities. |
| P4-D | Basic release/input-loss revokes routine NAV and latches. A permitted response uses one fresh current-position/MSL record and the reviewed one-attempt response contract, only while still Guided with current guards. No response enters Guided again or resumes on fresh input alone. Hold handoff and any measured hold predicate are reported separately. |

Review artifact: one small end-to-end demonstration with MOOS/packet/FC traces. If mode-first entry, yaw, altitude law or hold response contradicts the selected firmware, stop/revise that profile with evidence before extending tests.

## P5 — Qualify faults, timing and availability in SITL

Future work: a scoped repeatable fault runner and coverage ledger using [existing C/B/U cases](decisions-and-evidence.md#3-coverage-and-test-layers). It should classify preconditions, expected inhibition/response, actual local attempts and observed FC behavior separately.

| ID | Acceptance criterion |
| --- | --- |
| P5-A | Every relevant C/B/U case has evidence or an explicit NOT_RUN/FAIL/N/A reason. N/A is justified by the actual path, not counted as a pass. Source maps such as SDK helper retry paths or L1 active async-TX storage are not claimed tested by this direct-UDP backend. |
| P5-B | Exercise helm/MOOSDB loss, stale pose/estimator/landed reports, target/home/clock/reconnect changes, observed pilot/FC mode override, pending mode timeout/ACK ambiguity, backpressure and owner delay. On observed override, further eligible managed writes stop; this is not a claim of instantaneous awareness of an unreported mode change. Old queued work cannot return under a new enable. |
| P5-C | Proposed minimum nominal evidence: three clean-start runs with at least ten minutes of eligible control each, under a recorded finite load/network/timebase. Measure every attempt's input/observation/selected-work ages and visible acceptance gaps against the frozen budgets, including maxima and miss counts. A nominal miss fails that availability gate; correct inhibition under deliberately injected overload is a separate fault result. These samples are not a hard-real-time/statistical reliability proof. |
| P5-D | For each loss family, distinguish conditional response attempted/unavailable, local acceptance, FC observation and uncertainty. A crash promises no software cleanup; actual configured FC timeout/failsafe/GCS-heartbeat behavior is recorded. Restart/reconnect and later clear/fresh scalars never resume automatically. Any claim about physical capture or settling uses the predeclared observed predicate; no timer alone passes it. |

Review artifact: compact pass/fail/unknown ledger, timing distributions/maxima and representative traces. An unmet budget changes the qualified claim or a versioned profile; it is not fixed by timestamp renewal or undocumented timer relaxation.

## P6 — Package a reproducible SITL candidate

Future work: documented single-platform build/test entry, ordinary example mission configuration, evidence index and concise operator interface. The endpoint is reviewed simulation behavior for this profile, with explicit remaining hardware/other-vehicle work.

| ID | Acceptance criterion |
| --- | --- |
| P6-A | A clean environment reproduces the declared build and relevant test sequence with pinned binary/source/config/firmware identity, owned test endpoints/processes and retained evidence. No undocumented local state or modified stock helm is needed. |
| P6-B | A user can see observed FC mode versus bridge permission, current data ages/blocked reason, request outcome and required re-enable action. Documentation states the actual topology, quantities/references and limits, without requiring ordinary operators to reason about socket internals. |
| P6-C | The evidence bundle supports only its actual scope: local software tests, loopback transport and identified SITL scenarios. Capability/decision/profile documents cite results accurately; unimplemented MAVROS-like areas and other vehicles/platforms remain unqualified. Hardware qualification is separately scoped later. |
| P6-D | Receive-only/helm-disabled operation still passes after control integration. Copter policy remains outside the UDP/codec boundary and no control module bypasses the managed gate. A fake I/O seam is sufficient to check that separation; a second production backend/plugin framework is not required for this milestone. |

Treat phases as reviewable changes, not calendar commitments. No implementation PR, test executable, simulator, deployment or hardware workflow was created by drafting this plan.
