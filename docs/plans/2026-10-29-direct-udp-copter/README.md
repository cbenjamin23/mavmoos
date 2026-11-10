# Direction review and first implementation plan

October 29, 2026. **Draft plan; architecture-only. No implementation started and no phase gate passed.** The user requested planning for direct UDP. It is the planning baseline, with actual platform, dependency versions, firmware and topology still to be established. Stock MOOS-IvP/pHelmIvP remains unchanged under [decision 0004](../../decisions/0004-use-stock-moos-ivp.md). The dated Copter profile and earlier research/decisions remain preserved.

## 1. Big-picture assessment

The direction still serves the original MAVMOOS goal: a broadly useful MAVLink-to-MOOS bridge, with vehicle-aware helm integration. One modular app, useful telemetry without a helm, explicit authority and separate vehicle/control adapters remain sound design goals. A direct UDP first transport is a reasonable bounded way to exercise them.

The correction is **sequencing**. Our documentation is ahead of a demonstrated vertical slice. We have enough backend comparison to proceed toward small observable results once implementation is authorized. More generic contracts, plugin machinery or transport comparisons should be driven by a concrete failed gate or new deployment requirement. The current detailed interface sketches are proposals, not a framework that all must be implemented before reading a heartbeat.

The first slice should show:

1. One bridge app makes correctly labelled Copter telemetry useful in MOOS without a helm.
2. An ordinary stock-helm mission produces course/groundspeed/home-relative-altitude requests that the bridge can explain in shadow mode.
3. Explicit acquisition allows eligible requests to drive an already-airborne Copter in SITL; observed takeover/input loss/restart inhibits the bridge as specified.

That would be evidence toward MAVMOOS. A well-tested UDP socket alone would not establish the vehicle/helm integration, and Copter motion alone would not establish a reusable bridge.

## 2. Preserve the broad goal with a small first boundary

| Keep in the first slice | Leave for demonstrated need/later scope |
| --- | --- |
| One app; simple built-in status/telemetry/helm features selectable by configuration | Runtime plugin loading, a general plugin ABI or elaborate dependency resolution |
| Transport/codec seam; reusable observation and egress records | Production support for a second backend, serial/TCP, discovery, routing or multiple vehicles |
| Copter adapter with the profile's explicit quantities/owned loops | Plane/Rover/PX4 control laws and a universal zero-speed/fallback policy |
| Minimal acquire/release transaction and clear local/observed-state results | A complete general command, mission and parameter service catalog |
| Original age/deadline checks, one latest waiting target, scoped local fences | Stronger producer-generation guarantees or protocols requiring helm changes |
| Repeatable fake-peer and SITL evidence | Hardware/field qualification or broad MAVROS parity claims |

Use generated MAVLink and an appropriate I/O library rather than writing a codec/networking framework. Keep Copter constants, mode interpretation and the outer altitude law out of the UDP adapter. Keep socket/message details out of ordinary mission/operator flows. The FC retains its inner controllers/failsafes; this slice does not add a parallel marine PID/actuator chain.

The useful seams are the existing host/observation store, stock-helm adapter, Copter adapter, supervisor/transaction coordinator and ManagedEgress/UDP boundary. These can begin as small private classes/functions with a compiled-in module table. No second production backend is required to justify every interface in advance.

## 3. Phases and first review points

All entries below are **unpassed**. See [phase tasks and acceptance criteria](implementation-plan.md).

| Phase | Result to review | Evidence required to move on |
| --- | --- | --- |
| P0 | Staged readiness record | Basic build/link scope before P1; MOOS map before P3; concrete control/firmware/test choices before P4 |
| P1 | Receive-only app and MOOS telemetry | Known fixtures/target reports produce correct values/ages; packet capture shows no outbound MAVLink |
| P2 | Bounded egress and control core against fake I/O/UDP peer | Exact packets, original deadlines, honest results and old-generation fence behavior |
| P3 | Stock-helm shadow integration | All required stock inputs/state/all-stop are interpreted correctly with control output disabled |
| P4 | Basic Copter SITL control | Explicit mode acquisition and eligible velocity targets produce the predeclared observed response |
| P5 | Fault/availability qualification in SITL | Relevant C/B/U scenarios, nominal timing runs and correct inhibition/recovery under faults |
| P6 | Reproducible SITL candidate and operator handoff | Repeatable evidence bundle, usable diagnostics, documented limits and telemetry-only regression |

P0 is deliberately staged: unknown physical limits or firmware details do not block receive-only fixture work. They do block effectful SITL control. Technical gates are acceptance checks, not a request for a separate human confirmation after every phase. Executing any implementation remains outside this architecture-only request.

## 4. Assumptions to verify early

The most important unresolved assumption is the actual link topology. One configured UDP peer may be a gateway such as MAVProxy rather than the FC process. Record every hop, writer, endpoint and control path; demonstrate the intended binding before enabling motion. Do not quietly add last-sender learning/forwarding to make the narrow backend appear compatible.

First timing qualification uses a declared 1× MOOS/SITL timebase and monotonic local watchdogs. Actual source-clock uncertainty still needs evidence; 1× is not clock synchronization. An accelerated simulation requires a separately declared clock interpretation. AppTick/CommsTick requests are not evidence that the profile's send deadlines were met. [Documentation/source basis](decisions-and-evidence.md#5-documentation-and-source-basis).

Before motion tests, resolve numeric mission envelopes, required observation semantics, exact mode-attempt policy, local handoff/fence interpretation, conditional hold behavior and motion/settling tolerances. The [six decision groups](decisions-and-evidence.md#1-decisions-at-the-next-needed-boundary) identify when each is needed. No current timer/gain, loss packet or library reference becomes accepted merely by appearing in this plan.

## 5. What counts as progress

Prefer a short demonstration, packet/host trace and one clear pass/fail summary at each phase. Reuse the existing profile/backend/queue review cases instead of starting another large question register. Record a test as not applicable with a reason when its SDK or async-FIFO path does not exist; do not count it as passed.

The proposed endpoint is a **reproducible SITL candidate for one profile**, with source/configuration-specific evidence. It is useful enough to evaluate the architecture and choose the next feature. It is not hardware readiness, a physical-stop guarantee, proof of atomic stock-helm decisions or broad vehicle support. A failure of the real topology or control assumptions should revise the relevant profile/backend decision; a nominal scheduling miss should not be hidden by silently relaxing deadlines.

The [evidence/decision record](decisions-and-evidence.md) defines test layers, coverage and reporting. The manifest records four pinned source/documentation inputs, twelve preserved architecture inputs and two archived manual/extraction pairs. No new accepted decision is needed to archive this draft plan; decision 0006 remains conditional/proposed.
