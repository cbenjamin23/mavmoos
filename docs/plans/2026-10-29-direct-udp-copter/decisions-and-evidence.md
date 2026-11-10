# Readiness decisions, coverage and evidence

October 29, 2026. Companion to the [direction review](README.md) and [phase criteria](implementation-plan.md). All decisions below are unresolved for implementation unless explicitly recorded otherwise; planning defaults are proposals. No phase has execution evidence.

## 1. Decisions at the next needed boundary

| Group | Decision to record | Proposed starting point / limit | Needed for |
| --- | --- | --- | --- |
| D-01 | Platform, dependencies and actual endpoint/writer topology | One declared platform, generated codec/I/O pins, isolated configured peer; map FC, any MAVProxy/GCS and all other writers. Verify the bidirectional route before motion. | P0-A/P1; actual SITL route before P4 |
| D-02 | Minimal MOOS projections and request/results | A small typed/as-of/validity map plus acquire/release/status. Resolve mission XY origin, heading versus course, groundspeed and home-relative versus MSL altitude. Select ordinary stock behaviors producing all required dimensions. Do not design the full MAVROS variable/service catalog. | P0-B/P3 |
| D-03 | Local time, ownership, gates, capacities and receipts | Original monotonic expiries; explicitly comparable MOOS publication time; one serialized control-state owner and declared I/O execution policy. Define attempt intervals, FENCE_APPLIED versus local quiescence, and bounded ingress/egress/results. First timing qualification at 1×. | P0-D/P2; measured platform assumptions before P4–P5 |
| D-04 | Concrete execution-profile/vehicle configuration | Exact Copter build/frame/parameters, current home/mission origin, required dynamic reports and IN_AIR semantics, speed/height envelope, control law/masks and signing/addressing policy. Do not substitute an aggregate pre-arm flag for airborne navigation readiness. | P0-C/P4 |
| D-05 | Mode attempts and loss/re-enable policy | Planning default: explicit mode-first acquisition, no hidden retries, conservative all-stop latch and explicit reacquisition. The earlier one-wire-mode-attempt recommendation and conditional MSL hold remain proposed; freeze the chosen policy before effects, without rewriting draft-01 silently. | P0-C/P4 |
| D-06 | Observable motion, timing/load and completion oracles | Predeclare numerical tracking/hold tolerances and settling windows, nominal load and configured native timeout/failsafe/GCS behavior. Use the draft timing values as provisional review targets; specify every applicable value before a test can pass. | P0-C/P4–P5 |

These groups replace another broad open-question exercise. A blank control tolerance or firmware field blocks its motion qualification gate; it does not block fixture decoding. If the real link requires discovery/multiple peers/stream transport, revisit proposed decision 0006 rather than expanding the backend invisibly.

## 2. Timing and interface interpretation

The dated profile proposes 2 s helm publication/silence and selected heartbeat limits, 0.5 s pose/velocity age, 2 s landed-state age, 3 s required dynamic health evidence, 5 s acquisition admission deadline, sampled 0.5 s Guided confirmation, 10 Hz targets, 0.10 s selected-work age, 0.30 s visible handoff gap and a 0.50 s bounded response window. Those are **candidate software/control parameters**, not accepted airframe settings or demonstrated guarantees. The profile's clock-uncertainty/field-spread assumptions also remain in force.

Measure the original timestamps and exact work at the last controllable gate; successful local socket acceptance is a different fact from FC receipt and tracking. A delayed return/result must not renew its initiation/input age. An already accepted packet may arrive after a fence/hold. The proposed local fence prevents matching future write initiation after its serialized application; it does not promise zero old packet arrivals or remote command recall.

MOOSDB can retain scalar values after a publisher stops. Keep a useful last-known projection with explicit stale/as-of status where appropriate, but do not make a new Notify or a heartbeat count as a new observation. Stock consumers may not enforce the bridge's validity model; the bridge's own FC observation guards remain independent of whatever the helm continues to publish. Frame/heading/altitude labels must be settled before describing the selected NAV variables as interoperable.

First 1× qualification avoids mixing accelerated MOOS/simulator time with real monotonic watchdogs. It does not establish the allowed source-clock error automatically. The test manifest must record actual timebase settings, requested and observed cadences and all assumptions. A future accelerated-simulation mode needs an explicit conversion/qualification story rather than scaled or silently disabled watchdogs.

## 3. Coverage and test layers

Reuse [C-01–C-15](../../profiles/2026-10-29-arducopter-draft-01/loss-and-review.md#6-paper-cases-for-later-validation), [B-01–B-14](../../research/2026-10-29-backend-comparison/requirements-and-review.md#5-paper-scenarios-for-later-qualification) and [U-01–U-14](../../research/2026-10-29-udp-queue-assessment/direct-udp-and-review.md#5-paper-review-cases). They are currently paper cases. Associate actual test IDs and evidence later; do not copy them into a new claim of executed tests.

| Layer / phase | What should be established | What it cannot establish |
| --- | --- | --- |
| Fixtures/injected clock and I/O — P1/P2 | Decode/units/provenance, candidate law/masks, admission/expiry/replacement, error outcomes and exact fence/state transitions | Real OS backpressure, network/FC delivery or physical response |
| Isolated real UDP peer — P2 | Socket/framing/addressing, managed-path enforcement, real receive/write event association and close/lifetime behavior | Radio/router bounds, reliable FC acceptance, or a real EAGAIN condition unless actually reproduced |
| Real MOOSDB + stock helm, shadow — P3 | Mail/source/retained-data/state/all-stop semantics, mission projections and zero-control-output default | Atomic solver-generation coherence or effective vehicle control |
| Identified Copter SITL — P4/P5 | Native mode/state behavior, interpreted setpoints, normal motion and scenario-specific loss response under recorded configuration | Hardware qualification, all firmware/parameter sets or hard real-time reliability |
| Reproducible packaged candidate — P6 | Another clean run can reproduce the declared supported simulation scope and operator interface | Broad MAVROS parity, other transports/vehicles or flight readiness |

Most C-cases apply to the first profile. U-cases should be adapted to the actual path: U-03's active async-TX buffer is N/A for the proposed immediate direct send, while RX/close lifetimes still need testing; U-10 maps to delayed attempt/result observation without claiming an async FIFO was exercised. SDK-specific B-paths are N/A when absent. Generic cached-health, late callback/ACK, original-deadline and re-enable issues remain relevant. Provide a per-case reason for every adaptation or N/A.

Fault qualification should distinguish expected inhibition from unacceptable nominal unavailability. For example, injected owner overload can correctly inhibit and pass that fault criterion while a nominal gap miss still fails the nominal timing gate. Likewise, a non-clear all-stop during an ordinary behavior transition can correctly latch but expose an availability/usability tradeoff requiring profile review. Do not weaken it merely to obtain a green demonstration.

## 4. Future evidence bundle and status vocabulary

Use a run ID with a manifest, result ledger, relevant packet capture, MOOS log, bridge event trace and FC/SITL observation log. A compact summary should identify:

- Code/build/binary identity; exact stock MOOS/core, codec/I/O and SITL firmware revisions, frame, parameters and profile revision.
- Endpoints and any proxy/GCS/writer path; selected system/component/source identities, MOOS mission origin, envelope and telemetry mapping.
- Clock domains/timebase/uncertainty assumptions; requested/observed rates; finite nominal load, duration and numerical motion/hold tolerances.
- Case/criterion ID, preconditions, injected fault, expected oracle, observed result, counts/maxima and links to evidence.
- Input/report acquisition times and validity, exact work/transaction/session/generation, gate decision, attempt/result interval, fence application, local quiescence and any residual uncertainty.

Use NOT_RUN, PASS, FAIL and N/A-with-reason for tests. A phase passes only when all applicable criteria have the required evidence; presence of a config file/checklist or a successful packet send is not a pass for a different claim. Retain failures as evidence, record any profile revision, and rerun affected checks after a meaningful change. Keep test injection at controlled test seams rather than adding a production permission bypass.

The proposed three ten-minute nominal runs are a minimum initial evidence sample, not a statistical reliability proof. Fault cases must be repeatable and include delayed/reordered events and restart interleavings; choose repetitions/load explicitly in the run specification. The final report should state what remains unknown rather than extending a simulation result to hardware.

## 5. Documentation and source basis

The manifest preserves twelve dated architecture inputs at predecessor `e83d527`, records four pinned source/documentation files and verifies two archived manual/extraction pairs. No execution-profile draft, old research report or decision 0004–0006 was modified by this plan.

**MOOS documentation:** the live MIT PDF index could not be read through the web tool on October 29. Using the moos-ivp-docs workflow, this review used the already archived October 28 manuals plus pinned local source. The [MIT MOOS chapter](https://oceanai.mit.edu/ivpman/pdfs/chap_moos.pdf), archived PDF and numbered extraction, extracted-text lines 54–57 and 175–207, document acceleration/application callbacks and the requested tick-rate limitation. These are archived manual line spans, not canonical PDF anchors or a claim of a fresh manual download.

The [helm-as-MOOS chapter](https://oceanai.mit.edu/ivpman/pdfs/chap_helm_as_moos.pdf), archived extraction, extracted-text lines 459–479, describes summary/state heartbeat context. It does not fully settle the exact publication/all-stop ordering needed here. That comes from [HelmIvP.cpp 340–422](https://github.com/moos-ivp/moos-ivp/blob/023fb54e00692611ecee2cd6f1138d10eec2127e/ivp/src/pHelmIvP/HelmIvP.cpp#L340-L422), [500–549](https://github.com/moos-ivp/moos-ivp/blob/023fb54e00692611ecee2cd6f1138d10eec2127e/ivp/src/pHelmIvP/HelmIvP.cpp#L500-L549), and [1717–1769](https://github.com/moos-ivp/moos-ivp/blob/023fb54e00692611ecee2cd6f1138d10eec2127e/ivp/src/pHelmIvP/HelmIvP.cpp#L1717-L1769), already analysed in [stock compatibility](../../design/2026-10-29-built-in-modules/stock-helm.md).

**Local source:** `~/moos-ivp/ivp/src/pHelmIvP/HelmIvP.cpp`, spans above, matched the pinned MOOS-IvP revision. `~/moos-ivp/MOOS/MOOSCore/Core/libMOOS/App/MOOSApp.cpp`, lines 510–520 and 693–746, matched the independently pinned Core blob, re-fetched at exact commit; Core is vendored under the local MOOS-IvP checkout rather than a separate local Git repository. Its [time-warp configuration](https://github.com/themoos/core-moos/blob/24942423ce3f3ebdb3cee876edb71bdd04dc5555/Core/libMOOS/App/MOOSApp.cpp#L510-L520) and [mail/iteration path](https://github.com/themoos/core-moos/blob/24942423ce3f3ebdb3cee876edb71bdd04dc5555/Core/libMOOS/App/MOOSApp.cpp#L693-L746) complete checkout-specific context without changing upstream source. No timing measurement is inferred from code.

**SITL context:** [official overview](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html) and [usage](https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html) were consulted October 29. Their RST sources are pinned at wiki commit `23c3dcd846a92aefe7eaf584f222d8bec91fb095`. The [architecture description 55–62](https://github.com/ArduPilot/ardupilot_wiki/blob/23c3dcd846a92aefe7eaf584f222d8bec91fb095/dev/source/docs/sitl-simulator-software-in-the-loop.rst#L55-L62) and [launcher/GCS context 7–15](https://github.com/ArduPilot/ardupilot_wiki/blob/23c3dcd846a92aefe7eaf584f222d8bec91fb095/dev/source/docs/using-sitl-for-ardupilot-testing.rst#L7-L15), [35–48](https://github.com/ArduPilot/ardupilot_wiki/blob/23c3dcd846a92aefe7eaf584f222d8bec91fb095/dev/source/docs/using-sitl-for-ardupilot-testing.rst#L35-L48) motivate recording the real route/writers/build instead of assuming that one UDP endpoint is the entire simulator. Wiki provenance does not select firmware, a launcher command or parameter set.

The verdict, sequencing, minimum run sample and acceptance criteria are design recommendations derived from these inputs. Documentation checks may validate provenance/links/criterion coverage only; none of the proposed implementation, socket, shadow or SITL phases has been executed.

## 6. Documentation checks completed

At creation, four pinned source/documentation records, twelve preserved architecture inputs and two archived manual/extraction pairs passed byte/hash checks. Local links/headings and immutable source-span bounds passed; all seven phase groups, 28 phase criterion IDs and six decision groups were complete. Thirty-three checked historical files remained unchanged, including the dated profile and prior backend/queue reports. The README remained medium length. Every phase gate remains UNPASSED; these checks are not implementation or simulator evidence.
