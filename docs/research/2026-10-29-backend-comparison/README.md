# Backend comparison for the initial ArduCopter profile

October 29, 2026. **Static architecture/source study; no backend selected, implemented or timing-qualified.** Benchmark: [ArduCopter Guided draft-01](../../profiles/2026-10-29-arducopter-draft-01/README.md), using the existing [one-app/module sketch](../../design/2026-10-29-built-in-modules/README.md). Stock MOOS-IvP/pHelmIvP remains unchanged. The profile and earlier archives are preserved.

## 1. Assessment

Prefer a **thin transport boundary with MAVMOOS owning navigation validity, scheduling and acquisition retry policy**. Evaluate libmavconn first for reusable transport, but require a way to enforce expiry and invalidate unsent work at its write queue. Its current public API does not provide that mechanism. Direct MAVLink is the alternative when a small owned transport is cheaper to qualify than adapting a library queue. MAVSDK remains a viable richer backend candidate through its raw-message paths; its default Offboard/action helpers are not a direct implementation of this profile.

This is an architecture recommendation, not a dependency decision or a claim that custom code is inherently more reliable. [Proposed decision 0005](../../decisions/0005-host-owned-navigation-egress.md) records it and the remaining selection conditions.

The important result is that **none of the inspected options supplies this profile's freshness and cancellation semantics unchanged**. The comparison is between different layers: MAVSDK includes vehicle/services/command policy, libmavconn supplies transport/framing, and direct MAVLink means generated protocol code plus transport/services that MAVMOOS would own. All three still need the MOOS-side validity and authority model.

## 2. Reproducible baselines

| Reference | Inspected baseline | Why separate it |
| --- | --- | --- |
| pArduBridge's MAVSDK fork | `Scarter668/MAVSDK` at `e8ff09a5dada80ccded46e28df0395c917d2d997` | Actual local reference dependency; eight selected files matched Git |
| Current upstream MAVSDK research snapshot | `mavlink/MAVSDK` main at `8b1ad926a879a72aefe0e66f606e0349f8186f28` | Public API/layout/send paths differ from the fork; not a selected stable release |
| libmavconn | MAVROS ros2 at `2d3f4db1101c7950c8817a9d788f0d11106fb261` | Same pinned MAVROS baseline; branch head rechecked October 29 |
| Direct generated MAVLink | `mavlink/c_library_v2` at `a2d0ecf011a4aaa7840480e43f4e38d8c9e79d29` | Concrete codec/parser reference, not an existing MAVMOOS transport |
| Command/codec documentation | `mavlink/mavlink-devguide` at `b6d83e20c397f05e634986d400ddb1b42385fa19` | Distinguishes command retries/cancellation from setpoint traffic and local queues |

The 52-file manifest records hashes, lengths, revisions and inspection methods, including two pArduBridge source files. Branch heads were resolved on October 29; this is not a release/platform/firmware support assessment. [Evidence map](evidence.md).

## 3. Comparison against the profile

| Concern | MAVSDK | libmavconn | Direct MAVLink with owned I/O |
| --- | --- | --- | --- |
| Fresh mode/armed/landed reports | Typed callbacks are convenient, but callback execution time is not packet receipt time. Raw ingress/interception can provide the required evidence with adapter work. | Parse callback exposes packet and framing status; host can stamp/copy it before dispatch. No freshness policy supplied. | Host controls receive stamping, target filtering, parsing and event envelopes; must implement them correctly. |
| Position/health provenance | Some typed conversions omit source time; aggregate health can be republished by an unrelated heartbeat. Need underlying per-report provenance. | Raw fields remain available; host must interpret clocks, estimator validity and home changes. | Same protocol information; full responsibility for interpretation and reset/clock behavior. |
| Exact velocity/yaw masks | Raw Passthrough/MavlinkDirect can encode them. Offboard's velocity helper always requests yaw and starts a separate repeating stream. | Can encode the profile's exact messages/masks. | Can encode the same messages using generated code. |
| Acquisition retries | Inspected command sender retries independently of the host's acquisition deadline; higher-level public calls do not expose a per-request epoch/absolute deadline/cancel handle. | No command retry manager in the transport; MAVMOOS must supply one if used. | Same host responsibility; retry policy can share the navigation permission/deadline model. |
| Unsent navigation replacement | Host can replace its own slot. Already posted SDK work is separate; gate/association or a quiescence barrier is needed. | FIFO accepts copied buffers; no public latest-only replacement, per-message TTL or selective cancel. | Can design one latest waiting target plus bounded in-flight work; this is proposed code to build, not an existing guarantee. |
| Transport queues | Current upstream posts sends to Asio; UDP sends synchronously after its gate, while serial/TCP add bounded waiting queues. Fork uses different synchronous paths. | UDP/serial/TCP use async FIFO queues, with a 1,000-buffer limit. Capacity is not an age bound. | Queue, write completion and cancellation can be explicit, but kernel/driver/network residuals still remain. |
| Completion evidence | SDK success can mean local acceptance; some filtered/no-route paths are still successful locally. Need named send/result/observation evidence. | `send_message` returns void; overflow can throw, while closed/no-remote UDP can return without sending. No per-frame public completion receipt. | Can define precise local results; neither a socket write nor a command ACK proves the profile's physical response. |
| Integration cost | Most service reuse; more independent timers, callbacks, queues and version-specific behavior to account for. | Reuses transport/framing with smaller policy surface. Pinned packaging still uses ament/C++20 and other dependencies; standalone packaging/platform feasibility is open. | Least hidden policy and most owned code: I/O, reconnection, routing/signing integration, command/mission/parameter services and maintenance. |

Each conclusion is traced in [source traces](source-traces.md). No throughput, latency, cancellation race or vehicle test was run.

## 4. Freshness is an evidence problem

The profile proposes 2 s helm publication/silence bounds, 0.5 s position/velocity age, 2 s mode/landed-state age, a 5 s acquisition deadline, 10 Hz targets, a 0.30 s visible handoff-gap limit and 0.10 s selected-work age. **All remain review candidates, not achieved bounds.** A backend must preserve the original eligibility deadlines across its own queues; repeatedly sending or reading a cached value cannot refresh them.

There are at least four different times: FC sample/publication time where provided, host parse/receive time, later callback/host-consumption time, and outbound handoff time. Record them separately. Stamping a delayed SDK callback as “received now” hides earlier waiting. Conversely, raw parsing time still does not bound upstream network/OS buffering or prove physical sample age. Heartbeats lack the profile's full measurement provenance; packet sequence is not a producer session or control-generation identifier.

MAVSDK is not uniformly “change-only.” Both inspected versions dispatch mode/armed callbacks on heartbeat processing. The more important findings are that those callbacks are deferred and that health is also republished from cache on heartbeats. Use raw target heartbeat reports for the two-report Guided dwell and underlying estimator reports for their own age. The broad SDK `in_air` boolean also combines flying/takeoff/landing and retains its prior value for unknown landed state; use the actual fresh `IN_AIR` enum required by this profile. [Ingress trace](source-traces.md#1-ingress-and-observation-age).

No backend can repair stock helm scalar generation coherence. MOOS adaptation remains `ARRIVAL_GROUPED` with its declared clock/publication assumptions, regardless of the MAVLink transport.

## 5. Retry and cancellation boundaries

Treat these as separate mechanisms:

1. **Routine navigation scheduling:** a fresh evaluation of the latest eligible input set at each tick, not reliable-delivery retries of an old velocity packet. Suppressed/expired work must not become eligible because another field or grant later becomes fresh.
2. **Command-protocol retry:** another attempt at a particular transaction under its original permission, deadline and attempt budget. It must not survive override/timeout or silently restart after reacquisition.
3. **Local queue cancellation:** discard work that has not crossed the declared controllable boundary and stop future producers/timers. Ignoring a future result callback does not do either.
4. **Remote cancellation or compensation:** a separate supported protocol operation/vehicle action. `COMMAND_CANCEL` applies to supported long-running commands; it neither purges a local transport queue nor generically undoes a mode change or velocity setpoint. Already transmitted/FC-accepted work is not recalled by a local generation increment.

MAVSDK's current command sender defaults to **three retries beyond the initial attempt**; the fork stores the same retry count in each work item. These are library policies, not evidence that the profile's admission-to-completion deadline is met. Queue wait, progress handling and transport delay are separate. A host timeout alone does not stop that sender. [Command trace](source-traces.md#2-guided-acquisition-and-command-retries).

**Proposed clarification:** read the profile's “request GUIDED once” conservatively as one wire mode attempt for the initial version, with no automatic additional mode attempts. Fresh Guided observations can establish observed state without pretending an ACK was received. If protocol retries are desired, revise that policy explicitly to one logical transaction with a stated retry budget; every attempt must recheck the original deadline/current observations and stop when Guided is observed, acquisition leaves its phase, or an override occurs. Do not inherit SDK defaults accidentally. This recommendation has not changed or accepted the archived profile.

Cancellation must report what actually settled: host waiting work, SDK retry producers, library waiting buffers, and in-flight/remote uncertainty. A coarse link close may abort local operations and lose telemetry; it does not erase already emitted data. Reconnection/re-enable needs a barrier against old work returning. [Egress trace](source-traces.md#3-navigation-egress-and-queues).

## 6. Why a raw SDK gate is useful but insufficient alone

The current SDK's common send path posts a message, then invokes outgoing C/JSON interceptors before selecting connections. This offers a late rejection point for inspected traffic. It can suppress retries or expired packets at that point, including work that waited in the SDK event loop.

However, the callback receives a wire message, not the original MAVMOOS request/generation/deadline. Checking only “navigation currently enabled” can admit an old packet after a new enable. The adapter must prove sidecar association or settle/quarantine earlier work before a new generation is permitted. Standard packet sequence/time fields are not a new end-to-end epoch contract. Serial/TCP queues downstream of the interceptor have no per-frame eligibility recheck; routing/forwarding can also take a different path. A filtered message is not reported as a per-request cancellation result. [Gate details and limitations](source-traces.md#4-sdk-interceptors-forwarding-and-version-differences).

## 7. Recommended next decision surface

- Keep one host-owned latest navigation slot, explicit acquisition state, per-observation evidence and deadline checks whichever transport is used.
- For libmavconn, assess a narrowly scoped queue/write extension or adapter boundary that carries deadline/generation metadata, rejects expired work before write and acknowledges selective invalidation. A wrapper that only checks before FIFO insertion is insufficient for the stronger claim. No fork/extension has been created.
- Compare that cost with a small direct UDP-first transport design. Serial/TCP require separate treatment of partial/in-flight frames and downstream buffering; do not assume UDP results qualify them.
- Keep MAVSDK's raw paths as an alternative, subject to ingress stamps, queued-work association, helper/retry shutdown and transport-specific gate limits. Broader service reuse can be evaluated separately; avoid running two conflicting command managers against the same FC transaction.
- Require the [backend evidence/qualification checklist](requirements-and-review.md) before any implementation scope or backend is accepted.

The study resolves the abstraction choice more strongly than the dependency choice: MAVMOOS should own the policy that decides **whether and when** navigation or another mode attempt may be sent. A transport or service library must expose enough evidence and control to enforce that policy. Actual firmware, backend version, platform, queue limits and physical loss response remain unqualified.
