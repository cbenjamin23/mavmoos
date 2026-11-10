# Backend requirements and review scenarios

October 29, 2026. **Proposed qualification criteria, not an implemented API or executed test plan.** Applies the [initial Copter profile](../../profiles/2026-10-29-arducopter-draft-01/README.md) to the [backend comparison](README.md). Numerical limits remain the profile's unvalidated candidates. Existing profile/contract archives are unchanged; no source or mission work is authorized by this document.

## 1. What a backend must declare

The existing [ProtocolBackend sketch](../../design/2026-10-29-built-in-modules/interfaces.md) already requires declared gates, invalidation capabilities and residual limits. The comparison suggests the following concrete evidence fields for a later revision. Names/encoding remain proposals.

| ID | Required declaration / mechanism | Why this profile needs it |
| --- | --- | --- |
| BF-01 | Actual receive boundary, monotonic timestamp and retained packet/report identity; FC time meaning/mapping where available | 0.5 s pose/velocity age and sampled Guided confirmation cannot use delayed consumption time as receipt |
| BF-02 | Field-to-report provenance and reset rules | A heartbeat cannot renew estimator data, and an SDK in-air boolean cannot replace fresh IN_AIR |
| BF-03 | Exact outbound work identity: target/session, control generation, input-set revision, observation dependencies and immutable deadlines | A current grant must not admit old queued work that happens to have similar bytes |
| BF-04 | One host-owned latest waiting navigation target; bounded in-flight state and explicit overload policy | Do not turn a ten-Hz stream into a backlog of old intent |
| BF-05 | Named gate immediately before the backend's declared write/handoff boundary, checking exact work/current observations | The 0.10 s selected-work and other expiry bounds must survive backend waiting |
| BF-06 | Command retry owner, initial-attempt/retry counts, original admission deadline, retry predicates and producer-stop evidence | The five-second acquisition budget and pilot override must govern every additional mode attempt |
| BF-07 | Selective invalidation scope and a settlement barrier, including helper timers, posted sends, library queues and future retry producers | Dropping a callback or clearing only the host queue leaves other work live |
| BF-08 | Per-frame local admission/drop/write-start/write-completion evidence; honest unknowns for unavailable receipts | SDK/void transport returns are not necessarily successful handoff or FC receipt |
| BF-09 | Partial-frame policy, downstream/OS/driver/network residual model and connection-close consequences | Local cancellation cannot erase emitted bytes; stream writes differ from UDP datagrams |
| BF-10 | Controlled endpoint/forwarding topology and addressing/framing/signing policy | A gate on one common send path cannot govern traffic that bypasses it |
| BF-11 | Scheduling/backpressure model across navigation, commands, heartbeat, rate requests and bulk services | Loss hold cannot wait behind an unbounded service queue; shared I/O delays also affect fresh observation handling |
| BF-12 | Exact dependency revision/configuration/platform and remaining public-API or vendor-extension reliance | Neither a research snapshot nor a nominal sleep proves supported behavior |

Current assessment: direct code could expose these boundaries but has not been written; libmavconn lacks BF-03/05/07/08 mechanisms in its public FIFO API; MAVSDK raw paths offer useful receive/filter hooks but need qualified association/settlement and transport-specific handling. No option is “qualified” by this table.

## 2. Proposed work/deadline discipline

Keep the original input/permission/observation deadlines in the host's work envelope. Selecting a target, packing a new message, setting a packet timestamp, scheduling a retry or receiving a fresh unrelated report does not change them. Evaluate the earliest applicable deadline at selection and again at the declared controllable handoff. The current pose used by the vertical controller must independently remain eligible.

The latest waiting slot replaces only **unsent** navigation. It does not replace an active command transaction, an already started stream write, or the independent bounded loss-response permit. On revocation, invalidate that slot and future producers first. A response hold is admitted only with fresh independent Guided/armed/IN_AIR/pose evidence and a backend path that can meet its own dispatch bound. If old buffers cannot be cleared or bounded, report the hold unavailable rather than hiding it behind a FIFO.

For UDP, a later owned design can gate before a nonblocking whole-datagram send and record acceptance/error. Kernel acceptance is not FC receipt. For serial/TCP, distinguish a frame not yet started from one partially handed off; do not silently splice a newer frame into an unfinished one. Declare whether dispatch means frame start, completed local write or another precisely named event, and how the profile's 0.10/0.30/0.50 s candidate checks apply. A backend-boundary clarification may require a proposed profile revision; no completed-write/FC timing claim is inferred here.

An outgoing SDK interceptor can consult an immutable permission snapshot and record a rejection event. It must not block the SDK I/O thread awaiting the host, call a blocking SDK operation from the hook, or create a new permission itself. Host-state ownership and I/O-gate coordination need a real concurrency design before a hard cancellation/barrier claim can be made.

## 3. Mode requests and retries

Recommend **one initial wire mode attempt, zero automatic additional mode attempts** for the first profile review, matching a conservative reading of “request Guided once.” This reduces hidden recapture paths but can reduce availability on packet loss and does not claim reliable command delivery. Keep SDK/ACK outcome distinct from the two fresh observations that establish current native Guided state. This is a new proposal, not an accepted amendment to draft-01.

If review instead permits protocol retries, specify one logical acquisition transaction with a finite attempt budget. Every attempt must retain the original admission deadline and require current target/session, armed/IN_AIR, eligible initial mode, applicable operator permission and no contradictory override. Stop retries when Guided is observed, acquisition exits its phase, the deadline expires or permission is revoked. Repeating a mode command after a pilot selects RTL/LAND must never be a routine attempt to restore the old grant.

MAVSDK's default command retry count is not this policy. Calling an SDK command again after a host timeout can also create overlapping work; do not multiply host retries by helper retries unknowingly. For services whose ACK carries no unique request token, serialize conflicting operations and retain uncertainty across delayed ACKs/reconnects. A timed-out or cancelled local operation is not evidence that the FC did nothing.

## 4. Cancellation result and re-enable barrier

A future invalidation result should describe at least:

- Which target/session/control generation and local operation classes it covers.
- Whether host waiting work was discarded and future target producers stopped.
- Whether command retries/timers are stopped, merely filtered, or still unresolved.
- Whether library waiting buffers are known empty/discarded, bounded, or opaque.
- Whether a frame has started/completed local emission and what residual uncertainty remains.
- Whether the local barrier permits admitting a new generation, separately from remote outcome reconciliation.

Statuses such as LOCAL_QUEUE_ACCEPTED, DROPPED_BEFORE_WRITE, WRITE_STARTED, LOCAL_WRITE_COMPLETE, REMOTE_ACK_OBSERVED, MODE_OBSERVED and RESIDUAL_UNKNOWN describe different evidence. They are candidate internal/status concepts, not new MOOS channel names. A boolean `cancelled=true` would conceal too much.

If an SDK retry survives an inhibit and later reaches a filter while a new acquisition is allowed, the filter must identify it as old work. Packet content, an eight-bit sequence number or “currently enabled” alone is not sufficient proof of its local generation. Prove a sidecar association or stop/quarantine/drain old producers before re-enable. Closing a connection without stopping those producers can leave work available for a later connection; recreating a fresh instance is a possible coarse design, not a qualified reset protocol. No local barrier promises to erase remote state or already transmitted traffic.

## 5. Paper scenarios for later qualification

Expected outcomes below are proposed architecture review criteria, not executed results. Use the profile's existing C-cases alongside these backend cases.

| ID | Scenario | Required interpretation |
| --- | --- | --- |
| B-01 | SDK callback runs 0.8 s after pose parsing | Preserve original acquisition time; do not admit it under the 0.5 s candidate bound |
| B-02 | Heartbeats keep publishing cached healthy flags after EKF reports stop | Heartbeat/mode can remain fresh while required estimator evidence expires |
| B-03 | Broad SDK in_air stays true during LANDING or an undefined landed report | Require current exact IN_AIR; do not promote the retained boolean |
| B-04 | Two poll/callback executions read one cached Guided report | Do not satisfy the distinct-report/dwell requirement |
| B-05 | Mode command waits in a queue, then its retry timer begins near the host deadline | Original admission deadline still ends eligibility; library send-time timeout cannot extend it |
| B-06 | Pilot selects RTL while an earlier Guided command awaits an ACK/retry | Inhibit navigation and future mode attempts; acknowledge uncertainty for already emitted work |
| B-07 | Old packet is posted, permission is revoked, then a new acquisition starts before it drains | Reject by exact old work identity or keep the new generation inhibited until settlement |
| B-08 | libmavconn overflow/unknown remote, or SDK queued return followed by filtered/no-route delivery | Do not report successful handoff/hold confirmation from the convenience return |
| B-09 | Serial/TCP FIFO retains old velocity ahead of a fresh loss hold | Do not claim bounded capture from late filter/host-slot clearing alone |
| B-10 | An old stream frame is partially emitted when loss is detected | Apply declared finish/abort policy; expose residual bytes/remote uncertainty |
| B-11 | SDK router forwards a control message outside the common outgoing filter | Reject unmanaged topology for the governed path or classify it explicitly as bypass traffic |
| B-12 | A late ACK for the same command ID arrives after a new transaction/session | Preserve attribution limits; do not silently confirm the new grant |
| B-13 | I/O succeeds locally but network/modem delays delivery beyond FC expectations | Report the named local event; do not claim FC receipt, physical stop or timer qualification |
| B-14 | Reuse Offboard setter/stop at zero speed instead of exact profile packets | Detect the extra stream/yaw/mode policy; current SDK Hold maps to Brake, not this profile's conditional hold packet |

## 6. Questions still open

1. Which deployment/platform and physical link comes first? A direct local UDP link, radio serial channel and TCP router have materially different queue/residual behavior.
2. Which stable MAVSDK/libmavconn/codec revision would be evaluated for implementation? The main/fork research snapshots are not selections.
3. Can a small libmavconn queue/write extension expose exact-work metadata, selective invalidation and receipts without excessive maintenance or packaging cost? A front-end-only wrapper does not solve its internal FIFO.
4. Can an SDK raw adapter prove ingress age, queued-work association and a local settlement barrier through supported APIs? What remains outside its gate on the chosen transport?
5. Should “once” remain one wire mode attempt or permit explicitly bounded protocol retries? What observation/ACK policy reports a successful observed transition with a lost ACK?
6. What are the acceptable downstream residual bounds, partial-write/close policy and relation to actual GUID_TIMEOUT/native failsafes? No physical result follows from source inspection.
7. How should future mission/parameter/command services share the transport without competing retry managers or starving control/observations?

Resolve these against the [source traces](source-traces.md) before selecting a backend or opening implementation scope. Ordinary stock helm input/clock limitations remain in force throughout.
