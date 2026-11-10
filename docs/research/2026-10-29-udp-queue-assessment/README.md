# libmavconn queue changes versus a small direct UDP backend

October 29, 2026. **Architecture assessment only.** No library patch, backend, dependency/build scaffolding, mission or runtime experiment was created. Stock MOOS-IvP/pHelmIvP remains unchanged. The [Copter profile](../../profiles/2026-10-29-arducopter-draft-01/README.md), [earlier backend comparison](../2026-10-29-backend-comparison/README.md) and decision 0005 are preserved as prior proposals. Source baseline: 17 pinned files and five preserved architecture inputs.

## 1. Recommendation

For a first deployment that fits **one configured UDP peer and one Copter**, prefer an owned, narrowly scoped UDP backend as the next architecture candidate. It can use generated MAVLink and an I/O library such as Asio; “direct” does not mean writing a codec or a networking framework. This preference is conditional on the real first link/topology and is recorded as [proposed decision 0006](../../decisions/0006-first-single-peer-udp-candidate.md). It does not select a dependency/platform or authorize implementation.

The strongest libmavconn alternative is **a UDP-only, nonblocking write path that bypasses its FIFO**, with queue policy retained in MAVMOOS. A full managed-FIFO retrofit is feasible as a design, but adds active-buffer ownership, cancellation/completion races and an additional policy boundary. Assess that broader route when reusable async/serial/TCP transport is actually needed.

This refines the earlier “evaluate libmavconn first” recommendation: its reuse remains valuable, but the required controlled path also needs endpoint, receive-provenance and write-result work. For the narrow UDP case, much of the policy work is common to both choices, while a libmavconn route carries a vendor patch and packaging/rebuild obligations. No measured code-size, effort, reliability or timing advantage is claimed.

## 2. Three credible routes and the stock-wrapper baseline

| Route | Needed work | Assessment for the narrow first case |
| --- | --- | --- |
| Stock library with a host-side wrapper | Host admission/expiry checks before the existing FIFO | Useful earlier boundary, but it cannot selectively expire/cancel library waiting work or report each write through the public API. Does not establish the proposed stronger write-boundary contract. |
| **L1: Managed UDP FIFO** | Work envelopes, replacement/priority, stable active buffer, write-time gate, per-work results, scoped invalidation/fences; peer/receive changes | Broadest queue/state-machine change. Reuses async transport and can inform later stream designs, but UDP findings do not qualify serial/TCP. |
| **L2: Queue-bypass UDP extension** | Explicit exclusive managed-TX mode, immediate nonblocking datagram attempt/result, fixed peer and receive envelope; host owns bounded scheduling/fences | Smallest credible libmavconn route. It must really bypass the FIFO; merely giving it a shorter queue or invoking it on shared I/O is insufficient. |
| **D: Owned single-peer UDP** | Same host scheduler/fences plus socket lifecycle, peer binding, raw receive envelope, generated-codec integration and diagnostics | Preferred next candidate under the stated topology. Avoids a patched vendor send path; deliberately narrower transport support. |

Both L2 and D use the same proposed host-owned `ManagedEgress` responsibility. It is part of the one-app design, not a new MOOS process/plugin loader or helm interface. Neither libmavconn nor direct generated MAVLink supplies mission/parameter/command transaction policy; that common module work must not be counted as a benefit unique to either transport.

## 3. What the existing source adds to the assessment

The [evidence map](evidence.md) pins these findings:

1. **Active storage is borrowed from the FIFO.** `do_sendto()` passes the front entry's bytes to Asio and captures that entry by reference. Removing/replacing waiting work must preserve the active buffer until its handler finishes. A queue erase operation alone is not a safe cancellation design.
2. **A configured initial remote is not a pinned transmit peer.** Non-permanent-broadcast receive stores the sender in `remote_ep` and later transmits to that member. This supports discovery; the controlled profile needs an explicit stable peer binding instead.
3. **Receive evidence lacks a datagram envelope.** The public parse callback gives packet/framing status, with no receive timestamp/peer/session. Stamp once before parsing/host dispatch and preserve the actual source endpoint independently of TX routing.
4. **Buffer/capacity semantics need review.** RX storage is sized from the maximum MAVLink frame plus padding, not a declared multi-frame UDP-datagram budget. TX count capacity does not establish age. Define truncation, batch/flood and backpressure behavior rather than copying limits blindly.
5. **The library already coalesces its send-chain kick.** A replacement API that posts one unbounded executor task for every producer update could regress this. Bound/coalesce cross-thread admission as well as the final queue.
6. **Close and cancellation are different from a settlement receipt.** Socket-wide cancellation affects receive too. Modern Asio supports per-operation cancellation, but selected-version/platform behavior and completion/buffer lifetime still need qualification. No cancellation request recalls an already accepted datagram.

These are integration constraints and new design work, not claims that the library's existing discovery/FIFO behavior is a defect.

## 4. Minimum software contract for either candidate

- Carry exact work ID, connection/peer/vehicle binding, control generation, original input/permission deadlines, observation dependencies and selection time until the named send boundary.
- Maintain one latest waiting navigation item, a separate one-attempt loss-response item and bounded discrete/support work. Give invalidation and eligible loss response priority over waiting background traffic without pretending they can preempt an already started send.
- Serialize selection, final checks, write initiation and fence application on one egress owner. Bound/coalesce producer admission and receipt delivery too; no hidden executor backlog.
- Recheck exact-work permission and age after preparation, immediately before the datagram attempt. The backend does not repeat cached targets or own command-protocol retry timers.
- Report full local acceptance, would-block/rejection/error, replacement/expiry and any active residual by exact work ID. Distinguish an applied fence from local drain and from FC/remote reconciliation.
- Keep receiving/stamping observations during routine navigation revocation. Reopen/rebind creates a new local connection/peer generation and invalidates old readiness; it does not prove a new remote vehicle incarnation.

The [queue/fence design](queue-and-fence-design.md) gives the ownership, boundary, interface and file-level change sketch. The [direct UDP scope and review](direct-udp-and-review.md) defines the owned path, compares responsibilities and lists 14 paper scenarios.

## 5. Timing and loss behavior

The profile's 0.10 s selected-work age, 0.30 s visible handoff gap, 0.50 s conditional-response window and input/observation deadlines remain **unvalidated candidates**. UDP needs a precise handoff interpretation: this study proposes recording the attempt interval and full local socket acceptance separately. A later completion callback must not make an older initiation look like a new send. This is a proposed boundary clarification, not an amendment to the archived profile.

Nonblocking sends avoid waiting for socket buffer space; they do not create a hard scheduling bound or FC-delivery guarantee. OS/driver/router queues remain downstream. A loss hold is unavailable if it cannot pass fresh Guided/armed/IN_AIR/pose guards and the actual send path within its permit. No queue design should silently replace that with mode recapture, unbounded retries or a stale target.

| Copter draft requirement | Needed mechanism in L1/L2/D | Qualification still required |
| --- | --- | --- |
| Stock input publication/silence age, candidate 2 s | Host retains original per-field expiries through every work/queue stage | Existing MOOS clock/topology assumptions; transport does not certify one helm generation |
| Fresh FC pose/velocity/state/estimator evidence | Datagram/report envelope before deferred host processing; independent report ages | Actual telemetry/sample/queue age and duplicate/session handling |
| Acquisition ends 5 s after admission | Original transaction deadline and phase/permission checked for any attempt | “Once” versus explicitly bounded retries and unresolved remote mode effects |
| Selected-work age at most 0.10 s | Recheck after preparation and before the declared attempt; expire waiting work | Attempt interval, owner scheduling and downstream active-operation limits |
| Visible local handoff gap at most 0.30 s | Exact-work local acceptance evidence; failures/late callbacks do not renew it | Conservative event-time interpretation and actual loop/OS behavior |
| Conditional hold within 0.50 s and all observation expiries | Separate response permit/slot, priority over waiting work, one bounded attempt | Backpressure or active old work may make the response unavailable |
| No mode recapture or automatic resume | Scoped old-generation fences, stopped retry producers, explicit new acquisition and readiness | Local quiescence is separate from old transmitted effects/remote reconciliation |

This maps proposed mechanisms to the draft; it does not establish that any route meets the numerical bounds in execution.

## 6. Remaining decisions

Confirm the first platform/link/peer topology; choose a codec/I/O version and packet/signing policy; resolve packing ownership and receive size/age assumptions; define the exact local handoff/fence receipt; and qualify scheduling, races, errors and downstream residuals. If the first deployment needs serial/TCP, discovery, multiple peers or forwarding, revisit the direct-UDP preference before accepting it. All capability statuses remain Candidate.
