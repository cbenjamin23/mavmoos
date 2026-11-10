# Queue, write-boundary and cancellation design

October 29, 2026. Proposed interfaces/state semantics only. Names below are review vocabulary, not committed C++ signatures or new MOOS variables. Applies to [L1/L2/D in the assessment](README.md); no source was patched.

## 1. Common ownership

Keep a transport-neutral `ManagedEgress` responsibility inside MAVMOOS. The host supervisor owns authority, current state, navigation translation and any command-protocol retry decision. An egress owner serializes queue selection, codec/write preparation, the final check, write initiation, completion handling and invalidation fences. They may share an executor or use bounded event transfer; no new thread count is selected.

A `TxWork` needs:

| Field group | Purpose |
| --- | --- |
| Unique local work ID; transaction/attempt lineage where applicable | Exact admission/result association; sequence numbers and identical packet bytes are not sufficient |
| Connection epoch, immutable peer binding, vehicle/session identity and control generation | Prevent old or differently addressed work from becoming eligible under a later enable/rebind |
| Operation class and permission/response token | Routine NAV, acquisition attempt, loss response and support/heartbeat have different guards |
| Input-set revision, original absolute monotonic deadlines and current observation dependencies | Packing, queue movement or a new unrelated report cannot renew the work |
| Selected/prepared times, intended message/frame and destination | Enforce selected-work age and preserve the exact target being attempted |

The backend need not understand MOOS variable names or Copter control laws. A bounded admission check uses the supervisor's immutable policy/state snapshot and the exact work envelope. A current enabled flag alone is insufficient. Snapshot/version and fence ordering require an actual concurrency design; this document does not assert atomicity between an arbitrary host-thread update and an OS call.

The gate is a bounded decision over that snapshot and current time. It must not wait for the supervisor, re-enter library/host operations through held locks, perform I/O or create/renew a grant. A snapshot mismatch inhibits the attempt until the owner has current evidence.

## 2. Bounded admission and selection

Use one replaceable **waiting** NAV slot for the initial vehicle, a separate bounded one-shot response slot, and bounded discrete/support work. A replaced work item gets a terminal local result. No replacement silently modifies a command transaction or active buffer. For L1 this logical slot is delegated to the managed library scheduler; coalesced host admission must supersede older library waiting work before dispatch, rather than creating two independent queues. The final check verifies that the selected work is still the current permitted NAV item. Exact count/byte/event capacities remain to be selected; capacity is not an age guarantee.

Bound the producer-to-owner mailbox before posting executor work. Coalesce NAV updates and allow at most a bounded wake/control signal set; do not hide unlimited payload copies inside Asio's task queue. Reserve admission for invalidation/close/fence messages. Bound outgoing result/event transfer too, preserving settlement evidence or inhibiting/reconciling when it cannot be delivered reliably.

At each dispatch, process pending invalidation first, discard expired/revoked waiting items, and select eligible work under the host's schedule. An eligible loss response can precede waiting support work; it cannot preempt a send already started. Background requests/heartbeat work need explicit service budgets so neither TX nor RX processing starves the other's guards. No new numerical queue/fairness defaults are proposed.

For NAV, consume a selected item once. A later scheduled NAV item may be built from still-eligible inputs under their original deadlines; a backend timer cannot independently repeat the consumed packet. Mode/command retries are distinct host-authorized attempts under the transaction's original deadline. Existing “one wire attempt versus bounded retry transaction” proposals remain unresolved; this transport study does not change them.

## 3. Final gate and result evidence

Prepare a full frame under a single codec/channel/signing-state owner. Preserve the intended target, message content and work association. Recheck current connection/peer/vehicle binding, applicable token, exact-work deadlines, required observations and selected-work age immediately before the write attempt. If preparation used enough time to expire the item, drop it with an expired result.

Suggested internal events distinguish ADMITTED, REPLACED, EXPIRED, REJECTED_AT_GATE, ATTEMPT_STARTED, LOCAL_ACCEPTED, WOULD_BLOCK, LOCAL_ERROR, FENCE_APPLIED and LOCAL_QUIESCENT. They are not flight-controller success states. Record monotonic attempt-start and return/completion-observed times. Full local acceptance is a named transport fact; no ACK, observed Guided state or physical hold is inferred from it.

For synchronous nonblocking UDP, the call result brackets local acceptance between call start/return. For async UDP, acceptance is not established by initiation; completion may be observed later. Retain the initiation time and the uncertainty interval. Use conservative earlier-time evidence for watchdog/age reasoning, rather than refreshing a send's age at a delayed handler. A missed candidate budget is exposed as overrun/loss, not repaired by changing timestamps. Exact mapping of this evidence to draft-01's handoff timers requires review/qualification.

## 4. Invalidation, races and re-enable

Treat invalidation as a scoped operation, for example a request covering connection/vehicle/control generation and routine NAV/acquisition classes. Close producer admission for that scope and stop/invalidate host retry producers. The egress owner then applies a priority fence, discards matching waiting work and rejects later stale submissions. A response permit, if independently allowed by the profile, is a different token/class; revoking routine NAV neither grants arbitrary effects nor prevents an eligible bounded response automatically.

The fence's linearization boundary is its application on the serialized egress owner. A request arriving on another thread is not instantaneous cancellation of a call already being prepared/executed. Report these separately:

- **FENCE_APPLIED:** no further matching old-scope write initiation will be admitted after this owner boundary; waiting work was removed. Identify any active work and unresolved producers.
- **LOCAL_QUIESCENT:** matching local waiting/active operations and future eligible producers have settled. This does not mean kernel/network queues are empty or the FC has undone anything.
- **Remote reconciliation:** observed current state and any unresolved transmitted mode/target effect remain a separate authority concern.

New routine control requires explicit host enable/acquisition, new input readiness and the required local/remote reconciliation. Repeating invalidation is idempotent. Late callbacks update the old work record and cannot reopen a generation. A new connection object/port binding does not convert old incoming datagrams into proven fresh remote observations.

## 5. L1: Retrofitting the async UDP FIFO

The current [write chain](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/libmavconn/src/udp.cpp#L350-L419) borrows `tx_q.front()` storage and captures it by reference. Asio [requires buffer storage through completion](https://github.com/chriskohlhoff/asio/blob/231cb29bab30f82712fcd54faaea42424cc6e710/asio/include/asio/basic_datagram_socket.hpp#L644-L696). A credible retrofit needs more than metadata/erase:

1. Separate waiting records from a stable owning active-operation record. The completion handler retains that record, connection epoch and attempt ID. Waiting cancellation/replacement cannot invalidate active storage.
2. Add managed submission/result APIs carrying exact work metadata, bounded admission, latest NAV replacement and bounded discrete/response classes. Preserve the current coalesced send-chain kick rather than posting one unlimited task per update.
3. Evaluate the managed gate when selecting an operation and immediately before `async_send_to` initiation. Never drain an old FIFO automatically without current eligibility checks. There is still a downstream active-operation interval.
4. On a fence, discard matching waiting work, prevent old chain continuation and report the active operation separately. An optional per-operation cancellation request may accelerate local settlement; do not destroy its storage or publish terminal cancellation before the documented completion outcome. Modern [Asio per-operation cancellation](https://think-async.com/Asio/asio-1.36.0/doc/asio/reference/basic_datagram_socket/async_send_to/overload1.html) exists, but that reference version is not selected/installed evidence for this project.
5. Record one terminal local outcome per work/attempt. Full datagram success and local errors are distinct. An unexpected short successful datagram result is a transport anomaly, not permission to send its remaining bytes as another datagram. Do not transplant stream suffix-write logic into this contract.
6. Apply peer/receive changes in section 7, plus close/error paths that retain records until callbacks settle. A stopped/shared I/O context changes completion delivery; a port-close callback alone is not a per-work drain acknowledgement.

Socket-wide `cancel()` also cancels receives ([Asio API](https://think-async.com/Asio/asio-1.36.0/doc/asio/reference/basic_socket/cancel/overload1.html)); it is a coarse link action, not the normal routine-NAV revoke path. Waiting cancellation should keep observation reception alive. For an already initiated operation, any remaining uncertainty is reported; no fast settlement time is claimed. Extending the model to serial/TCP requires separate partial-frame/backpressure analysis.

## 6. L2: Smaller FIFO-bypass extension

Instead of teaching libmavconn a second policy scheduler, keep `ManagedEgress` in MAVMOOS and add an explicitly managed UDP connection mode with an immediate **nonblocking one-datagram attempt/result**. The controlled path never enqueues into the library's async FIFO. The host keeps exact-work association through the synchronous call and needs no opaque queued-message matching.

This is a source extension: UDP socket/send state is [private](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/libmavconn/include/mavconn/udp.hpp#L85-L103), so a stock external wrapper cannot simply call that path. Propose exclusive managed-TX mode from connection construction, rejecting legacy/raw bypass writes on that connection. Other legacy connections may retain their existing behavior. A live FIFO-to-managed transition would need a separate drain/fence protocol; it is outside this minimal route. Mixing the paths defeats the single controlled boundary.

The attempt returns full local acceptance, would-block, closed/no-peer or an explicit error. It does not retry, poll until writable, learn a new destination or silently treat a drop as success. The host applies the common fence/backpressure policy. [Asio's nonblocking setting](https://github.com/chriskohlhoff/asio/blob/231cb29bab30f82712fcd54faaea42424cc6e710/asio/include/asio/basic_socket.hpp#L1228-L1286) affects synchronous calls and does not itself turn async operations into this immediate path.

Packing is an explicit integration issue: the current [typed MsgBuffer constructor](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/libmavconn/include/mavconn/msgbuffer.hpp#L47-L79) finalizes using library TX/channel/signing state, whereas raw bytes/already packed messages do not get that work automatically. A bypass must expose preparation safely or deliberately use a separately owned codec state; it must not claim that raw `send_bytes` applies typed finalization. Recheck after preparation and avoid concurrent use of mutable channel/signing state. The exact public packing API remains open.

## 7. Peer, ingress and change footprint

Both L1 and L2 need a controlled peer mode. Always receive into a separate source endpoint; compare it with the pinned TX/allowed RX binding before admitting observations. Do not let ordinary reception mutate the governed destination. A configured initial remote currently does not supply this: [receive 309–346](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/libmavconn/src/udp.cpp#L309-L346) writes to `remote_ep` in the discovery path. Configured-peer or explicit learn-then-freeze admission would be new policy; the first design proposes configured-peer only. Binding is attribution, not authentication.

Capture one monotonic datagram acquisition time/source before parsing. Copy admitted packet bytes/framing results into an envelope with connection/peer epoch, datagram/frame identity and parse time, retaining FC times where available. Do not count legacy and new callbacks for the same frame as two observations. RX datagram-size/truncation and bounded batching need explicit limits; current storage is [frame-sized plus padding](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/libmavconn/include/mavconn/msgbuffer.hpp#L32-L38), with [UDP RX capacity](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/libmavconn/include/mavconn/udp.hpp#L97-L100) tied to it.

| Source area | L1 managed FIFO | L2 bypass |
| --- | --- | --- |
| `udp.hpp` / `udp.cpp` | Managed waiting/active records, gates/results, fence/close continuation, fixed peer and RX envelope | Exclusive managed mode, nonblocking attempt/result, fixed peer and RX envelope; no managed FIFO |
| `interface.hpp` / `interface.cpp` | Optional capability/result/receive hooks and safe codec ownership | Optional capability/receive/preparation hooks; scoped concrete API could avoid a generic virtual surface |
| `msgbuffer.hpp` or a new managed sidecar type | Preserve buffer utility; attach metadata through a stable owning record | Preserve/reuse or explicitly replace typed preparation; metadata stays in the host work record |
| `io_context_runner.hpp` interaction | Ensure shutdown/fences account for pending handlers and owned/shared execution | Same receive/close lifecycle concern; no managed async-TX completion queue |
| Serial/TCP source and generic ABI | Do not change/claim qualification for them in a UDP-only scope | Leave outside scope |

This is a change map, not a patch/line-count estimate. Adding fields/virtual APIs or altering class layout may require matched rebuilds; source opt-in behavior is not a promise of binary ABI compatibility. Existing [ament/C++20/dependency packaging](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/libmavconn/CMakeLists.txt#L10-L54) also remains to be assessed for the selected platform.
