# Command-transaction contract

Status: proposed, draft-01, October 28, 2026. Depends on the [shared model](shared-model.md), [observations](observation.md), and [authority operation scopes](authority.md#2-operation-scopes). Continuous setpoints use the navigation-intent contract rather than one transaction per transmitted packet.

## 1. Purpose and source basis

The [command trace](../../research/2026-10-28-operating-models/trace-02-acknowledged-command.md) distinguishes bridge admission, local send, protocol ACK, observed state, and completion. It also exposes retained MOOS requests, late ACK ambiguity, and operation-specific service behavior. This draft makes those boundaries explicit.

**CMD-1:** every canonical effectful request is a named operation with explicit identity, target, expiry, required authorization, and declared result semantics. Publishing a requested mode or a retained boolean cannot directly declare the operation admitted or completed.

## 2. Request and transaction identity

| Request field | Draft meaning |
| --- | --- |
| Issuer | `producer_id`, `producer_session`, validated `ingress_id` |
| Destination | `bridge_epoch`, `vehicle_id`, `vehicle_session`, component/endpoint scope if relevant |
| Request identity | Issuer-supplied `request_id`, unique within its producer session |
| Operation | Versioned operation/profile and arguments with explicit units/frames/references |
| Freshness | Bounded origin-age evidence or receiver-issued freshness window |
| Requested limits | Dispatch/result deadlines or bounded durations requested by issuer; bridge derives/enforces conservative local deadlines |
| Conflict policy | Explicit supersession/preemption request, if the operation supports it; no implicit cancellation by writing another value |
| Goal | Operation-declared acceptance/observation/completion predicates; callers cannot arbitrarily weaken the profile's required evidence |

The transaction key is `(producer_id, producer_session, request_id)` with immutable destination/operation/body binding. The bridge epoch remains in the request body and event identity. A new bridge cannot silently turn a retained old-epoch request into a fresh request.

**CMD-2:** an exact duplicate key/body returns the known admission/result/history without re-dispatch. A conflicting body under the same key is rejected as `REQUEST_ID_CONFLICT`. Canonical semantic comparison and ledger lifetime must be specified; a hash is an implementation aid, not a substitute for collision/conflict handling. [Q-CMD-01](open-questions.md#command-questions).

Duplicate handling is bounded by declared session/ledger guarantees. If the ledger is lost or compacted, old messages must be inadmissible through session/ingress invalidation. This proposal does not promise exactly-once physical action across process failure, unknown remote outcome, or unbounded storage loss.

## 3. Admission, dispatch, and deadlines

The proposed order is: validate schema → bind issuer/target/session → check duplicate/conflict ledger → check origin freshness for new executable work → evaluate operation permission/current prerequisites → record admission → schedule → recheck current target/permission/deadlines at egress → attempt protocol operation. Independently authorized history retrieval can return a known result after its dispatch window expires; it never refreshes or re-executes the request. An old binding that is no longer trusted needs a separately authorized query path to historical records.

Admission checks and the relevant ledger/grant revisions need serialized decision semantics. Two simultaneous duplicates must not both become first dispatches. Operation classes may run concurrently only where their profiles define compatible effects and wire correlation. [Q-CMD-02](open-questions.md#command-questions).

**CMD-3:** three deadlines are distinct: latest admission, latest permitted dispatch/retry, and latest result/observation wait. The bridge derives admission cutoff from the applicable origin/ingress/window and operation bounds; a caller cannot reset it through a re-notification. Expiry before dispatch prevents the effect and records a definite local expiry. Expiry after possible dispatch reports remote-outcome uncertainty where appropriate. Waiting for an observation may continue after the dispatch deadline; sending more work may not.

An ACK timeout does not prove the FC did nothing. A retry remains the same logical transaction/identity and is permitted only by operation-specific retry/duplicate rules, current permission, remaining dispatch budget, and protocol correlation. A caller must not create a new request ID solely to bypass an uncertain old transaction.

## 4. Result dimensions and lifecycle

A single success flag cannot represent the result. Candidate dimensions are:

| Dimension | Proposed states |
| --- | --- |
| Admission | `UNREVIEWED`, `REJECTED`, `ADMITTED` |
| Dispatch | `NOT_SENT`, `QUEUED`, `HANDED_OFF`, `UNCERTAIN` |
| Protocol | `NOT_APPLICABLE`, `NOT_OBSERVED`, `PENDING`, `IN_PROGRESS`, `ACCEPTED`, `REJECTED`, `TIMED_OUT`, `UNKNOWN` |
| Observation | `NOT_REQUIRED`, `WAITING`, `SATISFIED`, `EXPIRED`, `CONTRADICTED`, `UNKNOWN` |
| Task | `NOT_REQUIRED`, `RUNNING`, `COMPLETED`, `FAILED`, `CANCELLED`, `UNKNOWN` |
| Lifecycle | `OPEN`, `SETTLED`, with a settlement reason and evidence |

These dimensions need a constrained combination table in the final operation specification; they are not an unrestricted Cartesian product. For example, a locally rejected, never-dispatched request cannot claim protocol acceptance caused by that request. A profile can record a pre-existing satisfied predicate as a no-op without asserting that it sent anything.

**CMD-4:** the operation profile declares what ends its transaction. A send-only API can settle as local handoff evidence without claiming ACK or completion. An ACK-based operation can settle on protocol result while observed state remains a separate requirement/status. A mission/task operation may need later progress and completion predicates. If the requested guarantee is unavailable, reject it or explicitly negotiate the weaker guarantee before dispatch.

An operation's completion predicate must state observation identity, validity/freshness, timing floor, duration/sample coverage if relevant, and what is being claimed. Observed state does not prove which actor caused it. Re-evaluating a cached boolean for a dwell interval cannot stand for newly sampled evidence throughout that interval. [Q-CMD-03](open-questions.md#command-questions).

## 5. Events, history, and late evidence

Each transaction event carries the key/body binding, bridge epoch, target session, event sequence, as-of time, changed dimensions, reason code, raw protocol/library evidence where available, and observation references. Per-transaction event sequence is monotonic; missing events can be detected. Global ordering across unrelated transactions is not assumed.

**CMD-5:** a current-state projection is supplemented by a bounded history/query contract. Late subscribers must be able to distinguish a latest snapshot from a full event history and learn the retention limit. Results are published in causal order for each transaction, including admission before later effects. [Q-CMD-04](open-questions.md#command-questions).

Settling a transaction prevents further dispatch under it. A late ACK or observation is appended as supplemental evidence referencing the settled record; it does not silently reopen the transaction or re-acquire authority. A settled unknown outcome remains part of the audit history, even if subsequent fresh evidence clarifies the current vehicle state. Cross-session late evidence cannot confirm a different vehicle session.

## 6. Wire correlation, cancellation, and restart

Application request IDs may not be echoed by an autopilot. The adapter must document its correlation key, expected ACK source/target, concurrent-operation limits, retry behavior, and ambiguity window for a repeated wire command. Serialize or reject potentially ambiguous operations when stronger attribution is unavailable; report residual ambiguity honestly. [Q-CMD-02](open-questions.md#command-questions).

Cancellation is a separately identified request referring to the original transaction. Before dispatch it can prevent work; after dispatch it may only request a supported cancellation action or stop future retries. It does not undo an already completed effect. A successful local cancellation request cannot assert that the FC stopped an operation without its required evidence.

**CMD-6:** after bridge restart or vehicle-session discontinuity, reconcile possible in-flight effects before new conflicting operations. Do not replay old queued work automatically. An implementation without a durable transaction ledger must invalidate old ingress bindings and expose the uncertain recovery context. Exact restart retention/reconciliation is open under [Q-SH-04](open-questions.md#shared-questions).

Authority-changing commands are serialized with the authority state machine. Raw mode/mission/actuator interfaces cannot bypass its grant generation and egress checks merely by using a different plugin or SDK API. Read-only operations remain separately usable when their profiles allow them.

## 7. MOOS compatibility and an example

Candidate channels could carry request envelopes, event records, latest transaction snapshots, and history queries. Names and encoding are unresolved. A compatibility adapter for `ARM_UAV=true` needs an active operator/issuer association, explicit freshness/replay semantics, and a way to return correlation. This draft recommends keeping bare retained side-effecting booleans outside the canonical executable interface until that adapter is qualified. [Q-CMD-05](open-questions.md#command-questions).

Illustrative semantic request, not a wire schema or authorization to arm:

```yaml
contract_revision: draft-01
producer_id: operator-console
producer_session: OP9
ingress_id: I4
request_id: R31
bridge_epoch: B7
vehicle_id: vehicle-2
vehicle_session: S4
operation: arm
operation_profile: arm-profile-A
arguments: {armed: true}
freshness_window_id: W24
```

If admitted, an event may record `admission=ADMITTED`, `dispatch=NOT_SENT`, and `protocol=NOT_OBSERVED`. After possible dispatch with lost ACK, the transaction may settle with `dispatch=UNCERTAIN`, `protocol=TIMED_OUT`, `observation=UNKNOWN`, and a reason explaining the missing evidence. Resending R31 must retrieve that result rather than create another arm action. A fresh armed-state observation is reported separately and cannot by itself attribute the action to R31.

## 8. Review obligations and questions

Cases include retained requests, simultaneous duplicates, conflicting same-ID bodies, result gaps, late ACK after timeout, IN_PROGRESS/final sequences, uncertain cancellation, repeated wire-command ambiguity, deadline expiry in a queue, process crash after sending, and external mode change before an authority-sensitive command leaves egress. See [review scenarios](review-scenarios.md).

[Q-CMD-01 through Q-CMD-05](open-questions.md#command-questions) track identity/ledger, correlation/concurrency, operation completion, event retention, and compatibility. No operation profile or protocol library is selected or validated by this draft.
