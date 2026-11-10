# Origin freshness: Q-SH-03

October 28, 2026. Proposed semantic mechanism and paper reasoning. No exchange, clock calibration, watchdog, scheduler, or vehicle behavior was implemented or measured. This develops the [draft shared model](../../contracts/2026-10-28-draft-01/shared-model.md) without replacing it.

## 1. Five ages that must remain distinct

| Age or interval | What it answers | What it does not establish |
| --- | --- | --- |
| Measurement age | How old is the physical/state observation used for a declared purpose? | That a new decision was made or the bridge still has permission |
| Decision origin age | How long since the designated input-capture/decision start? | That all dynamic inputs were fresh, or that computation completed promptly |
| Decision completion/processing delay | How long did solving, adaptation and bridge processing take? | A substitute origin timestamp for an already old-started solve |
| Relay/bridge receipt age | How long since this receiver saw the packet? | How long the producer or an intermediary held it |
| Queue/transit/execution age | How long between selection, handoff, FC receipt and continued use? | Covered by a local bridge deadline without a bounded downstream path |

A live relay, a responsive MOOSDB, a new heartbeat and an active setpoint sender establish different kinds of liveness. None alone establishes a new eligible decision. The earlier [BlueBoat broker trace](../2026-10-28-operating-models/trace-03-continuous-setpoints.md) gives a concrete conditional example: cached fields can be forwarded repeatedly and renew a receiver's receipt-based watchdog after the original producer stops. It is a source-derived scenario, not a reported deployment incident.

Core MOOS retains message timestamps and serves existing values on registration. However, a new ordinary `Notify` without an explicit original timestamp constructs a new MOOS time. Thus the transport can carry time metadata, while a particular forwarding path may lose its original meaning. An opaque canonical envelope can preserve origin evidence independently of that outer transport timestamp. [E5](evidence.md#e5-core-moos-messages-mail-and-retention).

## 2. Recommendation and its premises

For the first proposed effectful helm profile, prefer a **bridge-issued, fixed freshness window** consumed by a participating trusted producer before a new decision starts. Use it when no source-to-bridge clock bound has been demonstrated. Preserve bounded source-clock mapping as an alternative, especially where measurement timestamps and multiple consumers need it.

This recommendation avoids making clock synchronization a prerequisite for bounding decision origin age. It does require a producer integration and a qualified exchange. It does not authenticate that producer, prove its solver ran, establish sensor age, guarantee delivery, or force the FC to stop using its last target. Those remain independent contracts.

Treat the designated origin as the decision's **input-capture/solve start**, or another explicitly more conservative event. Stamping only after a long solve gives the result a new apparent lifetime while hiding computation age. If the solver incrementally captures dynamic inputs, the profile must state the relevant earliest input/capture bounds and allowable skew. Static mission configuration need not be refreshed like a dynamic position sample, but its revision must remain identified.

## 3. Candidate fixed-window exchange

The following are proposed obligations, not a chosen wire protocol:

1. The bridge establishes a permitted producer/session/ingress, vehicle session, authority epoch/grant and profile revision under [Q-SH-01](boundary-and-recovery.md#1-trusted-ingress-q-sh-01). It issues window W at bridge monotonic time `w0`, with fixed `accept_until = a` and `use_until = u`, where `w0 <= a <= u`. Store the authoritative deadlines at the issuing bridge.
2. W is unique within the bridge epoch and bound to those identities/scopes. It is not a bearer permission. A producer claiming W still needs current authorization and the expected binding.
3. The original producer receives and binds W **before** the eligible decision starts. Its decision identity, input/configuration context and authority binding are fixed for that cycle. A relay cannot add W to cached output. A producer cannot stamp an old result with W or take a solve started under an old grant and rebind it after completion.
4. The producer emits a complete outcome, carrying W and its immutable decision identity/body. Numerical values can equal the previous decision. This is legitimate renewal only when a new cycle satisfying the producer obligations actually occurred.
5. For a new decision, the bridge validates origin/bindings/order/body and receives it by `a`. If admitted, its effective local deadline is no later than `u`, the current grant/ingress/session limits, producer validity that can be interpreted conservatively, and any tighter profile bound. Receipt does not add another full lifetime.
6. Retransmission preserves the same decision/window/body. A duplicate does not alter deadlines. A conflicting body for an existing identity is rejected. Out-of-order older objects cannot replace current intent. Later transport activity cannot revive an expired object.
7. Window renewal is an explicit new issuance. The producer must start a new eligible decision under the new window. Receipt of a new window by a healthy relay is not sufficient. Grant renewal remains an independent bridge policy event.
8. Restart, revoked binding, ambiguous producer incarnation, target/session change, or authority transfer invalidates the affected windows. Reacquisition requires a new binding and a new decision. A bridge monotonic deadline is never compared directly in another process's or another bridge's clock domain.

More than one decision may use W if the profile allows it. Each has a genuine increasing decision sequence and shares the fixed outer expiry. Overlapping windows can avoid an availability gap, but they do not permit an old-window object to become current after a newer decision, nor extend the old object's lifetime. This need not add one round trip per solver cycle, but issuing/receiving windows still consumes bandwidth and timing budget. If round-trip/computation delay exceeds the permitted age, the profile cannot remain available by extending old decisions. Resource bounds, outstanding-window limits, invalidation ordering and sequence wrap are still exchange-design work.

### Why this bounds decision age

Let `s` be the real decision-start instant expressed in the bridge's monotonic domain. Under the trusted causal producer obligation, `s >= w0`: the producer cannot bind a window that had not been issued. For any admitted use at bridge time `t <= u`, the decision age satisfies:

`t - s <= t - w0 <= u - w0`.

Therefore select windows such that `u - w0 <= A_use`, the profile's maximum decision-origin age at the named local use boundary. Likewise, `a - w0 <= A_admit` bounds age at new admission. Network and computation delay consume this budget; they do not restart it. This is an upper bound, not an estimate of the actual start time. It depends on trusted producer behavior, causal exchange, correct session binding and monotonic receiver time. A dishonest or incorrectly implemented producer can violate the premise.

A nonce by itself proves no solver provenance. Unpredictability may help an authenticated challenge protocol resist preplay, but a publicly readable window ID is not authentication and must not be described as such. The initial deployment trust model and later exchange design must state what they actually establish.

## 4. Bounded source-clock alternative

Suppose a qualified mapping places the decision origin in bridge monotonic interval `[L, U]`, including offset uncertainty, drift, timestamp quantization and any domain conversion. The oldest plausible origin is L. For maximum permitted age A, require:

`t <= L + A`.

Using `U + A`, or ignoring uncertainty because clocks appear synchronized, can accept a result older than A. Reject inconsistent/future timestamps outside the mapping's allowed uncertainty, and invalidate the mapping on the discontinuities defined by its session/reset model. Do not translate one timestamp using a different producer or vehicle boot incarnation's mapping.

The mapping must establish a finite error bound, not only an offset estimate. “Same machine,” MOOS skew checks, time synchronization being enabled, or plausible UTC numbers do not by themselves demonstrate that bound. MOOS time and time-warp are useful for logs/simulation; a real effectful profile needs a qualified conversion to its local monotonic deadline domain. Simulation-time expiry would be a separate explicitly bounded simulation profile, not an implicit replacement of real-time guards.

If both a source bound and W apply, take the earliest deadline. The summary's two-decimal `utc_time` and ordinary scalar notification times require their own provenance/precision treatment; do not regard either as a ready-made calibrated decision-age interface. [E2](evidence.md#e2-summary-encoding-and-parsing), [E6](evidence.md#e6-cycle-time-provenance).

## 5. Budget through the actual use boundary

At a bridge's last controllable egress gate, a candidate bound is:

`D_gate = min(D_origin, D_window, D_producer, D_grant, D_ingress, D_profile, D_operation)`.

Use only bounds with a qualified common interpretation. An uncalibrated producer `valid_for` cannot become `receipt_time + valid_for` without losing original-age semantics. The bridge can conservatively cap a request by its issued window and policy; it cannot manufacture the missing source clock.

Re-evaluate eligibility against current authority, session, prerequisite observations and `D_gate` at the gate, not only at initial admission. Cache replacement must be atomic at the semantic snapshot level. Egress work retains its original identity/deadline so old queued targets cannot reappear after a new intent or authority epoch.

If the required claim is age at FC receipt, and the profile has demonstrated a maximum remaining queue/transit delay `L_post`, the latest handoff deadline must be no later than `D_fc - L_post`. With no such bound, report only age at the named local gate. A closed link or successful library enqueue is not proof that previously queued/retried work cannot arrive later. [Q-AUTH-04 analysis](boundary-and-recovery.md#4-egress-and-residual-work-q-auth-04).

Repeated setpoint transmissions can execute the same still-eligible intent at the profile cadence. They preserve its expiry and decision identity even if the wire protocol requires a current transmission timestamp. Once ineligible, ordinary streaming stops being authorized under that snapshot and the profile's explicit failure/release action applies. The FC's independent timeout or target-hold behavior is additional release/configuration-specific evidence, not an implication of bridge expiry.

For an observation-dependent adapter, check every required dynamic parent at the gate. Without prediction or another qualified model, a bound measured at decision creation ages further during solving/queueing. Derived data uses conservative parent ages and allowed skew. A new decision using a cached position does not refresh that position. A bridge's check of current mode/position also does not establish the freshness of every observation previously used inside a behavior; claiming that stronger property requires producer input provenance and behavior-specific requirements.

## 6. Numerical paper examples

All numbers below illustrate arithmetic only. They are not recommended timeout/rate defaults.

| Case | Given | Required interpretation |
| --- | --- | --- |
| Fixed window | `w0=100.00`, `a=100.30`, `u=100.50`; decision starts at 100.12, arrives 100.25 | At 100.45 the actual origin age is 0.33 s and the receiver-established upper bound is 0.45 s. Both are below the illustrative 0.50 s window budget |
| Relay repeat | The same object arrives again at 100.44 | Expiry remains 100.50. It does not become 100.94 or acquire a new sequence |
| Slow solve/path | A result under W first arrives at 100.31 | Reject new admission after 100.30 even though W's use deadline has not yet passed |
| New decision, unchanged numbers | A real later cycle under a permitted window emits sequence N+1 with the same course/speed | It can become a new intent after all checks. A relay merely assigning N+1 cannot |
| Bounded clock | Origin maps to `[200.10,200.14]`, `A=0.50` | Conservative deadline is 200.60. A use at 200.62 fails even though `U+A=200.64` would appear to permit it |
| Remaining transit | Required FC-receipt deadline 300.50 and demonstrated `L_post=0.08` | Handoff must occur by 300.42. If `L_post` is unknown, the FC-receipt claim is unestablished |

At an exact deadline the eventual protocol must choose an inclusive/exclusive comparison and conservative handling of timer precision. The examples do not supply scheduling margin. Production bounds need explicit clock/timer uncertainty, processing overruns, downstream delay and miss-response allowances.

## 7. Liveness and expiry without fresh mail

Expiry evaluation cannot depend only on receiving another packet. At every effectful send, current monotonic time is compared to fixed deadlines. A separate deadline-driven authority response is needed where a profile requires action even when the normal send loop is idle. Requested AppTick/CommsTick rates do not prove a maximum scheduling delay; later implementation must establish response lateness and overload behavior.

Stopping a test's simulated clock must not silently freeze real vehicle permissions. A MOOSDB reconnect must not interpret a retained canonical envelope or an old `IVPHELM_STATE` as new work. Observation-only readers can still display those objects with historical/unknown/stale status.

## 8. Disposition

**Proposed choice:** fixed receiver windows for an initial strict decision path, with a producer obligation anchored before decision start; bounded clock mappings remain a qualified alternative. Preserve immutable origin evidence through every relay and use the earliest applicable deadline.

**Q-SH-03 remains open:** adoption, exchange schema, trust enforcement, maximum age/uncertainty, per-profile numeric budgets, producer integration, reset/wrap behavior and timing feasibility. The architecture now states what would make the mechanism sufficient and what it cannot establish. It does not infer measurement freshness, control safety, or FC behavior from the decision-age proof.
