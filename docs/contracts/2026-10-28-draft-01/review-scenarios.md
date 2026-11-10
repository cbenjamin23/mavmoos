# Cross-contract review scenarios

Status: proposed, draft-01, October 28, 2026. These are paper walkthroughs and future acceptance criteria. No tests, simulator runs, timings, or vehicle operations were executed to establish them.

## 1. Shared example identities

Use bridge epoch B7, vehicle `vehicle-2` session S4, helm producer/session `helm-primary/H9` with ingress I3 and freshness window W23, authority epoch A12, and grant G12. The independent operator issuer `operator-console/OP9` uses ingress I4 and window W24 for command R31. These labels match the illustrative records in the contracts. They carry no authority by themselves; the bridge owns the validated associations and deadlines.

All times below are logical ordering statements. No numeric timeout, stream rate, or fallback profile is implied. A scenario requiring an unresolved mechanism stays a review obligation, not a demonstrated guarantee.

## 2. Normal acquisition with an FC-required preparation stream

1. Current observations identify S4, an applicable profile, and the mode/estimator/state evidence required by that profile. Unknown measurement ages remain unknown; the profile decides whether the available evidence is sufficient.
2. An explicit acquisition transaction from the permitted issuer is admitted. The bridge creates A12/G12 in `ACQUIRING`, with a fixed transition deadline. Routine navigation remains prohibited.
3. The original helm producer completes decision 18 under current origin-freshness evidence. It sends a complete snapshot bound to B7/S4/H9/A12/G12. The bridge admits the snapshot only for the scope currently permitted.
4. A qualified profile establishes its preparation target/sequence and finite permission. The bridge starts only that preparation scope and issues the mode-entry transaction in the profile's required order.
5. A protocol acceptance event is recorded. Fresh FC observations then satisfy the separate confirmation predicate; a cached boolean repeatedly read during a dwell does not suffice.
6. With the intent still valid, no contradictory external evidence, and current scheduling/egress guards, the bridge atomically enters `ACTIVE` and enables its active scope. The preparation-to-active target transition follows the profile.
7. A repeated packet carrying sequence 18 cannot extend its use deadline. A newly completed sequence 19 may replace it within the existing grant. Grant expiry still needs a separate explicit renewal event or reacquisition.

Review failure: if no qualified preparation specification or no coherent/fresh candidate is available, the sequence cannot progress to preparation transmission or `ACTIVE`. This prevents both a circular entry requirement and an unbounded pre-entry stream. See [authority acquisition](authority.md#4-acquisition-and-release).

## 3. A command whose ACK is lost

An operator request R31 passes identity, expiry, duplicate, authorization, and prerequisite checks. The bridge records admission and attempts dispatch. The FC may act, but its ACK does not arrive in time.

The transaction settles with the available dispatch evidence and an explicit uncertain remote outcome. No generic “definitely failed” or “safe to retry with a new ID” conclusion follows. An exact duplicate R31 retrieves the ledger state rather than causing a second dispatch. A fresh later armed-state observation can clarify current state but cannot prove which caller caused it.

An old ACK arriving while a new same-number wire command is pending requires the operation's ambiguity policy. If the selected library cannot distinguish them, the bridge must serialize beyond that ambiguity window, reconcile, reject the stronger attribution guarantee, or apply another justified profile policy. Application UUIDs alone do not solve a protocol that does not echo them. See [command correlation](command-transaction.md#6-wire-correlation-cancellation-and-restart).

## 4. A live relay with a dead helm

The original producer stops after decision 18. A broker keeps forwarding the cached payload, and the downstream scheduler remains alive. New receipt/publication timestamps describe forwarding activity only; they cannot extend decision 18's origin bound, G12's permission, or W23's fixed deadline.

When eligibility ends, routine use of decision 18 stops at the specified local gate. The profile handles expiry/producer loss and any allowed bounded response. A fresh-looking FC liveness stream cannot replace upstream decision evidence. Reconnection or a new process creating H10 does not revive H9/A12 work; a new validated association and authority epoch are needed. See [intent renewal](navigation-intent.md#3-admission-replacement-and-renewal).

## 5. Scenario matrix

| ID | Trigger | Required outcome if this draft is adopted | Main open dependency |
| --- | --- | --- | --- |
| R-01 | Valid observation contains zero latitude, longitude, speed, or altitude | Evaluate quantity validity without a generic zero-invalid rule; preserve its actual reference/quality | Q-OBS-01, Q-OBS-02 |
| R-02 | Cached observation is re-notified with current publication time | Preserve original timing/provenance; do not upgrade unknown/stale measurement age | Q-OBS-01 |
| R-03 | Fresh position is combined with stale/unknown-age attitude | Derived record exposes parent age/skew; required control eligibility fails rather than becoming fresh by recomputation | Q-OBS-03 |
| R-04 | Origin/home/estimator context changes | Old-reference values remain historical; dependent observations/intents cannot be used until the new reference is resolved | Q-OBS-02, Q-NAV-04 |
| R-05 | A retained command from old bridge epoch B6 is delivered to B7 on registration | Reject old binding before side effects; do not synthesize a new request/session from receipt | Q-SH-01, Q-SH-04 |
| R-06 | Two identical requests race; another same-key request changes arguments | Admit at most one local dispatch transaction; duplicates query it; conflicting body is rejected | Q-CMD-01 |
| R-07 | ACK arrives after timeout or repeats while a same-number operation is considered | Preserve uncertainty and apply documented correlation/ambiguity policy; no silent attribution to new work | Q-CMD-02 |
| R-08 | Command reports IN_PROGRESS, then a final result | Preserve progress/final evidence according to operation semantics; acceptance and task completion stay distinct | Q-CMD-03 |
| R-09 | Late result subscriber has only a latest snapshot | Advertise snapshot/history scope and retention; permit gap detection/query within declared limits | Q-CMD-04 |
| R-10 | Helm renews unchanged valid quantities with a new completed decision | New sequence can replace intent using fresh origin evidence, bounded by the current grant | Q-NAV-01, Q-NAV-03 |
| R-11 | Relay repeats the same sequence or attaches its own new receive timestamp | Duplicate does not extend original intent/window/grant expiry | Q-SH-03 |
| R-12 | Heading arrives from one cycle, speed/altitude from another | No partial canonical replacement; arrival grouping cannot claim strict coherent intent | Q-NAV-03 |
| R-13 | Heartbeat remains live while solver stalls or a valid explicit all-stop arrives | Distinguish heartbeat from decision validity; explicit invalidation removes last-good eligibility without inventing a zero/disarm fallback | Q-NAV-02, Q-AUTH-06 |
| R-14 | Standby/tardy-primary decisions overlap | Follow explicit source/session policy; takeover uses a new authority epoch and old producer work stays ineligible | Q-AUTH-05 |
| R-15 | External mode/RC/failsafe change occurs between mode ACK and confirmation | Do not activate from the ACK alone; invalidate incompatible permissions and avoid automatic recapture | Q-AUTH-02, Q-AUTH-06 |
| R-16 | Mode boolean stays true in a cache without new usable evidence | It cannot satisfy a fresh-sample confirmation requirement merely by repeated dwell evaluation | Q-CMD-03, Q-OBS-01 |
| R-17 | A navigation grant is revoked with queued and in-flight packets | Gate new enqueue/dequeue work, invalidate controllable entries/retries, and report residual uncertainty; no claim to recall sent bytes | Q-AUTH-04 |
| R-18 | A blocking transaction or bulk transfer threatens the allowed stream gap | Enforce the selected scheduling/resource policy or declare its guarantee unavailable; no fresh-intent exemption from timing limits | Q-NAV-05, Q-AUTH-01 |
| R-19 | Bridge restarts or FC-link continuity is lost after possible discrete effects | Invalidate old bindings/work, expose recovery uncertainty, reconcile current state, then require new acquisition for navigation | Q-SH-04, Q-AUTH-02 |
| R-20 | MOOSDB is unavailable while FC transport remains healthy | Loss of valid upstream intent/permission is handled independently of link liveness; only the declared bounded response may proceed | Q-SH-03, Q-AUTH-06 |
| R-21 | Query arrives while navigation is inhibited | Permit it only under its independent read-only target/trust/resource policy; no helm grant is manufactured or required by default | Q-AUTH-01 |
| R-22 | Configuration/parameter change invalidates an active profile assumption | Serialize conflict handling, revise/invalidate eligibility as declared, and do not keep claiming the old profile guarantee | Q-AUTH-01, Q-NAV-04 |
| R-23 | Producer restarts, sequence wraps, or a time mapping jumps | Apply explicit new-session/order/clock rules; old source/intent evidence cannot become new merely through reset values | Q-SH-02, Q-SH-03 |
| R-24 | Request or intent expires while waiting in an SDK/transport queue | No later eligible dispatch under that object; account explicitly for queue points the selected library cannot control | Q-NAV-05, Q-AUTH-04 |

## 6. Review completion criteria

For a chosen future profile, each applicable scenario needs a declared state/result outcome, evidence source, time/identity basis, egress boundary, and observable reason/history. Unresolved mechanisms must remain marked rather than filled with assumed behavior. Later implementation validation should turn those criteria into meaningful source, simulation, timing, and hardware evidence as appropriate; the present document supplies none of that runtime evidence.
