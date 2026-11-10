# Paper review cases and future acceptance criteria

October 28, 2026. These are architecture walkthroughs and counterexamples, not executed tests. They extend the [24 draft-01 scenarios](../../contracts/2026-10-28-draft-01/review-scenarios.md). Proposed outcomes depend on adoption of this review's recommendations and completion of a concrete profile.

## Coherence and producer integration

| Case | Stimulus/counterexample | Proposed acceptance criterion |
| --- | --- | --- |
| P-01: split scalar generation | Iteration N is complete; N+1's counter and heading arrive while speed still belongs to N | Receiver cannot label the mixed cache as generation-verified. A strict record contains the complete N+1 vector or is not admitted |
| P-02: lost summary change | Summary 40 changes a value and is lost; 41 omits unchanged fields | Gap invalidates journal reconstruction. It cannot renew the state from 39 as if it were 41. Require a qualified full baseline |
| P-03: late registration | MOOSDB serves a retained delta or full record to a newly connected consumer | Registration does not establish an active producer binding, fresh age or executable request. A delta without its baseline remains incomplete |
| P-04: missing optional dimension | Summary contains `var=depth:varbalk`; selected profile needs depth | Preserve missingness and reject/invalidate for that profile. Never interpret the token as a zero-depth target |
| P-05: policy after summary | Solver report exists but mandatory-goal/completeness checks reject the decision | Strict producer outcome reports invalidity. A companion cannot infer final native validity from the summary alone without a separately qualified policy contract |
| P-06: same values, new solve | A genuine completed cycle N+1 produces values identical to N | New record can renew decision freshness within grant/window bounds; unchanged numeric equality is neither sufficient nor disqualifying evidence |

## Origin age and dependent observations

| Case | Stimulus/counterexample | Proposed acceptance criterion |
| --- | --- | --- |
| P-07: live relay, stopped producer | Relay repeats one canonical record every 50 ms after the producer stalls | Original identity/deadline never change; the record becomes unusable at expiry despite recent receipt. The illustrative 50 ms is not a profile default |
| P-08: old solve, new window | Solve begins under W1; W2 or a new authority epoch arrives before its result is serialized | Result keeps its original binding. A new window/epoch requires a new eligible decision begun under that binding; no post-completion relabeling |
| P-09: slow solve/path | Valid result first arrives after its fixed `accept_until` | Reject new admission; recent outer MOOS timestamp and remaining `use_until` do not override that limit |
| P-10: clock uncertainty | Source origin interval is `[200.10,200.14]`, allowed age is 0.50 s | Deadline is at most 200.60, using the oldest plausible origin. No use at 200.62 based on the later endpoint |
| P-11: no incoming mail | Producer and MOOS input both go quiet while a sender remains scheduled | Every effectful send rechecks monotonic deadlines; silence cannot freeze expiry. Required release/failure response has its own scheduled lateness bound |
| P-12: fresh solve, old position | New coherent decision was computed using an old dynamic observation, or an adapter now reads stale mode/altitude | Decision age does not certify observation age. Apply named dynamic-input/prerequisite requirements; stronger producer-input claims need provenance. Unknown age remains unknown |
| P-13: time-warp/reset | MOOS time pauses, jumps or changes rate while a real effectful permission exists | Unqualified MOOS time cannot extend monotonic permission. Invalidate any affected clock mapping and use the profile's loss policy |

## Invalidation, takeover, and egress

| Case | Stimulus/counterexample | Proposed acceptance criterion |
| --- | --- | --- |
| P-14: invalid before old expiry | Current producer declares all-stop/incomplete while last valid snapshot has remaining time | Remove its eligibility at the ordered invalidation boundary; do not transmit numeric zeros as an implicit universal fallback |
| P-15: delayed old primary | Standby has acquired a new bridge epoch; primary resumes with a valid-looking decision, then its all-stop | Old-epoch output cannot become current or revoke the standby's grant. Record residual already-sent work separately |
| P-16: revoke after enqueue | Target passes admission, enters a library queue, then its grant is revoked | Controllable dequeue/write gate rejects it. If it already escaped the final gate, report the residual rather than claiming recall |
| P-17: delayed downstream send | Local handoff occurs before expiry but delivery can occur after it | Local-age claim remains local. A claimed FC-receipt bound requires a demonstrated post-gate delay and earlier handoff cutoff |
| P-18: preparation consumes early | FC accepts a preparation target before mode confirmation is observed | Preparation permission/target must explicitly allow that effect. Intent cannot self-declare ACTIVE, and no universal zero/hold target is assumed |

## Session and recovery boundaries

| Case | Stimulus/counterexample | Proposed acceptance criterion |
| --- | --- | --- |
| P-19: engine restart, same process | Helm engine restarts and iteration count returns to its beginning | New producer incarnation/binding; app name or PID reuse cannot make old and new iterations one stream |
| P-20: target/endpoint ambiguity | A reconnect uses the expected system ID but continuity evidence is incomplete | Conservatively create a new vehicle session under the selected profile, expose evidence strength, and reacquire permission |
| P-21: crash around command handoff | Action may reach FC, bridge crashes before durable result evidence | New bridge rejects retained old requests; transaction effect remains uncertain. No generic automatic retry based on absent success record |
| P-22: origin/reference reset | Same physical vehicle changes home/local origin while a relative target remains cached | Invalidate affected reference-bound observations/intent. Retained numeric equality does not preserve meaning |

## What future validation must record

Later implementation validation should identify exact producer/bridge/library/FC revisions, profile and trust configuration, injected event order, original IDs/windows/epochs, observed timestamps at named boundaries, expected versus observed eligibility, residual work and coverage limits. Use deterministic source-level checks for generation/ledger behavior and controlled timing/failure experiments for scheduler/transport bounds. Simulation and hardware evidence must remain distinct.

This review only checks whether the proposed rules give a coherent answer to each paper case. No timing values, vehicle response, implementation coverage, or acceptance tests have been demonstrated.
