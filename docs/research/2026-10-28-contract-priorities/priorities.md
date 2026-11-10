# Prioritization of the 25 open questions

October 28, 2026. Recommendations, not accepted decisions. The original [25-question register](../../contracts/2026-10-28-draft-01/open-questions.md) is unchanged.

## How the order was chosen

Rank by how strongly a choice constrains other contracts, whether an incorrect assumption can admit old/mixed/wrong-source effects, and whether source investigation can narrow the question now. The user's focus gives continuous helm navigation the first review slot. This is a dependency/work order, not a numerical risk score or a claim that a lower-ranked command issue is less consequential.

Some dependencies must be worked together. Ranks 1–3 establish what a decision is, when it was produced, and which producer is trusted to assert that. Ranks 4–8 ask whether those properties survive invalidation, takeover, queueing, disconnect, and restart. They form one admission boundary. No effectful feature can bypass a lower-ranked prerequisite merely because it appears later in this queue.

**Focused** below means a substantive source/reasoning pass in this package with a proposed recommendation. **Queued** means ranked and scoped, with no claim of equivalent investigation. All 25 remain open for the remaining evidence/adoption listed.

## First: establish decision and admission legitimacy

| Rank | Question | Why here / main dependencies | This pass | Evidence or choice still needed to close it |
| --- | --- | --- | --- | --- |
| 1 | **Q-NAV-03: coherent completed helm generation** | Every navigation snapshot depends on distinguishing one decision from a collection of recent scalars. Jointly with ranks 2–3 | Focused: [helm output](helm-output.md) | Adopt producer-envelope or qualified alternative; specify integration and all outcome paths; later validate generation isolation |
| 2 | **Q-SH-03: original age mechanism** | A coherent cached decision can still be old. Establish the origin bound before choosing watchdogs | Focused: [freshness](freshness.md) | Adopt clock/window mechanism, trusted producer obligations, clock/reset rules, and profile age budgets; later measure feasibility |
| 3 | **Q-SH-01: trusted producer/session/ingress** | Neither source names nor window possession proves who made a decision. Required by ranks 1–2 | Focused: [trust boundary](boundary-and-recovery.md#1-trusted-ingress-q-sh-01) | Adopt deployment trust assumptions, issuer/path binding, enforcement and revocation mechanism; review each relay |
| 4 | **Q-NAV-02: gaps, all-stop, invalidation** | Determines when the old eligible decision must cease to be usable. Depends on identifiable outcomes | Focused: [invalidity](boundary-and-recovery.md#2-gaps-and-invalidation-q-nav-02) | Adopt per-profile gap/failure response and deadlines, including explicit invalid outcomes; qualify physical response later |
| 5 | **Q-AUTH-05: primary/standby/operator arbitration** | Coherent fresh output from the wrong leader is still ineligible. Depends on ranks 1–4 | Focused: [takeover](boundary-and-recovery.md#3-standby-and-operator-arbitration-q-auth-05) | Adopt priority/renewal policy and serialized transfer rules; validate delayed-primary and restart overlap |
| 6 | **Q-AUTH-04: queue/retry residuals** | Revocation is ineffective if old work keeps leaving downstream queues. Constrains all handoff/deadline promises | Focused: [egress](boundary-and-recovery.md#4-egress-and-residual-work-q-auth-04) | Select/inspect library and transport paths, identify last gate, bound residual work and acceptable interval; measure under load |
| 7 | **Q-SH-02: target identity/session continuity** | Prevents valid work being reused for a different or ambiguous vehicle incarnation. Required throughout | Focused: [vehicle sessions](boundary-and-recovery.md#5-vehicle-identity-and-continuity-q-sh-02) | Select provisioned identity/evidence model per FC, reconnect/reset criteria, and evidence-strength reporting |
| 8 | **Q-SH-04: restart ledger/uncertain effects** | Session invalidation cannot undo a command that may already have executed. Depends on target and egress boundaries | Focused: [recovery](boundary-and-recovery.md#6-restart-and-uncertain-effects-q-sh-04) | Adopt crash/durability/retention model, persistence ordering and reconciliation rules; later fault-inject at handoff boundaries |

This order does not imply that trust or vehicle binding can be added after an otherwise complete navigation interface. They are premises of that interface. The order selects where to spend analytical effort first.

## Next: make one execution profile internally complete

| Rank | Question | Why here / main dependencies | This pass | Evidence or choice still needed to close it |
| --- | --- | --- | --- | --- |
| 9 | **Q-AUTH-01: operation scopes/conflicts and bypasses** | The boundary must cover every bridge path that can conflict with the selected profile | Queued; boundary assumptions identified | Scope/conflict matrix plus plugin/raw/GCS route map; distinguish enforced and external paths |
| 10 | **Q-NAV-01: intent kinds/dimensions/omission** | Bounds the profile and establishes what counts as a complete decision | Queued; complete-snapshot recommendation retained | One initial control-level/vehicle contract, dimension requirements and explicit uncontrolled fields |
| 11 | **Q-OBS-02: frames, origins, vertical and NAV meanings** | A fresh complete vector is unusable if its quantities/references are ambiguous | Queued | Profile frame/origin/vertical definitions; yaw versus course and ground versus air speed; reset behavior |
| 12 | **Q-OBS-01: observation eligibility bounds** | Fresh decisions need current prerequisites, including mode/state evidence | Queued; age-budget structure developed | Per-quantity/per-use source-age, acquisition-age, quality and continuity limits, with available timing evidence |
| 13 | **Q-AUTH-02: per-profile phase guards/actions** | Entry, ACTIVE, release and recovery need the concrete quantities and observations from ranks 9–12 | Queued | Fully populated phase/guard/action table, confirmation predicates and contradictory-evidence handling |
| 14 | **Q-AUTH-06: external override/failsafe/loss** | Determines what the bridge can observe/control and what independent FC behavior is assumed | Queued | Trigger-specific detection/response table tied to FC release/configuration; detection gaps and no-recapture rules |
| 15 | **Q-NAV-04: adapter control laws/limits/degradation** | Must agree with frames, observations and authority rather than silently installing control loops | Queued | Named loop owners, transformations, gains/limits if any, supported dimensions and reject/degrade reporting |
| 16 | **Q-AUTH-03: preparation target/sequence** | Pre-entry streaming can move the vehicle before confirmation; requires ranks 10–15 | Queued | FC-specific order/target and early-consumption analysis, preparation permission and expiry response |
| 17 | **Q-NAV-05: scheduler/timing budgets** | Required rate alone is insufficient; feasibility depends on the whole age/queue/adapter path | Queued; budget equations developed | Numerical profile requirements, library/queue feasibility, deadline/miss policy and later timing measurements |

Ranks 10–17 should be developed as one profile rather than as universal defaults. The analysis can proceed before all implementation mechanisms are chosen; claims of achievable rates and response bounds cannot.

## Then: complete transaction and compatibility semantics

| Rank | Question | Why here / main dependencies | This pass | Evidence or choice still needed to close it |
| --- | --- | --- | --- | --- |
| 18 | **Q-CMD-02: ACK correlation, concurrency, retries** | A command ID often does not identify an application transaction; even sequential reuse can match a late ACK | Queued; prior source trace remains evidence | Per-operation correlation/conflict classes, late-ACK treatment, retry/quarantine/reconciliation policy; required before command execution |
| 19 | **Q-CMD-03: acceptance/state/task result goals** | Admission, ACK and fresh physical-state predicates have distinct outcomes | Queued | Operation catalog with result goals, predicate/sample coverage, no-op/cancellation and timeout rules |
| 20 | **Q-CMD-01: equality/IDs/dedup compaction** | Prevents duplicate admission and conflicting reuse; depends on ranks 8 and 18–19 | Queued | Exact semantic equality, allocation/order scope, ledger bounds and invalidation-before-forgetting proof |
| 21 | **Q-OBS-03: derived skew/covariance/family selection** | A complete observation profile needs parent-age/skew and missing-data rules | Queued | Derived quantity graph, uncertainty/skew limits and family selection; must move earlier if the first adapter needs fusion |
| 22 | **Q-OBS-04: retained numeric projections** | Status does not protect an unaware consumer still using retained NAV values | Queued; legacy limitations explained | Consumer qualification and loss behavior, projection generation/status association, supported interoperability boundary |
| 23 | **Q-SH-05: encoding/names/revision/errors** | Final encoding should express settled semantics, not determine them accidentally | Queued | Candidate comparison for precision, missingness, size, duplicates, evolution, result/error forms and operator use |
| 24 | **Q-CMD-05: bare legacy effectful variables** | Retained strings/booleans lack intent identity; safe compatibility depends on transactions and active sessions | Queued | Whether supported at all; qualified adapter exchange, intentional-repeat and retained-message rejection rules |
| 25 | **Q-CMD-04: history/query retention** | Needed for useful audit/recovery but detailed query shape follows transaction/result/persistence semantics | Queued | Latest/history/query contract, ordering/gap/retention/resource bounds and restart behavior |

Rank 25 defers the final history interface, not recovery evidence required by rank 8. Similarly, rank 22 does not authorize unsupported legacy consumers in an earlier profile. Dependencies can promote a queued question when the first profile is selected.

## Recommended disposition

Carry the eight focused recommendations into a review of [proposed decision 0003](../../decisions/0003-coherent-helm-and-origin-freshness.md). If adopted, produce draft-02 with those choices and explicit remaining profile/mechanism questions. Do not mark a whole question resolved solely because its preferred direction has been selected: source integration, resource/timing feasibility and validation evidence are separate closure work.
