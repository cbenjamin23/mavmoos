# Open questions and review choices

Status: proposed, draft-01, October 28, 2026. Every question below remains open. Candidate directions are recommendations for review, not accepted architecture decisions. “Blocks” describes the guarantee or future profile that cannot be claimed until the question is resolved; it does not block drafting or archival work.

## Shared questions

| ID | Open question | Candidate direction | Evidence or choice required / what it blocks |
| --- | --- | --- | --- |
| Q-SH-01 | How is a producer/session/ingress established and trusted through native MOOS and relays? | Explicit configured issuer/path policy plus an active receiver-session exchange; treat names/IDs as claims rather than authentication | Deployment threat/trust model, relay provenance, identity mechanism, revocation; blocks effectful admission guarantees |
| Q-SH-02 | What proves physical vehicle identity and session continuity when hardware identity or boot evidence is incomplete? | Provisioned target binding with advertised evidence strength; conservatively create a new session on ambiguity/reconnect | Autopilot/link identity capabilities and restart/wrap analysis; blocks wrong-vehicle/continuity claims |
| Q-SH-03 | Which mechanism bounds original request/decision age: calibrated source clocks, bridge-issued windows, or both? | Use fixed receiver-owned freshness windows when a clock bound is unavailable; never rebase validity on relay arrival | Exchange protocol, maximum age/uncertainty, time-warp/reboot behavior, producer obligations; blocks strict request/intent freshness |
| Q-SH-04 | What ledger survives restart and how are possibly executed effects reconciled? | Preserve bounded transaction evidence where feasible; otherwise invalidate all old ingress bindings and expose uncertainty | Durability/retention/crash model and reconciliation process; blocks exactly-once claims and automatic recovery of effectful work |
| Q-SH-05 | What canonical encoding, MOOS channels/namespaces, revision negotiation, and error envelope should carry these semantics? | A versioned compound representation with documented projections; compare candidate encodings before selection | Size/precision/unknown-value rules, evolution/compatibility review, operator ergonomics; blocks final wire/interface specification |

Q-SH-04 is not resolved merely by invalidating navigation grants: an uncertain discrete physical effect can outlive a bridge process. Likewise, freshness-window possession alone is not issuer authentication; Q-SH-01 and Q-SH-03 must be resolved together.

## Observation questions

| ID | Open question | Candidate direction | Evidence or choice required / what it blocks |
| --- | --- | --- | --- |
| Q-OBS-01 | Which source-age, acquisition-age, validity, and continuity bounds are required for each observation use? | Per-kind/per-consumer requirements; allow weaker display telemetry without presenting it as control-grade evidence | Measurement timing available from each adapter and use-specific thresholds; blocks control eligibility profiles |
| Q-OBS-02 | What are the canonical frame/origin/vertical references and NAV_HEADING/NAV_SPEED compatibility meanings? | Preserve distinct yaw/course and ground/air speed; make aliases profile-specific and references explicit | Supported vehicle/consumer matrix, geodesy/estimator reset contracts; blocks universal aliases and frame-dependent control |
| Q-OBS-03 | How are derived-input skew, covariance/uncertainty, and missing-family selection represented? | Retain parent IDs and conservative age bounds; explicit unknown uncertainty and per-family selection rules | Transformation/fusion specification and numerical verification plan; blocks quality guarantees for derived observations |
| Q-OBS-04 | What happens to legacy numeric projections when data expires but MOOSDB retains their old values? | Suppress fresh-looking stale updates, publish explicit status, and qualify unaware legacy consumers separately | Consumer loss behavior, retention/withdrawal options, projection generation mechanism; blocks safe legacy interoperability claims |

## Command questions

| ID | Open question | Candidate direction | Evidence or choice required / what it blocks |
| --- | --- | --- | --- |
| Q-CMD-01 | How are semantic-body equality, request ID allocation, duplicate-ledger retention, and compaction specified? | Immutable request key/body; conflict rejection; expiry/session invalidation before forgetting deduplication | Canonical comparison rules, finite ledger model, concurrent-admission analysis; blocks duplicate-admission guarantees |
| Q-CMD-02 | What wire-correlation/concurrency/retry rules apply when application IDs are not returned? | Per-operation matching and conflict classes; serialize or reject ambiguous work; retain explicit timeout uncertainty | Selected protocol/library behavior, late-ACK bounds and any quarantine/reconciliation rule; blocks strong result attribution and generic retries |
| Q-CMD-03 | Which operations require protocol acceptance, fresh state confirmation, or task completion, and under what predicates? | Versioned per-operation goals; explicit no-op and weaker send-only outcomes | Predicate validity/timing/sample coverage and cancellation semantics; blocks a universal success/completion flag |
| Q-CMD-04 | How can late subscribers query results/history, with what retention and event-order guarantees? | Latest transaction snapshot plus bounded per-transaction history and gap/retention reporting | Query/response encoding, storage/resource bounds, restart behavior; blocks complete audit/history claims |
| Q-CMD-05 | Can bare legacy request variables remain usable without causing replay or losing correlation? | Canonical executable envelopes; legacy side-effecting variables require a qualified active adapter/session | Operator workflows, MOOS registration behavior, intentional-repeat semantics, response mapping; blocks supported legacy command execution |

Even one in-flight wire command at a time may leave ambiguity when an old ACK arrives after a timeout and the same command number is reused. Q-CMD-02 requires a policy for that sequential case, not only concurrent exclusion.

## Navigation questions

| ID | Open question | Candidate direction | Evidence or choice required / what it blocks |
| --- | --- | --- | --- |
| Q-NAV-01 | Which canonical intent kinds/dimensions are supported and how is omission expressed? | Complete typed snapshots; explicit uncontrolled dimensions; no silent cross-snapshot inheritance | Initial vehicle/control-level matrix and adapter inputs; blocks final intent schema |
| Q-NAV-02 | What continuation, if any, is allowed during a short decision gap, all-stop, or explicit invalidation? | Explicit producer invalidation removes eligibility; any bounded gap behavior is profile-specific | Vehicle/failure analysis, helm transition behavior, acceptable continuation bounds; blocks a generic last-good policy |
| Q-NAV-03 | How can standard pHelmIvP supply a trustworthy completed decision generation without assuming heartbeat is a commit marker? | Prefer producer-atomic output or a source-backed generation-verifying adapter; do not certify arrival grouping as atomic | Further source/interface investigation and feasibility review of companion versus producer-side changes; blocks strict scalar-to-intent compatibility |
| Q-NAV-04 | Which loops, conversions, limits, and degradation rules belong to each vehicle adapter? | Explicit quantity/control contracts; independent facing policy; reject unsupported mappings | Vehicle/autopilot profiles, control-law ownership and later verification; blocks executable adapters and adopted gains |
| Q-NAV-05 | What scheduler, queue, transmission-gap, and observation-age budgets are achievable and required? | Separate navigation budget from potentially blocking operations; deadline-aware egress | Transport/library feasibility and later load/timing measurements; blocks bounded streaming guarantees |

No bridge can infer a newly completed solver decision solely from changed numbers, unchanged numbers, or a periodically renewed heartbeat. Q-NAV-03 is therefore a substantive interface question even if all scalar accesses are thread-safe.

## Authority questions

| ID | Open question | Candidate direction | Evidence or choice required / what it blocks |
| --- | --- | --- | --- |
| Q-AUTH-01 | Which operation scopes conflict, and what raw/GCS/proxy paths are inside the enforcement boundary? | Explicit scope/conflict table; classify unmanaged paths as external | Plugin/library/router integration and deployment map; blocks complete mediation claims |
| Q-AUTH-02 | What exact per-profile entry, active, release, failure, and recovery guards/actions refine the candidate phase table? | Separate bridge permission from observed FC state; use fresh named evidence and explicit uncertainty | Autopilot/release/vehicle requirements and failure walkthroughs; blocks qualified authority transitions |
| Q-AUTH-03 | What preparation sequence/target is appropriate when a stream is required before mode confirmation? | Bounded preparation scope, fresh candidate intent, explicit behavior if FC consumes it early; no universal zero/hold target | Adapter/FC semantics, target continuity and later simulation/hardware qualification plan; blocks that external-control acquisition profile |
| Q-AUTH-04 | Can every relevant queue and SDK retry be cancelled/drained/gated, and how much old work can still reach the vehicle? | Gate before enqueue and at last controllable egress; invalidate epoch-bound work and report residuals | Selected library/transport behavior and acceptable maximum residual interval; blocks immediate-stop/clean-handoff claims |
| Q-AUTH-05 | How are primary/standby helms, operator requests, and lease renewal prioritized? | Explicit source/session arbitration and new epoch on takeover; no arrival-order authority | Deployment policy, standby interface and transient overlap analysis; blocks automatic source handoff |
| Q-AUTH-06 | What response applies to each external override/failsafe/loss, and which independent FC behavior can be relied on? | Distinct triggers/responses; inhibit routine stale intent and avoid automatic recapture; report detection gaps | External-path observability and configured FC behavior by vehicle/release; blocks universal fallbacks and global ownership claims |

## Suggested review order

First resolve trust/session and origin-age assumptions, then select a concrete intent/observation/control profile for analysis. Specify coherent helm production, operation result goals, and handoff/preparation semantics for that profile. Assess library egress/queue feasibility before promising deadline or revocation behavior. Finalize encoding and operator compatibility against those semantics.

These questions can be investigated incrementally. Choices intended for adoption should be recorded separately as accepted or superseding decision records, with evidence and the affected contract revision. None is marked resolved here merely because the draft recommends a direction.
