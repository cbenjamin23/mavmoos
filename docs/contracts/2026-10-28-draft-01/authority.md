# Authority and execution contract

Status: proposed, draft-01, October 28, 2026. Depends on the [shared model](shared-model.md), [observations](observation.md), [transactions](command-transaction.md), and [navigation intent](navigation-intent.md). This is a candidate state/permission model, not an accepted autopilot mode map.

## 1. What authority means here

**AUTH-1:** authority is the bridge's permission to emit a defined operation/stream for a named producer, vehicle session, profile, and epoch. It is not proof of exclusive global control of a vehicle. External RC inputs, GCS paths, autonomous FC decisions, and failsafes may act independently.

Publish these facts separately: requested navigation owner, bridge execution phase, current grant/scope, effective bridge transmission permission, observed FC mode/arming/state with age, detected override/failsafe evidence, and uncertainty. `FC_CONTROL`, `HELM_CONTROL`, or `MANUAL` may remain useful UI concepts, but they are not a complete phase machine or proof of observed ownership.

The [operating-model comparison](../../research/2026-10-28-operating-models/operating-model-comparison.md) and [setpoint trace](../../research/2026-10-28-operating-models/trace-03-continuous-setpoints.md) motivate these separations. They do not establish one fallback, exclusivity mechanism, or supported vehicle profile.

## 2. Operation scopes

| Scope | Draft policy boundary |
| --- | --- |
| Observation/query | No helm grant required; target/trust/resource policy still applies |
| Configuration/parameter changes | Separate authorization and conflict rules; may affect active navigation eligibility |
| Arming/state-changing command | Explicit transaction and current profile policy; no implicit permission from helm liveness |
| Mode/mission-control command | Serialized with relevant authority transitions; may invalidate navigation grants |
| Preparation stream | Bounded permission during acquisition/release/failure response, with profile-defined targets and guards |
| Active navigation stream | One permitted producer/session in one current authority epoch for the controlled dimensions |
| Direct actuator/raw protocol | Separate explicit scope with enforceable limits or an advertised boundary outside the managed guarantee |
| Bulk transfer/payload operation | Independent function with specified resource impact and any required conflicts |

The exact scope taxonomy is open under [Q-AUTH-01](open-questions.md#authority-questions). The invariant is that plugins/SDKs/raw routes cannot bypass managed permission by choosing another interface. A transparent GCS proxy that the bridge cannot safely classify/gate must be documented as an external control path, not advertised as governed by its authority manager.

## 3. Candidate execution phases

| Phase | Meaning | Routine navigation permission |
| --- | --- | --- |
| `INHIBITED` | Current prerequisites/profile/policy prevent navigation admission | None |
| `READY` | Bridge has a resolved target/profile and can evaluate an acquisition request; this does not assert vehicle safety/readiness | None |
| `ACQUIRING` | One admitted takeover attempt; fresh intent/evidence is being collected and profile-specific preparation may run | None; only explicit bounded preparation scope |
| `ACTIVE` | One current grant permits a named producer's eligible intent | Only while all live guards remain true |
| `RELEASING` | Routine grant revoked; profile-specific release action and confirmation may be pending | None; only explicit bounded release scope |
| `RECOVERY_REQUIRED` | Effect, identity, session, or handoff outcome is uncertain; reconciliation is required | None |

Manual control, FC autonomy, and failsafe states are observations alongside these phases. They need not be forced into one mutually exclusive bridge-state label. An inactive navigation grant does not disable permitted telemetry/query functions.

Candidate transitions:

| From / event | Required local action and evidence | Candidate destination |
| --- | --- | --- |
| Startup | Create bridge epoch; invalidate inherited grants/queues; resolve identity/profile and possible prior effects | `INHIBITED` or `RECOVERY_REQUIRED` |
| `INHIBITED` / prerequisites established | Fresh target binding, applicable profile, ingress/policy ready to evaluate requests | `READY` |
| `READY` / acquisition admitted | Create new authority epoch and bounded grant; bind producer/session; set transition deadline | `ACQUIRING` |
| `ACQUIRING` / entry conditions satisfied | Current eligible intent, fresh FC observations, profile confirmation and egress guards; atomically permit active scope | `ACTIVE` |
| `ACQUIRING` / rejection, timeout, ambiguity | Remove preparation permission; execute only applicable bounded failure response; record uncertainty | `INHIBITED` or `RECOVERY_REQUIRED` |
| `ACTIVE` / release requested | Invalidate routine navigation grant/queue before further local routine sends; begin applicable release procedure | `RELEASING` |
| `ACTIVE` / guard loss or external takeover | Immediately remove routine eligibility; avoid automatic recapture; apply applicable failure/override response | `INHIBITED` or `RECOVERY_REQUIRED` |
| `RELEASING` / release evidence sufficient | Record profile's observation outcome and remaining external-ownership uncertainty | `READY` or `INHIBITED` |
| `RELEASING` / timeout or conflicting evidence | Report possible remote effects and residual transmissions | `RECOVERY_REQUIRED` |
| Any / target-session or bridge continuity loss | Invalidate grants, pending intent, and effectful queue eligibility; reconcile old transactions | `RECOVERY_REQUIRED` |
| `RECOVERY_REQUIRED` / explicit reconciliation complete | Establish new sessions/bindings as needed; no revival of old authority | `INHIBITED`, then separately `READY` |

This table requires profile-specific guard/action refinement. No transition is implemented, and the table does not nominate generic FC mode names. [Q-AUTH-02](open-questions.md#authority-questions).

## 4. Acquisition and release

A proposed acquisition sequence avoids the circular requirement that active authority must exist before an FC-required preparation stream:

1. Admit an authority-acquisition transaction for a validated producer/target/profile. The requesting issuer and requested navigation producer may differ, such as an operator requesting helm control; policy must authorize both that request and that producer binding. This operation alone does not arm or start a navigation stream.
2. Assign a new authority epoch and bounded grant reference in `ACQUIRING`. Initially permit collection of a fresh candidate intent and the observations needed to decide whether preparation is possible.
3. Obtain a complete current intent bound to that epoch, plus required fresh identity/mode/estimator/arming observations. Verify a qualified preparation specification and egress budget.
4. If the profile requires messages before mode entry, enable only its bounded preparation-stream scope, then execute the profile's explicit mode-entry sequence. Other profiles may require another ordering.
5. Observe mode/state confirmation with the required sample freshness/coverage. Recheck current intent, grant lifetime, target/session, external-override evidence, and scheduler health. Atomically change effective permission and phase to `ACTIVE`.
6. If any guard fails, invalidate the relevant permissions and follow the declared rejection/failure/recovery path. Do not resume acquisition merely because a delayed ACK arrives.

**AUTH-2:** a preparation stream must specify its target, reference, origin-validity requirements, finite lifetime, permitted operation kinds, and what happens if the FC starts using it before confirmation reaches the bridge. “Preparation” cannot be assumed motion-free. A profile may require a separately specified capture/hold target or another validated sequence; this draft chooses no universal target. Without a qualified entry sequence, acquisition is unavailable. [Q-AUTH-03](open-questions.md#authority-questions).

The same epoch/grant can move from bounded preparation to active permission through bridge-owned state, while accepted snapshots retain their bindings and original deadlines. The producer cannot upgrade the grant by editing its envelope. Any preparation target and the first active target must have a defined transition; unsupported discontinuity blocks activation.

For release, remove routine eligibility first, invalidate controllable queued work, then perform only the permitted release action and wait for its declared evidence. Record a release timeout as uncertainty rather than continued active authority. A release request does not prove the FC ceased applying an earlier target or that another actor assumed control.

Release/failure work uses a distinct bounded bridge-owned operation permit, with the triggering authority epoch retained as provenance. It cannot reuse the revoked producer grant as permission, accept arbitrary old-producer setpoints, or extend expired intent. The status/history identifies that permit, its profile, prerequisites, and deadline separately from the routine navigation grant.

## 5. Grant guards, renewal, and handoff

A grant binds producer/session, ingress/trust context, bridge epoch, vehicle session/target, authority epoch, controlled quantities, allowed phase/scopes, profile revision, fixed expiry, and revocation revision. The gate evaluates the current bridge-owned record, not the issuer's claimed grant contents.

**AUTH-3:** all active guards must hold continuously at the defined local evaluation/egress boundaries: valid source/session, eligible complete intent, current required observations, compatible FC mode/state, current permission, remaining lifetime, and scheduler/egress feasibility. An observed compatible FC mode is necessary where a profile says so, but is not sufficient by itself to grant a producer control.

Renewal within an unbroken active interval can extend a grant only through an explicit policy event with current origin/health/evidence. Expired, revoked, or transferred authority requires a new epoch. No mode heartbeat, new setpoint sequence, reconnect, or SDK success callback silently recreates a grant.

Producer takeover, including standby helm promotion, creates a new authority epoch and source binding. Old-producer snapshots, cached decisions, delayed callbacks, and old-grant renewal attempts remain ineligible. The policy for which helm wins and what overlap can be tolerated is unresolved; middleware arrival order is not the arbitration rule. [Q-AUTH-05](open-questions.md#authority-questions).

## 6. Failure and external actors

Each profile must distinguish intent expiry, producer loss, invalid decision/all-stop, MOOSDB loss, FC-link loss, stale mode/estimator observations, scheduler miss, external mode/RC takeover, FC failsafe, and identity reset. These triggers may require different local responses. No one “Loiter on failure” or “zero speed” behavior is adopted.

**AUTH-4:** after a guard loss, routine navigation from the invalid intent/epoch is ineligible. A bounded failure-response operation may be separately permitted only by the profile, using its own fresh evidence and deadline. It must not automatically recapture control from a recognized external operator/failsafe. If the link or observations cannot support that response, report the limitation and rely only on independently configured FC behavior for effects the bridge cannot cause.

The bridge cannot guarantee detection or arbitration of every external path. A proxy link, separate GCS link, or RC source outside the gate is identified as such in the deployment profile. A nominal mode match cannot establish that no external actor is commanding. [Q-AUTH-06](open-questions.md#authority-questions).

## 7. Egress and recovery guarantees

Revocation is ordered with intent replacement and operation dispatch through an explicit local gate/barrier. Pending queue entries and SDK retries retain epoch/expiry bindings and must be invalidated or bounded at a controllable boundary. Record the last known handoff, queue state, and remaining uncertainty. [Shared egress model](shared-model.md#5-scope-of-transmission-guarantees).

**AUTH-5:** no claim of “immediate stop,” absence of in-flight targets, or exclusive FC control follows merely from a state-label change. A future implementation must establish the transport/library queue-drain/cancellation behavior and maximum tolerable residual window for its profile. If that cannot be established, the corresponding handoff guarantee remains unavailable. [Q-AUTH-04](open-questions.md#authority-questions).

After bridge restart, target reconnect, or unresolved effect, expose `RECOVERY_REQUIRED` with old transaction references where retained, fresh observation requirements, and reason codes. Explicit reconciliation establishes current target/state and resolves or preserves uncertainties. A new acquisition uses a new authority epoch and fresh snapshot. Previously armed state may be observed, but must not trigger automatic replay or an implicit permission to disarm.

## 8. Authority status example and review questions

Illustrative status sketch, not a selected MOOS channel/schema:

```yaml
contract_revision: draft-01
bridge_epoch: B7
vehicle_id: vehicle-2
vehicle_session: S4
authority_epoch: A12
grant_id: G12
grant_revision: 3
phase: ACQUIRING
requested_producer: {id: helm-primary, session: H9}
routine_navigation_permitted: false
preparation_scope: profile-defined-and-bounded
observed_fc_mode: {known: true, observation_id: B7:obs-28}
confirmation: WAITING_FOR_FRESH_PROFILE_EVIDENCE
```

This status does not assert physical motion, FC command acceptance, or global ownership. Active permission later needs every guard; a copied status record cannot recreate it. [Q-AUTH-01 through Q-AUTH-06](open-questions.md#authority-questions) cover scopes, transitions, preparation, egress residuals, arbitration, and external/failure policies. [Review scenarios](review-scenarios.md) walk through their interactions without claiming executed validation.
