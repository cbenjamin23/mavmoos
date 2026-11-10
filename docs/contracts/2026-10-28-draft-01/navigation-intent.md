# Navigation-intent contract

Status: proposed, draft-01, October 28, 2026. Depends on the [shared model](shared-model.md), [observation](observation.md), and [authority](authority.md) contracts. This document describes navigation intent and eligibility for translation. It does not choose a controller or grant navigation permission by itself.

## 1. Purpose and source basis

The [setpoint trace](../../research/2026-10-28-operating-models/trace-03-continuous-setpoints.md) found separate scalar updates, independent stream scheduling, per-access locking without an atomic decision snapshot, all-stop zeros, and broker renewal of cached values. The proposed contract makes a complete decision, its producer, and its validity interval explicit.

**NAV-1:** continuous navigation consumes complete, expiring snapshots bound to one producer session, vehicle session, bridge epoch, authority epoch, and grant reference. A snapshot is a decision object; individual MAVLink transmissions are executions of that object while it remains eligible.

## 2. Logical snapshot

| Field group | Proposed meaning |
| --- | --- |
| Identity | Contract revision, `producer_id`, `producer_session`, `ingress_id`, `bridge_epoch`, `vehicle_id`, `vehicle_session` |
| Authority binding | `authority_epoch`, `grant_id`, execution-profile revision; bridge resolves current phase/permissions independently |
| Decision identity | Monotonic `intent_seq` within the producer/session/authority scope; decision/cycle identity and coherence basis |
| Origin validity | Decision-production time with a bounded mapping or receiver-issued freshness window; producer-declared validity bounded by bridge policy |
| Decision state | Explicit valid/invalid status and reason; associated helm state/all-stop/producer evidence and their identities/ages |
| Intent kind | Named quantity/control contract, such as local velocity or course/groundspeed with a specified vertical request |
| Quantities | Complete required values, units, frames/origins/references, constraints, and declared uncontrolled dimensions |
| Facing policy | Independent yaw/heading/yaw-rate request or explicitly uncontrolled; not inferred from travel direction |
| Receiver metadata | Admission/result reason, effective use deadline, accepted order, adaptation prerequisites, and current eligibility |

An intent kind specifies its required dimensions and the meaning of omission. No dimension silently inherits a value from an unrelated earlier snapshot. Partial transport updates are assembled and validated before they become a candidate complete object. [Q-NAV-01](open-questions.md#navigation-questions).

## 3. Admission, replacement, and renewal

The bridge evaluates schema, issuer/target/epoch bindings, duplicate/order rules, origin freshness, decision validity, required observations, profile support, and current grant phase. The whole snapshot is either admitted for its permitted phase or rejected with reasons. Independent raw field assignments cannot partially replace an active snapshot.

**NAV-2:** the same sequence with the same semantic body is a duplicate, not renewal. The same sequence with different content is a conflict. A lower sequence cannot become the current intent. A higher sequence can renew unchanged numerical values only when it represents a newly validated complete decision from the permitted origin, with fresh origin evidence. Mere heartbeat, cached relay output, or serialization does not establish that.

**NAV-3:** snapshot expiry is the earliest bound in the shared model. New intent may extend its own validity within the current grant's limit; it does not extend that grant implicitly. Grant renewal is a separate policy event, even if a later transport design carries it in the same envelope. Expired/revoked authority cannot be revived with a larger sequence number.

Explicit invalidation from the currently permitted producer/session invalidates its eligible intent without turning its numeric values into an action. A valid decision envelope declaring all-stop, incomplete decision, or invalid state is handled according to the profile's failure/transition policy. A malformed or unauthorized packet is rejected; it does not automatically gain the ability to revoke another producer's grant. The policy for loss/ambiguity of the authorized stream still applies.

Do not continue the last good intent after an explicit, valid invalidation merely because that old snapshot has remaining time. Whether a short decision gap permits a bounded continuation before invalidation is an explicit profile question, not an accidental cache behavior. [Q-NAV-02](open-questions.md#navigation-questions).

## 4. Coherence and conventional helm compatibility

Candidate coherence grades are:

| Grade | Evidence | Draft treatment |
| --- | --- | --- |
| `PRODUCER_ATOMIC` | Complete decision object emitted by the original decision producer after decision completion | Candidate for strict navigation admission |
| `GENERATION_VERIFIED` | Qualified adapter proves that required fields and state belong to one completed generation | Candidate for strict admission after that mechanism is established |
| `ARRIVAL_GROUPED` | Receiver groups recently arrived independent scalars | Does not establish one helm decision; diagnostic/compatibility use only under this draft's strict policy |

The current helm's scalar publications and heartbeat must not be assumed to provide an atomic decision marker. A companion adapter may be sufficient, or a producer-side completion interface may be necessary; the prior study does not settle that. This is a principal open question, [Q-NAV-03](open-questions.md#navigation-questions).

The compatibility adapter must explicitly map configured helm domains, prefixes, producer identities, optional dimensions, manual override, PARK/STANDBY/DISABLED states, and all-stop. It must account for the studied normal/all-stop prefix asymmetry. No zero vector is implicitly a “stop safely” operation, and a fresh `IVPHELM_STATE` is not proof of a new completed solver result.

The recommended first strict profile requires an atomic or generation-verified producer contract. A weaker scalar mode, if retained for interoperability, must advertise its reduced coherence/age guarantees and receive its own acceptance criteria. The draft does not authorize modifying pHelmIvP or implementing an adapter.

## 5. Vehicle-aware adaptation

**NAV-4:** adaptation names the controlled quantities and which loops it owns. Course versus yaw, airspeed versus groundspeed, local velocity versus position, altitude reference versus vertical rate, and actuator output remain distinct contracts. The adapter cannot silently substitute an unsupported quantity or install an undocumented feedback controller.

A requested relative altitude requires an explicit reference identity and current observation in that reference. Terrain altitude requires separate terrain evidence. Conversion to a velocity target needs a specified controller/gain/limit/observation-age contract if the adapter owns that loop. No gains from pArduBridge are adopted here.

Unsupported intent/profile combinations are rejected. Limits, saturation, or degradation must be declared by the profile and reported with the effective intent; a bridge must not quietly clamp or drop a controlled dimension while reporting the original intent as fully executed. A profile may reject rather than degrade. [Q-NAV-04](open-questions.md#navigation-questions).

Intent validity and successful message queueing do not establish FC tracking. Execution status can reference current observations and profile-defined tracking predicates, but must state what is being measured. Read-only telemetry remains available independently of navigation permission.

## 6. Scheduler and egress

The scheduler consumes an immutable admitted snapshot plus current observations/grant state. It checks target/session, authority phase, current permission, origin expiry, required observation freshness, and profile compatibility before selecting each target. It rechecks relevant bindings at the last controllable egress boundary. No blocking operation may silently turn a bounded intended stream into an indefinitely delayed one.

**NAV-5:** each execution profile specifies a required transmission envelope: desired cadence, maximum allowed gap/queue age, clock basis, priority and queue bounds, observation-age limits, and the response if those bounds cannot be met. No generic numeric defaults are selected. Record actual enqueue/handoff timestamps separately from intent age and report misses. [Q-NAV-05](open-questions.md#navigation-questions).

Queued copies must not outlive the snapshot/grant deadline. A receiver can use latest-intent replacement only where its egress/cancellation semantics are documented; old queued values cannot reappear after replacement or a new authority epoch. Already-sent bytes remain a residual uncertainty governed by the authority profile.

During `ACQUIRING`, routine navigation is not yet permitted. The authority contract may grant a bounded preparation-stream scope using a profile-defined preparation target and fresh candidate intent. Transition to `ACTIVE` requires current intent and fresh confirmation evidence; it cannot be triggered by the intent payload's own state label. [Authority handoff sequence](authority.md#4-acquisition-and-release).

## 7. Illustrative snapshot

This sketch describes a possible semantic object, not a selected vehicle profile, wire format, or controller:

```yaml
contract_revision: draft-01
producer_id: helm-primary
producer_session: H9
ingress_id: I3
bridge_epoch: B7
vehicle_id: vehicle-2
vehicle_session: S4
authority_epoch: A12
grant_id: G12
execution_profile: course-groundspeed-relative-altitude-profile-A
intent_seq: 18
decision_id: cycle-18
coherence: PRODUCER_ATOMIC
freshness_window_id: W23
decision_validity: VALID
horizontal: {kind: course_over_ground, course_deg: 90, groundspeed_mps: 2}
vertical: {kind: relative_altitude, reference_id: home-4, meters: 12}
facing: {kind: UNCONTROLLED}
```

The facing policy does not request yaw equal to course. The profile must support that distinction or reject the object. A duplicate sequence 18 does not extend its deadline. A newly completed, valid decision can issue sequence 19 with the same quantities under a current freshness window and grant.

## 8. Failure, recovery, and open questions

On expired/ineligible intent, stop admitting routine navigation under that snapshot and apply the authority profile's explicit response. Do not reinterpret expiry as a command to disarm, zero altitude, or enter one universal flight mode. After grant/session loss, reacquisition uses a new authority epoch and a newly bound decision; cached prior intent cannot resume automatically.

[Q-NAV-01 through Q-NAV-05](open-questions.md#navigation-questions) track canonical kinds, gap/invalidity response, producer coherence, adapters/control laws, and scheduling. The [review scenarios](review-scenarios.md) include identical-value renewal, relay duplicates, partial scalar updates, stale decision with live heartbeat, all-stop, standby overlap, and queued stale targets. They are design checks, not executed tests.
