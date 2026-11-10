# Observation contract

Status: proposed, draft-01, October 28, 2026. Depends on the [shared model](shared-model.md). This contract describes a reported fact or estimate. It does not grant control or imply that the vehicle accepted a command.

## 1. Purpose and source basis

The [telemetry trace](../../research/2026-10-28-operating-models/trace-01-position-telemetry.md) found separately cached measurements, conditional source-time conversion, implicit frame conventions, and repeated publication with current application time. The draft preserves the distinctions needed to interpret those observations correctly.

**OBS-1:** a canonical observation identifies its vehicle/session, producing source, quantity semantics, reference frame, times, and validity. Unknown metadata is explicit. A consumer can determine whether an observation is suitable for its use without guessing from a variable name or a recent notification.

## 2. Logical record

| Field group | Proposed contents and meaning |
| --- | --- |
| Contract/type | Contract revision, observation kind, quantity schema, profile revision |
| Identity | `bridge_epoch`, `vehicle_id`, `vehicle_session`, bridge-assigned `observation_id`; source system/component and producer/session where available |
| Provenance | Decoder/source endpoint, original observation or receipt identity, transforms applied, relay path, confidence of source/identity matching |
| Quantity | Named value/vector/record; explicit units; missing fields stay absent/unknown rather than becoming zeros |
| Frame/reference | Axis/frame convention, local-origin identity/version, vertical datum/reference, body reference, relevant estimator/frame-reset context |
| Time | Source sample time and clock/mapping quality; earliest trusted acquisition; bridge receive time; publication/as-of time |
| Freshness | Age bound and basis, local use deadline if known, status as of a named time; measurement age may remain unknown |
| Quality | Value validity, source validity/fix/estimator flags where supplied, uncertainty/covariance availability and frame, reason codes |
| Derivation | Parent observation IDs, oldest required input age bound, inter-input skew bound or unknown, derivation revision |
| Re-publication | `status_revision` or equivalent ordering within the observation, distinguishing a status/publication update from a new underlying sample |

Exact field names and serialization are open under [Q-SH-05](open-questions.md#shared-questions). The record may be implemented through a canonical compound payload plus documented projections; the table is not a proposed single massive payload for every sensor.

**OBS-2:** source/transport sequence numbers and bridge receipt IDs order available evidence but do not automatically prove a new physical measurement. If the original source offers no stable sample identity, advertise that limitation. Do not deduplicate or refresh measurement age based only on equal/unequal values.

## 3. Presence, validity, freshness, and applicability

Keep these dimensions separate:

| Dimension | Draft states | Interpretation |
| --- | --- | --- |
| Presence | `NO_SAMPLE`, `PRESENT` | Whether a retained observation exists |
| Value validity | `VALID`, `INVALID`, `UNKNOWN` | Whether values satisfy their declared semantics/source flags |
| Freshness | `FRESH`, `STALE`, `UNKNOWN` | Relative to the declared age basis and a specified profile requirement |
| Continuity | `CURRENT_SESSION`, `HISTORICAL_SESSION` | Whether identity/origin/connection assumptions still hold |
| Applicability | Profile-specific eligible/ineligible with reasons | Whether a particular consumer can use it now |

A recent receipt can be `FRESH` on an acquisition-age basis while measurement freshness remains `UNKNOWN`. A UI may display that observation; a control profile requiring bounded measurement age must reject it. There is no universal freshness threshold for all observation kinds or uses. [Q-OBS-01](open-questions.md#observation-questions).

**OBS-3:** zero is a legitimate numeric value where the quantity allows it. Invalid/missing coordinates, altitude, speed, covariance, or orientation must not be represented through an invented zero/sentinel with an undisclosed meaning. Source-provided sentinels are decoded into the declared validity model while preserving their original meaning for diagnostics.

**OBS-4:** retain historical values for explanation when useful, but prevent their reuse as current control evidence after expiry, source/session loss, or reference change. Publication of a stale-status transition must not fabricate a new measurement or reset its deadline. Consumers of retained state must evaluate expiry themselves; absence of a future status update is not permission to keep using a cached `FRESH` label.

## 4. Frames, units, and derived observations

Frame metadata must distinguish axis convention from origin alignment. Changing NED axes to ENU does not prove that the FC's local origin equals the MOOS mission origin. Global/local conversion records the origin and transform revision. Home, estimator, or reference resets either create new frame context or invalidate dependent observations until re-established.

Heading/yaw, course-over-ground, angular rate, ground/air speed, and different altitude references remain distinct kinds. The draft does not choose a global HEADING alias or call relative altitude terrain AGL. A profile can expose a compatibility alias only with documented semantics. [Q-OBS-02](open-questions.md#observation-questions).

**OBS-5:** derived data records its required inputs and their age/skew. Recomputing with the same parents does not renew parent freshness. Combining position with cached attitude must expose that dependency and honor the profile's input-age/skew requirements. An unknown required input age propagates to the relevant derived validity/eligibility, not to a fabricated fresh result.

Uncertainty/covariance must identify its quantities, units, reference frame, transformation, and unavailable/unknown status. Absence is not evidence of zero uncertainty. A transform or fusion rule needs a versioned specification and later numerical verification. [Q-OBS-03](open-questions.md#observation-questions).

## 5. Lifecycle and delivery

1. Bind the input to its source/vehicle session and decoder/profile.
2. Decode quantities, source validity, and available timing evidence.
3. Normalize frames/units or publish the original declared representation when conversion prerequisites are missing.
4. Admit an observation with its actual quality and derive any allowed projections.
5. Publish the canonical record and status; convenience projections refer to the observation or a defined projection generation.
6. Re-evaluate use eligibility when deadlines, source health, or frame context change, even without a new sensor message.
7. On discontinuity, retain historical evidence as such and establish a new session/reference before current use.

A malformed envelope/source identity is rejected as an input error. A correctly decoded but source-invalid measurement can be published as `INVALID` diagnostic evidence. Those are distinct outcomes.

**OBS-6:** transport loss, missing one message family, and estimator invalidity are independent. An “ever received” flag cannot stand for current availability. Loss of a covariance-bearing stream must not silently freeze the age of a lower-quality replacement or mark old covariance current; selection/fallback rules belong to the observation profile.

## 6. MOOS compatibility and retained state

Candidate projections include familiar `NAV_X`, `NAV_Y`, `NAV_SPEED`, and `NAV_HEADING`, with a configured vehicle namespace where required. Their semantic mapping and naming remain open. Scalar publications are not assumed atomic. A consumer requiring a coherent state uses the compound observation or a qualified generation/commit mechanism.

**OBS-7:** stale numeric data must not be repeatedly republished with a fresh-looking sample timestamp. A profile may suppress new numeric projection updates while publishing explicit invalidity/age, but old values can still remain in MOOSDB. That does not make an unaware legacy consumer safe. Such a consumer's compatibility and loss behavior need separate qualification; changing the number to zero is not an invalidation solution. [Q-OBS-04](open-questions.md#observation-questions).

Legacy/raw inputs without origin timing may be displayed under a weaker provenance/age basis. They must not be upgraded to stronger measurement validity because a bridge or broker re-notified them.

## 7. Illustrative record

This is a semantic sketch, not a chosen YAML/wire schema or safety configuration:

```yaml
contract_revision: draft-01
observation_id: B7:obs-27
bridge_epoch: B7
vehicle_id: vehicle-2
vehicle_session: S4
kind: local_position
source: {system_id: 1, component_id: 1, identity_basis: configured_link}
quantity: {x: 20, y: 10, z: 5, units: m}
reference: {axes: ENU, origin_id: estimator-origin-3, origin_alignment: unknown}
source_sample_time: {known: false}
freshness: {basis: trusted_acquisition, measurement_age: unknown}
value_validity: VALID
status_revision: 1
```

The quantities may be interpretable in that estimator frame while ineligible for a mission-origin-dependent controller. A later projection of the same record retains `B7:obs-27`, its original timing evidence, and original parents; a new publication time only changes its as-of/status metadata.

## 8. Review obligations and open questions

Proposed conformance cases include zero geographic coordinates, stale cached attitude, time-sync fallback, missing sample time, derived-input skew, disappearing message families, origin reset, delayed registration, and re-publication after expiry. Expected cross-contract outcomes appear in [review scenarios](review-scenarios.md).

Questions [Q-OBS-01 through Q-OBS-04](open-questions.md#observation-questions) cover required age/quality by use, aliases/references, derivation/uncertainty, and legacy projection loss behavior. No observation profile, covariance guarantee, or consumer compatibility is validated by this draft.
