# Compatibility with unmodified pHelmIvP

October 29, 2026. Proposed compatibility profile structure, constrained by [accepted decision 0004](../../decisions/0004-use-stock-moos-ivp.md). No new helm hook, behavior, envelope, nonce exchange or solver modification is required or permitted by this sketch. This document specifies what the bridge can consume and what it cannot certify.

## 1. Initial deployment assumptions

Analyze one configured vehicle context and one selected helm source in a trusted MOOS community, with direct helm-to-MOOSDB-to-bridge delivery. No cache-republishing command relay or automatic primary/standby arbitration is included in this initial compatibility basis. These are limits of the sketch, not exclusions from all future architecture work.

Source/community metadata are configured attribution claims, not authentication. The execution profile must state its MOOS clock/skew assumptions, required fields, accepted helm states, observation requirements, age budgets and failure/recovery policy. No vehicle-specific quantities, numerical timeouts or firmware support are adopted here.

## 2. Existing inputs

| Input | Proposed use | Important distinction |
| --- | --- | --- |
| `DESIRED_HEADING` | Configured course/heading quantity for the vehicle profile | Stock helm aliases its course domain to this name; it is not automatically body yaw |
| `DESIRED_SPEED` | Profile-defined speed quantity | Groundspeed versus airspeed must be declared |
| `DESIRED_ALTITUDE`, `DESIRED_DEPTH`, other configured dimensions | Only the dimensions actually required/supported by the profile | Absence is not zero; optional helm output may be insufficient for a particular adapter |
| `IVPHELM_STATE` | Selected producer's liveness/state evidence; normally require eligible DRIVE state | It is posted before solving and does not commit a decision; a `+` suffix does not grant takeover |
| `IVPHELM_ALLSTOP` | Latched selected-source status; a non-clear value inhibits the current input set | It is change-driven, not a periodic heartbeat; never interpret accompanying zeros as a universal action |
| `MOOS_MANUAL_OVERRIDE` | Recognized trusted inhibition evidence | A false value cannot by itself enable bridge navigation |
| `IVPHELM_ITER`, `IVPHELM_SUMMARY` | Optional diagnostics/progress only in this sketch | Their placement/journal semantics do not supply the proposed final atomic commit |
| Existing restart/PID evidence if configured | Conservative reset hint | Stock process names/PIDs/counters do not constitute an infallible producer-incarnation protocol |

`HELM_STATUS`, `DEPLOY`, and a bare `ALLSTOP` variable are project-specific interfaces in the reference apps, not silently interchangeable with the stock variables above. Operator enable/disable for MAVMOOS is a separate request, with final channel/encoding still open. The compatibility plugin observes helm output; mission apps/operators remain responsible for ordinary helm behavior/condition configuration.

The baseline uses ordinary unprefixed outputs. Configurable aliases may later support existing helm prefixes, but the selected source's all-stop scalar loop lacks the normal output prefix. Status/source filtering and explicit mappings must account for that asymmetry. Merely applying one prefix string to every subscription would be incorrect.

## 3. Receiving fields and building the local input set

For each configured required dimension, retain typed value, source/community, original MOOS message time, monotonic receipt time, ingress generation and a local field version. Reject wrong-source/type/non-finite/unsupported values as candidates; do not mutate a different producer's context. Preserve unknown clock metadata rather than substituting callback time as original production time.

At the end of input processing, expose an immutable `HelmInputSet` for translation. It contains the latest admissible values and their individual metadata. A local cache revision is for reproducibility and replacement, **not a helm generation number**. Use the existing weaker coherence label `ARRIVAL_GROUPED`; do not label it `PRODUCER_ATOMIC` or `GENERATION_VERIFIED`.

Require all profile-mandatory dimensions to have qualifying updates after the relevant local reset/enable boundary. Do not fill a newly enabled profile from indefinitely retained numbers. A missing optional helm output cannot be silently inherited forever; its eligibility follows its last qualifying update/deadline and the adapter's declared omission rules.

MOOS mail callbacks may contain values from several iterations or sources. End-of-batch construction avoids partial mutation during one host update, but does not prove a shared upstream cycle. Within a host input turn, recognized inhibitions take precedence over creating an eligible navigation plan; fresh-looking scalar zeros elsewhere in the batch do not override an all-stop.

## 4. Publication age and silence watchdog

There are two relevant checks for this stock path:

- Source-message age/skew checks under the profile's declared MOOS clock assumptions. These can reject stale registration data and old notifications when timestamps retain their meaning.
- Monotonic elapsed time since a qualifying receipt, evaluated even without further mail. This detects silence at the receiving boundary without allowing scheduler sends to refresh input age.

The initial profile must justify source-clock comparability and bound its allowed skew/future-time tolerance. Store a conservative local expiry derived at receipt from the permitted remaining publication age and local silence bound; take the earliest applicable field, permission and profile limit. Do not blindly compare remote monotonic timestamps or turn a fresh callback into a fresh original timestamp. If the required clock assumption becomes unknown/discontinuous, navigation is inhibited rather than silently changing to an undocumented weaker guarantee.

An exact duplicate field notification cannot extend its stored expiry merely because it was delivered twice. A new stock publication with unchanged numerical value can refresh that field's publication/receipt evidence; it still does not certify the other fields' solver generation. No refresh occurs merely because a plugin tick or setpoint send runs.

Crucially, source-message age here is **publication age**, not a proven decision-start or sensor-input age. A long solver call, independent behavior inputs, optional dimensions or interleaved scalar publications can remain invisible to these checks. A live heartbeat is another condition, not a replacement for field age or final-decision validity. These limits must be present in diagnostics and profile claims.

## 5. Permission and reset boundaries

The host starts with no navigation permission. An explicit operator acquisition request is required, along with a connected/current target context, eligible selected-source state, known compatible all-stop state, supported adapter, current FC prerequisites and all required post-boundary field updates. New input cannot grant permission to itself.

Reset relevant eligibility on host restart, MOOSDB disconnect, FC continuity loss, selected producer change, explicit disable/manual takeover, observed helm restart or a detected clock/reference discontinuity. Increment the relevant local generation, clear input readiness, invalidate controllable queued navigation and require deliberate re-enable where the profile specifies reacquisition. Do not auto-replay old stored values after reconnect.

With qualified comparable message time, the local boundary's corresponding MOOS time and monotonic receipt fence distinguish old retained messages from new post-boundary publications. A receipt after reconnect alone is insufficient. The implementation/profile still has to specify uncertainty/tolerance and exact ordering; no numeric rule is validated here.

`IVPHELM_ALLSTOP=clear` is a latched state assertion and may have an old timestamp during healthy operation, because repeated identical reasons are suppressed. A retained clear can seed the known as-of state, but cannot enable navigation, establish a new decision or reset any field deadline. Conversely, an unknown all-stop state is not silently treated as clear. Loss/overflow that makes control-state continuity uncertain inhibits the affected path until reconciled.

Explicit non-clear all-stop invalidates the current candidate immediately at the host's ordered processing boundary. Whether a transient empty/incomplete decision keeps an operator permission pending, requires explicit reacquisition, or permits later automatic recovery after clear plus fresh fields is a **profile decision still open**. No last-good continuation or automatic resumption is hidden in the scalar cache. The next profile must settle this before effectful capability is advertised.

Stock source metadata does not guarantee detection of every fast producer restart/incarnation change. Known restart signals, clock/counter discontinuities, source changes and liveness loss are usable conservative evidence, not a proof of an upstream session protocol. Likewise, a delayed primary's final publication must not select a new owner by arrival order; automatic multi-helm arbitration remains outside this initial profile.

## 6. What compatibility means

This sketch interoperates through stock published names and normal configuration. It does not claim that pHelmIvP violates its own interface or needs repair. It places explicit bounds and interpretations around a conventional downstream receiver.

The proposed protections are current local permission, required field presence, source/clock/publication-age checks, silence watchdogs, explicit state/all-stop handling, target/observation prerequisites and final egress validation. The remaining gaps include same-solver-generation proof, exact decision/input origin age, unobservable producer reincarnation, unmanaged external authority and downstream residual commands.

Therefore this compatibility profile **does not satisfy draft-01's stronger navigation coherence/origin contract as written**. The draft is preserved as historical design work; a future contract revision must describe the supported stock interface directly rather than pretend these gaps were solved by a wrapper. No producer-side change is a future action under the current accepted constraint.

## 7. Source basis

Exact inspected upstream source: [HelmIvP.cpp 340–422](https://github.com/moos-ivp/moos-ivp/blob/023fb54e00692611ecee2cd6f1138d10eec2127e/ivp/src/pHelmIvP/HelmIvP.cpp#L340-L422), [500–549](https://github.com/moos-ivp/moos-ivp/blob/023fb54e00692611ecee2cd6f1138d10eec2127e/ivp/src/pHelmIvP/HelmIvP.cpp#L500-L549), [1048–1064](https://github.com/moos-ivp/moos-ivp/blob/023fb54e00692611ecee2cd6f1138d10eec2127e/ivp/src/pHelmIvP/HelmIvP.cpp#L1048-L1064), [1717–1769](https://github.com/moos-ivp/moos-ivp/blob/023fb54e00692611ecee2cd6f1138d10eec2127e/ivp/src/pHelmIvP/HelmIvP.cpp#L1717-L1769). Core MOOS's [registration path](https://github.com/themoos/core-moos/blob/24942423ce3f3ebdb3cee876edb71bdd04dc5555/Core/libMOOS/DB/MOOSDB.cpp#L925-L968) supplies retained values. The [prior evidence index](../../research/2026-10-28-contract-priorities/evidence.md) links hashes, manual extracted-text spans and further ordering details; this is source-backed compatibility analysis, not runtime qualification.
