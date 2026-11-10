# 0005: Keep navigation eligibility and acquisition retry policy in the host

October 29, 2026. **Status: Proposed.** No backend, dependency version, queue extension or executable scope is accepted by this record.

## Context

The [initial ArduCopter profile](../profiles/2026-10-29-arducopter-draft-01/README.md) needs fresh target observations, bounded original input/work age, explicit Guided acquisition and loss/re-enable handling. The [backend comparison](../research/2026-10-29-backend-comparison/README.md) traced MAVSDK, libmavconn and generated/direct MAVLink against those needs. Stock MOOS-IvP remains unchanged under [accepted decision 0004](0004-use-stock-moos-ivp.md).

MAVSDK helpers can own repeating setpoints and command retries independently of host permission. Its raw/filter paths are useful but need exact-work association, settlement and transport-specific limits. libmavconn provides raw transport with an internal FIFO and no public per-work expiry/selective cancellation receipt. Generated MAVLink supplies codec/parsing; an owned transport must implement all remaining policy and I/O behavior. None is execution-qualified unchanged.

## Proposal

Keep navigation selection/cadence, per-field/observation eligibility, control generations, original deadlines and acquisition retry policy in MAVMOOS host services. Effectful backend work must preserve those bindings through a declared final gate, expose honest local outcome evidence and report cancellation/residual scope. A current enabled bit cannot authorize old queued work.

Evaluate libmavconn first as a transport reuse candidate, conditional on an achievable queue/write integration carrying exact-work metadata, expiry checks, selective invalidation and receipts. Compare that adaptation/packaging cost with a small direct UDP-first transport. Keep MAVSDK raw-message integration as an alternative when its service reuse outweighs the association/queue/helper-control work. No extension, fork or dependency installation is authorized or performed here.

For initial-profile review, recommend one wire Guided attempt without automatic additional mode attempts; a bounded retry policy is an explicit alternative requiring profile clarification. Never inherit library defaults implicitly. Broader command services may need reliable-delivery retries under their own original deadlines/permission; do not create conflicting SDK and host transaction managers for the same work.

## Alternatives and consequences

- Default SDK Offboard/action composition offers convenience but changes this profile's acquisition/stream/yaw/loss/retry semantics. It would need an explicitly different profile or additional control mechanisms.
- A host check before stock libmavconn FIFO insertion preserves an earlier admission boundary but does not establish write-time expiry or selective recall. We should not rename that weaker claim as qualification.
- Direct I/O makes local boundaries owned and visible but adds implementation, maintenance and transport/service-protocol responsibilities. It does not remove kernel/network/FC uncertainty.
- SDK raw/filter integration may qualify a narrower path without rewriting its services, if association, retry suppression/settlement and downstream residual behavior are demonstrated. The comparison does not reject MAVSDK categorically.

The proposed ownership boundary keeps one app with built-in modules and leaves broad telemetry/mission/parameter capabilities as candidates. No change to stock helm semantics, new producer protocol or universal physical-stop guarantee follows.

## Evidence and remaining questions

The comparison's [52-file evidence](../research/2026-10-29-backend-comparison/evidence.md), [source traces](../research/2026-10-29-backend-comparison/source-traces.md) and [backend criteria/scenarios](../research/2026-10-29-backend-comparison/requirements-and-review.md) identify the concrete gaps. Select an actual dependency/platform/link/firmware context, resolve mode retry policy, qualify exact-work gates/settlement/partial frames and estimate packaging/maintenance cost before accepting a backend or implementation scope. The archived profile remains unchanged.
