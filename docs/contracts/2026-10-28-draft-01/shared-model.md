# Shared identity, timing, and execution model

Status: proposed, draft-01, October 28, 2026. This vocabulary is shared by all four contracts. Names below are semantic fields, not a chosen MOOS variable set or serialization schema.

## 1. Identity and epochs

| Field | Draft meaning | Invalidation boundary |
| --- | --- | --- |
| `vehicle_id` | Configured logical vehicle identity, resolved against available physical/link evidence | A matching label alone cannot prove a physical vehicle |
| `vehicle_session` | Bridge-assigned identity/connection continuity context for this vehicle | Disconnect, identity ambiguity, detected FC restart, or other profile-defined continuity loss creates a new session |
| `bridge_epoch` | Unique bridge process incarnation | New on process restart; old grants/ingress bindings are invalid |
| `producer_id` / `producer_session` | Logical origin and one validated incarnation/leadership interval | Producer restart or leadership transfer changes session |
| `ingress_id` | Receiver-established association with a permitted producer/session/path | Expiry, trust/path loss, or session mismatch disables effectful admission |
| `authority_epoch` | One acquisition attempt and its possible active-control interval | New on every acquisition, takeover, or reacquisition; never reused to revive old intent |
| `grant_id` | Bridge-owned reference to permitted operations in one authority epoch | Lease expiry, revocation, state change, or binding loss removes the associated permissions |
| `operation_permit_id` | Optional separate bridge-owned permission for a bounded transition/release/failure operation | Its own deadline/guards apply; it cannot revive a revoked producer grant |
| Sequence or request IDs | Object-specific ordering/correlation within their stated identity scope | Never assumed globally unique without the surrounding scope |

**Draft requirement SH-1:** every effectful request and intent binds to the intended bridge epoch and vehicle session, as well as its issuer session. The bridge resolves these bindings before admission and rechecks relevant bindings before egress. Standard MAVLink messages may not carry these application IDs; the association must remain in bridge state.

A vehicle session is deliberately conservative: reconnecting to the same apparent vehicle can still create a new session and require explicit reconciliation. Source system/component IDs, network endpoints, MOOS communities, and process names are distinct fields. Observations record the actual evidence and its strength; missing physical identity is not silently replaced with a confident match. [Q-SH-02](open-questions.md#shared-questions).

IDs and advertised grant references are not secrets or authentication credentials by themselves. The draft assumes a configured trust boundary for accepting producer sessions. An envelope claiming to be a helm, or a MOOS source-name match, is not sufficient proof of that boundary. Trust/path validation remains a design blocker for effectful profiles. [Q-SH-01](open-questions.md#shared-questions).

## 2. Clocks and origin freshness

| Time | Purpose | Comparison rule |
| --- | --- | --- |
| Source sample/decision time | When the original producer made the observation/decision | Carries clock domain, source session, and known uncertainty; may be unknown |
| First trusted acquisition time | Earliest timestamp the bridge's trusted ingress can establish for this object | Receipt freshness is weaker than measurement/decision freshness |
| Bridge receipt and publication times | Processing/diagnostic events | Re-publication does not create a new sample or decision |
| Bridge monotonic deadlines | Local command admission, intent use, grant expiry, dwell | Valid only inside the named bridge epoch |
| MOOS/UTC/simulation time | Logs, UI, experiment correlation | Cannot be substituted for monotonic safety durations without a qualified mapping |

**SH-2:** unknown time remains unknown. Do not manufacture a source sample time from a callback or forwarding timestamp. An observation can expose receipt age while marking measurement age unknown. An intent or effectful request needs a profile-approved upper bound on origin age before it can be used.

Two candidate mechanisms meet that last requirement at the semantic level:

1. **Bounded source-clock mapping.** A validated mapping expresses source time in the bridge clock with bounded uncertainty, tied to both sessions. Expiry uses the conservative, earliest plausible deadline after accounting for uncertainty and transport age. Mapping discontinuity invalidates the bound.
2. **Bridge-issued freshness window.** An active exchange gives the original permitted producer a window/nonce bound to bridge, vehicle, and producer sessions, with fixed bridge-owned `accept_until` and `use_until` deadlines. A reply references that window. A window must exist before the producer forms the eligible object; it cannot be attached by a relay to old cached data. Duplicate replies never extend the fixed deadlines. This supplies a bounded window of permissible origin, not a precise measurement timestamp.

These are candidate mechanisms, not selected implementations. Neither works without a trusted producer that truthfully validates its decision/sample provenance. The bridge cannot prove from a packet alone that a solver actually ran. If neither a bounded origin-age mapping nor a qualified freshness-window exchange is available, this draft rejects effectful command/intent use; telemetry may remain available with weaker age metadata. [Q-SH-03](open-questions.md#shared-questions).

**SH-3:** effective expiry is the earliest applicable bound: producer validity, freshness window, ingress validity, authority grant, profile limit, and any operation-specific deadline. Repeated transport receipt, retransmission, logging, or serialization must not move it later. A fresh, higher-sequence decision may create a new object within current permissions; that differs from forwarding the same object again.

The final design must define timestamp precision, comparison tolerance, future-time rejection, sequence wrap, and clock-reset behavior. There are no numeric default durations in this draft.

## 3. Profiles and capabilities

A future versioned execution profile must name: autopilot/release/vehicle applicability; supported operation and intent kinds; target resolution; coordinate and vertical references; quantities/limits; required observations and age/quality bounds; authority-entry/release behavior; preparation targets if needed; scheduler/queue limits; confirmation predicates; failure responses; and external-override assumptions.

**SH-4:** unknown or unsupported combinations are explicitly rejected for effectful use. Capability advertisement lists only operations the profile and current session can support; it separates “known interface,” “configured permission,” “currently eligible,” and “validated coverage.” A raw protocol escape hatch is another operation scope subject to policy, not an implicit bypass.

Read-only telemetry can operate without helm authority. Other operation classes can have independent authorization: querying parameters, changing configuration, arming, changing mode, starting a mission, bulk transfer, and streaming navigation are not one undifferentiated permission. [Authority operation scopes](authority.md#2-operation-scopes).

A bounded bridge-owned release/failure operation uses its own operation permit and fresh prerequisites, referring to the triggering authority epoch for history. It does not run under a revoked producer grant or renew that producer's intent. The same identity/expiry/egress rules still apply.

## 4. Ordering, history, and replay

Each object type defines its own ordering scope. A transport packet number or re-notification is not automatically a new application object. Producers preserve IDs and origin validity through relays. Consumers treat gaps, duplicates, and unknown sessions explicitly; out-of-order data may remain historical evidence without becoming current control input.

**SH-5:** request deduplication and active grants must not be silently forgotten while their old messages could still be admissible. An implementation must either retain the required ledger or invalidate the old ingress/session bindings and reject old work. Durable reconciliation of an uncertain effect remains unresolved; this package does not promise exactly-once physical execution. [Q-SH-04](open-questions.md#shared-questions).

State snapshots are useful for late readers, but events/results need a defined retention and query path. A receiver's connection or registration must not itself execute a retained request. A cached status is only an as-of claim and must include enough epoch/expiry information for consumers to stop using it later.

## 5. Scope of transmission guarantees

**SH-6:** the implementation must check permission, session/epoch, expiry, and target before effectful enqueue and again at the last controllable egress/dequeue boundary. Queue entries retain their bindings and deadlines. Admission and grant revision changes need a defined ordering/barrier so a revoked epoch cannot keep producing newly eligible sends.

Cancellation or revocation cannot recall bytes already handed off or in flight. Ordinary MAVLink payloads may not let the FC distinguish the application authority epoch. A profile must therefore establish acceptable queue/drain/transit bounds or a stronger mechanism before claiming a clean handoff. Closing one link cannot be assumed to stop an SDK retry queue or another link. [Q-AUTH-04](open-questions.md#authority-questions).

Proposed post-revocation guarantees are local: no new routine navigation is admitted under the revoked epoch, controllable queued work is invalidated, and residual transmission uncertainty is reported. Claims about FC behavior, absence of already-sent commands, or exclusive global ownership require separate evidence.

## 6. Consistent vocabulary across the contracts

- **Admitted:** local schema/identity/freshness/policy checks permit proceeding; not FC acceptance.
- **Queued or handed off:** work reached a named local egress boundary; not wire receipt.
- **Protocol result:** the available protocol/library outcome, with its evidence and limitations.
- **Observed predicate:** a fresh observation satisfies a declared condition; not proof of which caller caused it.
- **Task completed:** a specific operation's declared completion rule is satisfied.
- **Grant active:** the bridge currently permits a named producer's scoped transmissions while all guards hold; not a claim to arbitrate external actors.

See the [review scenarios](review-scenarios.md) for how these distinctions interact. All proposed obligations still need implementation feasibility review and later validation.
