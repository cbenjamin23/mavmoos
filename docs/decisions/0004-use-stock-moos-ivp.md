# 0004: Use MOOS-IvP as supplied

Date: October 29, 2026. **Status: Accepted architectural constraint.** This records the user's explicit instruction: “definitely no changes to the helm. we take moos ivp as it is.” Implementation remains outside the authorized scope.

## Decision

MAVMOOS must work with stock MOOS-IvP and pHelmIvP. Do not require patches, forks, new producer hooks, new helm message formats, injected solver-generation tags, or a custom behavior whose purpose is to change the helm's output contract. Ordinary supported mission/helm configuration and existing published interfaces remain usable. All bridge adaptation belongs in MAVMOOS.

The current architecture sketch uses one MOOS application with built-in feature modules, as requested. Its proposed C++ interfaces, callbacks, dependency/library selection, timing values and vehicle profiles are not accepted implementation details merely because this constraint is accepted.

## Consequences

Use existing DESIRED_* publications and existing helm state/all-stop evidence. Preserve their source/time metadata and state the actual guarantee. Grouping scalars into a bridge-owned cache snapshot does not prove one solver generation. Stock compatibility must not be advertised as satisfying draft-01's stronger producer-atomic/generation-verified navigation contract without independently established evidence.

The complete producer-envelope proposal in [0003](0003-coherent-helm-and-origin-freshness.md) is therefore outside the current MAVMOOS design where it requires changing the helm. A receiver-issued exchange that requires new participation by pHelmIvP is likewise unavailable for this compatibility path. Earlier research remains a historical record of alternatives and source findings, not an instruction to implement them.

Age limits, source/clock assumptions, explicit permission, restart behavior, observed FC prerequisites, and vehicle-specific loss responses still need to be specified. The constraint removes an integration mechanism; it does not establish a stronger coherence or freshness claim than the existing interface supplies.

## Alternatives and evidence

Producer modification was investigated in the [earlier helm study](../research/2026-10-28-contract-priorities/helm-output.md) and made conditional in the [clarification](../research/2026-10-29-design-clarifications-and-plugins.md). The user has now excluded it. The [built-in module sketch](../design/2026-10-29-built-in-modules/README.md) applies this decision. Existing source/manual evidence remains versioned separately; no source checkout was changed or runtime behavior validated.
