# 0003: Complete helm outcomes and bounded origin freshness

Date: October 28, 2026. **Status: Proposed.** Architecture recommendation only. Adoption, producer integration, execution profiles and runtime qualification remain open.

**Current applicability:** [accepted decision 0004](0004-use-stock-moos-ivp.md), October 29, excludes routes that require helm changes. Producer-envelope/new-helm-exchange alternatives below are historical proposals outside the current stock compatibility design. Other timing/permission principles remain subject to their stated evidence and profile requirements.

## Context

[Contract draft-01](../contracts/2026-10-28-draft-01/README.md) requires coherent expiring navigation snapshots but leaves Q-NAV-03 and Q-SH-03 unresolved. The [focused follow-up](../research/2026-10-28-contract-priorities/README.md) found that the inspected helm posts state before solving, iteration and journal summary before final output checks, then independent decision scalars. The summary has baseline/continuity, missing-value, precision and policy qualifications. Relay receipt and repeated setpoint sends do not bound original decision age.

## Proposal

Prefer a producer-side complete outcome envelope for the initial strict navigation path. Form it after final decision validity/completeness checks. Carry all required quantities and explicit missing/uncontrolled dimensions, valid/invalid outcome and reason, producer incarnation, decision sequence, profile/reference revisions and bridge/vehicle/authority bindings. Cover early-return/state-invalidity paths. A new genuine cycle can renew identical numeric intent; retransmission preserves identity and expiry.

Prefer fixed bridge-issued freshness windows when source clocks lack a qualified bound. The trusted producer must bind its window and authority context before the decision's designated input-capture/solve start. Fixed admission/use deadlines are owned by the receiver and never rebased on forwarding or receipt. A new window cannot be attached to a cached or already-started old decision. A qualified source-clock interval remains an alternative, using the oldest plausible origin for expiry.

Keep dynamic observation age, decision age, permission and queue/transit age as separate eligibility checks. Explicit invalidity removes the producer's prior intent. A takeover/reacquisition uses a new authority generation and newly bound decision. Check permission/deadlines at the final controllable egress boundary and report residual work beyond it.

## Alternatives considered

- **Tagged scalars plus explicit commit/manifest:** can provide generation proof, but also needs producer changes and introduces partial assembly/retention complexity. Retain if a concrete constraint favors it.
- **Companion summary interpreter:** potentially reconstructs coherent solver vectors under qualified baseline, continuity, parsing, precision and policy assumptions. It needs separate origin freshness and is not automatically a certificate of final native output eligibility.
- **Arrival/time grouping of ordinary scalars:** useful legacy interoperability, but insufficient evidence for the proposed strict generation/age claims.
- **Calibrated source clocks:** viable where a finite mapping error/drift/reset bound is demonstrated. Do not treat synchronization configuration or plausible timestamps as that demonstration.

## Consequences and remaining work

The strict path requires a producer integration point or another independently qualified producer contract. This is a maintenance/deployment cost; no upstream patch, fork, behavior plugin, channel name or encoding is selected here. The recommendation does not require every MAVMOOS function or observation-only use to depend on a modified helm.

Before adoption into a contract revision, review the deployment trust model and producer integration approach. Before implementation claims, select a concrete profile, quantities/references, numeric age budgets, phase/failure/preparation behavior, egress/cancellation bounds, session/identity evidence and crash/reconciliation model. Complete operation-scope mediation and command-specific semantics before adding those effects.

The prior draft remains unchanged. No capability is promoted to Planned, Implemented or Validated. This record does not claim safe vehicle behavior, authenticated global ownership, fresh solver inputs, instantaneous revocation, or exactly-once physical execution.

## Evidence

See [helm analysis](../research/2026-10-28-contract-priorities/helm-output.md), [freshness reasoning](../research/2026-10-28-contract-priorities/freshness.md), [boundary/recovery analysis](../research/2026-10-28-contract-priorities/boundary-and-recovery.md), and [versioned primary evidence](../research/2026-10-28-contract-priorities/evidence.md). [Paper cases](../research/2026-10-28-contract-priorities/review-cases.md) are proposed acceptance criteria, not executed validation.

## October 29 review clarification

The [user-question follow-up](../research/2026-10-29-design-clarifications-and-plugins.md) narrows the current recommendation. Stale-command rejection and explicit restart/permission handling remain important, but the first deployment has not established a need for a receiver-issued window protocol or mandatory helm modification. Assess stock-helm compatibility with explicit limits first; choose a stronger producer interface or origin-age mechanism when the selected profile/topology requires it.

The proposal above remains a candidate for the stronger guarantee, with its original rationale preserved. This review accepts no weaker path as satisfying that guarantee, no specific watchdog value and no distributed lease/exchange design. Status remains Proposed.

The user's subsequent explicit instruction to take MOOS-IvP as supplied is recorded in [0004](0004-use-stock-moos-ivp.md). The [current module sketch](../design/2026-10-29-built-in-modules/README.md) therefore documents the weaker stock-interface guarantee directly, without planning a helm modification or declaring the original strict contract satisfied.
