# Contract priorities and focused architecture review

October 28, 2026. Status: completed static investigation and proposed design recommendations. Architecture only. No source repositories, bridge code, missions, dependencies, or vehicle configuration were changed, and no runtime experiments were performed.

This package follows the [operating-model study](../2026-10-28-operating-models/README.md) and [contract draft-01](../../contracts/2026-10-28-draft-01/README.md). The user requested prioritization of all 25 open questions and investigation of the most important ones, especially coherent helm output and freshness. Draft-01 is preserved unchanged. This review narrows choices and records evidence; it does not accept the proposed interface or claim that all questions are closed.

## Main recommendations

1. Prefer a **complete producer-side decision envelope** for strict helm navigation. Emit it after final decision eligibility checks; include explicit invalid outcomes and bind the cycle to its producer, session, authority generation, and original age. Standard scalar arrival grouping does not establish this guarantee.
2. Prefer **fixed bridge-issued freshness windows**, bound before the decision begins, for an initial effectful profile whose source clocks lack a demonstrated bound. Neither relay receipt nor a setpoint scheduler may extend a window. A calibrated source-clock route remains an alternative.
3. Keep **decision coherence, decision age, observation age, authority, and egress age separate**. Passing one check does not satisfy the others. A solver can make a new coherent decision using old observations.
4. Treat all-stop/invalidity and ownership changes as **eligibility events**, not numerical navigation commands. A standby takeover needs a new bridge authority epoch; delayed primary output must not regain permission through arrival order.
5. Carry expiry and generation bindings through the final controllable send boundary. State the residual queue/transit bound separately. Restart/reconnect invalidation prevents admission of old work but cannot undo an uncertain physical effect.

These are proposals for adoption. [Decision 0003](../../decisions/0003-coherent-helm-and-origin-freshness.md) records the preferred direction and alternatives with status **Proposed**.

## Read the analysis

| Document | Purpose |
| --- | --- |
| [All 25 priorities](priorities.md) | Ordered work queue, reasons, dependencies, and what would close each question |
| [Coherent helm output](helm-output.md) | Existing publication sequence, summary/iteration limitations, companion feasibility, recommended producer interface |
| [Origin freshness](freshness.md) | Clock/window alternatives, age bounds, relay rules, freshness budgets, and numerical paper examples |
| [Boundary, handoff, and recovery](boundary-and-recovery.md) | Trusted ingress, invalidity, standby arbitration, egress revocation, vehicle sessions, and uncertain effects |
| [Paper review cases](review-cases.md) | Counterexamples and proposed acceptance criteria; none executed |
| [Evidence and source manifest](evidence.md) | Primary documentation, pinned source spans, local provenance, and limits |

## What was answered, and what remains open

The focused pass covers ranks 1–8: Q-NAV-03, Q-SH-03, Q-SH-01, Q-NAV-02, Q-AUTH-05, Q-AUTH-04, Q-SH-02, and Q-SH-04. It establishes the inspected helm's publication order and report format, explains why the existing signals alone do not provide the proposed strict interface, and develops a concrete candidate freshness mechanism with stated premises. It also works through the session and permission rules needed to preserve those properties during failure and recovery.

The full questions remain open for adoption and qualification. Producer integration, an execution profile, numeric age limits, physical identity evidence, trust enforcement, transport cancellation bounds, and transaction durability have not been selected or demonstrated. Ranks 9–25 are prioritized with explicit completion evidence; this package does not present them as fully investigated.

## Recommended next architecture work

Review the proposed direction in decision 0003, then write one narrow execution profile with an explicit vehicle/autopilot release and control level. Complete its intent quantities, frames, observation requirements, authority transitions, preparation behavior, and timing budget together. Evaluate library queue/cancellation behavior against that budget before selecting a dependency. Work the command correlation/result/deduplication questions before adding effectful discrete operations.

The next contract revision should incorporate adopted choices and retain unresolved thresholds/mechanisms explicitly. Implementation remains outside the authorized scope.
