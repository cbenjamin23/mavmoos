# Execution-profile drafts

Profiles refine the proposed module interfaces for a named vehicle/control arrangement. They are architecture documents, not executable configurations, supported firmware claims or validation evidence. Preserve dated drafts and make changes in assumptions reviewable.

| Profile | Status | Scope |
| --- | --- | --- |
| [ArduCopter Guided, draft-01](2026-10-29-arducopter-draft-01/README.md) | Proposed; not execution-qualified | Already-airborne Copter, stock helm course/speed/relative-home altitude, explicit Guided acquisition and latched loss response |

[Accepted constraint 0004](../decisions/0004-use-stock-moos-ivp.md) keeps MOOS-IvP/pHelmIvP unchanged. The [module sketch](../design/2026-10-29-built-in-modules/README.md) defines the proposed host/plugin boundaries. No capability is promoted to Implemented or Validated by a profile draft.

The [Copter backend comparison](../research/2026-10-29-backend-comparison/README.md) evaluates MAVSDK, libmavconn and direct MAVLink against draft-01. It preserves that draft and proposes clearer gate/settlement/retry requirements; no backend or amended execution policy is accepted.

The [UDP queue assessment](../research/2026-10-29-udp-queue-assessment/README.md) maps the same draft to concrete queue/write/fence alternatives and a proposed local-handoff interpretation. Draft-01 remains unchanged; no numerical bound or backend is qualified by the assessment.

The [phased Copter implementation plan](../plans/2026-10-29-direct-udp-copter/README.md) defines staged readiness and future acceptance evidence against that profile. Concrete firmware/parameters, envelopes, policies and tolerances must be recorded before effectful SITL tests can pass. The plan does not amend draft-01 or qualify its candidate numbers.
