# Decision records

| Record | Status | Meaning |
| --- | --- | --- |
| [0001: Architecture-only repository](0001-architecture-only-scope.md) | Historical accepted scope | Documentation-only phase; limited P0-A/P1 scope change in 0007 |
| [0002: One modular bridge](0002-modular-bridge-preference.md) | Provisional preference | Current sketch uses one app with built-in modules; detailed mechanisms remain proposals |
| [0003: Complete helm outcomes and bounded origin freshness](0003-coherent-helm-and-origin-freshness.md) | Proposed / constrained | Historical strict proposal; routes requiring helm changes are excluded by 0004 |
| [0004: Use MOOS-IvP as supplied](0004-use-stock-moos-ivp.md) | Accepted constraint | User explicitly excludes helm/upstream changes; bridge adapts existing interfaces |
| [0005: Host-owned navigation eligibility and acquisition retries](0005-host-owned-navigation-egress.md) | Proposed | Thin transport evaluation with explicit gates/settlement; no backend selected |
| [0006: First single-peer UDP candidate](0006-first-single-peer-udp-candidate.md) | Proposed / conditional | Refines the transport recommendation for a narrow UDP topology; no implementation or backend selection |
| [0007: First receive-only increment](0007-first-receive-only-increment.md) | Accepted limited scope | P0-A/P1 implementation and fixture validation; no MAVLink transmission or helm changes |
| [0008: P2 bounded-egress laboratory](0008-p2-bounded-egress-laboratory.md) | Accepted P2 scope | Fake-I/O/owned-loopback implementation and validation; app vehicle control remains disabled |
| [0009: Backseat deployment and Copter policy defaults](0009-backseat-and-copter-policy-defaults.md) | Accepted architectural direction | Backseat host, configurable course-facing default, and automatic recovery after bounded normal handoffs; implementation details remain open |
| [0010: Modular Copter SITL workflow](0010-modular-copter-sitl-workflow.md) | Accepted implementation scope | Startup-loaded modules, shared services, complete isolated simulated lifecycle; no hardware control |
| [0011: Faults, parameters and Linux](0011-faults-parameters-and-linux.md) | Accepted implementation scope | Selected process-fault recovery, typed parameter service and clean source-built Linux executions; simulation only |
| [0012: Ingress age and owner continuity](0012-ingress-age-and-owner-continuity.md) | Accepted continuation scope | Conservative socket-queue bound, ABI 3 input boundary, deadline-first command handling and recovery advice; simulation only |
| [0013: Concurrent features and authority](0013-concurrent-features-and-command-authority.md) | Implemented continuation | ABI 4 normal-resume permits, command-operation gates, hold cancellation and write interlocks; simulation only |
| [0014: Paced parameter inventory](0014-paced-parameter-inventory.md) | Implemented continuation | Bounded indexed reads by default; legacy streamed list limited to ground bench; fixed original deadlines |
| [0015: Resident startup feature code](0015-resident-startup-feature-code.md) | Implemented loader policy | Process-lifetime code mappings with ordinary object destruction; no hot unloading/replacement |
| [0016: Capability-oriented providers](0016-capability-oriented-plugin-boundaries.md) | Accepted principle / implemented structural refactor | One ArduCopter control provider groups native actions and helm guidance; independent telemetry/parameters, ABI 5 descriptors and pure backend |
| [0017: Vehicle providers and mission data](0017-vehicle-providers-and-mission-data.md) | Accepted scope / implemented architecture | ABI 6 coherent Copter strategies, planar Rover, independent mission storage, shared bounded effect services; exact evidence in the batch ledger |
| [0018: Observations and final handoff](0018-observations-and-final-handoff.md) | Implemented continuation | ABI 7 conservative observation timing, optional read-only status, post-endpoint-proof expiry/eligibility checks and native partial-mission cancellation evidence |
| [0019: Attitude and concurrent observers](0019-attitude-and-concurrent-observers.md) | Implemented continuation | Optional fused attitude, unchanged ABI 7/authority/NAV, and observer coexistence with stock helm and independent protocol reads |
| [0020: RC observations and stream recovery](0020-rc-observations-and-stream-recovery.md) | Implemented continuation | Optional read-only input/output observations, bounded local-false stream retries and one fair protocol budget after control |
| [0021: Observer recovery, protocol faults and distance](0021-observer-recovery-protocol-faults-distance.md) | Implemented continuation | Reliable reset time, fail-closed initial host clock, mutation uncertainty and optional identified range observations; simulation only |

Use a new numbered record for a consequential choice. Include date, status, context, choice or proposal, alternatives, consequences, evidence, and remaining questions. Suitable statuses are proposed, provisional preference, accepted, rejected, and superseded. Link a superseding record instead of erasing the history.

Reserve “accepted” for an explicitly settled choice. Discussed ideas, feature candidates, and research results do not automatically become accepted decisions. Use the capability map for delivery and validation status.

The [contract package, draft-01](../contracts/2026-10-28-draft-01/README.md) contains review proposals and an explicit question register. Creating it does not accept its requirements or resolve the modular-bridge preference. Later decisions should cite the affected contract revision and evidence.

The [October 29 clarification](../research/2026-10-29-design-clarifications-and-plugins.md) distinguishes practical freshness/restart requirements from optional producer/window mechanisms and examines MAVROS plugin loading. Records 0002/0003 include dated review notes; neither is promoted to Accepted.

The later [module sketch](../design/2026-10-29-built-in-modules/README.md) applies accepted constraint 0004. Its interfaces and flows are proposed architecture, not accepted ABI/timing details or implementation evidence.

The [UDP queue assessment](../research/2026-10-29-udp-queue-assessment/README.md) retains 0005's host-policy ownership and compares the libmavconn changes with direct UDP. Record 0006 proposes a narrower conditional first-candidate preference; neither record is promoted to Accepted.

The [direction review and phased plan](../plans/2026-10-29-direct-udp-copter/README.md) use direct UDP as the user's requested planning target. They add staged readiness/acceptance criteria without another backend selection or promotion of 0005/0006 to Accepted. That dated plan preserved architecture-only scope. The later limited implementation authorization is recorded in 0007; the stock-helm constraint remains in force.

- [0022: Local CI, firmware identity and capability evidence](0022-local-ci-firmware-and-capability-evidence.md): accepted direction; actual qualification stays in the dated ledger.

- [0023 — Explicit simple AUTO, independent IMU and portable qualification](0023-simple-auto-imu-and-portable-qualification.md): current simulation expansion; exact qualification remains separate.

- [0024 — Checked external plugin startup and complete observer reference](0024-checked-external-plugin-authoring.md)

- [0025 — External command authoring and durable AUTO uncertainty](0025-external-command-and-auto-restart.md)

- [0026 — Traffic, plan inspection and flight logs](0026-traffic-plan-inspection-and-flight-logs.md)

- [0027 — One Rover provider for ground and surface boats](0027-rover-ground-and-boat.md)

- [0028 — Passive observations and scoped payload components](0028-passive-observation-and-payload-routes.md)

- [0029 — A rooted, read-only MAVLink file provider](0029-read-only-file-service.md)

- [0030 — Conventional Plane guidance and watchdog handback](0030-plane-guidance-and-watchdog.md)

- [0031 — Shared payloads and a dedicated PX4 Offboard provider](0031-shared-payloads-and-px4-offboard.md)

- [0032 — Vectored Sub control and the final shared v1 workflows](0032-bounded-sub-and-shared-providers.md)

- [0033 — Cross-firmware vehicle coverage](0033-cross-firmware-vehicle-coverage.md)

- [0034 — PX4 airplane and Rover control interfaces](0034-px4-airplane-and-rover-interfaces.md)

- [0035 — PX4 Ackermann motion, stop and mission ownership](0035-px4-ackermann-stop-and-ownership.md)

- [0036 — Core vehicle lifecycle and phase evidence](0036-core-vehicle-lifecycle-and-phase-evidence.md)

- [0037 — QuadPlane forward helm and standard Copter parity](0037-quadplane-forward-and-copter-frame-parity.md)

- [0038 — QuadPlane hover and the local release matrix](0038-quadplane-hover-and-local-release-matrix.md)
