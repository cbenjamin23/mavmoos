# 0017: Vehicle capability providers and independent mission data

Date: November 4, 2026. Status: accepted implementation scope and implemented architecture; qualification is limited to the evidence in the current batch ledger.

## Context and choice

The user approved an ambitious three-agent batch after the capability-oriented refactor: coherent Copter operation strategies, a mission-data provider, and a second vehicle profile. Related operator functionality belongs together; the single control owner must remain explicit. Stock pHelmIvP stays unchanged, default receive-only remains, and no real vehicle control is authorized.

One host now supports `telemetry`, `parameters`, `missions`, and exactly one of `arducopter_control` or `ardurover_control`. Startup descriptors and the configured catalog expose supported interfaces; readiness, permission and tested qualification are separate. Modules are trusted, process-resident C++ code using matching development ABI 6, with no live unloading or security isolation.

Copter owns a versioned operation interface alongside the unchanged legacy channels. Native FC actions, persistent bounded Guided-position goals, and configured stock-helm strategies share the same authority and send services. Helm strategies publish ordinary StationKeep behavior conditions/updates; they neither replace the helm nor claim atomic solver generations. Facing is course by default, with Keep/Heading options. Each strategy states who supplies navigation decisions, how completion is observed and what cancellation can retire.

Rover supplies distinct mode/identity/readiness/encoding policy and a true planar helm contract. Ordinary motion requires horizontal state and estimator evidence, not Copter altitude, home or airborne state. Native RTL needs home. Initial qualification targets a ground Rover; BlueBoat/surface-boat support is not inferred.

Mission storage is an independent protocol provider, rather than part of the motion owner. Download/progress does not acquire navigation. Upload/clear is limited to a disarmed idle owned simulator, serialized and followed by readback. Mutation fences commands/acquisition/parameter writes while active. No AUTO/mission-start/execution permission is added. Partial upload is non-atomic: cancellation, timeout and disconnect cannot imply rollback.

## Shared host consequences

- Typed navigation proposals declare dimensions, target kind and facing; typed mission proposals declare a stable operation deadline and short phase expiry.
- Mission and parameter work share one fair protocol slot per tick beneath control work. Queueing is bounded and final state/time/cancellation checks remain host-owned.
- A local cancellation remains effective after dequeue and encoding until the OS handoff. Grant revisions prevent an earlier target/recovery hold crossing a new permission boundary. Already-sent effects remain uncertain.
- Observational services expose current grant/busy state and the single configured telemetry projection. They grant no authority and do not renew observations.
- Effectful routes require the exact matching native executable/PID and owned loopback UDP peer; Copter and Rover guards are separate.

## Alternatives and limits

A generic sender plugin per MAVLink message would split related control ownership and make operation semantics harder to discover. A monolithic FC plugin absorbing all parameters, storage and payload services would erase useful independent lifecycles. Runtime component duplication, upstream helm changes, general serial/hardware routes and mission execution remain excluded.

MAVROS informs transport/context/plugin separation and independent mission/parameter protocol services. pArduBridge remains the primary UAV workflow reference; the stock-helm adapter preserves ordinary MOOS use. Pinned ArduPilot source determines actual modes, target interpretation and storage normalization; it is tested rather than assumed transferable across vehicles.

Evidence, source pins, preserved failures and remaining questions are in the batch ledger, [plan](../plans/2026-11-04-parallel-capabilities/README.md), [Copter](../plugins/arducopter_control.md), [Rover](../plugins/ardurover_control.md) and [mission](../plugins/missions.md) guides. No field, full-MAVROS-parity or real-time guarantee follows.
