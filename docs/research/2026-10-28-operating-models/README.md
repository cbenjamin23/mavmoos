# Operating models and three source traces

October 28, 2026. This is the first completed research pass under the [analysis plan](../analysis-plan.md): operating-model comparison and three representative static source traces. It is not completion of the entire architecture study or acceptance of an implementation design.

## Read the findings

| Report | Question answered |
| --- | --- |
| [Operating-model comparison](operating-model-comparison.md) | Which responsibilities belong to middleware, autonomy, bridge, controller, and hardware? |
| [Trace 1: Position telemetry](trace-01-position-telemetry.md) | How do bytes become application observations, with what identity, frame, timestamp, and freshness? |
| [Trace 2: Acknowledged ARM command](trace-02-acknowledged-command.md) | How do request admission, transport, ACK, result, and observed state differ? |
| [Trace 3: Continuous setpoints](trace-03-continuous-setpoints.md) | Who renews intent, schedules transmission, converts quantities, and responds to upstream failure? |
| Evidence and baselines | What exact source revisions and manual sections support the findings? |
| Source manifest | Which 48 inspected files and content hashes were recorded? |

The selected MIT manuals are archived with checksums and numbered text extractions. The plugin workflow was used to select them from the live MIT index. The web reader initially failed, but direct downloads succeeded. The MOOS application callback diagram on PDF page 5 of the MOOS chapter was visually checked; this is not a claim of full visual review of all manuals.

## Main findings

1. **The reusable design boundary is larger than message translation.** MAVROS separates transport, routing, UAS state, functional plugins, and external application behavior. MOOS adds a commonly used helm/control workflow, but the bridge must still operate without the helm. The direct BlueBoat hardware boundary is another distinct operating model. [Comparison](operating-model-comparison.md).
2. **MOOS already carries useful metadata and supports compound payloads.** It is not limited to doubles/strings, and notification delivery is not only a last-value read. The current DB also retains state and supplies it on registration. A new receiver must distinguish an old state value from a new operation. [Trace 1](trace-01-position-telemetry.md), [trace 2](trace-02-acknowledged-command.md).
3. **The selected MAVROS velocity plugin does not maintain its own stream.** It sends when its ROS subscription callback runs. An external publisher's cadence and a bridge-owned retransmission cadence are different choices, with different liveness implications. [Trace 3](trace-03-continuous-setpoints.md).
4. **Command result contracts differ even inside MAVROS.** The traced ARM helper waits for one ACK, with no helper-level retransmission/progress lifecycle; its COMMAND_INT and SetMode paths report send-path results instead. Actual observed state remains separate. MAVSDK's pinned fork has different matching/retry behavior. [Trace 2](trace-02-acknowledged-command.md).
5. **The current Copter helm path in pArduBridge is richer than its prose describes.** It streams a NED velocity target, supplies vertical feedback, and conditionally commands yaw. Plane uses a different, change-driven branch. This resolves the earlier yaw-only documentation discrepancy for the helm stream. [Trace 3](trace-03-continuous-setpoints.md#6-copter-translation-is-a-vehicle-specific-control-law).
6. **Fresh publication is not fresh observation or fresh intent.** pArduBridge can republish cached navigation state with current MOOS time. The inspected BlueBoat brokers can repeatedly forward cached values and renew local receive timers. These are conditional failure paths derived from source, not reproduced deployment incidents. [Trace 1](trace-01-position-telemetry.md), [trace 3](trace-03-continuous-setpoints.md).
7. **Heartbeat, decision validity, and permission to send are separate.** The helm heartbeat occurs before the solver path, parked/standby states can communicate, and all-stop can emit fresh zeros. The selected source also has a normal-output/all-stop prefix asymmetry. Numeric freshness alone cannot express authority. [Comparison](operating-model-comparison.md#4-helm-intent-is-not-an-actuator-command-or-a-health-lease).
8. **Thread safety does not imply coherent intent.** pArduBridge's wrapper protects individual member accesses; the sender's freshness check and separate heading/speed/altitude reads still do not identify an atomic decision snapshot. [Trace 3](trace-03-continuous-setpoints.md#4-pardubridge-input-storage-and-freshness).
9. **Control laws and reference frames need explicit ownership.** A marine PID/actuator chain and an FC velocity/stabilization chain operate at different levels. Course is not automatically yaw; relative altitude is not established terrain AGL. The bridge's small vertical controller is already a substantive design choice. [Comparison](operating-model-comparison.md#5-choose-the-control-level-deliberately).
10. **Transport recovery must not silently resume authority.** Routing, target selection, command results, observed state, input expiry, and restart/session identity need independent contracts. Neither an open connection nor a live sender proves that the right producer currently controls the right vehicle. [All three traces](trace-03-continuous-setpoints.md#9-proposed-contracts-and-later-verification).

## What follows from the evidence

The next design work should produce four reviewable contracts:

| Contract | Decisions it must make |
| --- | --- |
| Observation | Identity, quantities, origin/frame/units, sample/receipt/publication time, validity and age |
| Command transaction | Caller/target/session, IDs, expiry, duplicates/replay, admission, protocol result, observation and completion |
| Navigation intent | Supported quantities, coherent updates, producer identity, validity interval, legacy scalar compatibility |
| Authority/execution | Requested versus observed ownership, handoff evidence, scheduling budget, external override, failure and restart behavior |

Keep the one-modular-application preference provisional. Retain both direct MAVLink/transport reuse and MAVSDK as candidates until their capability/dependency tradeoffs are studied. Do not select a dynamic loader, universal fallback mode, common timeout, or process isolation mechanism based solely on these examples.

## Validation limits

This pass verified source locations, local-file/revision matches, documentation provenance, numerical mapping examples, archive hashes, and documentation navigation. It did not build or execute any bridge, run SITL, measure timing, send FC commands, or qualify hardware. The trace endpoint is the bridge/library wire boundary and application-visible observations; the autopilot's internal execution was not traced. The full protocol/vehicle support matrix, mission/parameter/file-transfer paths, security/concurrency audit, complete reconnect design, and architecture selection remain open.

Existing-source observations were recorded without modifying those projects. MAVMOOS still has no implementation, dependencies, build system, or runnable missions. Capability status remains candidate/unimplemented/unvalidated; completed research is not runtime evidence.
