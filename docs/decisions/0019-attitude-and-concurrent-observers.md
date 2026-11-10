# 0019: Fused attitude and concurrent observation providers

Date: November 4, 2026. Status: implemented continuation within the user's authorized simulator-only plugin/improvement work; exact qualification layers are recorded in the dated ledger.

The user again authorized clear further work while asleep and encouraged parallel agents. This increment adds useful orientation diagnostics and qualifies observers alongside stock-helm navigation and independent protocol reads. No new flight mode, motion strategy, hardware route, mission execution or parameter-write permission is introduced. Stock pHelmIvP remains unchanged.

## Capability boundary and representation

One optional `attitude` provider groups the FC's fused Euler/quaternion orientation and body angular rates. It is useful independently of the selected vehicle-control owner. Telemetry retains conventional NAV projection; system status retains battery/GPS/sensor/text diagnostics. Attitude owns neither NAV variables nor an effect service, authority grant or readiness gate. Raw IMU channels, calibration, covariance and physical camera mounting are outside this increment.

MAVROS's pinned IMU implementation informs the family grouping and source conventions. Its quaternion preference, ROS FLU/ENU conversion and noise-model interfaces are not imported. MAVMOOS retains both message families independently, their raw values and original local acquisition/read times. Quaternion-derived RPY is explicitly canonical and offset-unapplied; singular/nonunique Euler coordinates and invalid quantities remain visible. A display heading is source yaw wrapped into north-zero clockwise degrees, not travel course or independently established true heading. The AP/MAVROS/MAVLink source study documents frame wording and arithmetic rather than inferring a general vector-transform API from a single comment.

Development ABI 7 is retained. Existing metadata callbacks support this module without a new shared interface. A sample is fresh only within its original local bound and accepted source/session; freshness does not imply valid encoding, sensor quality or remote generation age. Distinct frames coalesced into one UDP datagram can have identical acquisition/read stamps: the last accepted frame may update the cache while retaining those original times. Timestamp equality is not duplicate identity; exact wire duplicates remain suppressed by the host's bounded ingress policy.

## Concurrent services and evidence boundaries

The host remains the sole owner of sending, deadlines and control permission. Observers declare stream requirements; the host constructs interval requests only in authorized simulation mode. Captured fake-I/O requests verify the union/minimum-interval policy. Native descriptor intent and observed telemetry rates do not prove delivery/ACK of every primary-link request.

The owned native fixture runs existing Guided/arm/takeoff, unchanged stock helm, simultaneous paced parameter inventory and HOME-only mission download, continued movement, and landing/disarm. A status-only comparison omits attitude while keeping the workflow. The first waypoint leg points east so the simulated attitude exercise includes nonzero yaw. Independent FC position, Guided target-state reports, telemetry and DataFlash are separate witnesses; their reception gaps are measured on one observer clock and are not uplink latency or motor-control-rate guarantees.

Acceptance tools reconcile actual NAV counter progression with phase/terminal boundaries rather than trusting a summary. They validate original service bounds, exact terminal outcomes and complete snapshots, late advancing observations, frame/norm/range consistency, source-root helper provenance, app/helm identity, and scoped process/port cleanup. Cross-process clocks are not assumed interchangeable. Negative controls, failed development runs and corrected reruns are preserved.

## Alternatives and remaining limits

Putting all observations inside a vehicle-control provider would couple shared diagnostics to motion ownership. Silently preferring/fusing independently received representations could conceal a missing or stale source. Automatically gating control on new diagnostic fields would change the accepted control profile without explicit policy/evidence. Those alternatives are excluded here.

The implementation ledger, [attitude interface](../plugins/attitude.md) and [plan](../plans/2026-11-04-attitude-and-observation-load/README.md) separate component, combined-host, clean Linux and native evidence. This remains profile-specific simulation qualification: source generation time, sensor accuracy, physical yaw tracking, full load capacity, all faults, raw IMU calibration and hardware deployment are unqualified.
