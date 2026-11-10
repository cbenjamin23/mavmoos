# MAVMOOS build queue

November 5, 2026. The delivery unit is a whole operator workflow in one provider. The active queue has three plugin milestones. Individual message and test items below supply implementation details and maintenance candidates.

The starting point is thirteen startup-loaded providers, matching ABI 9, checked module settings and a real loader. A separately built observer reaches MOOS through the app. A separately built command example exercises host admission and an observed result in owned Copter SITL. The current checkpoint passed both 109-case software gates with a five-provider app composition. The earlier full native matrix remains attached to its own source. [Provider guide](../../development/external-plugins.md) · Native examples and qualification · Current software evidence

## Plugin milestones

November 6 closeout. Traffic, flight/fence/rally inspection and flight-log retrieval are complete at their documented Copter SITL scope. System status also gained the board and estimator diagnostics from OBS-04 and OBS-05. The software and native reports retain separate exact source and artifact identities. Implementation ledger

### 1. Traffic picture

Add a `traffic` provider so an operator can inspect a bounded picture of nearby aircraft reported by the configured FC. A startup-selectable module publishes a documented MOOS contact snapshot with ICAO identity, validity flags, nullable position and velocity, reported ADS-B age, and local observation age. Accept reports only from the selected source and session. Track contact expiry and capacity independently. A repeated forwarded report can show link activity while the underlying track age remains uncertain.

Start from the MAVROS ADS-B receive handler's field and unit mapping. MAVMOOS owns the bounded contact table, validity and age checks, and MOOS publication. A separately built module must load in the real app, demonstrate several contacts, reject wrong-source and malformed updates, expire and clear contacts across a reconnect, and show zero wire output. ArduPilot has a conditional forwarding path, so a grouped native test needs an actual producer and an explicit identity. [Pinned MAVROS ADS-B receive handler](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros_extras/src/plugins/adsb.cpp#L75-L139) · [MAVLink traffic record](https://mavlink.io/en/messages/common.html#ADSB_VEHICLE) · [Pinned Copter forwarding source](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/AP_ADSB/AP_ADSB.cpp#L620-L659)

### 2. Mission, fence and rally inspection

Deliver a versioned pull workflow in the existing `missions` provider. An operator can ask what the FC has stored for each plan type and get a separate snapshot and terminal result. Reuse the existing mission transaction, with type binding at host admission, on the wire and on receipt. The current host accepts only `MAV_MISSION_TYPE_MISSION`. Extend it for typed reads and their protocol acknowledgements, while keeping upload, clear and execution proof scoped to the existing flight-plan contract.

Use the MAVROS `waypoint`, `geofence`, and `rallypoint` providers and their shared `MissionBase` as the reference for the pull sequence, type isolation and addressed replies. Adapt applicable upstream test scenarios to the existing MAVMOOS transaction. Prove nonempty and empty snapshots, wrong-type and other-client replies, cancellation and retry without renewing expiry. Demonstrate known fence and rally data in one owned disarmed Copter bench and keep ordinary mission transfers in regression coverage. [Waypoint](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/waypoint.cpp) · [Geofence](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/geofence.cpp) · [Rallypoint](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/rallypoint.cpp) · [MissionBase interface](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/include/mavros/mission_protocol_base.hpp) · [Protocol implementation](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/mission_protocol_base.cpp) · [Upstream flow tests](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/test/test_mission_flow.cpp) · [Existing MAVMOOS mission interface](../../plugins/missions.md)

### 3. Flight log retrieval

Add a `flight_logs` provider with one complete post-run workflow. List the logs on an owned disarmed Copter SITL, select one, download it into a bounded private directory, and get a complete-file receipt with size and local digest. The host needs a typed log-request service and a paced transfer lane before app integration. The provider owns chunk validation, retries within the original deadline and assembly. Local staging publishes a file only after complete coverage and removes partial output on failure.

The MAVROS plugin exposes raw request services and publishes raw entries and chunks. Use its list, data and end message handling as the wire reference. MAVMOOS owns the complete-file workflow. Check loss, repeated and out-of-order offsets, invalid lengths, cancellation, stale sessions and disallowed destinations with a fake peer. In native SITL, compare the retrieved bytes with a known simulator log and exercise `LOG_REQUEST_END`. The pinned logger can suppress writes during transfers. Account for protocol teardown even on cancellation and distinguish local send from evidence that logging resumed. [Pinned MAVROS log transfer](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros_extras/src/plugins/log_transfer.cpp) · [MAVLink log messages](https://mavlink.io/en/messages/common.html#LOG_REQUEST_LIST) · [Pinned logger request handling](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/AP_Logger/AP_Logger_MAVLinkLogTransfer.cpp#L30-L62) · [Pinned logging guard](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/AP_Logger/AP_Logger_Backend.cpp#L485-L495)

Traffic and mission inspection can build in separate worktrees. Log protocol and fault work can overlap after the integrator fixes the typed service boundary. The integrator owns shared host APIs, app/build wiring and the registry. Use one integrated normal and sanitizer run per source freeze and group native cases where they can share a fixture and still prove their distinct contracts. Timing and test-selection work supports these milestones.

## Tracking MAVROS ports

Use MAVROS as the first implementation reference for general MAVLink features. Carry over applicable field mappings, transaction steps and regression scenarios. Reuse self-contained source where appropriate and preserve the required notices. The MOOS interface, plugin lifecycle, source admission and final send policy belong to MAVMOOS. Use the existing pArduBridge and stock helm references for MOOS vehicle-control behavior.

Record the exact upstream commit, watched source and test paths, adopted behavior and local differences in each provider guide. Mark copied code explicitly with its provenance and applicable license. Keep the last reviewed upstream revision separate from the last behavior incorporated. The three milestones above were checked against MAVROS `2d3f4db1101c7950c8817a9d788f0d11106fb261`. [Upstream license variants](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/LICENSE.md)

At a new release or relevant fix, resolve the candidate to a commit and diff the watched paths, shared helpers and tests. Classify changes as wire/behavior, ROS integration, or outside the local capability. For a relevant wire or behavioral change, reproduce the scenario with a focused peer test. Run the app or owned SITL case where needed. A change that needs different generated MAVLink definitions or firmware gets its own pinned qualification.

An example is the comparison from MAVROS 2.16.0 to our reference pin. `adsb.cpp` gained uAvionix transceiver-health observations, a separate candidate from the initial contact display. The shared mission header gained an addressed-message filter with a companion test for replies sent to another client. That test is relevant to typed pulls even though the existing MAVMOOS mission path already checks target IDs. [Exact upstream diff](https://github.com/mavlink/mavros/compare/5c68b905ab30de6ce630822dc46c33467e8f23ea...2d3f4db1101c7950c8817a9d788f0d11106fb261) · [ADSB health handler](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros_extras/src/plugins/adsb.cpp#L141-L158) · [Mission reply test](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/test/test_mission_flow.cpp#L458-L511)

## How milestones move

1. Define the operator-visible result, owning provider, supported source and acceptance. Choose disjoint code ownership before starting parallel work.
2. Build the smallest scope and run exact focused cases in a private build. Exercise the normal app path for a new configuration or MOOS surface.
3. Give an independent checker the implementation, the wire definition and the exact sender or fixture. Check at least one plausible counterexample, such as a valid trimmed frame, wrong source, reset, retry or an unknown field.
4. Integrate on one source freeze and run the affected gates. Record the exact build, focused cases and real-app evidence. For native work bind the firmware, dependencies, owned endpoints and scenario to the receipt. A grouped native witness should cover only capabilities the FC actually emits or exercises.
5. Close the item with documentation, raw results and a scoped cleanup readback. Move to the next ready item. Preserve original failures and qualify each firmware/platform cell independently.

Run the normal and sanitizer software gates on an integrated feature wave. Keep the existing single native smoke lifecycle for changes that need it. Use the full pinned matrix at qualification checkpoints and firmware promotions. The [feedback plan](../2026-11-05-fast-feedback-and-observer-expansion/README.md) gives the warm feedback budgets and case-selection rules. Keep app work on stock `pHelmIvP`, local owned loopback and SITL. Default receive-only and shadow continue to require zero egress.

## Observation backlog

The completed first batch extended three existing providers with disjoint builder worktrees and independent reviews as slots freed. Root owned the combined app witness, registry, integration and archive. The acceptance layers were built modules, contract checks, actual app generated-wire delivery and stale/reset behavior, scoped zero egress, and both integrated software gates. Native support is a separate cell with an actual producer witness.

| ID | Work | Acceptance |
| --- | --- | --- |
| OBS-01 | Add `SCALED_PRESSURE`, `SCALED_PRESSURE2` and `SCALED_PRESSURE3` to `air_data`. | Independently age each FC-reported instance. Prove hPa/temperature encoding, differential-pressure availability ambiguity, valid trimming and null extension handling. Demonstrate the first instance through the app. |
| OBS-02 | Add independent `VIBRATION` and clipping counters to `imu`. | Validate nonfinite and reordered reports, preserve raw unsigned counts and original sample time, and exercise the actual app publication. |
| OBS-03 | Add `GPS_GLOBAL_ORIGIN` to `odometry`. | Preserve WGS84/MSL and the optional raw timestamp. Exercise the ArduPilot and PX4-shaped messages through the app. Keep the origin independent of home and local NED samples. |
| OBS-04 | Add board-power and MCU telemetry to `system_status`. | Separate board/servo supply from battery reports. Prove flags, validity and missing-domain behavior from sender fixtures. |
| OBS-05 | Expose estimator diagnostics through `system_status`. | Decode ArduPilot `EKF_STATUS_REPORT` and common `ESTIMATOR_STATUS` as independent observations, with capability/flag evidence. Keep the private control readiness rule separate. |
| OBS-06 | Add reported FC track diagnostics near navigation observations. | Publish `NAV_CONTROLLER_OUTPUT` with its sender’s angle, distance and error units. Verify stale cases and that no helm/NAV target changes. |
| OBS-07 | Expand GPS observability. | Bound secondary receiver and optional accuracy fields by exact producer. Demonstrate unknown and stale independently of the primary GPS report. |
| OBS-08 | Extend `rc_io` with a scoped actuator-output observer. | Prove the selected firmware publishes `ACTUATOR_OUTPUT_STATUS`, preserve channel masks/indexes, and distinguish observed command values from mechanical feedback. Start with wire fixtures if there is no pinned sender. |
| OBS-09 | Add obstacle-distance observations alongside range sensing. | Preserve bins, sensor orientation, frames, increment and unknown/sentinel values. Check oversized, incomplete and stale arrays. |
| OBS-10 | Add a bounded traffic observer for `ADSB_VEHICLE`. | Show a scoped identity route, per-contact age, validity flags and deterministic capacity. Prove input only with a simulated sender. |
| OBS-11 | Add passive mode inventory/state observations. | Audit pinned sender support for standard mode messages and retain firmware-specific heartbeat interpretation. Exercise missing/repeated inventory rows and source change. |
| OBS-12 | Extend air-data support beyond scaled pressure after tracing sender units. | Resolve `AIRSPEED` differential pressure and `HIGHRES_IMU` magnetic scaling against each pinned sender. Add conversion only where the wire and sender units agree or can be explicitly adapted. |

Current wire references for the first three items. [Scaled pressure](https://mavlink.io/en/messages/common.html#SCALED_PRESSURE) · [Vibration](https://mavlink.io/en/messages/common.html#VIBRATION) · [Global origin](https://mavlink.io/en/messages/common.html#GPS_GLOBAL_ORIGIN)

## Qualification and tooling

These tasks can overlap provider development when their files and runtime domains are separate.

| ID | Work | Acceptance |
| --- | --- | --- |
| QA-01 | Keep one reusable app peer for successive observer waves. | New message checks share the existing fresh/stale/recovery phase and prove zero egress without adding a boot for every family. |
| QA-02 | Capture a focused timing history per integrated checkpoint. | Record compile, test and teardown separately, with exact selected IDs. Show that added passive features leave the native smoke roster fixed. |
| QA-03 | Add exact-ID native diagnostic selection with supervised leases. | Demonstrate one targeted owned case and negative controls for missing pin, wrong artifacts, another process and incomplete cleanup. Results remain targeted. |
| QA-04 | Qualify one portable observer on a pinned second firmware family. | Build or locate an isolated PX4 SITL candidate. Demonstrate an actual local NED stream with the existing module, exact identity and zero bridge egress. |
| QA-05 | Measure a grouped passive witness on the pinned development Copter. | Verify the same observable families and actual producer differences without treating it as a version range. |
| QA-06 | Run one clean Linux source build at a selected integrated release checkpoint. | Archive exact platform/dependency pins, no-ambient-network execution where required, JUnit, artifacts and scoped cleanup. |
| QA-07 | Prototype an offline shard result merger. | Reject missing/duplicate required cases, mixed source/build/profile/platform and unknown cleanup. Run native shards only once equivalent isolated workers are available. |
| QA-08 | Build a release inventory and firmware promotion receipt. | Show active source and module ABI, the exact accepted scenario cells, and failed/unknown cells for a new firmware candidate. |

## Owned-SITL protocol work

Develop each as a bounded transaction or provider capability. Exercise peer faults and host policy first, then use a single appropriate native witness. Read requests can send MAVLink in an explicitly owned SITL session. Vehicle effects keep their existing host admission boundary.

| ID | Work | Acceptance |
| --- | --- | --- |
| DATA-01 | Add read-only geofence inventory to `missions`. | Bind exact `MAV_MISSION_TYPE_FENCE` end to end. Reject cross-type replies and exercise empty, loss, duplicate and cancellation cases. |
| DATA-02 | Add read-only rally inventory to `missions`. | Reuse transaction state while validating `MAV_MISSION_TYPE_RALLY`, limits, coordinates and an independent operation ID. |
| DATA-03 | Add log inventory and scoped file retrieval. | Separate list and download, constrain size/rate/destination, verify offset/length, retry, duplicate suppression, cancellation and partial cleanup. |
| DATA-04 | Add read-only MAVLink FTP browsing and retrieval. | Prove session, offset and sequence behavior on an inert peer first. Expose a bounded transfer status and a complete-file receipt. |
| DATA-05 | Audit a second parameter encoding profile. | Establish wire value encoding and supported types with exact round trips on an owned fixture, with profile-specific interpretation. |
| DATA-06 | Add a bounded time-observation experiment. | Compare remote stamp, local monotonic time and round trip. Publish uncertainty and never renew an original navigation expiry with a received sample. |

Mission transfer protocol reference. [MAVLink mission types](https://mavlink.io/en/services/mission.html#mission-type)

## Decisions that change the product contract

Queue a short proposal with an example and explicit choices before advancing these items. Software and read-only source work can continue around them.

| ID | Decision | Evidence needed before implementation |
| --- | --- | --- |
| CTRL-01 | Emergency LAND/RTL precedence during AUTO uncertainty. | Show which operator or FC event owns each handoff in concrete cases, including an already-transmitted command. |
| CTRL-02 | Extend command coverage to another vehicle profile. | Select Rover marine, Plane or PX4 and define native versus helm behavior, loss policy and required feedback for that vehicle. |
| CTRL-03 | Add camera, gimbal or landing-target inputs. | Identify the component and owner, expected vehicle effect, rate bounds and cancellation. Start with a simulated payload. |
| CTRL-04 | Route observations from multiple components or systems. | Define source admission and provenance independently from the control FC, then test mixed-source and restart behavior. |
| CTRL-05 | Add physical serial or a router deployment profile. | Decide link, startup/reconnect and ownership contract. Prioritize an isolated bench and explicit authorization for that environment. |
| CTRL-06 | Extend mission or parameter mutations. | Define allowed effect scope, uncertainty and confirmation, then run fake/owned-SITL controls for partial application. |

## Next disjoint batch

This table preserves the original maintenance handoff after `OBS-01` through `OBS-03`. Follow the [plugin milestones](#plugin-milestones) for delivery order. First wave results

| Item | Owner and handoff |
| --- | --- |
| OBS-04 | `system_status` and `tests/status/`. Add board and servo rail `POWER_STATUS` and bounded `MCU_STATUS` instances. Keep zero/unknown and raw flags explicit, reject fabricated MAVLink 1 MCU frames, and preserve the existing status publication and stream contracts. The pinned SITL board rail is synthesized and MCU emission is compile-gated. Start with the grouped app peer. [Pinned sender](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/GCS_MAVLink/GCS_Common.cpp#L215-L255) |
| OBS-06 | `odometry` and `tests/odometry/`. Start with the pinned Copter `NAV_CONTROLLER_OUTPUT` interpretation. Keep the separate stamp, raw signed values, distance saturation and the sender's unavailable airspeed-error distinction. A mode without a waypoint can publish zero fields. [Pinned sender](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/ArduCopter/GCS_Mavlink.cpp#L206-L222) |
| QA-02 | Offline CI timing and evidence tooling. Report the exact roster and build, inventory, supervised test and per-case time. Treat separately measured teardown as unknown for earlier receipts. Record which test windows overlap and preserve the original deadlines when probing timing failures. |

OBS-05 follows OBS-04 because both own the same provider and fixtures. The next wave can start from the current source without changing helm ownership or deployment scope.

Run private builds and focused component checks in parallel. When an overlapping qualification run hits a deadline or process-inventory failure, preserve the receipt and retry the integrated gate in an exclusive window with the original deadlines.
