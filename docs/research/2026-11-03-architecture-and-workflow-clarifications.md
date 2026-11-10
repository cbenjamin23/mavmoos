# Architecture, deployment and Copter workflow clarifications

November 3, 2026. Documentation only. This note records the user's annotated questions, reference checks and selected handoff preference. It distinguishes accepted direction in [decision 0009](../decisions/0009-backseat-and-copter-policy-defaults.md) from candidate interfaces and future implementation work. No application, library, firmware, mission or test behavior changed.

## Generated MAVLink and native UDP

Generated MAVLink means the standard C encoding/decoding library produced from MAVLink XML message definitions, used from C++. It does not mean AI-generated control logic or a new protocol. Our code fills typed fields; the library packs/parses the wire representation. The project pins generated headers and verifies the dependency archive. See the [official C-library guide](https://mavlink.io/en/mavgen_c/), especially receiving and transmitting.

The native UDP backend is small host-owned C++ socket I/O beneath that codec. It reads/writes MAVLink bytes using operating-system datagram sockets. The bounded send policy is another responsibility: it decides which work may reach that socket and when expired/cancelled work must be rejected. Generated codecs do not implement command transactions, transport queues, expiry or vehicle behavior by themselves.

The P1 app has native receive-only IPv4 UDP. The P2 experimental send library is separate, not linked into the app, and its real I/O is restricted to owned localhost fixtures. Neither establishes an operational FC link. MAVLink can also run over serial or TCP; choosing it does not select UDP.

## What the references already settle

The user is right that substantial precedent exists. Do not reopen every conventional service boundary as a new research problem. Preserve one modular app, shared vehicle state, common protocol/transaction infrastructure, a stock-helm adapter and vehicle-specific translation/policy. Build small feature modules with their tests as the seams stabilize. A generic runtime loader, a second backend and full MAVROS parity are not prerequisites.

pArduBridge's architecture at `e34a62ef49c2be42215a69c351850cbd85881486` explicitly places MAVSDK Action/Telemetry/MissionRaw/Param/Passthrough beneath the MOOS boundary and above MAVLink over UDP/TCP/serial. Its command path already covers ARM/DISARM, Copter takeoff, landing, RTL and native hold/loiter. Use that source and Steve's documented workflow as the starting point for MAVMOOS's lifecycle module.

There is still implementation work because MAVMOOS's direct backend does not inherit MAVSDK's action and transaction machinery. Command/result correlation, operation-specific retry rules and observed-state completion need a shared implementation. They should be implemented once and reused by features. A takeoff ACK is different from reaching the requested height; a landing request is different from confirmed landed state. Existing pArduBridge confirmation trackers are concrete precedents.

MAVROS at research revision `2d3f4db1101c7950c8817a9d788f0d11106fb261` provides arming, takeoff and landing services in its [command plugin](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/command.cpp#L433-L521). These expose operations; a complete mission sequence still requires application orchestration and outcome checks. The [node architecture](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/docs/nodes.md) separates MAVLink endpoints/router, shared UAS state and feature plugins.

Candidate early demonstration: connect, observe ready state, explicitly arm, take off to a specified home-relative height, confirm airborne/height, acquire helm navigation, fly a simple route, hand off to hold or landing, confirm landed state, and handle disarm under an explicit policy. The already-airborne case remains a smaller diagnostic test, not the intended feature ceiling. This is a proposed expansion of future implementation planning, not an execution result or authorization to operate a vehicle.

## Configurable facing and vertical reference

The user selected course-facing as the default. An illustrative configuration is `facing_policy=course`; `fixed_heading`, independently supplied yaw and FC-managed yaw are possible future modes. Names are proposals, not accepted MOOS parameters. Startup must reject unsupported policies rather than silently substituting another.

This is a bridge/profile policy implemented through supported Guided setpoint fields; it need not be a different FC flight mode. The FC remains responsible for executing yaw within its configured limits and behavior. In pArduBridge's inspected `UAV_Model.cpp:1317-1403`, course/speed are converted to NED velocity, the course is included as yaw, and the yaw field is ignored at speeds at or below 0.05 m/s. This is useful precedent; the numerical threshold is not field-qualified for MAVMOOS.

For example, flying east can default to a 90-degree nose target. With an independent yaw feature it could fly east while facing north. Omitting a yaw request delegates behavior to the FC and must not be described as a guaranteed fixed-heading hold. Avoid conflating yaw/nose direction, desired travel course and observed course over ground.

MAVROS's [raw setpoint plugin](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/setpoint_raw.cpp#L208-L252) independently accepts velocity, yaw, yaw rate and a field mask, with frame conversions. The caller can therefore implement a facing policy instead of MAVROS universally enforcing course-facing. ArduPilot's [Guided command documentation](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html) documents the supported velocity/yaw fields and masks.

Altitude reference can also be explicit configuration, but different references require different evidence/conversions. Home-relative height, MSL altitude and terrain clearance are not interchangeable. Keep home-relative height as the current draft's default proposal; expose additional references only when their required home/terrain/source validity is supported. The user's request for configurable options does not prove those capabilities exist.

## All-stop in operator terms

The question is whether a normal pause between behaviors requires pressing deploy/re-enable again. The user selected **automatic resumption after a bounded normal handoff**.

| Example | Selected direction or recommendation |
| --- | --- |
| A waypoint finishes and a loiter behavior starts shortly afterward, with a brief normal no-decision interval | Cancel the old motion target, request a qualified hold, and resume when fresh eligible outputs return within the window under continuing authority. No extra operator re-enable for this normal handoff. |
| No valid behavior returns before the window expires | Leave the recoverable-pause state; require explicit reacquisition. Repeated cached mail/all-stop announcements cannot extend the window. |
| Pilot selects RTL/LAND/manual, or the bridge restarts | Do not automatically reclaim control when helm messages return. These are separate authority/session boundaries. |
| An active behavior deliberately requests zero horizontal speed with a valid altitude target | Treat it as an ordinary eligible target, not necessarily all-stop or a request to land. |

These are intended behaviors, not demonstrated physical outcomes. The exact hold request remains provisional and requires current valid pose/FC mode. The normal-handoff exception does not permit using stale motion during the pause or bypassing fresh state and post-boundary input checks. Resolve the whitelist of normal reasons, maximum recovery duration, remaining fault classifications and action on failed hold in the next profile revision.

Stock-source check: `~/moos-ivp`, revision `023fb54e00692611ecee2cd6f1138d10eec2127e`, `ivp/src/pHelmIvP/HelmIvP.cpp:501-550` distinguishes `BehaviorError`, `NothingToDo`, `NoGoalBehavior`, missing decision variables and `clear`. `postAllStop` at lines 1717-1774 contains a one-iteration allowance for certain incomplete/no-goal transitions and may publish zeros to decision variables afterward. Its zero altitude must never become a new request to descend to home height. Check the reason and required new inputs; a speed zero or `clear` alone is insufficient permission.

The live MIT PDF index was inaccessible through the web tool on November 3. Exact behavior above is based on the inspected local stock source, not an unverified manual recollection. The existing archived source/manual evidence remains in the repository.

## Backseat deployment and transport

The intended host is settled: the vehicle's backseat computer. It runs the MOOS community, helm as configured, and MAVMOOS; the frontseat/FC handles stabilization, native flight modes and its failsafes. MAVLink is the protocol between the bridge and FC.

The remaining wiring detail is whether that backseat reaches the FC through direct USB/UART serial, a network endpoint or an existing MAVLink router. pArduBridge's README/quickstart already distinguish SITL UDP and Linux/Pi serial. MAVROS likewise supports MAVLink endpoints over serial/UDP/TCP. Do not ask again whether MAVMOOS should use MAVLink or presume that its UDP fixture dictates physical vehicle wiring.

## Architecture and parallel feature development

Agree a small set of shared interfaces, then implement feature modules and their meaningful tests concurrently. Keep shared app/build wiring and final effect authorization under one integrator. Proposed independent work streams are telemetry/NAV projection, stock-helm adaptation, and command/lifecycle handling. A separate integration/review pass should combine them against real stock MOOS processes and, in a later authorized phase, pinned Copter SITL.

Tests progress with implementation: unit/fake-I/O checks for conversions and transaction behavior; MOOS process tests for publications, retained mail and normal handoffs; SITL for FC command interpretation and full workflows; configuration-specific hardware qualification later. Module tests do not substitute for an integrated workflow. Use subagents when the user dispatches that work; this clarification turn does not start parallel implementation.

Continue consulting MAVROS for feature/message/service patterns and pArduBridge for the existing UAV workflow. Ask the user about consequential operator behavior that lacks a sensible established default, and explain alternatives with concrete mission examples. Routine configuration and already settled deployment choices should not become repeated blockers.
