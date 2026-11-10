# ArduPilot QuadPlane control

`arduquadplane_control` owns the native VTOL mission and the separate opt-in forward-flight and hover helm workflows for the standard fixed-motor QuadPlane. It shares the host's MAVLink connection, original deadlines, final send gate, mission transfer and durable AUTO marker. The native workflow uses ArduPilot's AUTO transition sequence. The conventional `arduplane_control` provider and PX4's standard VTOL profile have their own phase and authority rules.

## Owned SITL configuration

```text
vehicle_profile = arduplane_quadplane
expected_type = 22
expected_autopilot = 3
plugins = telemetry,arduquadplane_control,missions
helm_enabled = false
control_helm = false
control_mission_execution = true
firmware_requests = false
auto_recovery_directory = /absolute/private/run/directory
```

Use a fresh observed disarmed, stopped native QLoiter state for mission transfer and a writable, process-exclusive recovery directory. The selected FC must advertise type 22 and ArduPilot. The owned fixture verifies `Q_ENABLE=1` and `Q_MAV_TYPE=22`. The same recovery directory belongs to every bridge restart in the run.

The flight plan has exactly four items. Send it through `MAVMOOS_MISSION_REQUEST` and retain the successful `MAVMOOS_STORED_MISSION_PROOF` result.

| Sequence | Item | Constraint |
| --- | --- | --- |
| 0 | HOME | Absolute MSL and horizontal coordinates agree with FC HOME. |
| 1 | VTOL takeoff | At HOME with 25–55 m relative height. |
| 2 | Forward waypoint | 250–500 m from HOME with 25–55 m relative height. |
| 3 | VTOL land | Within 40 m of HOME with zero relative height. |

The provider checks typed items and geometry before upload. It requires exact semantic readback and a current host proof before binding an execution. The ordinary command grammar is `MAVMOOS_QUADPLANE_REQUEST=schema=1,id=unique,action=...`.

| Action | Evidence required |
| --- | --- |
| `arm`, `disarm` | Fresh ground QLoiter with multicopter phase and subsequent selected arming state. |
| `run,verification_id=...,revision=...,content_id=...` | Armed ground QLoiter, verified plan, persistent marker and a newly observed takeoff cursor before native AUTO. Flight completion requires hover, forward flight, hover on the landing approach, disarm and QLoiter. |
| `abort,target=run-id` | Retained ownership, a fresh in-air AUTO observation and the original HOME bound. A single stop-only request selects QRTL. Observe the mode and retain the marker. |
| `release,target=run-id` | Fresh disarmed, stopped QLoiter and a subsequent ground observation. Clear the local lease and marker without sending a movement command. |

`MAVMOOS_QUADPLANE_RESULT` reports local send separately from FC observation. After a normal landing, the host allows ground handback at the original landing point through the FC's expected HOME resurvey and cursor reset. During flight a changed HOME, stale observation, external mode change, or expired original deadline fences the lease and retires queued effects.

On restart the marker blocks new effects. The shared recovery request requires the new process session, a fresh prior disarmed ground QLoiter observation, and explicit handback. While fenced, the host can request only the three observation intervals needed for that evidence. See the [control qualification](../development/control-qualification.md) and test archive for exact cases.

## Stock helm in stable forward flight

Set `control_helm = true` in the owned configuration above to expose `MAVMOOS_NAV_CONTROL`. Keep the existing native mission provider, recovery directory and telemetry. Load stock `pHelmIvP` with ordinary course, speed and altitude objectives. This is an opt-in simulation profile for Plane 4.6.3 source `3fc7011a7d3dc047cbb17d8bd98ee94577d144c6`, a Quad X lift frame with fixed motors and `Q_GUIDED_MODE=0`. The bridge checks the firmware version and reads the profile's airframe, airspeed, native Loiter and GCS watchdog settings before enabling helm authority. The default native-only configuration uses its existing workflow.

| Operator step | Required evidence |
| --- | --- |
| Reach native Loiter | Observe stable forward flight, armed and airborne, with current position, HOME, estimator and active sensor 0 airspeed at or above 18 m/s. Native hover and a phase transition require their own FC flight controls. |
| Send `MAVMOOS_NAV_CONTROL=acquire` | An accepted addressed Guided request and a fresh subsequent selected FC Guided heartbeat are required. The adapter then waits for new stock helm fields. A retained or unresolved AUTO mission denies acquisition. |
| Provide stock helm intent | `DESIRED_HEADING` is ground course, `DESIRED_SPEED` is a nominal 22 m/s intent and `DESIRED_ALTITUDE` is 45–120 m above HOME. The bridge sends bounded course lookahead points, roughly 300 m ahead, and requires fresh matching FC target readback. The FC controls airspeed and inner loops. |
| Brief normal all-stop | `NothingToDo` or `NoGoalBehavior` may pause and resume inside the configured short window while a confirmed current target, mode, phase and navigation evidence remain valid. |
| Release or lose helm intent | One bounded request restores native Loiter while fresh forward phase and the owned Guided mode still permit it. A pilot mode or phase change fences old work. The bridge withholds the watched heartbeat if Guided persists without a valid owner or bounded handback, and the pinned FC applies its configured GCS failsafe. |

`MAVMOOS_HELM_STATUS` reports producer state and `MAVMOOS_RUNTIME_STATUS` includes `quadplane_phase`, `plane_profile`, `plane_watchdog_started`, `plane_guided_confirmed`, `plane_target` and `plane_handback`. After takeover, expiry or link recovery, establish fresh stable native Loiter and issue a new acquire.

## Stock helm in stable hover

For the same fixed-motor SITL model, set `plugin.arduquadplane_control.hover_adapter = true` alongside `control_helm = true`. The hover fixture installs the [Lua adapter](../../tests/quadplane_native/hover_adapter.lua) on the simulator and sets `Q_GUIDED_MODE=1`, `SCR_ENABLE=1`, `Q_RTL_MODE=3` and the matching watchdog profile. Acquire from armed, airborne QLoiter with current adapter and navigation evidence. The bridge turns ordinary course, speed up to 3 m/s, and height from 25–90 m above HOME into short position steps. The FC owns the inner loops.

Fresh Guided hover and a single-use adapter offer are required before sending each eligible target. The adapter checks its own expiry and updates the FC's target in place. On producer loss, QLoiter holds and new helm output requires explicit reacquisition. Native modes own both transitions and the forward segment in this configuration. The forward-helm configuration above uses `Q_GUIDED_MODE=0`. [Design decision](../decisions/0038-quadplane-hover-and-local-release-matrix.md)

Run the round-trip hover case from a clean checkout with the [local release inputs](../development/sitl-release.md).

```sh
python3 -B scripts/release/campaign.py --inputs "$LOCAL_INPUTS" \
  --output "$FRESH_CAMPAIGN" --case quadplane_hover
```

Qualified hover round trip
