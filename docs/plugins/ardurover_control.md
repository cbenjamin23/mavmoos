# ArduRover ground and boat control

`ardurover_control` is one startup-loaded provider for native FC actions, bounded Guided operations and stock-helm guidance. Select one profile at startup. Its private operation coordinator and the reusable helm adapter use the same backend, connection and authority service. The same provider owns the optional verified native mission workflow. [Native mission procedure](ardurover_mission.md)

| `vehicle_profile` | MAVLink identity from ArduPilot (autopilot 3) | Bridge speed limit | Native stationkeeping |
| --- | --- | --- | --- |
| `ardurover` | Ground Rover (type 10) | 3 m/s | Hold stops drive |
| `ardurover_boat` | Surface Boat (type 11) | 2 m/s | Hold stops drive; Loiter can apply propulsion to maintain position |

Default app operation remains receive-only. Effects require `simulation_only=true`, `shadow_mode=false`, target system/component 1/1 and the exact owned `ardurover` process and loopback route. If supplied, `expected_type` and `expected_autopilot` must agree with the selected profile. An incoming heartbeat of the sibling type cannot open the route. Use matching host/module builds. The supported models and physical bounds are defined by the [SITL workflow](../../missions/rover_expanded/README.md).

## Native replay directory

iMAVLink maintains one durable request journal for this route. Use the default beside the generated mission file, or select an existing absolute private directory with `control_replay_directory`. The former `plugin.ardurover_control.native_replay_directory` setting also selects the host journal and preserves its previously consumed IDs. Requests denied during pilot Manual are retired before handback. [Durable control requests](../development/control-replay.md)

## Operator interface

`MAVMOOS_COMMAND=id=unique,action=guided|arm|hold|rtl|disarm` remains available and reports to `MAVMOOS_COMMAND_RESULT`. The boat profile additionally accepts `action=loiter`. An operator can use the versioned interface for the same native actions or a direct Guided operation:

| `MAVMOOS_CONTROL_REQUEST` string | Requested behavior |
| --- | --- |
| `schema=1,id=g1,action=goto,strategy=guided,lat=42.358501,lon=-71.087589,speed=1.2` | Approach the given WGS84 goal by renewing short velocity targets from advancing position. |
| `schema=1,id=h1,action=hold,strategy=guided` | Send true zero velocity and observe a stopped result. |
| `schema=1,id=r1,action=return,strategy=guided` | Approach the home coordinates bound at admission. |
| `schema=1,id=h2,action=hold,strategy=native` | Request native Hold (mode 4) and observe the mode and speed. |
| `schema=1,id=l1,action=loiter,strategy=native` | Boat only: request native Loiter (mode 5) and observe mode and a low-speed dwell. |
| `schema=1,id=r2,action=return,strategy=native` | Request native RTL (mode 11) and observe mode, proximity to the bound home and stopped speed. |
| `schema=1,id=a1,action=arm,strategy=native` | Ordinary arm; requires fresh readiness and disarmed Guided or Hold. |
| `schema=1,id=a2,action=disarm,strategy=native` | Ordinary disarm; a disarmed heartbeat alone establishes no physical stop. |
| `schema=1,id=m1,action=guided,strategy=native` | Select Guided (mode 15). This alone does not grant helm authority. |
| `schema=1,id=c1,action=cancel,target=g1` | Retire the named versioned request. Already transmitted actions can still have effects. |
| `MAVMOOS_NAV_CONTROL=acquire` or `release` | Acquire or release stock helm guidance. |

Use a unique ID per operation, 1–48 ASCII letters, digits, underscore, dot or dash. The versioned native and Guided forms accept an optional original-lifetime `timeout` of 1–120 s. Defaults are 8 s for arm/disarm/Guided, ground Hold and Guided hold; 25 s for native boat Hold; 20 s for native Loiter; 90 s for native return (`rover_return_timeout`); and 60 s for Guided goto/return. Boat Hold allows time for a coasting hull to settle without changing the command handoff deadline or stopped-speed bound. Guided goto/return accept speed from 0.2 m/s to the profile limit (default 1 m/s); starting distance to the bound goal is limited to 100 m. Guided operation completion observes distance within 2.5 m and horizontal speed at most 0.3 m/s for one second of advancing position samples. A changed home binding cancels return. Native RTL may select a rally point in other configurations; this interface's completion criterion is home and the wiped fixture has no rally points.

`MAVMOOS_CONTROL_RESULT` reports the request ID, profile, action, strategy, status, reason and original deadline. `MAVMOOS_CONTROL_PROGRESS` adds distance, horizontal speed and observation time. A native `SENT` means a local UDP handoff. The first matching wire ACK in the observation window is reported as `ACK_OBSERVED` with its result code. That ACK has no MOOS request ID or requested mode, so it can belong to an older command. Confirmation and completion rely on fresh post-handoff observations. An ACK cannot fail a request or extend the initial observation window. If the expected state is unavailable at `command_ack_timeout` after handoff (4 s by default), or the original operation deadline arrives first, the request times out. For Guided, `SUBMITTED` means host navigation admission, and `CONFIRMED` starts from a new observation after target admission. Velocity targets have no FC command ACK; `COMPLETED` describes the observed outcome. A timeout or cancellation does not prove that an earlier packet did not take effect. Native and Guided operations share one pending slot, original publication deadlines, and a bounded per-session ID history. The native replay directory adds persistent protection for native IDs. Cancel an active direct operation before requesting a new native action.

When a fresh request selects the native mode already observed on the controller, the provider reports `OBSERVING` and waits for fresh completion evidence without sending another command. In pilot Manual the bridge denies native actions, including arm and disarm, and retires queued work. To hand authority back, the pilot selects Hold. A separate fresh request can then select Guided or acquire helm. The bridge gates on observed FC mode and cannot recall packets already handed to the network.

## Stock helm and mode ownership

The unchanged helm publishes `DESIRED_HEADING` (degrees clockwise from north), `DESIRED_SPEED` (m/s), `IVPHELM_STATE` and `IVPHELM_ALLSTOP`. The profile is planar and subscribes to no altitude field. Readiness requires heartbeat ≤2 s, valid position/heading ≤0.5 s and horizontal EKF evidence ≤3 s (attitude, horizontal velocity, absolute horizontal position, no uninitialized flag). Home is required for return.

Only armed Guided permits motion. A fresh explicit acquire can request Guided from native Hold or, for boats, native Loiter. The temporary entry allowance is limited to the original preparation mode and is retired when Guided is observed. A later mode change or disarm cancels the grant; a later Guided heartbeat cannot restore it. Guided operations also require fresh armed Guided before starting. Native commands and explicit release revoke prior navigation. A normal, short stock `NothingToDo` can resume within the original handoff window with fresh new fields. Expired input, mode takeover, broken observations and reset require a new explicit acquire.

The adapter preserves the original MOOS publication ages and producer and requires new scalars after acquisition. Default scalar timeout is 2 s, allowed publication spread is 0.5 s, and normal handoff is 0.5 s. Stock scalar mail cannot certify that two fields originated in the same helm decision.

## MAVLink and the three ways to hold

Moving course/speed is translated to north and east velocities in `SET_POSITION_TARGET_GLOBAL_INT` (mask `0x0DE7`), following Jeremy's BlueBoat bridge. At zero speed the bridge keeps zero horizontal velocity and uses fresh heading with mask `0x09E7`, so `atan2(0,0)` cannot add an arbitrary turn. The FC owns steering, speed, mode behavior and actuators. Course-facing is the only supported facing policy.

Guided zero is a velocity request. Native Hold stops drive. In a current, a boat may drift in either case; native Loiter uses its position controller and can drive back within the configured radius. Stock helm stationkeeping is another ordinary behavior configuration when MOOS should own navigation. All are found through the same Rover provider or its helm interface.

The host replaces waiting targets, checks authority and original expiry at final send, and can make one bounded zero-velocity attempt on a loss if fresh armed Guided still qualifies. The FC also has its own 3-second Guided velocity timeout. Releasing control never automatically selects Loiter or RTL. Neither endpoint can recall an already transmitted packet.

[SITL fixtures](../../missions/rover_expanded/README.md) · [Control coverage](../development/control-coverage.md)
