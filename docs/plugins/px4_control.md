# PX4 control

The `px4_control` provider supplies stock-helm Offboard and an optional native lifecycle for the quadrotor, conventional fixed-wing aircraft and standard VTOL in owned PX4 SIH. One `iMAVLink` process combines the selected profile with the shared `telemetry` module. [Native actions](#native-aircraft-actions) · [Lifecycle qualification](../../tests/px4_offboard/AIRCRAFT.md)

## Vehicle types and the shared link

PX4 vehicle types share the MAVLink link and basic protocol services such as heartbeat, telemetry and parameters. Offboard setpoint meaning and mode behavior depend on the airframe. A PX4 quadrotor uses velocity fields in `SET_POSITION_TARGET_LOCAL_NED`. The fixed-wing profile uses position fields and the receiver's accepted global frame enum. The Rover study separates the stable documentation from the exact receiver and drive controllers. [PX4 1.17 Offboard message support](https://docs.px4.io/v1.17/en/flight_modes/offboard#mavlink-messages) · [Pinned receiver](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mavlink/mavlink_receiver.cpp#L1034-L1268) · [Rover interface study](../research/2026-11-06-px4-rover-interface.md)

The provider selects the PX4 heartbeat identity from the configured quadrotor, fixed-wing or standard VTOL profile. Use `arduplane_control` for conventional ArduPilot Plane. The coverage matrix tracks further PX4 airframes and their applicable evidence. [PX4 1.17 airframe reference](https://docs.px4.io/v1.17/en/airframes/airframe_reference) · [Coverage matrix](../plans/2026-11-06-vehicle-family-coverage/README.md)

## Multicopter workflow

Use the dedicated test configuration:

```text
vehicle_profile = px4_multicopter
plugins = telemetry,px4_control
simulation_only = true
shadow_mode = false
control_native_actions = true
firmware_requests = false
bind_address = 127.0.0.1
bind_port = 14540
peer_address = 127.0.0.1
peer_port = 14580
target_system = 1
target_component = 1
sitl_pid = <owned local px4 PID>
```

The active route requires the exact Linux `/opt/px4/bin/px4` process and its nominated onboard UDP socket in the bridge's loopback-only namespace. RX-only observation may use a normal fixed selected peer and always sends zero packets. The pinned executable and startup scripts are verified separately by the test runner.

After the aircraft has taken off and entered native Hold, publish `MAVMOOS_NAV_CONTROL=acquire`. The stock `pHelmIvP` supplies `DESIRED_HEADING`, `DESIRED_SPEED`, `DESIRED_ALTITUDE`, `IVPHELM_STATE` and `IVPHELM_ALLSTOP`. The module uses the ordinary arrival-grouped rules, with new scalars after both acquisition and observed Offboard entry. Course is degrees clockwise from north, speed is 0–3 m/s, and altitude is 2–30 m above the observed home. Travel-facing is the default; keep yaw and explicit heading are configurable.

For entry the host needs fresh armed/airborne Hold, PX4 intended mode, home, global position, local NED and PX4 estimator status. The pinned EKF may set the legacy constant-position bit while at rest, so readiness requires valid local and global aiding flags and excludes GPS-glitch and accelerometer errors. The shared stream service requests ESTIMATOR_STATUS at 5 Hz and CURRENT_MODE at 2 Hz, both with a 1.6 s host age limit. A local send is never proof of the new cadence. After new valid helm data, the bridge holds a captured local NED point and requires at least 1.25 seconds and eight successful consecutive local setpoint handoffs before requesting Offboard. Motion uses local NED north/east velocity and an altitude correction limited to ±1 m/s. No send or ACK establishes mode or physical motion. Check `MAVMOOS_RUNTIME_STATUS` for `px4_local_ready`, `px4_offboard_confirmed`, `px4_native_hold_ready`, `px4_prestream_count`, `px4_hold_tx_accepted`, and `nav_tx_accepted`; operator-facing readiness also appears in `MAVMOOS_HELM_STATUS`.

A short `NothingToDo` or `NoGoalBehavior` pause can stream one captured position until the original handoff window expires. Fresh post-pause inputs can resume within that window. Release, takeover, expired data, clock loss, a local-reference jump or broken setpoint continuity withdraws the stream. The PX4 fixture explicitly configures and reads back its firmware-owned loss timeout and Hold response. The bridge does not write those parameters or acquire while the FC remains Offboard. After an Offboard loss, PX4 can report Hold while still intending Offboard. The bridge needs both active and intended Hold in a fresh CURRENT_MODE report before it will send a new prestream. The operator must explicitly select native Hold and then acquire with fresh inputs.

The earlier airborne-only SIH quad scenario passed on the pinned 1.18.0 release candidate and stable 1.17.0. The later native lifecycle was qualified on 1.17.0. These stable fixtures wait for the post-takeoff home reference to settle. If a home-reference change retires the grant, the operator must acquire again from fresh native Hold. Same-scenario comparison · [Pinned EKF status encoding](https://github.com/PX4/PX4-Autopilot/blob/fca3df865af36124a28c9d607e850f111dbaaea9/src/modules/ekf2/EKF/ekf_helper.cpp)

## Conventional fixed-wing workflow

Use the same owned PX4 SITL connection and module composition with `vehicle_profile=px4_fixed_wing` and `facing_policy=course`. The matching identity is `MAV_AUTOPILOT_PX4` and `MAV_TYPE_FIXED_WING`. Select conventional airborne native Hold before sending `MAVMOOS_NAV_CONTROL=acquire`. The same explicit `release` request withdraws helm authority.

Publish the usual stock helm course, nominal speed of 12–22 m/s and height of 35–100 m above home. Nominal speed scales a five-second forward lookahead. The bridge sends position only, with target altitude in MSL using the bound home reference. The flight controller tracks the target and manages airspeed and attitude. The prestream and brief normal handoff use a position ahead on the observed track at the captured local height. On abnormal loss or release, the stream stops and PX4 takes its configured loss action. The fixture reads back native Hold as that action. After any retained Offboard intent, select native Hold and wait for fresh active and intended Hold before acquiring again.

Admission checks the conventional-airborne report, fresh sensor airspeed reported in `VFR_HUD`, height, ground speed and a 500 m radius around home for current and commanded position. This radius reserves space for maneuvering and bounds which targets the bridge will send. PX4 controls the resulting trajectory. Airspeed age measures acquisition of the FC's report. The underlying sensor sample can be older. The simulator and scope are documented with the exact envelope and test command. [Fixed-wing runner](../../tests/px4_offboard/FIXED_WING.md) · [Profile plan and bounds](../plans/2026-11-06-px4-fixed-wing/README.md) · Pinned SITL results

## Native aircraft actions

Set `control_native_actions=true` on the same dedicated owned Linux SITL composition to enable `MAVMOOS_COMMAND`. For example, send `id=t01,action=takeoff,height=8` after a distinct successful arm request. IDs are unique for the life of the app. A single admitted action owns the control boundary and retires any helm grant. This facet defaults off, so an existing airborne-only configuration keeps its previous request surface. Send `MAVMOOS_NAV_CONTROL=acquire` again from independently observed active and intended Hold when you are ready to resume stock helm.

| Action | Admission | Completion |
| --- | --- | --- |
| `arm`, `disarm` | Fresh landed and mode evidence. Arm from a parked mode. Disarm on the ground outside Offboard. | The requested armed state in fresh ground telemetry. |
| `takeoff,height=<metres>` | Already armed on the ground in a parked mode, with fresh position, estimator and home. Quad 3–30 m, conventional plane 35–100 m, standard VTOL hover 25–65 m above home. | Observed Takeoff and then native Hold near the requested airborne height. |
| `hold` | Armed airborne in Offboard, Hold or Return. Can replace retained Offboard intent after firmware enters Hold on loss. | Fresh active and intended native Hold. |
| `rtl` | Armed airborne in Offboard, Hold or Return. | Fresh active and intended Return. The FC owns the return, descent and landing policy after mode entry. |
| `land` | Armed airborne in Offboard, Hold or Return. Standard VTOL must be in stable hover. | Observed native Land and fresh touchdown. An independent disarm request can follow touchdown. |

A local UDP send is reported separately as `MAVMOOS_TX_RESULT`. `MAVMOOS_COMMAND_RESULT` separates admission, `ACK_OBSERVED`, observation and completion. ACKs are diagnostic because they omit the MOOS request ID and command parameters. On an uncertain outcome, the original finite lease guards against an overlapping command. The app sends each admitted request at most once per process.

## Standard VTOL workflow

Use `vehicle_profile=px4_standard_vtol` and `facing_policy=course` with the stock SIH standard VTOL model `sihsim_standard_vtol`. The provider requires the PX4 `MAV_TYPE_VTOL_FIXEDROTOR` heartbeat and fresh `EXTENDED_SYS_STATE`. `MAVMOOS_PX4_VTOL_PHASE` reports the MAVLink VTOL state. Stock helm uses the same heading, speed and altitude variables in both phases. Stable hover uses local NED velocity with bounded vertical correction. Stable forward flight uses a global position ahead on course at the requested altitude. Forward speed scales the lookahead.

Enter native Hold and request `transition_fw` or `transition_mc` at 22–100 m above home within the qualification area. A transition retires the previous helm grant. Intermediate, stale or unexpected phase evidence blocks Offboard sends. Once the independently observed stable phase and native Hold agree, acquire with fresh phase-appropriate helm output. Hover accepts 0–3 m/s and 2–100 m above home. Forward accepts nominal speed of 7–16 m/s and 25–100 m above home. The app uses separate hover and forward flight envelopes with local and global continuity checks. The FC controls the transition and post-transition attitude. [PX4 v1.17 SIH models](https://docs.px4.io/v1.17/en/sim_sih/) · [Pinned PX4 transition handler](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/vtol_att_control/vtol_att_control_main.cpp#L132-L168)
