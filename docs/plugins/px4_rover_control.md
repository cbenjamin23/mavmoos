# PX4 Ackermann Rover control

`px4_rover_control` owns live helm and native mission execution for `px4_rover_ackermann`. The evidence is for the owned Linux ARM64 `sihsim_rover_ackermann` model on PX4 v1.17.0. The shared `missions` provider handles storage and verified readback. Both use the same `iMAVLink` connection and host authority checks. Qualification and replay

## Configure the owned simulator

Use the `iMAVLink` settings and planar stock `pHelmIvP` behavior file in the [qualification fixture](../../tests/px4_rover/README.md). Select `telemetry,px4_rover_control,missions` for the combined workflow, `simulation_only=true`, the exact owned PX4 PID, the paired loopback endpoint, system and component `1/1`, `expected_type=10` and `expected_autopilot=12`. Live helm uses `control_helm=true`, `helm_enabled=true`, `helm_source=pHelmIvP` and `facing_policy=course`. Enable mission execution with `control_native_actions=true`, `control_mission_execution=true` and a private `auto_recovery_directory`. The fixture owns native arming and disarming.

The provider verifies addressed readbacks of `SYS_AUTOSTART=10045`, `MAV_FWDEXTSP=1`, `COM_OBL_RC_ACT=5` (Hold) and `0.5 ≤ COM_OF_LOSS_T ≤ 1.5` seconds. The native witness used `COM_OF_LOSS_T=0.7`. Set and verify these on the simulator side. The provider requests readbacks and checks their age. Supply current mode, raw GNSS, horizontal estimator, position and HOME evidence. The fixture sets the GPS and mode stream rates on the simulator. Its configuration leaves ordinary firmware stream requests disabled.

## Live helm

Post `acquire` on `MAVMOOS_NAV_CONTROL` once armed, stationary in native Hold, with both the active and intended modes reporting Hold. The bridge prestreams bounded zero horizontal velocity, requests Offboard, and requires fresh selected heartbeat and `CURRENT_MODE` evidence of Offboard. It then consumes fresh course and speed from stock `pHelmIvP`. Course is degrees clockwise from north. Speed is 0–1.5 m/s. Positive speed requires a course within 70° of current heading. The host also limits operation to 100 m from the current home reference. PX4 steers and tracks north/east velocity. The `SET_POSITION_TARGET_LOCAL_NED` mask selects horizontal velocity, with independent yaw and altitude outside this profile.

A short, recognized stock all-stop handoff can supply zero horizontal velocity within the original handoff window and then resume with fresh helm input. Silence, expired input, incompatible mode and explicit `release` retire local authority. An Offboard stream withdrawal lets PX4 apply its configured native Hold. If PX4 still intends Offboard while Hold is active, select native Hold through the owned simulator and wait for fresh stopped evidence before sending another `acquire`. After a takeover into an incompatible mode, the bridge stops sending and leaves mode selection to the operator.

## Native missions

1. In disarmed, stopped Hold, upload through `MAVMOOS_MISSION_REQUEST`. Use two to eight `MAV_CMD_NAV_WAYPOINT` rows in the global-relative-altitude frame (integer coordinates), with zero altitude, simple parameters, and all items executable. Sequence zero is the first waypoint. Each row must be within 60 m of the bound home position. See the [fixture](../../tests/px4_rover/run_ackermann_app.py) for a complete upload string.
2. Require a completed upload, matching readback and `MAVMOOS_STORED_MISSION_PROOF`. Its nonzero PX4 plan ID must match current `MISSION_CURRENT`. Explicitly arm in native Hold and wait for `MAVMOOS_MISSION_EXECUTION_STATUS` to report `start_ready=true`.
3. Post `schema=1,id=rover-run,action=start,verification_id=...,content_id=...,proof_revision=...,timeout=120` on `MAVMOOS_MISSION_CONTROL` using the current proof. Use a new request ID for each `pause`, `resume`, `abort` or `release`, with `target=rover-run`. Observe execution result, status and progress. Start and resume require fresh native mode and cursor evidence. Pause requires confirmed native Hold and stopped dwell. The lease stays held across pause and natural completion. Release requires confirmed Hold and stopped evidence.
4. `abort` is a stop-only action for a retained mission lease. It can request Hold while Mission remains in control. If a pilot takes over, the final send guard respects the new mode. Mode or link uncertainty retains the ownership fence. Do not repeat start or resume to resolve an uncertain outcome.

The FC may continue an onboard mission when the bridge exits or communication is lost. On restart, reuse the private recovery directory. The new process reports retained uncertainty on `MAVMOOS_AUTO_RECOVERY_STATUS`. Select native Hold, confirm both modes and a fresh stopped dwell, then send `schema=1,id=rover-handback,action=handback,session=...` on `MAVMOOS_AUTO_RECOVERY_REQUEST` using the current process session. Successful handback clears the fence. Starting a mission or acquiring helm is a separate request.

`ACK_OBSERVED` is diagnostic. Pause and resume use the same MAVLink command number, so an old denial can arrive during a newer request. A locally sent phase settles only from its post-handoff observations within the original operation deadline. Unresolved effects retain the mission lease.

## What confirms a stop

`MAVMOOS_PX4_ROVER_STATUS` separates `config_verified`, `stop_confirmed`, `hold_ready` and `active_intended_hold`. The stopped flag uses selected, fresh `GPS_RAW_INT`. At least five reports must span 1.5 seconds in both local and producer time. Each needs a 3D fix or better, at least six known satellites, horizontal accuracy at most 1.5 m, reported speed accuracy at most 0.45 m/s, and measured horizontal speed at most 0.25 m/s. Pairwise excursion must stay within 1 m. Invalid, stale or regressed evidence breaks the dwell. Duplicate reports cannot advance it.

This is a stationarity decision for the qualified simulator configuration. Actual speed is independently measured from SIH ground truth during qualification. The profile has no field qualification or generalized braking guarantee. A different drive layout or receiver warrants its own motion and stop criteria.
