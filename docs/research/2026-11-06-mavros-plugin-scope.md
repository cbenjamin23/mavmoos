# MAVROS plugin scope and MAVMOOS counterparts

November 6, 2026. This is a proposed scope based on the official MAVROS ROS 2 index. It counts upstream entries worth consulting for at least one operator capability. The standard table has 25 entries, including the example `dummy`, and the extras table has 40. The ROS wiki blocked retrieval during this review. MAVROS separates standard and optional plugin packages and selects individual plugin classes at startup. The standard and extras manifests each export many classes from a package library. MAVMOOS ships one standard shared module per provider and chooses a composition at startup. [MAVROS index](https://mavros.readthedocs.io/en/stable/plugins/) · [Startup selection](https://mavros.readthedocs.io/en/stable/nodes/#plugin-loading) · [Standard registration](https://github.com/mavlink/mavros/blob/ros2/mavros/mavros_plugins.xml) · [Extras registration](https://github.com/mavlink/mavros/blob/ros2/mavros_extras/mavros_plugins.xml)

The module granularity differs in both directions. MAVROS's `command` and several `setpoint_*` modules inform three alternative MAVMOOS vehicle-control providers. Each groups native FC actions and helm behavior under one ownership policy. MAVROS's `waypoint`, `geofence`, and `rallypoint` inform one MAVMOOS `missions` provider. Conversely, MAVROS's `imu` publishes fused attitude, raw IMU and pressure together. MAVMOOS gives those observations to `attitude`, `imu`, and `air_data`, with independent source and age policies. [MAVROS IMU API](https://mavros.readthedocs.io/en/stable/plugins/std/imu/) · [MAVMOOS capability map](../capability-map.md)

## Standard entries worth tracking (19 of 25)

| Area | MAVROS references | MAVMOOS scope |
| --- | --- | --- |
| Observations (10) | `altitude`, `global_position`, `home_position`, `local_position`, `imu`, `nav_controller_output`, `rc_io`, `sys_status`, `sys_time`, `wind_estimation` | Existing telemetry, sensor, time and controller-state modules cover relevant subsets. Preserve frame/reference, validity and acquisition ages. |
| Protocols (5) | `ftp`, `geofence`, `param`, `rallypoint`, `waypoint` | Existing files, parameter and plan workflows. Fence and rally inspection come before any mutation workflow. |
| Guided and native control (4) | `command`, `setpoint_position`, `setpoint_raw`, `setpoint_velocity` | Reuse the command and setpoint wire semantics, including selected masks and readback. Keep public MOOS requests typed and vehicle-specific, with one shared authority and final-send boundary. |

The example `dummy` is a reference for authors, while MAVMOOS already has its own plugin examples. Direct actuator and pilot input, raw acceleration/attitude targets and trajectory streaming need a vehicle-specific reason before they enter the control roadmap. That leaves `actuator_control`, `manual_control`, `setpoint_accel`, `setpoint_attitude`, and `setpoint_trajectory` outside the initial coverage target.

## Extra entries worth tracking (21 of 40)

| Area | MAVROS references | MAVMOOS scope |
| --- | --- | --- |
| Targets and surroundings (4) | `guided_target`, `distance_sensor`, `obstacle_distance`, `terrain` | Target monitoring, range, obstacle sectors and terrain evidence. Some MAVROS paths write or request data; the current MAVMOOS modules observe it. |
| Vehicle observations (5) | `gps_status`, `gps_rtk`, `odometry`, `wheel_odometry`, `vibration` | Existing system, odometry and IMU modules cover relevant observations. Correction injection and estimator input are separate workflows. |
| Flow, air and link (4) | `optical_flow`, `vfr_hud`, `tdr_radio`, `cellular_status` | Flow, HUD, and link diagnostics. For cellular status, use the common wire fields for an inbound observation. |
| Traffic and diagnostics (4) | `adsb`, `debug_value`, `esc_status`, `esc_telemetry` | Traffic, debug and ESC observations, with source-specific limits and no implied status or control claims. |
| Payload and landing (3) | `camera`, `gimbal_control`, `landing_target` | Keep payload workflows together. Current modules expose observations; deliberate camera and gimbal operations and landing-target control need their own qualification. |
| Flight logs (1) | `log_transfer` | The existing file workflow adds completeness checks, cancellation and readback to the standard log protocol. |

The other 19 extras stay conditional on a real workflow. One qualified estimator input may use `gps_input`, `mocap_pose_estimate`, `vision_pose` or `vision_speed` as a reference. Native camera timing, a selected legacy range message, or a PX4-specific producer may change the useful set after those producers are exercised. Keep Remote ID observation separate from broadcast. Avoid opening a raw command or message tunnel to gain a line in the catalog. [Current roadmap](../plans/2026-11-06-overnight-plugin-queue/README.md)

The best additions within the current capabilities are a verified fence or rally edit workflow, a complete camera capture workflow, and a complete gimbal orientation workflow. Further firmware coverage can reuse already-built protocol and observation modules after actual wire and behavior qualification. The 19 and 21 counts include existing and partial coverage and have no implied one-to-one match to MAVMOOS modules or completion claims. [Pinned source crosswalk](../design/2026-11-05-provider-reference-crosswalk/README.md)
