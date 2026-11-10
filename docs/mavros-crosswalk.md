# MAVROS crosswalk

This guide maps MAVROS plugin interfaces to MAVMOOS. It uses MAVROS ROS 2 2.16.0 at [commit 5c68b90](https://github.com/mavlink/mavros/commit/5c68b905ab30de6ce630822dc46c33467e8f23ea). Related interfaces may have different message directions or cover different workflows. See the [MAVMOOS plugin catalog](plugin-catalog.md) for supported interfaces and tests.

## Standard MAVROS classes

### Implemented overlap

| MAVROS classes | Where the workflows meet and where scope stops |
| --- | --- |
| `command`, `setpoint_position`, `setpoint_raw`, `setpoint_velocity` | Vehicle-control providers use selected command and position/velocity target messages. `navigation_monitor` observes target readback. MAVMOOS exposes typed vehicle workflows behind one authority policy. Generic command, setpoint and ROS frame interfaces are outside this surface. |
| `global_position`, `home_position`, `local_position` | `telemetry` and `odometry` use selected global, HOME, origin and local state. Changing HOME or the origin and publishing ROS TF are distinct capabilities. |
| `imu`, `wind_estimation` | `attitude`, `imu` and `air_data` separate fused state, inertial samples, pressure and wind by source and age. |
| `nav_controller_output`, `rc_io`, `sys_status`, `sys_time` | Controller, radio control, state and time observations overlap. Raw RC overrides and active clock synchronization remain separate workflows. |
| `ftp`, `param` | Scoped list/get and parameter reads share the wire protocols. Parameter writes are allowlisted in SITL. General file mutation and parameter writing are outside v1. |
| `geofence`, `rallypoint`, `waypoint` | Shared `missions` handles typed plans. The Copter fence bench exercises reversible edit. Rally points are inspected. Flight execution is subject to the control provider's separate policy. |

### Related or omitted

| MAVROS classes | Scope decision or reason |
| --- | --- |
| `altitude` | Related. MAVROS's class decodes the dedicated ALTITUDE message. The MAVMOOS altitude projections use selected position and HOME observations, with profile-specific references. Adding ALTITUDE as another source would require a reference and validity policy. [MAVROS ALTITUDE source](https://github.com/mavlink/mavros/blob/5c68b905ab30de6ce630822dc46c33467e8f23ea/mavros/src/plugins/altitude.cpp) |
| `actuator_control`, `manual_control` | Omitted. The v1 control profiles give the FC its inner loops and the pilot its independent takeover path. A low-level or virtual stick input would add an authority owner with its own safety and timeout rules. |
| `setpoint_accel`, `setpoint_attitude`, `setpoint_trajectory` | Omitted. The existing helm contract supplies course, speed and a vertical reference for the vehicle profiles that use one. Acceleration, attitude and trajectory are separate intent models with distinct timing and loss behavior. |
| `dummy` | Documentation-only example. The XML disables it. MAVMOOS supplies external observer and disarmed command examples for its own host ABI. [Observer](../../examples/heartbeat_observer/) · [Command](../../examples/disarmed_command/) |

## Extra MAVROS classes

### Implemented overlap

| MAVROS classes | Where the workflows meet and where scope stops |
| --- | --- |
| `adsb`, `debug_value`, `distance_sensor` | `traffic`, `debug_values` and `distance_sensor` share inbound report families. The corresponding transmit paths are outside those observer workflows. |
| `esc_status`, `esc_telemetry`, `gps_status`, `gps_rtk` | `esc_status` and `system_status` cover selected ESC and GNSS/RTK observation families. Sending correction data would add a separate input and producer contract. [MAVROS RTK source](https://github.com/mavlink/mavros/blob/5c68b905ab30de6ce630822dc46c33467e8f23ea/mavros_extras/src/plugins/gps_rtk.cpp) |
| `guided_target`, `terrain`, `landing_target` | `navigation_monitor` observes global target readback. `terrain` and `landing_target` observe selected report families. Terrain tile service and precision-landing target injection remain separate workflows. |
| `odometry`, `wheel_odometry` | `odometry` preserves selected incoming samples and raw wheel distance. `external_navigation` adds one qualified ODOMETRY injection path. MAVROS also integrates wheel-derived motion and supports ROS frame and producer interfaces. |
| `optical_flow`, `px4flow` | `optical_flow` observes both raw and integrated messages. The latter shares OPTICAL_FLOW_RAD with `px4flow`. Both upstream classes also support injection. [MAVROS raw flow](https://github.com/mavlink/mavros/blob/5c68b905ab30de6ce630822dc46c33467e8f23ea/mavros_extras/src/plugins/optical_flow.cpp) · [MAVROS integrated flow](https://github.com/mavlink/mavros/blob/5c68b905ab30de6ce630822dc46c33467e8f23ea/mavros_extras/src/plugins/px4flow.cpp) |
| `tdr_radio`, `vfr_hud`, `vibration` | `link_status` shares common RADIO_STATUS. The older ArduPilot RADIO payload is separate. `air_data` and `imu` share HUD and vibration observations. [MAVROS radio source](https://github.com/mavlink/mavros/blob/5c68b905ab30de6ce630822dc46c33467e8f23ea/mavros_extras/src/plugins/3dr_radio.cpp) |
| `camera`, `gimbal_control` | `camera_status` consumes the image-captured report as one confirmation in a bounded capture operation. `gimbal_status` uses selected modern gimbal manager/device protocol paths. [MAVROS camera source](https://github.com/mavlink/mavros/blob/5c68b905ab30de6ce630822dc46c33467e8f23ea/mavros_extras/src/plugins/camera.cpp) · Payload evidence |
| `log_transfer` | `flight_logs` covers catalog, one log download and retirement. Erasing onboard logs falls outside the scoped workflow. |

### Related or omitted

MAVMOOS adds new inputs and outputs when a workflow has a defined source, an authority policy and tests for how the data is used.

| MAVROS classes | Scope decision or reason |
| --- | --- |
| `cellular_status`, `obstacle_distance`, `open_drone_id` | Related. The MAVROS classes supply outbound CELLULAR_STATUS, OBSTACLE_DISTANCE and Open Drone ID packets. MAVMOOS uses the corresponding capability areas for inbound diagnostics, obstacle observation and redacted Remote ID, with no producer or broadcast role. [Cellular source](https://github.com/mavlink/mavros/blob/5c68b905ab30de6ce630822dc46c33467e8f23ea/mavros_extras/src/plugins/cellular_status.cpp) · [Obstacle direction](https://mavros.readthedocs.io/en/stable/plugins/#extra-plugins-mavros_extras) · [Open Drone ID source](https://github.com/mavlink/mavros/blob/5c68b905ab30de6ce630822dc46c33467e8f23ea/mavros_extras/src/plugins/open_drone_id.cpp) |
| `mount_control`, `rangefinder` | Related. These use legacy mount and range messages. v1 covers modern gimbal manager/device workflows and common DISTANCE_SENSOR. A legacy fallback would need a specific producer and its own evidence. [Mount source](https://github.com/mavlink/mavros/blob/5c68b905ab30de6ce630822dc46c33467e8f23ea/mavros_extras/src/plugins/mount_control.cpp) · [Rangefinder source](https://github.com/mavlink/mavros/blob/5c68b905ab30de6ce630822dc46c33467e8f23ea/mavros_extras/src/plugins/rangefinder.cpp) |
| `fake_gps`, `gps_input`, `mocap_pose_estimate`, `vision_pose`, `vision_speed` | Omitted. These add distinct estimator inputs, with independent time, frame, uncertainty and source-loss contracts. v1 qualifies one bounded ODOMETRY source on a dedicated disarmed bench. |
| `obstacle_distance_3d`, `trajectory` | Omitted. They require an external obstacle or trajectory producer, a consumer policy and behavior evidence. v1 exposes observation and typed navigation surfaces. |
| `hil`, `sim_state` | Omitted. The native fixtures own simulator coupling and truth measurements. MOOS observes selected FC estimates and reports. |
| `cam_imu_sync`, `mag_calibration_status`, `play_tune` | Omitted. Camera timing, calibration reporting and aural commands are separate peripheral/operator workflows with no current v1 use case. |
| `companion_process_status`, `onboard_computer_status` | Omitted. These publish dedicated MAVLink reports from onboard software or hardware. The current MOOS app provides local diagnostics and scoped host status. |
| `tunnel` | Omitted. A raw bidirectional payload route would need explicit ownership, target filtering and budgeting beyond the typed provider interfaces. |
