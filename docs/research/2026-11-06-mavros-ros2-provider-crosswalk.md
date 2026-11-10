# MAVMOOS v1 evidence and MAVROS ROS 2 crosswalk

November 6, 2026. MAVMOOS builds 34 startup providers. Five own firmware-specific control and 29 supply shared observations, protocols or payload workflows. All 34 complete their declared v1 workflow and have required local software tests with happy-path and adverse assertions. The tracker cites 68 distinct required checks across the providers. Every cited check appears and passed in the archived release JUnit. The full software run passed 158 checks, including 155 required and three optional. The same release campaign qualified a relocated Linux ARM64 kit containing all 34 modules, loaded eight modules in its consumer test, and passed seven owned native SITL scenarios. The CI workflow definition uses manual dispatch. The passing receipts describe local execution at exact source and artifact identities. [Provider tracker](../plugin-progress.md) · [Test registry](../../config/capabilities/copter.json) · Local release archive · CI workflow

This comparison covers the original v1 milestone. The subsequent PX4 Ackermann increment adds a sixth control provider and records how it uses the same MAVROS command and mission protocol references.

The comparison uses MAVROS ROS 2 release 2.16.0 at [5c68b90](https://github.com/mavlink/mavros/commit/5c68b905ab30de6ce630822dc46c33467e8f23ea). MAVROS documents 25 standard entries and 40 extras. The standard manifest comments out the `dummy` authoring example, giving 24 loadable standard classes and 40 extras. Both manifest blobs match the earlier MAVMOOS source-review pin `2d3f4db1101c7950c8817a9d788f0d11106fb261`. The earlier scope proposal counted references worth studying. This review checks implemented overlap in both message and direction or in a specific operator workflow. [MAVROS release](https://github.com/mavlink/mavros/releases/tag/2.16.0) · [Plugin catalog](https://mavros.readthedocs.io/en/stable/plugins/) · [Standard manifest](https://github.com/mavlink/mavros/blob/5c68b905ab30de6ce630822dc46c33467e8f23ea/mavros/mavros_plugins.xml) · [Extras manifest](https://github.com/mavlink/mavros/blob/5c68b905ab30de6ce630822dc46c33467e8f23ea/mavros_extras/mavros_plugins.xml) · [Previous scope proposal](2026-11-06-mavros-plugin-scope.md) · [Pinned source review](../design/2026-11-05-provider-reference-crosswalk/README.md)

| MAVROS release catalog | Implemented overlap | Related, different contract | Outside v1 |
| --- | ---: | ---: | ---: |
| 24 loadable standard | 18 | 1 | 5 |
| 40 extras | 20 | 5 | 15 |

One MAVROS entry can overlap several MAVMOOS providers. Several MAVROS entries can also inform one provider. For example, MAVMOOS combines MAVROS `waypoint`, `geofence`, and `rallypoint` concepts under `missions`, splits the upstream `imu` publication surface across `attitude`, `imu` and `air_data`, and puts native actions and helm setpoints together under each vehicle-control provider. The common app owns authority, final-send checks and the FC connection. All five control providers consume ordinary stock pHelmIvP outputs through the shared adapter. [MAVMOOS architecture](../architecture.md) · [Capability map](../capability-map.md) · [MAVROS IMU source](https://github.com/mavlink/mavros/blob/5c68b905ab30de6ce630822dc46c33467e8f23ea/mavros/src/plugins/imu.cpp)

## Following upstream changes

Keep the provider boundary where the local operation shares a resource, authority or lifetime. Keep a source map from upstream files and wire messages to their MAVMOOS owners and tests.

| Upstream review path | MAVMOOS owner and reason | Regression entry points |
| --- | --- | --- |
| `waypoint.cpp`, `geofence.cpp`, `rallypoint.cpp` and shared `mission_protocol_base.cpp` | `missions` owns one FC transfer slot with typed flight, fence and rally data. Readback, cancellation and reply ownership are shared. The plan types retain their own validators and permissions. Copter AUTO holds a separate execution lease over verified flight-plan content. | `mission_runtime`, `mission_inspection_loopback`, `fence_operations`, `protocol_faults`, `mission_execution_runtime` |
| `imu.cpp` attitude handlers | `attitude` publishes fused Euler and quaternion orientation. Its age, reference and validity follow the attitude estimates. | `attitude_provider`, `observer_host` |
| `imu.cpp` raw and scaled inertial handlers | `imu` publishes per-instance accelerometer and gyro observations. A fresh fused attitude supplies no new raw sample. | `imu_provider_observations`, `observer_matrix_ardupilot` |
| `imu.cpp` pressure handler, `wind_estimation.cpp` and `vfr_hud.cpp` | `air_data` owns pressure, wind, HUD and airspeed families with their separate provenance and ages. | `air_data_observations`, `observer_extensions_plane` |
| `command.cpp` and the setpoint plugins | The selected control provider owns mode-specific command and target semantics. The shared host and helm adapter own freshness and authority across native, Guided, Offboard and stock-helm inputs. | [Command and transition matrix](../development/control-qualification.md) |

For an upstream update, compare the two exact MAVROS refs and review the changed handlers by message, direction, units, frame and lifecycle. Add the impacted local tests to the review and record the new ref with the decision. This also works when the plugin manifests are identical. For example, comparing MAVROS 2.16.0 with the repository's later source-review pin shows an added ROS NED Euler topic in `imu.cpp`. The relevant MAVMOOS review is the fused `attitude` output. The same comparison shows receive staging, target filtering and plan-ID tracking changes in `MissionBase`. Those call for a joint review of the three local plan types and the verified flight-plan boundary used by AUTO. A remote plan ID can identify a change without proving that the returned mission items match. [Exact upstream comparison](https://github.com/mavlink/mavros/compare/5c68b905ab30de6ce630822dc46c33467e8f23ea...2d3f4db1101c7950c8817a9d788f0d11106fb261) · [MAVROS MissionBase](https://github.com/mavlink/mavros/blob/5c68b905ab30de6ce630822dc46c33467e8f23ea/mavros/src/plugins/mission_protocol_base.cpp)

GPS has two established receive paths. `telemetry` projects the FC's fused `GLOBAL_POSITION_INT` into MOOS navigation. `system_status` exposes primary and secondary raw receiver fixes and separate RTK baseline diagnostics. Sending a replacement fix to the estimator with `GPS_INPUT` is a separate producer workflow, as is sending RTCM corrections. The current bounded external estimator input uses `ODOMETRY` in the dedicated ExternalNav bench. [GNSS observations](../plugins/system_status.md) · [GPS_INPUT wire definition](https://mavlink.io/en/messages/common.html#GPS_INPUT) · [ExternalNav](../plugins/external_navigation.md)

Passive provider checks can usually replay packets through the real app and assert the value, age, source and zero command output. Sequence or state still matters for reports such as event histories and mode catalogs. Transactions such as missions, parameters, logs and files also need interruption, cancellation and readback checks. For example, a camera capture needs both the command response and an independently observed new image, so a missing ACK cannot safely be treated as permission to take a second shot.

## Confidence in each MAVMOOS provider

**Workflow (12)** means an owned native simulator exercised the scoped operation and an independent observation verified its result. The camera and gimbal cases use modeled payload components alongside native Copter. **Observation (16)** means at least one real simulator producer supplied a live or replayed wire witness. Some message families in these rows still use generated evidence. **Modeled (6)** means the declared slice has positive and adverse software and app-loopback evidence from generated inputs. All three labels are bounded to the listed workflow, firmware, frame and evidence identity. Physical hardware and broader firmware or MAVROS parity fall outside these results. Read the linked provider receipts for their exact identities. Release verdict · Final provider receipts

### Firmware control

| Provider | Confidence | MAVROS overlap | Demonstrated scope |
| --- | --- | --- | --- |
| `arducopter_control` | Workflow | `command`, `setpoint_position`, `setpoint_raw`, `setpoint_velocity` | Copter 4.6.3 quad lifecycle, stock-helm Guided route/hold and observed pilot handback. Separate simple AUTO execution and restart recovery. Precision Loiter evidence covers command acceptance. Parity · Recovery · Release |
| `ardurover_control` | Workflow | `command`, selected position and velocity targets | Ground rover and motorboat stock-helm tracks, native Hold/RTL and boat Loiter under modeled disturbance, pilot takeover and reacquisition. Rover · Release |
| `arduplane_control` | Workflow | `command`, selected Guided targets | Conventional Plane 4.6.3 airborne course and height, airspeed bounds, Loiter/RTL, pilot FBWA handback and loss. The fixture owns takeoff. Plane |
| `px4_control` | Workflow | `command`, `setpoint_position`, `setpoint_raw`, `setpoint_velocity` | Linux SIH multicopter Offboard on the pinned 1.18.0 release candidate and stable 1.17.0 with settled HOME. Two stock-helm tracks, silence and explicit Hold handback/reacquire. The fixture owns arming, takeoff and landing. PX4 comparison · Release |
| `ardusub_control` | Workflow | `command`, selected position and velocity targets | Vectored ArduSub 4.6.0-dev and stable 4.7.2. Measured dive and translation, captured position, Position Hold, native Depth Hold on link loss and Manual takeover. Sub comparison · Release |

### State and diagnostics

| Provider | Confidence | MAVROS overlap | Demonstrated scope |
| --- | --- | --- | --- |
| `telemetry` | Observation | `global_position`, `home_position`, `sys_status` | Selected native position, HOME, state and estimator data feed ordinary MOOS NAV under an explicit origin and freshness policy. Release |
| `system_status` | Observation | `sys_status`, `gps_status`, `gps_rtk` | Native Copter status, board power and estimator reports. Additional GNSS, RTK and power families have generated coverage. Power · Diagnostics |
| `attitude` | Observation | Fused-attitude portion of `imu` | Both fused Euler and quaternion streams observed with stock helm and concurrent protocol reads. Each sample keeps its own age. Attitude |
| `imu` | Observation | Raw/scaled portions of `imu`, `vibration` | Selected Copter raw/scaled IMU and generated vibration. High-resolution IMU is a separate slice. Copter stable |
| `rc_io` | Observation | `rc_io` | Native Copter RC and scoped Rover observations. Servo reports describe commanded outputs. RC |
| `clock_status` | Modeled | Receive-side `sys_time` | Generated boot/UTC and independently aged TIMESYNC request/reply shapes. Active synchronization is a separate workflow. [Clock scope](../plugins/clock_status.md) |
| `link_status` | Modeled | `tdr_radio` for RADIO_STATUS. `cellular_status` is directionally related. | Generated selected radio and cellular observations with original age. [Link scope](../plugins/link_status.md) |
| `debug_values` | Modeled | Receive-side `debug_value` | Generated ArduPilot- and PX4-shaped scalars, vectors and arrays. Debug |
| `event_timeline` | Modeled | No dedicated release class | Generated event sequences, gaps and restart/replay status. Events |
| `component_metadata` | Modeled | No dedicated release class | Modeled declaration and a private local General v1 mirror. Covers source, integrity, schema, version and age. Metadata |
| `mode_catalog` | Observation | State display is adjacent to `sys_status` | Native PX4 capture replayed all 27 mode rows with current and intended state. Generated cases exercise revision changes, conflicts and fallback. Mode evidence |

### Navigation and sensors

| Provider | Confidence | MAVROS overlap | Demonstrated scope |
| --- | --- | --- | --- |
| `odometry` | Observation | `local_position`, `global_position` origin, `odometry`, `wheel_odometry` | Native Copter and PX4 local NED plus native PX4 odometry. Generated origin and signed wheel distance. Odometry |
| `air_data` | Observation | `wind_estimation`, `vfr_hud`, pressure portion of `imu` | Native Copter HUD and two native Plane airspeed sources. Generated wind, pressure and other report families have independent ages. Air data |
| `esc_status` | Observation | `esc_status`, `esc_telemetry` | Native ArduPilot ESC report plus generated common ESC info and status. ESC |
| `distance_sensor` | Observation | `distance_sensor`. `obstacle_distance` is directionally related. | Native Copter per-sensor range and generated obstacle sectors. Distance · [Obstacle sectors](../plugins/distance_sensor.md) |
| `optical_flow` | Observation | `optical_flow`, `px4flow` | Native OPTICAL_FLOW (100) and generated OPTICAL_FLOW_RAD (106), each independently aged. Flow |
| `terrain` | Observation | Receive-side `terrain` | Native Copter grid requests and terrain reports. The scoped workflow observes availability and backlog. Terrain |
| `navigation_monitor` | Observation | `nav_controller_output`, `guided_target`, readbacks in `setpoint_raw` | Native Copter NAV_CONTROLLER_OUTPUT (62) and generated masked local and global target readbacks (85/87). Controller evidence |
| `landing_target` | Modeled | Receive-side `landing_target` | Generated reports from the selected component through the real app. Covers frame, validity and age. [Landing-target scope](../plugins/landing_target.md) |
| `water_depth` | Observation | No dedicated release class | Native Rover motorboat WATER_DEPTH from two simulated downward sensors. This ArduPilot message has its own reference and health fields. Depth |
| `external_navigation` | Workflow | Transmit-side `odometry` | A single bounded ODOMETRY input on disarmed Copter 4.6.3. Independent telemetry and DataFlash verify estimator motion and later aiding loss. ExternalNav |

### Protocols and payloads

| Provider | Confidence | MAVROS overlap | Demonstrated scope |
| --- | --- | --- | --- |
| `parameters` | Workflow | `param` | Named reads, paced inventory, and an allowlisted simulator wind write with readback and restoration. Parameters |
| `missions` | Workflow | `waypoint`, `geofence`, `rallypoint` | Flight-plan transfer/readback/clear, typed flight/fence/rally inspection, and a reversible Copter fence edit. Rally scope is inspection. The Copter control provider owns the qualified AUTO execution workflow. Missions · Fence |
| `traffic` | Observation | Receive-side `adsb` | Native Copter ADS-B inbound observation with bounded aircraft slots and individual ages. Traffic |
| `flight_logs` | Workflow | `log_transfer` | Native Copter catalog and single-log download with exact bytes, staging, cancellation and retirement. Logs |
| `mavlink_files` | Workflow | `ftp` | Native Copter list/get/cancel inside a bounded simulator-owned root. FTP |
| `camera_status` | Workflow | `camera` image evidence | Copter plus a modeled camera. The scoped MOOS workflow adds status and one-shot capture with ACK, fresh image evidence, file creation and duplicate handling. Payloads |
| `gimbal_status` | Workflow | `gimbal_control` | Copter plus a modeled gimbal. The scoped workflow claims, points, confirms pose, releases and respects foreign takeover. Payloads |
| `remote_id` | Observation | `open_drone_id` is directionally related. | Native PX4 location and modeled identity and transmitter health. Observe and redact on the bound route. Remote ID |

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

The reasons below explain the current boundary where no individual deferral decision exists. The original scope plan already prioritizes typed requests, explicit authority and a concrete producer or operator use case before adding new ingress or egress. [Scope proposal](2026-11-06-mavros-plugin-scope.md) · [Capability and firmware roadmap](../plans/2026-11-05-capability-and-firmware-roadmap/README.md)

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

The same reference catalog still fits the original grouping decision. The proposed 19 standard areas now resolve to 18 overlapping classes and the related altitude case. The proposed extras list included two directionally related transmitters, while the installed optical-flow provider also covers `px4flow`'s receive family. The new payload, fence and ExternalNav receipts establish their specific workflows. The MAVROS counts are useful for tracking those references and gaps. The 34-provider tracker measures completion of the MAVMOOS v1 target. [Scope proposal](2026-11-06-mavros-plugin-scope.md) · [Provider tracker](../plugin-progress.md)
