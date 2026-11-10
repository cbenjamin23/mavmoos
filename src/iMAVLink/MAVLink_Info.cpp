/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: MAVLink_Info.cpp                                */
/*    DATE: November 4, 2026                                 */
/************************************************************/
#include "MAVLink_Info.h"
#include <cstdlib>
#include <iostream>

void showSynopsis()
{
  std::cout << "iMAVLink: capability-oriented MAVLink and stock-helm bridge for MOOS.\n"
               "Receive-only by default. Control requires explicit owned localhost Copter/Rover SITL.\n";
}
void showHelpAndExit()
{
  showSynopsis();
  std::cout << "\nUsage: iMAVLink mission.moos [--alias=NAME]\n"
               "  --help       Show this help\n"
               "  --example    Show ProcessConfig\n"
               "  --interface  Show MOOS interface\n"
               "  --version    Show increment version\n";
  std::exit(0);
}
void showExampleConfigAndExit()
{
  std::cout << R"(ProcessConfig = iMAVLink
{
  AppTick    = 20
  CommsTick  = 20
  bind_address = 127.0.0.1
  bind_port = 14600
  peer_address = 127.0.0.1
  peer_port = 14601
  target_system = 1
  target_component = 1
  vehicle_profile = arducopter // or ardurover, ardurover_boat, arduplane, arduplane_quadplane, ardusub
  // PX4 profiles: px4_multicopter, px4_fixed_wing, px4_standard_vtol, px4_rover_ackermann
  // copter_frame = quad // quad, hexa or octa on the matched owned Copter fixture
  expected_type = 2       // use 13 for hexa, 14 for octa, or omit with explicit copter_frame
  expected_autopilot = 3  // MAV_AUTOPILOT_ARDUPILOTMEGA
  max_datagrams_per_iter = 32  // 1..128; max 4096 bytes each
  max_queue_age = 0.25 // 0.01..2 s; conservative queue-empty-floor age bound
  heartbeat_timeout = 2.0
  position_timeout = 0.5
  landed_timeout = 2.0
  estimator_timeout = 3.0
  publish_interval = 0.2
  simulation_only = false // true requires matching native simulator PID and owned loopback endpoints
  shadow_mode = false
  helm_enabled = false
  firmware_requests = true // bounded read-only version requests in explicit SITL; RX_ONLY/shadow still transmit zero
  parameter_list_mode = paced // indexed reads, window <=4; default list timeout 60s
  // streamed is an explicit disarmed parameter/telemetry-only bench profile (20s default)
  // plugin_directory = /path/to/build/plugins
  // plugins = telemetry,parameters,arducopter_control // optional; matching ABI 10 builds
  // External observer alternative (separately built, matching headers/toolchain):
  // plugins = example_heartbeat_observer
  // plugin_directory = /absolute/path/to/standalone-example/plugins
  // plugin.example_heartbeat_observer.sample_timeout = 2.0
  // plugin.example_heartbeat_observer.publish_interval = 0.25
  // Local settings are checked as one startup batch; unknown owners/keys/duplicates reject.
  // Transport, target identity and control permission remain host-owned settings.
  // control_native_actions = true // startup facet of the selected control provider
  // control_helm = true           // startup facet; default follows helm_enabled for implicit composition
  // facing_policy = course // Copter: course|keep|heading; Rover: course only
  // facing_heading = 0
  // operation_helm_enabled = false // Copter opt-in; requires ordinary mission behavior bindings
  // operation_helm_tasks = survey,voronoi // opt-in selectors for configured stock helm behaviors
  // operation_helm_adjustments = false // opt-in stock behavior speed/altitude updates
  // control_copter_adjustments = false // off-helm Guided course and speed commands
  // command_speed_min = 0.25 // positive simulator operating minimum in m/s
  // control_precision_loiter = false // explicit observed Loiter and PLND proof on enable
  // dead_man_source = pDeadManPost // opt-in native RTL from this exact MOOS producer
  // landing_target_component = 158 // when loading landing_target for Copter loopback
  // operation_timeout = 60
  // operation_radius = 1
  // operation_speed = 1.5
  // plugins = telemetry,parameters,missions,arducopter_control
  // control_mission_execution = false // opt-in verified mission execution with explicit start and recovery
  // auto_recovery_directory = /absolute/private/run/directory // required for AUTO; keep the same directory across restarts
  // control_replay_directory = /absolute/private/replay/directory // optional override for the route's journal
  // mission_timeout = 60
  // mission_response_timeout = 1.5
  // plugins = telemetry,traffic // optional ADS-B observation
  // plugins = telemetry,optical_flow,terrain,navigation_monitor // optional sensor and FC target observations
  // plugins = telemetry,link_status // optional FC-bound radio/cellular observations
  // plugins = telemetry,camera_status,gimbal_status // explicit passive payload component routes
  // camera_component = 100
  // gimbal_component = 154
  // plugin.camera_status.capture_enabled = true // dedicated disarmed Copter SITL only
  // plugin.gimbal_status.point_enabled = true   // dedicated disarmed Copter SITL only
  // plugin.missions.fence_edit = true           // opt-in disarmed Copter fence save/replace/restore
  // PX4 SIH Offboard: vehicle_profile=px4_multicopter, plugins=telemetry,px4_control
  // For the conventional SIH plane, use px4_fixed_wing and facing_policy=course.
  // Standard SIH VTOL uses px4_standard_vtol and facing_policy=course.
  // Aircraft native commands require control_native_actions=true in owned SITL.
  // Ackermann SIH: vehicle_profile=px4_rover_ackermann, plugins=telemetry,px4_rover_control,missions.
  // Set control_mission_execution=true and a private auto_recovery_directory for native missions.
  // Linux loopback-only namespace, exact owned px4 PID, bind 14540, peer 14580.
  // Set firmware_requests=false and control_native_actions=false for that active profile.
  // Vectored Sub SITL: vehicle_profile=ardusub, expected_type=12, expected_autopilot=3.
  // Select plugins=telemetry,ardusub_control, control_helm=true, control_native_actions=false.
  // Set firmware_requests=false.
  // QuadPlane SITL: vehicle_profile=arduplane_quadplane.
  // Set plugins=telemetry,arduquadplane_control,missions,
  // control_mission_execution=true, firmware_requests=false and a private auto_recovery_directory.
  // Set control_helm=true to opt into live stock helm.
  // The default helm profile is stable forward flight. For the owned hover fixture,
  // use plugin.arduquadplane_control.hover_adapter=true and its matching Lua adapter.
  // Disarmed Copter ExternalNav SITL: plugins=telemetry,external_navigation.
  // Set firmware_requests=false and plugin.external_navigation.source to the local producer.
  // plugins = telemetry,component_metadata,mode_catalog,remote_id // passive catalogs and ID
  // component_metadata_component = 196
  // plugin.component_metadata.mirror_directory = /absolute/private/fixture
  // remote_id_component = 236
  // plugin.navigation_monitor.sample_timeout = 2
  // plugin.traffic.sample_timeout = 3
  // plugins = telemetry,mavlink_files // separate owned disarmed Copter SITL file bench
  // plugin.mavlink_files.directory = /absolute/private/run/file-downloads
  // plugin.mavlink_files.timeout = 45
  // plugins = telemetry,flight_logs // optional owned disarmed Copter SITL log workflow
  // plugin.flight_logs.directory = /absolute/private/run/log-downloads
  // plugins = telemetry,system_status // optional read-only diagnostics
  // status_timeout = 3
  // status_publish_interval = 0.5
  // status_text_timeout = 2
  // plugins = telemetry,system_status,attitude // optional read-only observers
  // attitude_timeout = 1
  // plugins = telemetry,imu // optional raw/scaled sensor observations
  // imu_timeout = 1
  // imu_publish_interval = 0.1
  // attitude_publish_interval = 0.1
  // plugins = telemetry,rc_io // read-only input/output observations
  // rc_io_timeout = 1
  // rc_io_publish_interval = 0.1
  // plugins = telemetry,distance_sensor // optional read-only range observations
  // distance_sensor_timeout = 1
  // distance_sensor_publish_interval = 0.1
  // plugins = telemetry,odometry,air_data,esc_status // passive reports from target 1/1
  // plugin.odometry.sample_timeout = 1
  // plugin.odometry.publish_interval = 0.1
  // Legacy plugins=commands/helm names map to one provider with original facets.
  // parameter_timeout = 8
  // parameter_list_timeout = 20
  // Full parameter-table fixture: AppTick=50 and max_datagrams_per_iter=128.
  // Parameter-only simulation can select plugins=parameters without enabling helm.
  // sitl_pid = 12345
}
)";
  std::exit(0);
}
void showInterfaceAndExit()
{
  std::cout << R"(Subscriptions:
  APPCAST_REQ (string, handled by AppCastingMOOSApp)
  LOADCAST_REQ (string, inherited)
  COMMS_POLICY (string, inherited)
  TERM_REPORT_INTERVAL (double, inherited)
Publications:
  APPCAST (standard AppCasting string, when requested)
  MAVMOOS_STATUS (string)
  MAVMOOS_CAPABILITIES (JSON schema=1: execution mode, actual providers/profiles,
    supported capability/request mappings and exclusive ownership claims)
  MAVMOOS_FIRMWARE_STATUS (JSON schema=1: bound FC software/custom versions,
    capability bits, original observation ages and bounded discovery diagnostics;
    reported identity is separate from qualified behavior and control permission)
  MAVMOOS_FIRMWARE_REQUEST_RESULT (JSON schema=1: original epoch/deadline and local outcome)
  MAVMOOS_HEARTBEAT (string)
  MAVMOOS_GLOBAL_POSITION (string)
  MAVMOOS_LANDED_STATE (string)
  MAVMOOS_HOME_POSITION (string)
  MAVMOOS_ESTIMATOR (string)
  Standard inherited MOOS/AppCasting timing and lifecycle diagnostics also apply.
Records: raw family/NAV diagnostics use schema=2; MAVMOOS_STATUS retains schema=1.
Age: age_basis=QUEUE_EMPTY_FLOOR, age_is_upper_bound=true; source age remains unknown.
  rx_mono_s/received_at uses the conservative queue-empty floor; read_mono_s/read_at
  retains actual local read time. This is not kernel arrival or FC generation time.
Ingress status: max_queue_age_s, queue_stale_dropped, queue_clock_dropped,
  queue_draining, rx_age_basis, rx_polls, empty_events, last_queue_age_upper_s,
  max_queue_age_upper_s, last_poll_gap_s, max_poll_gap_s, queue_epoch_at,
  last_read_at, last_effective_at.
  Inspect queue_draining and age/drop counters before treating resumed reads as current.
  See docs/implementation/ingress/ and docs/implementation/rx-only/interface.md.
Home: cached reference, not a periodically fresh pose. Encoding validity is not health.
Telemetry module: NAV_X/Y/LAT/LONG/SPEED/HEADING/COURSE/ALTITUDE/ALTITUDE_MSL,
  NAV_VERTICAL_SPEED, NAV_SAMPLE_RECEIVED_AT, NAV_VALID and quantity-validity diagnostics,
  MAVMOOS_NAV_STATUS. NAV_ALTITUDE is home-relative; NAV_ALTITUDE_MSL is mean-sea-level.
  Sub reports NAV_DEPTH and NAV_DEPTH_VALID with the FC home surface reference.
ArduCopter control provider, optional helm facet: DESIRED_HEADING/SPEED/ALTITUDE, IVPHELM_STATE,
  IVPHELM_ALLSTOP, MAVMOOS_NAV_CONTROL=acquire|release.
Helm results: MAVMOOS_HELM_STATUS; MAVMOOS_SHADOW_COURSE/SPEED/ALTITUDE.
  Rover consumes only DESIRED_HEADING/SPEED; no height/landed requirement for motion.
  Sub consumes DESIRED_HEADING/SPEED/DEPTH. Depth is positive down.
  MAVMOOS_SUB_HANDBACK=hold requests native Position Hold. The surface action
  requests native ascent and observes shallow Depth Hold before completion.
  See MAVMOOS_SUB_STATUS and MAVMOOS_SUB_HANDBACK_RESULT for observed outcomes.
  Status includes readiness, blocked_by, acquire_preparation, next_action,
  acquire_required, producer_resync_required and input ages/remaining lifetime.
  Diagnostics are MODULE_LOCAL; shared host gates remain independent.
  They never acquire control or resynchronize the helm automatically.
ArduCopter control provider, native facet: MAVMOOS_COMMAND=id=unique,action=guided|arm|takeoff|hold|land|rtl|disarm[,height=5]
Results: MAVMOOS_COMMAND_RESULT (ACK and observed completion distinct), MAVMOOS_TX_RESULT,
  MAVMOOS_RUNTIME_STATUS. RX_ONLY and SHADOW never transmit MAVLink.
  Admitted explicit commands revoke NAV and block acquire/resume until their transaction
  terminates or its original operation deadline expires. Finishing never restores NAV.
  Runtime status includes command_leases, resume_permit, control_profile and configuration_valid. Normal handoff resume uses
  a separate bounded host permit; it cannot override a queued HOLD/LAND/RTL request.
  Optional Guided adjustments: change_speed or change_course with delta, reset_speed_min.
  Optional Loiter precision control: precision_loiter_on|precision_loiter_off.
  DEAD_MAN_POST_INTERRUPT=true from configured dead_man_source can request native RTL.
  Optional landing_target provider publishes MAVMOOS_LANDING_TARGET observations only.
Copter operation interface: MAVMOOS_CONTROL_REQUEST (string)
  schema=1,id=unique,action=goto,strategy=guided,lat=42.3585,lon=-71.0875,height=5
  schema=1,id=hold1,action=hold,strategy=native|guided|helm
  schema=1,id=return1,action=return,strategy=native|helm
  schema=1,id=climb1,action=adjust_altitude,strategy=guided,delta=2
  schema=1,id=survey1,action=survey,strategy=helm
  schema=1,id=home1,action=visualize_home
  schema=1,id=cancel1,action=cancel,target=hold1
  Native arm/disarm/takeoff/land/guided are also supported. Unsupported combinations reject.
  Helm strategies require operation_helm_enabled and documented ordinary behavior bindings.
  Results: MAVMOOS_CONTROL_RESULT/STATUS. Cancellation is local; sent effects cannot be recalled.
Copter AUTO interface: MAVMOOS_MISSION_CONTROL (control_mission_execution=true)
  schema=1,id=run,action=start,verification_id=upload1,content_id=mission/1/1,proof_revision=1,timeout=90
  schema=1,id=pause1,action=pause,target=run,timeout=8
  schema=1,id=stop1,action=abort,target=run,timeout=8
  schema=1,id=release1,action=release,target=run,timeout=8
  Selectors come from MAVMOOS_STORED_MISSION_PROOF and are compared to typed host state.
  Initial profile: already armed/airborne/Guided; HOME plus2..7 simple relative-alt waypoints.
  Resume and partial ranges are unsupported. Upload alone grants no execution permission.
  Results/status/progress: MAVMOOS_MISSION_EXECUTION_RESULT/STATUS/PROGRESS.
  MAVMOOS_MISSION_EXECUTION_LEASE reports retained ownership, including uncertain AUTO after loss/expiry.
  Completion or cancellation does not acquire helm. Explicit abort observes Guided handback before release.
  Startup requires an existing private auto_recovery_directory for AUTO. Use it again after a bridge restart.
  MAVMOOS_AUTO_RECOVERY_STATUS reports the current process session and unresolved AUTO ownership.
  MAVMOOS_AUTO_RECOVERY_REQUEST=schema=1,id=unique,action=handback,session=<current process session>
  Fresh non-AUTO observation and an explicit current-session request are required for recovery.
  MAVMOOS_AUTO_RECOVERY_RESULT reports the handback outcome. Acquire control again explicitly.
Rover native actions: MAVMOOS_COMMAND=id=unique,action=guided|arm|hold|rtl|disarm.
  Ground and boat use ardurover_control with explicit matching vehicle_profile.
  Hold is mode 4; Guided is 15; RTL is 11. action=loiter (mode 5) is boat-only.
  Acquire stock helm in armed Guided, or explicitly from Hold (or boat Loiter).
Rover versioned operations: MAVMOOS_CONTROL_REQUEST (string), for example:
  schema=1,id=move1,action=goto,strategy=guided,lat=42.3585,lon=-71.0875,speed=1
  schema=1,id=return1,action=return,strategy=native
  schema=1,id=stop1,action=hold,strategy=guided
  schema=1,id=cancel1,action=cancel,target=move1
  Guided goto/return/hold require armed Guided. Guided hold asks for zero speed.
  MAVMOOS_CONTROL_PROGRESS and MAVMOOS_CONTROL_RESULT report Rover operation outcomes.
Conventional Plane SITL: vehicle_profile=arduplane, plugins=telemetry,arduplane_control.
  Takeoff is part of the simulator fixture. Acquire in verified airborne native Loiter.
  Helm accepts course, 22 m/s ground-speed intent and 45–120 m above home.
  MAVMOOS_NAV_CONTROL=release requests native Loiter; MAVMOOS_COMMAND supports
  id=unique,action=loiter|rtl during owned Guided. The FC controls airspeed.
  Read MAVMOOS_PLANE_STATUS and the plane_* fields in MAVMOOS_RUNTIME_STATUS
  for readiness, watchdog, target readback and handback. RX_ONLY and SHADOW have zero egress.
Simulation parameter module: MAVMOOS_PARAM_REQUEST (string)
  id=unique,action=read,name=FRAME_CLASS
  id=unique,action=list
  id=unique,action=write,name=SIM_WIND_SPD,value=0.25,type=REAL32
  Reads/list require fresh heartbeat, not helm authority or airborne state.
  Lists default to paced indexed reads with a fixed 60s deadline; no full-list burst.
  Explicit streamed mode requires a parameter/telemetry-only disarmed ground bench.
  Writes are restricted to SIM_WIND_SPD/REAL32/0..20 with fresh disarmed ON_GROUND
  and no pending explicit command transaction. Reads/list remain independent.
Parameter results: MAVMOOS_PARAM_RESULT, MAVMOOS_PARAM_VALUE,
  MAVMOOS_PARAM_SNAPSHOT (complete indexed JSON table), MAVMOOS_PARAM_TX_RESULT.
  Write echo and explicit readback are distinct; timeouts can leave write uncertainty.
  See docs/implementation/parameters/README.md for bounds and supported encoding.
Mission data provider (optional): MAVMOOS_MISSION_REQUEST=id=unique,action=download|upload|clear|cancel.
  Upload requires bounded item records; see docs/plugins/missions.md and tests/missions/mission_request.py.
  Downloads do not acquire navigation. Upload/clear require disarmed idle simulator state;
  ACK is followed by readback. Cancellation/timeouts cannot roll back partial mutation.
  Results: MAVMOOS_MISSION_RESULT/SNAPSHOT/PROGRESS/TX_RESULT. No execution permission is added.
  Inspection: MAVMOOS_MISSION_REQUEST=schema=1,id=unique,action=inspect,type=flight|fence|rally.
  The three typed snapshots and MAVMOOS_MISSION_INSPECTION_RESULT are separate from mutation/readback.
  Typed cancellation uses schema=1,id=<active id>,action=cancel,type=<active type>.
Optional mavlink_files provider, dedicated owned disarmed Copter SITL bench:
  MAVMOOS_FILE_REQUEST    schema=1,id=list1,action=list
                          schema=1,id=get1,action=get,catalog=list1,name=sample.bin
                          schema=1,id=stop1,action=cancel,operation_id=get1
  MAVMOOS_FILE_CATALOG, MAVMOOS_FILE_RESULT, MAVMOOS_FILE_TX_RESULT, MAVMOOS_FILE_RX_RESULT
  Bounded files in the simulator's mavmoos-ftp directory, no modification or control.

Optional flight_logs provider, owned disarmed Copter SITL:
  MAVMOOS_FLIGHT_LOG_REQUEST=schema=1,id=unique,action=list
  MAVMOOS_FLIGHT_LOG_REQUEST=schema=1,id=unique,action=download,catalog=<list id>,log_id=<id>
  MAVMOOS_FLIGHT_LOG_REQUEST=schema=1,id=unique,action=cancel,target=<active id>
  MAVMOOS_FLIGHT_LOG_CATALOG, MAVMOOS_FLIGHT_LOG_RESULT and MAVMOOS_FLIGHT_LOG_TX_RESULT.
  Complete files go to the configured owner-private directory. Local END has no FC acknowledgement.
Optional system_status provider: SYS_STATUS sensor masks and ambiguous battery fallback,
  identified BATTERY_STATUS records, primary GPS_RAW_INT and bounded STATUSTEXT assembly.
  Publications: MAVMOOS_SYS_STATUS, MAVMOOS_BATTERY_STATUS, MAVMOOS_GPS_STATUS,
  MAVMOOS_FC_TEXT events and MAVMOOS_SYSTEM_STATUS diagnostics (JSON schema1).
  Board and estimator reports use MAVMOOS_POWER_STATUS, MAVMOOS_MCU_STATUS,
  MAVMOOS_EKF_STATUS_REPORT and MAVMOOS_ESTIMATOR_STATUS.
  Observation age uses the conservative effective acquisition time; actual read time
  is retained separately. No status flag grants control permission or changes readiness.
Optional attitude provider: independent fused ATTITUDE and ATTITUDE_QUATERNION observations,
  raw FC NED/FRD conventions, body angular rates, explicit validity/age and unmapped remote time.
  Publications: MAVMOOS_ATTITUDE_EULER, MAVMOOS_ATTITUDE_QUATERNION,
  MAVMOOS_ATTITUDE_STATUS (JSON schema1). No NAV projection, control effect or readiness gate.
Optional rc_io provider: reported RC_CHANNELS input and per-port SERVO_OUTPUT_RAW
command observations, original sample ages and unknowns. No overrides, servo
commands, inferred pilot ownership or physical actuator feedback.
MAVMOOS_RC_INPUT, MAVMOOS_SERVO_OUTPUT, MAVMOOS_RC_IO_STATUS carry schema-1 JSON.
MAVMOOS_STREAM_STATUS and MAVMOOS_STREAM_REQUEST_RESULT report bounded local
stream-request attempts; local acceptance does not prove FC application.
IMU observer (optional imu plugin): RAW_IMU and SCALED_IMU/2/3 remain independent.
  Publications: MAVMOOS_IMU_SENSORS, MAVMOOS_IMU_STATUS and MAVMOOS_IMU_VIBRATION (schema1 JSON).
  Body FRD raw fields and source-convention SI values retain original ages/unknowns.
  Sensor reports do not replace NAV or establish health, orientation or control permission.
Shared firmware discovery observes AUTOPILOT_VERSION without changing authority.
firmware_requests=true permits at most3 read-only requests in the original10s
window only in explicit SITL. RX_ONLY/shadow transmit zero. Version and stream
setup share one protocol lane below control; ACK512 is not version evidence.
Optional distance_sensor provider: bounded per-ID DISTANCE_SENSOR records with
  orientation, raw/converted units, bounds, quality/extension unknowns and ages.
  Publications: MAVMOOS_DISTANCE_SENSORS and MAVMOOS_DISTANCE_STATUS (schema1 JSON).
  No range injection, terrain-altitude substitution, NAV ownership or control gate.
Optional passive observations from target 1/1, with separate wire families and original ages:
  event_timeline: MAVMOOS_EVENT_TIMELINE, MAVMOOS_EVENT_SEQUENCE_STATUS, MAVMOOS_EVENT_REPLAY_ERROR.
  water_depth: MAVMOOS_WATER_DEPTH, ArduPilot surface-boat bottom range and context.
  system_status: MAVMOOS_BATTERY_INFO and MAVMOOS_GENERATOR_STATUS remain separate from battery measurements.
  odometry: MAVMOOS_LOCAL_KINEMATICS, MAVMOOS_ODOMETRY and MAVMOOS_LOCAL_ORIGIN.
  air_data: MAVMOOS_VFR_HUD, MAVMOOS_WIND and MAVMOOS_WIND_COV.
    Pressure instances: MAVMOOS_SCALED_PRESSURE, MAVMOOS_SCALED_PRESSURE2 and MAVMOOS_SCALED_PRESSURE3.
    Fixed-wing ArduPilot: MAVMOOS_AIRSPEED and MAVMOOS_HIGH_LATENCY_AIR_DATA.
  esc_status: MAVMOOS_ESC_INFO, MAVMOOS_ESC_STATUS and MAVMOOS_ESC_TELEMETRY.
  traffic: MAVMOOS_TRAFFIC_CONTACTS with independently aged ADS-B contacts and separate reported track age.
Optional sensor and navigation observers:
  optical_flow: MAVMOOS_OPTICAL_FLOW and MAVMOOS_OPTICAL_FLOW_RAD.
  terrain: MAVMOOS_TERRAIN_REPORT and MAVMOOS_TERRAIN_REQUEST_OBSERVED.
  navigation_monitor: MAVMOOS_NAV_CONTROLLER_OUTPUT, MAVMOOS_POSITION_TARGET_LOCAL_NED,
    MAVMOOS_POSITION_TARGET_GLOBAL_INT. Observed FC reports do not grant control.
  link_status: MAVMOOS_RADIO_STATUS and MAVMOOS_CELLULAR_STATUS.
  clock_status: MAVMOOS_SYSTEM_TIME, MAVMOOS_TIMESYNC_REQUEST, MAVMOOS_TIMESYNC_REPLY.
  odometry also observes independent signed wheel distances in MAVMOOS_WHEEL_DISTANCE.
  system_status observes secondary GNSS in MAVMOOS_GPS2_STATUS and separate
    RTK baselines in MAVMOOS_GPS_RTK and MAVMOOS_GPS2_RTK.
  camera_status: MAVMOOS_CAMERA_INFORMATION, MAVMOOS_CAMERA_SETTINGS,
    MAVMOOS_CAMERA_CAPTURE_STATUS and MAVMOOS_CAMERA_STORAGE.
  gimbal_status: MAVMOOS_GIMBAL_DEVICE_INFORMATION and MAVMOOS_GIMBAL_DEVICE_ATTITUDE_STATUS.
  Payload observers require distinct configured components on the selected FC link.
  The opt-in disarmed Copter bench adds MAVMOOS_CAMERA_REQUEST/RESULT (single image)
  and MAVMOOS_GIMBAL_REQUEST/RESULT (point and release). See the module guides.
  distance_sensor also observes MAVMOOS_OBSTACLE_DISTANCE on the selected ArduPilot link.
  mode_catalog: MAVMOOS_MODE_CATALOG and MAVMOOS_CURRENT_MODE.
  component_metadata: MAVMOOS_COMPONENT_METADATA from one configured component and private mirror.
  remote_id: MAVMOOS_REMOTE_ID_IDENTITY, MAVMOOS_REMOTE_ID_LOCATION and MAVMOOS_REMOTE_ID_HEALTH
    from one configured component. Identifier and error text are redacted.
  These observers accept plugin.<module>.sample_timeout and plugin.<module>.publish_interval.
ExternalNav dedicated SITL bench: MAVMOOS_EXTERNAL_NAV_REQUEST, MAVMOOS_EXTERNAL_NAV_INPUT,
  MAVMOOS_EXTERNAL_NAV_STATUS. Only the named local producer can begin and feed the bounded run.
Providers: telemetry, parameters, missions, system_status, attitude, rc_io, distance_sensor,
  imu, odometry, air_data, esc_status, landing_target, traffic, flight_logs, mavlink_files, debug_values,
  event_timeline, water_depth, optical_flow, terrain,
  navigation_monitor, link_status, clock_status, camera_status, gimbal_status, component_metadata,
  mode_catalog, remote_id, external_navigation,
  arducopter_control, ardurover_control, arduplane_control, arduquadplane_control, ardusub_control,
  px4_control or px4_rover_control.
  PX4 aircraft native actions: arm, disarm, takeoff (height), hold, rtl, land.
  Standard VTOL also accepts transition_fw and transition_mc.
  One selected control provider owns firmware-specific commands and helm guidance.
  PX4 quadrotor and fixed-wing profiles expose stock-helm Offboard via MAVMOOS_NAV_CONTROL=acquire|release.
  The fixed-wing profile uses course, nominal speed for position lookahead, and height above home.
  PX4 Ackermann uses stock planar course and speed, verified Hold fallback and a raw GNSS stopped dwell.
  MAVMOOS_PX4_ROVER_STATUS reports verified configuration, stopped evidence and active/intended Hold.
  Its MAVMOOS_MISSION_CONTROL accepts start, pause, resume, abort and release for a verified ground plan.
  QuadPlane uses MAVMOOS_QUADPLANE_REQUEST and MAVMOOS_QUADPLANE_RESULT for owned VTOL missions.
  Its optional stock helm uses MAVMOOS_NAV_CONTROL=acquire|release from qualified forward flight.
  Catalog support/configuration is separate from readiness, authority and qualification.
  Duplicate capabilities/resources/request owners or control backends are rejected.
  Legacy commands/helm selections preserve their native-only/helm-only request surfaces;
  mixing canonical arducopter_control with those aliases is a configuration error.
  Copter's versioned operation interface supplements the unchanged legacy command grammar.
Modules load at startup with matching development ABI 10; no live reload or process isolation.
External local settings use plugin.<module_id>.<key>, checked as one complete startup batch.
Unknown owners/keys, duplicates, invalid values and reserved host-policy keys reject startup.
)";
  std::exit(0);
}
void showReleaseInfoAndExit()
{
  std::cout << "MAVMOOS 0.17.0 / iMAVLink modular SITL candidate\n";
  std::exit(0);
}
