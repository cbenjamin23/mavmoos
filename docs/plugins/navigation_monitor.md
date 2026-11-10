# FC navigation monitor

Load `navigation_monitor` at startup to inspect what the selected FC reports about its controller and target. It publishes independently aged schema-1 MOOS JSON strings:

| MOOS variable | MAVLink | Interpretation |
| --- | --- | --- |
| `MAVMOOS_NAV_CONTROLLER_OUTPUT` | `NAV_CONTROLLER_OUTPUT` 62 | Reported roll, pitch, bearings, distance and controller errors. Fields may be inapplicable or placeholders for a particular producer or mode. |
| `MAVMOOS_POSITION_TARGET_LOCAL_NED` | `POSITION_TARGET_LOCAL_NED` 85 | Reported local target with raw frame, boot milliseconds, mask, position, velocity, acceleration/force and yaw fields. |
| `MAVMOOS_POSITION_TARGET_GLOBAL_INT` | `POSITION_TARGET_GLOBAL_INT` 87 | Reported global target with raw frame/mask, exact latitude/longitude integers, known WGS84 degrees and the reported altitude reference. |

Ignored mask fields publish null even when their wire values are nonzero or nonfinite. Unknown frame numbers and mask bits remain visible. The module performs no ENU conversion or transform between body, local and global coordinates. For an unknown global frame it retains the raw coordinate integers and omits interpreted degrees. Active nonfinite fields and invalid recognized geographic coordinates are refused. Accelerations are in m/s² and forces in N according to the force bit. Boot milliseconds are raw producer data. They do not renew the original bridge acquisition time.

These are observations of the FC's reported target and control output. They do not identify the MOOS request that produced a target, confirm physical position, replace NAV, give permission, or send setpoints. The receiver only admits the bound system/component 1/1 in the selected app route. The cache expires by original acquisition age and clears on session loss. Configure `plugin.navigation_monitor.sample_timeout` (default 2 s) and `plugin.navigation_monitor.publish_interval` (default 0.1 s). Read-only is the app default; the module never requests additional streams by itself.

The pinned [MAVROS navigation controller reader](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/nav_controller_output.cpp) copies the wire controller fields. Its [setpoint_raw](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/setpoint_raw.cpp) and [guided_target](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros_extras/src/plugins/guided_target.cpp) expose the received target. MAVMOOS retains the wire axes and the effective/read timestamps, and keeps observations separate from command APIs. [Copter 4.6.3 sender](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/ArduCopter/GCS_Mavlink.cpp#L109-L222) reports global targets in MSL, sets high reserved mask bits, and reports zero for unused airspeed error in message 62. Generated and native evidence are tracked separately.
