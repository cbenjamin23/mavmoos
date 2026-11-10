# Current providers and their references

November 5, 2026. This is an orientation map of MAVMOOS source
`f6e8c61249d70f457cd903d7ca403ec94948644e` (F6), with **ten available standard
providers and development ABI 8**. The default loads telemetry only; the two
vehicle-control providers are alternatives. The external heartbeat example is
separate from the standard bundle. Availability, configured capability, current
permission and demonstrated behavior are different things.

The closest references below are specific MAVROS source files at
`2d3f4db1101c7950c8817a9d788f0d11106fb261`, already retained in the research
archive. They identify functional and organizational precedent, not a claim of
copied implementations, ROS API parity or equivalent qualification. MAVMOOS
uses generated MAVLink code, but does not link MAVROS, ROS or libmavconn. The
source map records inspected reference bytes, current
source hashes and the earlier pArduBridge receipts. This note adds no code,
settings, interface or qualification.

## The ten providers

| MAVMOOS provider | Specific MAVROS reference | Implemented MAVMOOS boundary and meaningful differences |
| --- | --- | --- |
| `telemetry` | [global_position][global] and [local_position][local], with [frame conversions][frames] | Projects accepted global position through MOOSGeodesy into east-X/north-Y `NAV_*`; distinguishes yaw heading from velocity-derived travel course, and MSL height from height above bound HOME. Retains original local acquisition/read metadata. It does not reproduce the ROS pose/TF/covariance/geoid interfaces or accept external estimator injection. See NAV contract. |
| `parameters` | [param][param]: named values, list state and set/pull services | Typed named reads, paced indexed inventory/snapshot, cancellation and original expiry. Writes are restricted to the declared disarmed simulator-wind workflow with echo/readback. No general FC configuration service, ROS parameter mirroring or PARAM_EXT parity. A received `PARAM_VALUE` has no MAVMOOS operation ID; local correlation cannot prove which request caused an untagged delayed reply. See parameter guide. |
| `missions` | [waypoint][waypoint] over [mission_protocol_base][mission] | Owns storage transfer: download, upload, clear, cancellation and mandatory post-mutation download/readback. Supplies typed host-held verified content; upload grants no execution permission. The initial storage profile does not cover fence/rally, partial push or the full mission dialect. Copter AUTO execution belongs to vehicle control, below. See [mission guide](../../plugins/missions.md). |
| `arducopter_control` | [command][command], [setpoint_velocity][velocity], [setpoint_position][position] and [setpoint mixins][setpoint]; mission references for AUTO | One public vehicle capability groups native actions, Guided operations, stock-helm guidance and optional simple AUTO execution. Includes lifecycle, goto, hold/loiter and return strategies with explicit ownership/outcomes. The reusable helm adapter is an internal component. AUTO requires explicit opt-in, verified stored content and an already armed/airborne Guided vehicle; it never implicitly arms, takes off or restores helm. This is not a generic MAVROS command passthrough, PX4 Offboard profile or all-Copter-command implementation. See [control](../../plugins/arducopter_control.md) and [AUTO](../../plugins/copter_auto.md). |
| `ardurover_control` | Same [command][command] and [velocity-setpoint][velocity] families | Reuses common orchestration/helm services with a genuine planar Rover backend: course/speed, native Guided/arm/disarm/Hold/RTL and actual zero-speed output. It does not inherit Copter height/airborne requirements. The profile accepts ground Rover, not boat/BlueBoat, and rejects independent sideways/facing strategies. Rover source/build and qualification remain separate from the selected Copter profile. See [Rover guide](../../plugins/ardurover_control.md). |
| `system_status` | [sys_status][status], plus raw GPS handling in [global_position][global] | Read-only system masks, independently identified batteries, explicitly ambiguous SYS_STATUS battery fallback, primary raw GPS and bounded status-text assembly. Original age, unknowns and reset remain visible. These reports are not an aggregate health/readiness verdict or control authority. No ROS diagnostic-updater, GPS2 or fused NAV replacement. See [status guide](../../plugins/system_status.md). |
| `attitude` | [imu][imu]: `handle_attitude` and `handle_attitude_quaternion` | Independently retains fused Euler/quaternion orientation and body rates in source NED/FRD conventions, including invalid/singular/offset cases. It does not choose a preferred family, combine samples into one atomic generation, convert to ROS ENU/FLU, synthesize covariance or substitute NAV. MAVROS groups these with other IMU publications; MAVMOOS separates fused attitude from raw sensor reports. See [attitude guide](../../plugins/attitude.md). |
| `rc_io` | [rc_io][rc]: RC input and servo-output handlers | Read-only `RC_CHANNELS` input and independently aged `SERVO_OUTPUT_RAW` ports, preserving raw slots and extension ambiguity. No universal channel-role or flattened actuator mapping; an FC-reported input need not originate from a physical transmitter. Unlike the reference's override subscriber and RAW fallback, this profile sends no RC/servo commands and ignores legacy `RC_CHANNELS_RAW`. See [RC guide](../../plugins/rc_io.md). |
| `distance_sensor` | Extras [distance_sensor][distance] | Bounded, identified `DISTANCE_SENSOR` cache with cm-to-m conversion, orientation, quality/extension unknowns and original times. No automatic NAV altitude, terrain clearance, avoidance/landing policy, outgoing range injection, ROS Range/TF mapping or legacy RANGEFINDER handling. See [range guide](../../plugins/distance_sensor.md). |
| `imu` | [imu][imu]: RAW/SCALED/HIGHRES sensor organization | Independently retains RAW_IMU and SCALED_IMU/2/3 in an eight-key cache, with explicit raw/SI units, BODY_FRD axes, original times and temperature/identity ambiguity. Families can overlap the same hardware; logical instances do not prove physical sensor count. No orientation fusion, covariance, health or NAV ownership. HIGHRES/pressure are deferred; the [IMU guide](../../plugins/imu.md) records the pinned sender-unit concern. |

The public boundary follows the capability someone looks for. For example,
native FC Loiter, a bridge-supplied Guided hold and configured helm
StationKeep are alternatives within Copter control; they do not become three
unrelated public plugins. Each still exposes who is making motion decisions,
which FC mode is involved and how ownership changes. Parameters, mission
storage and sensors remain independent because those are useful feature
families in their own right.

## Shared host services

These services are not additional providers, and loading several features does
not open several FC connections.

| Shared responsibility | Specific reference | MAVMOOS choice |
| --- | --- | --- |
| Transport and vehicle context | libmavconn [UDP][udp]/[interface][connection], MAVROS [router][router] and [UAS][uas] | One fixed-peer native UDP backend, generated MAVLink codec and shared session/vehicle context. Waiting navigation can be replaced; expired/cancelled work is rejected before output. This is ordinary UDP carrying standard MAVLink, not a modified wire format. It is not libmavconn's queue implementation or a general serial/radio/multivehicle router. Already transmitted packets cannot be recalled. |
| Time, frames and observation age | [UAS time support][uas-time], [sys_time][time] and [frame conversions][frames] | Original local acquisition floor and actual read time remain distinct. Conservative socket-queue age does not establish upstream measurement freshness; remote clocks remain unmapped. No synchronized source-clock guarantee or automatic ROS frame convention is adopted. |
| Firmware discovery | [sys_status][status]: `handle_autopilot_version` and `autopilot_version_cb` | Shared passive identity plus bounded read-only REQUEST_MESSAGE/AUTOPILOT_VERSION discovery where explicit SITL output is permitted. Reported versions, source prefixes and capability bits are observations, never permission or qualification. Immutable source manifests, platform build attestations and scenario evidence are separate. See [firmware guide](../../plugins/firmware_discovery.md). |
| Stream intent and protocol budget | [sys_status][status]: stream-rate and message-interval interfaces | Providers declare requested observations; the host unions demand and sends through one bounded fair protocol budget after control. Request/local-send success does not prove remote acceptance or delivered cadence. Receive-only/shadow still send zero. There is no independent request loop/connection per plugin. |
| Transactions and authority | [command][command] ACK transactions and [mission base][mission] state machines | The host separates queue admission, local send, protocol response and observed outcome. It owns original expiry, cancellation, session/endpoint checks and persistent AUTO ownership. The reference does not supply MOOS helm arbitration, remote rollback or proof that a sent action was undone. |
| Startup loading and configuration | [UAS][uas], [plugin base][plugin] and [plugin implementation][plugin-impl] | Startup-selected shared libraries, C factories and matching development ABI 8, with declared capabilities/resources/request owners and one control backend. Modules stay resident; no live replacement or stable binary SDK. MAVROS uses pluginlib and ROS nodes/interfaces. The proposed [generic configuration route](../2026-11-05-plugin-configuration/README.md) is **unimplemented**; the app still forwards a built-in key allowlist. |

## What comes from pArduBridge and stock MOOS-IvP

The [pArduBridge architecture][pardu-architecture] and
[operator/design documentation][pardu-readme] at
`e34a62ef49c2be42215a69c351850cbd85881486` grounded the first UAV workflow:
the MOOS app boundary versus vehicle model, Guided gating, native lifecycle
orchestration, takeover/watchdog handling and configurable travel-facing
behavior. [ArduBridge.cpp][pardu-app], [UAV_Model.cpp][pardu-model],
[SetpointManager][pardu-setpoints] and [ModeConfirmationTracker][pardu-mode]
are the concrete references. The earlier three-way comparison
also retains Jeremy's compact recovered bridge as a reference; it does not
establish BlueBoat support here.

MAVMOOS keeps these workflow precedents while using its own host services and
native UDP backend rather than MAVSDK. It makes original local expiry,
session boundaries and local-send versus observed completion explicit across
features. This is design lineage, not an assertion that pArduBridge lacked
every such safeguard or that equivalent behavior has been demonstrated.

Stock `pHelmIvP` remains unchanged. The adapter consumes ordinary
`DESIRED_HEADING`, `DESIRED_SPEED`, `DESIRED_ALTITUDE`, state and all-stop
publications. For the initial Copter profile those mean travel course,
groundspeed and height above HOME; the bridge translates to standard Guided
targets and the FC retains its inner control loops. The adapter checks source,
original MOOS age, bounds and post-boundary renewal. Separate scalar messages
remain `ARRIVAL_GROUPED`; neither their cache nor a forwarded publication proves
an atomic, newly generated helm decision. This MOOS-specific responsibility
belongs beside reusable control services, not in the transport or every sensor
plugin.

## Reading the evidence

Use the [capability map](../../capability-map.md) for delivered versus candidate
scope, the [developer guide](../../development/external-plugins.md) for current
module construction and the expansion ledger
for exact source/profile/platform/scenario cells. Dated ABI 6/7 and earlier
provider counts describe their original increments; they are not current build
instructions. This crosswalk does not widen any historical cell to F6, a new
platform, another firmware release or real hardware.

[global]: https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/global_position.cpp
[local]: https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/local_position.cpp
[frames]: https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/lib/ftf_frame_conversions.cpp
[param]: https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/param.cpp
[waypoint]: https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/waypoint.cpp
[mission]: https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/mission_protocol_base.cpp
[command]: https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/command.cpp
[velocity]: https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/setpoint_velocity.cpp
[position]: https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/setpoint_position.cpp
[setpoint]: https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/include/mavros/setpoint_mixin.hpp
[status]: https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/sys_status.cpp
[imu]: https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/imu.cpp
[rc]: https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/rc_io.cpp
[distance]: https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros_extras/src/plugins/distance_sensor.cpp
[udp]: https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/libmavconn/src/udp.cpp
[connection]: https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/libmavconn/src/interface.cpp
[router]: https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/lib/mavros_router.cpp
[uas]: https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/lib/mavros_uas.cpp
[uas-time]: https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/lib/uas_timesync.cpp
[time]: https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/sys_time.cpp
[plugin]: https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/include/mavros/plugin.hpp
[plugin-impl]: https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/lib/plugin.cpp
[pardu-architecture]: https://github.com/cbenjamin23/moos-ivp-uav-base/blob/e34a62ef49c2be42215a69c351850cbd85881486/wiki/pArduBridge/ARCHITECTURE.md
[pardu-readme]: https://github.com/cbenjamin23/moos-ivp-uav-base/blob/e34a62ef49c2be42215a69c351850cbd85881486/wiki/pArduBridge/README.md
[pardu-app]: https://github.com/cbenjamin23/moos-ivp-uav-base/blob/e34a62ef49c2be42215a69c351850cbd85881486/src/pArduBridge/ArduBridge.cpp
[pardu-model]: https://github.com/cbenjamin23/moos-ivp-uav-base/blob/e34a62ef49c2be42215a69c351850cbd85881486/src/pArduBridge/UAV_Model.cpp
[pardu-setpoints]: https://github.com/cbenjamin23/moos-ivp-uav-base/blob/e34a62ef49c2be42215a69c351850cbd85881486/src/pArduBridge/SetpointManager.h
[pardu-mode]: https://github.com/cbenjamin23/moos-ivp-uav-base/blob/e34a62ef49c2be42215a69c351850cbd85881486/src/pArduBridge/ModeConfirmationTracker.h
