# Local kinematics, raw odometry and wheel encoders

The optional `odometry` module publishes four independent read-only observations.
Each is bound to a fresh heartbeat for the selected FC at system 1, component
1. The module accepts ArduPilot and PX4 heartbeats matching host observations.
Its passive message handlers use the FC's existing streams.

| Input | MOOS variable | Data fields |
| --- | --- | --- |
| `LOCAL_POSITION_NED` (32) | `MAVMOOS_LOCAL_KINEMATICS` | `position_m` and `velocity_m_s` in north, east, down order relative to the FC's local origin, plus the raw `time_boot_ms` |
| `ODOMETRY` (331) | `MAVMOOS_ODOMETRY` | `position_m` and `quaternion_wxyz` in the reported pose `frame_id`, `velocity_m_s` and `angular_velocity_rad_s` in the reported twist `child_frame_id`, with `time_usec_raw`, covariances, reset counter, estimator type and quality |
| `GPS_GLOBAL_ORIGIN` (49) | `MAVMOOS_LOCAL_ORIGIN` | WGS84 `latitude_deg` and `longitude_deg`, `altitude_msl_m` positive up, raw `latitude_deg_e7`, `longitude_deg_e7` and `altitude_msl_mm`, and optional `time_usec_raw` |
| `WHEEL_DISTANCE` (9000) | `MAVMOOS_WHEEL_DISTANCE` | Signed raw encoder meters in wire order, reported wheel count, unknown indices, and unmapped `time_usec_raw` |

Each variable carries JSON schema 1 with an independently updated `data`
object. An absent sample has `data: null`. A cached sample can remain present
after its age crosses the configured timeout. Use `fresh`, the original
`effective_at` and `read_at`, and the explicit `age_basis`. Republishing preserves
both timestamps. A queue-empty floor bounds local age and leaves upstream
source age unknown. Heartbeat lapse or reset clears all four families. Remote
timestamps remain unmapped to MOOS and host clocks. `time_usec_raw` is a decimal
string that preserves the full wire `uint64_t` in JSON consumers.

The local NED record does not infer a geographic origin or altitude datum. Raw
odometry retains the transmitted frame IDs and quaternion. The covariance
arrays keep the wire's row-major upper-triangle ordering. A NaN first element
maps that entire covariance to `null` as specified by MAVLink. Otherwise, NaNs
map to individual `null` entries and finite values are preserved. PX4 v1.17.0
populates known diagonals and leaves off-diagonal terms as NaN. Wire quality
zero or omission maps to `null`, while `-1` records a sender-reported estimator
failure. Freshness describes reception of the report, including a failure report. MAVLink 2
omitted extensions decode to zero, so a zero reset counter or estimator type
can reflect omission. Nonfinite kinematics, an all-zero quaternion, an infinity
in a known covariance, or reserved quality values reject the sample. These records
expose neither navigation authority nor a request interface.
[MAVLink LOCAL_POSITION_NED](https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED) ·
[MAVLink ODOMETRY](https://mavlink.io/en/messages/common.html#ODOMETRY) ·
[PX4 v1.17.0 ODOMETRY sender](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/mavlink/streams/ODOMETRY.hpp#L104-L125)

The local origin preserves the FC's reported WGS84 and MSL reference. Valid
coordinates include 0° latitude, 0° longitude and the range endpoints. The
receiver decodes the fixed MAVLink 1 base fields and MAVLink 2 trailing-zero
trimming. A zero or omitted timestamp extension maps to `null`. Copter 4.6.3
sends its estimator origin with the current FC microsecond clock, while the
PX4 v1.17.0 sender reports the reference timestamp. Use the separate receipt
age for freshness. Each origin report has its own lifetime and stays separate
from `HOME_POSITION` and local NED samples. An event-driven origin can remain
present and stale between sender reports. [MAVLink GPS_GLOBAL_ORIGIN](https://mavlink.io/en/messages/common.html#GPS_GLOBAL_ORIGIN) ·
[Copter 4.6.3 sender](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/GCS_MAVLink/GCS_Common.cpp#L3058-L3069) ·
[PX4 v1.17.0 sender](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/mavlink/streams/GPS_GLOBAL_ORIGIN.hpp#L71-L99)

In the `ProcessConfig` block for `iMAVLink`, add `odometry` to the startup list.

```text
target_system = 1
target_component = 1
plugins = telemetry,odometry
plugin.odometry.sample_timeout = 1
plugin.odometry.publish_interval = 0.1
```

Settings are in seconds. The sample timeout range is 0.25 to 10 and the publish
interval range is 0.02 to 2. The publish interval must be at most the sample
timeout. Omitted settings use the values shown above.

MAVROS's local-position and odometry providers inform the message boundaries.
This module retains FC frames and publishes each message family separately.
The pinned Copter sources consume message 331 as an FC input. The component
fixtures exercise the four observations with a PX4-shaped heartbeat and
track native stream evidence separately. The origin publication also follows
MAVROS's separation of the reported origin from its home subscription.
[MAVROS local position](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/local_position.cpp) ·
[MAVROS odometry](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros_extras/src/plugins/odom.cpp) ·
[MAVROS origin publication](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/global_position.cpp#L135-L153) ·
[Pinned Copter 4.6.3 receiver](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/GCS_MAVLink/GCS_Common.cpp#L4382)


## Wheel encoders

`odometry.wheel_distance` publishes up to 16 signed cumulative encoder distances. `wheel_count` selects the entries in `distance_m` in original wire order. Forward rotation increases the reported value and reverse rotation decreases it. A wheel's physical position and encoder zero are properties of the sender configuration. A zero count is an empty report. A valid zero distance is a numeric sample in its reported slot.

A NaN in a reported slot maps to `null` and its index is listed in `unknown_wheel_indices`. Infinity in a reported slot, count above 16, or impossible wire framing rejects the frame. The receiver validates the reported slots only. Schema 1 preserves source, original effective and read times, and the raw decimal timestamp string. The record stays independent of pose, velocity, origin, and NAV. Its read-only capability covers wheel observations and leaves slip analysis, odometry integration, and actuator control outside the interface. [Generated MAVLink wheel definition](https://github.com/mavlink/c_library_v2/blob/a2d0ecf011a4aaa7840480e43f4e38d8c9e79d29/common/mavlink_msg_wheel_distance.h).

The pinned MAVROS `wheel_odometry` module optionally publishes the reported distance array and also has a separate integration path. The pinned ArduRover sender uses the encoder instance order and boot-derived send time, and schedules the message when wheel encoders are present. Its own Rover SITL test uses `WENC_TYPE=10` for quadrature simulation. This supports a future owned ground-Rover receive bench with private encoder parameters and the existing firmware stream. Current MAVMOOS wheel qualification uses generated MAVLink frames. The pinned PX4 v1.17.0 stream registry has no wheel-distance entry, so the PX4-shaped generated fixture establishes decoding and source binding only. [Pinned MAVROS wheel handler](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros_extras/src/plugins/wheel_odometry.cpp#L647-L678), [Pinned Rover sender](https://github.com/ArduPilot/ardupilot/blob/04cf6486e8d42ce138de816e8466244346ed77cb/Rover/GCS_MAVLink_Rover.cpp#L406-L435), [Pinned Rover encoder test](https://github.com/ArduPilot/ardupilot/blob/04cf6486e8d42ce138de816e8466244346ed77cb/Tools/autotest/rover.py#L4899-L4928), [PX4 v1.17.0 stream registry](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/mavlink/mavlink_messages.cpp).
