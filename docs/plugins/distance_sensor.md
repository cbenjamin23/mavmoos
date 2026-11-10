# Distance-sensor observations

`distance_sensor` is an optional read-only provider for individual `DISTANCE_SENSOR` (132) reports and `OBSTACLE_DISTANCE` (330) sector arrays from the selected FC route. It shares the host connection/session and adds no control backend, NAV projection, permission, readiness gate or external range input. Sensor distance is not automatically height above home or vertical terrain clearance. Current host/module development ABI **10** must match.

## Configuration and capability

```text
plugins                          = telemetry,distance_sensor
plugin_directory                 = /path/to/built/plugins
distance_sensor_timeout          = 1
distance_sensor_publish_interval = 0.1
```

These are ordinary fields within `ProcessConfig = iMAVLink`. The timeout accepts0.25–10s and publication interval0.02–2s. The descriptor profile is `ardupilot_distance_observations`, capabilities `distance.observation` and `distance.obstacle_sectors` (observation only), exclusive resource `observation.distance_sensor`, and requested stream132 at100000µs/10Hz. It has no subscriptions/request owners/backend, and is compatible with a telemetry-only streamed-parameter bench. Descriptor demand is not actual delivery or permission to transmit. Receive-only/shadow retain their zero-MAVLink-output boundary; the plugin never sends a rate request itself.

## Publications

| Variable | Schema-1 content |
|---|---|
| `MAVMOOS_DISTANCE_SENSORS` | Bounded non-atomic aggregate with `session_epoch`, `present`, `atomic:false`, `cache_capacity:8`, and `sensors` sorted by `data.id` |
| `MAVMOOS_DISTANCE_STATUS` | Session/binding/reset/rejection diagnostics plus `read_only:true`, `authoritative_for_control:false`, and the same aggregate under `sensors` |
| `MAVMOOS_OBSTACLE_DISTANCE` | Latest independent 72-sector observation, original time and FC identity, or `present:false` and `data:null` |

An absent aggregate has `present:false` and an empty sensor array; absence is not a zero distance. Each cached sensor has independent original metadata and expiry. At most8 distinct IDs are retained. Extra IDs are rejected and counted until session reset; existing IDs continue updating. Stale values stay visible and do not disappear or refresh when another ID updates. `present` means retained observations exist, not that every record is fresh or usable.

Each sample contains `schema`, `session_epoch`, `family:"DISTANCE_SENSOR"`, `present:true`, `encoding_valid:true`, `fresh`, `effective_at`, `read_at`, `age_s`, `age_basis`, `age_is_upper_bound`, `observation_id` and `data`. `encoding_valid` means shape-accepted decoded base wire fields; it is not physical distance accuracy, valid geometry or valid extension data. Inspect the independent per-field unknowns/limits/quality. Republication keeps the same sample ID and acquisition/read stamps. Nonfinite extension values become null raw JSON values and explicit unknown quantities rather than invalid JSON numbers.

`QUEUE_EMPTY_FLOOR` retains the host's conservative lower acquisition bound and an upper bound on local acquisition age. `READ_ACQUISITION` requires equal effective/read times and is not labelled a queue bound. Neither certifies upstream measurement age; `source_age:"unknown"` and `source_quality:"unassessed"` remain explicit.

## Distance, identity and orientation

Data retains `id`, uint32 `time_boot_ms`, `wire_version`, `payload_length`, `extension_bytes_observed`, and raw `min_distance_cm`, `max_distance_cm`, `current_distance_cm`. The corresponding `_m` fields are `{known,value,unit:"m",reason}` quantities scaled by0.01. Zero and65535 are preserved because this wire definition gives neither a universal distance sentinel. Even outside reported limits the numeric wire observation remains visible; `bounds_consistent` and `current_within_reported_limits` separately describe the reported relationship. These are not an aggregate measurement-validity verdict.

`type_raw`/`type_known`/`type_name` retain the technology enum. Known enum `UNKNOWN`4 is distinct from an unrecognized raw enum. `orientation_raw`/`orientation_known`/`orientation_name` retain every pinned standard rotation0–40, custom100 or an unrecognized raw value. The body frame is labelled `BODY_FRD`; no ROS frame conversion, mounting-position transform, beam-to-vertical projection, terrain estimate or automatic altitude substitution occurs. `distance_interpretation:"sensor_report_no_vertical_or_terrain_projection"` and `physical_accuracy:"unassessed"` remain explicit.

The remote timestamp is labelled `remote_clock:"boot_ms_unmapped_uint32"` and `timestamp_semantics:"ardupilot_message_send_time_not_certified_measurement_time"`. The pinned ArduPilot producer supplies `AP_HAL::millis()` with a measurement-time TODO. Repeated/wrapped remote integers neither renew local age nor prove a new boot/session.

## Covariance and extensions

`covariance` retains `raw_cm2` and a quantity in `m^2` (nonzero known values ×0.0001). Generic wire255 means unknown. This ArduPilot profile also treats0 as unknown/invalid because the inspected producer explicitly sends0 for that meaning. Zero is not reported as perfect variance. The profile difference from generic wire semantics is intentional and source-backed.

`signal_quality` retains the raw extension byte, extension availability and a percent quantity. Values2–100 are reported percentages;0 means unknown/unset/omitted,1 invalid signal and101–255 outside the percentage range. A reported quality percentage is not calibrated physical accuracy or control permission.

`horizontal_fov` and `vertical_fov` retain finite `raw_rad` and a radian quantity. Zero is unknown/unset/omitted; nonfinite, negative or greater-than-2π values are explicitly unknown. Positive finite values up to2π remain reported FOV, without a mounting or geometry calibration claim.

`quaternion.raw_wxyz` preserves source Hamilton order without normalization. Its finite/norm/unit-norm metadata uses a fixed0.001 norm tolerance. A finite unit quaternion is usable here only when orientation is custom100; it is not applied to standard orientation enums. Absent/all-zero/nonunit/nonfinite data cannot supply a custom orientation. No derived RPY, vector rotation or attitude preference is introduced.

MAVLink1 omits all extensions, even if a test's backing memory has dirty tail bytes: raw extension fields are null and quantities unknown. MAVLink2 decode zero-fills trimmed fields; zero FOV/quality/quaternion does not prove support or an actually measured zero. `extension_bytes_observed` describes payload length, not complete field support. Partial extension bytes are decoded by the generated codec. Base fields can also be trailing-zero trimmed in MAVLink2, so payload length alone is not a field-presence inventory.

## Session, time and scope

The provider waits for a post-construction/post-reset accepted ArduPilot heartbeat matching shared vehicle type, then binds exact system/component IDs. The host owns peer/target/CRC/signature/duplicate filtering. The component separately rejects impossible metadata, foreign identity, malformed wire shape and backwards per-sensor effective/read stamps. Distinct coalesced frames may share stamps and still replace the cached record; upstream exact-frame duplicate suppression remains separate.

Heartbeat loss or shared heartbeat invalidity clears all sensors and requires a new binding. Explicit reset clears every ID and raises the acquisition floor. An invalid/nonfinite/negative or behind-high-water Services clock latches the component fault until a valid explicit reset at least as high as the retained reliable time, followed by new heartbeat/data. Catching up by itself does not recover the component. Host raw-clock faults may instead arrive as a reset reason plus the retained good Services time; the reset reason and the component's numeric-clock state describe different facts.

Pinned [MAVROS distance_sensor](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros_extras/src/plugins/distance_sensor.cpp) supplies the capability-family precedent. Its configured ROS `Range` mappings, TF publication and external-range subscriber/send path are not copied. The initial profile ignores legacy `RANGEFINDER`173 rather than merging an unlabelled value into an ID-bearing sensor record. The source/qualification archive records generated-header and ArduPilot producer semantics separately from actual test outcomes. Physical sensors, range injection, automatic avoidance/landing changes and hardware control remain outside scope.

## Obstacle sector arrays

The provider preserves the 72 entries from MAVLink 2 in wire index order. `data.frame_raw` retains the original frame ID. `frame_name` interprets `GLOBAL` as north aligned and `BODY_FRD` as vehicle-front aligned. Other IDs have the label `UNINTERPRETED`. `frame_explicit` records whether the extension byte was present. A trimmed frame field decodes to the protocol's `GLOBAL` default.

`increment_f_deg_raw` takes precedence when nonzero. Otherwise `increment_deg_raw` provides the integer clockwise increment. `increment_deg`, `increment_source`, and `angle_offset_deg` retain the effective signed angle information. Sector zero uses the original offset and the remaining indices advance by the increment. The provider preserves the original geometry without a coordinate transform or resampling. The uint64 `time_usec_raw` is a decimal string with an unmapped boot-or-Unix clock.

Each sector carries `distance_cm_raw`, `classification`, and a nullable `distance_m`.

| Classification | Interpretation |
|---|---|
| `contact` | Wire zero indicates contact. The metric distance is zero, even when the reported minimum is positive. |
| `in_range` | The raw range is within the inclusive reported limits and has a metric distance. |
| `no_hit` | The raw value is maximum plus one and distinct from the unknown code. The metric distance is null. |
| `unknown` | Wire value 65535 marks an unknown or unused sector. |
| `out_of_range` | Other values outside the reported limits remain visible as raw centimeters. The metric distance is null. |

`no_hit_sentinel_cm` is null when maximum plus one collides with the unknown code or exceeds uint16. `frame_explicit` and `payload_length` describe actual wire extent. An accepted frame replaces only the independent sector cache. Its original timestamps and observation ID remain fixed on repeat publication, and the same timeout marks the cached frame stale. Session or clock reset clears it alongside the scalar cache. Source mismatch, older metadata, impossible framing, inverted range limits, zero or greater-than-360-degree increments, and nonfinite geometry are rejected before replacement.

Pinned MAVROS `obstacle_distance` takes a ROS LaserScan, converts its angles and ranges to this same 72-bin message, and sends it to the FC. The pinned ArduPilot 4.6.3 proximity receiver applies its own avoidance policy, including discarding contact and out-of-range bins internally. MAVMOOS reports wire contact as an observation with `authoritative_for_control=false`. The normal pinned Copter GCS path receives these packets. The separate Morse simulator's scanner has a 2D sender on component 13. The generated-frame app witness exercises the selected FC route, and the 3D MAVLink family is outside this provider's qualified interface. [Pinned MAVROS obstacle plugin](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros_extras/src/plugins/obstacle_distance.cpp), [MAVLink sector definition](https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE), [Pinned ArduPilot proximity receiver](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/AP_Proximity/AP_Proximity_MAV.cpp#L119-L215), [Pinned Morse scanner sender](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/SITL/SIM_Morse.cpp#L636-L668).
