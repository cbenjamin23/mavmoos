# Attitude observations

`attitude` is an optional read-only provider for the flight controller's fused Euler/quaternion attitude and body angular velocity. It is independent of raw accelerometer/gyro/magnetometer sensors, telemetry's NAV projections and the selected vehicle-control owner. It grants no permission, issues no effect requests and adds no readiness gate. Build it with the [matching host ABI](../development/external-plugins.md).

## Configuration and descriptor

```text
plugins                   = telemetry,attitude
plugin_directory          = /path/to/built/plugins
attitude_timeout          = 1
attitude_publish_interval = 0.1
```

These are ordinary fields inside `ProcessConfig = iMAVLink`. Receive-only remains the app default. This composition does not transmit MAVLink. The module advertises `ATTITUDE` (30) and `ATTITUDE_QUATERNION` (31), each at 5 Hz, to the host's observation-stream mechanism; it never opens an FC connection or sends an interval request itself. An absent stream stays absent in receive-only mode.

`attitude_timeout` accepts 0.25–10 seconds; `attitude_publish_interval` accepts 0.02–2 seconds. These are local policies rather than sensor/network guarantees. Descriptor profile `ardupilot_attitude_observations` declares observation-only capabilities `attitude.euler` and `attitude.quaternion`, exclusive resource `observation.attitude`, no request owners/subscriptions/control backend, and compatibility with an observation-only streamed-parameter bench. That descriptor is configured support, not throughput qualification.

## MOOS records

| Variable | Schema-1 JSON content |
|---|---|
| `MAVMOOS_ATTITUDE_EULER` | Latest independently acquired `ATTITUDE` record |
| `MAVMOOS_ATTITUDE_QUATERNION` | Latest independently acquired `ATTITUDE_QUATERNION` record |
| `MAVMOOS_ATTITUDE_STATUS` | Bounded two-family cache, binding/reset counters, `read_only:true`, `authoritative_for_control:false`, `atomic:false` |

Each family has `present`, `encoding_valid`, `fresh`, `effective_at`, `read_at`, `age_s`, `age_basis`, `age_is_upper_bound`, `observation_id`, `session_epoch`, and `data`. Absent samples are null, rather than a zero pose. Republication preserves original stamps and IDs. Freshness is independent from encoding validity and quality: a newly received invalid orientation is fresh diagnostic data, **not** a valid pose. Inspect `encoding_valid` and the individual orientation/rate fields.

`QUEUE_EMPTY_FLOOR` retains the conservative local socket-acquisition floor. Its age is an upper bound including potential local socket residence; actual read time remains separate. `READ_ACQUISITION` requires equal effective/read times and is not labelled a queue bound. Neither proves measurement generation time, upstream delay or remote clock synchronization. `source_age` stays `unknown`; `source_quality` stays `unassessed`.

Both families retain uint32 `time_boot_ms` under `remote_clock:"boot_ms_unmapped_uint32"`; it is neither a local freshness clock nor an inferred reboot epoch. Wrap or repeated boot timestamps do not renew an existing sample. A genuinely new local acquisition still has a new observation ID, without proving the remote measurement changed.

## Frames, representations and validity

Data declares `reference_frame:"LOCAL_NED"`, `body_frame:"BODY_FRD"` and `estimate:"fused_fc_attitude"`. The reported angular velocity is the source body's Forward/Right/Down vector, with x/y/z corresponding to roll/pitch/yaw body axes, measured in **rad/s**. These are not derivatives of the three Euler coordinate values. The pinned ArduPilot sender uses drift-corrected AHRS gyro estimates, not an exposed individual raw sensor.

Euler `raw_angles_rad` preserves roll/pitch/yaw in radians, using null only for nonfinite numbers. `reported_rpy` provides per-quantity `known`, `value`, `unit` and `reason`. Declared wire bounds are ±π, with a 10⁻⁶ rad float-representation allowance; finite out-of-range raw values remain visible with `orientation_valid:false`, `wire_declared_angle_range_conformant:false` and no derived valid pose/heading. Values are not silently wrapped into the accepted range.

Quaternion `quaternion_wxyz` preserves source Hamilton order `[w,x,y,z]`, where `[1,0,0,0]` is null rotation and q/−q encode the same orientation. `quaternion_norm`, `quaternion_components_finite` and `orientation_valid` are explicit. Unit-norm tolerance is a fixed local 0.001, rather than a sensor accuracy claim. A zero, nonfinite or nonunit quaternion cannot produce valid derived RPY. A near-unit accepted quaternion is normalized **only for the labelled derived calculation**; raw components remain unchanged.

`derived_rpy` uses the conventional Hamilton ZYX source NED/FRD interpretation, matching the pinned ArduPilot/MAVROS arithmetic. It is a canonical representation, not a promise of component equality for every equivalent Euler branch. At/near pitch ±π/2, roll/yaw are nonunique: `rpy_unique:false` suppresses derived roll, yaw and horizontal heading. The Euler message's original reported triple remains available even there, while its derived heading is suppressed. The dedicated `source_yaw_heading_deg` is source yaw wrapped to north-zero, clockwise degrees in `[0,360)`; a tiny negative angle that rounds to 360 during wrapping is canonicalized to 0. It is not ground course, independently verified true heading or an automatic `NAV_HEADING` replacement.

`representation_offset.raw_wxyz` preserves the optional display offset. All-zero means unsupported/omitted, distinct from identity `[1,0,0,0]`. A finite unit offset is labelled `unit_offset_unapplied`; invalid nonzero offsets are explicit. `display_adjustment_applied` is always false. No offset is silently composed into orientation/rates, no tailsitter display interpretation is claimed, and no FLU/ENU conversion or general body/world vector-transform API is provided. An invalid offset/rate makes the full record's `encoding_valid` false, even if the primary orientation separately remains valid.

No family suppresses or updates the other. The aggregate contains latest caches, not one atomic generation, a preferred representation or a fused synchronization result. Independent message timestamps are not solver/estimator generation IDs.

## Identity, lifecycle and limits

The provider waits for a post-construction/post-reset host-accepted ArduPilot heartbeat matching the shared vehicle type, then binds its exact system/component. The host retains peer/target/CRC/signature filtering. Heartbeat expiry (two seconds), reset and clock fault invalidate both cached families. A new heartbeat cannot revive old data. Clock faults latch until a valid explicit host reset; a conservative post-boundary floor remains required. Foreign IDs, impossible metadata, older effective times and backwards family read timestamps reject with counters. Distinct frames coalesced in one accepted datagram may share effective/read stamps: the last received representation replaces that family while retaining those original stamps and age. Metadata equality alone is not duplicate identity; host wire filtering owns duplicate handling. Only two sample slots exist; there is no growing sensor history or request queue.

The implementation/source/test record records exact qualification layers. This module does not establish covariance, estimator health, sensor quality, remote time synchronization, raw IMU calibration, camera mounting orientation or physical yaw tracking. Stock pHelmIvP remains unchanged.

MAVROS's [pinned IMU plugin](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/imu.cpp) informs orientation/rate grouping. It prefers quaternion once seen and publishes ROS frame/noise-model interfaces; those choices are not copied. Pinned [ArduPilot sending](https://github.com/ArduPilot/ardupilot/blob/04cf6486e8d42ce138de816e8466244346ed77cb/libraries/GCS_MAVLink/GCS_Common.cpp) determines the actual source and unused all-zero display offset. An AHRS header's passive frame wording differs from body-to-NED helper/MAVROS wording; retained wire values plus explicit source-frame derived math avoid promising a general vector transform from that comment alone.
