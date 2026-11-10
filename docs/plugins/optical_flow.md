# Optical flow

Enable `optical_flow` in the explicit `plugins` list. This passive provider observes `OPTICAL_FLOW` (100) and `OPTICAL_FLOW_RAD` (106) independently on the host-selected flight controller link. It binds after a matching system 1, component 1 ArduPilot or PX4 heartbeat. The app's selected peer and heartbeat identity still apply. Stock helm consumes its ordinary navigation inputs. Optical flow contributes observation records only.

~~~moos
plugins = telemetry,optical_flow
plugin.optical_flow.sample_timeout = 1.0
plugin.optical_flow.publish_interval = 0.1
~~~

The default sample timeout is 1.0 s, configurable from 0.25 to 10 s. The default publication interval is 0.1 s, configurable from 0.02 to 2 s and bounded by the sample timeout. The sender must already provide its telemetry stream.

## MOOS records

| Variable | MAVLink family | Values |
| --- | --- | --- |
| `MAVMOOS_OPTICAL_FLOW` | `OPTICAL_FLOW` 100 | Original integer flow and compensated X/Y wire values, optional angular-rate extensions, ground distance and quality |
| `MAVMOOS_OPTICAL_FLOW_RAD` | `OPTICAL_FLOW_RAD` 106 | Integrated optical and gyro angles in radians, integration interval and distance sample offset in microseconds, distance in meters, temperature in centidegrees Celsius and quality |

Each schema-1 envelope identifies its family and reports `present`, `sensor_limit`, `invalid_samples`, `capacity_drops` and a `samples` array. Each family retains up to eight different uint8 sensor IDs in the current bound session. Existing sensors can update at capacity. Additional IDs increment `capacity_drops` and leave the admitted samples intact. Reset clears the sensor sets. Counters are cumulative for the provider instance. An empty array means the family has no retained samples.

Each sample has `sensor_id`, `wire_version`, `payload_length`, `metadata` and `data`. The same ID in the two families identifies two distinct records. `atomic=false` describes the independent sample times in the envelope. `axis_basis=sensor_wire` preserves the reported axis signs. A mounting transform or an estimate of vehicle velocity requires separate configuration and evidence.

Per-sensor metadata retains the original effective and read times. `age_s` grows across repeated publications of that observation ID. Freshness uses the host observation age. The raw source time stays a decimal string in `time_usec_raw` and keeps its full 64-bit precision. Source clock and age remain unqualified. Out-of-order packets for a sensor cannot replace its latest observation. Heartbeat loss and disconnect clear both families. New data requires a newly bound heartbeat.

The payload's `quality_raw` is the sender's byte, with 0 representing poor or invalid flow and 255 representing maximum confidence. It is independent of arrival freshness and estimator health. Negative distance sentinels remain visible in the raw fields and map to null for the interpreted distance. Zero is ambiguous in legacy `ground_distance_m` and is a defined value in `OPTICAL_FLOW_RAD.distance_m`. Integrated gyro NaNs map to null because PX4 can forward unavailable gyro values. Infinite values and nonfinite required flow or distance values invalidate the sample.

Valid MAVLink 2 trailing-zero truncation applies to base fields and extensions. A legacy angular-rate extension with no bytes present is null because a trailing zero and an omitted extension can have the same encoding. Partial extension bytes use the generated zero-padding rules. The integrated family keeps zero-valued raw temperature and distance age fields because the message has no availability flag. PX4's stream can leave these fields at zero. An integration time of zero is retained and produces no averaged flow. [MAVLink definitions](https://mavlink.io/en/messages/common.html#OPTICAL_FLOW) · [PX4 integrated-flow sender](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/streams/OPTICAL_FLOW_RAD.hpp#L483-L523).

## Source differences and validation

MAVROS uses separate extras plugins for 100 and 106. They transform the reported angles and rates into ROS axes, assign synchronized ROS headers, and publish derived range and temperature topics. They also expose separate sends to the FC. MAVMOOS groups the two receive paths under one provider with separate wire records and independent freshness. Its descriptor has observation capabilities and zero request owners or stream setup entries. [Pinned MAVROS optical_flow, blob `83c9145`](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros_extras/src/plugins/optical_flow.cpp) · [Pinned MAVROS px4flow, blob `8aedd52`](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros_extras/src/plugins/px4flow.cpp).

Both inspected ArduPilot pins send message 100 with boot milliseconds in the field named `time_usec`. The sender casts sensor angular rates into `flow_x` and `flow_y`, fills the nominal compensated m/s fields with flow minus body angular rate, supplies AHRS height above ground or zero, and provides raw angular rates in the MAVLink 2 extensions. MAVMOOS therefore keeps the integer and compensated fields as raw values and avoids assigning a velocity or a source clock to them. Message 106 remains separate. [ArduPilot development pin](https://github.com/ArduPilot/ardupilot/blob/04cf6486e8d42ce138de816e8466244346ed77cb/libraries/GCS_MAVLink/GCS_Common.cpp#L2935-L2969) · [Copter 4.6.3 pin](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/GCS_MAVLink/GCS_Common.cpp#L2866-L2900) · [Generated codec pin](https://github.com/mavlink/c_library_v2/tree/a2d0ecf011a4aaa7840480e43f4e38d8c9e79d29/common).

The focused CTest passed all eight checks in Debug and ASan/UBSan on macOS ARM64 using the pinned generated headers. It loads the real provider and exercises both identities, two message families, sensor ordering and capacity, clock and connection loss, MAVLink 1 and trimmed MAVLink 2, wire validation and observer-only effects. The shared app fixture uses generated frames to check the MOOS records across prebind, freshness, loss and recovery.

A disarmed Copter 4.6.3 fixture can enable the simulator's own optical flow backend with `SIM_FLOW_ENABLE=1` and `FLOW_TYPE=10` and configure message 100 in the startup telemetry intervals. The backend's simulated quality is 51. Compare the app with the passive FC bytes and record zero app egress. Message 106 is a generated-frame witness for this profile. [Pinned SITL backend](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/AP_OpticalFlow/AP_OpticalFlow_SITL.cpp#L27-L119).
