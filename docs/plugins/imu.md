# IMU sensor observations

The optional `imu` startup module publishes read-only ArduPilot inertial and
magnetic sensor reports. It shares the host connection and bounded stream-request
service. It exposes no command, parameter-write, navigation or control interface.
The existing `attitude` module remains the source of fused FC orientation/rates;
IMU observations do not replace NAV, estimate attitude or remove gravity.

| Wire family | Message ID | Logical instance | Numeric conversions |
| --- | --- | --- | --- |
| RAW_IMU | 27 | Reported/default extension ID; MAVLink 1 defaults to 0 | ArduPilot sender convention: mg, mrad/s, mG |
| SCALED_IMU | 26 | 0 | MAVLink mg, mrad/s, mG |
| SCALED_IMU2 | 116 | 1 | Same |
| SCALED_IMU3 | 129 | 2 | Same |

Conversions are mg ×9.80665/1000 to m/s², mrad/s ×0.001 to rad/s, and mG ×1e-7
to tesla. Axes remain in ArduPilot's BODY_FRD convention. Raw integer triples and
original units/basis remain alongside conversions. These are FC reports;
ArduPilot may already have applied calibration, sensor rotation or filtering.
They are not raw ADC samples or certified physical measurements.

Families remain independent, even when they describe the same hardware. A
RAW_IMU ID 0 may be a default/trimmed extension, or a MAVLink 1 message without
the extension. A logical instance or a different message family does not prove
a distinct physical sensor. Missing components can be zero in ArduPilot's
scaled sender; zero values do not establish sensor presence or health. The
records retain `sensor_presence=unknown`, `sensor_health=unassessed`, unverified
physical identity and no covariance.

`MAVMOOS_IMU_SENSORS` is a JSON object containing independently aged `sensors`.
Each record keeps a session epoch, original `effective_at` and `read_at`, age
basis, observation ID, local age and a `fresh` flag. Republishing never renews
these values. `QUEUE_EMPTY_FLOOR` is marked as an upper-bound local age;
`source_age` remains unknown. Remote microsecond/millisecond boot timestamps are
retained as unmapped integers. A regressing remote timestamp is reported without
being converted to a local timestamp, declaring an FC restart or refreshing
another family.

Temperature is an optional MAVLink 2 extension, reported in centidegrees Celsius.
Wire 0 is unknown/omitted; MAVLink 1 has no temperature. The protocol recommends
wire 1 for a sensor at exactly 0°C, but 1 may also report 0.01°C, so its actual
numeric value 0.01°C is retained with this ambiguity explicit. No other integer
is treated as an invented unknown sentinel.

`MAVMOOS_IMU_STATUS` reports source/session binding and rejected metadata,
source, wire-shape, ordering and capacity counts. A fresh accepted ArduPilot
heartbeat from the current host target 1/1 is required before accepting samples.
Original acquisition/read times must be finite, ordered and after the current
session floor. Host or observed heartbeat expiry clears cached observations.
Clock faults latch until an explicit reset at or beyond the last reliable owner
time; reset requires a new post-floor heartbeat and new sample.

The cache holds eight `(message family, logical instance)` keys. Additional keys
are rejected without eviction; existing keys can still update. The descriptor
requests the four supported messages at 5 Hz through the existing shared budget;
successful local interval requests do not prove FC rate, sensor availability or
measurement freshness. Passive receive-only use needs no outbound request.

## Vibration and clipping

`MAVMOOS_IMU_VIBRATION` publishes a separate JSON schema 1 record from `VIBRATION`
(241). The provider keeps one report with its own observation ID, original local
times and freshness. It shares the IMU source binding, session and timing settings.
A missing report has `present=false` and `data=null`. Republishing a stale report
retains its original metadata. Heartbeat loss or reset clears it. Frames older
than the bound heartbeat or cached report cannot replace it.

The pinned Copter 4.6.3 sender takes `vibration_m_s2` from the first usable
accelerometer's filtered residual. It reports three unsigned `clipping_counts`
in wire order for accelerometer indexes 0, 1 and 2. A zero counter may come from
an unavailable index. The vector has no per-report sensor index, so it cannot
identify which clipping counter corresponds to the selected accelerometer.
The module preserves the reported values and leaves sensor presence and health
unassessed. Nonfinite or negative vibration levels reject the entire report.
[MAVLink VIBRATION](https://mavlink.io/en/messages/common.html#VIBRATION) ·
[Pinned Copter sender](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/GCS_MAVLink/GCS_Common.cpp#L2999-L3014) ·
[Pinned accelerometer implementation](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/AP_InertialSensor/AP_InertialSensor.cpp#L2234-L2291)

`time_usec_raw` is the unsigned sender timestamp as a decimal string, preserving
all 64 bits for JSON consumers. The pinned sender uses the FC boot clock at
serialization. The module retains regressions as observations without adjusting
local age or inferring counter wraps, resets or vibration thresholds.
The `imu.vibration` capability uses the FC's existing inbound stream.

Configuration is `imu_timeout=1` seconds (allowed 0.25–10) and
`imu_publish_interval=0.1` seconds (allowed 0.02–2). Startup selection and module
path follow the app's normal plugin configuration. These settings do not change
control freshness or navigation authority.

MAVROS's pinned `imu` plugin supplies the coherent sensor-capability reference.
MAVMOOS retains source frames and families rather than adopting ROS frame
transforms, fused-message cache composition or family precedence. HIGHRES_IMU 105
remains deferred. Both inspected ArduPilot pins multiply a
milligauss compass field by 1000 while labelling HIGHRES_IMU output gauss; this
source conflict needs separate resolution before adding a trustworthy magnetic
conversion. No physical defect or version-wide behavior is inferred solely from
that code. See the dated IMU source/evidence archive for exact revisions and
line-level reasoning.

The component is software-tested with an actual loaded module and fake Services,
including malformed/truncated wire frames and zero-effect checks. Native/SITL
support is a separate qualification result; this document makes no hardware,
sensor-accuracy or firmware-version-range claim.
