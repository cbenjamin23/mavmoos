# System status observations

`system_status` is an optional, read-only provider for ArduPilot system sensor masks, batteries, raw GNSS receiver and RTK diagnostics, flight-controller status text, supply rails, and estimator diagnostics. It shares the host's configured connection and selected vehicle. Its diagnostic records are separate from the private flight-readiness inputs and policy.

Current host/module builds must match development ABI **10**. Conservative effective acquisition time and actual read time are delivered separately. Build the optional provider with the matching host headers.

## Loading and settings

Add `system_status` to an explicit provider list in the normal `ProcessConfig = iMAVLink` block, retaining the providers needed by that mission:

```text
plugins                 = telemetry,system_status
plugin_directory        = /path/to/built/plugins
status_timeout          = 3
status_publish_interval = 0.5
status_text_timeout     = 2
```

The example selects observations only. The app's existing receive-only mode remains the default. Its descriptor advertises `SYS_STATUS` and `BATTERY_STATUS` at 1 Hz and `GPS_RAW_INT` at 2 Hz to the host's existing observation-stream mechanism. The host alone decides whether any interval request is permitted. The secondary GNSS, RTK and other diagnostics use streams already configured at the FC. `STATUSTEXT` is event-driven.

The descriptor permits coexistence in a streamed-parameter bench because this provider has no control effects. That is a composition declaration, not qualification of full-table throughput or FC telemetry scheduling with this additional observation load.

| Setting | Default | Accepted bounds |
|---|---:|---:|
| `status_timeout` | 3 s | 0.25–30 s |
| `status_publish_interval` | 0.5 s | 0.05–10 s |
| `status_text_timeout` | 2 s | 0.25–10 s |

Bounds are local policy. Limits are fixed at eight identified battery IDs, eight MCU IDs, four concurrent text assemblies, sixteen chunks / 800 bytes per assembly, and sixteen pending text outcomes. Excess IDs and assemblies increment rejection counters. Pending text outcomes drop the oldest with an explicit counter. Battery and MCU IDs are retained until a connection/session reset.

## Published interface

The variables are JSON strings, independently versioned as schema 1.

| MOOS variable | Content |
|---|---|
| `MAVMOOS_SYS_STATUS` | Latest system sensor masks, reported pre-arm check, load, communications loss and error counters |
| `MAVMOOS_BATTERY_STATUS` | One record per identified battery, plus a distinctly labelled ambiguous `SYS_STATUS` fallback |
| `MAVMOOS_GPS_STATUS` | Latest primary `GPS_RAW_INT` receiver report |
| `MAVMOOS_GPS2_STATUS` | Independent second receiver report from `GPS2_RAW` |
| `MAVMOOS_GPS_RTK` | First receiver relative baseline report from `GPS_RTK` |
| `MAVMOOS_GPS2_RTK` | Second receiver relative baseline report from `GPS2_RTK` |
| `MAVMOOS_FC_TEXT` | Once-per-outcome complete/incomplete text events and session invalidation events |
| `MAVMOOS_POWER_STATUS` | Board and servo supply rails from `POWER_STATUS` |
| `MAVMOOS_MCU_STATUS` | Bounded `instances` array of `MCU_STATUS` reports, sorted by MCU ID |
| `MAVMOOS_EKF_STATUS_REPORT` | ArduPilot EKF flags and reported innovation levels from `EKF_STATUS_REPORT` |
| `MAVMOOS_ESTIMATOR_STATUS` | Common estimator flags, test ratios, accuracy estimates and remote timestamp from `ESTIMATOR_STATUS` |
| `MAVMOOS_SYSTEM_STATUS` | Bounded aggregate snapshot with cached records, source binding, session/reset and rejection/capacity counters. Receiver records appear as `gps`, `gps2`, `gps_rtk` and `gps2_rtk`. The other diagnostics appear as `power`, `mcu`, `ekf` and `estimator`. |

Multiple batteries publish on the same MOOS variable, so its latest DB value is not an inventory. Use the aggregate's `batteries` array for a coherent bounded snapshot of the current cache. An explicit absent battery record is published when the cache is empty, including after reset, so the latest DB value does not retain an old fresh-looking battery record. `sys_status_battery_fallback` is separate even when identified battery 0 exists: `SYS_STATUS` does not carry the battery ID needed to equate them.

`prearm_check` extracts the flight controller's present, enabled and health bits from `SYS_STATUS`. `reported_healthy` is null for an absent or disabled check. In the pinned Copter sender, the health bit is set when the vehicle is armed or its enabled pre-arm checks pass. The record reports that observation with its own age. Use the Copter command result for the actual arm decision. [Pinned sender](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/GCS_MAVLink/GCS.cpp#L342-L351)

Each sample record includes `present`, `encoding_valid`, `fresh`, `effective_at`, `read_at`, `age_s`, `age_basis`, `age_is_upper_bound`, `observation_id`, and `session_epoch`. Absent fields are null, not invented zero measurements. Re-publication retains the original acquisition stamps and observation ID. A numerically well-formed sample can be stale; a fresh sample can contain unknown or invalid quantities. Check each quantity's `known`, `value`, `unit` and `reason`, rather than treating record freshness as quality or safety.

`QUEUE_EMPTY_FLOOR` is an intentionally conservative lower bound on local acquisition: age is an upper bound on time since acquisition, including potential local socket-queue residence. `READ_ACQUISITION` retains the actual local read time when no stronger bound is supplied. Neither establishes sensor generation time, age accumulated before reaching this host, clock synchronization, or source quality. `source_age` is `unknown`; `source_quality` is `unassessed`.

## Field interpretation

**System:** raw present/enabled/health bitmasks remain available. `unhealthy_enabled_present` intersects the three masks; inconsistent enabled/healthy bits outside the present mask are reported separately. Extended mask bytes are distinguished from no observed extension bytes, but all-zero extension fields cannot establish feature support. Load is percent (`wire / 10`), drop rate is percent (`wire / 100`), and error counters retain their raw counts. No aggregate `safe` or `healthy` boolean is emitted.

**Batteries:** temperature is °C, voltage V, current A, consumed charge mAh, consumed energy Wh (`hectojoules / 36`), remaining charge percent, and remaining time seconds. Sentinel unknowns are independent: temperature `32767`, current `-1`, consumed charge/energy `-1`, percentage `-1`, base voltage `65535`, and time remaining `0`. Other negative current values remain signed MAVLink current, rather than being reversed into the ROS BatteryState convention. Signed-wire overflow/rollover cannot be ruled out from this observation.

All fourteen voltage slots retain raw mV values. Slots can represent cells **or pieces of a split pack voltage**, so they are not labelled universally as cell readings. `reported_voltage_sum_v` is available only for a nonempty contiguous set of known slots; it is not a verified pack voltage or cell count. Extended slots use `0` as unsupported. This ArduPilot profile also treats extended `65535` as unused: the pinned FC copies its internal unused-cell sentinel there, and the pinned MAVROS reader accepts that convention. An extended raw `1` remains 0.001 V, with an explicit note that the wire convention may use it to encode measured zero. Mode `0` is ambiguous between normal and unsupported. A zero fault mask does not imply supported fault reporting; support is labelled only for the wire-defined FAILED/UNHEALTHY charge states.

**GPS:** raw latitude/longitude use degrees, altitude is MSL metres, velocity is ground speed in m/s, course is degrees of travel and is not heading, and DOP is a ratio. A valid receiver fix is required for coordinates and a 3D-or-better fix for altitude. Zero coordinates and zero speed are legitimate with a fix. Satellites `255` and DOP/speed/course `65535` remain unknown. Yaw `0` means unsupported, `65535` means configured but unavailable, and `36000` means north (0°). Position accuracy and ellipsoid-altitude extensions remain independent; zero extensions are conservatively labelled zero-or-unreported. No geoid conversion, fused vehicle pose, receiver selection, covariance inference, or estimator-readiness conclusion is made. The 64-bit remote timestamp is a decimal string with an unmapped boot-or-Unix clock, never a freshness clock.

**Second receiver.** `GPS2_RAW` uses the same fix, coordinate and unit
interpretation as the primary receiver. Yaw and the accuracy and ellipsoid
fields distinguish an extension absent from the payload from a field present
with a zero or sentinel value. The original differential correction age in
milliseconds and satellite count stay in `dgps_age_reported_ms` and
`dgps_satellites_reported_raw`. A pair of zeroes is ambiguous because the
pinned sender copies the driver state. It reports the second configured GPS
and places its last fix boot time in microseconds on the wire. Each of the
three added records has its own original local receipt, source IDs and session
epoch. [MAVLink GPS2_RAW](https://mavlink.io/en/messages/common.html#GPS2_RAW) ·
[Pinned ArduPilot second receiver sender](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/AP_GPS/AP_GPS.cpp#L1404-L1455)

**RTK baseline.** The two MAVLink message IDs identify the first and second
receiver record separately. `rtk_receiver_id_raw` preserves the additional
sender field. The reports retain signed baseline components in millimetres
and expose metres with explicit ECEF X/Y/Z or north/east/down axes when the
wire frame is known. An unknown frame retains the original triple and leaves
the interpreted baseline unavailable. The wire accuracy field carries
unspecified units. Boot milliseconds, GPS week/time-of-week, health, rate,
satellites and integer ambiguity hypotheses stay separate diagnostic values.
The pinned ArduPilot sender fills baseline time, receiver ID, health and rate
with zero placeholders. It sends these reports only for receiver backends
that advertise support. The observer leaves the baseline relative and the
solution quality unassessed. [MAVLink GPS_RTK](https://mavlink.io/en/messages/common.html#GPS_RTK) ·
[MAVLink GPS2_RTK](https://mavlink.io/en/messages/common.html#GPS2_RTK) ·
[Pinned ArduPilot baseline sender](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/AP_GPS/GPS_Backend.cpp#L176-L214)

**Supply rails.** `POWER_STATUS` preserves the raw board and servo millivolts
alongside independently interpreted voltages. A zero rail has a null
interpreted value and reason `zero_or_unreported`. The ArduPilot HAL uses
zero for unknown servo voltage, so a zero report alone leaves that domain
ambiguous. The flags carry the raw mask, named asserted or clear bits and
an unknown-bit mask. Brick/servo validity flags and voltage availability
remain separate observations. Battery voltage belongs to the existing
battery records.
[MAVLink POWER_STATUS](https://mavlink.io/en/messages/common.html#POWER_STATUS) ·
[Pinned ArduPilot sender](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/GCS_MAVLink/GCS_Common.cpp#L215-L255) ·
[Pinned HAL defaults](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/AP_HAL/AnalogIn.h#L30-L60)

**MCU.** Each instance preserves the original signed centidegree temperature
and the three millivolt values. The scaled temperature is the sender's
reported value. Current, minimum and maximum supply readings use the same
independent zero handling as the other rails. `reading_within_reported_range`
is null until all three voltages are available and otherwise compares the
reported values. MAVLink 2 is required for message ID 11039. The bounded
snapshot includes `capacity_rejected`, and each retained instance has its
own age and observation ID. The pinned Copter sender exposes one MCU where
the build enables the monitoring code.
[MAVLink MCU_STATUS](https://mavlink.io/en/messages/ardupilotmega.html#MCU_STATUS) ·
[Pinned ArduPilot sender](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/GCS_MAVLink/GCS_Common.cpp#L243-L258)

**Estimator.** The two report families have separate freshness, raw masks
and flag definitions. For example, bit 1024 means uninitialized in
`EKF_STATUS_REPORT` and GPS glitch in `ESTIMATOR_STATUS`. The pinned
ArduPilot EKF sends normalized innovation levels through the fields named
`*_variance`. The observer exposes dimensionless reported values.
ArduPilot terrain level zero may mean unused, and airspeed distinguishes
MAVLink 1 extension absence from zero or unreported on MAVLink 2. Common
test ratios retain zero as a reported value and position accuracy uses
metres. A negative or nonfinite metric produces an explicit unknown with
its reason. The common report's full remote timestamp is a decimal string.
The two message families can describe different estimation conditions and
are displayed independently.
[Pinned MAVROS common estimator handler](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/sys_status.cpp#L1274-L1322) ·
[Pinned EKF3 metric calculation](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/AP_NavEKF3/AP_NavEKF3_Outputs.cpp#L489-L508) ·
[Pinned EKF3 report](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/AP_NavEKF3/AP_NavEKF3_Outputs.cpp#L581-L651) ·
[MAVLink ESTIMATOR_STATUS](https://mavlink.io/en/messages/common.html#ESTIMATOR_STATUS)

## Session and text boundaries

The module waits for a genuinely post-construction/post-reset heartbeat accepted by the host, matching its ArduPilot vehicle context. It then binds to that heartbeat's exact system/component IDs. The host still owns the configured peer and target-ID filtering; this observer does not replace it. A first queue floor predating module construction cannot bind a new session. Heartbeat expiry (two seconds), reset or local clock rollback clears cached observations and text assemblies; a later heartbeat cannot revive old values. Clock faults latch until an explicit host reset. Counters are cumulative for the provider instance; session epochs distinguish invalidations.

`STATUSTEXT` uses the pinned UTF-8, 50-byte chunk format. ID zero is a single message. Nonzero IDs start at chunk zero and require consecutive sequence numbers and stable severity. Exact duplicate previous chunks are tolerated without adding bytes or extending the original first-chunk deadline. Missing starts, gaps, conflicting duplicates, severity changes, expiry, reset and bounds produce explicit incomplete outcomes. Arrival of a new partial stream cannot extend an existing deadline. Some FC-side truncation is already null-terminated; a locally complete outcome proves only the received chunk sequence, not that the FC retained the entire original log string.

Text is validated after assembly so UTF-8 may span chunks. JSON escapes control characters and quotes; invalid UTF-8 bytes are represented safely for display. `raw_hex` always reconstructs the received logical text bytes represented by that event, excluding null termination/padding. Do not reconstruct invalid bytes by re-encoding the display string. Gap outcomes contain the accumulated prefix; missing-start outcomes contain the received fragment. `severity_valid` is separate from completion. Text is diagnostic input, never a command or a flight-state assertion. A MOOS DB may retain its last event indefinitely: use its session, original timestamps and age rather than assuming the latest event is current.

## References and evidence

MAVROS informs the functional split: [system/battery/text](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/sys_status.cpp) and [raw GPS versus fused position](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/global_position.cpp). MAVMOOS adds its own explicit unknowns, original acquisition ages, cache invalidation, ambiguous battery fallback and bounded text assembly; these are not claims that MAVROS supplies the same policy.

The source comparison, reproducible test archive and remaining limits distinguish real loaded-module/fake-host evidence from native integration. There is no hardware or radio evidence for this provider.

The owned native read-only bench now has one passing disarmed Copter observation run, with advancing late samples, a simulated battery voltage checked through MOOS/independent MAVLink/DataFlash, and zero observed app UDP on the configured relay route. Its first harness attempt remains archived as failed; this is simulation evidence only.

The grouped `observer_matrix_ardupilot` app test uses generated packets to
exercise all four diagnostics alongside the existing status and traffic
records. It checks unavailable domains, independent aging, loss and recovery
with a fresh bound heartbeat, wrong source rejection and zero bridge egress.
The optional `--status-diagnostics` argument on the existing native observer
runner adds two producer checks to that same owned simulator boot. Copter
SITL synthesizes a 5 V board rail and leaves the servo rail unavailable.
The native trace checks those rails and ArduPilot EKF reports against the
forwarded wire. MCU and common estimator report coverage comes from generated
app frames for this firmware pin.
[Pinned SITL analog input](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/AP_HAL_SITL/AnalogIn.h#L32-L41)

The GNSS extension follows MAVROS's separate receiver and baseline reports.
MAVMOOS uses local acquisition age and preserves the raw remote clock fields
and coordinate frame. Its observer supports receive-only diagnostics and the
selected source and session boundary. Generated vectors exercise the loaded
provider with partial extensions, unknowns, signed baselines, independent
ages and reconnects. Native qualification of the additional GNSS families is
pending an owned fixture with the corresponding receiver producer.
[Pinned MAVROS GPS status](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros_extras/src/plugins/gps_status.cpp)

## Battery identity and generation

`MAVMOOS_BATTERY_INFO` exposes up to eight independently aged `BATTERY_INFO` (372) records. Each has an exact FC source and battery ID, device label, serial and manufacture date bytes, reported design/full capacity in Ah, pack current limits, cell voltage limits, nominal voltage, weight, cycles and reported state of health. Names are shown only when their bytes are printable ASCII and NUL-terminated. A missing terminator is explicit. All label bytes through the first NUL are also available in hex. Known unavailable values remain null with a reason; a full-charge capacity of zero remains a report, while zero design capacity denotes unavailable. These are static manufacturer or FC estimates. They do not replace `BATTERY_STATUS` or infer present energy from a battery ID.

`MAVMOOS_GENERATOR_STATUS` exposes the selected FC's latest `GENERATOR_STATUS` (373), which has no generator ID. It preserves signed charge/load current, setpoint, bus voltage, reported power, temperatures, runtime, rpm and negative past-due maintenance time. Unknown integer sentinels and NaNs remain explicit. Reported state and fault flags remain independent of an operator's control permission. The raw and unknown 64-bit masks are decimal strings. No generator control, battery configuration or stream request was added.

Both families use the existing `status_timeout` and preserve the original acquisition stamp. Infrequent static battery metadata may be stale even while battery status is current. The module retains these old observations visibly and clears them on the FC session boundary. There is no automatic battery or generator failsafe decision.

[MAVLink BATTERY_INFO](https://mavlink.io/en/messages/common.html#BATTERY_INFO) supersedes SMART_BATTERY_INFO; the older family is outside this increment. The pinned Copter source can report [GENERATOR_STATUS from the configured generator backend](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/GCS_MAVLink/GCS_Common.cpp#L5949-L5958). The [RichenPower sender](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/AP_Generator/AP_Generator_RichenPower.cpp#L491-L504) shows why some fields may be unavailable. MAVROS's [system-status provider](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/sys_status.cpp) remains the reference for separating battery reports from controls. The pinned Copter source has no MAVLink BATTERY_INFO sender. Battery metadata coverage is generated. The generator backend is a pinned-source reference and this increment uses generated reports, without a native generator fixture.
