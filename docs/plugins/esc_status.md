# ESC observations

The optional startup module `esc_status` publishes three independent ESC report
families from an ArduPilot flight controller with system and component IDs `1/1`.
Select it alongside the other desired modules in `ProcessConfig = iMAVLink`.

```moos
plugins = telemetry,esc_status
target_system = 1
target_component = 1
plugin.esc_status.sample_timeout = 1
plugin.esc_status.publish_interval = 0.1
```

The timeout accepts 0.25–10 seconds. Publication interval accepts 0.02–2 seconds
and must be at most the sample timeout. The module consumes passive streams
already sent by the selected source and publishes only JSON MOOS observations.

| MOOS variable | MAVLink report | Coverage and fields |
| --- | --- | --- |
| `MAVMOOS_ESC_INFO` | `ESC_INFO` (290) | `time_usec_raw`, packet `counter`, `count`, `connection_type`, `info_raw`, per-entry error count, failure flags, and temperature in centidegrees Celsius |
| `MAVMOOS_ESC_STATUS` | `ESC_STATUS` (291) | `time_usec_raw`, signed `rpm`, `voltage_v`, and `current_a` |
| `MAVMOOS_ESC_TELEMETRY` | ArduPilot `ESC_TELEMETRY_1_TO_4` (11030) | First reported group, with `voltage_cv`, `current_ca`, `total_current_mah`, `rpm_raw`, `packet_count`, and `temperature_c` |

Each MOOS value has `schema=1`, `present`, and a sparse `groups` array. A
group carries `first_index`, `message_id`, `entries`, and its own `metadata`
object. Indices are zero-based reported slots. The common messages accept first
indices 0 through 60 in steps of four, with at most sixteen groups per family.
INFO retains only entries in that packet's reported total count. An INFO count of
zero at index zero records the sender's empty group. STATUS has four reported
slots and has no count of its own. The two families age independently, including
when an earlier INFO packet reports a different count.

INFO temperature `INT16_MAX` and nonfinite STATUS voltage or current appear as
JSON `null`. Other numbers, including zeros and flags, retain their wire values.
Group membership describes a packet's slots. Device presence remains unknown and
health remains unassessed. The dialect calls the ArduPilot RPM field eRPM, while
the pinned sender fills it from its potentially scaled `get_rpm()` result.
`rpm_raw` preserves the value without a shaft or electrical RPM conversion. The
flight controller may apply its ESC telemetry offset before populating the group.
`time_usec_raw` is a decimal string that preserves the common messages' full
unsigned wire timestamp. Local freshness uses the observation clock. The
wrapping ArduPilot packet counter remains a reported field.
[Common report definitions](https://mavlink.io/en/messages/common.html#ESC_INFO) ·
[ArduPilot dialect](https://mavlink.io/en/messages/ardupilotmega.html#ESC_TELEMETRY_1_TO_4) ·
[Pinned ArduPilot sender](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/AP_ESC_Telem/AP_ESC_Telem.cpp#L407)

The metadata carries the source and session, original local effective and read
times, local age basis, and `fresh`. Republishing a cached group preserves its
observation ID and timestamps. The cache clears on heartbeat loss, host reset or
local clock regression. Until a post-boundary heartbeat and report arrive,
`present=false` and `groups=[]` describe the absence of a report. Source age and
physical measurement quality remain unknown. The observations are independent
and have no navigation or actuation interface.

MAVROS supplied the message-family boundaries and the four-entry group
interpretation. Its ROS status plugin assembles reports across groups. MAVMOOS
publishes independently aged groups through MOOS.
[Pinned MAVROS ESC status plugin](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros_extras/src/plugins/esc_status.cpp) ·
[Pinned MAVROS ESC telemetry plugin](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros_extras/src/plugins/esc_telemetry.cpp)

The focused `ctest.esc_status_observations` case loads the module with fake
host services and generated MAVLink frames. Its evidence covers decoding,
bounded groups, rejected input, freshness, reset and read-only behavior. Native
availability is a separate simulator result and depends on an active telemetry
producer. Common ESC frames in the test do not establish Copter delivery for
those message IDs.
