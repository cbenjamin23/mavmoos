# Clock status

Enable `clock_status` in the explicit `plugins` list to observe `SYSTEM_TIME` (2) and `TIMESYNC` (111). The provider binds to the selected FC route after a matching ArduPilot or PX4 heartbeat at system 1, component 1. All three observations are read-only.

~~~moos
plugins = telemetry,clock_status
plugin.clock_status.sample_timeout = 1.0
plugin.clock_status.publish_interval = 0.1
~~~

The sample timeout defaults to 1.0 s and accepts 0.25–10 s. The publication interval defaults to 0.1 s and accepts 0.02–2 s up to the timeout. The provider advertises zero request owners or stream-setup entries. Observations come from streams the sender already emits.

## MOOS records

| Variable | Wire class | Data |
| --- | --- | --- |
| `MAVMOOS_SYSTEM_TIME` | `SYSTEM_TIME` | Boot milliseconds, raw Unix microseconds and whether the sender reported a nonzero UTC field |
| `MAVMOOS_TIMESYNC_REQUEST` | `TIMESYNC` with zero `tc1` | Request-shaped packet, raw signed timestamp fields and any target address bytes |
| `MAVMOOS_TIMESYNC_REPLY` | `TIMESYNC` with nonzero `tc1` | Reply-shaped packet with the same raw fields and its own sample time |

Each schema-1 record reports its own `observation_id`, `effective_at`, `read_at`, `age_s`, `fresh`, `wire_version` and `payload_length`. Repeated publications retain the same original times. The three bounded caches age separately and clear on identity loss, link reset or host clock fault. Incoming samples require a bound heartbeat. `source_age=unknown` reflects the absence of a qualified clock mapping. `authoritative_for_control=false` applies to every publication.

`time_boot_ms` retains the unsigned 32-bit field including zero and wrap values. `time_unix_usec_raw` retains the unsigned 64-bit wire value as a decimal string. When it is zero, `utc_reported=false` and `time_unix_usec=null`. A nonzero field is copied as a string and describes the FC's report. It carries no accuracy qualification. A decreasing boot value remains an observation, since wrap and restart require additional evidence. [MAVLink system-time definition](https://mavlink.io/en/messages/common.html#SYSTEM_TIME).

`tc1_ns_raw` and `ts1_ns_raw` are exact signed decimal strings. A reply-shaped packet can arrive independently of a request. The type follows the protocol discriminator and says nothing about correlation. `timestamp_domains=unqualified` leaves boot versus epoch unresolved. The provider keeps the raw values and uses only local acquisition metadata for freshness. There is no host probe, response, pairing, offset estimate or clock adjustment. [MAVLink time synchronization protocol](https://mavlink.io/en/services/timesync.html).

The optional `target_system_raw` and `target_component_raw` are null when the corresponding MAVLink 2 extension byte is absent. Present zeros remain numeric zeros. Valid trailing-zero trimming can omit a zero-valued extension. `target_scope` describes the decoded address shape as `broadcast_or_unspecified`, `system_all_components`, `system_component` or `component_without_system`. The observer reports the address as evidence and applies source binding independently. `invalid_family_frames` for TIMESYNC counts malformed frames shared across request and reply because invalid wire shape cannot safely classify the role. [Pinned generated TIMESYNC header](https://github.com/mavlink/c_library_v2/blob/a2d0ecf011a4aaa7840480e43f4e38d8c9e79d29/common/mavlink_msg_timesync.h).

## References and validation

MAVROS `sys_time` separates FC time-reference publication from active TIMESYNC exchanges. Its active handler can respond to requests, accept positive response timestamps and derive a filtered offset. MAVMOOS uses the wire-family separation and keeps the raw observation path passive. [Pinned MAVROS sys_time, blob `27c0b47`](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/sys_time.cpp).

The pinned Copter 4.6.3 producer sends `SYSTEM_TIME` using the RTC's UTC value when available and boot milliseconds separately. Its own TIMESYNC request uses local microseconds multiplied by 1000 plus the system ID. Its response uses a receive-time estimate in nanoseconds and mirrors `ts1`. Both sends at this pin use the older address-free payload. The local data-collection timer sends its own request around every ten seconds on eligible non-private links. The development pin follows the same clock conventions. [Pinned Copter system-time sender](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/GCS_MAVLink/GCS_Common.cpp#L2052-L2063) · [Pinned Copter TIMESYNC timer](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/GCS_MAVLink/GCS_Common.cpp#L1910-L1923) · [Pinned Copter TIMESYNC code](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/GCS_MAVLink/GCS_Common.cpp#L3591-L3681) · [Development pin](https://github.com/ArduPilot/ardupilot/blob/04cf6486e8d42ce138de816e8466244346ed77cb/libraries/GCS_MAVLink/GCS_Common.cpp#L3729-L3819).

All seven focused checks passed in Debug and ASan/UBSan on macOS ARM64. They cover original age, legacy and trimmed frames, exact integer limits, address shape, source identity, clock faults, reset and passive callbacks. The generated app fixture exercises actual ingress and MOOS publication for independently aged requests and replies. A future disarmed native witness can passively observe scheduled `SYSTEM_TIME` and the FC's own TIMESYNC request. Generated unsolicited replies exercise the separate receive path.
