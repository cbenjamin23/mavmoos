# Link status

Enable `link_status` in the explicit `plugins` list to observe MAVLink `RADIO_STATUS` (109) and `CELLULAR_STATUS` (334). The provider uses the app's selected peer and a fresh matching ArduPilot or PX4 heartbeat at system 1, component 1. It publishes diagnostic observations. The descriptor owns zero request variables, control resources or stream setup entries.

~~~moos
plugins = telemetry,link_status
plugin.link_status.sample_timeout = 1.0
plugin.link_status.publish_interval = 0.1
~~~

The default sample timeout is 1.0 s, configurable from 0.25 to 10 s. The default publication interval is 0.1 s, configurable from 0.02 to 2 s and bounded by the timeout. A selected sender must already provide the messages.

## MOOS records

| Variable | Scope | Contents |
| --- | --- | --- |
| `MAVMOOS_RADIO_STATUS` | One sample for the bound FC route | Local and remote RSSI and noise, transmitter buffer availability, receiver error and correction counters |
| `MAVMOOS_CELLULAR_STATUS` | Up to eight distinct modem IDs in the bound session | Raw modem and radio type enums, quality, network codes and optional modem extensions |

Both schema-1 publications identify the MAVLink family and message ID. A radio record carries its observation metadata at the top level. A cellular envelope carries a sorted `modems` array with separate `metadata` and `data` for each modem. Its `atomic=false` flag reflects the separate sample times. Modem ID 0 means the sender has supplied no numbered instance. It is distinct from a numbered modem. New IDs beyond the eight-slot limit increment `capacity_drops`. Existing IDs can still update. Session reset clears the slots. [MAVLink cellular definition](https://mavlink.io/en/messages/common.html#CELLULAR_STATUS).

The original effective and read times age independently for each record. Sample expiry marks it stale and retains its last data until the bound heartbeat is lost or the provider resets. A fresh matching heartbeat is required before data can return. `read_only=true` and `authoritative_for_control=false` describe their scope. The wire messages provide no generation timestamp, so `source_age=unknown` is separate from the local sample age.

### Radio

Local and remote describe the sender's perspective. `rssi_raw`, `remrssi_raw`, `noise_raw` and `remnoise_raw` retain all bytes. Their `*_device_units` counterparts use null for 255 and preserve zero as a reported value. `signal_scale=device_dependent` describes the values. A modem-specific conversion requires its own qualified source.

`tx_buffer_free_pct` is remaining free transmit-buffer percentage. Lower values indicate less free space. The raw value remains visible when the interpreted percentage is unavailable because it exceeds 100. The 16-bit `rx_errors_raw` and `corrected_packets_raw` are sender counters. Deriving rates, counter wrap or restart requires a separately qualified sequence. The pinned ELRS simulator can supply buffer feedback with all four signal fields unknown and both counters zero. [MAVLink radio definition](https://mavlink.io/en/messages/common.html#RADIO_STATUS) · [Pinned ELRS sender](https://github.com/ArduPilot/ardupilot/blob/04cf6486e8d42ce138de816e8466244346ed77cb/libraries/SITL/SIM_ELRS.cpp#L108-L136).

### Cellular

`status_raw`, `failure_reason_raw` and `network_type_raw` preserve the wire enums, including values from newer senders. `failure_reason_applicable` is true in the defined FAILED state. The raw failure code is always retained. `quality_pct` is null outside 0–100, including the defined 255 unknown value, and `quality_pct_raw` keeps the byte. The raw MCC, MNC and LAC are preserved. The interpreted fields use null for MCC or MNC 65535 and LAC zero.

The pinned message defines optional rates, tower ID, band, channel, levels, receiver quality and SINR. Their zero sentinels become null. The two KiB/s fields keep the MAVLink TX and RX names because the message's download and upload descriptions do not establish the direction relative to the bridge. `rx_quality_raw` retains the modem-technology-specific value. If transmitted, `cell_tower_id_raw` gives the nine decoded bytes. A printable identifier becomes a string and stops at the first NUL. Binary or empty text is null. A nonfinite optional float invalidates the packet without renewing the previous sample.

Valid MAVLink 2 trailing-zero trimming applies across base and extension fields. `payload_length` records the wire length. `RADIO_STATUS` also accepts exact-length MAVLink 1. `CELLULAR_STATUS` requires MAVLink 2 because its message ID exceeds 255. [Pinned generated codec](https://github.com/mavlink/c_library_v2/tree/a2d0ecf011a4aaa7840480e43f4e38d8c9e79d29/common).

## Source and qualification scope

MAVROS's `tdr_radio` accepts both 109 and its historical predecessor, 166, reports raw modem fields, and applies the 3DR-specific RSSI conversion. Its filter expects the modem's own source identity. MAVROS's `cellular_status` plugin sends a ROS report to the FC. MAVMOOS covers inbound 109 and 334 observations from the selected FC route with a separate lifetime for each stream. [Pinned MAVROS 3DR source, blob `59a73a4`](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros_extras/src/plugins/3dr_radio.cpp) · [Pinned MAVROS cellular source, blob `c61c7a3`](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros_extras/src/plugins/cellular_status.cpp).

The pinned ArduPilot receive path consumes 109 and 166 for telemetry backpressure. Its router treats those packets as local to the incoming link. The ELRS simulation source is system 255 and component 0 and sends toward the autopilot. This module's supported route therefore uses generated FC-shaped frames. Historical 166 and dedicated modem identities remain outside that route. The test scope is message decoding, source isolation, observation lifetimes and passive app behavior. [Pinned ArduPilot radio handler](https://github.com/ArduPilot/ardupilot/blob/04cf6486e8d42ce138de816e8466244346ed77cb/libraries/GCS_MAVLink/GCS_Common.cpp#L878-L950) · [Pinned ArduPilot routing](https://github.com/ArduPilot/ardupilot/blob/04cf6486e8d42ce138de816e8466244346ed77cb/libraries/GCS_MAVLink/MAVLink_routing.cpp#L133-L141) · [ELRS source identity](https://github.com/ArduPilot/ardupilot/blob/04cf6486e8d42ce138de816e8466244346ed77cb/libraries/SITL/SIM_ELRS.cpp#L22-L36).

All eight focused checks passed on macOS ARM64 in Debug and ASan/UBSan. They exercise both supported heartbeat identities, malformed data and original age, unknown fields, valid trimming, bounded modem storage, reset and zero effectful callbacks. The generated shared app fixture adds prebind, stale, loss and recovery checks plus the app's zero-output witness.
