# Terrain observations

Enable the optional `terrain` module in the startup plugin list to observe flight-controller terrain reports and requests. It owns `observation.terrain` and advertises `terrain.report` and `terrain.request_observed` as observation capabilities. Both are available with a qualified ArduPilot or PX4 heartbeat on the selected host route. A matching source identifies who reported the bytes. Firmware terrain support and elevation quality require separate evidence.

```moos
plugins = telemetry,terrain
plugin.terrain.sample_timeout = 3
plugin.terrain.publish_interval = 0.1
```

The shared observer accepts `sample_timeout` from 0.25 to 10 seconds and `publish_interval` from 0.02 to 2 seconds, with the interval at most the timeout. The defaults are shown above.

| MOOS JSON output | Wire input | Meaning |
| --- | --- | --- |
| `MAVMOOS_TERRAIN_REPORT` | `TERRAIN_REPORT` (136) | WGS84 latitude/longitude in decimal degrees, reported spacing in meters, FC cache block counts, and reported heights. `terrain_available` records the wire availability indicator. `terrain_height_msl_m` is a number when spacing is nonzero and null when spacing is zero. `terrain_height_reported_msl_m` preserves the raw number in either case. `current_height_reported_m` preserves the FC value and `current_height_validity` is `unspecified`. |
| `MAVMOOS_TERRAIN_REQUEST_OBSERVED` | `TERRAIN_REQUEST` (133) | WGS84 southwest origin, positive grid spacing in meters, exact lower 56 bits as a 14-digit hexadecimal string, and the number of requested 4×4 blocks. A request is an independent observation of demand for data. |

Each family keeps one latest sample. Schema 1 records the receive session and source, original effective and local read times, age and freshness, and a separate payload under `data`. A stale sample retains its original timestamps. A cleared session publishes `present=false` and `data=null`. Source measurement age is unknown. The module rejects invalid coordinates, nonfinite height fields, malformed frame shapes, and requests with zero spacing or set bits above bit 55. A reported zero elevation with positive spacing is valid at the wire level. The native ArduPilot sender can also use (0, 0) as an unbound location fallback, Treat it as a report coordinate and use separate vehicle position telemetry.

In pinned Copter 4.6.3, zero spacing can accompany a nonzero extrapolated terrain estimate. The optional height above terrain is computed at the current AHRS location even when a report answers a check for another location. Its zero value can also mean unavailable. Cache counts describe the FC's loaded and pending work, including pending disk work. These fields are observations with `read_only=true` and `authoritative_for_control=false`. The MOOS interface publishes observations only. Terrain tile distribution and terrain-following authority are outside its scope. [Copter terrain report producer](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/AP_Terrain/TerrainGCS.cpp#L167-L258), [Copter height above terrain](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/AP_Terrain/AP_Terrain.cpp#L243-L267).

## Reference and evidence

Pinned MAVROS `terrain` (blob `7f69f19848a6eb7d5d44d6eb42e060b26ec1e82e`) publishes FC reports and requests on separate ROS topics. Its additional companion server paths translate tile data and point queries. MAVMOOS follows the two observation families and attaches original ingress-age metadata and explicit report validity. The wire format comes from generated MAVLink `a2d0ecf011a4aaa7840480e43f4e38d8c9e79d29`. [Pinned MAVROS terrain plugin](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros_extras/src/plugins/terrain.cpp), [MAVLink terrain protocol](https://mavlink.io/en/services/terrain.html).

The DSO fixture covers both messages, full masks, zero-spacing and zero-height cases, foreign sources, invalid payloads, old samples, heartbeat loss, reset, and zero effectful service calls. The real app generated-frame witness exercises the standard receive path. In the proposed owned disarmed Copter bench, a private simulator instance with an empty terrain store and the `EXTRA3` stream enabled can emit cache-status reports and requests. Availability in a real deployment remains tied to firmware, telemetry configuration, and loaded terrain. [Copter stream table and dispatch](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/ArduCopter/GCS_Mavlink.cpp#L336-L348).
