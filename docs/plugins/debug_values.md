# Debug value observations

`debug_values` is an optional passive module for flight-controller debug scalars and vectors. Select it at startup when a simulator or firmware build already emits the reports. The module receives the bound primary FC component on ArduPilot or PX4 routes and owns only observations.

```moos
plugins = telemetry,debug_values
plugin.debug_values.sample_timeout = 1
plugin.debug_values.publish_interval = .1
```

The common passive observer accepts timeouts from 0.25 to 10 seconds and publish intervals from 0.02 to 2 seconds, with the interval at most the timeout. Defaults are shown above.

| MOOS variable | MAVLink | Sample identity and data |
| --- | --- | --- |
| `MAVMOOS_DEBUG` | `DEBUG` (254) | Unsigned byte `index`, raw boot milliseconds, floating value and state |
| `MAVMOOS_DEBUG_VECT` | `DEBUG_VECT` (250) | Byte label, raw microseconds, three ordered `xyz` values and their `xyz_states` |
| `MAVMOOS_DEBUG_FLOAT_ARRAY` | `DEBUG_FLOAT_ARRAY` (350) | Unsigned 16-bit `array_id`, byte label, raw microseconds and 58 decoded floats with wire coverage |
| `MAVMOOS_NAMED_VALUE_FLOAT` | `NAMED_VALUE_FLOAT` (251) | Byte label, raw boot milliseconds, floating value and state |
| `MAVMOOS_NAMED_VALUE_INT` | `NAMED_VALUE_INT` (252) | Byte label, raw boot milliseconds, signed 32-bit value and state |

Each schema 1 publication carries `family`, `message_id`, `present`, `samples`, `sample_limit`, `invalid_samples`, and `capacity_drops`. A family holds the first eight identities admitted during its bound session. Known identities can update at capacity. New identities beyond the limit increment the capacity counter. The next session starts with an empty cache. Each member has its own `wire_version`, `payload_length`, `metadata` and `data`. The aggregate is `atomic:false`.

Name fields use up to ten decoded bytes through the first NUL or the end of the field. Named scalars and vectors use case-sensitive `name_raw_hex` as their identity. Printable ASCII has `name_state:"text"` and an escaped JSON `name`. Empty or non-ASCII labels use `name:null` with state `empty` or `non_ascii`. An empty label occupies one slot in a name-keyed family. Float and integer reports with the same label occupy separate caches. Arrays use `array_id` for identity and carry the label for display.

Finite scalar and vector values are `reported`. Float NaN and signed infinities are retained as JSON nulls with states `wire_nan`, `wire_positive_infinity`, and `wire_negative_infinity`. The signed integer range is preserved, along with floating negative zero. MAVLink 2 zero padding can restore bytes trimmed from a base field. The wire length and version remain available with the decoded value.

For arrays, `values`, `value_states` and `element_wire_bytes` each contain 58 entries in wire order. `extension_payload_bytes` counts the bytes after the 20-byte base. An element can have zero to four wire bytes because MAVLink 2 trims trailing zeros by byte. A finite value with at least one wire byte has state `reported`. An element restored entirely by zero padding has state `zero_padded`. [Generated DEBUG_FLOAT_ARRAY definition, blob `8248cba`](https://github.com/mavlink/c_library_v2/blob/a2d0ecf011a4aaa7840480e43f4e38d8c9e79d29/common/mavlink_msg_debug_float_array.h)

`metadata.fresh` reflects the original local acquisition interval and expires independently per member. A recent publication can contain stale members. A fresh sample with a nonfinite value state still records that the FC sent the report. A bound heartbeat expiry, reset or local clock fault clears the cache. Remote boot milliseconds are exposed as `time_boot_ms_raw`. Vector and array `time_usec_raw` use decimal strings for full 64-bit precision, with the possible boot or UNIX clock domain marked unmapped. The descriptor carries observation capabilities and zero stream requests.

The pinned MAVROS `debug_value` plugin dispatches all five types through separate handlers, assigns ROS headers and exposes a debug send subscription. MAVMOOS implements the five receive paths as passive observations with a bounded cache and per-identity original ages. [Pinned MAVROS source, blob `c4616bb`](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros_extras/src/plugins/debug_value.cpp)

The generated MAVLink definitions fix the wire sizes and field types. [DEBUG](https://github.com/mavlink/c_library_v2/blob/a2d0ecf011a4aaa7840480e43f4e38d8c9e79d29/common/mavlink_msg_debug.h) · [DEBUG_VECT](https://github.com/mavlink/c_library_v2/blob/a2d0ecf011a4aaa7840480e43f4e38d8c9e79d29/common/mavlink_msg_debug_vect.h) · [NAMED_VALUE_FLOAT](https://github.com/mavlink/c_library_v2/blob/a2d0ecf011a4aaa7840480e43f4e38d8c9e79d29/common/mavlink_msg_named_value_float.h) · [NAMED_VALUE_INT](https://github.com/mavlink/c_library_v2/blob/a2d0ecf011a4aaa7840480e43f4e38d8c9e79d29/common/mavlink_msg_named_value_int.h)

The loadable-module fixture covers provider metadata, exact payload values and labels, malformed packets, source and order gates, original-age expiry, reset, configuration bounds and per-family capacity. `DebugFrames.h` supplies ten generated app vectors and `wire_fixture.py` checks their exact projections and lifecycle.
