# Remote ID observation

Select `plugins = telemetry,remote_id` and an explicit `remote_id_component` in 1–255. This provider uses the owned SITL loopback route and a fresh selected flight controller heartbeat from 1/1. Select component 1 for Remote ID fields emitted by the simulated flight controller. Select the explicit sidecar component for an owned simulated transmitter. It consumes three MAVLink 2 message families from that component. [MAVLink Open Drone ID](https://mavlink.io/en/services/opendroneid.html)

| MOOS variable | What it reports |
| --- | --- |
| `MAVMOOS_REMOTE_ID_IDENTITY` | Basic ID type, aircraft type, ID presence and a process-session identity change counter |
| `MAVMOOS_REMOTE_ID_LOCATION` | Aircraft location, WGS84 and barometric altitude, referenced height, course, speed, reported status and raw accuracy categories |
| `MAVMOOS_REMOTE_ID_HEALTH` | Reported transmitter status and presence of error text |

Identity and location admit self reports with an empty relay identifier, addressed to the selected system or broadcast. The provider redacts the aircraft ID and transmitter error text. It does not expose operator ID, operator position, or neighboring aircraft. Identity generations begin at 1 within each heartbeat session and advance when the declared identifier changes. They are local counters and carry no cross-session identity meaning. Reported transmitter health is an observation and confers no arming permission.

Each family retains its original receive time. An expired family publishes null data independently of the other families. Source loss clears all retained reports. Location uses the protocol's unavailable sentinels for position, height, altitude, course, speed and timestamp. The age of a source fix is unknown, even when a UTC second-of-hour is present.

Configure `plugin.remote_id.sample_timeout` in seconds, default 3.5, and `plugin.remote_id.publish_interval`, default 0.1. The observer does not relay or broadcast packets. The simulated endpoint witness covers MAVLink reports and local MOOS publications. [Open Drone ID message definitions](https://mavlink.io/en/messages/common.html#OPEN_DRONE_ID_BASIC_ID)

Pinned native PX4 supplied location. The owned modeled transmitter supplied Basic ID and health through the actual MOOS app. Remote ID evidence
