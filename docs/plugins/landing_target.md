# Landing target observation

The optional `landing_target` provider publishes the latest `LANDING_TARGET`
report received from one configured source component on the selected Copter
system. It provides the operator's target availability and pose inspection
workflow from pArduBridge. `fresh` describes local observation age. The
autopilot's target acquisition and Precision Loiter outcome are separate.

```text
plugins = telemetry,landing_target
landing_target_component = 158
plugin.landing_target.sample_timeout = 0.5
plugin.landing_target.publish_interval = 0.1
```

Choose the component ID used by the sensor or routed stream. The component is
required and must be an integer from 1 through 255. `1` selects reports from the
configured flight controller itself. The optional sensor component shares the
existing configured system ID and UDP peer. Only message 149 is admitted from
that component. The provider waits for a fresh, accepted ArduPilot quadrotor
heartbeat at 1/1 before accepting sensor reports. It requests no streams and
sends no MAVLink.

`MAVMOOS_LANDING_TARGET` is schema-1 JSON with `present`, `fresh`, source system
and component, session and observation IDs, original effective/read times and
local age. `data` includes the target number, raw frame, angular offsets in
radians, distance in metres, and the exact remote microsecond field as an
unmapped decimal string. `position_m` is XYZ in the reported frame when
`position_valid` is true. It is null otherwise. The target-type extension is
null when its byte is absent, including MAVLink 1 and MAVLink 2 trailing-zero
trimming. Angular offsets may be null for a valid position report. An angular
report can carry a null distance while retaining valid angles. A malformed
report cannot renew the previous observation. The configured local age limit
defaults to 0.5 seconds, and expiry of the FC heartbeat clears the retained
report until a new heartbeat and sample arrive.

The provider does not transform frames, inject target messages, infer target
acquisition or grant control authority. Tests exercise the real provider with
generated MAVLink 1 and MAVLink 2 reports, wrong source, lifecycle transitions
and reset.
[MAVLink Landing Target Protocol](https://mavlink.io/en/services/landing_target.html)
