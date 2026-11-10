# Gimbal status and pointing

`gimbal_status` observes gimbal protocol v2 device information and attitude
status. It shares the host's selected MAVLink route and system identity. An
explicitly configured component supplies gimbal messages, and the selected
flight controller heartbeat establishes and keeps the session.

```text
plugins                       = telemetry,gimbal_status
gimbal_component              = 154
plugin.gimbal_status.sample_timeout = 1
plugin.gimbal_status.publish_interval = 0.1
```

Choose the distinct gimbal component ID from 2 through 255. A component-native
device occupies record slot 0. MAVLink's optional device identifier supplies
slots 1 through 6 when a manager and a device share a component. Records retain
the raw extension or its absence. Passive mode accepts the two device
message IDs from the selected component. The observer leaves connection
selection and frame validation to the host. This capability uses broadcasts
or streams that already reach the configured endpoint. [MAVLink gimbal v2](https://mavlink.io/en/services/gimbal_v2.html)

| MOOS variable | MAVLink message | Contents |
| --- | --- | --- |
| `MAVMOOS_GIMBAL_DEVICE_INFORMATION` | `GIMBAL_DEVICE_INFORMATION` (283) | Identity and logical device ID, raw hardware UID, vendor/model/custom names, packed versions, capability masks, axis limits |
| `MAVMOOS_GIMBAL_DEVICE_ATTITUDE_STATUS` | `GIMBAL_DEVICE_ATTITUDE_STATUS` (285) | Quaternion, yaw-frame and axis flags, angular rates, failure flags, optional delta yaw and rate |

Both variables publish JSON schema 1 with a bounded `devices` array.
Information and attitude each have their own records per device. Every entry
carries original local observation ID, read and effective stamps, age and
session epoch. A valid selected FC heartbeat is required, and heartbeat expiry
or a connection reset clears the caches. Gimbal heartbeats and data cannot
keep the session alive. Repeated MOOS publication retains the original stamp.
Remote boot milliseconds remain unmapped.

Device names include the bounded display text and its original bytes in
hexadecimal, up to the first NUL or the field limit. UID is a decimal string
to preserve its full wire width. A raw zero UID marks unknown identity.
Capability flags preserve both masks when the extension is present, prefer
the extended nonzero mask per the protocol, and expose mismatched lower bits.
Limits retain radians and report unknown or invalid nonfinite values. Reversed
bounds remain visible with an unordered marker.

Attitude retains wire quaternion order `w,x,y,z` and a separate unit-length
diagnostic. Angular rates retain rad/s. Explicit yaw frame flags distinguish
earth north and vehicle heading. The legacy yaw-lock flag supplies that
distinction when neither explicit flag is set. Conflicting flags leave the
yaw frame ambiguous. The delta yaw fields preserve the raw finite number
when carried and expose an interpreted value only with an explicit,
unambiguous yaw frame. Nonfinite reports and absent extensions have separate
states. Target IDs, flag masks and unknown bits stay visible as reported.
The passive publications retain the wire representation. [Device information](https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_INFORMATION) · [Attitude status](https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_ATTITUDE_STATUS)

The pinned MAVROS `gimbal_control` receives the same two device reports
alongside manager and command paths. Its older `mount_control` handles
mount orientation and status. MAVMOOS uses the device observations through
a fixed component and local session age. The optional point workflow combines
manager ownership, device feedback and commands in the same plugin. [Pinned gimbal plugin](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros_extras/src/plugins/gimbal_control.cpp) · [Pinned mount plugin](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros_extras/src/plugins/mount_control.cpp)

The loaded-module fixture covers the component and session boundary,
independent expiry, MAVLink 2 extension truncation, identity, units and
malformed records. The generated loopback harness uses the real MOOS app to
check fixed-component message filtering, FC-heartbeat loss and recovery,
and zero wire egress. Native producer qualification requires a separate
configured gimbal fixture.

## Point and release in SITL

Pointing is off by default. Add the following settings to a dedicated owned
ArduCopter SITL bench with `telemetry,gimbal_status` selected. Supply the
bench's matching PID and loopback endpoints. The host gates each send on
fresh disarmed and on-ground evidence. The bench composition can also include
`camera_status` with a distinct component.

```text
simulation_only                    = true
shadow_mode                        = false
vehicle_profile                    = arducopter
gimbal_component                   = 154
plugin.gimbal_status.point_enabled = true
```

The supported layout is one component-native gimbal with the manager and device
reports from the same selected component. The manager's `gimbal_device_id` and
command parameter 7 equal the configured component ID, which must be at least
7. The device attitude report uses logical device ID 0. The commands address
system 1 and that component. [Gimbal manager and device mapping](https://mavlink.io/en/services/gimbal_v2.html#mapping-from-gimbal-managers-to-gimbal-devices)

Post a versioned string to `MAVMOOS_GIMBAL_REQUEST`. This example requests
30° down and 40° clockwise relative to the vehicle's nose. Pitch accepts
finite values from −75° through 20° and yaw from −90° through 90°. The
cancellation identifies the active point by its original request ID.

```text
schema=1,id=aim-1,action=point,pitch_deg=-30,yaw_deg=40
schema=1,id=cancel-1,action=cancel,operation_id=aim-1
```

Use unique request IDs of 1–40 ASCII letters, digits, dots, underscores or
hyphens. The provider accepts one point operation at a time and retains accepted
IDs for its lifetime. The bounded history holds up to 128 IDs, with an extra
slot reserved for cancellation. Requests must be published after the session
boundary and reach the app within two seconds.

The provider claims an unowned manager using
`MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE`. It sends one
`MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW` only after a newer manager report
names MAVMOOS (245/191) as primary. The command uses explicit vehicle-frame
yaw and leaves angular rates unset. A new valid vehicle-frame attitude
report must place both axes within 2° of the request before the provider
reports `POINTED` and requests release of its primary slot. A newer
manager report confirming the slot is free completes the operation. Command
ACKs are advisory. [Configure command](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE) · [Pitch and yaw command](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW)

The original MOOS publication sets one eight-second deadline for the entire
operation. The claim, point and release phases are capped at two, three and two
seconds respectively. A queued send expires after at most 750 ms. Each phase
requires fresh evidence within one second, and confirmation requires a local
effective observation time after the relevant send. Repeated or older remote
boot counters do not renew evidence. A component restart can require a new host
session. The plugin publishes the selected report as JSON in
`MAVMOOS_GIMBAL_MANAGER_STATUS`, with the original local observation stamps.

`MAVMOOS_GIMBAL_RESULT` reports the request ID, state, reason and observed
ownership. `CONFIRMED` records both a matching pose and release. `CANCEL_ACCEPTED`
acknowledges the separate cancellation ID, while the original ID receives the
terminal `CANCELLED` result. Cancellation retires pending point work and may
release the same observed ownership before the original deadline. A takeover
retires the local operation without issuing a release against the new owner.
`UNCERTAIN` reports that an effect may have occurred but its outcome could
not be confirmed, such as when the release report is missing. The app does
not resend any phase.

The loaded-module test drives valid pointing, stale and foreign evidence,
cancellation, takeover, reset and release timeout. The owned payload fixture
tracks manager ownership and simulated angles independently of the MOOS
result. This qualifies the defined software workflow with a simulated
component. ArduPilot documents that reported gimbal-manager ownership can be
advisory. MAVMOOS uses its host gate and observes the selected manager
throughout the operation. [ArduPilot gimbal interface](https://ardupilot.org/dev/docs/mavlink-gimbal-mount.html#MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE-to-take-control-of-the-gimbal-optional)
