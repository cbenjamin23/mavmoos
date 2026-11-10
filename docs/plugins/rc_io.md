# RC input and servo-output observations

`rc_io` is an optional, read-only provider grouping reported RC input and commanded servo output. It uses the shared FC connection and vehicle session. It has no override, servo command, motion backend, parameter access, NAV projection, readiness gate or inferred control authority. Build it with the [matching host ABI](../development/external-plugins.md).

## Configuration and descriptor

```text
plugins                = telemetry,rc_io
plugin_directory       = /path/to/built/plugins
rc_io_timeout          = 1
rc_io_publish_interval = 0.1
```

These are ordinary fields inside `ProcessConfig = iMAVLink`. Receive-only remains the app default and sends no MAVLink. The provider declares `RC_CHANNELS`65 at10Hz and `SERVO_OUTPUT_RAW`36 at5Hz to the host's observation-stream mechanism; it never requests a stream itself. Declared rates express configured demand, not delivered throughput or permission to transmit. An absent stream remains absent in receive-only.

`rc_io_timeout` accepts0.25–10 seconds; publication interval accepts0.02–2 seconds. Defaults are1s and0.1s. Descriptor profile `ardupilot_rc_observations` declares observation capabilities `rc.input` and `rc.servo_output`, resource `observation.rc_io`, no request owners, mail subscriptions or control backend, and compatibility with an observation-only streamed-parameter bench.

## Records

| Variable | Schema-1 content |
|---|---|
| `MAVMOOS_RC_INPUT` | Latest `RC_CHANNELS` sample, including all18 raw slots and the original reported count |
| `MAVMOOS_SERVO_OUTPUT` | `present`, `atomic:false`, `cache_capacity:8`, and `ports`: independently acquired records sorted by numeric port |
| `MAVMOOS_RC_IO_STATUS` | Binding/reset/rejection diagnostics plus the input record and complete bounded servo aggregate; `read_only:true`, `authoritative_for_control:false`, `atomic:false` |

Each input/per-port sample has `schema`, `session_epoch`, `family`, `present`, `encoding_valid`, `fresh`, `effective_at`, `read_at`, `age_s`, `age_basis`, `age_is_upper_bound`, `observation_id` and `data`. Absent input has null data/stamps; absent servo output has an empty port list. A stale retained port does not vanish merely because another port updates. Once8 distinct ports are cached, additional ports are rejected and counted until session reset; existing ports still update. This is a bounded cache, not an inventory of every physical output. Aggregate records are not atomic generations.

`encoding_valid` means a decoded, shape-accepted unsigned wire record. It does **not** prove available channels, valid pulse range, receiver health or physical output. Examine count and per-quantity known/unknown fields. `fresh` independently means recent local acquisition under a live bound session. Republication preserves original stamps and observation ID. Per-port and input ages remain independent. `QUEUE_EMPTY_FLOOR` retains the conservative acquisition floor and upper-bound local age; `READ_ACQUISITION` requires equal effective/read times and is not labelled a queue bound. Neither measures upstream generation age: `source_age:"unknown"`, `source_quality:"unassessed"`.

## RC input semantics

Data retains `time_boot_ms` as an unmapped uint32 boot clock, `reported_channel_count`, `visible_channel_count=min(count,18)`, `channels_beyond_message`, `wire_version`, `rssi` and18 `channels`. Each channel preserves its1-based `slot`, integer `raw_us`, `reported_by_count` and `pwm_us:{known,value,unit,reason}`. Only slots within the reported count have a known reported PWM quantity; outside-count raw payload fields remain visible. Count0 explicitly supplies no available channels. Counts19–255 are preserved rather than silently clamped: only18 values are represented.

The pinned `RC_CHANNELS` definition does not define0 or65535 as channel sentinels. We preserve both and do not invent one; `pulse_validity:"unassessed"` means a known wire quantity is not a certified useful RC pulse. RSSI raw255 means invalid/unknown with null quantity;0–254 remain in **device-dependent units**, not a normalized percentage. Pinned ArduPilot scales its configured RSSI source by254, but the generic field does not establish that calibration for other senders.

`channel_roles:"unmapped"` is deliberate. No universal channel-to-roll/throttle/mode mapping is inferred from slot number. ArduPilot sends `radio_in`, which its channel update can obtain from an active override or receiver input. This is an FC-reported input observation, not proof of a physical transmitter, pilot action, ownership or failsafe state. Legacy `RC_CHANNELS_RAW`35 is outside this provider's profile and ignored.

## Servo output semantics

Data retains `time_usec` as `remote_clock:"unmapped_uint32_us_no_epoch_inference"`, numeric `port`, `wire_version`, `extension_bytes_observed`, and16 `channels`. A channel's `slot` is local to that message's port. No universal `port*8`/`port*16` actuator index is created. The generic message comment describes8 outputs per port, while the pinned ArduPilot sender emits16-slot groups for ports0 and1; retaining both coordinates avoids promising the wrong physical map. Port labels are not assumed to mean MAIN/AUX on every board.

The first8 PWM fields exist in MAVLink1 and2. MAVLink1 extension slots9–16 have `raw_us:null`, unknown quantity and reason `mavlink1_extension_absent`. In MAVLink2 all16 decoded raw values are retained, including protocol zero-fill of truncated trailing bytes. A nonzero extension is a reported number; a zero extension cannot distinguish unsupported, omitted, disabled or literal zero. `extension_bytes_observed` means at least one extension byte appeared, not that all8 extension quantities are supported. It is false for a valid MAVLink2 packet truncated to its base fields.

Zero output quantities remain unknown with the raw zero preserved and an explicit ambiguity reason. The pinned ArduPilot sender replaces HAL output65535 with0, so zero cannot establish an actual physical0µs pulse. A received65535 is preserved without inventing an undocumented generic sentinel. Nonzero `pwm_us` is the reported unsigned number, with `pulse_validity:"unassessed"`; no actuator calibration, enabledness, useful pulse range or function is inferred. `output_kind:"fc_command_observation"` and `physical_feedback:false` distinguish these reports from measured servo position, motor rotation or physical actuator response.

The uint32 µs clock wraps about every72 minutes. The generic epoch/magnitude comment cannot establish an epoch from this32-bit field; the pinned ArduPilot sender uses boot micros. Neither remote clock controls local expiry or session identity. New local acquisitions with repeated/wrapped remote times are new observations without proving a changed remote measurement.

## Boundaries and references

The provider waits for a post-construction/post-reset host-accepted ArduPilot heartbeat matching shared vehicle type, then binds exact system/component IDs. The host owns peer, target, CRC, signature and duplicate-frame filtering. The provider independently rejects impossible metadata, foreign IDs, invalid wire shape and backwards per-family/per-port acquisition times. Distinct frames in one datagram may share both original stamps and still update their cache; timestamp equality is not frame identity.

Heartbeat expiry after2s, explicit reset and latched clock fault clear input and every port. A new heartbeat cannot revive old observations. Clock faults require an explicit reset with finite nonnegative owner time at least as high as its last reliable value; a reset while the clock is still behind remains latched, and merely catching up does not recover it. Boundaries retain a conservative post-reset acquisition floor. This observer adds no authority or readiness decision, even when fresh PWM resembles a mode-switch or pilot-takeover input. Stock pHelmIvP remains unchanged. The independent-review regression records this tightened clock boundary.

MAVROS's [pinned rc_io plugin](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/rc_io.cpp) informs input/output grouping. Its override subscriber, RAW35 fallback, ROS time synchronization, count clamping and flattened output vector are not copied. Pinned [ArduPilot sender](https://github.com/ArduPilot/ardupilot/blob/04cf6486e8d42ce138de816e8466244346ed77cb/libraries/GCS_MAVLink/GCS_Common.cpp), [RC input selection](https://github.com/ArduPilot/ardupilot/blob/04cf6486e8d42ce138de816e8466244346ed77cb/libraries/RC_Channel/RC_Channel.cpp), and generated [RC_CHANNELS](https://github.com/mavlink/c_library_v2/blob/a2d0ecf011a4aaa7840480e43f4e38d8c9e79d29/common/mavlink_msg_rc_channels.h)/[SERVO_OUTPUT_RAW](https://github.com/mavlink/c_library_v2/blob/a2d0ecf011a4aaa7840480e43f4e38d8c9e79d29/common/mavlink_msg_servo_output_raw.h) establish the retained semantics. The implementation archive separates software and native simulation evidence.
