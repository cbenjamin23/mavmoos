# Camera status and single-image capture

Enable the optional `camera_status` provider with one separately addressed camera component on the selected flight-controller system. In its default observation mode the app accepts the four reports below from that component. The system 1/component 1 ArduPilot or PX4 heartbeat anchors the session and aircraft identity. Select a separate MAVLink camera component in the range 2–255. The standard first camera component is 100. Autopilot-multiplexed camera device IDs are outside this profile.

```moos
plugins = telemetry,camera_status
bind_address = 127.0.0.1
peer_address = 127.0.0.1
target_system = 1
target_component = 1
camera_component = 100
plugin.camera_status.sample_timeout = 3
plugin.camera_status.publish_interval = 0.1
```

The sample timeout range is 0.25–10 seconds. The publication interval range is 0.02–2 seconds and must be at most the timeout. A status may be event-driven or supplied by an already-configured stream. The module publishes what it receives and can remain absent between source reports.

| MOOS JSON output | MAVLink input | Scope |
| --- | --- | --- |
| `MAVMOOS_CAMERA_INFORMATION` | `CAMERA_INFORMATION` (259) | Bounded vendor and model, firmware bytes, raw capability flags, optics in mm, pixel resolution, and optional lens, gimbal and definition-version fields. `definition_uri_present` is a boolean. |
| `MAVMOOS_CAMERA_SETTINGS` | `CAMERA_SETTINGS` (260) | Raw and named camera mode, nullable zoom and focus in percent. |
| `MAVMOOS_CAMERA_CAPTURE_STATUS` | `CAMERA_CAPTURE_STATUS` (262) | Raw and named image/video state, interval in seconds, nullable recording time, capacity and image count. |
| `MAVMOOS_CAMERA_STORAGE` | `STORAGE_INFORMATION` (261) | Up to eight independently aged storage IDs sorted by wire ID. Each includes raw and named status/type, bounded name, raw usage flags, and capacity or transfer-rate metrics in MiB or MiB/s. |

All records carry schema 1 metadata, including the selected system and camera component, original effective and read times, age and freshness. The camera's boot timestamp remains an unmapped remote value. Stale records retain their timestamps. An FC link reset clears observations, and rebind requires a fresh FC heartbeat. The storage aggregate has `atomic=false`. Its top-level time describes the latest accepted storage report, and each entry has its own metadata. A new distinct storage ID beyond capacity is counted and discarded until reset. An existing ID can still update.

A valid zero zoom differs from an absent or NaN zoom. Zero recording time and an omitted or negative image count map to null where the sender gives no usable measurement. Nonready storage has null capacity fields. Zero firmware, resolution, lens, gimbal and definition-version sentinels are unknown. Text is read within each 32-byte wire field, stops at NUL, and uses JSON escapes for control and non-ASCII bytes. Raw reserved enum values remain distinguishable by their numeric fields and an `UNRECOGNIZED` label. Invalid IDs, infinite reported measurements, out-of-range numeric settings, foreign sources and older observations cannot replace accepted records. [MAVLink camera information and status](https://mavlink.io/en/messages/common.html#CAMERA_INFORMATION).

The passive startup capabilities are `camera.information`, `camera.settings`, `camera.capture_status` and `camera.storage`. Pinned MAVROS `camera` decodes `CAMERA_IMAGE_CAPTURED` (263) and publishes image URI and pose. MAVMOOS uses the same MAVLink event as evidence for its optional capture request. Pinned `cam_imu_sync` decodes `CAMERA_TRIGGER` timestamps. The Camera Protocol v2 report definitions supply the current-state decoders. [MAVROS camera](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros_extras/src/plugins/camera.cpp), [MAVROS camera IMU synchronization](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros_extras/src/plugins/cam_imu_sync.cpp), [MAVLink Camera Protocol v2](https://mavlink.io/en/services/camera.html).

## Single image on the payload bench

Set `plugin.camera_status.capture_enabled = true` to load `camera.capture.single`. Effects require the dedicated, exact owned ArduCopter quad SITL bench on loopback, `simulation_only = true`, `shadow_mode = false`, and a fresh disarmed, on-ground FC state. Select `telemetry,camera_status` with optional `gimbal_status`. The host selects the camera address and uses its standard bounded send path.

Post a string on `MAVMOOS_CAMERA_REQUEST` with the following fields.

```text
schema=1,action=capture,id=shot-001
schema=1,action=cancel,id=shot-001
```

Use a unique ID of 1–48 ASCII letters, digits, underscores, hyphens or periods. One request can be active at a time. Up to 256 capture IDs are retained for the app process, including across reconnect. Cancellation targets the active ID. The send deadline is two seconds from the original MOOS publication, and the evidence deadline is six seconds from that publication. Capture admission requires `CAPTURE_IMAGE` and idle image and video reports at most two seconds old, or younger if the observation timeout is shorter. The camera status boot timestamp must advance for a new report to renew readiness. The selected bench camera uses a zero-based image index, so a fresh idle count of 7 predicts image index 7. If the count field is omitted, including when MAVLink 2 trailing-zero trimming removes a count of zero, the bridge requires another status with a usable count.

The host sends one addressed `MAV_CMD_IMAGE_START_CAPTURE` with an image count of one and an increasing positive command sequence. `MAVMOOS_CAMERA_RESULT` is schema 1 JSON keyed by the request `id`. It carries `status`, `reason`, `system_id`, `component_id`, and nullable `capture_sequence`, `expected_image_index`, `observed_image_index`, and `ack_result`. `QUEUED`, `SENT` and `ACK_OBSERVED` describe progress. `CONFIRMED` requires a fresh successful `CAMERA_IMAGE_CAPTURED` from the selected camera, received after local send with the expected index and an advanced camera boot timestamp. The capture command sequence is absent from both replies, so this correlation relies on the dedicated route and exclusive use of the selected camera for the shot. A missing ACK leaves `ack_result` empty, while a missing image leaves the outcome uncertain. [MAVLink single-image command](https://mavlink.io/en/messages/common.html#MAV_CMD_IMAGE_START_CAPTURE) · [MAVLink image event](https://mavlink.io/en/messages/common.html#CAMERA_IMAGE_CAPTURED)

`REJECTED`, `CANCELLED`, `EXPIRED` and `UNCERTAIN` are terminal. `may_have_captured` records whether local send accepted the frame, including on a late cancellation or ambiguous result. Cancellation can retire queued work and stop evidence collection after handoff. The bridge sends one command and requires a newer idle image count after handoff before admitting another capture. The app process retains its capture sequence across connection resets. Interval and video capture, image replay, media transfer and camera-definition retrieval are separate workflows.

The passive fixture exercises all four reports, foreign source and excluded image event, original timing, independent storage ages, unknown fields, bounded text and session reset. The capture DSO fixture exercises paired evidence, wrong sender and replay, timing and cancellation, and lost bench eligibility. The simulator-side camera fixture writes a deterministic private PGM before reporting a new image. Camera hardware and vendor streaming behavior require separate qualification.
