# Mission-data provider

The startup-loaded `missions` module transfers stored mission data over the host's shared MAVLink connection. It owns download, full upload, verified clear, cancellation and observed progress. It has no flight-control backend and never starts a mission, arms a vehicle, changes a flight mode, acquires helm permission or opens another FC connection. Stock pHelmIvP is unchanged.

The provider handles ArduPilot flight-plan storage (type 0), typed fence and rally inspection, and an opt-in Copter fence editing workflow (type 1). A legacy `MISSION_REQUEST` from pinned ArduPilot can advance a flight-plan upload. MAVMOOS still replies with `MISSION_ITEM_INT`. The separate vehicle control provider owns execution.

## Loading and configuration

Add `missions` to `plugins` in the existing `ProcessConfig = iMAVLink` block and point `plugin_directory` at the built modules. Its exclusive resource is `protocol.missions`; only one module may own that protocol family. Loading the module advertises configured capabilities, without granting mutation permission. The default app remains receive-only. Effectful native tests additionally require the host's exact owned simulator PID, loopback endpoint and `simulation_only` guard.

| Configuration | Default | Accepted range / meaning |
|---|---:|---|
| `mission_timeout` | 60 s | 1–120 s, original whole-operation duration, including upload/clear readback |
| `mission_response_timeout` | 1.5 s | 0.25–1.5 s after an accepted local send, bounded by the original deadline |
| `plugin.missions.fence_edit` | `false` | Exact `true` opts in to typed save, replacement and restore for a disarmed on-ground ArduCopter quad in owned SITL |

The following limits are fixed for this initial profile: one operation, 4,096 items, 1 MiB request text, five retries per protocol phase, 32 retries across the whole operation, and one second to receive the exact local-send result. There is no independent `mission_phase_timeout`, `mission_max_items` or `mission_retries` setting. The host additionally caps the shared mission queue at four entries and alternates one protocol packet per iteration with parameter traffic when both families are waiting; navigation/control has priority. Maximum table size is a memory bound, not a throughput or completion guarantee.

Read-only transfer needs a fresh FC heartbeat. Upload and clear additionally need disarmed, idle state with no navigation permission, handoff resumption window or pending native command. Copter also needs fresh on-ground evidence. Rover does not inherit a Copter landed-state requirement. These checks occur again at final sending in the host. An accepted mutation holds a host lease through readback, blocking acquisition/commands until terminal outcome or original expiry. External arming stops further mutation; already permitted readback can still resolve observations.

## Request interface

Publish string requests to `MAVMOOS_MISSION_REQUEST`:

```text
id=read-1,action=download
id=clear-1,action=clear,timeout=20
id=upload-1,action=upload,timeout=30,items=0:16:0:0:1:0:0:0:0:423584560:-710875890:10|1:16:3:0:1:0:0:0:0:423584660:-710875890:5
id=upload-1,action=cancel
```

### Inspecting flight, fence and rally plans

Use the versioned read interface to inspect each separately stored plan. For example, publish these one at a time and wait for each terminal result before the next request.

```text
schema=1,id=flight-read-1,action=inspect,type=flight
schema=1,id=fence-read-1,action=inspect,type=fence,timeout=20
schema=1,id=rally-read-1,action=inspect,type=rally
schema=1,id=rally-read-1,action=cancel,type=rally
```

The three type names are case-sensitive and map to MAVLink values 0, 1 and 2. A cancellation uses the same schema, type and operation ID as the active inspection. Inspection and legacy transfers share one protocol slot and the process-lifetime ID history. Flight upload and clear remain on the legacy interface. Typed fence replacement and restore have a separate opt-in. The host binds each operation to its original type and expiry and independently checks permission to mutate.

`MAVMOOS_MISSION_INSPECTION_RESULT` is a schema-1 key/value record with the request ID, plan type, numeric `mission_type`, session epoch, status, reason, original expiry and progress. Pair `COMPLETED` with the same ID and epoch on the applicable schema-1 JSON publication.

| Requested type | Snapshot |
| --- | --- |
| `flight` | `MAVMOOS_MISSION_INSPECTION_FLIGHT_SNAPSHOT` |
| `fence` | `MAVMOOS_MISSION_INSPECTION_FENCE_SNAPSHOT` |
| `rally` | `MAVMOOS_MISSION_INSPECTION_RALLY_SNAPSHOT` |

A complete snapshot holds the ordered raw `MISSION_ITEM_INT` rows, the observed plan ID, local collection time, unknown source age and `atomic:false`. Fence rows accept the pinned ArduPilot global frame, polygon vertices, circle radii and return points. Rally rows preserve the FC’s global, above-home or above-terrain altitude frame. Coordinates are signed degrees times 10⁷ in the supported global frames. Flight inspection uses the legacy finite raw row shape. Collection validates each row independently and reports the FC’s storage representation, including dynamic flight HOME/current values. Execution proof comes from the existing flight upload with verified readback. Both flight read interfaces invalidate retained proof when complete observed executable rows contradict it. Matching observations preserve the original proof revision and time. Fence and rally observations have independent storage scope.

Each versioned operation uses MAVLink 2 `MISSION_REQUEST_LIST`, numbered `MISSION_REQUEST_INT` and a final accepted `MISSION_ACK` of the selected type. Replies need the selected FC endpoint and source, the exact local MAVLink recipient, the selected type and a live post-send phase. A matching zero count followed by local ACK acceptance completes an **empty** plan. An explicit matching `MAV_MISSION_UNSUPPORTED` reply to the list request produces `UNSUPPORTED`. Silence, old framing or unrelated traffic can leave the result timed out with support unknown. Incomplete, unsupported and cancelled operations preserve the last complete publication for that type. A session reset publishes `invalidated` for all three snapshots and advances the epoch.

The protocol lacks a remote operation nonce, so a delayed correctly addressed reply within a new valid phase is ambiguous. The type binding and local send receipt improve correlation within that limit. Use a unique MAVLink identity and avoid parallel GCS writes during a pull. Local cancellation retires waiting sends and releases the slot. A packet already accepted by the local OS can still arrive at the FC.

This workflow follows the shared pull logic behind MAVROS waypoint, geofence and rallypoint, including independent type filtering, indexed request and matching final ACK. MAVMOOS uses one host-owned operation across plan types. The pinned Copter sender returns a requester-addressed typed count or an unsupported ACK and generates typed `MISSION_ITEM_INT` rows. [MAVROS pull protocol](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/mission_protocol_base.cpp#L198-L234) · [MAVLink mission protocol](https://mavlink.io/en/services/mission.html#plan-types) · [ArduPilot request dispatcher](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/GCS_MAVLink/GCS_Common.cpp#L606-L623) · [ArduPilot typed row sender](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/GCS_MAVLink/MissionItemProtocol.cpp#L142-L174)

### Saving, replacing and restoring a Copter fence

The initial bench targets ArduCopter 4.6.3 quad SITL. It uses `FENCE_ENABLE=0` and `FENCE_AUTOENABLE=0` as fixture preconditions and independently reads both values before and after the workflow. The module edits stored geometry only. Fence enablement and flight-mode policy belong to the FC. The runtime checks the owned SITL route, Copter identity, fresh disarmed ground state and authority exclusions at admission and again immediately before send.

Use a new ID for each operation except cancellation. Wait for its terminal result before sending the next request.

```text
schema=1,id=save-1,action=save,type=fence
schema=1,id=replace-1,action=replace,type=fence,backup=save-1,items=423580560:-710879890|423588560:-710879890|423584560:-710871890
schema=1,id=replace-1,action=cancel,type=fence
schema=1,id=restore-1,action=restore,type=fence,backup=save-1
```

`save` obtains every typed row and publishes the raw snapshot. The restorable baseline may be empty or one ordered, strictly convex inclusion polygon of 3–16 vertices. Supply each replacement vertex as integer `latitude_e7:longitude_e7`, without repeating the first vertex. Latitude lies within ±85° and longitude within ±180°. Each coordinate axis spans at most 0.018°. The provider emits canonical global integer inclusion vertices with zero altitude, flags and unused parameters. Inspect can still observe circles, return points, exclusion polygons and multiple shapes. Rally editing stays outside this profile.

`replace` independently downloads the current fence again and compares every raw row with the saved baseline before starting a typed full upload. If any row changed, it reports `fence_changed_since_backup` without submitting a mutation. Pinned Copter requests upload items with message 40 `MISSION_REQUEST` in MAVLink 2, carrying type 1. MAVMOOS also accepts the typed integer request and always replies with typed `MISSION_ITEM_INT`. After the FC accepts the upload, a second complete download must match every proposed stored field before the operation completes. Fence snapshots mark this verification as `exact_fence_match`. These comparisons establish what the selected FC returned across a sequential pull. Operate with one writer so no other GCS can change storage between the comparison and upload.

`restore` is an explicit new request referencing the original save ID. It uploads the saved rows, or issues a type-1 clear when the original was empty, and independently verifies the final contents. Restoration uses the same ground and ownership gates. Fence mutation neither grants nor invalidates a flight-plan execution proof.

The typed result adds `backup_id` and `restore_required`. Once a write might have reached the FC, including after a verified replacement, the baseline is retained and new save/replace requests are blocked until an explicit verified restore. If cancellation or timeout follows a possible write, `mutation_may_have_applied=true` exposes the uncertainty. A failed restore leaves that obligation outstanding. Wait for any FC-side transfer to settle, inspect the current fence, and send a fresh restore ID when appropriate. Local cancellation cannot retract a sent packet or terminate a remote transfer.

Component and pinned native fence evidence includes both empty and nonempty originals, cancellation during partial upload, and verified restoration.

The save is a single in-memory baseline tied to the module connection epoch. Keep a copy of the published snapshot. A connection reset invalidates the in-memory baseline and current snapshot. After a reset, use a new inspection and reconcile the observed storage against the retained snapshot. Automatic replay and durable rollback are outside this bench scope.

The design reuses MAVROS's type-specific geofence transfer and the shared MAVLink mission exchange. MAVMOOS adds a required saved baseline, a pre-write comparison and independent typed verification of both replacement and restore. [MAVROS geofence plugin](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/geofence.cpp) · [MAVLink mission types](https://mavlink.io/en/services/mission.html#plan-types) · [Pinned ArduPilot fence storage](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/GCS_MAVLink/MissionItemProtocol_Fence.cpp)

### Legacy upload, readback and clear

IDs contain 1–48 ASCII letters, digits, `_`, `-` or `.`. Keys are exact and unique; unknown actions/keys, extra delimiters, numeric mail and unsupported rows reject explicitly. Cancellation uses the active operation's original ID and has no timeout/items field. Other requests need a new ID. A duplicate ID is reported without replaying effects. The module retains at most 128 admitted IDs for its process lifetime, including across reconnect/reset; once full it rejects new operations rather than evicting replay protection. A future durable request-history policy is still open.

Mail must be freshly published after the current module connection boundary. Publication/receipt ages are bounded at two seconds with at most 0.1 seconds future skew. The operation expires at original publication time plus its duration; retries, duplicate messages, queueing, reconnects and readback cannot renew that deadline. A late terminal result does not authorize another mutation.

Each `items` row has twelve colon-separated fields, and rows are separated by `|`:

```text
seq:command:frame:current:autocontinue:param1:param2:param3:param4:x:y:z
```

`x`/`y` are signed integer latitude/longitude in degrees times 10^7 for the supported global frames; `z` is metres in the specified frame. Every row retains command, frame and wire flags. All floats must be finite. Sequence starts at zero and is contiguous. There is no implicit insertion/removal of a HOME row.

| Row / command | Frame | Supported payload |
|---|---|---|
| Row 0: `NAV_WAYPOINT` (16), explicit HOME placeholder | `GLOBAL` (0) | Zero parameters, finite bounded coordinates/altitude |
| Later `NAV_WAYPOINT` (16) | `GLOBAL` (0) or `GLOBAL_RELATIVE_ALT` (3) | Integer hold time `param1` 0–600 s; other parameters zero |
| Later `NAV_TAKEOFF` (22), `NAV_LAND` (21) | `GLOBAL` (0) or `GLOBAL_RELATIVE_ALT` (3) | All four parameters zero |
| Later `NAV_RETURN_TO_LAUNCH` (20) | `MISSION` (2) | All parameters and x/y/z zero |

Upload rows require `autocontinue=1` and ordinary `current=0` or `1`. Pinned ArduPilot interprets `current=2`/`3` as immediate Guided/change-altitude requests, so both module and host reject those values. Latitude/longitude must be within ±90°/±180°; the storage profile's altitude envelope is −1,000 to 20,000 metres. Those bounds are data-validation limits, not a flight envelope or vehicle qualification. An empty upload is rejected; use explicit clear.

`tests/missions/mission_request.py` prints requests from a QGroundControl WPL 110 file without posting to MOOS or contacting the FC:

```sh
python3 tests/missions/mission_request.py --id upload-2 --action upload --timeout 30 --waypoints plan.waypoints
```

It requires the same explicit HOME and supported subset, preserves ordinary current flags, rounds coordinates to 10^-7 degrees using half-away-from-zero, and normalizes deprecated global frame numbers 5/6 to 0/3. The provider validates the generated string independently.

## Results and observations

`MAVMOOS_MISSION_RESULT` is a schema-1 key/value string containing ID, action, status, reason, received/expected item counts, sequence, retry/phase counts, original expiry, and:

- `mutation_may_have_applied`: true once a mutation was accepted by the local OS, or a submitted mutation has no definitive send receipt when abandoned. A known admission/send rejection alone leaves it false.
- `remote_accepted`: an accepted final mission ACK was observed after the relevant send.
- `readback_confirmed`: an independently requested readback matched the supported semantic profile, or verified clear produced an empty list.

Intermediate states include `ADMITTED`, `SUBMITTED`, `SENT`, `COUNT_RECEIVED`, `ITEM_RECEIVED`, `RETRY_DUE` and `ACKNOWLEDGED`. Terminal outcomes are `COMPLETED`, `REJECTED`, `FAILED`, `TIMED_OUT`, `CANCELLED` or `INCOMPLETE`; `DUPLICATE` reports a reused ID. `SENT` means local OS acceptance, not remote acceptance. A final ACK alone does not complete upload or clear.

The host also publishes `MAVMOOS_MISSION_TX_RESULT` with ID, local result, MAVLink message ID, `mission_type` and `at`, a full-precision host-monotonic post-attempt result timestamp. Provider results carry `at` on the same host clock. These diagnostic times let an auditor order accepted sends against local retirement without assuming MOOS delivers different variables in producer order. They are not wall-clock/FC timestamps or exact kernel-send timestamps, and do not resolve remote packet transit or processing.

`MAVMOOS_MISSION_SNAPSHOT` is schema-1 JSON containing ordered raw mission rows, collection time, FC opaque plan ID, `atomic:false`, `source_age:"unknown"` and verification metadata. `status:"complete"` means all requested rows were collected and our final ACK was locally accepted. Always check `verification` and the matching terminal result: a complete collection can reveal a different uploaded mission and produce `FAILED`. Partial/reset outcomes do not masquerade as a new complete snapshot; reset publishes explicit invalidation.

An upload completes only after accepted remote ACK and a full independent readback under the same original operation deadline. The comparison is exact for sequence, command, coordinates and autocontinue, with parameter tolerance 10^-5 and altitude tolerance 0.010001 m for ArduPilot centimetre storage. Frames and parameters also match, except for two exact pinned-ArduPilot storage rules: a non-positional RTL uploaded with MISSION frame 2 may read back with GLOBAL frame 0; a LAND uploaded with default parameter4=0 may read back with parameter4=+1. The latter is ArduPilot's stored default direction representation, not a request for configurable landing yaw. No other command, frame or direction value receives either exception. `supported_storage_normalizations` describes these rules while raw uploaded/observed rows remain unchanged. ArduPilot reads row zero from current HOME, and current-item flags are dynamic; those fields are exposed but explicitly excluded from identity comparison. Therefore completion means a semantic match for this profile, not byte identity or proof that HOME changed. A clear completes only after accepted ACK plus independently observed zero count.

`MAVMOOS_MISSION_PROGRESS` reports raw `MISSION_CURRENT`/`MISSION_ITEM_REACHED` observations with the conservative local acquisition time, actual read time and unknown source age. The provider requires a fresh post-session heartbeat and a fresh, ordered post-session acquisition. `MISSION_CURRENT.total=0` means unsupported or unknown. UINT16_MAX means no mission, and a supported total excludes HOME. Zero mission-state or mode extension fields remain unknown. The publication is diagnostic.

## Failure and concurrency limits

Retries require the exact local-send callback and wait for a fresh reply after that send. Sender, target, mission type and expected sequence are checked. Replies before the matching send, unrelated targets, reordered future items, stale receipts and callbacks from old/reset phases cannot advance the operation. A changed item, count, or conflicting known plan ID ends collection. The recoverable ArduPilot `INVALID_SEQUENCE` case is ignored during upload without extending its deadline.

ArduPilot may truncate/change mission storage before accepting the whole upload. Failure, cancellation, link loss, missing final ACK or reset can therefore leave a changed or partial mission. MAVMOOS retires waiting local work and its lease; it cannot roll back FC storage, recall a transmitted packet or cancel ArduPilot's remote transfer. Inspect the result uncertainty and perform a new explicit download before deciding what to do next. Never automatically replay a failed upload.

The wire protocol carries no MAVMOOS operation/phase ID or source timestamp. Exact local callbacks and one active host operation improve local correlation, but cannot prove that a delayed correctly addressed FC reply belongs to this operation, or that another GCS left storage unchanged during collection. The opaque ID is observational, and snapshots are explicitly non-atomic. Other clients must not share MAVMOOS's 245/191 identity or concurrently mutate the same mission. Stronger remote versioning, broad command support, authenticated field links and durable replay policy remain future work.

Private fake-I/O and real-module tests are archived in the implementation ledger. A separate owned, disarmed native-Copter bench passed the HOME/two-WAYPOINT upload, readback, download and clear workflow. A separate final native-Copter flight also completed a HOME-only mission download and ordinary parameter read while stock pHelmIvP retained navigation continuously.

The overnight cancellation record adds explicit upload cancellation and owned bridge pause/reset. Both left actual partial storage, rejected old-ID replay and initially received remote DENIED on a new download while the FC's old receive protocol was still active. After a bounded fixture wait beyond this pinned FC's receive timeout, a new-ID download recovered the partial prefix and a fresh clear verified an empty plan. The wait is a test fixture policy, not an automatic retry/settlement service or general FC timing guarantee. No mission execution or arming occurred. Arbitrary item types, other transfer faults, airborne mutation/execution and real vehicles remain unqualified.
