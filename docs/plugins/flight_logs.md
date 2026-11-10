# Flight logs

`flight_logs` retrieves one closed DataFlash file from a disarmed, on-ground Copter SITL and saves it in a configured private directory. It owns the catalog and download workflow. The shared iMAVLink host owns the selected peer, send deadlines, cancellation and the one operation lease. The provider can request LIST, DATA and END through the typed host service. That service has no erase operation.

The [MAVROS log_transfer provider](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros_extras/src/plugins/log_transfer.cpp) exposes raw list/data/end/erase services and publishes entries and chunks. MAVMOOS adds catalog selection, byte assembly and local file release. The wire messages follow the [MAVLink log definitions](https://mavlink.io/en/messages/common.html#LOG_REQUEST_LIST).

## Setup

Use an explicitly owned, verified Copter SITL loopback setup. Select the provider and point it at an existing directory owned by the iMAVLink process user with group and other permissions clear. All path components must be real directories. The startup configuration rejects relative paths, symbolic links, `.` and `..`. The app opens and retains the directory identity. Use the canonical absolute path from `pwd -P` when a parent path has a symbolic alias.

```text
plugins = flight_logs
plugin.flight_logs.directory = /absolute/path/to/private-downloads
plugin.flight_logs.timeout = 60
plugin.flight_logs.max_bytes = 4194304
```

The timeout is an integer from 2 to 120 seconds, measured from the original MOOS request. Each request attempt has a shorter original cutoff. The configured file limit ranges from 1 byte through 4 MiB. One transfer runs at a time. The operator manages storage for completed downloads. Receive-only and shadow remain zero-output. The host requires fresh disarmed and on-ground evidence and excludes conflicting control, mission mutation and streamed parameter operations during the lease.

## Operator workflow

Post a string to `MAVMOOS_FLIGHT_LOG_REQUEST` for each step.

| Operation | Example value |
| --- | --- |
| List onboard logs | `schema=1,id=postflight-list,action=list` |
| Retrieve log 1 from that list | `schema=1,id=postflight-copy,action=download,catalog=postflight-list,log_id=1` |
| Cancel a pending copy | `schema=1,id=stop-copy,action=cancel,target=postflight-copy` |

Use a fresh request ID beginning with an ASCII letter or digit, with at most 48 letters, digits, periods, hyphens or underscores. A matching cancel can retire a pending release while its END is still queued. Use a new list after connection reset, a ground-state interruption, catalog expiry or a reported change. A catalog is usable for 60 seconds after its last accepted entry. List and select a closed file. The MAVLink catalog does not report whether the recorder still holds it open.

- `MAVMOOS_FLIGHT_LOG_CATALOG` publishes one complete catalog with its ID, observed source, local acquisition time and a row per log ID. Each row carries reported bytes and FC UTC seconds when available. A successful empty list has count zero. Catalog invalidation has a separate status.
- `MAVMOOS_FLIGHT_LOG_RESULT` reports each operation's state, immutable expiry, retries and cleanup outcome. A completed download includes a generated basename, released byte count and FNV-1a 64-bit checksum of those bytes. The checksum is local bookkeeping. The protocol supplies neither a trusted FC digest nor a reply transaction ID.
- `MAVMOOS_FLIGHT_LOG_TX_RESULT` reports host handoff receipts. `LOCAL_ACCEPTED` describes the local socket. END has no FC acknowledgement, and `local_end_accepted_unacknowledged` records that limit explicitly.

Catalogs are capped at 128 rows. Downloads recheck the selected ID, reported length, date and catalog count before and after the data phase. A data window asks for at most 32 90-byte chunks. Out-of-order chunks can fill requested offsets, identical duplicates do not count twice, and a conflicting or short chunk fails the transfer. Retries request missing ranges, with at most three retries without progress and 32 total. A separate one-byte request at the advertised boundary must produce a zero-byte reply. A zero-byte reply before the boundary is incomplete. MAVLink 2 may omit trailing zero payload bytes, which the generated decoder restores. Replies bind to the exact selected 1/1 source, a current accepted request window and the original deadline. A delayed response matching all current fields remains ambiguous on this protocol.

The file is staged only after full coverage, the boundary and metadata checks, and a locally accepted END. It is released as `log-<id>-<request>.bin` with mode restricted to the owner. Atomic link creation preserves an existing destination. A partial wire transfer never creates an output. Local storage errors fail the release. A process crash during local staging can leave a hidden `.partial` entry for the operator to inspect. The completed receipt reports staging removal and directory sync separately.

## Firmware scope and verification

The pinned [ArduPilot MAVLink handler](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/AP_Logger/AP_Logger_MAVLinkLogTransfer.cpp) uses one-based IDs, reports a zero row for an empty catalog and serializes its logger transfer. Its [file backend](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/AP_Logger/AP_Logger_File.cpp#L605-L669) reports file size and time, can stop the active recorder when opening a download, and closes its read descriptor on END. The [logging guard](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/AP_Logger/AP_Logger_Backend.cpp#L485-L495) can suppress disarmed logging during a recent download. Cancel, timeout and validation failure make a bounded END attempt when the same disarmed session still permits it. Lease expiry allows up to two seconds for cleanup. Session loss or rejected END leaves remote state unknown. Keep other log clients out of this workflow because FC transfer state is shared.

The [Runtime fault suite](../../tests/flight_logs/FlightLogProtocolTests.cpp) loads the actual module, uses generated messages and fake I/O, and covers loss, reorder, cancellation, ground/session loss and filesystem restrictions. The [app fixture](../../tests/flight_logs/run.py) checks zero output with a generated peer and runs the native workflow on a verified quad profile. The native case records a real DataFlash log in a first owned disarmed boot. A second owned disarmed boot keeps that closed file stable and serves the actual iMAVLink operator workflow. The independent comparison checks file bytes and SHA-256. A fresh catalog after END demonstrates continued protocol availability. The fixture's disabled disarmed recording on the second boot leaves actual logging resumption unmeasured. Stock pHelmIvP is unchanged and the workflow is qualified separately from motion control.
