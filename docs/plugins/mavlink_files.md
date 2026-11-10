# MAVLink files

`mavlink_files` provides one read-only workflow on an owned, disarmed ArduCopter SITL instance. An operator lists the simulator's relative `mavmoos-ftp` directory, selects one listed regular-file record and downloads it to a private local directory. A new download takes a new catalog. This provider is separate from [flight logs](flight_logs.md), which use MAVLink's log protocol.

```text
plugins = telemetry,mavlink_files
vehicle_profile = arducopter
simulation_only = true
shadow_mode = false
helm_enabled = false
plugin.mavlink_files.directory = /absolute/private/run/downloads
plugin.mavlink_files.timeout = 45
```

Supply the normal explicit owned-SITL PID and direct loopback addresses and ports. Run the simulator in a fresh private working directory. Prepare `mavmoos-ftp` there with ordinary files and no symlinks. The named simulator must keep its original file bytes stable during a transfer. This fixture policy limits the paths this client requests; it is not a portable filesystem sandbox for another MAVLink server. The app verifies the live simulator working directory and the remote root by owner and inode. It refuses symlinks, unsafe entry types, publicly accessible files and replaced roots before sending; the local owned fixture must stay stable. The host requires exactly `telemetry,mavlink_files` for active operation and current heartbeat and on-ground evidence. The provider cannot compose with vehicle control in this bench. RX_ONLY and shadow accept no operation and transmit nothing. The destination is an existing absolute owner-private directory with no symlink components.

Send the following string mail to `MAVMOOS_FILE_REQUEST` with a fresh ID per request:

```text
schema=1,id=list1,action=list
schema=1,id=get1,action=get,catalog=list1,name=sample.bin
schema=1,id=cancel1,action=cancel,operation_id=get1
```

A `LISTED` result accompanies a `MAVMOOS_FILE_CATALOG` schema 1 record with the list ID and named `file` or `directory` entries, with a size in bytes for files. The catalog lasts at most 30 s and clears at use or loss of ground evidence. The wire listing is bounded to 128 entry indices, including excluded or skipped rows. Only single ASCII basenames starting with a letter or number are selectable; no subdirectory, parent, virtual filesystem or absolute path is sent. Individual files may be empty and are limited to 65,536 bytes. The result does not identify symlinks or establish file content identity.

`MAVMOOS_FILE_RESULT` includes `id`, `action`, `status`, `reason`, `remote_session`, `bytes` and `file`. `QUEUED` means local admission. `COMPLETE` means ordered bytes, expected length, explicit EOF, matching close acknowledgment and a synced local release to `<GET id>.bin`. The app never uses a remote name as a local filename and never overwrites an existing destination. The file is never staged before a matching remote close received within the original deadline. Local disk sync may finish later. An unconfirmed remote close, timeout or cancellation never stages a local file. A failed storage rollback reports possible local residue separately. A file can legitimately change on the server without changing size; the workflow does not request a remote checksum. `MAVMOOS_FILE_TX_RESULT` and `MAVMOOS_FILE_RX_RESULT` expose correlated packet diagnostics without publishing file bytes.

The host serializes one typed intent at a time into standard `FILE_TRANSFER_PROTOCOL` (110). Its allowlist is ListDirectory (3), OpenFileRO (4), ReadFile (5) and TerminateSession (1). It independently checks the route, exact scope, file/offset shape, operation identity, ground evidence and original deadline at actual handoff. The provider reuses the pinned ArduPilot server's client-selected session and binds each reply to sender, target, opcode, sequence, offset where the pinned sender echoes it and a current local receipt. The conservative acquisition age still bounds stale traffic. The 2–60 s operation limit uses the original request timestamp. Retries reuse the logical packet; expiry or cancellation retires data work and permits at most the original bounded same-session cleanup. No resets of other sessions, modifying opcodes, burst, parameter or mission effects. The typed file service and receipt interfaces extend the host without changing the base plugin ABI. If the link or ground evidence is lost, local sending stops and unresolved remote state is explicit.

[MAVLink FTP service](https://mavlink.io/en/services/ftp.html) · [Pinned Copter 4.6.3 GCS FTP sender](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/GCS_MAVLink/GCS_FTP.cpp) · [MAVROS FTP reference](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/ftp.cpp)
