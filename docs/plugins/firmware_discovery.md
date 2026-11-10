# Shared firmware discovery

The host provides firmware discovery for the bound FC regardless of which providers are selected. Use modules built with the [matching host ABI](../development/external-plugins.md). The [firmware profiles](../../config/firmware/README.md) distinguish source identity, native build identity and demonstrated behavior.

Firmware discovery publishes shared host diagnostics. Providers use the selected profile and its evidence for readiness and qualification.

`MAVMOOS_FIRMWARE_STATUS` is JSON schema 1. It carries the target system/component, discovery epoch and observation ID, original effective/read times, age and age basis, presence/freshness/state, and reported AUTOPILOT_VERSION values. Version zero stays unknown. Capability bits remain exact hexadecimal/decimal strings; large UIDs do not pass through imprecise floating-point values. ArduPilot's eight custom-version bytes may contain an ASCII source prefix or opaque bytes; they are not a full Git revision.

The initial state waits for a correctly bound post-boundary heartbeat. A received version sample is `OBSERVED` through its thirty-second observation-age limit and then `STALE`. Stale identity remains useful as labelled historical session evidence; status publication does not renew its original times or trigger periodic rediscovery. Disconnect/reset clears the observation and retires queued discovery attempts. Invalid clock handling and later read-only recovery are separately tested; recovery never grants navigation.

The app's `firmware_requests = true|false` setting defaults to true, but effectful discovery additionally requires valid configuration, explicitly enabled owned simulation control, non-shadow operation and a fresh accepted FC heartbeat. Receive-only and shadow transmit zero MAVLink even with the default setting. The embedded Runtime's active request switch starts disabled until its owner sets it. Passive observation remains available regardless of the switch.

Active discovery uses ordinary MAVLink COMMAND_LONG / MAV_CMD_REQUEST_MESSAGE (512), requesting AUTOPILOT_VERSION (148). It has one original ten-second validity window, at most three locally accepted attempts, two-second accepted-response spacing and a bounded 0.2-second retry for local rejection. A COMMAND_ACK for 512 is not a version response. The final sender checks the original expiry, current epoch, heartbeat/configuration and exact owned native endpoint immediately before attempting UDP output.

Discovery and telemetry setup alternate within their existing shared protocol lane; mission and parameter lanes still share a single protocol-packet budget below control. `MAVMOOS_FIRMWARE_REQUEST_RESULT` reports attempt/epoch/original expiry, local acceptance and clock/current-work status. Local acceptance does not prove remote receipt or response causality.

Every status explicitly says `read_only: true`, `authoritative_for_control: false` and `qualification: not_evaluated`. The host does not derive permission, readiness, feature support or a qualified firmware range from a reported version or capability mask. Native fixtures separately verify an immutable source manifest and platform-specific build attestation; new broader Copter scenarios also require an early matching original version advertisement.
