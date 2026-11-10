# External observer: a receive-only MOOS mission

This single headless community loads the separately compiled heartbeat example
into the actual `iMAVLink` app. `pAntler` starts MOOSDB and iMAVLink. A scoped
local fake peer sends standard MAVLink HEARTBEAT packets and independently
records any UDP bytes returned to the primary peer. No helm, GUI, ArduPilot,
flight controller or vehicle is involved.

Build the host and the [standalone example](../../examples/heartbeat_observer/README.md)
against the same development SDK9 checkout. Use fresh directories; earlier
SDK8 binaries are intentionally incompatible. This is a matching C++ build
interface, not a stable binary SDK. Build paths used by pAntler must not contain
whitespace; the launcher diagnoses this rather than emitting an ambiguous Run
entry. Other substitutions reject MOOS syntax injection.

```sh
./missions/external_observer/launch.sh --help
./missions/external_observer/launch.sh \
  --build=/absolute/path/to/new-host-build \
  --module=/absolute/path/to/standalone-example/plugins/mavmoos_example_heartbeat_observer.so \
  --moos-root=/absolute/path/to/built/moos-ivp \
  --mport=19131 --bind_port=14631 --peer_port=14630 --just_make --nogui
```

Inspect `targ_external_observer.moos` in this mission directory. The normal
`ProcessConfig = ANTLER` roster contains only MOOSDB and the exact app path;
the app selects only `example_heartbeat_observer`. Run the same command without
`--just_make` to open one foreground uMAC console. The mission exits after12s,
or when the console exits. Add `--auto` (alias `--xlaunched`) for a foreground
bounded run without uMAC; Ctrl-C/TERM still triggers scoped cleanup. Use
`--work-dir=/fresh/local/directory` to keep generated targets and logs outside
the source tree. Time warp is fixed at1; this community has no shoreside or
pShare ports.

The startup batch is intentionally visible in the generated mission:

```text
plugin.example_heartbeat_observer.sample_timeout = 0.5
plugin.example_heartbeat_observer.publish_interval = 0.1
```

These are module settings, not FC modes or host policy. `--sample_timeout` and
`--publish_interval` override them; the app/module validates the complete
batch. `--defaults` omits both lines, retaining the module's2s/0.25s defaults.
Additional namespaced settings can be supplied with repeatable
`--setting=plugin.OWNER.KEY=VALUE` arguments. This preserves the exact startup
batch, including duplicates, for the app's checked routing; it cannot change
the mission's host policy. Typos or unselected owners fail in the app.
The host remains RX_ONLY with `simulation_only=false`, `shadow_mode=false`,
`helm_enabled=false` and `firmware_requests=false` in every case.

Look for `EXAMPLE_HEARTBEAT_OBSERVATION`, `MAVMOOS_CAPABILITIES`,
`MAVMOOS_STATUS` and `MAVMOOS_RUNTIME_STATUS` in uMAC (or query the generated
mission with ordinary MOOS tools). The fake source initially advertises raw
custom mode42, then pauses. With the configured0.5s timeout the cached sample
becomes stale while retaining its observation ID/effective/read timestamps.
After the host's two-second heartbeat expiry it clears and advances its local
observer epoch. Recovery advertises mode84 in the new epoch. A different
MAVLink source ID from the configured peer and a correct source from a different
UDP peer must not replace or refresh it; the host's rejection counters record
these filters. Those are local age/source diagnostics, with remote source age
explicitly unknown. They do not establish readiness, health or permission.

Each run saves `logs/run-*/result.json`, `pantler.log`, scoped process records,
the exact source packet trace, and `primary-egress.bin`. A clean receive-only
run has an empty independent egress capture, zero app/runtime send counters,
retired peer thread, no owned processes remaining, all three selected ports
clear and no protected input drift. The exact MOOSDB/iMAVLink process roster
must remain live with the same PIDs/arguments during supervision. A successful supervisor receipt proves its
lifecycle/capture; the separate test below also checks the actual MOOS values.

`./clean.sh` removes generated targets and logs only. Stop the foreground
launcher first. It never kills processes. Logs are local demonstration evidence,
not physical transport, supported firmware, vehicle or operating-system timing
qualification.

The [test driver](../../tests/external_observer/README.md) builds a separate
read-only MOOS subscriber and follows this launcher on non-default ports,
including invalid-config and wrong-ABI launches.
