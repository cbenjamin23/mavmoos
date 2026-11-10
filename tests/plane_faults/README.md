# Plane fixture identity and direct-link fault injection

The [Plane 4.6.3 manifest](../../config/firmware/arduplane-4.6.3-plane.json) selects the conventional `plane` simulator and the locally attested Darwin arm64 build. `plane_firmware.py` checks the attestation, native executable, model defaults, protected source files, stock MOOS-IvP revision and stock `pHelmIvP` binary. Run the checks before opening any sockets and repeat them after the run. The independent operator path verifies every required active `PARAM_VALUE` separately from the input-file check. The manifest makes no behavioral qualification claim. [Build evidence](../../docs/implementation/2026-11-06-plane-build/README.md)

```sh
python3 -B tests/plane_faults/plane_firmware.py \
  --plane-build-root "$plane_build_root" --moos-root "$stock_moos_root"
```

The test-only `mavmoos_plane_fault_tap` library interposes `sendto` and `recvmsg` in the bridge process. It accepts exactly one nominated tuple of distinct `127.0.0.1` UDP ports. The app still binds its own socket and checks the live SITL PID and socket before each actual outbound syscall. The simulator owns the direct peer. Install the library only in the scoped app child, keep the mode and capture files in the same private run directory, and use fresh captures for different processes.

| Mode file value | Effect on the nominated tuple |
| --- | --- |
| `pass` | Record completed local TX and RX syscalls. |
| `drop_rx` | Consume inbound packets and present them to the app as truncated. The app only observes an empty kernel queue after a real empty read. Outbound continues. |
| `drop_mode_ack`, `drop_takeoff_ack`, `drop_rtl_ack` | Discard the addressed `COMMAND_ACK` for the selected mode, takeoff or RTL command. Other traffic on the route continues. |
| `drop_tx` | Report local acceptance to the app and discard outbound packets before the kernel send. Inbound continues. |
| `drop_loiter` | Discard only a valid bridge-to-FC `COMMAND_LONG` requesting mode 12. |
| `hold_point` | Preserve one valid bridge-to-FC Guided `DO_REPOSITION` datagram. Suppress additional matching points and continue other traffic. |
| `release_point` | On the next app `sendto` to the same tuple and file descriptor, deliver the preserved bytes once before continuing that send. The hold expires after 60 seconds. |

Set modes by atomic file replacement using `fault_tap.set_mode`. The resulting timestamp bracket and the raw trace use the bridge's steady clock. A missing or malformed mode file, bad trace, or exhausted trace budget blocks further matched egress. The trace distinguishes forwarded, discarded, held and released datagrams. `read_trace` validates the tuple, monotonic times, complete records and held/released byte identity. For native evidence, combine the trace with independent FC telemetry and DataFlash to determine actual mode and motion. A local UDP send alone gives no such observation.

`plane_fault_tap_loopback` verifies actual interposition using two owned loopback receiver sockets and one probe. `plane_firmware_manifest` verifies offline identity and active parameter checks. Neither CTest starts a simulator. Native fault coverage is recorded separately with the associated flight scenario.
