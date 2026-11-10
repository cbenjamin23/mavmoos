# Native PX4 passive observer witness

This local test boots a pinned official ARM64 PX4 SITL image with its stock
software-in-the-loop quadrotor physics model. It captures the GCS and onboard
MAVLink links, then replays the exact onboard datagrams through the real
receive-only MOOS app. The producer and app are separate executions. PX4
retains its default telemetry configuration. [PX4 SIH](https://docs.px4.io/main/en/sim_sih/) · [Stock SITL image and MAVLink ports](https://docs.px4.io/main/en/simulation/px4_sitl_prebuilt_packages#mavlink-and-qgroundcontrol)

## Reproduce

Use the two immutable local images recorded in `run_capture.py`, an ARM64
Docker engine and a Python environment with `pymavlink`. The capture launcher
uses Docker's offline pull policy and an isolated network namespace with
loopback. The only writable bind is the private evidence directory. The
capture process opens receive sockets on ports 14550 and 14540 and accepts
the matching stock sender ports 18570 and 14580. It writes the complete
received datagrams, their relative monotonic acquisition times and byte
offsets. The audit joins both sender sockets to the running firmware
process and verifies the executable digest.

```sh
python -B tests/px4_native/run_capture.py --output build-px4/native
python -B tests/px4_native/replay_app.py \
  --capture build-px4/native \
  --build /path/to/mavmoos-build \
  --moos-root /path/to/stock/moos-ivp \
  --output build-px4/app-replay
```

Supply a real iMAVLink build with the telemetry and odometry modules and
the `observer_matrix_probe`. The host replay starts private loopback
endpoints and stock MOOSDB through pAntler. It preserves onboard datagram
boundaries, byte order and the original interarrival schedule. iMAVLink
assigns its own local observation times during replay. The audit compares
all observed pose, velocity, quaternion, covariance and source fields
against an independent decode and checks clearing after silence. An
independent socket capture and both app and runtime counters measure
egress. Stock pHelmIvP is pinned and hashed alongside MOOSDB and pAntler.
The replay mission launches only iMAVLink and MOOSDB.

`capture-identity.json` records the OCI image, isolation configuration,
fixture hashes and container cleanup. `capture-result.json` records the
firmware executable, namespace and socket proof, per-link message counts
and raw evidence hashes. The independent decoder preserves the bytes and
source of unrecognized MAVLink messages. Those frames contribute no payload
or CRC assertions. `app-replay-result.json` records exact source and artifact
identities, replay counts, publication and egress checks, and process
cleanup.

## Qualification on November 6, 2026

The official image digest was
`sha256:b61cc6dd50b97da78fb27b4d399ff892d8fd51656ae848f72cba2d723926b962`.
The daemon reported PX4 1.18.0 c0 at
`fca3df865af36124a28c9d607e850f111dbaaea9`. This is one candidate SIH
artifact. The capture covered 8,180 exact datagrams and saw only disarmed
PX4 quadrotor heartbeats and on-ground landed states. The onboard link
produced 328 `ODOMETRY`, 328 `LOCAL_POSITION_NED` and 401
`GLOBAL_POSITION_INT` reports.

The actual macOS ARM64 app at
`007afa13df26231278fa1f6bb9dda64d52289a7a` replayed all 4,357 onboard
datagrams. The audit matched 150 fresh local kinematics records and 150
fresh odometry records, then observed the caches clear after heartbeat loss.
The independent capture recorded zero app UDP datagrams, and all owned
processes exited. The first host attempt stopped at the loopback bind under
the workspace sandbox. The loopback-approved repeat passed.

The local evidence directories are `build-wave/px4/passive-native.dPB7ml`
and `build-wave/px4/host-replay.74xZJg` in the integration worktree. The
native capture receipt hash is
`1ec944dc8801bb5db81e5ece03672f4c3e40a81c0e35193616bd23b1d346f182`.
The host replay receipt hash is
`248363ca5d0ce47f68b30800fdd3e7e98f655cda3d006d416236fdc43460e20e`.
This qualification covers native SIH telemetry production and the app's
receive-only handling of its recorded stream.

## Direct Linux witness

The optional direct test uses the immutable local ARM64 toolchain image,
a frozen Linux build volume and a read-only volume containing the pinned PX4
package. The two processes share an isolated network namespace. iMAVLink
binds loopback port 14540 and receives directly from PX4 port 14580. A separate
receiver captures the native GCS stream on port 14550. The app counters and
PX4's sampled link rates cover the onboard direction, while the macOS replay
uses an independent socket to measure every app egress datagram.

```sh
python -B tests/px4_native/run_live.py \
  --build-volume mavmoos-px4-wave-build \
  --px4-volume mavmoos-px4-wave-package \
  --app-source /path/to/frozen/production-source \
  --app-revision FULL_MAVMOOS_COMMIT \
  --app-sha256 LINUX_APP_SHA256 \
  --output build-px4/live
```

The build volume supplies `mavmoos-passive-build` and the stock `moos-ivp`
tree. The runner checks the complete `src` and top-level CMake source
snapshot against the selected commit. It verifies app and firmware binary
digests before starting either one, then records the app's mapped plugins
and both processes' UDP socket identities. The probe uses the same
observer source as the host replay. Ending the owned PX4 process provides
the heartbeat-loss boundary. The pass receipt is `live-result.json`.

The November 6 direct case passed with Linux iMAVLink production source
`d389894f8c7a9ff48ba7b6c754b6dceeed3d3828`. Its 105-file source snapshot
matched that commit. The probe observed 117 distinct fresh records from each
of local kinematics and odometry, and heartbeat validity expired after PX4
exited. The independent GCS capture recorded ten disarmed heartbeats, ten
on-ground reports and 502 local position reports. Both app and runtime
reported zero transmissions. PX4 sampled zero incoming bytes per second
on both links. The app's capability catalog and mapped modules were retained,
and the container exited and was removed.

The local evidence directory is `build-wave/px4/live-linux.38wv5o` in the
integration worktree. Its `live-result.json` hash is
`d7179a65c9d2c150cf771f30e888862869ad324ae2b7df7c0ebc0fb317172dcc`.
