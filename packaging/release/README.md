For regular use, [clone and build MAVMOOS against MOOS-IvP](../../README.md#build-and-test). The recipes here describe the bundles used for Linux qualification.

## Install the Linux ARM64 v1 bundle

On Ubuntu 24.04 ARM64, follow the [runtime guide](runtime-v1.md) to check the archive, extract it in your home directory and start the receive-only example. The archive includes iMAVLink, its standard providers, the headless stock MOOS-IvP tools and the altitude behavior. Firmware and simulator setup are separate.

For a stock helm mission that uses `BHV_ConstantAltitude`, set `ivp_behavior_dir` in `ProcessConfig = pHelmIvP` to the absolute path of the installed `lib` directory.

To build the bundle locally, follow the builder command below with `--v1` and the public stock MOOS-IvP revision in [the setup manifest](../../config/setup.json). It writes `mavmoos-1.0.0-rc.1-linux-arm64.tar.gz` and its `.sha256` sidecar. The output also includes `px4_offboard_moos_probe`, a test fixture for the installed-runtime SITL run.

Use the retained offline PX4 v1.17.0 package described in the [stable firmware qualification](../../docs/implementation/2026-11-06-stable-px4/README.md#evidence-and-reproduction). The test extracts and verifies the exact v1 archive, then runs the stock helm and bridge from that installation. It uses a separate read-only firmware volume and an isolated loopback network.

```sh
python3 -B packaging/release/qualify_v1.py \
  --kit /path/to/v1-kit-output \
  --px4-package-volume "$PX4_PACKAGE_VOLUME" \
  --output /path/to/new/v1-installed-sitl-evidence
```

### Replay the installed Copter starter

Use the same runtime archive and the pinned Linux ARM64 tools image. Export the pinned ArduCopter source with `scripts/firmware/export_source.py` and the `arducopter-4.6.3-quad.json` manifest. Use the attested ARM64 firmware binary from `verification-copter.zip` at `build/sitl/bin/arducopter` in that export. The ZIP also contains the matching build attestation, test probe and wheels. Install the wheels offline with the tools image's Python 3.12 into a separate directory and pass it as `--python-packages`.

```sh
python3 -B packaging/release/qualify_copter_v1.py \
  --archive "$ARCHIVE" --firmware "$FIRMWARE_EXPORT" \
  --firmware-attestation "$ATTESTATION" --probe "$PROBE" \
  --python-packages "$PYTHON_PACKAGES" --output "$NEW_EVIDENCE_DIR"
```

## Build and verify the existing Linux kit

The controller requires the effective Docker endpoint to resolve to an existing local Unix socket before copying any source. It pins that socket with an isolated CLI configuration. Ownership is recorded before creation so cleanup can recover a container or work volume after a lost reply. [Docker CLI configuration](https://docs.docker.com/reference/cli/docker/)

Use Python 3.11 or newer, Git, and a local ARM64 Docker engine with the immutable builder image in `build_linux.py`. The stock MOOS-IvP and `moos-ivp-uav-base` checkouts need the Git objects for the pinned revisions. Supply the pinned MAVLink, PROJ 5.2.0, and CPM 0.43.1 files from local storage. The receipt records their exact digests. The builder uses a fresh container with loopback networking, fixed resources, and a nonprivileged account. The app source archive omits historical evidence and Git packfiles.

```sh
python3 -B packaging/release/build_linux.py \
  --source-ref "$(git rev-parse HEAD)" \
  --moos-source /path/to/stock/moos-ivp \
  --uav-source /path/to/moos-ivp-uav-base \
  --cpm-file /path/to/CPM_0.43.1.cmake \
  --proj-archive /path/to/proj-5.2.0.tar.gz \
  --mavlink-archive /path/to/mavlink.tar.gz \
  --output /path/to/new/linux-kit-evidence
```

The output contains `mavmoos-linux-arm64.tar.gz`, a checksum-bound `result.json`, and build and consumer receipts. The kit includes the bridge, the selected commit's built-in module roster, and the stock MOOSGeodesy library needed by telemetry. The build and installed kit use the same relative library layout. Its operator guide covers extraction, payload verification, stock MOOS tools, and the receive-only mission. [Runtime guide](runtime.md)

A second fresh container extracts the same archive under a different prefix. It checks dynamic dependencies and runs the actual bridge and stock MOOSDB with eight installed modules and a generated loopback peer. The witness checks passive app counters, independently measures UDP egress, and requires scoped process and port cleanup. It also removes one module in the private installation and checks that the real app refuses startup configuration. The checksum verifier rejects the missing module, changed module bytes, and an unexpected source revision.

The command removes its containers and temporary staging files. For the PX4 release checks, add `--work-volume mavmoos-release-<unique-name>`. This retains one newly created volume after a pass. Mount it read-only at `/work` in the profile runner. It contains the frozen app source, app and module build, `px4_offboard_moos_probe`, and stock MOOS tools and behavior. The `build-witness-artifacts.json` receipt binds those artifacts separately from the package layout.
