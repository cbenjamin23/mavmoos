# MAVMOOS 1.0.0-rc.1 for Linux ARM64

Use Ubuntu 24.04 on ARM64. The bundle includes the headless stock MOOS-IvP tools, matching MAVMOOS modules and a separately built altitude behavior. Each MOOS and behavior source revision is recorded in `manifest.json`. Simulation firmware has its own install and configuration.

## Install

On a minimal Ubuntu install, install the runtime packages first. The stock MOOS sockets use the system protocol database from `netbase`. The archive itself installs in your home directory.

```sh
sudo apt-get update
sudo apt-get install -y netbase libstdc++6 zlib1g
```

Run the following commands from the directory with the archive and its checksum.

```sh
sha256sum --check mavmoos-1.0.0-rc.1-linux-arm64.tar.gz.sha256
install_dir="$(mktemp -d "$HOME/mavmoos-1.0.0-rc.1.XXXXXX")"
tar -xzf mavmoos-1.0.0-rc.1-linux-arm64.tar.gz -C "$install_dir"
cd "$install_dir/mavmoos"
sha256sum --strict --check SHA256SUMS
./bin/iMAVLink --help
```

Keep the `bin`, `lib` and `plugins` directories together.

## Start the receive-only example

Run this command from the extracted `mavmoos` directory.

```sh
PATH="$PWD/bin:$PATH" pAntler share/mavmoos/receive-only.moos
```

The example uses loopback and starts MOOSDB on TCP 9100. It listens for an ArduCopter-shaped MAVLink stream on UDP 14600 from a loopback peer on UDP 14601. Use available ports and match the peer's port to the mission if you change them. This mission observes telemetry. Use Ctrl+C to stop the community.

To use the bundled helm in a separate mission, put this install's `bin` on `PATH` and set the mission's `plugin_directory` to the absolute path of this install's `plugins` directory. The altitude behavior is in `lib`. Select only an owned SITL profile and its matching configuration for active control.

Notices are in `share/mavmoos/licenses/`. MOOS-IvP and the altitude behavior sources are available at their recorded revisions in the [MOOS-IvP repository](https://github.com/moos-ivp/moos-ivp) and the [UAV base repository](https://github.com/cbenjamin23/moos-ivp-uav-base). The geodesy build uses [PROJ 5.2.0](https://github.com/OSGeo/PROJ/tree/5.2.0).
