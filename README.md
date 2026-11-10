# MAVMOOS

MAVMOOS connects MOOS-IvP to ArduPilot and PX4 over MAVLink. It is designed to run on a vehicle’s companion computer alongside the IvP Helm (`pHelmIvP`). The `iMAVLink` app loads plugins at startup and defaults to receiving telemetry over UDP.

The current release enables active control only in software-in-the-loop (SITL), where autopilot firmware controls a simulated vehicle. Control on physical vehicles still needs field testing.

**[Documentation website](https://cbenjamin23.github.io/mavmoos/)** · Architecture, setup, and release testing.

## Start here

- [Vehicle support](docs/vehicle-support.md) and [plugin catalog](docs/plugin-catalog.md)
- [Copter SITL starter](missions/copter_release/README.md#run-the-demo)
- [Architecture](docs/architecture.md)

## Build and test

Put the built MOOS-IvP checkout’s `bin/` on `PATH`. Use CMake 3.24 or newer, Ninja, a C++17 compiler and Python 3. From the MAVMOOS checkout, build and run the receive-only check.

```sh
./build.sh -j4
ctest --test-dir build -R '^moos_normal$' --output-on-failure
```

[Setup from scratch](docs/development/from-scratch.md) · [Plugin authoring](docs/development/external-plugins.md) · [Release testing](docs/development/evidence.md)
