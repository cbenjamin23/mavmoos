# MAVMOOS

MAVMOOS connects MOOS-IvP to ArduPilot and PX4 over MAVLink. It is designed to run on a vehicle’s companion computer alongside the IvP Helm (`pHelmIvP`). The `iMAVLink` app loads plugins at startup and defaults to receiving telemetry over UDP.

The current release enables active control only in software-in-the-loop (SITL), where autopilot firmware controls a simulated vehicle. Control on physical vehicles still needs field testing.

**[Documentation website](https://cbenjamin23.github.io/mavmoos/)** · Architecture, setup, and release testing.

## Start here

- **Find your vehicle.** Check the exact firmware, frame and qualified workflows in the [vehicle support matrix](docs/vehicle-support.md). The [plugin catalog](docs/plugin-catalog.md) covers shared telemetry, mission, parameter and other services.
- **Run your first mission.** The [Copter SITL starter](missions/copter_release/README.md#run-the-demo) lists the pinned simulator and altitude behavior inputs and gives the launch command. Stock helm flies a waypoint route and stationkeeps before the flight controller lands.
- **Understand the system.** The [architecture diagram](docs/architecture.md#mavmoos-on-the-backseat-computer) shows how stock helm, iMAVLink and the flight controller exchange observations, requests and targets.

## Build and test

Clone MAVMOOS with your built MOOS-IvP checkout's `bin/` directory on `PATH`. Use CMake 3.24 or newer, Ninja, a C++17 compiler and Python 3.

```sh
./build.sh -j4
./build/bin/iMAVLink --help
```

The build finds the checkout through `pHelmIvP` on `PATH`, with an adjacent `moos-ivp` checkout as a fallback. Set `MOOS_IVP_ROOT` to select a checkout explicitly. App code is in `src/`, runnable configurations are in `missions/`, and the build places `iMAVLink` in `build/bin/` and its plugins in `build/plugins/`. Set `MAVMOOS_MAVLINK_ARCHIVE` to the pinned header archive to build offline.

Run the receive-only check to confirm that a local MAVLink fixture reaches MOOSDB without outbound MAVLink.

```sh
ctest --test-dir build -R '^moos_normal$' --output-on-failure
```

## Run from scratch

Use the [setup and workflow guide](docs/development/from-scratch.md) for a fresh MOOS-IvP workspace, the Copter simulator starter, or a plugin test recipe.

## Technical documentation

- [Plugin authoring](docs/development/external-plugins.md) covers external observer and command plugins.
- [Command and transition coverage](docs/development/control-coverage.md) lists the tested actions, handoffs and failure cases.
- [Release testing and evidence](docs/development/evidence.md) covers the latest results and reproducibility.
- [Design references](docs/references.md) links to MOOS-IvP, MAVLink, the autopilots and the MAVROS crosswalk.
