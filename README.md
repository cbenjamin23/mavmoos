# MAVMOOS

<!-- Temporary check of the Pages path filter for an unrelated edit. -->

MAVMOOS connects MOOS-IvP to ArduPilot and PX4 over MAVLink. It is designed to run on a vehicle’s companion computer alongside the IvP Helm (`pHelmIvP`). The `iMAVLink` app loads plugins at startup and defaults to receiving telemetry over UDP.

The current release enables active control only in software-in-the-loop (SITL), where autopilot firmware controls a simulated vehicle. Control on physical vehicles still needs field testing.

**[Documentation website](https://cbenjamin23.github.io/mavmoos/)** · Architecture, setup, and release testing.

## Start here

- [Vehicle support](docs/vehicle-support.md) and [plugin catalog](docs/plugin-catalog.md)
- [Architecture](docs/architecture.md)

## Build and test

Put your built MOOS-IvP checkout’s `bin/` on `PATH`, then run this from the MAVMOOS checkout.

```sh
./build.sh
```

See [Get started](https://cbenjamin23.github.io/mavmoos/start.html) for dependencies and the receive-only telemetry check.

## Repository layout

- [`src/`](src/) contains `iMAVLink`, its plugins, and shared libraries. [`examples/`](examples/) shows how to write external plugins.
- [`missions/`](missions/) holds runnable MOOS mission configurations. [`config/`](config/) holds capability definitions, firmware pins, and test fixtures.
- [`tests/`](tests/) contains software checks and simulator scenarios. [`scripts/`](scripts/) and [`.github/`](.github/) hold setup and validation automation.
- [`docs/`](docs/) contains technical guides and [`site/`](site/) contains the documentation website source.
- [`packaging/`](packaging/) contains Linux packaging and release tooling.
