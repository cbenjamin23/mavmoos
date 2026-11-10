# Run MAVMOOS from scratch

If you already have a built MOOS-IvP checkout, put its `bin/` on `PATH` and use the [MAVMOOS source build](../../README.md#build-and-test). The pinned setup below is for a fresh workspace or reproducing a qualified workflow.

Start from a MAVMOOS source checkout. A shallow checkout is sufficient for these runs. Use Git, Python 3.12 or newer, CMake 3.24 or newer, Ninja, curl, patch, lsof and a C++17 toolchain. The first setup fetches the pinned sources, checks the dependency archive hashes and builds headless MOOS-IvP, the altitude behavior and MAVMOOS in a private directory.

- On macOS, install the Xcode Command Line Tools with `xcode-select --install`. With Homebrew, install the remaining build tools with `brew install cmake ninja python@3.12 pkg-config`. Use `python3.12` in the commands below if your default `python3` is older. To select the standalone Command Line Tools for a source build, run `DEVELOPER_DIR=/Library/Developer/CommandLineTools ./build.sh`.
- On Ubuntu 24.04, install the prerequisites with `sudo apt install build-essential cmake ninja-build git ca-certificates curl patch pkg-config python3 python3-venv zlib1g-dev lsof procps`.

## First run

```sh
WORKSPACE="$PWD/../mavmoos-work"
python3 -B scripts/setup.py --workspace "$WORKSPACE"
python3 -B scripts/reproduce.py run receive_only --workspace "$WORKSPACE"
```

Use an empty workspace on the first run. Later runs reuse its sources, downloads and build. The command prints a new run directory under `$WORKSPACE/runs/`. Look for `"result": "PASS"` in `result.json`. This exercise launches real MOOSDB and iMAVLink. A local generated MAVLink peer produces telemetry, and the test checks both MOOS publications and zero outgoing frames. See `ctest.log`, `junit.xml` and case evidence in the same run directory. Setup logs are under `$WORKSPACE/logs/`.

## Copter starter

On Apple Silicon macOS, extend the same workspace with the pinned native ArduCopter 4.6.3 build and its Python environment.

```sh
python3 -B scripts/setup.py --workspace "$WORKSPACE" --native copter
python3 -B scripts/reproduce.py run copter_operator --workspace "$WORKSPACE"
```

The fixture starts its own simulator on loopback, runs a stock-helm waypoint route with stationkeeping, exercises handback and reacquisition, then lands and disarms. Check `result.json` for the proof and cleanup fields. Read `workflow.log` for the phases and find the wire capture and onboard log beside it. This starter accepts the exact public stock revision in [the setup pins](../../config/setup.json). The historical release evidence binds its own MOOS checkout and artifact digests. [Copter scenario and acceptance](../../missions/copter_release/README.md#acceptance)

## Find a workflow

```sh
python3 -B scripts/reproduce.py list
python3 -B scripts/reproduce.py show provider.attitude
python3 -B scripts/reproduce.py run provider.attitude --workspace "$WORKSPACE"
```

Each `provider.<name>` recipe selects the required software witnesses from the [plugin catalog](../plugin-catalog.md). The catalog also gives the named native cases, expected result and log location. Use `run software` with the same workspace for the full local software suite. Use a fresh output directory on a retry and keep the earlier result for diagnosis.

### Exact native qualification inputs

The public bootstrap covers the receive-only and provider software recipes and the Copter starter. Other native recipes reuse the original fixture guards. Some require the historical MOOS pin from a local Git source, exact previously built firmware or stock-tool digests, or the qualified Linux image and PX4 volumes. The [release guide](sitl-release.md#prepare) describes that input set. The [control qualification guide](control-qualification.md#running-the-gates) describes the focused-native inputs. `show <workflow>` lists the specific keys for each recipe.

Keep exact input paths in a private JSON file using the release input names. The wrapper merges those paths with the workspace setup record. The Plane lifecycle uses the pinned Plane artifacts and a matching app build.

```sh
python3 -B scripts/reproduce.py show plane_lifecycle
python3 -B scripts/reproduce.py run plane_lifecycle \
  --workspace "$WORKSPACE" --inputs "$RELEASE_INPUTS"
```

For workflows that require the historical MOOS source, use a separate workspace and `setup.py --historical-moos --moos-source "$HISTORICAL_MOOS_SOURCE"` with the exact commit. `setup.py --help` lists the offline inputs, including an existing UAV source, archive cache and Copter source or wheelhouse. A rebuilt tool can have a different binary digest. Keep the exact artifacts for release replay and qualification.

Native commands use owned simulators only. The release campaign requires a clean checkout. Linux kit and PX4 recipes can retain a verified work volume for further release runs. See the [package instructions](../../packaging/release/README.md) for the volume details and [release evidence](evidence.md) for on-demand archive recovery.

The [handoff verification](../implementation/2026-11-07-v1-handoff/README.md) records the first clean public-source run, its platform and observed results.
