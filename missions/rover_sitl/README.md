# Stock-helm ground Rover mission

This ordinary one-community mission uses stock `BHV_Waypoint` and `BHV_StationKeep` to produce travel course and speed. There is no altitude domain, custom behavior or pHelmIvP modification. The `ardurover_control` provider translates those two quantities into ArduRover Guided targets. The initial profile is a ground Rover, not a boat; `StationKeep` describes a MOOS behavior, not a promise of native boat Loiter.

One local MOOSDB hosts the bridge, helm, logger and process watcher. There is no shoreside viewer or second community to share with, so this mission intentionally omits shoreside and pShare files/ports. `--nogui` is accepted. Interactive use opens uMAC; automation uses `--xlaunched`. Time warp stays at 1.

With a built app/modules and stock MOOS binaries on PATH, generate without starting anything:

```sh
./launch.sh --just_make --nogui --veh_mport=19021 --bind_port=14621 --peer_port=14620
```

The default is shadow mode. For an owned SITL run, pass `--native_replay_dir=/absolute/private/directory` along with `--simulation-only --sitl_pid=PID`. Create the replay directory with mode 0700 and keep it across bridge restarts. Use `--app=/absolute/path/iMAVLink` and `--plugin_dir=/absolute/path/plugins` when using a build other than `build/`. `--speed=1.2`, `--waypoints='0,8:8,8:8,0:0,0'`, and `--lat_origin/--long_origin` configure the ordinary route. Origin and simulator home must agree for useful NAV_X/Y and waypoint interpretation.

The reproducible effectful workflow is the owned wrapper, after integration has built `rover_workflow_probe`, `iMAVLink` and its three modules:

```sh
python tests/rover/run_rover.py --ardupilot-root=/path/to/pinned/private/ardupilot \
  --moos-root=/path/to/moos-ivp --build=/path/to/mavmoos/build \
  --output=/path/to/fresh/output
```

Run it from the repository root using the ArduPilot Python environment with `pymavlink`. Output must be a new directory. The wrapper holds the common native-test lock, starts an exact owned `ardurover`, verifies its UDP/RC socket ownership and identity, generates this mission with explicit PID/ports, and uses only localhost. Its independent MAVLink observer never writes. A separate fixture pilot sends neutral RC1/RC3 and switches RC8 from native Hold to Manual; DataFlash must confirm a real RC-command mode transition. Cleanup targets only recorded owned processes and endpoints.

The automated sequence is unsupported-flight rejection, Guided, ordinary arm, explicit helm acquire and 2D motion, native Hold while the helm remains deployed, acquire from Hold, pilot Manual takeover, pilot Hold handback and explicit acquire recovery, native RTL, Hold and normal disarm. It observes command outcomes, movement and authority; no altitude or airborne evidence is needed. The fixture has no rally points and pins WP_RADIUS=1 m, so RTL is evaluated against bound home. The source FC may otherwise select a closer rally point.

For manual inspection, the corresponding MOOS requests are `MAVMOOS_COMMAND=id=...,action=guided|arm|hold|rtl|disarm` and `MAVMOOS_NAV_CONTROL=acquire|release`. `DEPLOY=true` starts the configured behaviors. Native Guided does not grant helm permission. Native Hold stops ground motion and revokes helm; it is distinct from continuing MOOS stationkeeping in Guided. Each command needs a fresh unique ID. See the [operator guide](../../docs/plugins/ardurover_control.md) for readiness, freshness and supported policies.

`clean.sh` removes only generated target/log files and never stops processes. Launch scripts terminate their directly owned pAntler child; the automated wrapper provides the stronger descendant/PID-and-argv cleanup and endpoint readback used for test acceptance. No real vehicle or serial-hardware use is supported by this mission.
