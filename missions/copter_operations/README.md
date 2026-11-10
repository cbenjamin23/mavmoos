# Copter operations with a stock helm

One ordinary local MOOS community, defaulting to shadow/no MAVLink output. The actual `arducopter_control` provider groups native actions, bridge Guided-position targets and configured stock-helm strategies. No upstream helm or behavior changes are required. An existing `BHV_ConstantAltitude` extension supplies the altitude objective.

Build the matched host/modules against stock MOOS-IvP, then generate first:

```sh
./launch.sh --just_make --nogui --veh_mport=19651 \
  --app=/absolute/build/bin/iMAVLink --plugin_dir=/absolute/build/plugins \
  --behavior_dir=/absolute/path/to/uav/lib
./launch.sh --nogui --veh_mport=19651 \
  --app=/absolute/build/bin/iMAVLink --plugin_dir=/absolute/build/plugins \
  --behavior_dir=/absolute/path/to/uav/lib
```

Time warp is fixed to 1. This is one community; there is no shoreside or pShare port. `--veh_mport`, `--bind_port`, `--peer_port`, `--lat_origin`, `--long_origin`, `--facing_policy` and `--facing_heading` are forwarded by the normal launcher. For effectful tests only, the parent-owned native simulator fixture supplies `--simulation-only --sitl_pid=PID` and proves the exact loopback FC socket; no hardware/serial route is supported.

The helm interface uses ordinary `MAVMOOS_HELM_OPERATION=route|hold|none` conditions and standard behavior `updates`. The provider emits point/altitude updates and acquires control only for explicit requests. A source-qualified, fresh post-acquisition helm output is still required. The fixed-point goto/hold/return binding uses stock StationKeep for transit and then holding at the requested point, with per-operation speed/radius settings. It is not a path planner. Legacy/custom Waypoint missions continue through the unchanged ordinary navigation interface. Each point update has an operation label so repeated goals remain independently inspectable without a custom helm command.

Example requests posted with uPokeDB or another ordinary MOOS client:

```text
MAVMOOS_CONTROL_REQUEST=schema=1,id=guided1,action=guided
MAVMOOS_CONTROL_REQUEST=schema=1,id=arm1,action=arm
MAVMOOS_CONTROL_REQUEST=schema=1,id=takeoff1,action=takeoff,height=5
MAVMOOS_CONTROL_REQUEST=schema=1,id=goto1,action=goto,strategy=guided,lat=42.35850,lon=-71.08750,height=5,facing=keep
MAVMOOS_CONTROL_REQUEST=schema=1,id=hold1,action=hold,strategy=helm
MAVMOOS_CONTROL_REQUEST=schema=1,id=cancel1,action=cancel,target=hold1
MAVMOOS_CONTROL_REQUEST=schema=1,id=home1,action=return,strategy=helm,height=5
MAVMOOS_CONTROL_REQUEST=schema=1,id=rtl1,action=return,strategy=native
```

Wait for each operation's appropriate result before the next. Native return is FC RTL and completes on observed ground state. Helm return travels to the bound home at the chosen home-relative height; it does not land. Guided/helm hold stays active until cancellation or its original deadline. Cancellation is local and cannot recall transmitted targets or native commands. Native Loiter follows pilot input; the simulator workflow supplies and independently verifies scoped neutral RC throttle during its Loiter check.

Read `MAVMOOS_CONTROL_RESULT`, `MAVMOOS_CONTROL_STATUS`, legacy command/helm status and shared runtime diagnostics. The authoritative workflow script is `tests/operations/run_operations.py`; its integration evidence belongs in the dated Copter record.
