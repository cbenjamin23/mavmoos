# Observers beside active stock helm and protocol reads

This is a native simulator acceptance fixture, not a field launcher. It starts one owned Copter at timewarp1, one localhost MOOS community with unchanged stock pHelmIvP, and one MOOS probe. The six-module profile is `telemetry,system_status,parameters,missions,arducopter_control,attitude`. An optional `--profile status` comparison omits only attitude. Default receive-only behavior in the app is unchanged; this fixture explicitly selects the existing simulator-control guard and proves its exact owned native UDP endpoint.

The C++ probe performs the existing Guided/arm/takeoff5m workflow, explicitly acquires ordinary stock-helm control, then concurrently requests a paced indexed parameter inventory and a HOME-only mission download. Parameter wind reads before/after verify unchanged state. It requests no parameter write, mission upload/clear/execution or new control mode. It continues navigation for five seconds after both read results, then lands/disarms. Observations never enter the probe's readiness/acquisition conditions.

The outer250s deadline covers native GPS/EKF convergence, flight, the original60s inventory/15s mission bounds and landing. It does not extend individual service deadlines. The optional native CTest should allow280s including shutdown and DataFlash extraction, run serially and hold `/tmp/mavmoos-native-sitl.lock`. Root assigns the native window; never run beside another native case.

## Build and offline checks

The root build should add `observation_load_probe` from `ObservationLoadProbe.cpp` and link `${MOOS_LIBRARIES}`. A private compile is available:

```sh
cmake -G Ninja -S tests/observation_load -B build-observation-load-private \
  -DMOOS_IVP_ROOT=/path/to/built/moos-ivp
cmake --build build-observation-load-private
python3 -B tests/observation_load/ObservationLoadEvidenceTests.py
```

The offline tests use synthetic evidence only and launch no processes/sockets. They reject early-only/cached observer samples, impossible metadata, wrong units/frames, unsupported ownership, renewed deadlines, incomplete snapshots, mutations/reacquisition, stale final state, missing FC target reports and falsely matched orientation. Separate Python observer-clock offsets leave valid checks unchanged.

## Native invocation after root approval

Use a Python containing pymavlink and an entirely new output directory:

```sh
PYTHON tests/observation_load/run_observation_load.py \
  --ardupilot-root /path/to/pinned/ardupilot \
  --moos-root /path/to/built/moos-ivp \
  --behavior-dir /path/to/ordinary/uav/behavior/lib \
  --build /path/to/frozen/mavmoos/build \
  --source-root /path/to/matching/mavmoos/source \
  --probe /path/to/frozen/observation_load_probe \
  --profile attitude --output /path/to/new/evidence
```

`--expected-app-sha256` can require the exact root-frozen app. Both support helpers are freshly loaded from the explicit source checkout, with actual filenames/hashes protected before/after; stale cached helper modules from another worktree cannot win. App/native/probe/modules, templates, relevant source and startup defaults have independent before/after hashes. The runner proves native primary and passive-source sockets, repeatedly verifies the same mission app/helm PID/argv and bridge socket, and tears down only its owned process tree. It checks released ports and reads the closed DataFlash before accepting the result.

## Evidence layers and limits

`probe.jsonl` retains raw stdout including any MOOS banner; the runner parses only lines beginning with `{`. The strict checker requires complete distinct inventory, exact terminal results/original observed bounds, HOME matching the bound FC reference, read-only mission wire IDs, active authority/helm continuity and advancing fresh observations both near read completion and in the separate continuation. It reports local accepted-NAV counter gaps as such.

`fc-observer.mavlink` and its decoded JSONL come from a separate receive-only native serial1 UDP endpoint. Startup interval files configure native telemetry; the observer sends zero. Its Python receipt windows are bracketed by live receipt of atomic C++ phase marker files. These values are never subtracted from C++ probe/provider clock values. Independent position/mode/landed state corroborates flight, movement and final landing/disarm. Message85 exposes pinned ArduPilot's current Guided velocity/acceleration target state; its reception gap is not the primary uplink send interval or a motor-control update rate.

Late attitude samples are paired only with the same FC message family using nearest `time_boot_ms` within300ms. Euler differences are wrapped per angle; quaternion comparison accepts sign equivalence. This is a value/convention check for the short no-wrap run, not source-age clock synchronization or atomic coherence between Euler and quaternion messages. The pinned simulated battery stays12.6V with capacity disabled; known battery0 voltage is checked through MOOS, passive MAVLink and DataFlash. This fixture-specific bound is not a general battery-health policy.

Descriptor intent and received families/rates do not prove receipt of each primary SET_MESSAGE_INTERVAL request or ACK. Direct owned native routing is retained; no relay/capture bypass is introduced. The parent batch independently decodes interval requests under fake I/O. DataFlash confirms configured parameters, unchanged simulator wind, ordinary arm/disarm and Guided/Land modes. Every claimed outcome remains simulation/profile-specific; there is no radio, serial hardware, physical sensor, host-wide wire capture, all-fault, load-capacity or worst-case latency qualification.
