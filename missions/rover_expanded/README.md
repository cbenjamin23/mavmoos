# Ground and boat Rover qualification

The headless fixture runs the same stock pHelmIvP waypoint behavior in two explicit profiles. The ground profile selects ArduRover type 10 and model `rover`. The boat profile selects type 11 and the pinned differential-thrust `motorboat-skid` model. The fixture retains the same dynamic `ardurover_control` module and native executable across the two configurations.

`meta_vehicle.moos` defines the single owned community. The harness generates it and reuses the ordinary `BHV_Waypoint` and `BHV_StationKeep` behavior file from [the original Rover fixture](../rover_sitl/meta_vehicle.bhv). Stock pHelmIvP comes from the pinned MOOS-IvP checkout. The fixture supplies a 2D course and speed domain at timewarp 1.

The qualification runner owns all processes, allocates loopback endpoints and records the exact connected simulator sockets before starting the app. It needs an existing pinned native Rover build, the stock MOOS-IvP tools, the MAVMOOS build and a Python environment with pymavlink.

```bash
python tests/rover_expanded/run_native.py \
  --profile ground \
  --ardupilot-root /path/to/pinned/ardupilot \
  --moos-root /path/to/pinned/moos-ivp \
  --build /path/to/mavmoos/build \
  --output /path/to/fresh/ground-run
```

Repeat with `--profile boat` and a fresh output path. For repeat local execution, pass `--output-parent` instead of `--output` to allocate a fresh run directory. The runner takes the same lock as other owned native SITL fixtures. Its five-minute outer test timeout allows for native estimator convergence, real-time navigation and the boat disturbance observation.

The MOOS probe drives a normal arm, 2D stock helm waypoint travel, stock all-stop expiry, pilot Manual takeover, FC link interruption and explicit recovery. It also exercises native Hold and RTL, versioned Guided goto, hold, cancel and return, and final disarm. The boat adds native Loiter and explicit reacquisition from Loiter. The independent FC monitor receives a separate MAVLink stream without writing to it. Its time-aligned position and mode records and the DataFlash log corroborate physical movement, operation outcomes, pilot takeover and normal arming/disarming. Every run retains its generated mission, trace, selected source and artifact hashes, and cleanup readback.

The boat run measures current-driven drift during its initial armed native Hold before sending drive commands. It uses a 0.18 m/s simulated tide from the east with wind and waves disabled. It compares that drift against a 25-second native Loiter window with a 1 m radius. Sustained drive correction after reaching the radius and a reduced displacement relative to the measured Hold rate provide evidence under this specific model and fixture. Guided hold commands zero velocity and uses advancing low-speed observations for completion. Native Loiter actively navigates within its configured radius under the modeled disturbance.

The separate generated-peer fixture covers RX-only and shadow in both profiles. It sends sibling-profile and expected MAVLink types, verifies rejection and observation through the real app and stock helm, and counts every return datagram.

```bash
python tests/rover_expanded/run_passive.py \
  --moos-root /path/to/pinned/moos-ivp \
  --build /path/to/mavmoos/build \
  --output-parent /path/to/generated-runs
```
