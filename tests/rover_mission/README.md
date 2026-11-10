# ArduRover mission SITL

The default `mission` scenario exercises a complete native mission, pilot takeover and restart while the controller is still in AUTO. The `paused-restart` scenario restarts only the bridge while an armed mission is paused in ground Hold or powered boat Loiter. The original simulator, MOOSDB, stock pHelmIvP and recovery directory remain active.

Run each profile with a clean MAVMOOS build and the pinned ArduPilot and stock MOOS-IvP inputs. The Python interpreter must have pymavlink. Each run needs a fresh output path and takes the common host SITL lock. [Pinned inputs and operator interface](../../docs/plugins/ardurover_mission.md)

```sh
python3 -B tests/rover_mission/run_native.py \
  --ardupilot-root "$ROVER_ROOT" --moos-root "$MOOS_IVP_ROOT" --build "$BUILD_DIR" \
  --profile ground --scenario paused-restart --output "$RUN_DIR/ground"
```

Repeat with `--profile boat` and `--output "$RUN_DIR/boat"`. The paused restart case requires:

- A verified plan, independent AUTO motion, and a stopped native pause with cursor 1 before restart.
- A fresh process session that still blocks control in the supported handback mode. An early current-session handback fails until fresh stop evidence arrives. A previous-session handback, old mission resume and abort, competing native Guided request, and explicit helm acquire all fail to move the vehicle or hand off commands while the fence holds.
- Explicit current-session handback. Fresh, positive-speed stock helm output still has no authority, and both a delayed resume and an old plan proof are rejected.
- Ordinary stock-helm redeployment supplies a new all-stop state after handback. A separate acquire then leads to independent Guided mode and at least 2 m of travel. Native Hold and disarm finish the run. Independent heartbeats, position, cursor data, and onboard mode records verify the effects. The MOOS trace checks local transmit acceptance separately.

`result.json` records the bounds and observations, input hashes and cleanup checks. The run directory holds raw FC packets, decoded FC samples, MOOS traces and the simulator's onboard log.
