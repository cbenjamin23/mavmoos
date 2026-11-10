# Copter release operator demo

Run one owned ArduCopter 4.6.3 quad and one MOOS community at time warp one. The demo uses the existing [Copter community template](../copter_sitl/README.md), stock pHelmIvP, and the existing `BHV_ConstantAltitude` behavior. The selected app starts telemetry, parameters, and Copter control. Ordinary app launches default to receive-only.

## Run the demo

Use the [from-scratch starter](../../docs/development/from-scratch.md#copter-starter) to build the public stock MOOS-IvP and ArduCopter sources and run the mission. The following commands reproduce the original release profile with the historical MOOS checkout.

From the MAVMOOS repository root, [build the app and tests](../../README.md#build-and-test) against stock MOOS-IvP at `023fb54e00692611ecee2cd6f1138d10eec2127e`. Set `STOCK_MOOS` and `BUILD` to those checkout and build directories. Set `BEHAVIOR_DIR` to the directory containing the compiled `BHV_ConstantAltitude` library and `SITL_PYTHON` to a Python environment with pymavlink. See the dependency recipe and the [firmware guide](../../config/firmware/README.md) for the pinned inputs.

Set `COPTER` to the pinned ArduCopter 4.6.3 checkout and `COPTER_ATTESTATION` to its build attestation. Choose a fresh output path for `FRESH_EVIDENCE`.

```sh
./missions/copter_release/launch.sh \
  --python "$SITL_PYTHON" --build "$BUILD" --moos-root "$STOCK_MOOS" \
  --ardupilot-root "$COPTER" --firmware-attestation "$COPTER_ATTESTATION" \
  --behavior-dir "$BEHAVIOR_DIR" --output "$FRESH_EVIDENCE"
```

Pass `--expected-app-revision` when qualifying a selected release commit. The supervisor checks a clean checkout, build source and stock MOOS paths, Copter manifest and binary attestation, and runtime input hashes. The run needs time for simulator estimator readiness, route transit, and safe landing. Its outer deadline is 200 seconds. The supervisor owns its ephemeral loopback ports and serializes against the other local MAVMOOS simulator workflows.

## Acceptance

1. The flight controller rejects an unarmed takeoff. Guided, arm and takeoff to 5 m complete, and independent FC position and landed-state evidence corroborates flight.
2. Stock helm drives a two-leg waypoint route. A single `SIM_WIND_SPD` read completes during active navigation and the accepted navigation counter advances. The test makes a read request and verifies the configured zero-wind value.
3. A short ordinary no-goal gap suspends guidance and resumes with fresh stock outputs inside the existing recovery window. A longer gap latches authority, and returning outputs stay inhibited until an explicit acquire.
4. The stock waypoint behavior posts completion and `route_finished_hold` becomes the sole horizontal objective. Its low priority during the route keeps course and speed available on the completion tick. Independent FC telemetry verifies approach to the turning waypoint and the final station's position, drift, speed, and height.
5. The independent pilot endpoint selects native Loiter on the owned FC. The bridge fences old navigation while stock outputs continue, then accepts a separate explicit reacquisition. The fixture independently observes the mode acknowledgement and the FC's Loiter and Guided heartbeats.
6. Native LAND reaches ground and disarms. All owned processes and scoped ports clear, and input hashes match the preflight. The closed DataFlash log corroborates neutral simulator throttle during Loiter and its restoration after the handback.

`result.json` links the exact fixture, generated mission, app and module digests, simulator wire capture, MOOS trace, and DataFlash. Its `files` index records the bytes and SHA-256 for each original output. `profile-result.json` binds that receipt to the release case and names its independent proofs. The pilot and monitor use separate exact owned simulator endpoints. Only the explicitly scoped simulator pilot path supplies RC throttle and one mode request.
