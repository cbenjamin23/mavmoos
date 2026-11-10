# Stock-helm autonomy in SITL

These two software-in-the-loop (SITL) missions run the actual ArduPilot controller against stock IvP Helm. A separate receive-only MAVLink connection measures what the vehicle does. The same MOOS community runs stock `pMissionEval`, which writes `results.txt` after the independent criteria are met.

- **Copter survey.** Start a northbound route at 5 m above home. A simulated operator submits one `SURVEY_UPDATE` after the first waypoint. The stock waypoint behavior applies `xpoints` while retaining its active waypoint index, then flies the revised survey legs. Its completion flag activates a separate helm return behavior. The runner verifies home stationkeeping before requesting native LAND.
- **Boat transect.** Follow a northbound transect in the differential-thrust ArduRover boat. A scripted contact crosses from starboard at constant speed. It supplies ordinary MOOS `NODE_REPORT` messages while stock waypoint and collision-avoidance behaviors are active. The runner measures separation from the contact's analytic trajectory and independent boat positions, then verifies route resumption and finish. The contact model uses one scripted vessel and calm water.

The provider mission guide covers the other five control providers, their pinned simulators, and how to replay their qualification results.

The compound stress missions exercise all seven control providers with competing native modes, faults, and fresh helm acquisition in the same run.

## Run locally

Build MAVMOOS and the tests with [the repository instructions](../../README.md#build-and-test) using stock MOOS-IvP at `023fb54e00692611ecee2cd6f1138d10eec2127e`. In the build directory, build `autonomy_mission_probe` if it is not already built. Use a Python environment with pymavlink. The [Copter starter](../copter_release/README.md) explains the 4.6.3 quad build attestation and the existing compiled altitude behavior. The boat uses the exact [qualified ArduRover build](../rover_expanded/README.md) at `04cf6486e8d42ce138de816e8466244346ed77cb`.

From the repository root, with the inputs in the environment, run:

```sh
./missions/autonomy_sitl/launch.sh --python "$SITL_PYTHON" copter-survey \
  --build "$BUILD" --moos-root "$STOCK_MOOS" --ardupilot-root "$COPTER" \
  --firmware-attestation "$COPTER_ATTESTATION" --behavior-dir "$BEHAVIOR_DIR" \
  --output "$FRESH_SURVEY_EVIDENCE"

./missions/autonomy_sitl/launch.sh --python "$SITL_PYTHON" boat-transect \
  --build "$BUILD" --moos-root "$STOCK_MOOS" --ardupilot-root "$ROVER" \
  --output "$FRESH_BOAT_EVIDENCE"
```

Each run uses fresh output, loopback ports and the shared native simulator lock. Use `--just-make` to inspect inactive targets without a firmware checkout. Use `--moos-port PORT` to choose the private MOOSDB port. The live supervisor owns arming, completion, the outer waits and scoped teardown.

A passing run contains one `grade=pass` in `results.txt` and `result=PASS` in `result.json`. The receipt records the physical checks, inputs and cleanup. The output also holds the generated targets, MOOS log, read-only MAVLink capture and simulator log. Replay a passing run with `"$SITL_PYTHON" -B tests/autonomy_missions/audit.py "$RUN_DIRECTORY"`.
