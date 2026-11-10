# Optional disarmed native boat witness

The manual witness uses the already pinned `04cf6486e8d42ce138de816e8466244346ed77cb` `ardurover` artifact, `motorboat-skid`, its three original model defaults and a private overlay. It follows ArduPilot's own DepthFinder configuration. Simulated serial NMEA supplies sensor 0 with temperature, the forward I²C sensor 2 supplies the orientation exclusion, and downward I²C sensor 5 supplies missing temperature. The local startup schedule requests the firmware's ordinary telemetry producer every 200 ms through its configuration file. The app and the witness remain passive while the boat stays disarmed. [Pinned ArduPilot DepthFinder fixture](https://github.com/ArduPilot/ardupilot/blob/04cf6486e8d42ce138de816e8466244346ed77cb/Tools/autotest/rover.py#L6038-L6110) · [Pinned simulated NMEA range and temperature](https://github.com/ArduPilot/ardupilot/blob/04cf6486e8d42ce138de816e8466244346ed77cb/libraries/SITL/SIM_RF_NMEA.cpp#L25-L51)

One owner-proved loopback socket receives and timestamps the FC's raw datagrams before making a one-way byte copy to the actual RX_ONLY iMAVLink with only `water_depth` loaded. The separate app peer captures all return traffic. The audit replays both raw byte spans, checks disarmed native and app state, excludes the forward sensor, and matches at least four fresh MOOS records per downward sensor by actual payload and original acquisition time. The witness uses the Darwin raw monotonic clock shared with the app. The pinned simulator inputs and stock MOOS artifacts are checked before and after, alongside process and port cleanup.

Use a clean source and frozen CMake build with `iMAVLink`, `observer_matrix_probe` and `mavmoos_water_depth.so`. The existing native Rover executable is reused.

```sh
"$boat_python" -B tests/water_depth/run_native.py \
  --rover-root "$pinned_rover_root" --moos-root "$stock_moos_root" \
  --build "$app_build" --app-revision "$app_revision" \
  --app-sha256 "$app_sha" --probe-sha256 "$probe_sha" --module-sha256 "$module_sha" \
  --check-inputs
```

Replace `--check-inputs` with `--output "$fresh_private_run"` in the reserved native window. The runner also acquires the shared native lease. Archive `result.json`, the raw MAVLink, captured egress, MOOS probe, ownership and the hashed boot fixture. The initial pinned run passed with two downward sensors, the forward sensor excluded and zero bridge egress. Exact evidence. The component and generated cases cover stale data and invalid values separately.
