# Copter payload operation qualification

The manual native cell uses the pinned ArduCopter 4.6.3 quad as the actual MAVLink router. Serial 0 is the direct application peer. Serial 1 is the independent Python camera and gimbal component route. The router learns both component addresses from their standard MAVLink reports. The app, stock MOOSDB and pAntler run from their selected builds. The runner checks the exact owned simulator PID and connected loopback sockets before handing serial 0 to the app or issuing any test request.

Build `payload_operations_moos_probe`, `iMAVLink`, `mavmoos_telemetry`, `mavmoos_camera_status` and `mavmoos_gimbal_status` from the same clean checkout with the pinned stock MOOS root. Use a local Python environment with pymavlink and the firmware build's matching attestation. The runner acquires the common native SITL lease and requires a fresh evidence directory.

```sh
python -B tests/payload_operations/run_native.py \
  --build "$build" \
  --expected-app-revision "$(git rev-parse HEAD)" \
  --moos-root "$stock_moos_root" \
  --ardupilot-root "$pinned_copter_root" \
  --firmware-attestation "$pinned_copter_attestation" \
  --output "$fresh_evidence_dir"
```

The first cell requests one camera image and a gimbal point with the same operator ID. It requires two independent confirmations, a new private image file, matching simulated pitch and yaw, and observed gimbal release. The second cell repeats the capture ID, then drops the next camera ACK and gives the gimbal to another controller just after point handoff. It checks that the duplicate generates no effect, the bound image confirms the new capture without a retry, and the gimbal sends no release after takeover. An independent native heartbeat and landed-state stream must show a disarmed, on-ground Copter throughout the payload operations.

The private output retains input hashes, raw component-side MAVLink, MOOS mail, gimbal actions, images, test result and process/port cleanup evidence. The simulator forwards standard commands to fixture-owned simulated payloads. The test qualifies routing and the end-to-end app operations at those component boundaries.
