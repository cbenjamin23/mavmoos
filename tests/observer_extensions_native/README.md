# Stable Copter passive observer bench

One owned disarmed quad boot groups the optical flow, terrain and navigation monitor witnesses. The runner uses the verified Copter 4.6.3 source manifest and platform build attestation. Its dedicated native UDP socket receives actual firmware datagrams and copies them to the real app from a separate local peer. The peer captures every return datagram and sends only to the owned app. The fixture records the same stock MOOS dependency before and after execution.

The simulator enables its optical flow backend with `SIM_FLOW_ENABLE=1` and `FLOW_TYPE=10`. A fresh output directory gives it empty terrain storage. `TERRAIN_ENABLE=1` and `TERRAIN_OPTIONS=0` permit its own terrain requests, and the observer leaves them unanswered. Preconfigured serial message intervals supply the telemetry stream.

| Native frame | Required evidence |
| --- | --- |
| `OPTICAL_FLOW` (100) | Fresh reports from simulated sensor 0 with quality 51 and wire values matched to MOOS. The stable sender puts boot milliseconds in the wire field named `time_usec`, so the audit compares the raw value. |
| `TERRAIN_REQUEST` (133) | Positive spacing, the requested grid mask and its block count matched to independent bytes. |
| `TERRAIN_REPORT` (136) | An unavailable current-location tile, raw reported height, and pending/loaded counts corroborated. |
| `NAV_CONTROLLER_OUTPUT` (62) | Reported diagnostics matched to native frames, including the stable sender’s raw airspeed-error zero. |

`OPTICAL_FLOW_RAD` and the local/global position target observers retain their generated-fixture scope in this bench. The runner records whether the native wire included those families and rejects an app observation for any family absent from the independent wire. Flight remains disarmed and on the ground. Qualification depends on valid original observation ages, two or more fresh observations per required family, zero captured egress, immutable source and artifact readback, and cleanup of exact owned processes and endpoints.

Run from a clean selected MAVMOOS source with the app, modules and `observer_extensions_native_probe` already built.

```bash
python -B tests/observer_extensions_native/run_native.py \
  --source-root "$PWD" --ardupilot-root /path/to/verified/copter \
  --moos-root /path/to/pinned/moos-ivp --build /path/to/mavmoos/build \
  --firmware-manifest "$PWD/config/firmware/arducopter-4.6.3-quad.json" \
  --firmware-attestation /path/to/native-attestation.json \
  --output /path/to/fresh/run
```

[Stable optical flow sender](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/GCS_MAVLink/GCS_Common.cpp#L2866-L2900) · [Stable terrain sender](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/AP_Terrain/TerrainGCS.cpp#L104-L149) · [Stable navigation sender](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/ArduCopter/GCS_Mavlink.cpp#L206-L222)
