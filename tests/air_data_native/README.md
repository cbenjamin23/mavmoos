# Manual Plane airspeed witness

One disarmed native `plane` boot exercises the actual `iMAVLink` process in RX_ONLY with the sole startup provider `air_data`. Stock MOOSDB and pAntler start the app and the existing observer probe subscribes to `MAVMOOS_AIRSPEED`. The conventional Plane control workflow and the generated `HIGH_LATENCY2` vectors have separate acceptance. [Plane source and build preflight](../../docs/plans/2026-11-06-overnight-plugin-queue/arduplane-source-preflight.md) · [Air-data contract](../../docs/plugins/air_data.md)

## Pinned producer and route

- Use the attested Darwin ARM64 `Plane-4.6.3` executable built from `3fc7011a7d3dc047cbb17d8bd98ee94577d144c6`. The runner verifies its exact executable, build-attestation and stock `plane.parm` SHA-256 values before reserving any ports. The separate overlay selects conventional Plane, configures two SITL differential-pressure backends with `ARSPD_TYPE=100` and `ARSPD2_TYPE=100`, and pins both backends' use and simulated noise/failure settings. Both sensor IDs must appear from the actual producer. [Pinned Plane defaults](https://github.com/ArduPilot/ardupilot/blob/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6/Tools/autotest/models/plane.parm) · [Pinned backend selection](https://github.com/ArduPilot/ardupilot/blob/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6/libraries/AP_Airspeed/AP_Airspeed.cpp#L343-L369) · [SITL backend data and temperature](https://github.com/ArduPilot/ardupilot/blob/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6/libraries/AP_Airspeed/AP_Airspeed_SITL.cpp#L3-L42)
- A private boot-time `message-intervals-chan1.txt` schedules 295 every 200 ms, position every 250 ms and extended state every 500 ms. The pinned sender emits one enabled sensor per call and advances its per-channel sensor cursor. The runner receives an owned FC datagram, records its original bytes and actual receive time, then copies it through the separate loopback peer to the app. The return direction terminates at the egress capture socket. The app sends no stream request and the witness sends zero datagrams to the FC. [Pinned scheduler](https://github.com/ArduPilot/ardupilot/blob/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6/libraries/GCS_MAVLink/GCS_Common.cpp#L6616-L6687) · [Pinned sender](https://github.com/ArduPilot/ardupilot/blob/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6/libraries/GCS_MAVLink/GCS_Common.cpp#L2364-L2412)
- The independent pymavlink v2 `all` dialect decodes AIRSPEED 295 with CRC extra 234. The pinned simulator builds the `all.xml` dialect, which includes the development message at this pin. The installed Python `ardupilotmega` dialect omits it. The runner verifies the independent decoder's CRC and field layout before launch and replays the original wire and forwarded suffix after the capture closes. [Pinned Waf dialect](https://github.com/ArduPilot/ardupilot/blob/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6/wscript#L724-L733) · [MAVLink AIRSPEED](https://mavlink.io/en/messages/common.html#AIRSPEED)

Pass requires both sensors with at least eight distinct fresh, time-matched MOOS observations each, all associated with actual independently decoded wire frames in one session. The capture samples the Darwin raw monotonic clock used by the app's C++ steady clock. This puts both actual read timestamps on the same clock for the 300 ms comparison. The audit checks CAS, raw differential pressure, conditional Pa-to-hPa interpretation at the app's nine-significant-digit JSON precision, temperature, flag bits and actual payload length. It checks that republished records retain original data and times. Grounded and disarmed state must agree between raw wire and app reports, the module catalog must contain only `air_data`, and both app counters and captured egress must stay at zero. Exact PID/socket ownership, teardown and input/output hashes are recorded in the private receipt.

## Invocation

Use the existing Python environment with pymavlink and a frozen CMake build containing `bin/iMAVLink`, `bin/observer_matrix_probe` and `plugins/mavmoos_air_data.so`. Select the actual clean application build source root and revision. The runner records its own test source revision separately.

```sh
"$plane_python" -B tests/air_data_native/run_native.py \
  --plane-build-root "$plane_build_root" --moos-root "$stock_moos_root" \
  --build "$frozen_app_build" --app-source-root "$app_source" \
  --expected-app-source-revision "$app_revision" \
  --expected-app-sha256 "$app_sha" --expected-probe-sha256 "$probe_sha" \
  --expected-module-sha256 "$module_sha" --check-inputs
```

The preflight flag performs source, dependency and byte checks before reserving sockets or starting the simulator or MOOS. In the centrally reserved native window, replace `--check-inputs` with `--output "$fresh_private_run"`. The runner also acquires the shared native lease. Keep the fresh output directory with its `result.json`, boot overlay, interval file, raw MAVLink, datagram offsets, probe output, egress capture and ownership evidence.
