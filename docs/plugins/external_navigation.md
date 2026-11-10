# External navigation

`external_navigation` feeds one named local producer's odometry into the estimator on the owned, disarmed ArduCopter quad SITL bench. The shared host owns the connection, one pending sample, the original deadlines and the final send checks. Ordinary telemetry continues to come from the flight controller. The measured qualification covers the pinned 4.6.3 horizontal ExternalNav position and velocity workflow. [MAVLink ODOMETRY](https://mavlink.io/en/messages/common.html#ODOMETRY) · [Pinned Copter packet handler](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/GCS_MAVLink/GCS_Common.cpp#L3876)

The fixture opens the exact direct loopback route to its own simulator process. Select `telemetry,external_navigation`, disable firmware requests and configure `plugin.external_navigation.source` with the local MOOS client's exact name. The host admits this opt-in only on the dedicated SITL composition with fresh disarmed and landed evidence. The fixture uses ordinary MOOS mail at time warp one. [Native runner](../../tests/external_navigation/run_native.py)

The configured source publishes `MAVMOOS_EXTERNAL_NAV_REQUEST` to begin or stop a run.

```text
schema=1,action=begin,run=1
schema=1,action=stop,run=1
```

A begin opens a lease of up to 30 seconds. Run IDs are positive and strictly increase within one app process. During a run the same source publishes `MAVMOOS_EXTERNAL_NAV_INPUT` at 10 Hz. Each input is a string with the following fields and units.

| Field | Contract |
| --- | --- |
| `schema`, `run`, `seq` | Schema 1, the open run ID and a strictly increasing positive sequence. |
| `capture_us` | Original measurement timestamp from the same host's steady clock in microseconds. The producer captures and then publishes, with up to 5 ms conversion tolerance. |
| `frame` | Literal `local_frd_body_frd`. Position uses a fixed local Forward/Right/Down frame. Velocity and rates use body Forward/Right/Down. The quaternion rotates from the body frame into the local frame. The native fixture aligns local X with north. |
| `position`, `velocity`, `rates` | Slash-separated X/Y/Z vectors. Bounds are ±30 m, ±3 m/s and ±2 rad/s per axis. |
| `quaternion` | Slash-separated W/X/Y/Z. Squared norm must be within 0.002 of one. The encoder normalizes it. |
| `position_var`, `angle_var`, `velocity_var`, `rate_var` | Three slash-separated positive diagonal variances. Bounds are 0.0025–4 m², 0.0001–1 rad², 0.0025–4 (m/s)² and 0.0001–4 (rad/s)² respectively. |
| `quality` | Integer 50–100 supplied by the measurement producer. |

The packet uses MAVLink 2 `ODOMETRY` with estimator type VISION, reset counter zero, and zero covariance cross terms. A run stays in a fixed local frame. The measurement timestamp stays in the producer's boot-relative steady-clock domain. The app combines it with the mapped MOOS publication time and gives the sample at most 250 ms from the earlier of the two. The host chooses the newest waiting sample and limits handoff to 20 Hz. Neither queue replacement nor the transport handoff renews age. Copter 4.6.3's generic MAVLink VisualOdom backend uses the pose covariance diagonals to derive position and orientation error and applies its configured velocity noise when submitting velocity to the EKF. [Pinned VisualOdom backend](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/AP_VisualOdom/AP_VisualOdom_MAV.cpp)

The producer's valid measurements keep the run live. After 500 ms of silence, explicit stop, expiry, lost ground eligibility or a connection reset, pending input is retired. Resuming requires a new run ID and a new measurement captured after that begin. `MAVMOOS_EXTERNAL_NAV_STATUS` exposes the current run, reason, and counts of admitted samples, local handoffs and rejections. The host also exposes its accepted and replaced counts in `MAVMOOS_RUNTIME_STATUS`. Native estimator use is verified independently.

The fixture sets generic MAVLink VisualOdom and selects ExternalNav for horizontal position and velocity. It retains native barometric vertical estimation and compass yaw. The independent serial link records a fused position change while simulated GPS stays stationary, then records loss of horizontal aiding after input silence. DataFlash independently binds the accepted pose and velocity to the original measurement timestamps. This bench also checks that stale input, a retired run and an explicitly stopped run produce no new measurements. [ArduPilot external estimation configuration](https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html)

Build `iMAVLink`, `external_navigation_moos_probe` and the configured modules against the pinned stock MOOS root. The two CTest cases `external_navigation_provider` and `external_navigation_runtime` exercise the module and host boundary separately. Run the native fixture with the exact build and firmware attestation, a clean app revision and a fresh evidence directory.

```sh
python -B tests/external_navigation/run_native.py \
  --build "$build" --moos-root "$stock_moos_root" --ardupilot-root "$pinned_copter_root" \
  --firmware-attestation "$pinned_copter_attestation" --expected-app-revision "$(git rev-parse HEAD)" \
  --output "$fresh_evidence_dir"
```
