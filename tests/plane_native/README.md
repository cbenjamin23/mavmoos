# Owned conventional Plane scenarios

The manual Darwin ARM64 cell uses the [pinned native build](../../docs/implementation/2026-11-06-plane-build/README.md), stock MOOS-IvP and the built `BHV_ConstantAltitude` from `moos-ivp-uav-base`. Build `iMAVLink`, `plane_native_probe`, the three mission modules and `mavmoos_plane_fault_tap` from the clean checkout being qualified. Use a pymavlink environment with MAVLink 2 `all` dialect AIRSPEED 295 support.

```sh
rev=$(git rev-parse HEAD)
caffeinate -i "$plane_python" -B tests/plane_native/run_native.py \
  --plane-build-root "$plane_build" --moos-root "$stock_moos" \
  --build "$mavmoos_build" --expected-app-revision "$rev" \
  --behavior-dir "$uav_base/lib" --output "$fresh_case_output" --case normal

"$plane_python" -B tests/plane_native/replay.py "$fresh_case_output"
```

The native runner holds the shared host SITL lease and runs the simulation at 1×. It records the pinned artifacts before and after the run, verifies actual firmware parameter readback and empty mission and rally plans, and separates serial0 bridge traffic, serial1 passive observation and serial2 simulator operator traffic. Operator writes are scoped to the owned socket with system ID 249. The configured GCS watchdog watches system ID 245. The independent monitor records zero writes.

| Case | Evidence required |
| --- | --- |
| `prelude` | Normal firmware pre-arm checks, arm and TAKEOFF through the simulator operator, then independently observed airborne Loiter. This case needs only `--plane-build-root`, `--moos-root` and `--output`. |
| `rx-only`, `shadow` | Armed Loiter, genuine stock helm scalars and zero bridge egress. Shadow additionally reaches eligible helm state. |
| `normal` | Explicit acquisition, paired wire target and FC readback, two physical flight legs, bounded stock all-stop handoff, release and native Loiter orbit, and explicit Loiter/RTL with an observed return and home orbit. |
| `silence`, `invalid` | Stopped stock producer or out-of-envelope speed respectively, native Loiter handback while FC telemetry continues, and explicit reacquisition with fresh stock output. |
| `takeover` | Owned simulated RC switch to FBWA, maintained watchdog across the healthy pilot window, old Guided target released while outside Guided, and later RC Loiter plus explicit reacquisition. |
| `loss-rx`, `loss-tx`, `loss-handback`, `loss-process` | Separate inbound loss, outbound loss, unconfirmed Loiter handback and bridge process death/restart. Each observes the owned watch heartbeat, independent RTL with DataFlash reason 5 after loss during Guided, physical return, and explicit reacquisition after recovery. Inbound-only loss can deliver one unconfirmed Loiter request while the last FC heartbeat remains fresh. |
| `lifecycle` | Start on the runway. Use actual `iMAVLink` requests for Manual, arming, verified mission takeoff, stock helm, RTL and Loiter, direct waypoint, Guided orbit, physical airspeed changes, verified landing, touchdown, disarm and Manual. Reject malformed runway geometry, an unverified proof, in-air disarm and unsafe airspeed. |
| `native-autoland` | In the opt-in 4.7.2 fixture, repeat the full lifecycle and observe literal native AUTOLAND 26 through touchdown and Manual handback. The exact manifest also verifies the initialized HOME row, release identity, native approach parameters and stock helm. |
| `operator-nudges` | Publish a fresh projected home marker without control output. Reject direct altitude adjustment while stock helm owns Guided. From idle Loiter, project a +45° leg, measure travel, then request a +12 m height change and measure climb before native Loiter handback. |
| `helm-tasks` | Repeat the operator case and record at least 1.25 s of fresh native altitude within 2 m of the requested target after handback. Exercise stock Waypoint return, survey and Voronoi with an exact post-selection active flag, separate acquire, paired FC target, physical movement, cancellation, original expiry and external FBWA takeover. |

Trajectory acceptance requires at least 3.8 seconds of Guided tracking with ground course within 23° of helm intent, 16–29 m/s ground speed, height within 11 m of the requested value, healthy active sensor 0 samples in 16–30 m/s, and at least 68 m of horizontal travel. Orbit evidence requires at least 10.5 seconds of independent trajectory with a clockwise sweep of at least 140°, radius 49–120 m and height within 20 m of the FC's reported target. The watchdog case accepts a 4.7–6.65 s observed heartbeat-to-RTL interval around the configured five-second watchdog. The physical RTL gate requires at least 70 m of approach to the bound home and a final distance within 160 m. These are observed fixture bounds.

`result.json` binds the scenario, native and bridge inputs, per-artifact SHA-256, process/socket teardown and mode attribution. The output preserves raw application datagrams, independent FC frames, MOOS mail and the native DataFlash log. The replay verifies hashes, the independently decoded native telemetry, application datagrams and DataFlash cause evidence. Each attempt uses a new output path so a failed attempt can be retained with its original receipt.
