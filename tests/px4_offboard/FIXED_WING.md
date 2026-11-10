# PX4 fixed-wing stock helm qualification

This test runs an airborne control profile on the upstream PX4 v1.17.0 SIH airplane. The fixture performs runway takeoff and landing through the private simulator console. The stock helm supplies a waypoint course, 16 m/s nominal speed and 60 m above home. The profile uses nominal speed to place a position target ahead of the airplane. PX4 owns airspeed control. The test checks fresh independently observed VFR_HUD airspeed, local ground speed and global relative altitude.

The pinned executable and startup scripts come from the existing stable PX4 package. The airplane additionally pins the stock `10041_sihsim_airplane` and `rc.fw_defaults` inputs. The fixture uses the stock `PX4_HOME_ALT=15` site setting and verifies the configured origin and actual nonzero HOME altitude. It waits through the firmware's 120-second post-takeoff HOME correction window and verifies the same bridge and helm processes throughout that setup. It verifies zero accepted setpoints before explicit acquisition.

The acceptance sequence requires at least ten prestream handoffs over at least 1.25 seconds before independent Offboard evidence. Each of two acquisitions must produce at least 65 m eastward travel with 10–35 m/s measured airspeed, 8–35 m/s ground speed, and 20–130 m above home. A real stock helm all-stop must pass through the bounded handoff, send a continuation target and resume with fresh helm output in Offboard. Silence must retire permission and the FC must enter native Hold while retaining Offboard intent. New stock inputs and an acquire from retained-intent Hold must leave the bridge latched. Observed native Hold handback permits a fresh acquire. The fixture takes over from the active second leg by selecting native Hold. Fresh stock helm output must leave the retired grant latched with no new sends. A third brief acquire exercises explicit release and independently observed native Hold, followed by fixture-owned Land, touchdown and disarm.

Use a clean source revision and the same local stock MOOS and firmware volumes as the earlier stable qualification. Point `--mavlink-archive` at a local copy of the project's hash-pinned archive. The launcher uses the pinned local Linux ARM64 tools image and takes the existing Linux-domain lease before native execution. Both containers require a read-only root and only loopback. Stock dependencies, source, and firmware are read-only. The candidate app, plugins and operator probe are built out of tree and then mounted read-only for flight.

```sh
python3 -B tests/px4_offboard/qualify_fixed_wing.py \
  --mavlink-archive "$MAVLINK_ARCHIVE" --output build-px4-fixed-wing
python3 -B tests/px4_offboard/qualify_fixed_wing.py \
  --built build-px4-fixed-wing --vehicle quadrotor --output build-px4-quad-regression
python3 -B tests/px4_offboard/qualify_fixed_wing.py --audit build-px4-fixed-wing
```

Use `--build-only` to seal the Linux artifacts before reserving the native test window. Reuse those artifacts with `--built` at the same source revision. The legacy stable multicopter fixture remains in [run_stable_candidate.py](run_stable_candidate.py) and retains its original artifact pins. The new multicopter regression exercises the newly built app against the same stable stock dependencies.

Archive `qualification.json`, the build manifest, container and lease receipts, and the complete `native/` result. The raw independent GCS capture, MOOS trace, generated mission, native console and onboard ULog sit under `native/`. Build artifacts are reproducible from the recorded source and inputs.
