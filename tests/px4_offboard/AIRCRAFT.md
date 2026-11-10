# PX4 aircraft lifecycle qualification

The isolated Linux ARM64 fixture builds the selected clean MAVMOOS source with offline pinned dependencies. It then runs the actual app, installed-style startup-loaded PX4 provider, stock pHelmIvP and independent GCS observer in an owned PX4 v1.17.0 SIH namespace. PX4 CLI sets and reads back simulator parameters and samples the home reference. MOOS requests supply the lifecycle actions and the stock helm supplies the navigation legs. [Provider guide](../../docs/plugins/px4_control.md) · [Acceptance](../../docs/plans/2026-11-06-px4-aircraft/README.md)

The fixture verifies the exact Docker image, stable PX4 package, stock MOOS build, firmware executable, simulator airframe scripts and offline MAVLink archive. The default immutable-volume names and artifact hashes are recorded in the runner and [firmware pin](firmware-v1.17.0.json). Use an absolute path to the pinned MAVLink archive and unique ignored output directories. The second and third cases reuse the same app, module and probe build at the same exact source.

```sh
python3 -B tests/px4_offboard/qualify_aircraft.py --vehicle quadrotor \
  --mavlink-archive "$MAVLINK_ARCHIVE" --output build-aircraft/quad
python3 -B tests/px4_offboard/qualify_aircraft.py --vehicle standard-vtol \
  --built build-aircraft/quad --output build-aircraft/vtol
python3 -B tests/px4_offboard/qualify_aircraft.py --vehicle fixed-wing \
  --built build-aircraft/quad --output build-aircraft/plane
```

Native qualification runs in one leased Linux domain. Audit the retained receipt from the clean repository with `qualify_aircraft.py --audit <evidence-directory>`. Run `audit_aircraft_wire.py <evidence-directory>/native` for independent decoding of GCS bytes, reconciliation of the MOOS command trace and onboard readback of commands, addressed replies and phase-specific accepted Offboard controls. Raw evidence is compressed after scoped teardown. Qualification ledger

The existing [quad](README.md) and [airplane](FIXED_WING.md) fixtures cover bounded helm handoff and Offboard loss with explicit recovery. The focused runtime suites load the actual provider and exercise timestamp expiry, late and foreign evidence, cancellation at the final send and VTOL phase revocation.
