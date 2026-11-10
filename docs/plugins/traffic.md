# ADS-B traffic observation

Enable the optional `traffic` module to view aircraft reported by the selected
ArduPilot FC. It listens for `ADSB_VEHICLE` (246) through the app's existing
connection. The FC source must match the app's selected identity and a fresh
ArduPilot heartbeat. The initial profile uses system 1, component 1. ICAO
addresses identify aircraft in the reports. They are independent of the
MAVLink identity that forwarded them.

Add the module to the `ProcessConfig = iMAVLink` startup list.

```text
target_system = 1
target_component = 1
plugins = telemetry,traffic
plugin.traffic.sample_timeout = 3
plugin.traffic.publish_interval = 0.1
```

These defaults are in seconds. The sample timeout range is 0.25 to 10. The
publication interval range is 0.02 to 2 and must fit within the sample timeout.
The module receives traffic from a configured FC stream and requests no
message rate. It owns no MOOS requests or control services. Traffic injection,
transceiver health and collision avoidance are separate capabilities.

## Reading the contact snapshot

Subscribe to the MOOS string variable `MAVMOOS_TRAFFIC_CONTACTS`. Its JSON
schema 1 carries `contacts`, sorted by numeric `icao_address`. Each entry is
one report and has independent `metadata`. The snapshot has `atomic: false`
because entries can come from different transmissions. `latest_report`
summarizes the last accepted report still in the table and exposes the current
source binding and rejection counters when the table is empty. An empty
snapshot means no retained reports. Airspace coverage is unknown.

| Contact field | Meaning |
| --- | --- |
| `icao_address` | Nonzero 24-bit numeric address |
| `flags_raw`, `simulated` | Wire flags and the sender's simulated bit |
| `latitude_deg`, `longitude_deg` | Geographic degrees with the coordinates-valid bit and both values in range |
| `altitude_m_asl`, `altitude_type_raw` | Reported ASL metres with the altitude-valid bit, a non-sentinel value and a defined type. Type 0 is pressure QNH and type 1 is geometric. |
| `course_deg` | Course over ground, from 0 inclusive to 360 exclusive, with the heading-valid bit |
| `horizontal_velocity_m_s`, `vertical_velocity_up_m_s` | Independent validity bits and sentinels. Vertical velocity is positive up. |
| `callsign`, `squawk_decimal` | Flagged printable, nonblank, NUL-terminated callsign and decimal squawk with digits 0–7 |
| `emitter_type_raw`, `reported_tslc_s` | Wire emitter type and reported time since last communication |

Unavailable interpreted fields are JSON `null`. The ICAO identity and raw
flags, altitude type, emitter type and `tslc` remain visible. Both MAVLink 1
and MAVLink 2 are accepted. MAVLink 2 trailing-zero trimming can omit a
trailing zero-valued field.

The local `effective_at`, `read_at`, `age_s` and `age_basis` describe this
bridge's acquisition of a report. The original stamps survive republication.
The observer expires contacts independently at the configured local timeout,
and an already-expired queued report cannot re-enter the table. At 32 live
contacts, new addresses wait for a slot to expire and existing contacts can
still update. `capacity_rejected`, `expired` and `stale_ingress_rejected`
are cumulative counters. Loss of the bound heartbeat, a host reset or a clock
fault clears the table. A post-boundary heartbeat is required before reports
can populate it again.

The wire `reported_tslc_s` is separate from local acquisition age. The pinned
Copter forwarding path copies the stored age from the received ADS-B record.
Repeated identical reports can advance the local observation ID while the age
of the underlying aircraft track remains unverified. Treat the snapshot as
operator information with `authoritative_for_control: false`.
[MAVLink ADS-B record](https://mavlink.io/en/messages/common.html#ADSB_VEHICLE) ·
[Copter 4.6.3 stored-report forwarding](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/AP_ADSB/AP_ADSB.cpp#L620-L659)

## Reference and acceptance

The pinned MAVROS handler supplies the units and the separation between
message decoding and publication. It stamps each received record with the
ROS node's current time and exposes the wire `tslc`. MAVMOOS retains the host's
acquisition basis and publishes a bounded table for MOOS consumers.
[Pinned MAVROS ADS-B receive handler](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros_extras/src/plugins/adsb.cpp#L75-L139)

- `traffic_observations` loads the module directly and exercises field mapping,
  MAVLink wire versions, malformed reports, expiry, capacity, queue age,
  ordering, clock failure and heartbeat loss.
- `observer_matrix_ardupilot` loads `traffic` with the ordinary app and MOOSDB.
  It exercises concurrent contacts, wrong identities, malformed ICAO, separate
  expiry, heartbeat loss, rebind and zero captured bridge egress in the shared
  generated loopback lifecycle.
- The opt-in `tests/observer_matrix/run_native.py --scenario rx_only --traffic`
  uses the pinned, disarmed Copter 4.6.3 native fixture. An in-process
  `sim:adsb` Shortcut peripheral on serial1 produces several aircraft. The
  FC forwards reports on its isolated serial2 telemetry channel with its
  own 1/1 identity. The fixture leaves avoidance disabled. An independent decoder replays the
  captured wire and checks the app's MOOS fields and simulator signature. The
  app's peer captures egress and has no return path to the simulator.
  [Pinned SIM_ADSB producer](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/SITL/SIM_ADSB.cpp#L150-L285)
