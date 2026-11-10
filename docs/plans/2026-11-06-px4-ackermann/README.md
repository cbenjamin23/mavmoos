# PX4 Ackermann control and handback

November 6, 2026. Deliver one `px4_rover_control` provider for the pinned PX4 v1.17.0 SIH Ackermann airframe. Use stock `pHelmIvP` and the shared mission transfer provider. The initial native workflow verifies and runs two to eight ground waypoints and supports pause, resume, abort and release. The live workflow consumes planar stock helm course and speed. Both operate through the same host and selected FC connection. [Direct firmware witness](../../research/2026-11-06-px4-rover-interface.md)

Live Offboard uses velocity-only local NED setpoints, with forward speed bounded to 1.5 m/s and a fresh heading check before accepting a forward course. Qualify the stock airframe and its forwarding and loss settings by addressed readback. Use bounded zero-velocity prestream and an independently observed Offboard transition. On release, stale intent or link loss, retire the stream and let the pinned FC select native Hold. A new grant requires explicit acquire, fresh active and intended Hold and fresh stopped evidence.

Stopped evidence uses an advancing dwell of selected raw GNSS sensor reports. Bound reported speed, reported accuracy and position excursion, and clear the dwell on invalid, stale or regressed evidence. The native bench compares this criterion with SIH ground truth. Record the usable receiver envelope and the distinction between a qualified stationarity decision and a bound on actual vehicle speed.

The mission host retains the verified plan identity and an exclusive lease. The fixture supplies explicit native arming from Hold. Start consumes exact two-to-eight-item readback with all rows executable. Pause, resume and release each require observed results from newer evidence. On uncertainty, retain the lease and the durable recovery marker. A stop-only Hold can be requested while Mission owns the vehicle. After link loss or restart, current safe mode and a new stopped dwell support explicit handback. Read-only setup observations may continue while effects are fenced.

## Acceptance

- Verify the actual loaded app, typed mission proof and command gates with exact selected identity, settings, deadlines and input age. Include invalid or duplicate GPS, incompatible drive geometry, an ACK without the requested mode, stale/foreign acknowledgements, queued takeover, conflicting mission and helm requests, and recovery from uncertainty.
- In owned Linux SITL, verify raw GNSS stops against advancing independent ground truth. Confirm native Hold as the configured Offboard fallback from actual motion. Observe and record active and intended mode after fallback.
- Through `iMAVLink`, upload and read back the plan, start from explicit armed Hold, observe motion, stop and pause, resume and reach the final item, and release with fresh Hold and stop evidence. Run actual stock-helm movement, turn, ordinary handoff and loss, followed by explicit recovery and final scoped teardown.
- Preserve the frozen source, firmware, vehicle model, build and stock helm hashes, native attempts and independent wire and onboard evidence. Regress the shared PX4 and Copter authority paths in local software CI.
