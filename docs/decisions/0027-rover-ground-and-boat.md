# 0027 — One Rover provider for ground and surface boats

November 6, 2026. Implemented within owned ArduRover SITL. Qualification belongs to the exact model, fixture, source, binaries and scenario.

Use one `ardurover_control` provider and select either `ardurover` (heartbeat type 10) or `ardurover_boat` (type 11) at startup. The app validates identity and uses the same exact-owned `ardurover` executable/PID/loopback guard for both. Each profile owns its vehicle predicates and 3 m/s or 2 m/s bridge velocity limit. Native modes retain their FC configuration. One app, one FC connection and one host authority service serve the selected profile. MOOS-IvP and pHelmIvP remain stock.

Jeremy's archived BlueBoat bridge informed the planar MOOS interface, original publication stamps, the manual/mode boundary and course/speed-to-MAVLink-velocity conversion. Keep exact zero and current heading at a stop. Do not arm, disarm or select a fallback mode as an implicit side effect of stock helm output. Ordinary bounded all-stop handoffs follow the existing resume rule; expired input and takeover require explicit reacquisition.

Group native mode actions, planar stock helm and direct Guided operations under the same owner. Keep the legacy MOOS request; the versioned interface adds a nearby fixed goal, Guided zero hold, home return and cancellation with original deadlines. Direct routes refresh targets from current position and stop advancing after revocation. Native operations use exact local handoff receipts and fresh observation; Guided velocity targets have no operation ACK. Cancelling local work does not undo effects the FC has received.

Expose native Hold (mode 4) on both profiles and native Loiter (mode 5) only on the boat. A boat can drift when propulsion is stopped. Loiter actively applies propulsion within the FC's configured controller/radius. Acquire from a fresh armed Guided or an eligible preparation mode. Bind the mode-entry request to the original mode and discard it on first Guided evidence, replacement, disarm or takeover. Do not preserve queued work across a handoff or silently resume a retired grant.

Use one grouped native workflow for each model and keep fast feedback outside the full native tier. Qualification must include original stock-helm publications, a real pilot/mode and link boundary, independent position and mode evidence, both return paths, boat correction under a measured current and scoped process/port cleanup. Generated peer tests cover cross-type refusal and zero output from receive-only/shadow. Prior ground qualification remains independently scoped.

The [source trace](../research/2026-11-06-ardurover-profile-source.md), [operator guide](../plugins/ardurover_control.md), and [SITL fixture](../../missions/rover_expanded/README.md) record the details. Other hulls, sail, balance, rally selection, Rover AUTO and hardware have no claim from these fixtures.
