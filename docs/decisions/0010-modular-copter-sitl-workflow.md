# 0010: Modular Copter SITL workflow

Date: November 3, 2026. Status: accepted implementation scope and selected simulation defaults; hardware operation excluded.

## Authorization

The user requested: “go ahead with the full Copter SITL workflow using three subagents. Use the defaults for now, keep stock pHelmIvP unchanged, and archive the decisions and test results. No real vehicle control.” This supersedes 0008's P2-only execution limit. Preserve earlier discussions, profiles and research as dated history.

## Architecture and choices

- One AppCasting host loads feature shared libraries at startup through a versioned create/destroy factory interface. Standard modules are telemetry, commands and stock-helm compatibility. Configuration selects modules; all share vehicle observations and host-authorized command/navigation services. This replaces the earlier built-in-only first-slice recommendation. The C++ interface is a development ABI requiring matching builds; live reload and process isolation are not provided.
- Stock MOOS-IvP/pHelmIvP stays unchanged. The ordinary external `BHV_ConstantAltitude` behavior from the UAV reference supplies the altitude objective through the standard IvP behavior API. The helm adapter labels its independently received input set `ARRIVAL_GROUPED`, never solver-atomic.
- Default operation is receive-only. Shadow mode computes proposed navigation without transmitting. Effectful operation requires explicit `simulation_only=true`, loopback addresses, target1/component1 ArduPilot quad identity, and an actual native `arducopter` PID owning the configured peer UDP socket. The socket owner is checked at startup and every write. This is an accidental-misrouting guard for trusted local simulation, not a sandbox for malicious plugins or a hardware backend.
- Use the existing native ArduCopter build and pin its observed identity: source `04cf6486e8d42ce138de816e8466244346ed77cb`, boot `ArduCopter V4.7.0-dev (04cf6486)`, QUAD/PLUS. Exact binary hashes, mission/parameters, routes and logs belong to each execution record. This is a specific development firmware candidate, not a support claim for all Copter releases.
- Candidate simulation defaults: 1x time; course-facing; home-relative target5m; route speed1m/s; 0.5s bounded normal-handoff window; 2s helm publication lifetime and 0.5s field spread. Explicit acquisition and fresh post-boundary fields are required. A normal pause may recover within its window; expiry, external mode changes and resets require explicit reacquisition.
- Command operations are explicit requests with unique IDs. Admission, local send, ACK, observed state and operation completion are separate. Initial actions are guided, arm, takeoff, native Loiter hold, land, RTL and disarm. Do not force arm or disarm an airborne vehicle. One attempt per admitted command; general retry/mission/parameter services remain future work.
- The integration reuses P2's pure Copter translation/encoding. Its common runtime owns a small bounded command queue and one latest waiting navigation target with original expiry and final current-state/time checks. The full P2 `ManagedEgress` API remains a separately tested laboratory component; do not transfer all of its detailed receipts/guarantees to the new runtime by implication.

## Scope and evidence

Build/test the modules concurrently, integrate through shared services, and demonstrate the complete simulated lifecycle with ordinary MOOS requests. Include source/timestamp/expiry/override/reset tests, real stock-helm shadow zero-output capture, normal-handoff recovery, expired-window latching, native Loiter without automatic recapture, explicit reacquisition, landing and disarm. Independently corroborate the flight using another FC telemetry endpoint which issues no packets.

Use implementation and evidence for actual outcomes. First-pass failures remain archived. A passed workflow is configuration-specific simulation evidence. Hardware, serial control, Linux runtime qualification, other vehicles/firmware, load/timing guarantees and the full planned fault catalog remain outside this evidence.
