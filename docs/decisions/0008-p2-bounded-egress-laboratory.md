# 0008: P2 bounded-egress laboratory

Date: October 29, 2026. Status: accepted P2 scope; local implementation choices are not operational vehicle approval.

## Authorization

The user requested: “implement and test bounded sending, expiry, and cancellation against fake I/O and a loopback peer. Archive the results, keep stock pHelmIvP unchanged, and leave vehicle control disabled.”

This extends 0007's P1-only implementation boundary to P2. It does not authorize helm integration, a real FC route, SITL control or hardware work.

## Choice

Build a separate experimental egress library and test executables. Keep it out of iMAVLink's link dependencies and expose no MOOS send/enable request. The concrete real-I/O class is restricted to 127.0.0.1; tests bind and hold their own receiving peer throughout the run.

Use one serialized owner, typed target/probe submissions, one latest waiting NAV, one response, four FIFO probes, and sixteen preallocated reserved terminal records. Final dispatch checks scope, original expiry, current observations, permission and selected age. Make one nonblocking datagram attempt without a hidden FIFO, repeat timer or transport retry.

Routine fencing revokes NAV while preserving independent probes/RX and a separately granted bounded response. All-class fencing/close revoke every class. Application and local quiescence are explicit boundaries, with already accepted bytes retained as possible residuals. A new generation requires an applied fence ID, fresh tokens and strictly post-boundary input evidence.

Tests supply the FC state and permission assertions; there is no real authority/acquisition coordinator. The draft Copter transformation and conditional MSL hold are encoded only for fake I/O and owned loopback. No arming, takeoff, mode change or mission/parameter command is implemented.

## Alternatives and consequences

An async FIFO or vendor patch would add cancellation/buffer lifetime boundaries that this narrow synchronous lab does not need. No libmavconn/MAVSDK/Asio library or generic plugin framework is introduced. The generated MAVLink codec and stock MOOS pins remain unchanged.

A result is evidence of local admission/attempt/acceptance, not FC receipt or physical behavior. A fence cannot recall a datagram; a host acknowledgement of reconciliation is not native FC proof. Foreign-thread callers need a future bounded handoff design; they cannot call this owner-only API directly.

## Evidence and remaining work

The P2 implementation, interface and archived results record tests, traces, exact packets, capacities, source/binary identity and limits. The P1 MOOS zero-output regression remains part of the suite; stock MOOS-IvP/pHelmIvP are unchanged.

Actual source-clock qualification, NAV origin/mapping, native health predicates, command transactions, acquisition, send-gap loss policy, simulator/firmware/parameter snapshots and physical behavior remain unresolved. P3–P6 are not passed or automatically authorized. Preserve the dated design/profile/research and P1 evidence unchanged.
