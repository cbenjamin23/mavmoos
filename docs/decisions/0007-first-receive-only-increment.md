# 0007: First receive-only implementation increment

Date: October 29, 2026. Status: accepted limited scope; implementation choices apply only to this increment.

## Context and authorization

After the direction review and [phased plan](../plans/2026-10-29-direct-udp-copter/README.md), the user said “Go ahead”. The announced working scope is the first increment: P0-A readiness and P1 receive-only UDP/MOOS telemetry, built and checked before moving toward control. This supersedes record 0001's documentation-only restriction for that increment. It does not accept every proposed mechanism, future phase or flight profile.

## Decision and local implementation choices

Deliver iMAVLink as a small AppCasting application, with a receive/store library and test-only fixtures. Use the unmodified local stock MOOS-IvP dependency, generated MAVLink C headers pinned by commit/archive hash, and native bounded nonblocking POSIX IPv4 UDP on the selected macOS host. Keep one configured peer and exact target IDs/expected heartbeat fields. Expose raw records with units, identity and freshness limitations.

The production application contains no MAVLink transmit capability, helm adapter, command requests or NAV projection. Runtime verification may create only short isolated localhost MOOS/UDP fixtures and must clean up their processes/ports. Stock pHelmIvP remains unchanged under 0004.

## Alternatives and consequences

Asio/libmavconn/MAVSDK are unnecessary dependencies for the passive first slice. This choice does not establish a control transport, queue policy, timing guarantee or Linux qualification. A later sender must separately satisfy P2's deadline/attempt/fence tests; it cannot inherit a pass from the RX-only ledger's constant zero counters.

The broader modular design remains provisional. There is no shared-library loader, frozen plugin ABI or second backend. Decoding/provenance remain independent of MOOS; Copter control laws have not entered the UDP layer.

## Evidence and unresolved work

The increment record and evidence package state the platform, revisions, source hashes, fixtures, MOOS publications, zero endpoint output and cleanup. Records 0005/0006 remain proposals for later effectful transport/policy work; they are not promoted wholesale.

Real FC/SITL topology, firmware/parameters, navigation mapping, authority acquisition, egress cancellation, motion/loss behavior and physical freshness remain unresolved and unqualified. P0-B/C/D and P2–P6 are not passed. Preserve dated research, original notes and draft plans unchanged.
