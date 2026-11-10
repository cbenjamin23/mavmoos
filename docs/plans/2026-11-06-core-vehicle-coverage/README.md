# Core vehicle completion work

November 6, 2026. Scope follows supported ArduPilot and PX4 vehicle workflows and the pArduBridge source crosswalk at `e34a62e`. Track capabilities at the firmware, frame and workflow level. The existing PX4 Ackermann case stays an opt-in pinned SITL integration. [Vehicle coverage matrix](../2026-11-06-vehicle-family-coverage/README.md) · [PX4 vehicle support tiers](https://docs.px4.io/main/en/airframes/index)

The qualification ledger records the delivered workflows, native closeouts, final software results and remaining coverage by profile.

| Workflow | Acceptance evidence |
| --- | --- |
| Conventional ArduPlane parity | Use the normal MOOS interface to arm, launch the verified takeoff path, fly direct and stock-helm guidance, change speed, orbit and return, land and disarm. Observe flight, final states and cancellation independently. Record exact native AUTOLAND availability by release. |
| ArduPilot QuadPlane | Establish hover, fixed-wing flight and both transition directions with explicit ownership, phase-specific commands and evidence that queued work from the previous phase cannot send. Exercise takeover and loss. |
| PX4 multicopter and fixed-wing | Complete native arm, takeoff, hold/return, land and disarm on each applicable profile, alongside stock helm. Confirm the actual FC mode, position and landed state. |
| PX4 VTOL | Fly a supported standard VTOL through hover, fixed-wing flight and both transitions. Independently measure phase, progress, handback and loss. |
| ArduCopter Precision Loiter | Start with a detected offset beacon and verify ordinary Loiter holds away from it. Enable through the public request and require measured travel and a stationary dwell at the beacon. Disable, reposition and verify it stays away in native Loiter while the beacon remains detectable. Keep command acceptance and capture evidence separate. |
| ArduCopter mission and frame coverage | Enumerate supported plan items, lifecycle and applicable airframes. Require a verified plan and effect evidence for additional capabilities. Track cases by exact frame and release. |
| ArduRover ground and boat | Execute a verified native mission through start, pause, resume, completion, abort and handback. Confirm mode, cursor and motion, preserve the exclusive lease through uncertainty and test restart. Verify any additional steering or propulsion variants with their own evidence. |
| ArduSub | Verify native Surface, measured ascent, permission retirement and handback in an owned simulator. Qualify additional layouts against the motion and depth contract they can actually meet. |

Use stock pHelmIvP and owned isolated SITL. A command ACK establishes the command result. Independent observations establish vehicle effect. Test ordinary operation, rejected inputs, timeout, stale evidence, competing control, recovery and cleanup. Retain compact receipts and useful failures at exact source, firmware and artifact hashes. [Control qualification](../../development/control-qualification.md)
