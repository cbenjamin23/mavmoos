# MAVMOOS local SITL release qualification

The release candidate is a clean-source Linux runtime kit, a runnable stock-helm Copter mission, and one qualification record for the same MAVMOOS commit across the supported simulation profiles. Each cell records its own platform, firmware, frame and artifacts. All control and fault injection use owned SITL endpoints.

## Acceptance

| Deliverable | Required evidence |
| --- | --- |
| Linux runtime kit | Build from the selected Git commit and pinned inputs in a clean local Linux environment. Unpack into a new directory, load the installed built-in modules through the real app, verify the default receive-only route and bind the package and artifacts by SHA-256. |
| Copter operator mission | Use the documented launcher and stock pHelmIvP. Observe takeoff, waypoint travel and stationkeeping, bounded normal resumption, external simulated pilot takeover, explicit reacquisition and landing. Complete a read-only parameter request while the helm owns motion. Capture independent FC and MOOS observations. |
| Ground Rover and surface boat | Reuse the two pinned native workflows. Prove motion and authority loss, native handback and bounded reacquisition for each frame. |
| Conventional Plane | Run the nominal path and the separate pilot takeover scenario on the pinned Plane build. Verify Guided motion, native mode observation and scoped teardown. |
| Vectored Sub | Verify positive-down motion, captured hold, native handback, heartbeat loss and Manual takeover with stock helm. |
| PX4 multicopter | Run the Offboard workflow with the candidate Linux app and the same stock Linux helm as the kit. Observe entry, measured motion, silence and explicit recovery. |
| Release gate | Run the complete registered software tier. Require the same exact MAVMOOS revision across the software, package and native receipts. Check artifact, raw evidence and teardown bindings and reject incomplete or mixed-revision runs. |

Use the existing pinned firmware cells as the baseline. A different upstream release is a separate candidate cell with its own artifact, configuration and observed outcomes. Count a firmware change only after running the comparable scenario. Keep fast software feedback and the full native matrix separate so a routine edit can use focused checks.

Keep one working build per platform and one cache per dependency pin. The campaign preserves logs, receipts, relevant raw traces and the verified runtime kit. Scratch source copies and staged inputs are rebuildable. Record enough identity information to run the same cell again without accumulating a full source export per attempt.
