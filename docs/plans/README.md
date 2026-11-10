# Implementation plans and execution status

Plans describe proposed work and acceptance evidence. They do not authorize implementation, establish working features or qualify a vehicle. The subsequent receive-only and P2 laboratory increments are authorized and recorded separately in the [execution index](../implementation/README.md).

| Plan | Status | Intended first result |
| --- | --- | --- |
| [Direct UDP / initial ArduCopter](2026-10-29-direct-udp-copter/README.md) | Dated draft preserved; P0-A/P1 and P2 local evidence linked separately | Useful receive-only MOOS telemetry, then a stock-helm/Copter SITL vertical slice with explicit control/loss behavior |
| [PX4 and ArduPilot vehicle coverage](2026-11-06-vehicle-family-coverage/README.md) | Project goal recorded. Profile order and additional qualification are proposed. | PX4 fixed-wing control and a PX4 Rover interface study |
| [PX4 fixed-wing and Rover interface](2026-11-06-px4-fixed-wing/README.md) | Implemented airplane profile and qualified direct Rover firmware workflows | Airborne stock-helm Offboard, Ackermann mission and generic MAVLink Offboard. Evidence |

Keep dated plans and execution evidence separate. Revisit a plan when the actual platform/topology or a failed acceptance gate contradicts its assumptions; do not infer completion from a checklist's presence.
