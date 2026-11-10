# Vehicle support

MAVMOOS receives MAVLink telemetry over UDP. This table covers active control verified in software-in-the-loop (SITL) on specific firmware and vehicle configurations. The plugin guides list the available commands, helm inputs, and conditions for each workflow.

| Autopilot | Tested firmware and vehicle | Control guide |
| --- | --- | --- |
| ArduCopter | 4.6.3 quad, hexa plus, and octa plus | [Copter](plugins/arducopter_control.md) |
| ArduRover | `04cf6486` ground rover and differential-thrust surface boat | [Rover](plugins/ardurover_control.md) |
| ArduPlane | 4.6.3 and 4.7.2 conventional fixed-wing | [Plane](plugins/arduplane_control.md) |
| ArduPlane QuadPlane | 4.6.3 fixed-motor VTOL, with a separate hover configuration | [QuadPlane](plugins/arduquadplane_control.md) |
| ArduSub | 4.7.2 vectored and vectored 6DOF | [Sub](plugins/ardusub_control.md) |
| PX4 | 1.17.0 SIH quadrotor, conventional fixed-wing, and standard VTOL | [PX4 aircraft](plugins/px4_control.md) |

The [command coverage matrix](development/control-coverage.md) shows which actions, handoffs, and failure cases the tests exercise. The [release report](implementation/2026-11-10-release-28/README.md) records the 28 simulator scenarios from the latest full run. [PX4 Ackermann](plugins/px4_rover_control.md) is an experimental option tested separately from the release matrix.
