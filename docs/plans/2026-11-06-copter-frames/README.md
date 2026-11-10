# Standard Copter hexa and octa

Use the same `arducopter_control` provider and ordinary stock helm for the standard plus-layout SITL frames. Set `vehicle_profile=arducopter` and select `copter_frame=hexa` or `copter_frame=octa` at startup. The selection binds the corresponding exact MAVLink heartbeat type and the existing owned ArduCopter route. The default quad retains its established scope. [Pinned SITL models](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/SITL/SIM_Frame.cpp#L263-L282) · [Pinned model defaults](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/Tools/autotest/pysim/vehicleinfo.py#L41-L93)

| Frame | Exact HEARTBEAT type | Pinned default class and layout |
| --- | --- | --- |
| `hexa` | `MAV_TYPE_HEXAROTOR` (13) | `FRAME_CLASS=2`, `FRAME_TYPE=0` |
| `octa` | `MAV_TYPE_OCTOROTOR` (14) | `FRAME_CLASS=3`, `FRAME_TYPE=0` |

Active simulation requires `plugins=telemetry,arducopter_control`, `firmware_requests=false`, enabled ordinary native actions and stock helm, and the exact owned PID and loopback endpoint. It exposes Guided, arm, disarm, takeoff, native Loiter, Return, Land and stock helm acquire/release. AUTO, the extended operation interface, precision Loiter, native adjustments, dead-man requests and other effect plugins retain their existing quad qualification scope.

Qualification uses the same independently attested ArduCopter v4.6.3 executable with each model's stock defaults and exact active identity. The original binary attestation names its reference quad fixture. This frame runner separately checks the actual model in the command, the on-wire heartbeat type and DataFlash frame parameters. Each scenario exercises prearm takeoff rejection, ground arm/disarm/rearm, airborne stock helm, bounded normal handoff, expired handoff with explicit reacquisition, native Loiter and reacquisition, Return to touchdown, and a separate native Land cycle. The runner verifies the actual loaded capabilities, raw independent telemetry and postflight ownership cleanup.

Run `tests/run_sitl.py --copter-frame hexa` or `--copter-frame octa` with its required source, build, stock MOOS and firmware-attestation inputs. Supply the exact clean source with `--expected-app-revision`. The fixture keeps the stock pHelmIvP executable unchanged.
