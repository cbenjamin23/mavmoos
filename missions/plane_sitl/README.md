# Conventional Plane SITL mission

The mission uses stock `pHelmIvP`, its constant-course and constant-speed behaviors, and the existing `BHV_ConstantAltitude` from `moos-ivp-uav-base`. The helm produces `DESIRED_HEADING`, `DESIRED_SPEED` and `DESIRED_ALTITUDE` through its normal solver. The configuration starts idle and the fixture drives `DEPLOY`, the three behavior update variables, and explicit acquire and handback requests.

`meta_vehicle.moos` starts one private MOOSDB, the pinned helm and `pLogger` through `pAntler`. The [native runner](../../tests/plane_native/README.md) generates the targets with strict `nsplug` and starts production `iMAVLink` using its own `ProcessConfig` in the generated mission. Separate ownership of the bridge process permits fault injection and restart with the same simulator and helm still running. The runner chooses all ports and verifies the exact connected native peer before any fixture writes.

The first flight leg uses a 60° ground course, 22 m/s cruise-envelope intent and 65 m above FC home. The firmware sets and tracks actual cruise airspeed. Native Loiter and RTL are airplane orbits. The runner uses a separate owned simulator operator to establish the armed airborne condition before a MAVMOOS acquisition.
