# Architecture

MAVMOOS runs as the MOOS app `iMAVLink`. It loads plugins at startup and shares one MAVLink UDP connection to the selected flight controller or simulator. The IvP Helm (`pHelmIvP`) and other MOOS apps exchange navigation data, guidance and requests through MOOSDB. The [vehicle support guide](vehicle-support.md) lists the tested control profiles.

## MAVMOOS on the backseat computer

The intended deployment runs `iMAVLink` and MOOS-IvP together on the vehicle’s companion computer. Active control in this release is enabled only for configured SITL profiles. Physical vehicle control still needs field testing.

![The IvP Helm and mission apps connect through MOOSDB to the iMAVLink plugin host. The plugins share a runtime and one MAVLink connection to the flight controller.](../site/assets/backseat-deployment.svg)

## Plugin responsibilities

Enable at most one vehicle-control plugin. It translates native mode requests and live helm guidance for the selected firmware and vehicle. Shared plugins handle telemetry, sensors and services such as parameters, missions and logs. Storing a mission and granting it control are separate operations. [Plugin catalog](plugin-catalog.md) · [Mission interface](plugins/missions.md)

The shared runtime owns vehicle identity, queues and final send checks. Plugins use its services to submit work. External plugins load at startup and must match the host’s application binary interface (ABI). [External plugin guide](development/external-plugins.md)

## Helm input and authority

The helm publishes its usual heading, speed and state variables. MAVMOOS checks that the values it needs are recent and were published close together. Some vehicle profiles also use altitude or depth.

The operator requests helm control. The bridge sends guidance once the controller is ready and the helm has a current target. A mode change or takeover can revoke permission and retire queued guidance. Some profiles allow a brief hold and resumption after an ordinary gap. A longer loss, pilot takeover or restart requires the operator to enable helm control again.

For native commands, MAVMOOS distinguishes handing a packet to the transport, receiving an acknowledgement, and observing the requested effect. It checks authority and expiry before sending. Discrete control request IDs are reserved before sending and survive a bridge restart on the same route. [Control request replay protection](development/control-replay.md) · [Command and transition coverage](development/control-coverage.md)

## Firmware and testing

Receive-only mode publishes telemetry. Shadow mode also publishes the targets the bridge would send so a MOOS mission can be checked without transmitting commands. Software checks and SITL scenarios test the bridge against named firmware versions, frames, and workflows. [Release campaign](development/sitl-release.md) · [Latest release report](implementation/2026-11-10-release-28/README.md)
