# 0033 — Cross-firmware vehicle coverage

November 6, 2026. The project goal is to let users across the PX4 and ArduPilot vehicle families integrate with MOOS-IvP. The 34-provider v1 tracker measures the original workflow milestone. Track vehicle coverage separately by firmware, vehicle behavior, workflow and evidence. [Vehicle coverage matrix](../plans/2026-11-06-vehicle-family-coverage/README.md) · [V1 provider tracker](../plugin-progress.md)

Use the common MOOS observation and request interfaces wherever their semantics match. Retain stock `pHelmIvP`, the selected vehicle-control provider, and shared host authority and freshness checks. Reuse control code for profiles with equivalent behavior. Give a profile its own motion envelope, mode rules and qualification when the firmware or vehicle changes those semantics. Shared observations, native actions, stored mission execution and continuous helm control each earn their own evidence. [Provider boundary](0031-shared-payloads-and-px4-offboard.md) · [Qualification guide](../development/control-qualification.md)

The matrix records current SITL scope and proposed expansion order. It leaves the control interface open for families where the firmware requires a different route, including PX4 1.17 Rover. Every new effectful profile starts with local owned SITL. [PX4 1.17 Offboard interfaces](https://docs.px4.io/v1.17/en/flight_modes/offboard#mavlink-messages)

[Decision 0034](0034-px4-airplane-and-rover-interfaces.md) records the subsequent pinned airplane and direct MAVLink Rover evidence.
