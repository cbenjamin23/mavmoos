# 0036 — Core vehicle lifecycle and phase evidence

November 6, 2026. Implemented in owned simulation under the user's authorization to close the remaining control workflows and strengthen their tests. Stock pHelmIvP and the existing default receive-only behavior continue to define the MOOS boundary.

## Context and choice

The original pArduBridge workflow includes native lifecycle actions, direct guidance and operations selected through the helm. A protocol ACK establishes acceptance. The bridge also needs independent mode, motion and final-state evidence for an operation's declared outcome. The same wire command can have different semantics across vehicle types or firmware releases.

Each selected control provider owns its native commands and guidance together. Shared Runtime services own the FC connection, absolute publication-based expiry, final eligibility, protocol scheduling, stored-content proof and durable uncertainty marker. Frame and phase restrictions belong to the selected profile. [pArduBridge crosswalk](../research/2026-11-06-mavros-ros2-provider-crosswalk.md) · [Shared provider principle](0016-capability-oriented-plugin-boundaries.md)

Conventional ArduPlane retains one provider. Version-bound AUTOLAND is an explicit capability of that provider. PX4's multicopter, fixed-wing and standard VTOL profiles share their provider with distinct phase and control policies. ArduPilot QuadPlane uses an additional native-mission provider. Its tested pin permits explicit VTOL transition requests in AUTO, so its current workflow binds an entire verified takeoff, forward waypoint and VTOL landing sequence. The hybrid workflow has its own phase and ground-handback contract. [QuadPlane interface](../plugins/arduquadplane_control.md) · [Pinned transition receiver](https://github.com/ArduPilot/ardupilot/blob/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6/ArduPlane/quadplane.cpp#L2022)

## Consequences

- Mission transfer, verified content and execution permission have distinct lifetimes. A completed or uncertain native mission retains the lease until the profile's explicit observed handback.
- A pilot-selected mode exit retires the old Copter, Rover or QuadPlane execution's authority to stop in AUTO. Rover's deliberate pause and resume retain their intended ownership. A fresh explicit handback can acknowledge the profile's current safe native state. Re-entering AUTO preserves the takeover boundary.
- A verified terminal landing can account for the FC's expected cursor reset and ground HOME resurvey while retaining the original landing geometry, fresh ground observations and explicit release.
- An unresolved QuadPlane restart can request its three required observation intervals. Native navigation and mission mutation remain fenced until current-session handback.
- Additional Copter frames expose their qualified basic lifecycle and stock-helm surface. Quad-specific advanced operations retain their exact evidence scope.
- Native cases group a complete workflow per owned simulator boot. The required software tier tests the loaded modules, failed transitions and final-send boundaries. Dedicated native scripts retain the firmware and platform inputs for release qualification.

## Evidence and open coverage

The core ledger records the final software gates, native cases, source revisions, useful failed attempts and compact archive readback. The [tracker](../plugin-progress.md) separates the original v1 milestone from expanded vehicle coverage.

Live QuadPlane helm adaptation, additional VTOL geometries, traditional helicopters and additional Rover/Sub layouts need their own control and motion qualification. Physical backseat transports and field operation have a separate deployment scope. The selected standard native workflows remain bounded to their named firmware pins, frame models and fixtures.
