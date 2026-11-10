# 0037 — QuadPlane forward helm and standard Copter parity

November 6, 2026. These additions keep one selected vehicle-control provider in charge of the vehicle's supported native operations and helm guidance. Mission transfer stays shared. A verified upload gives the control provider a plan to evaluate and leaves execution permission with the shared host. [Control-provider boundary](0016-capability-oriented-plugin-boundaries.md) · [Previous native vehicle scope](0036-core-vehicle-lifecycle-and-phase-evidence.md)

## QuadPlane

The standard fixed-motor ArduPlane 4.6.3 QuadPlane supports ordinary stock-helm guidance in stable forward flight. The optional helm facet needs an explicitly verified profile, including `Q_GUIDED_MODE=0`, healthy airspeed and fresh selected phase and state. It reuses the fixed-wing lookahead and independent Guided target readback. Explicit acquisition from native Loiter must produce a local mode handoff and subsequent Guided observation. Fresh stock inputs after that boundary authorize the next bounded target.

The normal all-stop permit keeps its original deadline. A phase change, pilot mode change, missing observations, stale targets or a prolonged halt retires the permit and pending targets. Fresh qualified forward Guided may receive bounded native Loiter handback. If the bridge cannot establish fresh FC state, it withholds the watched heartbeat and the FC can apply its verified watchdog policy. The native stored plan and restart marker still fence helm. Hover, transitions and landing are demonstrated through native workflows. [QuadPlane operator interface](../plugins/arduquadplane_control.md)

The firmware uses a fixed-wing approach for its ordinary Guided point. With `Q_GUIDED_MODE=1` it can hover at the destination. A new point can take it out of hover because the setter resets the VTOL loiter state. Horizontal position-target streams do not implement continuous hover steering. The pinned firmware offers separate onboard scripting hooks for location updates and VTOL velocity matching. Any such integration needs its own protocol, loss, takeover and evidence contract. [ArduPilot Guided behavior](https://ardupilot.org/plane/docs/quadplane-auto-mode.html#guided-mode) · [Guided MAVLink messages](https://ardupilot.org/dev/docs/plane-commands-in-guided-mode.html) · [Pinned target update](https://github.com/ArduPilot/ardupilot/blob/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6/ArduPlane/commands.cpp#L71-L110) · [Pinned Lua hooks](https://github.com/ArduPilot/ardupilot/blob/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6/ArduPlane/ArduPlane.cpp#L930-L966)

## Standard Copter frames

Quad, hexa plus and octa plus share one native, Guided, configured-helm and verified AUTO command surface. Explicit frame selection binds the matching heartbeat type. Fixtures use the same attested 4.6.3 binary and independently verify the selected native model, stock default files, and actual frame class and layout. Mission transfer preserves the disarmed ground requirement. Typed execution and its durable restart marker bind the selected type and all previous freshness, HOME and plan checks.

For Loiter-to-Guided helm acquisition, a locally accepted mode command and a subsequent selected Guided observation establish bridge ownership. An acquire that starts in already-observed Guided is a separate explicit operator choice. Loss of the queued acquisition retires both old navigation and any unconfirmed hold. Shared payload and observation providers retain their own evidence boundaries.

Exact results and limitations are in the qualification ledger. Stock pHelmIvP publishes ordinary course, speed, height, state and all-stop mail. The bridge owns their original ages and the control policy.
