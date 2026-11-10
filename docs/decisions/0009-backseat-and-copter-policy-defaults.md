# 0009: Backseat deployment and Copter policy defaults

Date: November 3, 2026. Status: accepted architectural direction; implementation and numerical settings remain unqualified.

## User direction

The user clarified that MAVMOOS should run on the vehicle's backseat computer and communicate with the flight controller using MAVLink. Facing the direction of travel should be the default, with configurable alternatives. In a subsequent explicit choice, the user selected automatic resumption after a bounded normal helm handoff rather than requiring re-enable after every all-stop.

## Choices

1. **Deployment:** the intended operational host is the backseat/companion computer. MAVLink is the FC protocol. Serial, UDP and TCP describe possible carriers of MAVLink; this decision does not select the physical FC connection or qualify the experimental UDP backend for it.
2. **Facing:** the initial Copter profile defaults to facing commanded travel course while moving. Expose facing policy as bridge/profile configuration. Alternative policies and independent yaw input remain feature proposals; no setting exists in the current application. Near-zero-speed behavior and thresholds still require specification/testing. Ignoring the yaw field does not guarantee a fixed heading.
3. **Normal handoffs:** a brief, recognized normal helm handoff may resume automatically within a bounded recovery window after fresh eligible output returns. Resume remains conditional on uninterrupted vehicle/session identity, current operator authorization, suitable FC mode and fresh state. The bridge must first cancel old navigation work. Neither cached targets nor all-stop-generated zeros become permission to resume.
4. **Exceptional recovery is separate:** this choice does not permit automatic takeover after pilot/GCS intervention, a bridge restart, loss of vehicle identity, or expiration of the recovery window. Detailed fault classifications and recovery rules remain to be finalized.

The numerical window, eligible helm reasons, hold mechanism and required post-pause input evidence remain open. In particular, automatic recovery means authorizing newly eligible work under continuing operator permission; it does not resurrect cancelled P2 tokens or old queued targets, or automatically command GUIDED after an external mode change.

## Consequences for earlier drafts

The October 29 Copter draft's blanket latched treatment of all-stop is no longer the selected behavior for bounded normal handoffs. Preserve that dated draft as history; apply the distinction above in the next profile revision. Its already-airborne scenario remains a useful navigation subtest. Arming, takeoff and landing are candidates for an early command/lifecycle module rather than permanent exclusions from the initial product.

This record changes architecture documentation only. [0008](0008-p2-bounded-egress-laboratory.md) remains the implementation boundary: the app is receive-only, experimental real egress remains restricted to owned loopback fixtures, and later integration/control phases have not been executed. Stock MOOS-IvP remains unchanged under [0004](0004-use-stock-moos-ivp.md).

## Evidence and remaining work

The choices above come from the user's direction, not from reference code. The [clarification note](../research/2026-11-03-architecture-and-workflow-clarifications.md) explains the reference support, concrete operator examples, proposed configuration and parallel development approach. No simulator or vehicle result is implied.
