# PX4 aircraft acceptance

The provider exposes native arm, takeoff, Hold, Return, Land and disarm for the pinned PX4 v1.17.0 SIH quad and conventional airplane. The same provider adds a standard VTOL profile with stable hover and forward-flight guidance. Native actions are an explicit startup facet for the owned loopback simulator. [PX4 SIH models](https://docs.px4.io/v1.17/en/sim_sih/) · [Fixed-wing takeoff](https://docs.px4.io/v1.17/en/flight_modes_fw/takeoff)

| Workflow | Evidence required |
| --- | --- |
| Ground lifecycle | Fresh addressed heartbeat and landed state gate normal arm and disarm. Match the local send and addressed ACK, then independently observe the requested armed state on the ground. Deny takeoff before arm and disarm in flight. |
| Takeoff | Require explicit arm and a fresh home and navigation estimate. Send a finite bounded height and target. Confirm native Takeoff after send. Complete after airborne native Hold and arrival at the requested height within the stated tolerance. |
| Hold and Return | Retire helm permission before a native mode command is sent. Require fresh, agreeing heartbeat and CURRENT_MODE after send. Independently measure Hold behavior and movement toward home under Return. The Return command result confirms mode entry. |
| Land | Confirm native Land and observe touchdown with fresh independent telemetry. Test explicit disarm after touchdown. |
| VTOL | Bind the exact standard VTOL airframe identity. Observe the requested stable phase after a bounded, acknowledged transition. Exercise both directions and stock helm motion in both phases. Deny stale inputs, unexpected phase changes and new acquisition during transition. |
| Loss and recovery | Exercise acknowledgement without effect, addressed/foreign response filtering, malformed and stale requests, original expiry, pilot takeover and release. Confirm stock helm cannot resume without a fresh acquisition after authority or phase changes. |

Use the matching production modules and stock helm in the owned Linux SITL domain. Pin the source, firmware, tool image and runtime inputs. Archive one compact bundle with accepted and failed receipts, independent observations and scoped teardown. [Existing PX4 qualification procedures](../../../tests/px4_offboard/FIXED_WING.md)
