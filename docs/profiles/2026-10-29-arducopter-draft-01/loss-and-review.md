# Loss behavior and review cases

October 29, 2026. Proposed refinement of [ArduCopter profile draft-01](README.md), not executed behavior or a hardware operating procedure. Every response requires the current target/session and the backend's demonstrated gate/queue capability. No change to stock pHelmIvP is involved.

## 1. Ordered response

For a loss that invalidates active/acquiring navigation:

1. Remove routine velocity permission, invalidate the current local control generation and discard controllable queued targets before authorizing any response. Record reason, input/observation ages, observed mode and remaining backend uncertainty.
2. Latch the loss. New valid scalars, a later all-stop clear, a returning heartbeat or a reconnect cannot automatically resume navigation.
3. Where the table below allows it, create one separate bounded response permission. Require current target continuity, fresh GUIDED/armed/IN_AIR evidence, valid fresh global position/altitude/velocity and the required estimator readiness. Check those again at the final controllable boundary.
4. Capture a fresh current-position target and attempt at most one bounded handoff. If the evidence expires or the deadline passes before handoff, report the response unavailable/expired. Do not renew the permit indefinitely, chase a moving pose with repeated holds, or reuse the lost helm altitude target.
5. Remain navigation-inhibited. If a mode request, backend queue or remote effect may still be unresolved, use RECOVERY_REQUIRED. Reacquisition requires explicit operator reconciliation/enable, a new control generation and new qualifying helm/state evidence.

Within the draft's 0.5 s response window, effective dispatch expiry is the earliest of trigger-plus-window, required pose/state/health expiry, current permission/session validity and backend limit. Already handed-off work cannot be recalled by incrementing a local generation. This is local revocation, not a claim that the FC instantaneously stopped.

## 2. Proposed conditional capture target

The reference response schedules a current-position Guided hold. This draft proposes the same physical intent, but a deliberately explicit packet plan:

- Snapshot current latitude, longitude and **absolute MSL altitude from one current position record**, retaining its receipt/source evidence. Zero latitude/longitude are valid coordinates; unknown position is represented as unavailable, not zero.
- Use `SET_POSITION_TARGET_GLOBAL_INT`, `MAV_FRAME_GLOBAL_INT`, position-only mask **3576**, ignoring velocity, acceleration, yaw and yaw rate. Latitude/longitude encoding follows the selected protocol; all required position axes are present.
- Issue only while the FC is still observed in native GUIDED. **Do not call a helper that implicitly switches back to Guided** before sending the hold. This deliberately differs from pArduBridge's general goto helper, which can request Guided first.
- The target is current MSL altitude, not the last commanded relative altitude and not an inferred terrain height. No new yaw target is requested. Its actual yaw/braking/position behavior depends on the selected FC configuration and remains a qualification item.

Differences from the reference are intentional draft choices: pArduBridge's inspected hold encoder uses a home-relative global frame and a yaw target; this plan uses one directly observed MSL pose and sends no new yaw target. The profile does not assume that this change has been validated.

A successful queue return is reported as local queue evidence; a known last-boundary handoff may be reported as HOLD_HANDED_OFF. Neither is HOLD_CONFIRMED. This setpoint path has no generic command-ACK/completion guarantee. Continue publishing fresh observed motion/state with the appropriate evidence basis; no stationary-vehicle or target-capture success predicate is invented in this draft.

## 3. Trigger-specific policy

| Trigger during ACQUIRING/ACTIVE | Proposed bridge action | Position-hold attempt? |
| --- | --- | --- |
| Required helm field expired, rejected or missing; excessive field spread | Revoke/invalidate/latch | Only if all independent FC/pose response guards remain current |
| Selected helm heartbeat expires, state leaves eligible DRIVE, or non-clear `IVPHELM_ALLSTOP` | Revoke/invalidate/latch; ignore numeric zero postings as actions | Conditional on fresh FC still Guided and response guards |
| Explicit operator release or trusted MOOS manual override | Yield helm permission; no automatic RTL | Conditional capture only if FC remains Guided; no mode recapture |
| MOOSDB loss or MOOS source-clock assumption fails while process/backend remain alive | Invalidate input readiness and routine permission | Conditional, using independent monotonic FC evidence and separate response permission |
| FC mode changes to RTL, LAND, AUTO, pilot mode or any non-Guided state | Stop managed navigation and yield; latch | **No** hold, Guided request, mode reversal or autonomous recapture |
| Disarmed, landed, taking off/landing, or unknown/stale airborne evidence | Inhibit unsupported navigation and expose observed state | **No** airborne hold under this profile |
| Stale/missing global position, relative altitude, velocity or required estimator readiness | Inhibit; do not derive a new feedback/position target from old data | **No** target when its prerequisites are unavailable |
| Vehicle link/target continuity loss; relevant home/reference ambiguity | Invalidate session/binding as needed and reconcile | **No** speculative target using ambiguous/stale pose or reference |
| Send-gap/queue budget violated | Invalidate controllable navigation; report overload/backend limitation | Only if a fresh bounded response can reach the declared gate; do not enqueue behind an unbounded backlog |
| Mode acquisition rejected/times out | No routine stream; cancel further acquisition work and report uncertain effects if already submitted | Only after a fresh Guided observation and all response guards; otherwise no speculative hold |
| Host crashes or loses its monotonic execution/timing basis | No software cleanup action can be promised | **No promised bridge response**; independent FC behavior/configuration governs |

For the initial draft, even a transient non-clear empty/incomplete/no-goal outcome during active control triggers the latch. This chooses explicit recovery over copying the native helm's one-iteration scalar grace. It can cause additional operator reacquisitions during behavior transitions; review that availability tradeoff before adopting the profile. Idle startup all-stop simply prevents acquisition and does not create an unsolicited hold.

## 4. Independent flight-controller behavior

Current official documentation describes Guided command timeout separately from GCS-heartbeat failsafe. The latter depends on actual heartbeat-source/configuration choices, not whether the helm is producing decisions. Record the selected firmware's `GUID_TIMEOUT`, GCS/RC/EKF/battery/fence behavior and relevant option masks before claiming a loss response. The bridge does not rewrite those settings to make its own timer appear sufficient.

Link loss, a bridge crash, stale telemetry and an external mode change therefore have different responses. This profile does not promise that stopping traffic switches the FC to Loiter, that GCS failsafe notices a healthy bridge repeating stale commands, or that any timer alone establishes stopping distance. See [official/pinned evidence](evidence.md).

## 5. Explicit recovery

Do not clear the recovery latch solely because healthy input returns. Re-establish target/reference/clock continuity, current FC observations and a declared resolution of pending mode/queue effects. Then accept a new explicit acquisition request under the [normal sequence](README.md#4-control-acquisition). If the FC is in a mode outside the proposed LOITER/GUIDED entry allowlist, reject this profile's acquisition rather than changing it in the background.

Queued mode commands are especially important: a timeout can occur before an SDK retry or already transmitted request takes effect. A new local generation only prevents work that the backend can still gate. Until the dependency's retry/cancel/residual model is qualified, a clean automatic transfer is unestablished.

## 6. Paper cases for later validation

These are proposed expected outcomes, not executed tests:

| ID | Scenario | Expected profile interpretation |
| --- | --- | --- |
| C-01 | Course 90°, speed 2 m/s, target relative altitude two metres above current | North approximately 0, east 2, down -1 m/s; moving yaw request follows course |
| C-02 | Speed zero with nonzero altitude error | Horizontal zero does not mean no vertical correction; yaw request ignored |
| C-03 | Negative speed, absent altitude, unknown home or out-of-envelope height | Reject candidate; no silent zero/clamped replacement |
| C-04 | New mode reports never arrive while cached mode says Guided | Dwell cannot finish; acquisition expires at its candidate deadline |
| C-05 | New Guided reports meet sample/dwell rule but input readiness is old/retained | Remain non-active; mode alone cannot authorize streaming |
| C-06 | Bridge receives fresh scalars but helm state is PARK/non-clear all-stop | Inhibit/latch; do not send the scalars, including zeros |
| C-07 | Silence with a still-running sender or a duplicate notification | Original input deadlines expire; neither loop nor duplicate renews them |
| C-08 | Local two-second input bound followed by illustrative three-second FC timeout | Do not report a two-second physical stop; late repeated targets can delay the independent FC timer |
| C-09 | Pilot selects RTL/LAND/manual mode during streaming | Stop managed stream; no Guided or position-hold recapture |
| C-10 | MOOSDB disappears but current FC/pose evidence remains usable | Separate bounded response may attempt hold; no cached helm altitude reuse |
| C-11 | Pose/health/link evidence is stale at loss | No invented current-position hold; expose unknown outcome and rely on recorded independent FC behavior |
| C-12 | FC reports IN_AIR below five metres, or ON_GROUND above a home-relative threshold | Use actual fresh state/profile envelope; altitude alone does not authorize airborne navigation |
| C-13 | Home changes, link reconnects or an old SDK result arrives | Invalidate affected binding; old work/results cannot resume the stream |
| C-14 | Send/hold work waits past a deadline or an external mode change happens before handoff | Gate rejects controllable work; already escaped work remains residual uncertainty |
| C-15 | Later all-stop clear and fresh scalars arrive after a loss latch | Require explicit reconciliation/acquisition; no automatic restart |

## 7. Still open before execution qualification

1. Exact ArduCopter firmware/build, airframe, SDK/backend and parameter snapshot; native mode/landed/health mapping and home-reference evidence.
2. Horizontal-speed/height envelope, altitude gain/limit and yaw threshold suitability; no generic airframe values are selected.
3. Actual source-clock/skew/queue bounds, observed telemetry cadence, retained-message handling and field-spread tradeoff. All new numeric candidates require review.
4. Vehicle behavior between mode entry and first target; sufficient report cadence for acquisition; no hidden preparation behavior.
5. Capture/braking/yaw behavior of the proposed MSL position-only response, its difference from the reference, and what stronger confirmation would require.
6. SDK mode retries, latest-target replacement, final gate, cancellation/drain and downstream residual limits; chosen send timestamp semantics.
7. Actual FC failsafe/RC/GCS topology and loss behavior, including process crash. No universal parameter action is assumed.
8. Whether the initial all-stop latch policy is appropriate during ordinary helm transitions and how operator re-enable is exposed in MOOS.

This is a complete initial proposal for review with explicit qualification gaps. It is not an executable flight profile, a claim of accepted timer/controller values, or validation evidence.
