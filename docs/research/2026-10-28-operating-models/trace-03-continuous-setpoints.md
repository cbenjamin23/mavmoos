# Trace 3: Continuous navigation setpoints

October 28, 2026. Primary MAVROS path: `SetpointVelocityPlugin` velocity/yaw-rate callback to `SET_POSITION_TARGET_LOCAL_NED`. MOOS comparison: pHelmIvP decision publications through pArduBridge's Copter sender. The aim is to locate intent, authority, scheduling, conversion, and freshness responsibilities. No controller or vehicle was run. Evidence.

## 1. MAVROS callback-driven send path

```mermaid
sequenceDiagram
    participant A as External ROS controller / publisher
    participant P as SetpointVelocityPlugin
    participant M as Setpoint mixin
    participant U as UAS / Router / libmavconn
    participant FC as Flight controller
    loop publisher chooses update cadence
        A->>P: TwistStamped or Twist
        P->>P: extract velocity/yaw rate; apply frame conversion
        P->>M: target values + type mask + timestamp
        M->>U: SET_POSITION_TARGET_LOCAL_NED
        U->>FC: routed packet if transport accepts it
    end
    Note over P: No independent retransmission timer in this plugin
    Note over FC: Execution depends on FC mode, capabilities, and policy
```

The plugin subscribes to `~/cmd_vel` (`TwistStamped`) and `~/cmd_vel_unstamped` (`Twist`), using `SensorDataQoS`. It registers no incoming MAVLink handlers. Each callback sends once through `send_setpoint_velocity`; this class contains no scheduler, cache retransmission timer, or FC-ACK transaction. A continuously arriving upstream stream is different from an independently maintained bridge stream. MR06, lines 47–85 and 140–158.

This is a finding about this plugin, not every MAVROS setpoint plugin. The supplied PX4 example demonstrates an external application timer at 20 Hz using a position plugin; that is supporting evidence for where application-level repetition can live, not a velocity-plugin execution trace. MR06, example lines 42–73 and 110–142.

## 2. Exact conversion and send semantics

The default `mav_frame` is LOCAL_NED. The callback extracts linear XYZ and angular Z. For a local-frame command it converts ENU velocity to NED; for BODY_NED/BODY_OFFSET_NED it converts base-link axes to aircraft axes. It similarly converts the yaw-rate vector's vertical sign. `header.frame_id` is not used to select or look up a TF transform in the inspected stamped callback; the configured MAVLink frame controls the branch. A caller must therefore satisfy that configured convention. MR06, 103–158.

The mask is `(1<<10) | (7<<6) | (7<<0)`, decimal 1479: ignore position XYZ, acceleration XYZ, and yaw; use velocity XYZ and yaw rate. The mixin fills target IDs, timestamp, coordinate frame, mask, all vectors, yaw/yaw-rate, then invokes UAS send. A zero-filled field that the mask ignores is not a command to drive that quantity to zero. MR06, mixin 45–85.

For a simple local example, ROS velocity `(east=1,north=2,up=0)` becomes NED `(2,1,0)`, and ENU angular Z `+0.3` becomes yaw rate `-0.3` in the NED convention. This numerical example only illustrates the inspected axis conversion. It does not establish FC acceptance or vehicle response.

The stamped variant uses the supplied stamp; the unstamped variant uses `node->now()`. The helper converts nanoseconds to the message millisecond field without a source-age or intent-validity check. The selected callbacks do not check command expiry, finite values, permission to control, armed state, or whether the FC is in a suitable mode. UAS send serializes and publishes; transport can queue or drop. There is no ordinary `COMMAND_ACK` wait in this setpoint path. MR03, clock helper 280–289; MR02; MR01.

These omissions are boundaries of the inspected mechanism, not evidence that any arbitrary command is safe or that the FC will execute it. An application/controller and the FC must supply the rest of the contract.

## 3. Start at the MOOS producer: the IvP decision

For a successful decision the helm verifies mandatory decision dimensions and publishes each available domain value separately. Course is historically published as `DESIRED_HEADING`; normal publications include a configurable helm prefix. The selected UAV mission includes course, speed, and altitude. The names alone do not make heading a yaw command, speed airspeed, or altitude terrain-relative. MO02, helm 500–549; PA01, mission domain.

Successful decisions are posted even if their numeric values remain unchanged. A bridge can therefore observe deliberate renewal without needing a numerical change. But decision publications, heartbeat, and all-stop are distinct signals:

- `IVPHELM_STATE` is posted near the start of Iterate; it can be current even when a solver result has not subsequently been produced.
- PARK/STANDBY/DISABLED are not active navigation authority merely because the application is communicating.
- All-stop can generate fresh zero-valued decisions. The source contains transition delays and state checks; it is not a continuous “publish zero every tick” rule.
- Source's normal decision prefix and its unprefixed all-stop decision loop differ. A prefixed consumer must not assume it sees all-stop through the same scalar channel.
- A standby takeover can briefly overlap a tardy primary's last decision, as discussed explicitly in the manual.

Sources: MO02, source 340–350, 1048–1064, 1717–1770; manual extracted lines 551–623. Therefore numerical freshness, decision validity, source identity, and authority are separate admission conditions for a future bridge.

## 4. pArduBridge input storage and freshness

The bridge registers separate heading/speed/altitude variables and reads them in `OnNewMail`. Each setter stores the value and sets its own `steady_clock::now()` receipt timestamp. `setpointsFresh` requires all three to have been received and the oldest receipt age to be within the configured timeout, default two seconds in the constructor. Repeated identical values refresh receipt time; change detection is a separate API used by the Plane branch. PA01, 102–138, 1218–1250; PA02, SetpointManager.

`m_helm_desiredValues` is a `ThreadSafeVariable<SetpointManager>`; its arrow proxy holds a mutex during the member-access expression. The sender's freshness check and its three later reads are separate member accesses/locks. Individual access is synchronized, while a complete intent snapshot and its decision-cycle identity remain unspecified. PA01, app 1562–1587, header 251; PA02, proxy 61–75.

The selected mail branch excludes messages from the bridge itself but does not validate a helm producer identity or use the original MOOS timestamp for freshness. An old value delivered on registration, or a broker's re-publication, can therefore acquire a new receipt timestamp. This does not prove a currently deployed failure; it identifies what the current receipt-age test actually establishes. MO01, PA01.

The bridge's registered control switch is project-specific `HELM_STATUS`; the inspected subscription list does not include `IVPHELM_STATE` or `IVPHELM_ALLSTOP`. Its bridge-state predicates should not be mistaken for direct validation of those standard helm signals.

## 5. Bridge authority and scheduling

The sender is enabled when the bridge's `isHelmCommanding` predicate is true, meaning a bridge state beyond active/idle in its enum ordering. The command thread wakes for queued work or enabled streaming, executes a queued operation if present, then calls the registered desired-value function. It sleeps 100 ms when the queue is empty or 10 ms otherwise, in addition to operation time. This is an independent transmission loop but not a hard scheduling guarantee. A blocking queued command can delay the next setpoint. PA01, 566–578; PA02, loop 2111–2168.

For Copter, `sendDesiredValuesToUAV` checks bridge ownership, timeout latch, and input freshness, then invokes `commandCopterHelmSetpoint`. The model additionally requires Copter, airborne state, and a Guided-compatible mode. It does not attempt to re-enter Guided from a native FC mode during routine streaming. That separates ongoing execution from a discrete takeover attempt, although a complete authority/reconnect audit remains pending. PA01, 1562–1588; PA02, 1317–1334.

The Guided handoff path is initiated by a separate operation, then polled for mode-match dwell and fresh Copter intent. That evidence should not be reduced to a single requested-mode variable. Dwell does not by itself prove new FC telemetry; see [trace 2](trace-02-acknowledged-command.md#6-set-mode-and-confirmation-are-different-contracts).

## 6. Copter translation is a vehicle-specific control law

The actual Copter path normalizes course, clamps negative speed to zero, and computes:

```text
v_north = speed * cos(course)
v_east  = speed * sin(course)
v_down  = clamp(-0.5 * (target_relative_altitude - current_relative_altitude), -1, +1)
```

The gains/limits above are values observed in the existing code, not recommended MAVMOOS settings. The message uses LOCAL_NED, ignores position and acceleration, and ignores yaw rate. At speed greater than 0.05 m/s it also requests yaw equal to course; at or below that threshold yaw is ignored. Thus the moving mask is 2503 and the near-zero-speed mask is 3527. Zero speed in this path does not mean disarm or zero actuator thrust. PA02, 1336–1388.

For course 90 degrees and speed 2 m/s, the horizontal target is approximately north 0, east 2. If target altitude is two meters above current relative altitude, the vertical target reaches the observed -1 m/s down-axis limit. The law couples yaw and travel direction while moving; that is a design choice, not an inevitable interpretation of the helm's course.

The target is sent to the discovered system and component AUTOPILOT1 through MAVSDK passthrough queueing. The timestamp is based on the sender's steady-clock millisecond epoch; this trace does not establish that it matches FC boot time. A successful queue result proves neither receipt nor closed-loop tracking. The relative-altitude source is not verified terrain AGL despite the existing identifier names. PA02, 1357–1403 and header 274; SD01, passthrough queue.

**Resolved documentation discrepancy:** the pinned bridge README/architecture prose describes Copter heading as yaw-only and generalized changed-value sending. The current helm Copter source instead continuously sends horizontal velocity with vertical feedback and conditional yaw. The yaw-only description may describe another command path, but it is not a correct account of this helm-stream branch. The separate Plane branch still uses changed-value course/speed/altitude operations. No source or legacy document was edited in the reference repository.

## 7. Stale intent and transport health are different failures

When Copter intent is stale, the sender sets an atomic timeout-detected flag and returns without sending another stale target. The MOOS Iterate handler later latches the condition, disables desired sending, reports the watchdog event, changes bridge state, and schedules a current-position Guided hold with current altitude. Actual hold establishment depends on the later operation and FC response; setting the flag is not immediate physical holding. The inspected Plane branch does not use this same Copter freshness gate. PA01, 1473–1496, 1562–1636.

The FC also has its own behavior when the stream stops. Current [PX4 Offboard documentation](https://docs.px4.io/main/en/flight_modes/offboard) describes a continuous liveness stream before/through external control; current [Copter Guided documentation](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html) calls for periodic velocity/acceleration updates. These moving documents illustrate distinct execution requirements. They do not justify copying one bridge timeout or selecting a universal fallback across firmware/vehicles. The MAVROS path traced here uses MAVLink, not PX4's separate native ROS 2 OffboardControlMode mechanism.

An FC that sees a steady stream of cached targets can consider the downstream sender alive even when the helm has died. An FC stream-loss policy cannot by itself detect that upstream failure. Conversely, a live helm does not prevent a congested or blocked sender from losing the external-control stream. Both conditions require independent observations and responses.

## 8. What the BlueBoat broker adds to the failure analysis

The broker field payload contains type/value, not per-field producer time. Backseat input handling caches the latest value, and each Iterate builds another outgoing frame from that cache. Frontseat receive republishes values with new local notifications. Navigator's thrust input handler updates its receipt timer on those notifications. If configured to carry the relevant values, a healthy pair of brokers can make the downstream timer observe continued arrivals despite upstream producer silence. Packet/drop counters do not resolve producer-intent freshness. BB01.

This is a conditional path derived from source; the currently modified frontseat mission configuration was not used to assert deployment behavior. It demonstrates a general requirement for MAVMOOS: renewal at a transport boundary must not silently become renewal of original intent or authority.

## 9. Proposed contracts and later verification

Define an intent snapshot with decision/source/session identity, quantities/frames/units, validity, receipt and producer times, and expiry semantics. Decide how legacy scalar publications feed it and how incomplete updates are treated. Require source/state/authority evidence appropriate to the selected helm, including standby and all-stop behavior. Separate navigation scheduling from unbounded transaction work and preserve FC mode/failsafe independence.

Later checks should include unchanged but deliberately renewed intent, a helm heartbeat preceding a stalled solver, all-stop zeros, standby/tardy-primary overlap, partial scalar updates, old registration data, broker cache renewal, blocking transactions, external mode change during streaming, stale FC mode cache, and reconnect to a new vehicle/session. Measure actual transmission gaps separately from decision cadence. No runtime tests or controller recommendations are established by this trace.
