# ArduCopter AUTO mission execution

AUTO lets the FC execute a verified stored mission for standard quad, hexa and octa configurations. This optional capability lives inside `arducopter_control`, alongside native, Guided and helm operations. The independent `missions` provider owns transfer and storage. Execution permission requires a separate start request. The current effectful scope is the exact owned localhost Copter simulator with stock `pHelmIvP`.

For an already configured owned-SITL mission, select matching ABI 10 host and modules with these explicit settings.

```text
plugins = telemetry,parameters,missions,arducopter_control
control_native_actions = true
control_mission_execution = true
auto_recovery_directory = /absolute/private/run/directory
```

`control_mission_execution` defaults to false. AUTO requires the native facet and an existing private directory owned by the bridge user. Create a directory for this owned simulator session with `mkdir -m 700` and reuse its absolute path when restarting the bridge. These selection fields supplement the normal SITL endpoint configuration. Receive-only and shadow modes still send zero MAVLink. The capability catalog advertises `control.mission.start/pause/resume/abort/release/progress` when configured. Current permission is reported separately.

## Select and start

1. Upload through [mission data transfer](missions.md) while disarmed, on ground and idle. Wait for successful mandatory readback.
2. Read `MAVMOOS_STORED_MISSION_PROOF`. Require current `valid=true`; copy its `verification_id`, `content_id` and `revision`. Those opaque selectors identify the host's verified stored content and session. A cached or externally posted publication cannot create permission.
3. Explicitly enter Guided and arm using the native control interface. For an airborne entry, take off separately. For a plan whose first executable item is `NAV_TAKEOFF`, remain stopped on the ground beside the bound takeoff point. Both entries require fresh heartbeat, position, landed and horizontal-estimator evidence plus a valid bound HOME. The start request consumes the chosen entry state.
4. Submit a unique start request on `MAVMOOS_MISSION_CONTROL`:

```text
schema=1,id=run1,action=start,verification_id=upload1,content_id=mission/session1/1/7,proof_revision=7,timeout=90
```

Copy the actual current proof's selectors, mapping publication `revision` to request `proof_revision`. The values shown are examples. The host compares them against its typed retained object and flight session. The content ID is opaque and local to that bridge process.

Start binds persistent AUTO ownership, selects the first executable item, then requests AUTO. It waits for local handoff and fresh AUTO/progress observations. The enabled feature requests primary mission progress through the shared bounded observation lane. Missing progress cannot be replaced by cached values or a protocol reply.

## Results and ownership

Watch `MAVMOOS_MISSION_EXECUTION_RESULT`, `MAVMOOS_MISSION_EXECUTION_STATUS` and `MAVMOOS_MISSION_EXECUTION_PROGRESS`. Status includes `lease_retained`; the host also publishes `MAVMOOS_MISSION_EXECUTION_LEASE`. Check current app/publication age when reading retained MOOS values.

| Result | Meaning |
| --- | --- |
| SUBMITTED / SENT / ACK_OBSERVED | Queue admission / local send acceptance / diagnostic wire response. An ACK can belong to an older request and does not advance or fail a phase. |
| ACTIVE | Fresh post-start AUTO and mission progress were observed. |
| Parent COMPLETED | Required first/final progress, final reached/COMPLETE and settled final geometry were observed. **Ownership remains held**, because the FC may continue holding in AUTO. |
| PAUSED | Mission progression is observed paused in AUTO. Braking takes time; PAUSED does not mean instantaneous zero speed. Ownership remains held. |
| TIMED_OUT / LOST / FAILED | Local work stops or is cancelled and uncertainty is retained. AUTO may continue remotely; returning data cannot silently resume permission. |

A waypoint finish requires final position within 2 m, height within 0.5 m and horizontal speed at most 0.5 m/s, settled for one second. A landing finish requires the active terminal LAND cursor after observed flight, then disarmed ground within 3 m of the bound landing point and one second of settled evidence. The native LAND path can reset the cursor at disarm. The observed landing and physical ground state carry completion through that reset. While AUTO ownership is retained, competing MAVMOOS helm/native navigation and mission mutation are blocked. Read-only mission and parameter traffic can continue without renewing proof or the original execution deadline.

## Pause, resume, abort and release

Use a fresh unique ID for each request, targeting the original start ID:

```text
schema=1,id=pause1,action=pause,target=run1,timeout=8
schema=1,id=resume1,action=resume,target=run1,timeout=8
schema=1,id=stop1,action=abort,target=run1,timeout=8
schema=1,id=release1,action=release,target=run1,timeout=8
```

Pause is supported only while a waypoint is actively progressing; ArduPilot can reject it at the destination or in another submode. Successful pause stays in AUTO and the FC brakes toward a hold. An explicit resume requires a fresh paused airborne execution and confirms the same cursor ACTIVE after the command. The original execution lifetime still applies.

Abort is a new bounded **stop-only** request to enter Guided while the same session retains observed native AUTO. Its own fresh deadline can be used after the original start expired. A pilot-selected exit from AUTO retires that wire authority for the retained lease. Use the explicit release from the observed non-AUTO state. Abort COMPLETED requires matching receipt/ACK, fresh post-handoff Guided evidence and host-confirmed lease release. It cancels a nonterminal parent; an already COMPLETED/TIMED_OUT/LOST parent keeps its existing outcome. Rejection, missing state or lost communication cannot be reported as a confirmed stop.

Release sends no command. It acknowledges an already observed non-AUTO handback, with no pending phase and a fresh heartbeat after the last handoff; the host must confirm release. It cannot assume AUTO stopped after link loss. After abort or release, helm remains unacquired. Request helm acquisition, a helm return operation or another native action explicitly through its normal interface.

IDs contain 1–40 letters, digits, `_`, `-` or `.`, and are retained across reset; reusing one returns DUPLICATE and does not renew anything. Start timeout is 1–120 seconds (default 90); other actions are 1–15 seconds (default 8). The overall lifetime starts at the original request publication. Each protocol phase has an additional five-second bound inside that lifetime; progress, ACKs and read completion cannot extend either deadline. Local cancellation cannot recall an already transmitted packet.

## Recover after an iMAVLink restart

Keep the same owned Copter SITL process and recovery directory. Keep the directory configured even if you turn off new AUTO starts. The bridge records `auto-may-continue` before it admits AUTO execution. The marker contains no command, mission content, deadline or permission. An observed unowned AUTO mode also fences control.

Read the current `MAVMOOS_AUTO_RECOVERY_STATUS`. `WAIT_HEARTBEAT` waits for accepted FC evidence, `UNRESOLVED` retains possible AUTO ownership, and `STORAGE_FAULT` indicates an unsafe recovery store. Navigation, competing native commands and writes stay fenced in those states. Heartbeat and allowed read-only queries can continue. The recovery status has its own process session and indicates when fresh non-AUTO evidence permits handback.

After the operator changes the owned simulator out of AUTO and the current bridge reports readiness for handback, publish a new `MAVMOOS_AUTO_RECOVERY_REQUEST`:

```text
schema=1,id=handback1,action=handback,session=<current recovery status session>
```

Use the live process session and publish after the qualifying observation. The request expires after two seconds. A rejected request reports its reason in `MAVMOOS_AUTO_RECOVERY_RESULT`. A `RELEASED` result retires the uncertainty fence and the old proof and request state. Wait for current observations and acquire helm explicitly if you want to use it. Handback itself sends no mode command.

## Supported plan and evidence

The executable profile contains HOME and two to seven contiguous executable rows. HOME is GLOBAL and must match the bound reference within 1.5 m horizontally and 0.5 m vertically. Other positional rows use GLOBAL_RELATIVE_ALT within 100 m of HOME and require autocontinue.

| Item | Bound |
| --- | --- |
| Waypoint | At least one. Height 1–20 m above HOME and integer dwell of 0–30 seconds. |
| Takeoff | Optional first executable item at HOME, height 3–20 m. Start from armed, stopped Guided at the takeoff point. |
| Land | Optional terminal item, zero relative height and a specific landing point. Verify touchdown and disarm there before handback. |

Native RTL is a separate command. Arbitrary stored commands and partial execution ranges are outside the execution profile. Use one mission writer in the owned fixture. The host binds the content it read back, and the FC's mission cursor identifies progress within that contract.

The waypoint workflow and restart recovery passed on the pinned ArduCopter 4.6.3 quad profile. The typed workflow verified ground takeoff, a paused and resumed cursor, timed dwell, position-bound LAND and explicit handback on quad, hexa and octa. Quad evidence · Hexa and octa evidence · Earlier restart qualification
