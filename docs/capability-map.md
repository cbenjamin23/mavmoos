# Capability map

Updated November 6, 2026. The [plugin progress tracker](plugin-progress.md) records all 34 scoped workflows. The latest qualification ledger separates native control and estimator results from passive observations and required software CI.

The [vehicle coverage matrix](plans/2026-11-06-vehicle-family-coverage/README.md) records the broader PX4 and ArduPilot target, exact profile evidence and planned family gaps.

The PX4 provider includes quadrotor, conventional fixed-wing and standard VTOL profiles with native lifecycle actions and phase-specific stock helm. PX4 Ackermann is a separate opt-in provider. The ArduPilot QuadPlane provider owns the verified native VTOL workflow and opt-in forward stock helm. Current control evidence · Ackermann evidence

[Decision 0009](decisions/0009-backseat-and-copter-policy-defaults.md) settles backseat deployment, configurable course-facing as the initial Copter default, and automatic recovery after bounded normal helm handoffs. Their delivered simulation subset is listed below. The [clarification note](research/2026-11-03-architecture-and-workflow-clarifications.md) proposes early command/lifecycle development using pArduBridge and MAVROS precedent; later delivery evidence is recorded separately below.

The [capability-oriented review](design/2026-11-04-capability-plugins/README.md) supersedes the earlier commands-versus-helm public partition. The current startup roster includes telemetry, system status, attitude, RC, range, IMU, odometry, air data, ESC status, landing target, traffic, parameters, missions, flight logs, optical flow, terrain, navigation monitoring, link and clock status, camera and gimbal, debug values, event timeline, surface-boat water depth, read-only MAVLink files, and mode catalog, component metadata, Remote ID and ExternalNav, and separate ArduCopter, ArduRover, ArduPlane, ArduSub, ArduPilot QuadPlane, PX4 aircraft and PX4 Ackermann control providers. Telemetry alone is the default. Select one vehicle-control provider for the corresponding firmware profile. Native, Guided, stock-helm and optional bounded AUTO execution strategies share the selected Copter provider. Payloads, further sensors and bulk services retain coherent independent responsibilities. The [current provider/reference crosswalk](design/2026-11-05-provider-reference-crosswalk/README.md) maps concrete MAVROS and pArduBridge sources to the implemented boundaries and omissions. The broad candidate table below remains an inventory.

Current builds require matching development ABI 10 host/modules. The dated increments below retain their original ABI numbers and delivery limits; read the later AUTO/sensor expansion and its exact qualification ledger before treating an earlier limit as current support or widening an earlier test result.

The earlier full macOS ARM64 cell at `119f723` passed 133 of 133 cases, including all 30 native cases. Both 103-case software gates passed separately. The native cell pins stable Copter 4.6.3 and a separate Rover fixture. Earlier Linux and macOS results remain tied to their own revisions. Earlier full qualification · Earlier expansion

## Status vocabulary

- **Candidate:** identified for analysis; may have a draft plan, without a delivery commitment.
- **Planned:** an implementation scope and acceptance criteria have been accepted for delivery.
- **Implemented:** code exists at a cited revision; does not imply validation.
- **Validated:** cited evidence identifies the scenario, autopilot/version, vehicle, and configuration covered.
- **Deferred / excluded:** an explicit decision records the reason and scope.

Design status and implementation/validation evidence should be tracked independently as work proceeds. The delivered subset and its exact evidence layer are recorded below; broader capability rows do not imply full parity.

## Current modular simulation subset

Implementation, [decision 0010](decisions/0010-modular-copter-sitl-workflow.md) and results establish the delivered boundaries:

| Capability | Implemented / validated layer |
| --- | --- |
| Startup-loaded feature factories, shared context/services | Missing libraries/exports, ABI rejection, duplicate names, dispatch, reset and destruction tests |
| Capability provider grouping and discovery | ABI 6 descriptors, duplicate capability/resource/request-owner checks, one control backend, actual configured catalog, legacy native/helm facet mapping; November 4 evidence |
| Conventional NAV projection and validity | MOOSGeodesy conversions, yaw/course and altitude references, stale/reset tests, live stock-helm mission |
| Copter lifecycle requests | guided/arm/takeoff/hold/land/rtl/disarm implemented; nominal guided/arm/takeoff/Loiter/land/disarm observed in exact native SITL; native RTL now has an actual away-home return/landing scenario in the qualified Copter record |
| Stock helm adaptation | Actual ordinary pHelmIvP outputs, shadow zero-output, arrival-grouped freshness/source checks and post-boundary eligibility |
| Normal handoff and authority | Brief normal pause auto-resumes, expired pause latches, native Loiter cannot be automatically recaptured, explicit acquisition resumes; reset/final-clock/state guards unit-tested |
| Parameters | Typed reads, bounded paced indexed inventory and complete snapshot; restricted disarmed simulator wind write with echo/readback/restore; native macOS and Linux evidence in the parameter record |
| Concurrent feature authority | ABI 4 command-operation interlocks, nonrenewable normal-handoff permits and obsolete-hold cancellation; current increment separates unit/scheduling/native evidence |
| Process faults | Helm pause, MOOSDB replacement, simulator pause/restart and command timeout; exact per-platform outcomes in the fault record |
| Clean Linux reproduction | Source-built stock dependencies, ordinary behavior and MAVMOOS, ELF/source checks and native executions; recipe and results |
| Socket-queue age and owner continuity | Bounded stale draining, distinct effective/read times, pre-ingress expiry and ABI 3 hook; new increment |
| Operator recovery advice | Phase-aware module blockers, acquisition preparation and next action, with host authority separately labelled; guide |
| Simulation-only transport | Direct native arducopter PID owns exact loopback peer; wrong PID/port denied; receive-only and shadow output zero; no hardware backend |

No complete MAVROS parity, all-fault P5 result, hardware readiness or real-time timing guarantee follows. The broad candidate inventory below remains applicable beyond this delivered subset.

## November 4 vehicle and mission expansion

The vehicle/mission batch ledger supersedes the earlier subset's delivery limits where stated. That frozen qualification used ABI 6; current builds require matching ABI 10. The following dated observation increment introduced ABI 7, whose historical evidence remains pinned to that source.

| Capability | Implemented scope; qualification tracked in the ledger |
| --- | --- |
| Coherent Copter operations | Versioned native/Guided/configured-helm goto, hold and return; cancellation, original lifetime, observed completion; stock behavior bindings unchanged |
| Configurable Copter facing | Course default, Keep or Heading; per-request Guided facing and startup helm policy |
| Mission storage/progress | Independent download/upload/readback/clear, bounded retries, data-only flags, partial-write uncertainty; no mission execution/fence/rally |
| ArduRover provider | Separate ground and boat identities, planar stock helm, horizontal estimator gates, native Hold/RTL, boat Loiter and versioned Guided goto/hold/return/cancel. [Operator interface](plugins/ardurover_control.md) |
| PX4 Ackermann provider | Owned v1.17.0 SIH Ackermann with planar stock helm, native waypoint execution, verified Hold fallback and stationary handback. [Operator interface](plugins/px4_rover_control.md) · Native and software evidence |
| Shared mission lane | One active transfer, at most four phases, one fair protocol packet per tick; mutation/command/NAV interlocks and final handoff cancellation |
| Shared projection and authority witnesses | One telemetry projection service; observation only. Grant revisions prevent old navigation/hold crossing a fresh grant |

## Overnight observation and robustness increment

The current ledger identifies current source, artifacts and qualification layers separately. It preserves the preceding batch unchanged.

| Capability | Implemented scope; exact qualification in the ledger |
| --- | --- |
| Optional system observations | SYS_STATUS masks, up to eight identified batteries plus ambiguous fallback, primary GPS_RAW_INT, bounded UTF-8/text assembly; no control gate or effects |
| Observation timing metadata | ABI 7 effective acquisition floor and actual read time delivered separately; protocol callback correlation stays read-time based |
| Post-proof handoff checks | Original expiry and live host eligibility revalidated after native PID/socket proof, before the nonblocking write; no atomic syscall/remote recall guarantee |
| Continuity binding | Connection revision prevents previously queued work becoming eligible after reset and subsequent fresh telemetry |
| Native partial-upload cancellation | Explicit local cancel and bridge pause/reset retire old work, preserve mutation uncertainty, reject old-ID replay, then verify FC-settled partial storage and fresh-ID clear; no execution |

## Attitude and concurrent observation increment

The later attitude and observation-load increment adds the following, with exact layers separated in that ledger:

| Capability | Implemented scope; qualification in the dated record |
| --- | --- |
| Fused attitude/rates | Independent ATTITUDE/ATTITUDE_QUATERNION caches, source NED/FRD, explicit validity, norm/offset/singularity handling and original ages; no raw IMU or control ownership |
| Observer/host separation | Actual Runtime/MODULE tests for source/session/coalescing, unchanged target schedule/geometry/authority and protocol actual-read correlation under observer floods |
| Observation stream intent | Fake-I/O decoded host requests verify union/minimum interval selection; no native request-ACK inference |
| Concurrent observation workflow | Owned Copter profiles compare status-only with status+attitude beside stock helm, inventory and mission read; measured native results rather than a general capacity guarantee |

## RC observation and stream recovery increment

The RC batch ledger records the following scopes and exact qualification layers:

| Capability | Implemented scope; qualification in the dated record |
| --- | --- |
| RC input observations | All18 reported raw slots, original count/RSSI/remote-clock unknowns and original local ages; no physical transmitter or authority inference |
| Servo command observations | Up to8 independently aged ports, local slots/extension ambiguity and command-only semantics; no actuator feedback or universal flattening |
| Bounded stream setup | Standard511 requests, fixed10s/3-attempt bounds, local-false retries only, no accepted-packet replay or remote ACK/application inference |
| Shared protocol arbitration | One fair mission/parameter/stream lane per tick after control, original expiry and final epoch/heartbeat/session fencing |

## Observer recovery, protocol faults and distance increment

The current batch ledger separates component, combined software, native service-bench and stock-helm flight evidence.

| Capability | Implemented scope; qualification in the dated record |
| --- | --- |
| Reliable observer reset time | Finite/nonnegative/high-water reset predicates; invalid initial Runtime clock denies configuration/effects until reconstruction |
| Protocol fault handling | Actual-module loss/duplicate/reorder/cancel/restart tests; missing mutation confirmation retains uncertainty and requires explicit verification/recovery |
| Identified range observations | Optional eight-ID `DISTANCE_SENSOR` cache, original local ages, raw/converted distances, body-FRD orientation and per-quantity unknowns; no NAV/terrain projection |
| Untagged remote reply limits | Local IDs and phases do not prove which request caused a correctly addressed delayed `PARAM_VALUE` |

## Historical P1 subset

| Implemented subset | Evidence / limit |
| --- | --- |
| One fixed-peer IPv4 UDP receiver | Bounded reads, peer/target/CRC/size checks; no sender; tested on macOS only |
| Raw heartbeat, global/relative altitude and NED velocity, landed enum, global home subset, EKF status | Generated wire fixtures; conversions, sentinels, unknown/invalid cases; no health or NAV projection |
| Per-family acquisition-age/status and AppCasting | Independent expiry with continuing heartbeat and retained receipt; source age unknown; reference home has separate semantics |

See the implementation record, interface and evidence. P0-A is ready for this fixture layer; P1-A–D pass at the recorded local layer only. P2 evidence is described below. That earlier P1 app could not transmit; the current app can transmit only in the declared owned native simulation mode. Later delivery is described above.

## P2 local egress subset

The P2 record adds pure Copter mapping/encoding, bounded latest-NAV/response/probe work, result reservation, final expiry checks and class-specific fences. Evidence covers fake I/O and an owned loopback peer, including independent byte checks and supported sanitizers. It establishes local software behavior only.

The experimental managed-egress API is not linked into iMAVLink; that P2 increment kept the app receive-only. Current simulation integration separately reuses the pure translation/encoding component. Its loopback sender is not an operational vehicle/backend qualification. The later integration adds NAV projection, acquisition and selected FC interpretation tests. General retries, the full physical-loss contract and hardware behavior remain candidates.

## Initial comparison

| Capability area | MAVROS reference or comparison | Proposed MAVMOOS direction / open question | Broad design status |
| --- | --- | --- | --- |
| Serial, UDP, TCP | `libmavconn` | Transport lifecycle; evaluate reuse and reconnect policy | Candidate beyond delivered RX subset |
| Startup-loaded feature modules | UAS/plugin factories and message dispatch | One modular app; configured library factories, shared context and host-gated effects | Implemented; see subset above |
| Routing and vehicle identity | Router, UAS | Endpoint policy, system/component IDs, per-vehicle contexts | Candidate |
| Position, GPS, home, altitude | `global_position`, `local_position`, `home_position`, `altitude`, GPS extras | MOOS telemetry with explicit frames, units, origins, and validity | Candidate beyond delivered RX subset |
| Attitude and inertial data | `imu` | Fused source-frame Euler/quaternion and body rates implemented above; raw inertial sensors/calibration remain separate | Optional attitude subset implemented; broader inertial capabilities remain candidates |
| Battery, system health, modes | `sys_status` | Observed flags, per-quantity unknowns and sample ages; no aggregate safety verdict | Optional system_status observation subset implemented above; broader variants remain candidates |
| Time synchronization | `sys_time` | Separate remote timestamps, local timing, and freshness | Candidate |
| Commands and acknowledgments | `command`, mode-related interfaces | Correlated requests, transaction outcomes, observed effects | Candidate |
| Missions, fences, rally points | `waypoint`, `geofence`, `rallypoint` | Transaction interface and transfer semantics | Candidate |
| Parameters | `param` | Typed values, request lifecycle, caching policy | Restricted ArduPilot simulation subset implemented/validated above; broader writes/encodings remain candidates |
| Helm intent and setpoints | Setpoint plugin family; MOOS-specific adapter | Stock scalar/state/all-stop compatibility; explicit coherence/age limits and vehicle-aware translation | Candidate |
| Navigation authority | MAVROS comparison plus MOOS-specific design | Requested/confirmed authority, handoffs, external overrides | Candidate |
| Manual and RC interfaces | `manual_control`, `rc_io` | Observe/control boundaries and interaction with helm authority | Candidate |
| Range and obstacle sensing | `distance_sensor`, `rangefinder`, obstacle extras | Optional identified ArduPilot range observations implemented above; external injection, obstacle policy and terrain projection remain separate | Optional distance_sensor subset implemented; broader sensing/policy remains candidate |
| Landing, cameras, gimbals | `landing_target`, `camera`, `gimbal_control`, `mount_control` | Preserve specialized functionality; define appropriate MOOS interfaces | Candidate |
| Files and logs | `ftp`, `log_transfer` | Consider out-of-band payloads with MOOS-visible operation status | Candidate |
| Other standard and extra modules | Full plugin index | Expand inventory and assess relevance individually | Candidate |

## Research history and later evidence

These paragraphs describe status at each archived research step. Those documents did not implement features; the receive-only delivery above is later evidence. Original dated snapshots remain unchanged.

### Evidence required later

For each implemented area, link its interface specification and implementation revision. For validation, record exact autopilot and firmware, vehicle type, configuration, scenario, expected result, observed result, and limitations. Exercise unsupported requests and failures as well as normal behavior. SITL evidence should remain explicitly labeled as simulation evidence.

The [operating-model comparison and three source traces](research/2026-10-28-operating-models/README.md) are completed research evidence for telemetry, transactions, routing, time, and helm setpoints. All capability statuses remain Candidate. Static source analysis does not change a feature to implemented or validated in MAVMOOS.

The [draft contract package](contracts/2026-10-28-draft-01/README.md) adds proposed semantics and review criteria for these areas. It does not promote any capability to Planned, Implemented, or Validated, and it selects no executable vehicle profile.

The [contract-priority review](research/2026-10-28-contract-priorities/README.md) adds static helm-interface findings and proposed origin-freshness, invalidation, handoff, egress and recovery rules. All statuses remain Candidate. Its routes requiring producer changes are now excluded by decision 0004; achievable timing/cancellation bounds remain open.

The October 29 bridge comparison adds Jeremy's Rover reference alongside pArduBridge. It distinguishes shared SDK/telemetry/transaction mechanisms from vehicle-specific yaw, altitude, mode-entry and stop/loss policies. Existing implementations are comparison evidence, not MAVMOOS delivery evidence; all statuses remain Candidate.

The [module sketch](design/2026-10-29-built-in-modules/README.md) gives proposed interfaces/flows for built-in features and stock helm adaptation. [Decision 0004](decisions/0004-use-stock-moos-ivp.md) accepts the external dependency constraint only; it promotes no capability or execution guarantee.

The [initial ArduCopter Guided profile](profiles/2026-10-29-arducopter-draft-01/README.md) adds proposed course/speed/home-relative altitude semantics, already-airborne Guided acquisition, sample-backed confirmation, freshness/cadence candidates and conditional loss handling. It remains a paper profile: exact firmware, numerical envelope, backend and timing/physical behavior are unqualified. Every capability remains Candidate; this does not establish ArduCopter support.

The [backend comparison](research/2026-10-29-backend-comparison/README.md) adds 52-file source evidence for receive provenance, SDK helpers/retries, libmavconn FIFO behavior and direct-codec responsibilities. [Proposed decision 0005](decisions/0005-host-owned-navigation-egress.md) recommends host-owned eligibility/scheduling/retry policy with a qualified transport boundary. No backend is selected and no capability status changes; achievable gates, settlement and residual bounds remain open.

The [UDP queue assessment](research/2026-10-29-udp-queue-assessment/README.md) adds a proposed managed-FIFO change map, a smaller FIFO-bypass alternative, common bounded work/fence semantics and a scoped direct-UDP design. [Proposed decision 0006](decisions/0006-first-single-peer-udp-candidate.md) conditionally prefers owned UDP for the first configured single-peer topology. It selects no backend/platform and qualifies no transport; all statuses remain Candidate.

The [phased implementation plan](plans/2026-10-29-direct-udp-copter/README.md) targets a small receive-only-to-SITL Copter slice, with staged readiness and acceptance criteria. It was archived as an architecture draft with no passed phase. The later receive-only increment is separately authorized and evidenced; broader capability areas and later phases remain candidates. Later evidence must distinguish local tests, loopback transport, stock-helm shadowing and configuration-specific SITL validation.

## CI and firmware evidence increment

See the batch ledger for source/artifact/platform-specific results. That app 0.9.0 checkpoint had nine providers and ABI 7; shared firmware discovery was host context, not another plugin. Its recorded results remain bound to that source.

| Capability | Implemented scope and evidence separation |
| --- | --- |
| Local CI tiers | Exact software/smoke/full identities, fresh private CTest plans, JUnit/raw-report checks, original time bounds and scoped cleanup; hosted workflow remains inactive source |
| Firmware identity | Strict versioned source/submodule/profile manifests and per-platform build attestations; baseline 4.7.0-dev and candidate official 4.6.3 remain separate |
| Firmware discovery | Bound AUTOPILOT_VERSION with original age/session/unknowns; bounded standard request in shared protocol budget, no authority/readiness/capability qualification |
| Capability/test registry | 29 definitions, 99 test mappings, explicit implemented/deferred/unsupported states; eleven historical cells retain their prior source identity |
| Broader Copter scenarios | Existing stock-helm route/helm hold/Guided hold/cancel/return/land and repeated fixed-wind cycles; exact observed operations become additive cells only after archived semantic replay |
| Upgrade discipline | New pinned candidate → source build → selected test tiers → scoped result cells; no automatic promotion, supported-version range or field claim |

The integrated baseline completes 99/99 checks (25 native), normal 74/74, sanitizer 73/73 and clean Linux software 73/73. The pinned 4.6.3 candidate completes 95/99 and remains unqualified: three default-mission fixture assumptions and one range-parameter fixture mismatch fail. Two additive baseline scenario reports preserve exact capability cells separately from the eleven historical cells; no candidate cells or supported version range follow from this batch.

## AUTO and independent sensor expansion

App0.10 candidate adds the optional tenth `imu` provider and development ABI8. The expansion ledger distinguishes implementation and exact qualification; the historical tables above retain their dated scope.

| Capability | Current implementation boundary |
| --- | --- |
| Simple Copter AUTO | Explicit opt-in start/pause/stop-only abort/release within `arducopter_control`; already armed/airborne/Guided and verified HOME plus2..7 simple relative-altitude waypoints; no resume/partial ranges/general mission execution |
| Persistent AUTO authority | Host typed stored-content proof, original expiry and persistent/quarantined in-process lease; command ACK and completion never restore helm; no durable whole-process restart journal; fresh observed handback and explicit reacquisition remain separate |
| IMU observations | Independent RAW_IMU/SCALED_IMU/2/3 reports, raw/SI units, original ages and unknowns; no fusion, readiness or NAV ownership; HIGHRES deferred |
| Firmware fixtures | Exact source/profile-selected HOME shape and physical-range-equivalent parameter schemas; no traffic-driven expectations or automatic version support |
| External development | Separately built read-only example through actual PluginHost/ABI8; trusted resident code, no live unload or stable binary ABI |
| Qualification pipeline | Exact registry/plan/JUnit/native inputs and owned cleanup, source-built Linux and bounded isolated execution domains; no push/hosted activation |

See [decision0023](decisions/0023-simple-auto-imu-and-portable-qualification.md), [AUTO interface](plugins/copter_auto.md), [IMU](plugins/imu.md) and [external plugin guide](development/external-plugins.md). The host/runtime foundations transfer to other features; AUTO mode/mission policies remain Copter-profile-specific.

## External observer authoring increment

The app 0.11 candidate adds checked startup module settings and a complete external heartbeat reference, with a standalone consumer build and actual receive-only MOOS demonstration. The authoring ledger distinguishes module, application/loopback and current-source native evidence. Standard provider count remains ten; no new control policy is introduced.

## External command authoring and AUTO restart

App 0.12 adds the optional external command reference and durable Copter AUTO uncertainty in development ABI 9. Both workflows have individual native evidence and passed in the full stable matrix. Current evidence and exact test scopes

| Capability | Implemented and qualified scope |
| --- | --- |
| External command authoring | A copied, separately built module issues one non-forced disarm against an already disarmed Copter on the ground. The host owns admission, expiry, cancellation and final send. The native case observes the ACK and fresh state separately. |
| AUTO bridge restart | The same owned Copter and private marker directory preserve the uncertainty fence. Fresh non-AUTO observations and explicit current-session handback precede a separate new acquisition. Bounded reads continue while command and mutation paths are fenced. |

## Passive observer expansion

| Provider | Implemented interface |
| --- | --- |
| [Odometry](plugins/odometry.md) | Local NED position and velocity from `LOCAL_POSITION_NED`. Separate `ODOMETRY` pose/twist with frame IDs, exact raw timestamp, covariance, reset and quality. |
| [Air data](plugins/air_data.md) | Independent `VFR_HUD`, `WIND` and `WIND_COV` reports with producer semantics and nullable fields. |
| [ESC status](plugins/esc_status.md) | Common `ESC_INFO` and `ESC_STATUS` indexed groups, plus ArduPilot `ESC_TELEMETRY_1_TO_4` as a distinct report. Each group has its own original age. |

One owned disarmed Copter 4.6.3 run matched local NED, HUD and ArduPilot ESC wire frames to app observations. Generated-frame tests exercised all eight message families in the app, including a separate PX4 identity for the two odometry messages. Exact observer evidence

## Pressure, vibration and local origin

| Provider | Implemented interface |
| --- | --- |
| [Air data](plugins/air_data.md) | Three independently aged `SCALED_PRESSURE*` instances with reported temperature and nullable fields. |
| [IMU](plugins/imu.md) | Separate `VIBRATION` report with body-axis values, exact sample time and raw clipping counters. |
| [Odometry](plugins/odometry.md) | Independent `GPS_GLOBAL_ORIGIN` WGS84/MSL observation with an optional exact remote timestamp. |

The integrated software gates and ArduPilot-shaped app peer covered all three additions. The app exercised pressure instance 1 and the module covered instances 2 and 3. A PX4-shaped peer also exercised local origin. Both app peers verified expiry, session reset and zero egress. Exact software and app evidence

## Copter workflow parity

| Owner | Added or strengthened |
| --- | --- |
| `arducopter_control` | Guided speed/yaw/relative altitude, home marker, stock-helm task selection and tuning, precision Loiter toggle, and source-bound dead-man RTL. |
| `ardurover_control` and `arducopter_control` | Native command confirmation uses accepted local handoff and separate post-handoff FC evidence. |
| `landing_target` | Independent passive report from one configured Copter component, with original age, validity and reset. |
| Telemetry, status, parameters and missions | Corrected altitude validity, session order, parameter-inventory isolation and mission progress and plan correlation. |

Native Copter and Rover evidence is scoped to their exact workflows. Precision Loiter has native command-acceptance evidence. The independent landing-target message path has app-loopback evidence. Copter parity and robustness evidence · [Capability and firmware roadmap](plans/2026-11-05-capability-and-firmware-roadmap/README.md)

## Traffic, plan inspection and log retrieval

| Provider | Operator workflow |
| --- | --- |
| [Traffic](plugins/traffic.md) | Inspect reported aircraft on the selected ArduPilot link with nullable fields and per-aircraft acquisition age. |
| [Missions](plugins/missions.md) | Read separate FC flight-plan, fence and rally snapshots while keeping mutation and AUTO proof scoped to the existing flight-plan interface. |
| [Flight logs](plugins/flight_logs.md) | Catalog the logs, select a current entry and retrieve one complete local file from the owned disarmed Copter profile. |
| [System diagnostics](plugins/system_status.md) | Inspect separate board and servo rails, bounded MCU instances, and independently aged ArduPilot and common estimator reports. |

[Decision 0026](decisions/0026-traffic-plan-inspection-and-flight-logs.md) records the service boundary, original deadlines and cleanup uncertainty. Exact software and SITL evidence

## Overnight observers

Seven new observers and three expansions share the app without affecting authority. The generated ArduPilot and PX4 peers cover the common observers; scoped ArduPilot messages include obstacle and GNSS families. Camera and gimbal use separately nominated components and one grouped test. The disarmed native Copter evidence is limited to optical flow 100, terrain 133/136 and controller output 62. Operator interfaces and exact evidence · [Longer build queue](plans/2026-11-06-overnight-plugin-queue/README.md)

## Files, debug values and fixed-wing air data

The file service has a native disarmed Copter list, get and cancel witness. The standard debug families have generated ArduPilot and PX4 app coverage. Fixed-wing AIRSPEED and HIGH_LATENCY2 have generated app coverage. The pinned native Plane observation matched two actual airspeed sensors. Exact evidence

## Events, water depth and power diagnostics

Event sequence, wrap and original ages, boat depth source/health, and battery identity/generator scope have focused component and generated real-app evidence. The pinned native boat water-depth witness also passed. Evidence and exact source
