# Comparative architecture analysis plan

Proposed October 28, 2026. This is a research plan, not an approved implementation architecture. It extends the [current architecture](../architecture.md) and existing bridge survey. Work remains architecture-only.

Progress: the [operating-model comparison and three representative static source traces](2026-10-28-operating-models/README.md) are now archived. They complete the first requested research deliverable; interface/authority design, broader capability analysis, architecture selection, and runtime validation remain open. Initial access limitations below describe the planning pass; the follow-up study successfully retrieved the selected MIT PDFs.

Further progress: the [four-contract draft package](../contracts/2026-10-28-draft-01/README.md) now proposes observation, command-transaction, navigation-intent, and authority semantics with a shared model, question register, and review scenarios. These drafts advance interface/authority design without accepting the choices, qualifying any vehicle profile, or completing the broader study.

The [contract-priority follow-up](2026-10-28-contract-priorities/README.md) now ranks all 25 questions and investigates eight foundational issues, especially coherent helm output and origin freshness. It records proposed recommendations, source evidence and paper cases. The next architecture task is review/adoption and one concrete execution profile; implementation and runtime validation remain outside the present scope.

## Objective

Determine what a broad MAVLink bridge must provide for MOOS-IvP, what can be reused from existing projects, and which contracts require a MOOS-specific design. Analyze middleware, autonomy, vehicle execution, and operator workflows separately. ROS and MOOS applications can play similar roles, but matching component names does not establish matching semantics.

Use the current MAVROS ROS 2 baseline for detailed comparison. If ROS 1 material is consulted, label its version and avoid blending its interfaces and execution model into ROS 2 conclusions. Similarly, distinguish upstream MOOS documentation, the installed checkout, and project-specific conventions.

## Initial evidence and source access

The first inventory and source-reading observations are recorded in the existing survey; a complete comparative analysis has not been performed.

Additional local inspection for this plan used `~/moos-ivp`, HEAD `023fb54e00692611ecee2cd6f1138d10eec2127e`:

- `ivp/src/pHelmIvP/HelmIvP.cpp`, lines 500–549: decision completeness/all-stop handling and publication of `DESIRED_*` values, including course-to-heading aliasing.
- The same file, lines 1717–1770: `postAllStop` includes status checks, transition handling, and zero-valued decision publications. This is a checkout-specific observation, not a universal aircraft response policy.
- `ivp/src/pMarinePIDV22/MarinePID.cpp`, lines 65–79, 145–176, and 248–257: navigation inputs, manual-override handling, and rudder/thrust/elevator output paths.
- [Pinned Helm source location](https://github.com/moos-ivp/moos-ivp/blob/023fb54e00692611ecee2cd6f1138d10eec2127e/ivp/src/pHelmIvP/HelmIvP.cpp).
- [Pinned MarinePID source location](https://github.com/moos-ivp/moos-ivp/blob/023fb54e00692611ecee2cd6f1138d10eec2127e/ivp/src/pMarinePIDV22/MarinePID.cpp).

The live MIT PDF index was unavailable through the web reader during this planning pass, so these statements use local source, not verified manual text. ROS documentation access also failed; detailed ROS interface and QoS claims remain research items. The [MAVLink command protocol](https://mavlink.io/en/services/command.html) was read and explicitly distinguishes command acceptance from completion.

## 1. Establish comparable operating models

Produce a versioned baseline inventory and diagrams for MAVROS, pArduBridge, and the BlueBoat path. Identify responsibilities rather than assuming equivalent process layouts. Include MOOS core messaging, helm autonomy, external controllers, the bridge, flight-controller execution, and operator/GCS input.

Document representative uses: telemetry only; direct commands; FC mission execution; helm navigation; payload operations; multiple vehicles. For every use, identify who makes navigation decisions, who stabilizes the vehicle, who may override, and what remains useful without the helm.

Separate MOOS conventions from requirements imposed by a particular mission. Separate MAVLink protocol capability from firmware support, vehicle applicability, and MAVROS's exposed interface.

Output: baseline inventory, operating-mode diagrams, and a responsibility table. Resolve the pArduBridge source/documentation discrepancy before describing its Copter helm behavior as established.

## 2. Trace representative workflows through actual source

Start with three paths: position telemetry, an acknowledged mode-related operation, and a continuously transmitted navigation setpoint. Expand to mission upload and parameter access after the main boundaries are understood. Include the relevant MOOS producer/consumer path, not only the bridge function.

For each path record sender/receiver identity, source symbols, transport and routing, state ownership, threading/queues, timing, conversion, interface output, protocol result, observed effect, and timeout/reconnect behavior. State where source inspection ends and inference begins.

Compare against pArduBridge for MAVSDK integration and the BlueBoat brokers for control/hardware separation. Do not assume the direct Navigator interface is an autopilot bridge or that its fault isolation follows from a diagram alone.

Output: source-backed sequence diagrams and a table of differences and uncertainties. Each consequential claim needs a pinned primary reference.

## 3. Specify the MOOS-facing contracts

Study ROS 2 messages/services/actions, QoS, frame tools, and executors as reference mechanisms. Identify which guarantees they actually provide and which remain application responsibilities. Determine what MOOS already supplies before proposing additional mechanisms.

For telemetry, specify names, data types, schema versions where needed, units, frame/origin, timestamp meaning, validity, source identity, update policy, and behavior after disconnect. Evaluate conventional scalar navigation variables alongside richer sensor/status records.

For commands, specify issuer, target, request identity, duplicate handling, ordering, preemption, acknowledgment/progress/result, expiry, cancellation if supported, and asynchronous state observations. A repeated cached value must not accidentally become a repeated operator transaction.

For helm intent, specify validity and freshness separately from value changes. Investigate multi-variable consistency: can heading, speed, and altitude from different decision cycles form one command? Examine received-message provenance, late subscribers, reconnect state, community forwarding, alternate publishers, and competing/standby helms. Do not assume either last-writer behavior or a proposed publisher allowlist is sufficient authority management.

Output: a draft interface contract with concrete examples for telemetry, a command transaction, and a helm decision. All names remain provisional until reviewed.

## 4. Resolve navigation and control authority

Define navigation authority, flight-controller mode, arming state, bridge health, and command eligibility as separate dimensions. Identify what the bridge can enforce on its own link and what RC/GCS paths or FC failsafes can independently change.

Draft transition tables covering requested, pending, confirmed, rejected, timed out, externally overridden, disconnected, and recovering situations. Specify entry evidence, transmission policy during handoff, confirmation evidence, and the fate of cached intent. Account for autopilots that require a stream before entering an external-control mode.

Explicitly investigate `MOOS_MANUAL_OVERRIDE`, helm standby/parking/all-stop behavior, `IVPHELM_STATE`, `IVPHELM_ALLSTOP`, and the relevant project-specific signals. A helm all-stop must have a deliberate vehicle-aware interpretation; zero heading/speed/altitude cannot be assumed to express a valid aircraft fallback.

Output: proposed authority state/transition tables, sequence diagrams for takeover/release/override, and written invariants that later tests can challenge. Define observable rejection reasons rather than hiding transitions inside a generic mode string.

## 5. Resolve vehicle semantics, geometry, and timing

Build a matrix by autopilot, firmware version, vehicle class, requested control quantity, and supported mode/message. Distinguish heading, course, yaw, yaw rate, airspeed, groundspeed, position, velocity, depth, altitude, and vertical rate. Evaluate whether an existing MOOS controller or an FC controller owns each loop; avoid accidental parallel or nested controllers without a deliberate design.

Specify NED/ENU/body/local/global conversions, local origins, heading conventions, vertical signs, home changes, and AMSL/ellipsoid/relative-altitude distinctions. Treat invalid sensor data and estimator resets explicitly. Verify conversions against source and known numerical examples in the later validation phase.

Separate telemetry source time, receipt time, MOOS time/time warp, monotonic elapsed time, helm decision cadence, AppTick/CommsTick, and MAVLink transmission cadence. Investigate queue bounds, blocking transactions, scheduling delay, stale data, and restart. A live transmission thread is not proof of fresh upstream intent.

Output: vehicle/control compatibility matrix, coordinate/units contract, timing diagram, and failure-response matrix. Do not invent universal rates, timeout values, or fallback modes.

## 6. Compare implementation strategies and functional breadth

Evaluate direct MAVLink libraries, standalone transport reuse such as libmavconn, MAVSDK, and any justified hybrid. Compare supported operations, extension escape hatches, autopilot/version coverage, platform/build burden, licensing, dependency lifecycle, testability, and ability to keep MOOS details outside protocol logic.

Evaluate static functional modules versus dynamic plugins only after module boundaries and extension needs are clear. Likewise, compare process layouts based on state ownership, latency, isolation, observability, and deployment needs rather than copying MAVROS's ROS composition model.

Expand the capability map across standard and extra MAVROS functions. Classify each as directly useful, requiring a different MOOS interface, needing out-of-band bulk transfer, or deferred with a rationale. Include advanced use without forcing bulk sensor/file traffic into conventional navigation scalars.

Examine multiple MAVLink systems/components, multiple links, GCS forwarding, loop prevention, identity conflicts, per-vehicle state/authority, source authorization, signing configuration, configuration changes, and credential handling. Record security assumptions in relation to actual deployment paths.

Output: scored tradeoff tables with evidence and explicit unknowns. An option is not accepted merely because it is already used in a reference project.

## 7. Review the proposal through failure scenarios

Walk through helm crash with a healthy transmitter; MOOSDB loss with a healthy FC link; GCS/RC takeover mid-handoff; duplicate/late acknowledgments; partially updated navigation intent; stale telemetry; reconnect with an already-armed FC; FC restart with reused IDs; wrong-vehicle routing; and payload-transfer load during navigation. Add vehicle-specific cases rather than imposing one fallback everywhere.

Specify the bridge's observable behavior and what independent observations would verify it. Plan logging/AppCasting, command/authority histories, relevant `.alog` evidence, and reproducible ArduPilot/PX4 SITL cases. Simulation results must remain distinct from hardware qualification.

Output: review scenarios, acceptance criteria, and a future validation plan. Writing these is still architecture work; launching experiments or implementing a harness is a later scope decision.

## Completion criteria and sequencing

Begin with operating models and the three representative source traces, then draft interfaces and authority transitions. Use vehicle/timing findings to challenge those drafts before selecting dependencies or plugin mechanisms. This ordering reduces the chance of choosing infrastructure around an incorrect control contract.

The study is ready for architectural review when its major behavior claims are sourced, all interfaces specify identity/units/frames/freshness, authority transitions have explicit evidence and failure paths, representative scenarios can be followed end to end, and unresolved questions have a stated impact. Keep design options, recommendations, and accepted decisions distinct.

Deliver a concise recommendation supported by detailed evidence, with a list of choices that must be resolved before implementation and questions that can be deferred. The next implementation milestone should be selected only after that review. No source code, build system, or runnable mission is authorized by this plan.
