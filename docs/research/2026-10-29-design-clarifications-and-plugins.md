# Clarifying helm integration, freshness, and plugins

October 29, 2026. Architecture-only follow-up to the comparison and the user's design questions. This note distinguishes requirements, stronger optional guarantees, and implementation mechanisms. It narrows earlier recommendations without changing the archived contract drafts or claiming an accepted interface. No application code, dependency, build or mission was added or run.

## 1. What producer-side integration means

The producer is the application making the navigation decision, normally pHelmIvP. A complete-record proposal would have that producer publish all required decision values and final validity together after its solver and output checks. It might require an optional helm output hook or a maintained source change. It does not mean modifying ArduPilot or PX4.

pArduBridge currently receives separate DESIRED_HEADING/SPEED/ALTITUDE messages, stores them in SetpointManager, and updates each field's local receipt time. The Copter sender checks freshness and reads the stored fields. It uses a ThreadSafeVariable wrapper; individual accesses are synchronized, but that does not tag values with a common solver generation. A bridge-side object made from the latest cache would make its own reads coherent without proving that the original values came from one completed helm cycle.

The stronger producer record addresses that specific guarantee. It is not a prerequisite already met by pArduBridge, a claim that ordinary scalar interfaces are unusable, or a settled requirement to patch pHelmIvP for MAVMOOS v1. A companion can also be considered if it establishes equivalent generation evidence. Merely grouping recent arrivals does not establish it. See the earlier [helm source investigation](2026-10-28-contract-priorities/helm-output.md).

**Current recommendation:** assess compatibility with stock pHelmIvP first. Define exactly which coherence/age guarantees its initial profile can support. Retain complete producer output as an optional stronger path until a concrete use case justifies the integration cost. A weaker compatibility path must not be advertised as satisfying the stronger draft contract automatically.

## 2. What the pArduBridge documentation already contributes

The referenced local documentation is substantive. Its INDEX, canonical README, ARCHITECTURE, command reference and hardware qualification notes describe several useful decisions:

- ArduBridge owns MOOS mail/publication and bridge workflow state; UAV_Model owns SDK integration and FC-oriented functions.
- Bridge policy, SDK/protocol acceptance and later telemetry confirmation are distinct stages.
- AUTOPILOT_MODE describes bridge/helm ownership rather than mirroring the FC mode.
- Health/landed-state age requirements depend on the operation. LAND deliberately has different stale-evidence policy from DISARM.
- Callbacks, result queues and confirmation trackers have stated concurrency roles; SITL claims are separated from hardware qualification.

These decisions are a useful starting point, not missing design work that MAVMOOS must invent again. Their current source also includes a Copter setpoint watchdog, a timeout latch, separate Guided acquisition and a streamed velocity path that does not routinely re-enter Guided after an external mode change.

One important stale subsection remains: ARCHITECTURE.md lines 48–56 describes changed-only setpoints and Copter CONDITION_YAW. At the inspected commit, ArduBridge.cpp lines 1562–1588 and UAV_Model.cpp lines 1317–1403 instead implement the Copter helm velocity stream, altitude feedback and conditional yaw. The selected Plane branch still uses changed-value operations. This discrepancy concerns the specific setpoint path, not the value of the rest of the design notes. No reference documentation was edited.

## 3. Freshness requirement versus freshness-window mechanism

**Rejecting stale navigation is important; a receiver-issued challenge/window protocol is conditional.** A direct, trusted producer-to-MOOSDB-to-bridge deployment has different needs from a chain that caches/relabels/repeats commands. We have not established that the first MAVMOOS deployment requires the additional exchange proposed in decision 0003.

An initial compatibility profile should state its topology, clock/timestamp assumptions, required field presence, age/skew limits, monotonic local watchdog, explicit stop/override handling and restart/re-enable behavior. It should reject demonstrably stale retained input and expire permission even when no new mail arrives. Those are practical goals already partly illustrated by the two reference apps; numeric limits still require the selected vehicle/control profile.

The distinctions remain important: receipt-age checks detect silence at the receiving boundary; original-time checks can detect earlier delay when clock/provenance assumptions hold; neither by itself certifies a common completed solver generation. A relay can conceal producer silence if it continually renews a cached message's apparent origin. If such a relay is in scope, preserve original timestamps/IDs with a qualified mapping, avoid cache renewal, or establish another sufficient origin bound. Fixed windows remain one candidate when calibrated clocks are unavailable, not the default response to every timing question.

This note recommends choosing a mechanism after the deployment assumptions are explicit. It does not silently weaken the earlier strict contract's original-age requirement or establish that a simple watchdog meets every guarantee.

## 4. Restart and takeover at a practical level

The useful initial principle is straightforward: clear eligibility of stored navigation after a relevant restart/disconnect/takeover; require current input and deliberate control enable/acquisition; keep observed FC mode separate; account for work already handed to a library or transport. A restarting bridge should not automatically resume an old motion command merely because it still has the values.

This does not by itself require a general distributed lease system. How much identity/generation machinery is needed depends on multiple producers, relays, queued work and the recovery model. The user's agreement with the practical principle is not interpreted as acceptance of every field/mechanism in the draft authority contract. No formal design record changes to Accepted here.

## 5. Which parts MAVROS supplies as a reference

The inspected MAVROS baseline is **ROS 2**, revision `2d3f4db1101c7950c8817a9d788f0d11106fb261`. The conclusions below are scoped to its actual paths, not every MAVROS plugin or ROS release.

| Proposal | Useful MAVROS reference | What still belongs to MAVMOOS or its upstream controller |
| --- | --- | --- |
| Complete command/decision records | Setpoint plugins consume typed compound ROS messages such as TwistStamped | The upstream controller must form a coherent decision; MAVROS does not compute an IvP solution or certify that its inputs share a generation |
| Freshness/expiry | Timestamp/frame helpers, FC connection diagnostics and known publisher-driven setpoint patterns | The inspected velocity callback does not enforce upstream intent expiry or implement bridge-issued windows. FC timesync is not proof of upstream planner age |
| Restart/takeover | Heartbeat-based connection state, connection-change callbacks, separate mode/armed observations | These do not provide the proposed permission generations, source arbitration or a complete replay/reacquisition policy |
| Modular host | Plugin base/factories, manifest discovery, configurable loading, shared UAS context, message-ID dispatch | MOOS subscriptions/publications, authority gating, vehicle control profiles and lifecycle semantics need a MOOS-native design |

All three earlier ideas can use MAVROS patterns as reference; none is supplied as the exact proposed MAVMOOS guarantee. Functional coverage and module hosting are especially reusable architectural references. ROS-specific plugin binaries are not directly MOOS plugins.

## 6. How MAVROS plugins work in code

There are two similarly named concepts to distinguish. pArduBridge constructs MAVSDK Action/Telemetry/MissionRaw/Param/Passthrough objects: these are SDK API plugins. MAVROS's plugin system hosts its own ROS-facing feature classes. A future MAVMOOS functional module could use SDK APIs without reproducing ROS's plugin loader.

In the pinned MAVROS implementation:

1. A feature class derives from `mavros::plugin::Plugin`. The base carries a non-owning UAS pointer and owns/uses a ROS node. Many plugins create their own sub-node; they remain C++ objects in the host process.
2. The feature's constructor sets up ROS topics/services/parameters. For example, SetpointVelocityPlugin creates TwistStamped and Twist subscriptions. Its incoming-MAVLink subscription list is empty because this selected feature is an outgoing sender.
3. An incoming feature returns message handlers from `get_subscriptions()`. LocalPositionPlugin returns handlers for LOCAL_POSITION_NED and its covariance variant. `make_handler` records message ID/type and supplies filtering/decoding before calling the member handler.
4. `MAVROS_PLUGIN_REGISTER(...)` registers a `PluginFactoryTemplate<Feature>` with class_loader. That factory constructs the feature with the host's UAS context.
5. `mavros_plugins.xml` lists class names, factories and a shared library. CMake builds the standard features into the shared `mavros_plugins` library and exports that description through pluginlib. Dynamic plugins do not imply one separate library per feature.
6. UAS discovers declared classes, applies configured allow/deny patterns, loads their factories, creates instances and keeps them alive. It registers MAVLink handlers by message ID and adds applicable plugin nodes to its executor.
7. Received messages reach `plugin_route`, which calls the registered handlers. Outgoing feature helpers use the shared UAS send path, then Router/transport. Plugin ROS callbacks use executor scheduling; the interface does not imply one process or one dedicated thread per plugin.

For the velocity example, the path is ROS TwistStamped callback → frame conversion/message helper → UAS send → Router/transport. It sends per callback; that class does not independently refresh a cached setpoint on a timer or reject it because its original decision has expired. Typed grouping helps express a command object, but does not prove how its publisher computed it.

## 7. Recommended MAVMOOS module approach

Continue with the provisional preference for one modular MOOS application. The host would own connection/vehicle context, dispatch, diagnostics, transaction state and permitted send paths. Functional modules would declare MOOS inputs and relevant SDK/raw-message interests, retain their feature state, publish results/observations, and submit effects through the host. Helm translation and vehicle-specific control adaptation remain distinct responsibilities.

Start by specifying the C++ module interface and lifecycle. Built-in factories selected by configuration are a simple initial mechanism; runtime shared-library loading can later be evaluated when independently distributed extensions are needed. Static registration can still support optional enabled features and independent source modules. This is a recommendation, not accepted loader/ABI/threading design.

No plugin needs its own MOOSDB connection merely because it is modular, and no module should get an unchecked navigation bypass merely because it can access a library. A shared process is not process isolation; the final design must still define callback ownership and blocking-operation treatment. Claims of a light runtime need measurement, not just a plugin label.

## 8. Primary evidence and provenance

Source manifest records hashes/line counts for 19 files, exact revisions and three remotely matched MAVROS Git blobs. pArduBridge local bytes matched `e34a62ef49c2be42215a69c351850cbd85881486`; unrelated mission changes were preserved. MAVROS bytes come from the pinned source archive already used by this study.

- pArduBridge design: [ARCHITECTURE.md 22–84](https://github.com/cbenjamin23/moos-ivp-uav-base/blob/e34a62ef49c2be42215a69c351850cbd85881486/wiki/pArduBridge/ARCHITECTURE.md#L22-L84), [87–105](https://github.com/cbenjamin23/moos-ivp-uav-base/blob/e34a62ef49c2be42215a69c351850cbd85881486/wiki/pArduBridge/ARCHITECTURE.md#L87-L105), [175–199](https://github.com/cbenjamin23/moos-ivp-uav-base/blob/e34a62ef49c2be42215a69c351850cbd85881486/wiki/pArduBridge/ARCHITECTURE.md#L175-L199); [canonical README 154–178](https://github.com/cbenjamin23/moos-ivp-uav-base/blob/e34a62ef49c2be42215a69c351850cbd85881486/wiki/pArduBridge/README.md#L154-L178).
- pArduBridge actual scalar path: [ArduBridge.cpp 124–138](https://github.com/cbenjamin23/moos-ivp-uav-base/blob/e34a62ef49c2be42215a69c351850cbd85881486/src/pArduBridge/ArduBridge.cpp#L124-L138), [1562–1588](https://github.com/cbenjamin23/moos-ivp-uav-base/blob/e34a62ef49c2be42215a69c351850cbd85881486/src/pArduBridge/ArduBridge.cpp#L1562-L1588); [SetpointManager.h 44–115](https://github.com/cbenjamin23/moos-ivp-uav-base/blob/e34a62ef49c2be42215a69c351850cbd85881486/src/pArduBridge/SetpointManager.h#L44-L115).
- MAVROS base/factory: [plugin.hpp 64–128](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/include/mavros/plugin.hpp#L64-L128), [160–189](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/include/mavros/plugin.hpp#L160-L189), [294–317](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/include/mavros/plugin.hpp#L294-L317); [registration macro 29–32](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/include/mavros/mavros_plugin_register_macro.hpp#L29-L32).
- MAVROS loading/dispatch: [UAS 195–230](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/lib/mavros_uas.cpp#L195-L230), [261–270](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/lib/mavros_uas.cpp#L261-L270), [319–400](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/lib/mavros_uas.cpp#L319-L400), [454–475](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/lib/mavros_uas.cpp#L454-L475); [plugin XML](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/mavros_plugins.xml); [CMake 142–202](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/CMakeLists.txt#L142-L202).
- MAVROS examples: [velocity plugin 43–85](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/setpoint_velocity.cpp#L43-L85), [103–165](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/setpoint_velocity.cpp#L103-L165); [local-position handlers 110–115](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/local_position.cpp#L110-L115).
- MAVROS FC connection evidence: [target heartbeat handling 965–974](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/sys_status.cpp#L965-L974), [timeout callback 1326–1329](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/sys_status.cpp#L1326-L1329), [connection callback 1406–1416](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/plugins/sys_status.cpp#L1406-L1416).

This review updates recommendations, not implementation status. The [original contract package](../contracts/2026-10-28-draft-01/README.md), [priority study](2026-10-28-contract-priorities/README.md) and [proposed decision 0003](../decisions/0003-coherent-helm-and-origin-freshness.md) remain preserved for review.

Subsequent user instruction, October 29: helm changes are now explicitly excluded by [accepted constraint 0004](../decisions/0004-use-stock-moos-ivp.md). The [built-in module sketch](../design/2026-10-29-built-in-modules/README.md) develops the stock-interface route; conditional producer-modification alternatives in this earlier note are not future actions under the current scope.
