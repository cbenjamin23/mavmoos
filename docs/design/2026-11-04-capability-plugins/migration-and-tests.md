# Capability-plugin migration and tests

November 4, 2026. Read-only inventory at `11dc5c308ba8db2140589d1e41f8477cd427062d`. This document proposes compatibility and validation rules for the capability review. It does not implement a refactor, add a control operation, or qualify another vehicle/backend. Stock pHelmIvP remains unchanged; effectful tests remain confined to exact owned native Copter simulator endpoints.

The practical migration is to package the existing Copter control behavior as one coherent **control capability**, with native-FC transactions and the stock-helm adapter as private strategies. Telemetry and parameters remain independent capabilities. `control` is a proposed canonical name in this document, not an already supported configuration token. The broader boundary review belongs in the companion design review; this document inventories the migration cost and tests that would make a refactor reviewable.

## Actual build and composition dependencies

| Surface at this base | Current behavior | Migration consequence |
| --- | --- | --- |
| [`src/plugins/CMakeLists.txt`](../../../src/plugins/CMakeLists.txt) | Builds four `MODULE` targets: `mavmoos_telemetry`, `mavmoos_commands`, `mavmoos_helm`, `mavmoos_parameters`, with files in `build*/plugins`. Telemetry alone links MOOSGeodesy. | Replace the control package's target/source wiring deliberately. Keep a clean build so obsolete module files cannot masquerade as delivered features. |
| [`src/iMAVLink/CMakeLists.txt`](../../../src/iMAVLink/CMakeLists.txt) | The app depends on all four targets. Its compiled default directory is the build's plugin directory; it uses `CMAKE_SHARED_MODULE_SUFFIX` when forming module paths. | Change dependencies and canonical module resolution together. Do not replace platform suffix handling with a hard-coded `.so` in the app. |
| [`Plugin.h`](../../../src/libmavmoos_runtime/Plugin.h) | Development ABI 4, one plugin factory/destructor pair per DSO, a name, subscriptions and callbacks. Every module receives the same `Services` interface. | A new virtual descriptor or changed service layout requires an ABI bump and a matched rebuild. Configuration aliases cannot provide compatibility with old C++ binaries. |
| [`PluginHost.cpp`](../../../src/libmavmoos_runtime/PluginHost.cpp) | Rejects missing libraries/exports, null factories, empty or duplicate plugin names. Startup code uses `RTLD_NOW`, `RTLD_LOCAL`, `RTLD_NODELETE`; plugin objects are destroyed normally. | Retain object ownership and process-resident code. No live reload or unload is needed for this refactor. |
| Host routing | Subscription names are deduplicated for MOOS registration, but mail is delivered to **every** subscribing plugin. All MAVLink messages, resets, input boundaries and parameter-send callbacks are broadcast. There is no capability/dependency/operation-ownership registry. | A descriptor must distinguish read-only subscriptions from effectful request ownership. A deduplicated subscription list does not establish exclusive handling. |
| [`MAVLink.cpp`](../../../src/iMAVLink/MAVLink.cpp) startup | Implicit order is telemetry → commands → parameters → helm, skipping commands/parameters outside simulation and helm when disabled. Explicit `plugins=` values are parsed through `std::set`, so their effective order is alphabetical. | Do not base control correctness on a spelling or incidental load order. Resolve aliases, validate composition, and choose a deterministic effective order before running callbacks. |
| App name checks | `helm_enabled` is recomputed by finding literal name `helm`. Legacy `parameter_list_mode=streamed` accepts only literal loaded names `parameters`/`telemetry`. Status publishes a count, not the effective name/strategy set. | Replace these name tests with validated effective composition. The streamed bench must stay control-free after a rename. Publish actual resolved composition rather than preserving the old count. |
| Configuration dispatch | Host sends every accepted plugin setting to every loaded plugin. Plugins ignore unrelated keys. | Keep existing settings working during packaging. A later namespaced configuration design needs its own validation; do not silently drop or reinterpret old timing values. |

Current shared services already own the final socket effect, common vehicle state, command leases, NAV permission/resume permits, latest waiting target, queue cancellation and expiry. Feature packaging should retain those host-owned gates. A capability declaration is useful for composition and diagnostics; plugins remain trusted code in one process, not security-isolated components.

The existing app also closes its UDP socket and removes the runtime when plugin configuration fails. Preserve that rollback: rejecting a bad composition after binding must leave `CONFIG_ERROR`, no reads/effects, no retained feature instances, and a reusable port. The invalid-streamed MOOS fixture already exercises this boundary.

## Existing public operations versus new proposals

| Existing public surface | Implemented meaning | What a packaging refactor may preserve |
| --- | --- | --- |
| `MAVMOOS_COMMAND`, `id=…,action=guided` | Requests Copter Guided mode through `MAV_CMD_DO_SET_MODE`, mode 4. | Same request parsing, deadlines, host interlock and observed completion. |
| `action=arm` / `disarm` | Ordinary non-forced arm/disarm. Disarm requires current on-ground evidence. | Same guards; no generalized forced operation. |
| `action=takeoff,height=…` | Guided, armed, on-ground Copter takeoff, bounded height and observed height outcome. | Same qualified profile and completion test. |
| `action=hold` | **Native Copter Loiter**, mode 5; completion observes that mode. | Preserve native Loiter semantics, including the FC's pilot-input policy. A complete hold-altitude guarantee is not inherent in the mode-change transaction. |
| `action=land` | Native LAND with observed on-ground completion. | Preserve actual landed evidence and original deadline; do not substitute mode entry for touchdown. |
| `action=rtl` | Native `MAV_CMD_NAV_RETURN_TO_LAUNCH`; implementation observes mode 6 and eventual on-ground outcome. | Package the existing code path. This is not new native flight qualification of RTL; the main lifecycle fixture exercises LAND, not an RTL flight. |
| `MAVMOOS_COMMAND_RESULT` / `MAVMOOS_TX_RESULT` | Operation progress/terminal outcomes versus local send outcomes. | Keep names, request IDs, states and the distinction between queue admission, local acceptance, ACK and observed completion. |
| `MAVMOOS_NAV_CONTROL=acquire/release` | Explicit stock-helm navigation permission; the host can acquire Guided from Loiter. | Preserve explicit acquisition, post-acquisition inputs, source/freshness checks and reset/takeover latching. |
| Stock `DESIRED_HEADING`, `DESIRED_SPEED`, `DESIRED_ALTITUDE`, `IVPHELM_STATE`, `IVPHELM_ALLSTOP` | The adapter consumes ordinary stock output; it labels scalar grouping `ARRIVAL_GROUPED`. Normal `NothingToDo`/`NoGoalBehavior` handoffs have a bounded resume permit. | Keep stock pHelmIvP and existing ordinary altitude behavior unchanged. Preserve original publication expiry and the limits of scalar coherence. |
| `MAVMOOS_HELM_STATUS` | Adapter-local eligibility, acquisition preparation, stop/recovery reasons and field ages. | Keep it available as the stock-helm strategy's status even if the physical DSO is renamed. It must not imply that every independent host gate has passed. |
| `MAVMOOS_PARAM_REQUEST`, `MAVMOOS_PARAM_RESULT`, `MAVMOOS_PARAM_VALUE`, `MAVMOOS_PARAM_SNAPSHOT`, `MAVMOOS_PARAM_TX_RESULT` | Independent read/list service and restricted simulator wind write/readback/restore; paced inventory by default. | Keep the parameter-only composition, original operation deadlines, bounded request window, and read-only work during NAV. |
| `NAV_*`, `MAVMOOS_NAV_STATUS`, raw observations and `MAVMOOS_RUNTIME_STATUS` | Telemetry projection, raw validity/age and shared runtime state. | Preserve quantity meanings, home-relative altitude, validity metadata and zero-output default/shadow behavior. |

Sources: [`CommandPlugin.cpp` at the baseline, now `CopterCommands.cpp`](../../../src/libmavmoos_control/CopterCommands.cpp), [`HelmPlugin.cpp` at the baseline, now `HelmAdapter.cpp`](../../../src/libmavmoos_control/HelmAdapter.cpp), [`ParameterPlugin.cpp`](../../../src/plugins/ParameterPlugin.cpp), [`TelemetryPlugin.cpp`](../../../src/plugins/TelemetryPlugin.cpp) and [`Runtime.cpp`](../../../src/libmavmoos_runtime/Runtime.cpp).

Several superficially similar APIs do **not** exist at this base:

- No public `backend=helm` Hold/Return request, `action=return`, unified `MAVMOOS_HOLD`, or unified `MAVMOOS_RETURN` operation is accepted by the current command parser. Unknown extra request fields are rejected.
- The host's conditional, fresh-position Guided capture after revoking NAV is an internal loss response. It is not a user-addressable helm-hold implementation, a persistent hold controller, or a substitute for native Loiter.
- `NAV_CONTROL=release` revokes navigation. It does not install a helm loiter behavior or create a return route.
- A helm return/hold strategy would need an ordinary mission/behavior activation contract, explicit authority and interruption semantics, and independent flight tests. The existing scalar adapter alone does not supply those workflows.
- Helm-assisted landing, payload actions, mission transfer and arbitrary controller/mode backends are separate future capabilities. Merely publishing them in discovery would not make them implemented.

The current code paths can therefore move behind one control package without adding behavior. Its descriptor can identify native actions and stock-helm navigation separately, and mark future variants unavailable. Changing the default meaning of `hold`, adding automatic fallback between strategies, or adding a strategy-selecting field is a behavior change requiring an explicit contract and tests, rather than a packaging detail.

## Recommended compatibility policy

Preserve the MOOS request/status surfaces above for this migration. Prefer one canonical new package plus configuration aliases; do not ship two forwarding DSOs that each instantiate a full control provider. A caller using old configuration should get one control owner, with the same enabled surfaces as before.

| Legacy selection | Proposed resolved package/strategy selection |
| --- | --- |
| Default receive-only | Telemetry only, as today. |
| Default simulation with helm disabled | Telemetry, parameters, control native transactions; stock-helm strategy disabled. |
| Default simulation with helm enabled | Telemetry, parameters, control with native transactions and stock-helm navigation. |
| Default `simulation_only=false`, `shadow_mode=true`, helm enabled | Telemetry and stock-helm strategy; no MAVLink output and no newly enabled native request surface. |
| `plugins=parameters` | Parameters alone; no telemetry-projection/helm dependency is added. Raw observation publication remains app-owned. |
| `plugins=commands` | One control package with native command surface enabled and stock-helm strategy disabled. |
| `plugins=helm` | One control package with stock-helm strategy enabled and native command surface disabled. |
| `plugins=commands,helm` | One control package, both existing surfaces enabled. |
| Canonical new `plugins=control,…` | Explicit new profile whose enabled variants are documented; retain effect gating independently of discovery. |

The `simulation_only=true` plus `shadow_mode=true` combination follows the old simulation-based selection (including parameters/native surfaces) while still transmitting zero frames; shadow alone does not determine composition. Preserve or explicitly reject that combination rather than silently treating it as the ordinary shadow fixture.

Exact names/enable fields are proposed here. The important rule is that the `commands` alias must not silently enable helm acquisition, and the `helm` alias must not silently add a native-command consumer. Whether those aliases are retained indefinitely or deprecated later should be documented. Retaining them for the initial migration is inexpensive compared with maintaining old binaries.

Reject exact duplicate tokens as today. Reject ambiguous mixtures such as canonical `control` plus a legacy control alias, unless a documented normalization rule gives a single unambiguous profile. The legacy pair `commands,helm` is intentionally one combined selection, not a duplicate provider. Unknown modules, conflicting ownership and incompatible settings should fail the entire startup before operational effects.

If plugin interfaces change, bump the development ABI and rebuild the host, every bundled DSO and all fake factories. An old ABI-4 `.so` should fail clearly; loading it under a changed vtable while still advertising ABI 4 is unsafe. Old mission files can be compatible while old C++ binaries are deliberately incompatible.

Do not preserve `loaded_modules=4` after combining two packages. Keep that field as the actual count and add a versioned composition/discovery record containing resolved canonical modules, alias resolution, enabled strategies and the owner of each effectful request surface. Distinguish:

1. **Implemented/static capability:** code exists for the chosen vehicle profile.
2. **Loaded/enabled composition:** that code/strategy was selected for this process.
3. **Current eligibility/authority:** live observations and explicit permission allow an operation now.
4. **Qualification:** exact software/simulation evidence exists; no discovery value may imply hardware qualification.

Discovery is observational. Querying it must not change mode, acquire NAV, retry work, extend deadlines or enable a strategy.

## Meaningful tests for the new boundary

| Test family | Required assertions | Existing reuse |
| --- | --- | --- |
| Descriptor/discovery | Every canonical package has an identity and declared operation/strategy set. Unsupported helm Hold/Return variants remain unavailable. Report loaded composition distinctly from current authority and qualification. | Extend [`PluginHostTests.cpp`](../../../tests/PluginHostTests.cpp) with fixtures, rather than asserting descriptor text alone. |
| Ownership and startup | Duplicate effect providers are rejected even when plugin names differ. A read-only subscription may be shared. One operator request reaches one owner and produces one transaction; missing owners fail explicitly. Alias normalization produces one object, not two. Failed validation tears down instances/socket without effects. | Existing loader failures/destruction tests and `moos_invalid_streamed` rollback. |
| Composition/compatibility | Exercise telemetry-only, parameter-only, native-only control, helm-only shadow, and full control+parameters. Old aliases produce the same enabled request surfaces as the table. An unavailable/disabled strategy cannot start work. Different explicit token ordering cannot change command-versus-handoff authority. | Existing app fixtures plus a small composition matrix; assert exact effective names/owners, not just a count. |
| Native transaction engine | Preserve malformed/stale/future/duplicate/busy handling, non-forced arm/disarm guards, post-submit observation, ACK versus completion, hard deadlines, reset cancellation, queue expiry and command lease retirement. | Keep the substance of [`CommandPluginTests.cpp`](../../../tests/CommandPluginTests.cpp); after extraction, test the private engine directly or through its coherent package. |
| Stock-helm strategy | Preserve source filtering, cached-before-acquire rejection, retained all-stop semantics, scalar expiry/spread, normal bounded handoff, restart/takeover/home/pose loss, and fresh explicit recovery. | Preserve [`HelmPluginTests.cpp`](../../../tests/HelmPluginTests.cpp) assertions with changed factory/fixture wiring only. No pHelm changes. |
| Shared host authority | Explicit native HOLD/LAND/RTL must defeat suspended auto-resume and queued Guided acquisition; old conditional position captures must not leak after replacement/reacquisition. Bare host permits must fail on mode, pose, landed, armed and session continuity loss. | [`AuthorityConflictDiagnostic.cpp`](../../../tests/concurrency_review/AuthorityConflictDiagnostic.cpp), [`ResumePermitDiagnostic.cpp`](../../../tests/scheduling/ResumePermitDiagnostic.cpp), [`RuntimeTests.cpp`](../../../tests/RuntimeTests.cpp). |
| Parameter/NAV composition | Parameter transfer cannot renew helm fields or monopolize the send lane; pose loss leaves independent reads intact, session loss cancels old work. Restricted writes remain blocked in flight and during command transitions. | [`SchedulingTests.cpp`](../../../tests/scheduling/SchedulingTests.cpp), ingress/runtime tests and write interlock tests. Keep actual control+parameter modules in the deterministic integration, rather than replacing them with mocks. |
| Receive-only/shadow | Descriptor presence, aliases, native actions and fresh helm inputs all produce zero MAVLink output outside the explicit effectful simulator profile. | `runtime`, `moos_normal`, `moos_shadow`, and existing fixture peer observation. |
| Process lifecycle | Matching ABI checks, duplicate-owner rejection, normal destruction, idempotent clear, process-resident DSO code and owner-thread reset ordering survive the package move. | Loader tests, sanitizer suite and existing disconnect/cleanup fixtures. |

A shared `MAVMOOS_COMMAND` entrypoint needs one routing decision. If capabilities are later separated further, several plugins must not all subscribe to that variable and independently reject or execute the same request. Route ownership can be by parsed operation under one gateway; the current multicast subscription mechanism is not that gateway. A single control package avoids introducing this problem for the immediate migration while still allowing private native/helm strategies.

Tests should inspect emitted MAVLink IDs/parameters, original deadlines, authority transitions and observable outcomes. A test that only verifies a renamed factory returns `name()=="control"` is insufficient. Keep request IDs/outcome schemas stable; a module rename must not change what counts as a completed LAND, a successful arm, or a resumed helm decision.

## Harness and build work that must be carried with a refactor

[`tests/CMakeLists.txt`](../../../tests/CMakeLists.txt) directly compiles `CommandPlugin.cpp` and `HelmPlugin.cpp` into different unit-test executables, and passes separate DSO paths to scheduling/authority/resume tests. Combining files by linking both unmodified public factories would create duplicate exports. Extract private engines/adapters or provide one coherent factory, then update unit and integration wiring together. Scheduling currently loads commands → parameters → helm; it must load the new control provider **once**, with both strategies enabled where required.

The same split exists independently in [`LinuxCore.cmake`](../../../tests/ingress_review/LinuxCore.cmake), which builds commands/helm/parameters without the MOOS app or telemetry Geodesy dependency. [`LinuxConcurrentCore.cmake`](../../../tests/concurrency_review/LinuxConcurrentCore.cmake) passes their separate target paths to integration diagnostics. Update both recipes, including argument counts and module identity expectations. Keep a pure-core path so a control package does not accidentally acquire an AppCasting/MOOS dependency just to parse existing plain `Mail` records.

The full Linux [`build-app.sh`](../../../packaging/linux/build-app.sh) uses ordinary root CMake and records actual `mavmoos_*.so` artifacts. The independent core snapshot scripts already collect the source tree's C++/Python/CMake files, but their build recipe still needs the new target graph. Use a fresh container/source archive and record exact hashes; earlier Linux 17/17 evidence does not cover a new package or ABI.

Native/shadow fixture coupling is mostly to public behavior, which is useful:

- [`meta_vehicle.moos`](../../../missions/copter_sitl/meta_vehicle.moos) enables helm but omits `plugins=`, so workflow, shadow, concurrent and process-fault missions depend on the app's default selection. Make the effective resolved composition an assertion in each result; do not infer it from files present in the directory.
- [`run_parameters.py`](../../../tests/parameters/run_parameters.py) explicitly selects `plugins=parameters`, names `mavmoos_parameters.so` for its artifact hash, and requests legacy streamed inventory on its control-free ground bench. This case must remain independent of control/helm and preserve its fresh-ground prerequisite for writes/list.
- [`run_moos.py`](../../../tests/run_moos.py) intentionally uses `plugins=parameters,commands` with streamed mode to test invalid composition. It must continue failing after the old command alias resolves to control; add a canonical-control spelling as a second invalid case if needed.
- Concurrent and fault runners currently hash **every** `mavmoos_*` file in the plugin directory. A fresh build directory prevents stale artifacts; future provenance should distinguish delivered artifacts from the exact loaded set reported by the host.
- The lifecycle/fault/concurrent probes consume `MAVMOOS_COMMAND_RESULT`, `MAVMOOS_HELM_STATUS`, `MAVMOOS_NAV_CONTROL`, runtime authority and existing stock fields. Preserving those surfaces lets them remain behavioral regressions instead of being rewritten to approve new behavior.

## Feasible validation sequence

1. Freeze the package/alias/descriptor contract and write the composition tests first. Keep the existing request grammar and strategy meanings. Preserve the `11dc5c3` source/evidence boundary; add new records instead of editing earlier archives.
2. Refactor private code and build wiring in one integrator-owned pass. Rebuild all DSOs/factories against the new ABI if changed. Run unit, real-module fake-I/O, app rollback/zero-output, cleanup and sanitizer tests. Verify a clean build's actual artifact set and `--interface`/example text.
3. Run the updated independent Linux core recipe in a fresh isolated container. A full Linux app build is additional evidence; do not describe a core-only run as an app or flight test. Keep the ordinary external altitude behavior and stock dependency pins.
4. Freeze one exact host/module/probe/harness package before native work. Run the corrected lifecycle fixture with its explicit neutral simulator pilot input and independent altitude/RCIN check, the parameter-only bench, strict concurrent bulk and interrupted-transfer/recovery case. Repeat strict bulk on the same package; the earlier streamed candidate passed once and failed on repeat.
5. Run the six existing process-fault scenarios on the frozen package when shared host/control callbacks or reset routing change. The optional CMake native matrix contains eleven cases (lifecycle, parameter bench, three concurrent cases, six faults). Serialize native runs under the common lock and the simulator's fixed-helper-port constraint; do not parallelize by weakening endpoint ownership. Linux/software work may run independently of the frozen native queue.
6. Archive failures and fixes separately, including exact source/artifact/composition identities, generated mission/defaults, before/during/after authority, complete snapshot when claimed, raw FC observer data, neutral pilot evidence for native Loiter, ground/disarmed outcome and scoped cleanup. A session-cancelled inventory is not a completed inventory; a corrected fresh fixture is not a rewrite of an earlier failed batch.

The existing fixtures already demonstrate why these layers are needed: deterministic send fairness did not prevent a remote FC full-list telemetry interaction, and an apparently successful native mode transition did not establish Loiter altitude hold under the simulator's default pilot input. Packaging changes should carry those tests and qualifications forward rather than treating a new module boundary as new capability evidence.

## Decisions still explicit

The review can confidently recommend coherent packaging, alias normalization, an ABI rebuild if interfaces change, discovery/ownership checks and reuse of existing behavior tests. The following remain product/contract decisions rather than automatic consequences of that packaging:

- Canonical capability/strategy names and alias deprecation lifetime.
- Whether a future unified Hold/Return API is desired, its default strategy and whether unsupported variants reject explicitly or are absent from discovery. Automatic fallback is not implemented.
- Whether shared request ownership is per public variable or per parsed operation when future capabilities are separate providers.
- What future helm Hold/Return workflows require from ordinary mission configuration, and their completion/loss/pilot-interaction semantics.
- Which native RTL/other future strategy scenarios need new independent simulator qualification before a wider capability claim.

No source/build/native mutation was performed during the initial inventory. The test rows above are proposed migration acceptance; the separately authorized test migration checkpoint below records its own narrower checks.

## Test migration implementation checkpoint

The integrator subsequently selected canonical module `arducopter_control`, ABI 5 and startup flags `control_native_actions`/`control_helm`. The existing native transaction and stock-helm suites now instantiate private components through `makeCopterCommands` and `makeHelmAdapter` with an explicit pure Copter backend. Their behavioral assertions remain; ABI/export checks now belong to the actual combined module. Current Runtime integration rigs load that provider once, while bare-host comparisons disable its public facets. The legacy streamed-list host fixture deliberately has no control provider; a new group requires control acquisition, navigation and commands to fail closed when no provider is loaded. The archived `943ff39` backlog reproducer remains unchanged.

Private local checks passed the native component's eight groups, the helm component's 37 cases, nine new combined-plugin groups, 35 Runtime groups, five ingress/runtime groups, authority conflicts, paired bare-host/real-helm resume cases, the write interlock and all 12 scheduling groups. Six existing MOOS probe translation units passed syntax checks. These are private fake-service/fake-I/O checks against a development snapshot, not a claim that a final shared build, Linux run, MOOS delivery or new native flight has passed. Their logs remain under ignored `build-test-migration-private` for the integrator's execution archive.

Existing app/native probes now record `MAVMOOS_CAPABILITIES`. A shared evidence validator checks the observed schema/execution mode, exact resolved module set, enabled native/helm variants, unique request ownership and internal-only Guided recovery, then hashes the exact loaded artifacts. Files merely present in the plugin directory remain separately recorded as delivered artifacts. The validator is observational and adds no control request. Lifecycle, parameter and flight/fault behavior is unchanged; no unified Hold/Return grammar was introduced. Final packaged validation and any simulator results must be recorded separately by the integrator.
