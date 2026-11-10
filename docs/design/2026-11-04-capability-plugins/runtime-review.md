# Runtime and vehicle-control boundary review

November 4, 2026. Reviewed baseline: `11dc5c3` (`feat: validate concurrent parameters and helm authority`), initially clean. The initial sections record source review and an interface proposal; the final sections separately record the frozen ABI 5 implementation and private software evidence. The relevant existing files are `Runtime.cpp`, `Plugin.h`, `PluginHost.cpp`, `HelmPlugin.cpp`, `CommandPlugin.cpp`, `CopterModel.cpp`, and their tests. There is no `ControlModel` file at this baseline; the current pure model is named `CopterModel`.

## Recommendation

Ship one actual ArduCopter vehicle-control module that contains stock-helm adaptation, explicit command transactions and the Copter control profile. Keep their internal classes small and separately testable. The meaningful capability is vehicle control; explicit native FC actions and helm-driven motion are coordinated implementations of that capability. A separate shared object for each mode or internal state machine would expose deployment choices that do not belong in an operator's plugin list.

The host must still own the single connection, decoded observation cache, owner clock, authority state, normal-handoff permit, command-operation leases, bounded queues, original expiries, cancellation and final send. The profile supplies vehicle meaning and pure construction of proposed work. It never obtains a socket or a function that bypasses the host's send checks.

Keep telemetry projection and the parameter protocol independent. A parameter-only ground-bench process must not need to load a control module merely to observe ground state or perform its existing restricted simulator write/readback. Loading a control capability also does not enable helm authority: explicit acquisition and new eligible stock-helm inputs remain necessary.

The first implementation should preserve the existing Copter behavior. Broader vehicle support, different altitude/facing policies, autonomous holding modes, new loops and arbitrary plugin ABIs are separate changes.

## Current ownership and Copter assumptions

Line references below identify baseline `11dc5c3`; movement during the refactor will change them.

| Location | Existing assumption | Proposed owner |
| --- | --- | --- |
| `Runtime.cpp:44–57` | A heartbeat must be Quadrotor/ArduPilot; Guided is mode 4, Loiter is mode 5; Loiter is temporarily tolerated during acquisition | A compatible bound vehicle identity plus Copter profile mode predicates. The host decides whether a mismatching profile invalidates control; the profile does not own the connection |
| `Runtime.cpp:33–38,105–111` | Heartbeat/landed/pose/estimator lifetimes are 2/2/0.5/3 s, and flight readiness requires home, pose, landed and estimator state | Lifetime evaluation remains a common host utility; the Copter profile chooses which observations are required and their bounds |
| `Runtime.cpp:79–85` | EKF flags must contain mask 51 and nonnegative finite position variances | ArduPilot observation interpretation/profile eligibility. It is not a universal MAVLink estimator definition |
| `Runtime.cpp:153–168` | Only ARM/DISARM, TAKEOFF, LAND, RTL and SET_MODE are admitted; ARM requires observed ground and estimator; TAKEOFF requires armed Guided ground and height 1–20 m; mode changes only select 4 or 5 | Copter profile command validation. Generic ID/finite-number/deadline/capacity checks and command-operation interlocking stay in the host |
| `Runtime.cpp:193–206` | Acquisition is allowed from airborne armed Guided/Loiter; Loiter acquisition queues Guided, with a five-second allowance | Copter profile returns an acquisition plan. The host privately tags setup work, owns its lifetime, and cancels it on revocation |
| `Runtime.cpp:175–190,286–297` | A normal resume requires uninterrupted Guided/armed/airborne prerequisites | The profile provides the motion-eligibility predicate; the host consumes/revokes its own permit before ingress, after observations and at resume entry |
| `Runtime.cpp:239–275` | Revocation may produce one current-position hold only while still freshly Guided; its altitude is global MSL, and its life is limited by pose age | Copter stop-response construction plus a host-owned response slot, expiry and cancellation. This is distinct from explicit native Loiter HOLD |
| `Runtime.cpp:259–264` | Navigation uses course/speed/home-relative height; speed 0–5 m/s and requested height 1–50 m; target life is bounded | Copter navigation envelope, with common host finite/original-expiry/authority checks |
| `Runtime.cpp:266–275` | Raw state is converted into the P2 Copter model, with a fixed home revision and specific sample windows | Copter control backend adapter. A single bound home still requires change invalidation; the fixed revision is not a general multi-home model |
| `Runtime.cpp:314–324` | FC families 33/245/193/242 are requested at pinned Copter rates 20/4/2/2 Hz | Profile-specific telemetry subscription demands, not generic control authority. Preserve the current rates during a structural refactor; do not accidentally remove them from the parameter/telemetry bench |
| `Runtime.cpp:342–357` | NAV dispatch interval is 0.1 s, and the P2 model builds/encodes Copter frames | The host schedules the profile's required cadence; the Copter backend builds its payload, and the host rechecks immediately before sending |
| `Runtime.cpp:96–101,250,350,364–373` | Target/source IDs are 1/1 and 245/191; ACK targeting uses that source identity | Shared connection binding/configuration, not vehicle kinematics. Keep the tested defaults, but pass the binding rather than hiding constants in new plugins |
| `CommandPlugin.cpp:78,128–168,187–213` | Public actions, command parameters, mode 4/5/6/9 interpretation, observed completion, takeoff altitude tolerance and prerequisites are Copter-specific | `CommandTransactions` inside the control capability, delegating mappings and predicates to the Copter profile |
| `CommandPlugin.cpp:37–42,81–90,174–183,222–228` | Request history, duplicate/busy handling, queued age, ACK versus confirmation, terminal callbacks and original operation deadlines | Reusable transaction component. Do not fold these into the transport or make ACK alone mean completion |
| `HelmPlugin.cpp:73–94,178–183` | Stock `IVPHELM_*` stop/state and independent course/speed/altitude publications are grouped by arrival and bounded spread | Reusable `StockHelmAdapter`. It continues to expose `ARRIVAL_GROUPED`, never a fabricated producer generation |
| `HelmPlugin.cpp:92–93,139–172,193–212` | Envelope, airborne requirement, Guided/Loiter numeric tests and operator preparation diagnostics are Copter-specific | Profile predicates and profile-labelled diagnostics used by the stock adapter. Preserve the adapter's existing lower-level guard behavior until an intentional change is reviewed |
| `CopterModel.cpp:41–118` | ArduCopter prerequisites; course to north/east velocity; altitude feedback gain 0.5 with vertical speed limited to ±1 m/s; course-facing above 0.05 m/s; masks 2503/3527/3576 | Plain Copter model/backend library; these are control law and frame choices, not a generic MAVLink sender |
| `CopterModel.cpp:120–147` | LOCAL_NED velocity/yaw targets and GLOBAL_INT current-position stop response, with wire time wrapping | Copter payload encoding. Host retains source binding, codec sequence ownership, fresh final eligibility and I/O |
| `iMAVLink/MAVLink.cpp:159–163` | Effectful runs require owned `arducopter`, loopback and the exact target/type/autopilot | Retain as the explicit current simulator deployment restriction. Extracting a profile must not silently broaden it to other FCs or hardware |

The parameter module also has intentional ArduPilot scope: C-cast interpretation, the paced indexed protocol and `SIM_WIND_SPD` write allowlist. Those belong to the parameter capability/profile rather than the Copter control module. The host's final disarmed-ground/command-transition write gate must remain independent of whether control is loaded. Standard decoded landed state is common evidence; the meaning of a vehicle being eligible to fly or accept setpoints is profile policy.

`Services::sendCommand` is currently a vehicle-control service despite its broad name. Do not route telemetry-rate, camera or other future protocol commands through the Copter allowlist and NAV-revocation rule merely because they also use MAVLink COMMAND_LONG. A broader transaction service would need explicit capability/effect classification while preserving one host serializer and ACK-correlation policy.

## Concrete minimum interface

The names here are proposed; the root should freeze a single shared header before parallel edits. Prefer a borrowed startup-lifetime provider over a second loader or global singleton:

```cpp
enum class ControlUse { Acquire, ContinueAcquisition, Motion, NormalResume, StopResponse };
struct ControlCheck { bool eligible; std::string reason; };
struct AcquirePlan {
  bool eligible = false;
  std::string reason;
  std::optional<CommandRequest> setup_command;
  double mode_grace_until = 0; // clipped/validated by host, never authority itself
};

class VehicleControlBackend {
public:
  virtual ~VehicleControlBackend() = default;
  virtual std::string profileId() const = 0;
  virtual bool compatible(const VehicleIdentity&) const = 0;
  virtual ControlCheck ready(ControlUse, const Vehicle&, double now) const = 0;
  virtual ControlCheck commandEligible(const CommandRequest&, const Vehicle&, double now) const = 0;
  virtual ControlCheck navigationEligible(const NavigationTarget&, double now) const = 0;
  virtual AcquirePlan acquire(const Vehicle&, double now) const = 0;
  virtual BuildResult buildNavigation(const NavigationTarget&, const Vehicle&,
                                    const ConnectionBinding&, double now) const = 0;
  virtual BuildResult buildStopResponse(const Vehicle&, const ConnectionBinding&,
                                      double now) const = 0;
};

class Plugin {
public:
  // Existing lifecycle methods remain; declaration advertises one coherent role.
  virtual VehicleControlBackend* vehicleControl() { return nullptr; }
};
```

`VehicleIdentity`, `ConnectionBinding` and `BuildResult` are small shared value types. `BuildResult` must carry a reason and a bounded proposal; it must not carry an executable callback that can transmit. Either return a typed MAVLink payload and let the host encode it, or pass the host-owned codec by reference during synchronous construction. The latter is a smaller first move from `CopterModel`, but the host still performs a fresh clock/eligibility check after construction. The provider cannot return a renewed expiry or its own authority token. A response deadline must be no later than the original pose bound and existing policy limit.

The profile should expose named predicates such as `guided`, `acquirableMode`, `nativeHold`, `landMode` and `rtlMode`, or use typed profile actions, so neither `HelmAdapter` nor `CommandTransactions` copies raw mode IDs. Keep native actions and helm strategies together under this capability; do not create `LoiterPlugin`, `GuidedPlugin`, or `AltitudePlugin` merely because their implementations differ.

A `ControlUse` distinction matters. The current host allows fresh armed/airborne Loiter during explicit acquisition but requires Guided for motion and normal resumption. Replacing this with one `ready()` boolean would reintroduce the paused-permit takeover bug or break supported Loiter-to-Guided acquisition. The host retains its independent command-lease and permit checks even when the same module contains both producers.

The exact identity fields should be facts in the decoded cache, not inferred from which shared object was loaded. Transport validation still binds incoming source IDs and configured type/autopilot; the active control backend independently rejects a mismatching vehicle. Without a control provider, ordinary bound telemetry remains observable and all effectful control services return an explicit unavailable result. This is not a claim that every existing projection supports every MAVLink vehicle.

## Provider registration and lifecycle

1. Keep startup composition and one logical owner. `PluginHost` currently rejects duplicate *names* (`PluginHost.cpp:50–55`), which is insufficient for two differently named control modules. Reject a second `vehicle_control` provider by capability role before accepting it. Do not let load order select a winner.
2. Obtain the borrowed provider pointer only from the accepted plugin instance. The plugin owns its backend and internal components. The host clears the binding before destroying the owning instance, and effectful services stay closed until startup configuration/provider validation is complete. A rejected provider must leave the previous accepted registry unchanged.
3. Missing control is valid for telemetry/parameter-only composition, and invalid for a configuration requesting helm or explicit vehicle-control effects. Report provider/profile IDs and enabled paths in the appcast/status; a plugin filename is not enough to explain capability availability.
4. Preserve `RTLD_NODELETE` and normal object destruction. Code residency prevents code unmapping and address-reuse problems; it does not make a destroyed object's provider pointer valid. Never put a `Services*`, authority state, pending transaction or provider registration into a DSO static/global merely because code is resident.
5. Destroy internal command/helm state before its backend and before the Runtime services they reference. `Runtime` currently declares `m_host` before queues/counters, so reverse member destruction would destroy some service state before plugin objects. Existing trivial destructors do not exercise this, but a new combined plugin must not call services from its destructor after those members die. Clear the plugin host explicitly in a Runtime destructor/body while services still exist, or require and test inert destructors. Prefer explicit ordering.
6. Add the provider interface at the next development ABI and rebuild all modules. A C++ virtual interface change is not ABI 4-compatible just because it has a default method. Keep factory ABI rejection tests. This remains one trusted process and a matching-toolchain development ABI, not an untrusted extension security boundary.

Capabilities and implementation profiles are separate metadata. `vehicle_control` is the role; `arducopter` is the present implementation/profile. A future alternative can replace the provider at startup without adding a second simultaneous writer or claiming that Rover uses Copter flight prerequisites. Dynamic selection does not imply live unloading or replacement.

## Suggested file movement

| Baseline | Proposed result | Intent |
| --- | --- | --- |
| `src/plugins/HelmPlugin.cpp` | `src/libmavmoos_control/StockHelmAdapter.{h,cpp}` | Preserve source/publication checks, state transitions, expiry, grouping and diagnostics as a plain component; remove shared-library exports from it |
| `src/plugins/CommandPlugin.cpp` | `src/libmavmoos_control/CommandTransactions.{h,cpp}` | Preserve request/ACK/observed-completion lifecycle; delegate Copter action mapping/guards to the selected profile |
| New | `src/plugins/ArduCopterControlPlugin.cpp` | One real factory, combined subscriptions/configuration and deterministic lifecycle delegation, one owned backend, one provider declaration |
| New | `src/libmavmoos_control/ArduCopterBackend.{h,cpp}` | Move mode meaning, motion/command envelope, acquisition setup and bridging to the existing pure model out of Runtime |
| New | `src/libmavmoos_runtime/ControlBackend.h` | Shared values and the borrowed pure backend interface; no socket or raw-send service |
| `CopterModel.{h,cpp}` | Keep and reuse initially; optionally relocate to the control library later | Avoid rewriting the tested P2 model or merging the entire `ManagedEgress` laboratory into the runtime during this refactor |
| `Runtime.cpp` | Smaller owner/authority/scheduler with backend calls | Keep command/NAV/parameter queue semantics and final send timing together |
| `Plugin.h`, `PluginHost.*`, app/build wiring | Provider-role metadata, unique-provider validation, ABI bump and new module selection | Startup composition replaces the two control DSOs; generic capabilities remain independent |

For combined callback order, preserve command expiry/observation processing before helm motion evaluation. Dispatch `onInputBoundary` to both components before any new input updates can replace old evidence. A new explicit command arriving in the same tick must revoke authority before any NAV leaves the host; the safety property must not depend on this internal delegation order alone.

Avoid a source-level include of the old `.cpp` files with their factory names renamed by preprocessor macros. That would package two modules into one binary without creating clean component ownership or a coherent provider boundary. Small internal factories can help a mechanical move, but their lifetime/subscription/terminal behavior must be explicit.

Telemetry-rate demands need particular care. Their 20/4/2/2 Hz profile came from actual concurrent-transfer failures, and the parameter bench also uses the common runtime. The first extraction can leave the existing block unchanged and label that remaining Copter-specific configuration; a later move should place it with telemetry subscription/profile data, using a non-authority protocol path. Do not make a telemetry-rate command revoke control as though it were an operator mode change.

## What must remain unchanged and what requires a decision

Structural changes: one control DSO instead of two; internal component classes; one borrowed backend; capability/provider metadata; matching ABI; source/build/test wiring. Keep existing MOOS requests, result statuses, original deadlines, source checks, frame quantities, masks, control law, facing policy, mode choices, stream rates and simulator-only restrictions.

Configuration compatibility is a deliberate choice. Recommended first delivery: default composition becomes `telemetry,parameters,arducopter_control`, with explicit internal `helm_enabled` gating. Either reject old `helm`/`commands` names with a concrete migration message or normalize the complete legacy pair once to the combined module. Never silently load the combined module twice or silently enable an unrequested helm path. Command-only and shadow-only profiles need explicit tests because their former separate-DSO selection no longer expresses enablement.

Do not introduce a new user-visible native-vs-helm method selector merely to demonstrate packaging. The current APIs already distinguish explicit lifecycle actions from continuing helm navigation. Later alternatives such as native mission execution versus helm routing, or a native camera ROI versus a helm-driven pointing policy, should share one coherent capability contract and declare the different completion/cancellation/authority semantics. A capability name alone does not make those methods equivalent.

Native Loiter remains mode 5 with pilot-throttle semantics, and the fixture's neutral simulated pilot remains outside product control. A different hold mode, new altitude controller, altered loss action, automatic arming, generalized writes or changed timing is a separate behavior change requiring explicit rationale and fresh native evidence.

## Preservation and new acceptance checks

| Invariant | Existing evidence/tests to keep |
| --- | --- |
| RX_ONLY/SHADOW emit zero; effects remain owned-native-only | `RuntimeTests`, `run_moos`, shadow/native app guards |
| No retained or wrong-producer stock-helm intent; `ARRIVAL_GROUPED` is honest | `HelmPluginTests`, stock-helm mission, reset/source/field-age cases |
| Original command deadlines beat late ACK/state; ACK differs from completion | `CommandPluginTests`, ingress command-deadline diagnostic, command timeout fault |
| Explicit commands defeat paused automatic recovery and clear obsolete response work | `AuthorityConflictDiagnostic`, `SchedulingTests` |
| Paused permits die on takeover, disarm/landed state and observation gaps; valid Guided pause and explicit Loiter acquisition still work | `ResumePermitDiagnostic`, 33 baseline `RuntimeTests` groups |
| Private internal acquisition origin cannot be impersonated by an `acquire-` ID | Runtime and scheduling origin regression |
| Parameter lane remains independent; own ARM cannot be followed by a waiting SET | `WriteInterlockDiagnostic`, parameter cases and simultaneous scheduling |
| Full paced transfer does not change fake NAV opportunities; no full-list burst is silently reintroduced | The 12 scheduling groups, paced inventory unit cases and exact-source native concurrency |
| Copter conversion, masks, vertical feedback, idle yaw and bounded stop response remain identical | P2 model/fake/loopback tests and payload comparisons |
| Native lifecycle still includes handoff/expiry/Loiter/reacquire/LAND/disarm with independent observations | Revised full Copter workflow, independent RCIN/height proof and fault cases |

Add unique-capability tests with differently named provider fixtures, missing-provider control denial, incompatible identity denial, partial-load rollback, old ABI rejection, correct component destruction/provider unbinding, and no control side effect during construction/configuration. Verify standard telemetry and the ground parameter bench with no control provider. Verify command-only and shadow/helm compositions explicitly. Load/unload test *objects* repeatedly while code remains resident; do not turn this into a promise of live module replacement.

Do the refactor in small checkpoints: freeze shared header/ownership and baseline source identity; move plain components without changing behavior; register the combined provider and replace Runtime's selected Copter helpers; update app composition/compatibility; then run existing model/host/component/combined tests, sanitizers, stock MOOS integration, native workflow/fault/concurrency cases and the clean Linux core recipe against exact frozen sources. Previous passes do not validate moved code automatically, and fake-clock equality is not an OS/FC timing guarantee.

## Frozen interface and pure-backend implementation checkpoint

The root froze ABI 5 in `ControlBackend.h`, `Plugin.h` and `ControlComponents.h`. The implemented interface expresses the proposed control-use distinctions through separate `flightReady`, `acquisitionAllowed`, `activeModeAllowed` and `helmVehicleReady` predicates, rather than the illustrative `ControlUse` enum above. `Plugin::controlBackend()` provides the borrowed interface; descriptors expose capabilities, request ownership and exclusive resources. `ArduPilotObservations.h` keeps the existing critical observation demands usable independently of a motion-control provider.

`src/libmavmoos_control/ArduCopterBackend.cpp` now implements that frozen pure interface. It has no `Services`, socket or send reference. The profile ID is `arducopter`; its compatible tested identity remains Quadrotor/ArduPilot. It preserves the distinct original host/helm freshness checks, Copter command guards, acquisition-mode proposal, field/envelope rules and exact P2 model adapter math. The generic host remains responsible for request identity, finite command parameters, original deadlines, queue capacity, authority and final send. A defensive pre-conversion check rejects nonfinite or unrepresentable direct frame-clock inputs; it does not alter valid owner-clock behavior.

`tests/control/ControlBackendTests.cpp` passed **18 groups normally and under ASan/UBSan**, with no compiler or sanitizer diagnostics and `detect_leaks=0`. Generated-MAVLink roundtrips check cardinal velocities/course facing, altitude feedback/clamps, zero-speed yaw, target/source IDs, global MSL stop response, sequence handling and explicit wire-time wrap. Guard cases cover separate acquisition/motion/helm readiness, exact observation-age boundaries, nonfinite values, unsupported commands, original expiry and P2's strict sample-expiry boundary. In particular, this does not accidentally tighten the host's inclusive age predicate to the model's stricter frame-construction boundary.

The normal log, sanitized log, exact copied sources, compile commands and hash manifest retain this private checkpoint. Every copied compile input still matched the worktree after execution. These results do not validate the root-packaged combined plugin, native FC control, Linux build or hardware; those require separate integration checkpoints. Only the assigned new backend/test files and this review archive were changed by this review stream.

## ABI 5 integration review and provider-lifetime closure

Read-only review of the first ABI 5 host integration found three concrete lifetime schedules, reproduced with ASan and no socket I/O:

1. A valid startup configuration rebuilt its owned backend. `Runtime` still cached the freed old pointer; its next status query dereferenced it.
2. Reverse teardown destroyed a last-loaded control provider before an earlier observer's destructor called `Services::revokeNavigation`. The host dereferenced the deleted backend during that callback. `RTLD_NODELETE` preserved code, not the object.
3. A configuration rebuilt its backend and then introduced a duplicate capability. Validation threw, but a caller that caught the exception could query a Runtime retaining the freed provider and partially configured composition.

All three baseline traces contain a real heap-use-after-free report, with allocation/free identities retained. The root fixed these paths: refresh the borrowed provider after successful configuration; detach it and disable effects before plugin teardown; and, on failed configuration, detach the provider, invalidate configuration, clear authority/queues/composition, then rethrow. Further effects, loading and settings require a fresh Runtime after that failure. Services storage remains alive while observer destructors run.

The **exact original diagnostic module bytes** were reused against the fixed host; all three formerly failing schedules passed under ASan/UBSan. The regressions were then promoted into `tests/control/ProviderLifetimeFixtures.cpp` and `ProviderLifetimeTests.cpp`. The normal and sanitized fixture suites pass **four groups**, including caller continuation after rejected configuration, fresh observations that cannot revive it, observer callbacks during teardown, and zero unintended I/O. The fourth group selects a deliberately synthetic mode-77 backend: its acquisition proposes SET_MODE 77, observed mode 77 enables a valid NAV proposal, and the host still rejects ungranted, revoked and expired work. This demonstrates delegation without claiming a second supported vehicle or changing the production Copter profile.

The final shared-host source also closes request-metadata omission: every advertised `access=request` capability must declare its request variable in that module's `request_owners`; observation-only subscriptions remain permitted. Duplicate provider/resource/capability/request-owner checks remain independent. Stream aggregation keeps first-seen wire order while choosing the fastest valid interval for each family, preserving the prior rate requests for control-free telemetry/parameter compositions. Missing providers close control services. Host command leases, normal-handoff invalidation and final clock/freshness gates remain in Runtime; the final NAV gate explicitly rechecks permission after backend construction.

The baseline sources, baseline replacement, baseline teardown, baseline rejected-configuration trace, fixed identical-fixture replacement, fixed teardown, fixed failure continuation, normal promoted suite, sanitized promoted suite, final copied sources and source/artifact manifest preserve the separate checkpoints. The preceding initial private fixed checkpoint is retained in its own subdirectory. `detect_leaks=0` remains declared; no sanitizer suppression or manual memory unpoisoning was used. Baseline traces include external-symbolizer warnings, which are retained rather than hidden.

No further blocker was found in the reviewed shared-host paths after these closures. The review stream added only its assigned new backend/tests/fixtures and archive; production host fixes belong to the root. Private copied-source passes remain distinct from final packaged, Linux and native execution results.

A later independent clean Linux ARM64 core execution passed **21/21 CTests in 4.08 s**, including the combined module, provider-lifetime/delegation, selection and preserved authority/scheduling tests. All 96 source paths and 480 generated headers matched the frozen inputs/worktree; the offline container built as UID 65534 with network disabled and was removed. The separate Linux archive retains exact commands, source/header inputs, ELF hashes and the initial host-sandbox tooling failure. This validates the Linux core recipe, not the MOOS application, native SITL or hardware.
