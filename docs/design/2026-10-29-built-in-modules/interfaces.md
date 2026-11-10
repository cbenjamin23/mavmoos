# Module and service interfaces

October 29, 2026. Proposed semantic API sketch. The signatures below use C++-like notation for review only; types/names are placeholders, not compilable headers, a stable ABI, a MOOS wire schema or implementation. [Scope and responsibilities](README.md).

## 1. Built-in registry and lifecycle

```text
BuiltinRegistry:
  describe_available() -> List<ModuleDescriptor>
  create(module_id) -> Owned<FeatureModule> | UnknownModule

FeatureModule:
  describe() const -> ModuleDescriptor
  configure(ConfigView, ProfileView) -> BindingPlan | ConfigurationError
  start(ReadOnlyContext) -> Status
  on_event(EventView, ReadOnlyContext) -> Effects
  on_tick(MonoTime, ReadOnlyContext) -> Effects
  stop(StopReason) -> FinalStatus
```

`ModuleDescriptor` names the feature, provided capabilities, required host services/features and supported configuration revision. `BindingPlan` declares resolved MOOS inputs/output keys, backend event interests, periodic work and required observation kinds. Configuration may resolve a namespace/profile before the host registers subscriptions. Factories are compiled and linked with the application; selection instantiates objects rather than loading arbitrary libraries.

Lifecycle: construct → configure → validate dependencies/names/capabilities → bind → start → events/ticks → stop. Unknown modules, unresolved dependencies, conflicting publication owners, invalid profile bindings or unsupported effectful capabilities are configuration failures. No active vehicle effects are permitted merely because a factory or constructor ran.

The host owns module lifetime. Module callbacks run on the state owner's serialized execution path and must not wait for SDK futures, perform blocking I/O or start unregistered background loops. Stop releases module state; destructors do not invent a disarm/Hold action. A profile-defined shutdown operation is coordinated separately by the host before its deadline/backend close.

## 2. Read views and event records

```text
ReadOnlyContext:
  vehicle_binding, connection_generation
  immutable_observation_snapshot
  current_navigation_permission, control_generation
  operation_status_view, capability_view
  clock_view, configuration_and_profile_revisions

Event:
  local_arrival_id, receipt_mono, ingress_generation
  variant: MoosMailBatch | BackendObservation | RawMavlinkFrame
           | BackendOperationEvent | ConnectionChanged | DeadlineDue

MoosSample:
  key, typed_value, source, source_aux, community
  message_time, message_clock_basis
  receipt_mono, ingress_generation, local_arrival_id
```

The backend event retains target/source evidence and the earliest trusted acquisition/receipt time available. Queue draining must not manufacture a new acquisition time. For MOOS input, `message_time` is the notification timestamp; it is not automatically a sensor sample time or the helm's decision-start time.

Context views are immutable for one callback/processing revision. They are not permission to send later using a stale view. IDs and generation counters are local lifecycle/order tools, not authentication credentials or proof of physical vehicle identity. The initial trusted deployment uses a configured vehicle/producer binding with evidence strength stated explicitly.

## 3. Effects returned to the host

```text
Effects:
  observation_updates: List<ObservationUpdate>
  publications: List<MoosPublication>
  operation_requests: List<OperationRequest>
  helm_input_updates: List<HelmInputUpdate>
  diagnostics: List<DiagnosticEvent>
```

These are proposals staged for host processing, not immediate I/O. The host checks declared ownership, target/session, schema and policy before committing them. Publications are limited to declared keys; a telemetry feature cannot overwrite a helm input or grant navigation permission by publishing a favorable status. Unknown/expired observations remain visible with their metadata rather than being silently upgraded.

`OperationRequest` carries the logical issuer/request ID, destination, operation/arguments, requested result goal and interpretable validity bounds. The coordinator owns duplicate/conflict detection, admission and operation-specific wire correlation. A plugin tick cannot create a new logical user request simply to bypass expiry, uncertainty or duplicate handling. Compatibility with legacy side-effecting booleans needs a separate adapter policy; this sketch does not automatically execute retained booleans.

Observation updates are committed before guards dependent on them are evaluated. Modules do not recursively call each other's callbacks through the store. If another feature needs committed observations, the host supplies the resulting view in a later defined phase/event. This avoids a hidden reentrant dependency graph.

## 4. Stock helm candidate and vehicle adapter

```text
HelmInputSet:
  compatibility_profile, selected_source_binding
  local_cache_revision, current_control_generation
  required_fields: Map<Quantity, ValueWithMoosSampleMetadata>
  latest_helm_state, latched_allstop_state, override_evidence
  coherence = ARRIVAL_GROUPED
  field_expiries_mono, clock_assumption_status, rejection_reasons

VehicleAdapter:
  describe_profile() -> SupportedQuantitiesAndOperations
  evaluate_prerequisites(ReadOnlyContext) -> GuardReport
  plan_navigation(HelmInputSet, ReadOnlyContext) -> NavigationPlan | Rejection
  plan_operation(OperationRequest, ReadOnlyContext) -> OperationPlan | Rejection
  plan_loss_response(LossReason, ReadOnlyContext) -> ResponsePlan | Unspecified
```

`HelmInputSet` is a bridge-owned view of the latest admissible scalar cache, not a producer-atomic intent. Its local revision is never labeled a solver iteration or a new upstream decision sequence. It deliberately does not claim compliance with the stronger generation-verified navigation draft. [Compatibility rules](stock-helm.md).

The adapter describes controlled quantities, units/frames/origins, required observations, facing policy, owned feedback loops, limits and supported FC states. It returns a plan; it does not obtain its own connection or grant itself permission. Required dimensions come from the profile: a Rover-like course/speed mapping need not require altitude, while a particular Copter mapping may require it. Unsupported or unknown required dimensions produce a rejection, not inheritance from an unrelated input set.

A `NavigationPlan` carries target, profile/configuration revision, local control generation, input field-version/provenance references, prerequisite references, effective target quantities and a fixed local deadline no later than the earliest applicable field/permission/profile bound. Publishing or sending it again does not change its identity/deadline. Adaptation with current observations can produce an updated target while the same input remains eligible; that does not renew the helm input's age.

No Rover minimum speed, Copter vertical gain, yaw/course coupling, zero-vector action or fallback mode is selected by this interface. `ResponsePlan | Unspecified` forces the profile to acknowledge a missing response specification rather than defaulting to disarm or Hold.

## 5. Host authorization and backend submission

```text
OperationCoordinator:
  admit(OperationRequest, CurrentState) -> AdmissionAndTransaction
  consume(BackendOperationEvent) -> UpdatedEvidenceAndResults
  cancel_or_settle(RequestReference, Reason) -> Outcome

ControlSupervisor:
  request_navigation_enable(OperatorRequest, CurrentState) -> Pending | Rejected
  evaluate(CurrentState, HelmInputSet) -> PermissionAndEligibility
  inhibit(Reason, RecoveryRule) -> UpdatedControlGeneration

EgressService:
  select_navigation(LatestEligiblePlan)
  submit_operation(AdmittedOperationPlan)
  invalidate(ControlOrConnectionGeneration, Reason)

ProtocolBackend:
  describe_capabilities() -> OperationsAndBoundaryCapabilities
  open(ConfiguredTarget) -> Status
  submit(AuthorizedWork) -> AcceptedLocally | RejectedLocally
  invalidate(Generation) -> CancellationAndResidualReport
  close() -> FinalBoundaryReport
```

Only the host's authorized egress path submits effects. Read-only queries, parameter writes, arm/mode changes and continuous navigation are distinct operation classes; “helm active” is not permission for all of them. The control supervisor evaluates current state independently of a plugin's claimed readiness. An enable request is pending until its fresh profile guards and post-boundary input conditions hold.

If the selected FC/profile needs a preparation stream before external-mode confirmation, acquisition may coordinate that stream and mode request under a separate bounded preparation permission. It is not routine active navigation, does not self-confirm acquisition, and must specify its target, deadlines and possible early FC consumption. This sketch selects no preparation target or sequence; a profile cannot hide that prerequisite behind a guard that requires the mode to be active before any preparation is allowed.

`AuthorizedWork` includes the target/session binding, scope/permission revision, operation or navigation identity, relevant control generation and latest dispatch deadline. Admission and final controllable handoff recheck current bindings and deadline with a defined ordering against invalidation. Workers may not turn a previously authorized copy into an indefinitely reusable permission.

Backend submission must be nonblocking/bounded from the state owner's perspective. A backend may use SDK/I/O workers; completions return as identified events. Its capability description states the final controllable gate, hidden queue/retry behavior, supported invalidation and any known residual bound. `invalidate` does not promise recall of already handed-off bytes or an SDK queue it cannot cancel. If the selected dependency cannot meet a profile's required boundary, that effectful capability remains unavailable or its claim must be explicitly revised.

Backend events retain evidence type: local queue result, SDK result, raw ACK/progress, observed state, or timeout/uncertainty. The coordinator does not collapse these into one success boolean. A late old-generation result may update history without re-enabling navigation or confirming a different current vehicle session.

The subsequent [backend comparison criteria](../../research/2026-10-29-backend-comparison/requirements-and-review.md) refine the evidence this sketch would need: original receive provenance, exact-work deadline/generation association at the write gate, retry-producer shutdown, selective queue invalidation and a scoped settlement/residual report. These are proposed refinements, not an accepted API. Stock SDK/libmavconn convenience returns do not establish them.

The [UDP queue/fence sketch](../../research/2026-10-29-udp-queue-assessment/queue-and-fence-design.md) further distinguishes bounded admission, the latest waiting target, an active operation, FENCE_APPLIED and LOCAL_QUIESCENT from remote reconciliation. A libmavconn bypass or owned UDP attempt can share that host responsibility. The proposed receipt/packing/peer interfaces remain unimplemented and unaccepted.

The [first implementation plan](../../plans/2026-10-29-direct-udp-copter/implementation-plan.md) implements only the private interfaces needed by each observable milestone, when later authorized. These signatures remain design vocabulary, not a requirement to build a generic framework or promise a stable ABI before receive-only telemetry.

## 6. Diagnostic and compatibility output

The host should expose module readiness, target/connection evidence, navigation requested/permitted/inhibited state, selected source, per-field ages and age basis, all-stop/state evidence, coherence grade, current control generation, blocking reasons, send-gap/deadline misses and backend residual capability. Names/encoding remain open.

Legacy NAV projections and AppCast are useful views. They must not label new publication time as a new physical sample, turn an unknown measurement into zero, or claim that an unaware legacy consumer handles retained stale values correctly. The [observation and command drafts](../../contracts/2026-10-28-draft-01/README.md) remain semantic references, with the stock-navigation limitation explicitly identified rather than hidden inside a generic intent type.
