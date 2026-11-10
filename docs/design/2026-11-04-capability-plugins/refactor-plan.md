# Refactor plan and acceptance

November 4, 2026. Baseline `11dc5c3`; implementation authorized by the user's capability-oriented architecture request. Scope: existing providers and behavior, stock pHelmIvP unchanged, simulation-only effects. General new vehicle support and a new Hold/Return command language are outside the behavior-preserving first increment.

## Phase 1: establish provider contracts

Add ABI 5 descriptors and exclusive capability/resource/request checks. Reject missing ABI, malformed metadata and competing vehicle-control providers before admission. Metadata is inspectable through the normal MOOS/app interface and is separate from permission/readiness. Preserve startup-only image residency and ordinary instance cleanup.

Acceptance: loader tests exercise distinct-name collisions, declared request overlap, malformed descriptors, actual composition and destruction of rejected providers. Failed startup configuration must leave no receiver socket or surviving plugin context.

## Phase 2: group the existing control implementation

Move command and helm implementation into internal components under `src/libmavmoos_control`. Export one `mavmoos_arducopter_control` module. Extract the Copter command/mode/navigation predicates and frame builders from Runtime into a pure backend owned by that plugin. Keep authority, queues, original deadlines and final send checks in Runtime.

Acceptance: preserve the existing command and helm unit suites, add pure-backend roundtrip/guard checks and compound-provider dispatch/reset tests. Runtime without a provider must reject navigation/native control while parameter/telemetry-only operation remains usable. A second distinct control provider must be rejected. No one may introduce an independent socket or bypass the owned-SITL endpoint guard.

## Phase 3: migrate composition without changing public operations

Canonical composition uses telemetry, parameters and arducopter_control. Retain the existing `MAVMOOS_COMMAND`, result, NAV-control, stock helm inputs, parameter and observation schemas. Old `commands`/`helm` configuration names normalize to one provider with their original enabled request surfaces: native-only, helm-only, or both. Reject a mixture of canonical and legacy names as ambiguous. Publish actual modules/capabilities rather than retaining the obsolete four-module count.

Default receive-only remains telemetry-only. Default shadow composition follows the old helm-enabled flag without enabling native effects. Simulation composition includes native actions and the configured helm facet. Optional `control_native_actions` and `control_helm` settings describe startup facets, not live mode switches. Capability-based bench checks must continue rejecting legacy full-list streaming whenever vehicle control or another incompatible provider is loaded.

Acceptance: pure module-selection tests; app fixtures for receive-only/shadow/config rollback, legacy selections and canonical discovery; no-output assertions remain. Update current help/config/mission reproduction and current Linux recipes; preserve all dated archives and historical baseline-replay recipes.

## Phase 4: verify composed behavior

Run a fresh packaged build and appropriate software/localhost tests, then supported ASan/UBSan and a clean Linux core build. Frozen app/module/source manifests identify each execution. Validate native scenarios serially using the common owned-SITL lock:

1. Existing full lifecycle with independently verified neutral simulator pilot input during native Loiter, height stability, landing/disarming and scoped cleanup.
2. Parameter-only read/list/restricted write/readback/restore with explicit ground evidence.
3. Concurrent paced inventory during active stock-helm navigation, plus an exact-build repeat if needed to establish reproducibility after packaging changes.
4. Existing interruption/reset recovery and relevant six process-fault scenarios because callback composition/authority dispatch is changing.

Keep failed attempts, source/binary hashes, original deadlines, independent FC observations and cleanup proofs. Fake scheduling equality is not an OS deadline; native behavior applies to the pinned Copter/configuration only. Lifecycle acceptance still requires FC landed state, never altitude alone. New tests must not erase the existing distinction between session-cancelled inventory and completed inventory.

## Later strategy-routing increment

After the structural refactor is validated, a proposed next increment is to specify public selection of native, Guided-target and helm-behavior strategies for Hold/Return and other operations as coherent capabilities. Each strategy needs target/frame/reference, supported configuration, control entry/release, cancellation, completion and operator-input semantics. Unsupported variants must be discoverable as unavailable rather than silently mapped to a different action. This does not yet have accepted delivery scope and is not a prerequisite for preserving the existing working operations.

## Open contracts beyond this refactor

- Choose the next concrete control workflow and its observable success/failure criteria before adding public strategy routing. No automatic native/helm fallback is specified.
- Define target shapes and required helm quantities when adding another vehicle or guidance family. The current `NavigationTarget` and adapter require course, speed and home-relative height; extracting their Copter predicates does not make that input set universal. Position, attitude/rate or terrain-relative control would need explicit typed contracts.
- Extract additional firmware observation/protocol traits when an actual second profile needs them. Raw EKF interpretation, parameter C-cast encoding and fixed initial wire addressing remain current-profile assumptions.
- Define second-provider selection, identity and startup-facet defaults explicitly. Current app composition defaults target ArduCopter; the pure backend seam alone is not a drop-in qualification path for another FC/vehicle.
- Define a non-motion command/effect policy for payloads and other future capabilities. It must preserve shared transport/deadlines without assigning every MAVLink command to the flight-control allowlist or revoking navigation for every payload action.
- Decide alias deprecation, provider-specific configuration namespaces and any future third-party ABI support when external providers become a concrete requirement. Startup selection does not imply live replacement.

## Execution state

Phases 1–4 are complete for the structural scope: final software and ASan/UBSan suites each passed 31/31, Linux core passed 22/22, and native coverage includes the lifecycle, parameter-only bench, three concurrent scenarios, six process faults and the strict bulk repeat. The initial native matrix remains 10/11 because an audit false failure is preserved; after a test-only correction, its fresh affected case and the exact-build repeat passed on unchanged production binaries. This is not a claim of a single final 11/11 invocation, broader vehicle support or hardware qualification. Exact commands, failures, source/artifact identities and layer-specific outcomes are in the execution ledger.
