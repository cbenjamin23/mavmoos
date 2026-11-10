# Capability-oriented plugin boundaries

November 4, 2026. Baseline `11dc5c3`. The user asked for a whole-architecture review applying the principle that related functionality, including native-FC and helm-driven alternatives, should have one coherent home. Loiter is an example of that principle, not a request for a special-case Loiter plugin. The user authorized a proposal, archived refactor plan, and implementation/testing where the direction is clear. Stock pHelmIvP remains unchanged; no real vehicle control is authorized.

## Selected direction

Keep one `iMAVLink` host with shared transport, observations, time/expiry, scheduling, authority and final effect checks. Load capability providers at startup. Generic parsing, command tracking, profile predicates and target conversion are ordinary internal components; each helper need not be a loadable plugin.

The current deliverable has three public provider modules:

| Module | Coherent responsibility | Internal implementation |
| --- | --- | --- |
| `telemetry` | Vehicle observations and conventional MOOS navigation projections | Geodesy, frames, validity and publication helpers |
| `parameters` | Parameter read/inventory/restricted-write lifecycle | Encoding traits, paced/streamed strategies, accounting, deadlines and readback |
| `arducopter_control` | ArduCopter native actions and stock-helm navigation integration | Native command transactions, helm input/lifecycle adapter and pure Copter control backend |

`vehicle.control` is the exclusive role/resource. `arducopter_control` is the initial module ID and `arducopter` its descriptor profile, not a container for every MAVLink feature. A future Plane, Rover or PX4 provider would implement the same host seam while retaining its actual vehicle/mode semantics. None is implemented or qualified by this refactor.

The host rejects duplicate capabilities, exclusive resources, request ownership and multiple control backends, including providers with different filenames. Metadata describes supported/configured functions; it does not grant permission, prove readiness or establish hardware qualification. The retained `MAVMOOS_RUNTIME_STATUS`/helm/command status surfaces remain authoritative for those separate questions.

## Whole capability inventory

The [capability review](capability-review.md) supplies the detailed source-backed inventory. The broader boundaries are:

- Vehicle control groups related native and autonomy-driven strategies: navigation, hold/loiter, return and lifecycle operations. Exactly one provider proposes vehicle motion; common host arbitration applies to every strategy.
- Telemetry and system health remain independently usable. Read-only per-device sensor providers may extend observations without loading motion control.
- Parameter access remains independently usable. ArduPilot C-cast encoding and observation conventions are firmware/protocol traits, not dependencies on a helm/controller plugin.
- Mission/fence/rally **data transfer** is an independent protocol capability. Starting or overriding native mission execution must use the existing control/authority path. A public mission workflow can compose these without creating a second motion owner.
- Cameras, gimbals and other payloads group their operations and feedback by meaningful capability. They should not inherit a rule that every payload command revokes vehicle navigation.
- Standard LAND belongs to vehicle control. Optional landing-target sensing/assistance collaborates with that provider; observing a target cannot itself acquire control or start landing.
- File/log transfer is an independently bounded bulk service. Future manual/RC interfaces, time synchronization and routing require explicit ownership/provenance contracts; their implementation helpers are not automatically public plugins.

These are future boundaries, not a claim that those features exist. Discovery advertises only the actual loaded/configured providers and their supported interfaces.

## Existing control choices and explicit gaps

| Functional area | Existing native path | Existing helm/bridge path | This refactor |
| --- | --- | --- | --- |
| Navigation | Explicit Guided-mode request | Acquire/release plus ordinary course/speed/home-relative-height outputs | Both belong to `arducopter_control` |
| Hold/Loiter | `MAVMOOS_COMMAND action=hold` requests native Copter Loiter | Configured `BHV_StationKeep` continues through the helm; conditional Guided position capture is an internal loss/handoff response | One provider/doc/catalog home; preserve distinct authority and pilot-input semantics |
| Return | `action=rtl` requests native RTL | A general explicit helm-return workflow is not currently exposed | Native return is advertised; future strategy routing remains proposed |
| Takeoff/landing/arming | Existing explicit lifecycle operations | Helm navigation can follow takeoff or yield to landing | Lifecycle and helm ownership now compose within one provider |

There is no new `controller=...` Hold/Return request grammar in the first refactor. Adding that grammar without defining mission activation, target/reference, cancellation and completion would silently introduce behavior changes. A later public strategy-routing increment can add it, with the native and helm choices staying together. Helm behaviors continue to run inside stock pHelmIvP through ordinary configuration; no bridge plugin contains a replacement solver or helm patch.

## Implemented seams and limits

- ABI 5 adds bounded provider descriptors, exclusive resource/request claims, observation-stream requirements and a pure `VehicleControlBackend` interface. Rebuild matching host/modules; no stable third-party binary ABI or hot unload is promised.
- The control backend supplies identity compatibility, Copter mode/command predicates, acquisition proposals, input envelopes and target/stop encoding. Its interface receives observations/time and exposes no socket, publication or host-send service; this is an engineering contract, not sandboxing of trusted C++ code.
- The host retains original expiry, queue bounds, private acquisition origin, command leases, nonrenewable handoff permits, clock sampling and the last current-state/send checks. A backend proposal is not permission to transmit.
- Observation stream requirements remain available to telemetry/parameter-only configurations. The initial raw observation codec and parameter profile remain ArduPilot-specific; extracting motion policy does not establish general PX4/Rover support.
- Actual module names/profile/capabilities/request mappings are published as `MAVMOOS_CAPABILITIES`. Internal recovery capture is labelled internal, never a user-requestable Loiter strategy.

Implementation and validation status are recorded in the [refactor plan](refactor-plan.md) and execution ledger. Historical November 3 evidence is not relabelled as validation of ABI 5.

## Reviews

- [Whole-capability boundaries](capability-review.md): coherent public grouping, firmware traits and future collaborators.
- [Runtime/profile extraction](runtime-review.md): baseline Copter assumptions, exact interface and authority/lifetime invariants.
- [Migration and tests](migration-and-tests.md): build/module/config dependencies, compatibility surfaces and acceptance matrix.

The MAVROS reference supplies shared connection/context and feature-hosting precedent; its protocol-oriented plugin list is not a required public partition for MOOS. The primary UAV workflow remains the pinned pArduBridge reference. Exact source references are in the reviews and [reference index](../../references.md).
