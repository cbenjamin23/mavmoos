# 0002: Prefer one modular bridge with an internal helm adapter

Discussion date: October 20, 2026. Recorded: October 28, 2026.

Status: provisional preference, not a finalized implementation decision.

## Context

The architecture discussion considered a separate `iMAVControl` process between the helm and MAVLink bridge. It leaned toward one modular bridge, tentatively `iMAVLink`, with an internal helm adapter and explicit control-authority management.

## Current preference

Keep transport, routing, vehicle state, functional modules, adapters, authority, and MOOS integration independently maintainable inside one application. Allow non-helm uses such as telemetry, commands, missions, and parameters.

Functional modules should share protocol handling across autopilots where practical. Handle vehicle/autopilot differences through adapters rather than duplicating every plugin. Dynamic loading remains an option; no loader, ABI, thread model, transport dependency, or final executable name is selected.

## Alternatives and rationale

Two required applications could provide process separation but would need their own coordination contract for intent, authority, and failures. A single monolithic class would simplify neither maintenance nor testing. The proposed modular application aims to centralize the bridge's command gating and connection lifecycle while preserving internal boundaries. These are expected benefits, not measured results.

## Evidence and open questions

The [pinned MAVROS node document](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/docs/nodes.md) is an architectural comparison for routing and plugin hosting. It does not resolve the MOOS-specific authority problem.

Revisit this preference after source traces and analysis of authority transitions, adapter contracts, concurrency, fault isolation, multi-vehicle organization, and the MOOS interface. A later accepted or superseding record should identify the evidence and tradeoffs. No runtime behavior has been demonstrated.

## October 29 review

The [plugin source review](../research/2026-10-29-design-clarifications-and-plugins.md) traces MAVROS's C++ plugin base/factory, XML discovery, shared-library loading, message dispatch and executor registration. The current recommendation is one modular host with an interface defined before a runtime loader; built-in factories selected by configuration are a simple initial option. This retains the provisional preference and accepts no ABI, dynamic loader, thread model or dependency.

The user subsequently requested a concrete sketch with one app, built-in plugins and stock helm compatibility. The [module package](../design/2026-10-29-built-in-modules/README.md) develops that basis. [Decision 0004](0004-use-stock-moos-ivp.md) accepts the no-upstream-changes constraint; the detailed module interfaces, scheduling and library selection remain proposals. Dynamic loading is outside this sketch.

## November 3 implementation decision

[Decision 0010](0010-modular-copter-sitl-workflow.md) accepts one host with startup-loaded feature libraries for the authorized simulated workflow. It supersedes the earlier built-in-only first-slice recommendation; detailed API stability and broader capabilities are not promised. Earlier discussion remains preserved above.
