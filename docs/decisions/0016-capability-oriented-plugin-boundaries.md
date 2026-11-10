# 0016: Capability-oriented providers and vehicle-control composition

Date: November 4, 2026. Status: accepted user principle; implementation choices under the explicitly authorized review/refactor. Stock pHelmIvP unchanged; no real vehicle control.

The user clarified that related native-FC and helm-driven functionality should have one coherent plugin home, with Loiter only one example of the general principle. The previous telemetry/commands/parameters/helm shared-library partition reflected implementation mechanisms more than complete capabilities.

Select one shared host with independent telemetry and parameter providers, plus one ArduCopter vehicle-control provider containing native command/lifecycle and stock-helm integration components. Reusable helpers remain ordinary library classes. A pure backend owns Copter mode/target semantics; the host retains final authority, queueing, expiry and sending. The provider filename is separate from its exclusive vehicle-control role. Future mission transfer, payloads, sensing and landing assistance have coherent collaborator boundaries, not automatic placement inside one FC catch-all.

ABI 5 adds capability/profile descriptors, exclusive resource/request ownership, independently available observation stream requirements and the backend interface. Actual configured capabilities are published for discovery. Competing providers are rejected even under different names. Metadata is not authorization or evidence of vehicle qualification.

Preserve public MOOS operation grammar and tested behavior during the structural move. Legacy plugin names normalize to one provider while preserving native-only versus helm-only surfaces. A later unified strategy request can group native/Guided/helm variants, but requires separately defined behavior activation/completion and qualification. The refactor does not imply that currently internal Guided recovery is already a public hold command, or that a helm-return workflow exists.

The [design and whole inventory](../design/2026-11-04-capability-plugins/README.md), [refactor plan](../design/2026-11-04-capability-plugins/refactor-plan.md) and three linked reviews record alternatives, source mapping, migration details and remaining work. Later implementation evidence must identify its own source/binary snapshot rather than inheriting November 3 results.
