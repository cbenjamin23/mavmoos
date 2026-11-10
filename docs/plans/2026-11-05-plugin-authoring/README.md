# Complete the external observer authoring path

November 5, 2026. Baseline `fa8d1902d523334e49251b02ced6f8f0c5d8113f` is preserved. The user requested another parallel push and a completeness pass before reporting a deliverable finished. This increment closes the concrete read-only plugin-authoring gaps identified in the previous review. It does not claim a universal SDK or add vehicle-control policy.

## Bounded deliverable

A developer can separately build the heartbeat observer against matching host headers, configure its own settings without editing the app parser, run the documented receive-only MOOS mission, observe fresh/stale/reset behavior and verify zero MAVLink output. Typos, invalid batches, missing owners and ABI mismatches fail clearly. The source, launcher, tests and documentation provide one complete observer reference. A future command/actuation example, AUTO restart/emergency policy, durable journaling and CI cache/sharding implementation are separate increments.

## Shared configuration contract

Adopt the previously archived checked-startup proposal with one deliberate development ABI increment to9. `ModuleSetting` contains `key,value`; `ModuleConfigStatus` is Accepted/UnknownKey/Invalid; `ModuleConfigResult` contains `status,key,reason`. `Plugin::configureModule(const std::vector<ModuleSetting>&)` accepts an empty batch by default and rejects nonempty settings. The old flat built-in `configure(key,value)` route remains compatible. Host/modules must be rebuilt together.

The app routes `plugin.<actual_module_id>.<local_key>` to exactly that selected module. Batches are complete, startup-only and bounded. Reject unknown/unloaded owners, noncanonical names, duplicate keys, reserved host-policy names and invalid values; configuration cannot queue/send MAVLink or acquire control. Module code owns defaults, strict finite parsing, units and cross-field validation, commits a candidate only after full validation, and reports the offending key. A failed startup invalidates the host and closes operational I/O. No runtime reconfiguration, hot unloading or authority grant is introduced.

The sample exposes `sample_timeout` in[0.25,10] seconds and `publish_interval` in[0.02,2] seconds, with publication interval <= timeout. Omitted settings preserve2s/0.25s defaults. All existing timestamp/source/reset semantics stay intact. Its fake Services must intercept every current state-changing/effectful service, including stored-proof and AUTO services, and reject attempted use.

## Parallel ownership

| Worker | Owns | Completion evidence |
| --- | --- | --- |
| Example | `examples/heartbeat_observer/` | Actual independently compiled module, configurable behavior/defaults/atomic rejection, all effect guards, positive and negative normal/sanitizer contracts |
| MOOS demonstration | `missions/external_observer/`, `tests/external_observer/` | Human-runnable launch/clean/help; caller-controlled local ports; actual pAntler/MOOSDB/iMAVLink path, observed publications/catalog/configuration and independent zero-output capture; scoped cleanup |
| Independent review/qualification | New `tests/plugin_configuration/`, separate receipts, clean Linux qualification | Adversarial configuration/composition/startup/ABI checks, follow the guide as a consumer, review root integration, closed source-built Linux/software/native evidence after root grants |
| Root | Shared Plugin/PluginHost/Runtime API, app parser/help, shared CMake/registry, current docs, integration/archive | Coherent startup lifecycle and host policies, matching standard modules, combined checks and independent completeness closeout |

Workers use fresh isolated worktrees. Do not modify shared files without a root handoff; do not rebuild any previous frozen build. Native runs remain serial per reserved host/container domain; the previously qualified one-Linux-domain isolation/resource policy remains in force. No native run begins from elapsed time or an assumed grant.

## Definition of complete

1. The checked configuration route works through the actual app. Unknown, duplicate, invalid, missing-owner and policy-injection cases fail with useful diagnostics and zero effects. Legacy flat defaults/constraints and default RX_ONLY/shadow remain intact.
2. A fresh standalone consumer build uses the actual matching API/loader, with successful normal/sanitizer contracts and rejected mismatched ABI; the guard detects every current effect-service attempt.
3. The documented mission generates valid targets and runs from its launcher with a normal ProcessConfig/pAntler roster. Non-default ports work. Its live observations demonstrate configured timing, stale provenance and reset without creating freshness, readiness or permission. The independent peer captures no outbound MAVLink. All owned processes and selected ports are clear afterward.
4. A separate reviewer runs the consumer workflow and an explicit completeness matrix. Discovered in-scope omissions are repaired and retested before final reporting. Unimplemented command plugins and future policies are labelled as outside this observer deliverable.
5. Combined normal/sanitizer checks and appropriate current-source Linux/owned-native regressions close with exact source/artifact/profile identities and whole-run cleanup. Root freezes a clean source before effectful qualification and does not move it through whole postflights. Counts come from actual inventories, not this plan.
6. Archive current sources, configuration/failure/lifecycle/wire/cleanup evidence, independent review, preservation and exact result scopes. Update current README, developer guide, interface/help, capability/decision records and historical links. Keep every earlier failure/archive/frozen build unchanged; commit locally with a clean worktree and no push.

Stock MOOS-IvP/pHelmIvP remains unchanged. No real vehicle, physical transport, general parameter writes, RC/servo actuation, hosted activation or external publication is authorized by this increment.
