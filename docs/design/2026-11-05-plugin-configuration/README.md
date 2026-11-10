# External-plugin configuration routing — proposal

November 5, 2026. **Architecture only; unimplemented and unqualified.** This note
reviews source G, `4806e87267ddc0bb5a701081cb44ca5d1d952036`, without changing its
ABI 8 interfaces, parser, plugins, registry, missions or qualification records.
The running campaigns retain G. This is a recommendation for a later increment,
not an accepted API or evidence that the example accepts these settings today.

## Recommendation

Use one generic namespaced route to one loaded module, paired with a checked
startup-batch API in that module. Keep defaults, value parsing and cross-field
rules with the feature. Do not start with a host-wide typed configuration schema
or a runtime parameter server. This lets an independently built plugin add its
own settings without another app parser branch, while still rejecting typos.

Namespace syntax and validation are separate requirements. Simply forwarding
`plugin.<module>.<key>` to today's `void configure(key,value)` is insufficient:
the default implementation does nothing, and existing modules ignore keys they
do not recognize. The heartbeat example inherits that no-op method. A bare
forwarder could claim to accept `sample_timeout=20` while the example continues
using its fixed two-second constant. The proposed checked method must default
to rejecting nonempty settings, not silently accepting them.

## What exists at G

- `MAVLink::OnStartUp` reads the ordinary ProcessConfig block, lowercases option
  names, preserves the trimmed right-hand-side value, and rejects unhandled app
  options. Its allowlist collects known plugin settings. It opens the receiver,
  creates Runtime, loads the selected modules and applies those settings before
  attaching operational message dispatch.
- `PluginHost::configure` broadcasts each key/value to every loaded plugin and
  validates composition afterwards. It has no recognized/unknown result.
  Provider code owns current numeric/enum checks. Runtime latches configuration
  failure, clears pending work/ownership and modules, and requires restart.
- The capability/resource/request descriptors are not configuration schemas.
  The example's timeout and publication period are constants, 2 s and 0.25 s.
  Its README explicitly identifies the app allowlist limitation.

These are source findings, not additional runtime tests. Exact source bytes,
Git objects, line spans and the pinned MAVROS comparison are indexed in
`source-manifest.json` and retained in
`primary-sources.zip`.

## Minimal public contract

An illustrative future fragment, added to an otherwise valid receive-only
configuration, would be:

```text
plugins = example_heartbeat_observer
plugin.example_heartbeat_observer.sample_timeout = 2.0
plugin.example_heartbeat_observer.publish_interval = 0.25
```

**G does not accept these new option names.** The existing example fragment is
the current supported form. Omitting the future settings should keep exactly
its existing 2 s / 0.25 s defaults; numeric limits below are proposals to test,
not validated timing guarantees.

1. Reserve `plugin.` as a generic app routing prefix. Parse owner and local key,
   then deliver only that owner's batch. Resolve the actual loaded module ID,
   never a filename guess, prefix match, capability name or first handler.
   A missing/disabled owner, empty component or unknown local key is an error.
2. New routable public module IDs should use lowercase ASCII letters, digits
   and underscores, starting with a letter; local keys follow the same simple
   convention. Require the selected ID and `Plugin::name()` to agree. This fits
   all G bundled/example IDs and the app's existing module-selection syntax.
   Do not case-fold or alias two different loaded modules into one owner.
3. Preserve current MOOS right-hand-side parsing. The host treats the resulting
   value as bounded opaque text; it adds no shell/environment interpolation,
   include files, evaluation or new quote/escape language. The module performs
   strict full-value, finite numeric or enum parsing as appropriate.
4. Gather all namespaced entries before applying any of them. Reject duplicate
   `(owner,key)` entries, even identical duplicates; preserve the original
   option/block for diagnostics. Batch order must not alter the result.
5. Call each module once with its complete local batch, including an empty
   batch for defaults. The module validates a candidate configuration, including
   cross-field rules, and commits only after all entries pass. Return Accepted,
   UnknownKey or Invalid with the offending local key and a useful reason.
   An exception is also a startup failure. Accepted means every supplied entry
   was recognized and valid; partial or warning-only acceptance is unsupported.
6. Configuration is startup-only. Keep operational dispatch and effectful
   Services/transport work disabled until all batches, composition checks and
   host policies succeed. A receiver may be bound as it is today; configuration
   must not send MAVLink or enqueue later effects. On failure, use the existing
   invalidation/queue-clearing path, close the receiver and require restart.
   MOOS diagnostic publication is distinct from MAVLink output.

A conceptual interface sketch is one new Plugin virtual, not a new Services or
Vehicle layout:

```cpp
// Proposed only; no header or ABI number is changed by this note.
ModuleConfigResult configureModule(const std::vector<ModuleSetting>& settings);
// Result: Accepted, UnknownKey, or Invalid; plus local key and reason.
// Default: Accepted for empty settings, UnknownKey for nonempty settings.
```

The example could parse into temporary `sample_timeout` and `publish_interval`
values, propose ranges `[0.25,10]` s and `[0.02,2]` s, and require publication
interval <= sample timeout. An error leaves it unused. Successful startup still
uses original observation acquisition times; configuration never manufactures
new measurement freshness, FC readiness, navigation permission or sensor health.

## Host policy and ownership

The existing host settings remain host settings: transport/peer/target identity,
SITL PID/profile, module selection/path, execution/shadow mode, firmware requests,
MOOS scheduling and control-facet permissions. Reserve their names from local
routing initially. In particular, `plugin.foo.simulation_only=true` cannot
select SITL, and a plugin cannot turn `shadow_mode` off by consuming a string.

After configuring all modules, revalidate the complete composition and every
existing host policy, including one control provider, exclusive resources and
request ownership, stream bounds, streamed-parameter bench compatibility,
Rover facing restrictions and explicit Copter execution eligibility. Run those
checks on the effective configuration irrespective of option spelling; do not
leave a policy enforced only inside one legacy parser branch. Rebind cached
backend pointers only after that validation succeeds.

A configured capability is still not permission to operate it. Authority,
original expiries, cancellation and final-send checks remain host-owned.
External modules are trusted native code in this process, not a security
sandbox. The API and its tests constrain well-behaved modules; they cannot stop
malicious code from using unrelated system APIs. No hot reload, live unloading,
configuration request over MOOS mail or automatic reacquisition is proposed.

## Comparison

| Choice | Benefits | Costs and limits |
| --- | --- | --- |
| Namespaced target + checked module batch **(recommended)** | One app route; exact owner; feature owns defaults/units/cross-field rules; default rejects unknown settings; complex future features do not extend a host DSL | Requires explicit checked API and module-specific validation tests; automatic typed forms/documentation are limited |
| Descriptor-declared typed schema | Host can reject undeclared keys and primitive type/range errors; can generate help/forms and discover defaults | Adds descriptor/ABI types, schema-version/encoding decisions and default serialization; cross-field/context rules still need code; duplicated defaults/validation can drift |
| Namespaced target + existing unchecked ABI 8 callback | Small host-only change, no module rebuild | Cannot establish that a key was consumed; silently wrong configuration is a concrete counterexample above; unsuitable as the final contract |

A later optional descriptive schema can support tooling if generated from the
same module definitions that implement validation. It must not become a second
source of defaults or a declaration that grants authority. A complete schema
language is not needed to solve the present external-plugin limitation.

## MAVROS reference

At pinned MAVROS `2d3f4db1101c7950c8817a9d788f0d11106fb261`,
[`plugin.cpp`](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/src/lib/plugin.cpp)
constructs plugin-local ROS nodes and applies overrides matching their resolved
node identity. [`plugin.hpp`](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/mavros/include/mavros/plugin.hpp)
provides typed declaration/default and per-parameter watcher helpers. That
supports putting configuration ownership with each feature. MOOS keeps one flat
ProcessConfig block here, so MAVMOOS needs an explicit routing convention rather
than ROS nodes or YAML remap machinery. MAVROS's watcher callback does not itself
provide cross-plugin atomic rollback: earlier callbacks may already have run
before a later exception. The proposed startup/discard boundary is our choice;
we do not infer runtime atomicity or need dynamic parameter mutation from MAVROS.

## ABI and migration

Adding the checked virtual changes the Plugin vtable. Plan one deliberate next
development ABI increment (likely 9), rebuild host and every module against the
same headers, retain strict factory mismatch rejection and rerun qualification.
Do not silently extend ABI 8 or promise a stable cross-compiler SDK. The existing
C factory names and Services/Vehicle layouts need not change for this proposal.
Descriptor schema expansion would also change ABI if its returned layout grew.

Keep the current flat built-in keys/defaults and their existing configuration
path initially. Add the generic namespaced route and port the external example;
bundled modules with no local API override continue rejecting nonempty local
batches. There is no automatic stripping of `imu_`/`status_` or inference that an
arbitrary flat option belongs to a plugin. Future built-in namespace aliases
need an explicit per-module migration and duplicate/conflict policy. Once both
spellings address one setting, reject mixed explicit assignments rather than
letting file order choose. Legacy-only duplicate ordering and accepted settings
for an unloaded built-in should retain current behavior during that migration;
warn about ineffective settings before separately considering stricter errors.
New namespaced references to unloaded owners are errors from the start.

The inherited empty-batch implementation must leave existing built-in settings
alone. When porting a built-in, combine its legacy values, supported namespace
aliases and host-provided context before the one candidate build. Do not apply
defaults again after a legacy timeout or mission origin has already been set.
Keep the existing precedence of those inputs explicit in its migration tests.

## Meaningful future acceptance tests

- Build an external module separately against the new copied ABI headers; load
  it with an unchanged generic app parser, apply both sample settings, and
  observe the changed expiry/publication behavior with preserved timestamps.
  Also prove omitted settings preserve the current example defaults.
- Use two modules accepting the same local key and verify each gets only its own
  values. Reject missing owner, selected/reported identity mismatch, ambiguous
  normalization, typo, malformed namespace, duplicate and oversized text.
- Reject NaN/infinity, trailing numeric junk, wrong enum and cross-field failures
  in both entry orders. A later-module failure after an earlier acceptance must
  leave no active callbacks, queued effect, stale backend pointer or permission.
- Prove the default no-settings implementation rejects a nonempty batch, and
  retain mismatched old/new ABI factory controls. A module's own tests must prove
  that Accepted changes its behavior; status text alone is insufficient.
- Exercise reserved host-key injection and all host policy checks through every
  supported spelling, including streamed parameter lists beside control,
  shadow/RX_ONLY zero-output, explicit AUTO facet requirements and source/PID
  guards. No routing shortcut may bypass those checks.
- Retain flat-key/default mission regressions, meaningful loader composition
  controls and fresh normal/sanitizer/native qualification for the changed host.
  Preserve G and all prior failures; do not relabel their evidence as ABI 9.

Open decisions before implementation: final public ID/key length bounds; the
first example setting limits; whether to expose read-only effective-config
introspection; and the later per-built-in alias migration. Recommended defaults
are bounded lowercase IDs/text, startup-only application, plugin-owned validation
and no automatic legacy alias migration. No user decision is needed to finish
this architecture note; no implementation is authorized by the note itself.
