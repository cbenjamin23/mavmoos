# Developing a startup-loaded provider

Start with the optional [external heartbeat observer](../../examples/heartbeat_observer/README.md).
It builds independently of the app, uses the real loader, and owns two checked
startup settings. Follow the [launchable receive-only mission](../../missions/external_observer/README.md)
to see its actual MOOS publications from a local heartbeat peer. For an effectful
provider, use the [standalone disarmed command example](../../examples/disarmed_command/README.md)
and its [owned Copter bench](../../missions/external_command/README.md). Keep a
new provider outside `src/plugins` until its feature boundary and tests justify
adding it to the standard bundle.

The [current provider/reference crosswalk](../design/2026-11-05-provider-reference-crosswalk/README.md)
shows the original provider boundaries, shared host services and concrete
MAVROS/pArduBridge references. Current host/modules use development ABI 10;
dated evidence at earlier ABIs remains historical.
For new ports, record the upstream commit, relevant paths and local behavior
differences in the provider guide. The [build queue's porting procedure](../plans/2026-11-05-autonomous-build-queue/README.md#tracking-mavros-ports)
shows how to review upstream changes against those records.

## Define one coherent capability

A provider owns a recognizable feature family. Native/Guided/stock-helm
strategies for the same vehicle-control capability belong together; mission
storage, parameters and read-only sensors remain separate families. A provider
does not open another FC connection. The host supplies parsed observations,
current vehicle context, publication and common effect services.

Describe capabilities, exclusive resources, request owners and any requested
observation streams in `PluginDescriptor`. Shared read-only subscriptions can
coexist, but effectful requests need one declared owner. A descriptor states
configured availability, not current readiness, permission or qualification.
An observer should not request streams merely to make its example test pass;
the host's bounded stream service owns any real scheduling and effects.

## Use the existing owner-thread callbacks

Keep callbacks bounded. Preserve original `MessageTime` rather than replacing
acquisition time during iteration, forwarding or MOOS publication. Label the
reference frame, units, source identity, unknown values and source-age limits.
Clear or invalidate state on `onReset`, and use `onInputBoundary` when stale
state must be observed before incoming traffic refreshes it. A newly read
cached value is not a newly measured value.

## Own the module's settings

In the app's `ProcessConfig` block, route options to the actual selected module:

```text
plugins = example_heartbeat_observer
plugin.example_heartbeat_observer.sample_timeout = 0.5
plugin.example_heartbeat_observer.publish_interval = 0.1
```

Override `configureModule(const std::vector<ModuleSetting>&)`. Validate the
complete batch into a candidate, then commit it once. Apply defaults for omitted
options; check complete finite values and relationships between fields. Return
`Accepted` with empty diagnostics, or `UnknownKey`/`Invalid` with the offending
key and a useful reason. The inherited implementation accepts only an empty
batch. Every selected module receives one batch, including modules without
settings. Existing built-in flat options are applied first and retain their
startup behavior.

The host rejects invalid or unloaded owners, duplicate/noncanonical keys and
reserved host policy names before calling any module. Module/key names use
lowercase ASCII letters, digits and underscores, beginning with a letter, at
most 96 characters. Values are single-line text, up to 4096 bytes. The host caps
batches at 32 owners, 32 keys per owner, 128 keys and 64 KiB combined text.
Transport, target identity, provider selection, execution mode and control
facets remain host settings. A local `publish_interval` is separate from the
host's flat publication interval.

Keep constructors, configuration callbacks and startup metadata getters free
of effect-service calls. An attempted mutation fails startup even if the
module ignores the refusal.
Read-only queries and MOOS diagnostics remain available. A failed startup
discards the composition and closes the receiver; fix the configuration and
restart. New checked configuration has no live reconfiguration path. See
[decision 0024](../decisions/0024-checked-external-plugin-authoring.md).

## Build a matching module

Include the exact checkout's `src/libmavmoos_runtime/Plugin.h` and its pinned
generated MAVLink headers. Export `mavmoos_create_plugin(unsigned, Services*)`
and `mavmoos_destroy_plugin(Plugin*)`. Reject unsupported `plugin_abi` and null
services before constructing an instance. Keep host and every selected module
on matching source, compiler/standard library and ABI builds. There is no stable
binary SDK promise.

Use a fresh output directory after an ABI, compiler or feature-set change so
old libraries cannot be mistaken for delivered modules. Explicit startup
`plugins` tokens select the `mavmoos_<token>` filename from `plugin_directory`.
The loader rejects duplicate names/capabilities/resources/request owners and
conflicting control providers. Feature code remains resident until exit;
restart the app for a new composition.

## Verify progressively

First exercise the actual module through `PluginHost` with fake Services and
clocks. Check observable behavior: original ages, unknowns, reset, invalid data,
bounded caches, malformed requests and zero effect calls for an observer. Test
the real factory and loader failures. Avoid a fake copy of the Plugin API or a
test that only compares descriptor text. Run normal and sanitizer builds.
When Services gains a mutating method, extend the example's effect guard and
its intentional-call controls before claiming complete observer coverage.

Then integrate the selected module with a matching receive-only app in an owned
local mission and inspect actual MOOS output/catalog/status and zero wire
effects. An effectful provider needs a separately authorized owned SITL
scenario, explicit physical behavior/uncertainty and cleanup assertions. Host
campaigns serialize under the centrally reserved host lock. The
[expansion plan](../plans/2026-11-04-expansion/README.md#qualified-parallel-execution-domains)
also permits one separately leased, resource-bounded isolated Linux domain;
its inner native cases remain serial and its process IDs stay container-local.
Software, app/loopback, host-native, Linux-native and physical evidence are
distinct qualification layers.

The heartbeat reference includes separate consumer builds and actual app tests
for configured/default behavior, invalid settings and ABI rejection. The root
CTest/CI graph includes the matching integrated module and all nine loopback
cases; the separate consumer proof builds its DSO outside that graph. Preserve
the raw publications, independent peer capture, process/port cleanup and exact
source/artifact identities. Apply this sequence and a fresh-consumer review to
each new provider before reporting its declared scope complete.

The command reference separates original expiry, queue admission, local
transport acceptance, a strictly post-handoff matching ACK and a fresh
observation. Its one fixed typed command is permitted only for an already
disarmed, on-ground owned Copter. Use fake I/O to check the exact outgoing frame,
expiry and final cancellation. Use the native bench to check the real app path
and retain the MOOS outcomes and independent FC observations. A local transport
receipt and an independent FC observation are different witnesses.

For operators, follow the example's receive-only configuration fragment first.
Check the catalog's actual modules and request owners separately from live
eligibility. A missing library, mismatched ABI or invalid descriptor is a
startup configuration failure; fix the composition and restart. Do not use
control acquisition or a mode change as a way to diagnose an observer.
