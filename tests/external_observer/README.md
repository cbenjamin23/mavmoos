# Actual external observer app demonstration

The human mission lives in
[`missions/external_observer`](../../missions/external_observer/README.md).
The tests here are deliberately separate from its ANTLER roster. The subscriber
only registers/fetches MOOS variables; it never posts a request or opens a
MAVLink connection. The peer in the ordinary launcher independently captures
the primary app egress endpoint.

Build the host and [external example](../../examples/heartbeat_observer/README.md)
against matching SDK9. Then independently build the subscriber (or use the same
target in a fresh combined host build):

```sh
cmake -G Ninja -S tests/external_observer -B build-example-mission \
  -DMOOS_IVP_ROOT=/absolute/path/to/built/moos-ivp
cmake --build build-example-mission -j2
ctest --test-dir build-example-mission --output-on-failure
```

On macOS the standalone Command Line Tools compiler/sysroot arguments from the
example guide may be needed. Use the same built stock MOOS-IvP directory
configured in the host CMake cache. The archived qualification uses revision
`023fb54e00692611ecee2cd6f1138d10eec2127e`; the general demo also accepts a
configured source snapshot without Git metadata. It records null Git revisions
and hash-only identity for those sources. The independently built example's input record must match the app's
actual source `Plugin.h` and `PluginHost.cpp`; hashes document selected inputs,
while the real load/publications prove that this artifact loaded.

```sh
python3 -B tests/external_observer/run_demo.py \
  --source-root="$PWD" --build=/absolute/path/to/new-host-build \
  --module=/absolute/path/to/standalone-example/plugins/mavmoos_example_heartbeat_observer.so \
  --moos-root=/absolute/path/to/built/moos-ivp \
  --probe="$PWD/build-example-mission/external_observer_probe" \
  --output="$PWD/build-example-mission/normal-live" --scenario=normal
```

The output directory must be fresh. Ports are allocated locally, passed through
the documented launcher and retained in the receipt. The normal case checks
effective0.5s/0.1s settings and actual publication cadence, fresh→stale without
renewal→reset→new observation, unknown remote age, bound-source/peer rejection,
the observed SDK9 external-only capability catalog, zero authority/send
counters, independently empty primary egress and exact scoped cleanup. This
does not claim exact OS scheduling bounds or coherent remote measurements.

Run fresh outputs with `--scenario=defaults`, `--scenario=invalid_config`, and
`--scenario=wrong_abi --wrong-abi=/absolute/path/to/standalone-example/plugins/example_wrong_abi.so`.
The additional `unknown_key`, `duplicate_key`, `missing_owner`,
`reserved_policy` and `noncanonical` cases exercise the actual app parser,
owner routing and complete batch, rather than substituting loader-only tests.
The defaults case observes2s/0.25s; its sample timeout coincides with host session
expiry, so the configured normal case supplies the distinct stale-before-reset
proof. Invalid startup cases must visibly remain CONFIG_ERROR with zero loaded
modules, zero reads, an app UDP port directly witnessed unbound, no module or
catalog publications, useful AppCast diagnostics, zero primary egress and
clean shutdown. The wrong ABI
artifact is copied to a private canonical loader filename; original artifacts
are never changed. Every live case has an original27s supervisor deadline.
Invalid cases use an eight-second source/capture window; configured/default
positive cases use twelve seconds. The supervisor checks that the exact app
and MOOSDB stay live, rather than inferring roster health from pAntler alone.

`EvidenceTests.py` is inert and rejects altered timestamps, freshness, effective
settings, observed cadence, epoch continuity, loaded ABI/control ownership,
filter counters, process uncertainty and nonzero independent egress. It also
checks strict ports, MOOS substitution injection, generation-only launch
behavior and the standard fixture packet against an independently generated
pinned MAVLink golden frame. The original expected-failure test-development
checkpoint is retained in component evidence separately from final results.

`result.json` retains actual source/host/MOOS revisions, app/module/probe/helper
hashes, before/after identity, unmodified probe/launcher/peer logs and cleanup
uncertainty. This is a local fake-source app demonstration. It adds no native
simulator, vehicle control, hardware route, firmware qualification or field
readiness claim.
