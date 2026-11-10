# External configurable heartbeat observer

This separately built MAVMOOS plugin uses the matching checkout's actual
`Plugin.h`, pinned generated MAVLink headers and real `PluginHost`. It does not
copy an alternate SDK or modify the app parser. One diagnostic MOOS output
shows original observation times, stale cached values, reset boundaries and
unknown source age. It opens no connection, requests no stream and proposes no
command, parameter, mission or navigation effect.

The example is optional, outside the ten standard providers, and requires
explicit startup selection. It supplies no vehicle-health or control authority.
HEARTBEAT custom mode remains an uninterpreted integer.

## Build and test from a matching checkout

Use a fresh build directory, CMake 3.24+, a C++17 compiler and Python 3:

```sh
cmake -G Ninja -S examples/heartbeat_observer -B build-example-observer \
  -DMAVMOOS_SOURCE_ROOT="$PWD" \
  -DMAVMOOS_MAVLINK_ARCHIVE=/absolute/path/to/mavlink-a2d0ecf.tar.gz \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build build-example-observer -j 2
ctest --test-dir build-example-observer --output-on-failure
```

The local archive must have SHA-256
`3fc41a3cd46717b1aa226b958aed0829fbe7b7c9793bc8ffdcca76345a9dfde3`.
Omitting that argument retrieves the same immutable MAVLink revision. No
MOOS-IvP build is needed for this module/loader contract test. For an external
copy of the example, use its directory after `-S` and an absolute matching host
checkout after `-DMAVMOOS_SOURCE_ROOT=`. Do not substitute copied or stale API
headers. Select a Python interpreter with `-DPython3_EXECUTABLE=/path/to/python3`
if CMake needs an explicit choice.

On macOS, a standalone Command Line Tools installation may require these
additional configure arguments:

```sh
-DCMAKE_CXX_COMPILER=/Library/Developer/CommandLineTools/usr/bin/clang++ \
-DCMAKE_OSX_SYSROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk
```

Use another fresh build with `-DMAVMOOS_EXAMPLE_SANITIZERS=ON` for ASan/UBSan.
The module and real-loader/fake-service fixture are both instrumented. With
`-DBUILD_TESTING=OFF`, only the observer module is built; the deliberately wrong
ABI module and contract executable are test fixtures. `example-inputs.json`
records exact API/loader/backend and example-source hashes at configuration.
It is input provenance, not proof of a completed build or a vehicle run.

Rebuild the module against the exact host/toolchain before app loading. The
factory uses the checkout's `plugin_abi` constant, currently development ABI 10.
This is a matching-build C++ interface, not a stable cross-version/compiler SDK.
The contract loads both the real module and an intentionally wrong factory-ABI
variant, checking ABI, null context and duplicate-composition rejection.

## Checked startup configuration

Place the [`observer.moos.fragment`](observer.moos.fragment) settings in the
`ProcessConfig=iMAVLink` block of a normal mission. Use an absolute module
build-directory path and the owned local peer's actual ports. For example:

```text
plugin_directory = /absolute/path/to/build-example-observer/plugins
plugins = example_heartbeat_observer
plugin.example_heartbeat_observer.sample_timeout = 0.5
plugin.example_heartbeat_observer.publish_interval = 0.1
```

| Module-local key | Default | Accepted seconds |
| --- | ---: | --- |
| `sample_timeout` | 2 | inclusive [0.25, 10] |
| `publish_interval` | 0.25 | inclusive [0.02, 2], at most `sample_timeout` |

Values are full finite decimal/scientific numbers, without whitespace, a
leading `+`, hexadecimal notation, units or other suffixes. The host routes
`plugin.<actual_module_id>.<local_key>` only to the selected owner. Unknown or
unloaded owners, malformed names, duplicate keys, reserved host-policy keys and
invalid values fail startup with diagnostics; they grant no permission. The
local `publish_interval` belongs to this example and is distinct from any flat
host publication setting.

The module receives one complete startup batch. Omitted settings take their
defaults. It validates a temporary candidate, recognizes every key, checks both
ranges and the cross-field constraint, then commits only after the whole batch
passes. It returns `Accepted` with empty `key` and `reason` fields. Rejected
`UnknownKey` or `Invalid` results carry the offending key and a useful reason. A failed batch leaves its previous settings intact; the app still
discards a failed startup rather than continuing operationally. Configuration
calls no Services method, publishes nothing and changes no timestamp, source
binding, observation identifier or session. Runtime reconfiguration is not
supported; settings changes require a fresh process.

`EXAMPLE_HEARTBEAT_OBSERVATION` is schema 1 JSON. Additive
`sample_timeout_s` and `publish_interval_s` fields report effective settings.
Its `present`, `fresh`, `effective_at`, `read_at`, `age_s`, `age_basis`,
`observation_id`, `session_epoch` and rejection counters retain their original
meaning. Ordinary publication is driven by host iteration and respects the
configured minimum interval; it is not an OS scheduling guarantee. Reset/fault
invalidation may publish on the next iteration. Expiry changes `fresh` while
retaining the sample's original timestamps and identity.

The fragment selects only this module, so it does not also publish the standard
telemetry provider's NAV values. The app resolves the actual module ID to
`<plugin_directory>/mavmoos_example_heartbeat_observer` plus its platform module
suffix. `MAVMOOS_CAPABILITIES` lists `example.heartbeat.observation` with no control
provider, request owner or stream demand. Inspect `MAVMOOS_STATUS` and the example
output; an absent or stale sample never requests a mode change or control.
For a launchable app workflow, see the matching checkout's
[external observer mission](../../missions/external_observer/README.md).

## Observation and lifecycle rules

The host dispatches already parsed, selected messages with `MessageTime`.
`QUEUE_EMPTY_FLOOR` gives a conservative local acquisition-age upper bound;
`READ_ACQUISITION` is accepted only with equal effective/read times. Neither
establishes remote measurement age or the freshness of FC internal inputs.
Cached publications never renew their original effective/read timestamps.

The first accepted source IDs remain bound until reset. Reset clears the sample,
advances the local session epoch, retains settings and the monotonically
increasing observation counter, and requires a sample strictly after that
boundary. Invalid/backward local clocks clear the sample and latch a fault.
Explicit reset with a valid nondecreasing clock is required for recovery.
Neither the local epoch nor these diagnostics grant host identity or permission.

## What the contract tests establish

The driver uses the actual shared-library loader and checked host startup route,
plus separate direct candidate-validation cases. Fake Services/clock tests cover
configured and default expiry/cadence, exact freshness boundaries, omitted
settings, both numeric parsers, range endpoints, duplicates, unknown keys,
cross-field rejection and no partial commit. Existing foreign-source,
out-of-order, invalid-metadata, stale-provenance, reset, clock-fault and malformed
wire controls remain in place.

Every current mutating Services method is overridden by a throwing guard,
including proof mutation and AUTO execution/abort/finish. A separate intentional
virtual-dispatch self-test exercises all 16 guards, so a silent default method
cannot masquerade as zero effects. The observer's own fake-service contexts
retain zero effect attempts throughout. All Services access is counted and
forbidden during configuration, including otherwise read-only queries.

These checks use no MOOSDB, sockets, simulator or real vehicle. They establish
this consumer/module contract, not the separate app mission, firmware support,
physical transport, sensor quality or field readiness. Plugins remain trusted
native code in one process: Services is an interface, not a security sandbox.
The app's default receive-only sender gate is a separate boundary. Startup-loaded
code stays resident; there is no live unloading or replacement promise.
