# IMU component and native observation tests

`ImuScenarioDriver.cpp` loads the actual optional DSO against fake Services that
reject every effectful operation. `ImuProviderTests.py --driver DRIVER --plugin
MODULE [--trace FRESH]` checks all four supported wire families, SI conversions,
extension/identity unknowns, bounded cache, clock/reset and original-age behavior.
It has no socket, MOOS Run or native simulator.

`NativeEvidenceTests.py` runs pure/mock acceptance and preflight negative controls.
`WireCodecTests.py` is a separate optional in-memory test using installed
pymavlink: valid frames and exact six-byte native SiK startup bytes, altered
checksums/source/arming, incomplete tails, decoded-field tampering and local read
clock faults. It creates no transport or simulator. Pymavlink is required for the
native runner and this codec test, not the core module/fake-Services test.

The manual native runner interface is:

```text
run_native.py --scenario rx_only --source-root SOURCE --build BUILD
  --ardupilot-root AP --moos-root MOOS --output FRESH
  --firmware-manifest SOURCE_JSON --firmware-attestation ATTESTATION_JSON
  [--probe BUILD/bin/imu_moos_probe]
  [--expected-app-sha256 SHA] [--expected-probe-sha256 SHA]
```

The manifest pair may also come from the existing explicit CI environment.
Conflicting explicit/inherited selections fail. The existing strict verifier
binds exact native source, platform, executable bytes and attestation before any
socket/process/output. Copies are checked before transport startup and after
cleanup. Expected app/probe hashes are additional mismatch guards, never native
identity exceptions. Stock MOOS source must match the manifest dependency pin.

The centrally scheduled fixture runs disarmed Copter, stock MOOSDB and the actual
app with exactly `telemetry,imu`. Native serial0 reaches the app directly and
serial1 reaches a separate passive endpoint. There is no helm or relay, no
MAVLink request, parameter write, RC input, arming, takeoff or flight. Selected
message intervals and serial protocols are preboot fixture files only.

The subscriber runs for 25 seconds. The coordinator pauses only its exact owned
native process after eight seconds, resumes it after at least three seconds,
and retains actual signal/observation times. App processing continues: original
sensor ages become stale, heartbeat loss clears the cache, and a new post-floor
session produces fresh observations after native recovery. The native startup
deadline is 20 seconds, app ownership startup is 10, subscriber supervisor is
35, passive capture is bounded, and the outer CI case timeout is 120 seconds.

The passive worker is the already protected receive-only fixture helper. It
timestamps actual reads and proves the first sender against the exact native
PID/socket inventory before accepting it, rejects changed/foreign endpoints,
and joins before file finalization. The module and app process, native source,
firmware copies, helpers, decoder/interpreter and dependency bytes are protected
before/after. Scoped cleanup must be proven with no remaining process and all
selected ports free; uncertainty is FAIL.

`audit_native.py` replays original MOOS JSON, checks raw-to-SI arithmetic, unknowns,
cache/age/session invariants, exact catalog and actual app/runtime zero-send
counters, and requires fresh →stale →cleared →new-epoch recovery. At least one
available raw/scaled family is required, not all families or a physical sensor
count. `wire_evidence.py` freshly decodes retained passive MAVLink bytes and
compares every decoded message field/order to the JSON trace, excluding only
external original read timestamps. Later noise, invalid CRCs or an incomplete
MAVLink tail fail. DataFlash is retained if the simulator writes it, without a
whole-file completeness or physical measurement claim.

Different channels need not contain the same samples. Passive evidence confirms
the exact native source, disarmed/ground state and at least one corresponding
message family. Each MOOS record's SI conversion is checked against its own raw
fields; no clock alignment or primary/passive sample equivalence is inferred.
Zero TX is bounded to frozen source/configuration plus observed app/runtime
counters. It is not an independent packet capture of the primary egress route.

Default result fields include `firmware_identity/_after`, `copied_firmware`,
source/AP/MOOS revisions, app/probe/native/module hashes, protected before/after
maps and drift, actual catalog/module hashes, `provider_evidence`, passive FC and
byte-replay evidence, capture retirement, `cleanup_proven`, port/process cleanup
and the raw-file hash inventory. A copied manifest, firmware advertisement,
module descriptor or a pure test pass does not qualify a native workflow.

At component handoff the new native fixture is implemented and software-tested,
and its probe is compiled only. The integrator records actual native/SITL results
separately after source/build freeze and the shared native-window allocation.
