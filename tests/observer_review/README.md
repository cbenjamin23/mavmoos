# Combined observer-host review tests

These tests instantiate the real `Runtime`, load actual telemetry/status/attitude/control/parameter MODULEs and route generated messages through the real callback interfaces. Sending is a synchronous fake callback; no socket, MOOS process, simulator or vehicle is opened. Control cases use explicit Services-level acquisition/target requests, rather than claiming that a stock-helm process ran.

Thirteen groups cover actual Telemetry datagram decoding/coalescing and target/CRC/duplicate filtering; effective/read metadata and original age; independent family expiry; post-reset source binding; stale/foreign/invalid observations; control continuity; parameter actual-read correlation through observer floods; exact target schedule/geometry comparison with healthy, absent and invalid observers; pose loss; fixed normal-resume expiry; and heartbeat-session cancellation. Invalid samples may be newly received and fresh while remaining explicitly invalid; they cannot become a valid pose or grant authority.

The test-only `observer_interval_fixture` requests slower position updates and faster GPS updates than existing providers. Captured fake-I/O MAVLink COMMAND_LONG 511 packets must contain one request per union member using the minimum interval. This deliberately proves the host's request construction/deduplication policy, not actual native uplink delivery, acknowledgments or stream compliance. The fixture is not a shipped capability provider and is never loaded by the app/default/native mission.

Direct Runtime injections assume already-accepted input. The coalescing group additionally feeds real encoded byte frames to Telemetry, proving the actual configured target/CRC/identity gate before Runtime/attitude dispatch. Two different ATTITUDE frames in one datagram share the same acquisition/read stamps; the latest frame must survive without renewing those stamps. Replaying exact bytes is suppressed upstream. This reproduces and catches the original attitude equal-read rejection.

The Python reader validates actual JSON publications, descriptors, frame labels, timestamp/age consistency, absent/invalid outcomes and completed-case coverage. Each recorded variable includes its real fixture publication time: the latest MOOS-style cache may legitimately retain an older publication between throttled updates. A 1 ns comparison allowance covers decimal roundoff. The test requests status/attitude publication intervals of 0.05/0.02 s to expose the deterministic stage changes, without changing FC interval descriptors or source ages. Result artifacts include before/after executable/MODULE hashes and reject drift.

For a private build, supply immutable generated headers to `tests/observer_review/CMakeLists.txt`. The main integrator should link `ObserverHostDriver.cpp` to `mavmoos_runtime` and `mavmoos_rx`, build `IntervalRequestFixture.cpp` as a MODULE against `Plugin.h`/generated headers, and call:

```sh
python3 -B tests/observer_review/ObserverHostTests.py \
  --driver DRIVER --telemetry TELEMETRY_MODULE --status STATUS_MODULE \
  --attitude ATTITUDE_MODULE --control COPTER_CONTROL_MODULE \
  --parameters PARAMETERS_MODULE --rate-fixture INTERVAL_FIXTURE_MODULE \
  --output-dir FRESH_EVIDENCE_DIRECTORY
```

The output directory is optional and, when supplied, must not exist. Omitting attitude requires both `--attitude -` and the explicit `--allow-status-only` flag; that is a partial checkpoint, never full attitude qualification. Root owns normal/sanitizer/Linux wiring and actual native workflows. Development failures and final private evidence are archived in the dated observer-review implementation directory.
