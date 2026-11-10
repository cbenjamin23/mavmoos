# Write a bounded command module

This optional example uses the development ABI 10 `Services` interface. It owns one MOOS request variable and submits one idempotent, non-forced disarm to an already disarmed quad on the ground. The existing ArduCopter provider supplies the backend and observation streams. Its ordinary command, helm and AUTO request handlers are disabled in the example mission.

The example submits `MAV_CMD_COMPONENT_ARM_DISARM` (400) through `sendCommand`, with all seven parameters zero. The host owns the queue, MAVLink encoding, exact owned SITL endpoint, final expiry gate and send callback. The pinned Copter handlers accept an ordinary disarm while already disarmed. [ArduPilot command definition](https://software.ardupilot.org/dev/docs/mavlink-arming-and-disarming.html) · [Pinned development handler](https://github.com/ArduPilot/ardupilot/blob/04cf6486e8d42ce138de816e8466244346ed77cb/libraries/GCS_MAVLink/GCS_Common.cpp#L5212-L5236) · [Pinned stable handler](https://github.com/ArduPilot/ardupilot/blob/92b0cd788ec29406f26c6f9c31d5ceedbd1cc538/libraries/GCS_MAVLink/GCS_Common.cpp#L5033-L5036)

## Build an independent consumer

Set `MAVMOOS` to the checkout that built your host and `WORK` to a fresh writable directory.

```sh
mkdir -p "$WORK"
cp -R "$MAVMOOS/examples/disarmed_command" "$WORK/consumer"
cmake -G Ninja -S "$WORK/consumer" -B "$WORK/build" \
  -DMAVMOOS_SOURCE_ROOT="$MAVMOOS"
cmake --build "$WORK/build"
ctest --test-dir "$WORK/build" --output-on-failure
```

Use the host compiler and C++ runtime. On macOS with the Command Line Tools installation, pass `-DCMAKE_CXX_COMPILER=/Library/Developer/CommandLineTools/usr/bin/clang++` and `-DCMAKE_OSX_SYSROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk` if needed. For an offline build supply the matching pinned source archive through `-DMAVMOOS_MAVLINK_ARCHIVE=/absolute/path/to/mavlink-a2d0ecf.tar.gz`. The build validates its checksum. A separate build with `-DMAVMOOS_EXAMPLE_SANITIZERS=ON` runs the same loader and module contracts with ASan and UBSan. The example artifact is `build/plugins/mavmoos_example_disarmed_command.so` on the tested platforms. `disarmed-command-inputs.json` records the exact configure-time source inputs.

Use the independently built DSO as `MODULE` in the mission guide. That guide covers strict generation, a manual MOOS request, the owned simulator benchmark and offline replay. Keep the build's input manifest alongside the DSO. [Example mission](../../missions/external_command/README.md) · [Configuration fragment](command.moos.fragment)

## Request and observe

Post one string to `EXAMPLE_DISARM_REQUEST` from the configured `request_source`. It defaults to `uPokeDB`. The source name selects the intended local publisher. MOOS publisher names provide a routing check and carry no authentication guarantee.

```text
schema=1,id=example-1,action=disarm,timeout=5
schema=1,id=example-2,action=cancel,target=example-1,timeout=1
```

IDs contain 1–32 ASCII letters, digits, underscores or hyphens. Timeouts are finite seconds in [1,5]. The module uses the original MOOS publication time mapped by the app into the host's owner clock. Mail must be at most two seconds old, after the current session boundary and strictly before its original expiry. A checked local `observation_timeout` in [0.5,1.5] seconds defaults to 1.5. Choose a value that accommodates the configured observations. Fresh ArduPilot quad heartbeat and landed state must both show disarmed on ground. The packet expires at the earlier of the request deadline and the admission observations' deadlines. New samples never extend either bound.

Read `EXAMPLE_DISARM_RESULT` for the separate states.

- `REQUESTED` records local admission. `SUBMITTED` records host queue acceptance.
- `SENT` records an accepted local handoff. `ACKNOWLEDGED` records a matching accepted FC ACK read strictly afterward. ACK frames carry no unique transaction ID. [MAVLink command protocol](https://mavlink.io/en/services/command.html)
- `COMPLETED` adds fresh heartbeat and landed observations whose conservative timestamps both follow handoff. The expected state remains disarmed on ground.
- `CANCELLED`, `LOST`, `TIMED_OUT`, `FAILED` and `REJECTED` explain local retirement. A separately identified cancel request reports `CANCEL_ACCEPTED` when its target retires. After submission, cancellation leaves uncertainty until the original deadline and cannot retrieve a datagram already handed off. There are no automatic retries.

One operation can be active. The process remembers up to 64 request IDs and retains capacity to cancel its last active operation. The module accepts a complete startup settings batch and has no live reconfiguration. Requests use the shared host command path. Default RX and shadow admit no MAVLink output. Explicit SITL mode can also send the host's ordinary heartbeat and observation interval requests. This example provides one scoped command authoring reference for local simulation.

The copied consumer passed the actual MOOS/app workflow on pinned ArduCopter 4.6.3. The record includes the request rejection controls, one accepted handoff, the FC acknowledgement, a fresh ground observation and scoped teardown. Standalone native evidence
