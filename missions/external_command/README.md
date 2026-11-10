# External DISARM command bench

This headless mission runs stock `MOOSDB` and one `iMAVLink` under `pAntler`, with timewarp 1 and loopback ports. It loads the built-in ArduCopter backend and the optional [external DISARM example](../../examples/disarmed_command/README.md). The backend's ordinary native command, helm and AUTO request handlers are disabled. The mission defaults to RX mode. Use a fresh work directory for every generated community.

## Generate or launch

Set `BUILD` to your matching MAVMOOS build, `MODULE` to the independently built example DSO, `MOOS` to the built stock MOOS-IvP checkout and `WORK` to a fresh path without spaces. Choose distinct unused ports.

```sh
./launch.sh --build "$BUILD" --module "$MODULE" --moos-root "$MOOS" \
  --work-dir "$WORK" --mport 19131 --bind-port 14631 --peer-port 14630 \
  --just_make --nogui 1
```

Omit `--just_make` and choose a new work directory to run the foreground mission. Press Ctrl-C to retire its owned process tree. Default RX and optional `--mode shadow` exercise the same module without enabling MAVLink output. The launcher stages copies of the matching example and backend in the run directory.

For an explicit simulation-only run, use `--mode sitl --sitl-pid "$SITL_PID"`. Select the exact already-running owned ArduCopter executable process and the local primary UDP endpoint it owns. Configure its UDP client to reach the chosen bridge port before launching the mission. The app verifies that direct endpoint during startup and handoff. Keep the simulator disarmed on the ground. In a second terminal, after the app has observed its heartbeat and on-ground state, request the one action.

```sh
"$MOOS/bin/uPokeDB" "$WORK/targ_external_command.moos" \
  'EXAMPLE_DISARM_REQUEST=schema=1,id=example-1,action=disarm,timeout=5'
```

Read the latest result retained by the database. Query again if it still shows an intermediate status.

```sh
cd "$WORK"
"$MOOS/bin/uQueryDB" targ_external_command.moos \
  '--condition=EXAMPLE_DISARM_RESULT != uninitialized' \
  --check_var=EXAMPLE_DISARM_RESULT --wait=5 --esv
cat .checkvars
```

Use a fresh ID for another explicit request. The [example contract](../../examples/disarmed_command/README.md#request-and-observe) defines the intermediate results and cancel request.

## Reproduce the owned benchmark

Set `MAVMOOS` to the clean selected source, `AP` to an exact locally built ArduPilot checkout, `MANIFEST` and `ATTESTATION` to its selected profile and native attestation, and `SITL_PYTHON` to Python with pymavlink. Set `OUT` to a new output directory. The integrated build supplies `bin/disarm_sitl_probe`. The example DSO can come from the independent consumer's build. Its adjacent `disarmed-command-inputs.json` is validated against the selected source, or it can be named through `--module-inputs`.

```sh
"$SITL_PYTHON" -B "$MAVMOOS/tests/external_command/run_native.py" \
  --source-root "$MAVMOOS" --build "$BUILD" --moos-root "$MOOS" \
  --ardupilot-root "$AP" --firmware-manifest "$MANIFEST" \
  --firmware-attestation "$ATTESTATION" --probe "$BUILD/bin/disarm_sitl_probe" \
  --module "$MODULE" --output "$OUT"
```

To exercise the separately compiled consumer, set `MODULE="$WORK/build/plugins/mavmoos_example_disarmed_command.so"` using the consumer build directory from the example guide. Use a distinct launch or evidence work directory. The CTest case selects the module from the integrated build instead. Each result records `example_build_kind` and the actual loaded module hashes.

The runner reserves the common native test lock, launches only that attested quad, and uses the same strict target generator and ordinary pAntler roster. The MOOS probe checks wrong source, malformed, expired, inactive cancel and duplicate requests around one explicit successful disarm. It captures the local handoff receipt, accepted primary-link ACK and post-handoff observed result separately. An independent receive-only serial 1 peer records actual FC bytes and verifies disarmed ground and stationary position before and after the probe. The native case and the real Runtime fake-I/O tests record separate evidence for the local native handoff and exact outgoing frame. The latter also exercise cancellation of active work at the final gate. The native result retains owned process, port, artifact and firmware receipts. `--verify-only` checks inputs without launching the bench.

Only the selected owned loopback simulator receives the action. The launcher and runner do not modify the stock MOOS-IvP checkout. This mission does not start a helm, arm, change FC mode or send motion targets.

## Replay closed evidence

Use the same selected source revision as the completed run. The replay uses the retained bytes and checks the indexed hashes, module result sequence, primary ACK and independent FC observations. It runs offline and needs only the standard library.

```sh
python3 -B - "$MAVMOOS" "$OUT" <<'PYREPLAY'
import json
from pathlib import Path
import sys

source, output = map(Path, sys.argv[1:])
sys.path.insert(0, str(source / "tests/external_command"))
from evidence import replay_archived

raw = json.loads((output / "result.json").read_bytes())
proof = replay_archived(raw, lambda name: (output / name).read_bytes())
print(json.dumps(proof, indent=2))
PYREPLAY
```
