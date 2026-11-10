# Local SITL release qualification

A release gate is the set of checks required for one clean commit: the software suite, a Linux ARM64 (64-bit Arm) runtime package tested from a different installation directory, and the [simulator matrix](../../scripts/release/matrix.json). Software-in-the-loop (SITL) runs real autopilot firmware against a simulated vehicle. Each matrix entry names the workflow, firmware version and frame. Experimental Ackermann retains its [separate qualification](../../tests/px4_rover/README.md). [Latest release evidence](../implementation/2026-11-10-release-28/README.md)

## Prepare

Build MAVMOOS against the pinned stock MOOS-IvP tree using the [README](../../README.md#build-and-test). Use a Darwin ARM64 host for the pinned ArduPilot artifacts and the isolated Linux ARM64 domain for the kit and PX4. The [kit guide](../../packaging/release/README.md), [Copter launcher](../../missions/copter_release/README.md) and [profile guide](../../tests/release_profiles/README.md) describe the existing offline inputs.

Save the inputs as private local JSON with absolute paths. Use a fresh Linux work volume name and keep the venv interpreter path as it appears in the venv.

```json
{
  "build": "/path/to/this-release-host-build",
  "moos-root": "/path/to/built-stock-moos-ivp",
  "moos-source": "/path/to/pinned-stock-moos-ivp",
  "uav-source": "/path/to/moos-ivp-uav-base",
  "behavior-dir": "/path/to/built-altitude-behavior/lib",
  "fixture-python": "/path/to/sitl-venv/bin/python",
  "cpm-file": "/path/to/CPM_0.43.1.cmake",
  "proj-archive": "/path/to/proj-5.2.0.tar.gz",
  "mavlink-archive": "/path/to/pinned-mavlink.tar.gz",
  "copter-root": "/path/to/pinned-copter-4.6.3",
  "copter-attestation": "/path/to/copter-native-attestation.json",
  "rover-root": "/path/to/pinned-rover",
  "plane-build-root": "/path/to/pinned-plane-4.6.3-build",
  "plane-stable-build-root": "/path/to/pinned-plane-4.7.2-build",
  "stable-sub-root": "/path/to/pinned-sub-4.7.2-source",
  "linux-work-volume": "mavmoos-release-my-candidate",
  "stable-px4-package-volume": "mavmoos-stable-px4-4b2eac609afc45619159fc755701a8a3-package"
}
```

## Run

```sh
python3 -B scripts/release/campaign.py --plan --inputs "$LOCAL_INPUTS"
python3 -B scripts/release/campaign.py --inputs "$LOCAL_INPUTS" --output "$FRESH_CAMPAIGN"
```

The plan prints the exact commit, required cases and local input names. A full run writes `release-result.json` and stops on the first failed stage. It requires 28 SITL scenarios, including the six stable compound stress missions and their independent replays. The baseline PX4 cases use the same build as the relocated Linux kit. The PX4 VTOL stress mission uses its dedicated Linux qualifier and verifies its own build from the same source. Each fixture retains its own firmware, model, raw observations and cleanup receipts. Use the [focused checks](control-qualification.md#running-the-gates) for everyday development.

## On-demand stress tier

Run the compound helm and flight-controller missions separately during development. Use a clean checkout and a fresh output directory.

```sh
python3 -B scripts/release/campaign.py --stress --plan --inputs "$LOCAL_INPUTS"
python3 -B scripts/release/campaign.py --stress --inputs "$LOCAL_INPUTS" --output "$FRESH_STRESS"
python3 -B scripts/release/campaign.py --stress --case stress_copter --inputs "$LOCAL_INPUTS" --output "$FRESH_COPTER"
```

Repeat `--case` to select several missions. The host cases build their selected targets once from the configured Darwin ARM64 build. The PX4 cases use the existing pinned offline Linux qualifiers. Their firmware volume comes from `stable-px4-package-volume`, and `stress-stock-volume` optionally selects their stock dependency volume.

The on-demand run writes `stress-result.json` with case verdicts, checksummed native receipts and independent replay summaries. An experimental Ackermann mission failure is advisory after the runner proves process, port, container and lease cleanup. Setup, scope, integrity or cleanup failures fail the run. Keep raw captures with the local evidence archive.

## Resume and verify

Use `--case <name>` for a separate stage from the same clean commit, including a required stress mission. Keep host native cases serial in one lease domain. Linux PX4 cases use a separate lease. The baseline PX4 cases require the passed kit stage. Give the software suite an exclusive window for its time-sensitive checks.

For a retry, confirm scoped process, port and container cleanup in the original runner evidence. Move the failed stage directory and all of its `supervisor/<case>.*` files into a private attempt directory inside the campaign, then rerun that stage. A different commit needs a fresh campaign.

```sh
python3 -B scripts/release/qualify.py --revision "$RELEASE_COMMIT" \
  --evidence "$CAMPAIGN" --output "$CAMPAIGN/release-replay.json"
```

The verifier reads the matrix from the selected commit. Unstarted cells yield `INCOMPLETE`. Failed, unfinished or mismatched receipts yield `FAIL`. Archive the package, report, supervisors, indexed raw evidence and useful failed attempts together. The package build's verified work volume remains available for exact PX4 reruns.
