# Stable PX4 Offboard qualification

The November 6, 2026 Linux ARM64 SITL run passed the existing stock-helm Offboard workflow on PX4 v1.17.0 with a stable-specific simulator setup step. The same retained MAVMOOS app, two loaded modules, stock MOOS binaries, compiled operator probe, and mission templates served both the stable candidate and the pinned v1.18.0-rc1 baseline.

| Native SIH quad case | Result | First and second eastbound legs | Accepted hold prestream targets |
| --- | --- | --- | --- |
| Pinned v1.18.0-rc1 | Pass | 4.021 m and 4.008 m | 13 and 13 |
| v1.17.0 with original immediate acquisition, two attempts | Authority withdrawn on home update before the first motion target | Zero accepted motion targets in both attempts | 23 and 22 before withdrawal |
| v1.17.0 after witnessed home settle | Pass | 4.001 m and 4.005 m | 13 and 13 |

The settled run verified disarmed startup, native airborne Hold, fresh course, speed and height from stock `pHelmIvP`, and zero setpoints before explicit acquisition. Independent Offboard heartbeats preceded both motion legs. Helm silence retired bridge authority in 0.862 seconds and the independent observer saw Hold in 2.841 seconds. Resumed inputs left authority latched. An acquire with Hold reported and Offboard still intended was rejected. Explicit native Hold handback yielded Hold as both reported and intended mode. A fresh acquire then enabled the second leg. Operator release preceded landing and disarm. All five private ports cleared, the input digests matched after teardown, and the container and Linux-domain lease closed.

## Home correction and the fixture

Both immediate attempts reached independent PX4 Offboard and then reported `home_changed` with zero accepted motion targets. The first attempt's independent GCS HOME_POSITION messages showed home altitude go from 489.314 m to 489.398 m, beyond MAVMOOS's 0.05 m reference-change limit. The native log showed larger intermediate corrections. PX4 v1.17.0's commander can reconcile GPS and barometric altitude during the first 120 firmware seconds following detected takeoff. The MAVMOOS reference-change revocation behaved as designed. [Pinned PX4 home update logic](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/commander/HomePosition.cpp#L353-L386) · [Pinned correction bound](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/commander/HomePosition.hpp#L57-L58)

The stable fixture observed independent armed-airborne Hold, closed its first owned probe, then read the simulator's native clock and actual takeoff time through its local CLI. The bounded setup required native Hold and at least 120 elapsed firmware seconds. The home publication timestamp and update count stayed identical across four samples spanning 6.665 wall seconds. The samples covered 120.264 through 124.800 firmware seconds after takeoff. The same bridge and stock helm process IDs and start ticks survived the dwell. A new instance of the exact probe then independently observed fresh Hold/Hold and the bridge confirmed zero accepted setpoints before the original acquire, loss, handback and landing assertions. The fixture kept the original per-step flight limits and added time for the bounded setup.

This qualifies the v1.17.0 SIH quad workflow after the observed home-settle setup. Immediate acquisition after simulated takeoff remains a reproduced fail-closed case. The runtime retains its existing response to a changed home reference in either workflow.

## Pinned inputs

- PX4 stable tag `v1.17.0`, signed tag object `a5eb12d2ab591251faa009f76b2685b8cc64405d`, source commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`. GitHub reports a valid tag signature. The immutable release listed 211 assets and the versioned Docker Hub query returned HTTP 404. The ARM64 package was built locally from that source and its exact build inputs. [PX4 release](https://github.com/PX4/PX4-Autopilot/releases/tag/v1.17.0) · [PX4 1.17 SIH guide](https://docs.px4.io/v1.17/en/sim_sih/)
- Stable build target `px4_sitl_default` with the stock `10040_sihsim_quadx` airframe and stock startup scripts. The baseline package uses the separate `sih` build variant. Stable binary SHA-256 `5855d78ee5b79f4ecddd9b1df0fedd270efade9f82cb60bff447d4f9605cae2a`. Stable package tar SHA-256 `4318d53a7c6f03feb5132ac29d84fed9243b8fcb031201e5d1ee9cc08e66361d`.
- Frozen production revision `4eb4e295acdbfd7f119040da49e4847d6e8dc337` and stock MOOS revision `023fb54e00692611ecee2cd6f1138d10eec2127e`. App SHA-256 `3f34ab99453133d0e0a13de47cf0013a0d939770688c2d0517e4433d443236f0` and stock helm SHA-256 `81f2bf8931a77977b8e0bd5de17bffec7a64a5d37f6cbceea59071f74e679b0e`. The [candidate input pin](../../../tests/px4_offboard/firmware-v1.17.0.json) also binds the exact probe, stock helpers, modules, behavior and simulator startup artifacts.
- Settled fixture source `7b31f8dfc73b84c7af5b2b22497d6e0192ccbb06`. Pinned Linux tools image `sha256:644154814d3e5c162392b229f26bd68e3ad55c44acf863e78ee83e7568cbed85`. Runtime used offline loopback, a read-only root and protected inputs, and no effective capabilities. Each native run verified exact simulator socket ownership and performed scoped teardown.

## Evidence and reproduction

The evidence archive and its SHA-256 index contain the upstream release and tag responses, the staged build recipes and dependency hashes, three setup failures and their cleanup, all three stable native attempts, and the baseline receipt. The setup failures covered Python scratch capacity, a Python 3.12 typing dependency, and the upstream Micro-CDR build's external Git lookup. The completed build used its own volume and a read-only, exact Micro-CDR v2.0.1 input to finish offline. The archive also includes CRC-checked independent GCS home readback and the exact retained artifact and cleanup inventory. The full settled native receipt SHA-256 is `f43060076bed8e5614aff9b4e59f100d337cca7b5f55741d66a46d598378c6a2`. Pinned RC qualification

From the project root, use the retained release work volume and stable package volume for a new candidate run.

```sh
python3 -B tests/px4_offboard/run_stable_candidate.py \
  --px4-package-volume mavmoos-stable-px4-4b2eac609afc45619159fc755701a8a3-package \
  --output build-stable-px4/new-candidate
```

The host can audit the extracted settled receipt without a simulator. The audit checks the recorded Git fixture blobs, package and retained work hashes, independent home and authority proofs, trace hashes, and owned cleanup.

```sh
unzip -q docs/implementation/2026-11-06-stable-px4/evidence/stable-px4-evidence.zip \
  'native/settled-1/*' -d build-stable-px4/evidence
python3 -B tests/px4_offboard/run_stable_candidate.py \
  --audit build-stable-px4/evidence/native/settled-1
```

All 14 release profile tests and the extracted settled evidence audit passed. The stable candidate remains a separate receipt from the RC release profile. The verified package tar is retained locally under `build-stable-firmware/px4-v1.17.0-linux-arm64/` together with the labeled package volume. The temporary owned compile volume and upstream source and wheel staging were released after readback.
