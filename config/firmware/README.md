# Firmware source profiles and native artifact identities

These schema1 profiles declare immutable test inputs. They do not declare broad firmware support or grant control permission. `baseline` retains the existing exact4.7.0-dev source, `candidate` selects the official4.6.3 tag for independent qualification, and `canary` is reserved for explicitly pinned development candidates.4.6.3 is an intentionally selected official release, not a claim that it is the latest upstream stable release.

The current profiles bind ArduPilot Quadrotor1/1, the `quad` model, timewarp1, original source/version/default-parameter witnesses, every recursive submodule revision, generated-MAVLink/MOOS/ordinary-UAV-behavior dependencies and a base fixture. Per-scenario overrides such as controlled preboot wind remain separately declared, bounded and recorded by the scenario registry/runner. They are not inferred from a binary attestation. Rover retains its independent profile/qualification; a Copter result cannot relabel it.

`scripts/firmware/manifest.py` is importable with the standard Python library only:

```python
identity = verify_native(manifest_path, attestation_path, ardupilot_root,
                         expected_profile="arducopter")
```

The result contains `profile_id`, exact file `manifest_sha256`, `attestation_sha256`, `source_revision`, fixed resolved `native_path`, `native_sha256`, host platform, frame, base fixture and `qualification:"not_inferred"`. Validation happens before a runner opens sockets or launches any process. Native executables cannot be overridden through a path/hash option: the profile fixes `build/sitl/bin/arducopter`; validation rejects escaped/symlinked/nonexecutable/changed artifacts, wrong platform/format, source drift and mismatched source/module/version witnesses. Production still proves the live owned executable/PID/profile/loopback socket before sending. These are trusted local test records, not a plugin security sandbox or cryptographically signed build.

Examples from the repository root, with caller-owned paths:

```sh
python3 -B scripts/firmware/manifest.py validate --manifest config/firmware/arducopter-4.6.3-quad.json
python3 -B scripts/firmware/export_source.py --manifest config/firmware/arducopter-4.6.3-quad.json --source-root /path/to/ardupilot --output /path/to/fresh/export
python3 -B scripts/firmware/build_native.py --manifest config/firmware/arducopter-4.6.3-quad.json --source /path/to/fresh/export --output /path/to/fresh/build-evidence --python /path/to/build-venv/bin/python --jobs 4
python3 -B scripts/firmware/manifest.py verify --manifest config/firmware/arducopter-4.6.3-quad.json --attestation /path/to/build-evidence/native-attestation.json --ardupilot-root /path/to/fresh/export
```

The exporter reads existing exact local Git objects without modifying that checkout or its refs. A fresh shallow root clone plus recursive Git archives records every original file/symlink hash, including submodules without `.git` metadata. The resulting `mavmoos-firmware-source.json` is rechecked before/after the build. Original files cannot silently drift; generated products are separate additions. In that export record, `manifest_sha256` is the canonical semantic identity and `manifest_file_sha256` is the exact source manifest file; artifact attestations explicitly carry both identities as `manifest_identity_sha256`/`manifest_sha256` respectively.

Attestation provenance is explicit. `existing_artifact_identity` checks an existing binary alongside a clean pinned Git checkout; it does not reconstruct how that binary was built. `recorded_source_build` additionally embeds the exact successful build receipt, command/log hashes, environment and artifact binding. It is observed local provenance, not proof that an arbitrary compiler is trustworthy, byte-identical reproduction across toolchains, or behavioral vehicle qualification. Full compatibility evidence stays in the capability/test registry and archived scenario results.

The build helper never runs the native executable. On this macOS host it uses the already-working CLT compiler; an owned `gcc` alias handles old generators which invoke that name literally, avoiding a full-Xcode license shim without changing the system or upstream source. Preserve a virtualenv interpreter's logical path: resolving its symlink escapes its package context. The selected4.6.3 source generator needs `pkg_resources`; its isolated build used setuptools80.9.0 instead of83.0.0. Toolchain/package differences are recorded rather than hidden in a firmware-support claim.

## Conventional Plane fixture

The [Plane 4.6.3 manifest](arduplane-4.6.3-plane.json) binds an independent conventional `plane` executable, the local Darwin ARM64 build attestation, stock helm and the active fixed-wing parameters. Run `tests/plane_faults/plane_firmware.py` before a native Plane scenario. The [Plane runner](../../tests/plane_native/README.md) also verifies the active parameter and mission/rally state on the owned simulator. The Plane ledger gives the exact behavioral scope.
