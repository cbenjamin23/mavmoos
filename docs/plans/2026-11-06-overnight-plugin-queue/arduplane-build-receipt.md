# Offline ArduPlane 4.6.3 build receipt

November 6, 2026. The official Plane 4.6.3 source built successfully on Darwin ARM64. The Waf configure step took 1.508 seconds and the two-job Plane build completed 1,347 tasks in 3 minutes 45.140 seconds. This receipt covers compilation and static artifact readback. The conventional Plane flight workflow has its own qualification criteria. [Workflow and offline recipe](arduplane-source-preflight.md)

| Evidence | Recorded value |
| --- | --- |
| Official tag and commit | `Plane-4.6.3` at `3fc7011a7d3dc047cbb17d8bd98ee94577d144c6` |
| Root tree | `2f054de4203be3890f7200d4dfdb34858de04757` |
| Recursive Git inputs | All 20 pinned gitlinks matched the expected inventory |
| Stock `plane.parm` SHA-256 | `93ba9a70c771609a90b81249d6a1d5a9df8d48bef7d149b42b2d9c7fbd06494a` |
| Original files in private export | 23,763 path, type, executable bit and byte identities rechecked after compilation |
| Host | macOS 26.6.2 `25G83`, Darwin 25.6.0 ARM64 |
| Tools | Apple Clang 21.0.0 `clang-2100.1.1.101`, MacOSX26.5.sdk, Python 3.14.7, Empy 3.3.4 |
| Effective commands | `waf configure --board sitl --no-submodule-update` and `waf plane -j 2` |
| Output | `source/build/sitl/bin/arduplane`, arm64 Mach-O executable, 4,357,896 bytes |
| Executable SHA-256 | `b977ff6cd559f6a6c3d16cfb57c1e32840291210eb593e4f7254fea701137fa5` |
| Static image readback | Release banner `ArduPlane V4.6.3`, source stamp `3fc7011a` and native libc++ and libSystem dependencies |

The driver read Git objects over the local file protocol and used the existing Python environment read-only. It compiled from a separate private export and kept temporary and cache paths under that directory. The effective recipe derives from MAVMOOS commit `7ec72c7730d551aadb9ac3133d258e22283fe8b0` and records the worktree-relative interpreter path and disk guards used for this attempt.

## Preserved local evidence

The private run is under `outputs/mavmoos-worktrees/overnight-plane-preflight/build-plane-4.6.3-offline-01` in the Codex workspace. Its `driver.log`, `export.log`, `toolchain.log`, `configure.log`, `build.log`, `build-recipe.sh`, `source-files.json` and `SHA256SUMS` remain beside the binary. The verifier independently read back the tag, tree, version, stock model bytes, executable format, dynamic dependencies and archived file hashes.

| Receipt | SHA-256 |
| --- | --- |
| `build-attestation.json` | `3ddd2117e19fe1238e1d05cb483946a5c9801aea4c467f2fbdcac5d9f6e34a75` |
| `build.log` | `634ccdc3c5915cecfb9d35acd022dc1dcf0f2b875f2c73e9f71b67939e6f3548` |
| `source-files.json` | `d395f5fe3faba2c7d34bb85a390c40af50fcb312a195b7ae686ca571184752b5` |
| `SHA256SUMS` | `cdb2a806592844a5e7a7e53dda7271764d4e9476fb3c15ab18bd337c2f937a32` |

The [repo evidence bundle](../../implementation/2026-11-06-plane-build/README.md) preserves the effective build inputs and receipts for offline readback.
