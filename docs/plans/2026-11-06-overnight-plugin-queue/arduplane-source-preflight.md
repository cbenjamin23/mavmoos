# Conventional ArduPlane source and build preflight

November 6, 2026. This is the source and offline build preflight for the [whole conventional Plane workflow](arduplane-readiness.md). The proposed simulator cell uses a separately built `arduplane`, the stock `plane` model and the unchanged stock `pHelmIvP`. The preflight checked local source, Git objects and tools. The offline build has its own [successful build receipt](arduplane-build-receipt.md). Native launch and behavioral qualification remain separate.

## Source and model selection

- Use the official `Plane-4.6.3` tag at `3fc7011a7d3dc047cbb17d8bd98ee94577d144c6`. The release page, official SITL version receipt and local tag agree. The locally available `92b0cd788ec29406f26c6f9c31d5ceedbd1cc538` Copter pin is its immediate child. The complete tree diff is one line in `ArduCopter/version.h`. Both contain `ArduPlane V4.6.3` and the official release type. An artifact built from the child still carries the child's source identity. [Plane release](https://github.com/ArduPilot/ardupilot/releases/tag/Plane-4.6.3) · [Official SITL version receipt](https://firmware.ardupilot.org/Plane/stable-4.6.3/sitl/git-version.txt) · [Pinned Plane version header](https://github.com/ArduPilot/ardupilot/blob/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6/ArduPlane/version.h)
- At this pin, the default ArduPlane frame is `plane`, its Waf target is `bin/arduplane` and its default parameter file is `Tools/autotest/models/plane.parm`. That file has SHA-256 `93ba9a70c771609a90b81249d6a1d5a9df8d48bef7d149b42b2d9c7fbd06494a`. It sets `AIRSPEED_CRUISE=22`, `AIRSPEED_MIN=10`, `AIRSPEED_MAX=30`, `ARSPD_USE=1`, `WP_LOITER_RAD=80` and `NAVL1_PERIOD=15`. Verify active values after boot and record the separate fixture defaults overlay. The conventional profile expects `MAV_TYPE_FIXED_WING=1`, `MAV_AUTOPILOT_ARDUPILOTMEGA=3` and FC `1/1`. Require the conventional model and QuadPlane disabled independently of heartbeat type. [Pinned model map](https://github.com/ArduPilot/ardupilot/blob/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6/Tools/autotest/pysim/vehicleinfo.py#L251-L337) · [Pinned default parameters](https://github.com/ArduPilot/ardupilot/blob/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6/Tools/autotest/models/plane.parm) · [Pinned heartbeat and mode definitions](https://github.com/ArduPilot/ardupilot/blob/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6/ArduPlane/GCS_Mavlink.cpp#L8-L27)

## Offline executable build

**Inputs ready.** The local `~/ardupilot` has the exact root and all 20 recursive submodule commit objects, independently walked at the Plane pin. The gitlink inventory exactly matches the `source.submodules` entries in the existing Copter 4.6.3 manifest. The shallow Copter export lacks the Plane commit object, so use the local Git source, with Git's local-file protocol only.

The working stable-build virtual environment still imports Empy 3.3.4 and setuptools 80.9.0 with `pkg_resources` under Python 3.14.7. This host has Apple Clang 21.0.0 and the Command Line Tools SDK link resolves to `MacOSX26.5.sdk`. The direct CLT tools avoid the full Xcode selection and license shim. Keep the virtual environment's logical interpreter path and use it read-only. Old generators invoke `gcc` literally, so create a private alias to CLT Clang as in the archived successful build. Existing firmware build receipt and lessons

The following is a build-only recipe for a fresh, caller-owned path after the active timing gates. Run from the primary MAVMOOS checkout on the recorded Darwin ARM64 host. The Python block first checks the local source objects, then exports them. The existing manifest supplies **only** its verified matching gitlink list here. The export and before/after source hash map bind the local Plane build to a separate input and output.

```bash
set -eu
mavmoos_repo=$(git rev-parse --show-toplevel)
plane_input=~/ardupilot
plane_work="$mavmoos_repo/build-plane-4.6.3-offline-01"
plane_source="$plane_work/source"
plane_py="$mavmoos_repo/../mavmoos-worktrees/cfc-firmware/build-firmware-private/venv-stable/bin/python"
plane_clt=/Library/Developer/CommandLineTools/usr/bin
test -x "$plane_py"
test -x "$plane_clt/clang++"
test ! -e "$plane_work"
mkdir "$plane_work" "$plane_work/toolchain"
export GIT_ALLOW_PROTOCOL=file PYTHONDONTWRITEBYTECODE=1
export PIP_NO_INDEX=1 PIP_DISABLE_PIP_VERSION_CHECK=1 LC_ALL=C

"$plane_py" -B - "$plane_input" "$plane_source" \
  "$mavmoos_repo/config/firmware/arducopter-4.6.3-quad.json" \
  >"$plane_work/export.log" 2>&1 <<'PY'
import io
import json
from pathlib import Path
import subprocess
import sys
import tarfile

source, output, reference = (Path(p).resolve() for p in sys.argv[1:])
revision = "3fc7011a7d3dc047cbb17d8bd98ee94577d144c6"
tag = "Plane-4.6.3"

def git(root, *args):
    return subprocess.check_output(["git", "-C", str(root), *args])

def gitlinks(repository, pin, prefix=Path()):
    for entry in git(repository, "ls-tree", "-rz", pin).split(b"\0"):
        if not entry:
            continue
        header, name = entry.split(b"\t", 1)
        mode, kind, sha = header.decode().split()
        if (mode, kind) != ("160000", "commit"):
            continue
        relative = prefix / name.decode()
        local = source / relative
        if Path(git(local, "rev-parse", "--show-toplevel").decode().strip()) != local:
            raise ValueError(f"Expected a separate submodule repository at {local}")
        git(local, "cat-file", "-e", sha + "^{commit}")
        yield (relative.as_posix(), sha)
        yield from gitlinks(local, sha, relative)

if git(source, "rev-parse", f"refs/tags/{tag}^{{commit}}").decode().strip() != revision:
    raise ValueError("Local official Plane tag differs")
modules = sorted(gitlinks(source, revision))
expected = json.loads(reference.read_text())["source"]["submodules"]
if len(modules) != 20 or modules != sorted((row["path"], row["revision"]) for row in expected):
    raise ValueError("Exact recursive submodule inventory differs")
subprocess.run(
    ["git", "clone", "--no-local", "--depth=1", "--branch", tag, "--no-recurse-submodules",
     source.as_uri(), str(output)],
    check=True,
)
if git(output, "rev-parse", "HEAD").decode().strip() != revision:
    raise ValueError("Exported root differs")
for relative, sha in modules:
    target = output / relative
    target.mkdir(parents=True, exist_ok=True)
    data = git(source / relative, "archive", "--format=tar", sha)
    with tarfile.open(fileobj=io.BytesIO(data)) as archive:
        archive.extractall(target, filter="data")
print(json.dumps({"revision": revision, "release_tag": tag, "submodules": modules}, indent=2))
PY

cat >"$plane_work/snapshot.py" <<'PY'
import hashlib
import json
import os
from pathlib import Path
import stat
import sys

source, record, phase = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3]

def identity(path):
    mode = path.lstat().st_mode
    if stat.S_ISLNK(mode):
        return ["symlink", hashlib.sha256(os.fsencode(os.readlink(path))).hexdigest()]
    if stat.S_ISREG(mode):
        with path.open("rb") as stream:
            digest = hashlib.file_digest(stream, "sha256").hexdigest()
        return ["file", bool(mode & stat.S_IXUSR), digest]
    raise ValueError(f"Unexpected type for original source file {path}")

if phase == "before":
    files = {}
    for path in source.rglob("*"):
        relative = path.relative_to(source)
        if ".git" not in relative.parts and (path.is_symlink() or path.is_file()):
            files[relative.as_posix()] = identity(path)
    record.write_text(json.dumps(files, sort_keys=True) + "\n")
elif phase == "after":
    for relative, original in json.loads(record.read_text()).items():
        if identity(source / relative) != original:
            raise ValueError(f"Original source changed at {relative}")
else:
    raise ValueError("Expected before or after")
PY
"$plane_py" -B "$plane_work/snapshot.py" "$plane_source" "$plane_work/source-files.json" before

printf '%s\n' '#!/bin/sh' 'exec /Library/Developer/CommandLineTools/usr/bin/clang "$@"' \
  >"$plane_work/toolchain/gcc"
chmod 755 "$plane_work/toolchain/gcc"
export CC="$plane_clt/clang" CXX="$plane_clt/clang++"
export SDKROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk
export PATH="$plane_work/toolchain:$(dirname "$plane_py"):$plane_clt:$PATH"
test -d "$SDKROOT"
{
  uname -srm
  sw_vers
  printf '%s\n' "$CC" "$CXX" "$SDKROOT" "$plane_py"
  ls -ld "$SDKROOT"
  "$CXX" --version
  "$plane_py" --version
  "$plane_py" -B -m pip freeze
} >"$plane_work/toolchain.log" 2>&1
(
  cd "$plane_source"
  set -x
  "$plane_py" -B waf configure --board sitl --no-submodule-update
) >"$plane_work/configure.log" 2>&1
(
  cd "$plane_source"
  set -x
  "$plane_py" -B waf plane -j 2
) >"$plane_work/build.log" 2>&1
"$plane_py" -B "$plane_work/snapshot.py" "$plane_source" "$plane_work/source-files.json" after
test "$(git -C "$plane_source" rev-parse HEAD)" = 3fc7011a7d3dc047cbb17d8bd98ee94577d144c6
test -x "$plane_source/build/sitl/bin/arduplane"
test ! -L "$plane_source/build/sitl/bin/arduplane"
file -b "$plane_source/build/sitl/bin/arduplane" >"$plane_work/executable-format.txt"
rg -q '^Mach-O 64-bit executable arm64' "$plane_work/executable-format.txt"
shasum -a 256 "$plane_source/build/sitl/bin/arduplane" "$plane_work"/{*.log,source-files.json,snapshot.py,executable-format.txt,toolchain/gcc} \
  >"$plane_work/SHA256SUMS"
```

Retain the recipe, source inventory, logs, exact SDK/compiler/Python versions and executable digest with the new artifact. Re-run only in a fresh numbered directory and preserve any failure. The helper above has read-only preflight and syntax evidence. Build reproducibility here means reconstructing the exact inputs and commands with a recorded environment.

The current `manifest.py` schema validator and `build_native.py` command bind Copter, and the app's `UdpReceiver::isSitlEndpoint` accepts only the Copter/Rover executable names. Extend those explicitly for `arduplane` / `plane` / fixed-wing `1/1` before opening a production-path SITL cell. Require a clean Plane source export, its own successful build/attestation, exact owned native PID and socket, fresh configured heartbeat and zero egress in RX/shadow. Verify the other profiles still fail closed when fed a Plane identity. [Current manifest restrictions](../../../scripts/firmware/manifest.py) · [Current builder](../../../scripts/firmware/build_native.py) · [Current endpoint guard](../../../src/libmavmoos_rx/UdpReceiver.cpp)

## Stock helm and fixed-wing navigation contract

- Use ordinary course, speed and altitude domains, stock waypoint and altitude behaviors and the existing bound helm source. The stock helm checks every required dimension, publishes course as `DESIRED_HEADING` and emits individual scalar notifications. `IVPHELM_ITER` is separate and `IVPHELM_ALLSTOP` is change-driven. All-stop may post zeros. Keep existing original publication age, source checks, fresh `DRIVE` and latest `clear`, timestamp span and post-acquisition barriers. Label it arrival-grouped. Pass no zero-speed or stale/replayed scalar as motion. The MIT PDF index was unavailable during this preflight. The boundary is verified against local MOOS-IvP `023fb54e00692611ecee2cd6f1138d10eec2127e`, `ivp/src/pHelmIvP/HelmIvP.cpp` lines 505–548, 602 and 1717–1770. The observed stock binary SHA-256 was `5b3d2e705609c37b5b5000384655b4491010d6d797b94c43830d9972e684a876`. [Shared helm adapter](../../../src/libmavmoos_control/HelmAdapter.cpp)
- Interpret `DESIRED_HEADING` as ground course clockwise from true north and `DESIRED_ALTITUDE` as height above the bound FC home. Keep `DESIRED_SPEED` in m/s as a MOOS ground-speed intent. The first cell admits the configured cruise-route setting of 22 m/s and reports explicitly that it selects the tested envelope. The FC's separately verified `AIRSPEED_CRUISE` and TECS govern flight speed. Record requested speed, actual groundspeed and actual sensor airspeed separately. A different desired speed is an envelope rejection for this cell. Plane's Guided slew speed accepts only airspeed between the configured min/max. Plane `DO_CHANGE_SPEED` can set an airspeed target or a minimum groundspeed, which is a separate policy from tracking a ground-speed setpoint. Keep separate slew and speed transactions outside the first route. [Guided slew handlers](https://github.com/ArduPilot/ardupilot/blob/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6/ArduPlane/GCS_Mavlink.cpp#L911-L1017) · [Fixed-wing speed handling](https://github.com/ArduPilot/ardupilot/blob/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6/ArduPlane/commands_logic.cpp#L982-L1025)
- Gate moving authority on fresh FC heartbeat, armed and independently airborne state, valid horizontal and vertical estimator/position evidence, stable home and healthy, actively used airspeed for the configured sensor. VFR HUD can fall back to an AHRS estimate or zero. The pinned native `AIRSPEED` 295 sender reports the enabled sensor index, measured speed and explicit unhealthy/using flags. Both pinned wire definitions have length 12 and CRC extra 234. Require the allowed sensor ID, using flag, clear unhealthy flag, physically bounded measurement and fresh acquisition. Cross-check the simulator's actual track and altitude from the independent receive channel and DataFlash. Treat ages as local bounded evidence. [Plane VFR and landed-state sources](https://github.com/ArduPilot/ardupilot/blob/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6/ArduPlane/GCS_Mavlink.cpp#L277-L297) · [Pinned airspeed sender](https://github.com/ArduPilot/ardupilot/blob/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6/libraries/GCS_MAVLink/GCS_Common.cpp#L2363-L2412)
- In confirmed Guided mode 15, translate the decision into a bounded geographic lookahead point. Use a typed `COMMAND_INT` / `MAV_CMD_DO_REPOSITION` 192 with target `1/1`, `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`, integer latitude/longitude and height in metres. Use `param1=-1` and `param2=param3=param4=0`. This clears `CHANGE_MODE` and uses the fixture's verified positive loiter radius and clockwise direction. The pin checks coordinates and fence, maps home-relative height to AMSL internally, uses the position and optional orbit radius and ignores `param1`. The local/global `SET_POSITION_TARGET_*` paths at this pin handle altitude rather than the Copter-style velocity workflow. Space position updates by a declared minimum interval and minimum material change because each accepted point resets the leg and loiter state. Start with a long, straight leg and a lookahead of multiple qualified orbit radii. Prove the candidate destination, possible intercept and eventual orbit are inside the fixture envelope. [Pinned reposition handler](https://github.com/ArduPilot/ardupilot/blob/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6/ArduPlane/GCS_Mavlink.cpp#L836-L888) · [Pinned alternative setpoint handlers](https://github.com/ArduPilot/ardupilot/blob/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6/ArduPlane/GCS_Mavlink.cpp#L1380-L1439) · [Pinned Guided waypoint state](https://github.com/ArduPilot/ardupilot/blob/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6/ArduPlane/commands.cpp#L71-L102)
- Keep at most one replaceable pending navigation target and at most one position command awaiting bounded outcome. Fence both by original decision age, authority/connection epochs and the final socket check. Generic command retries must never resend a superseded point. An ACK can identify command 192 and its result without uniquely identifying a repeated point. Use fresh `POSITION_TARGET_GLOBAL_INT` after local handoff with matching lat/lon and AMSL target plus physical track and height. The pinned sender reports `next_WP_loc` with global altitude and reserved high mask bits set. This is target-state readback. Verify motion separately. [Pinned target sender](https://github.com/ArduPilot/ardupilot/blob/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6/ArduPlane/GCS_Mavlink.cpp#L243-L274) · [Existing Runtime final handoff](../../../src/libmavmoos_runtime/Runtime.cpp)

## Ownership, handoff and independent loss

Acquire only by explicit new operator intent from fresh, armed, qualified native Loiter mode 12. Issue a bounded `MAV_CMD_DO_SET_MODE` with the custom-mode flag and explicit destination mode, then require fresh FC heartbeat mode evidence and new producer values beyond the acquisition boundary. Use the same typed mode operation for handback and RTL. At this pin the native `NAV_LOITER_UNLIM` and `NAV_RETURN_TO_LAUNCH` handlers return ACCEPTED even if their `set_mode` call fails. Guided entry initially captures the aircraft's current position as a navigation/loiter center. A timeout at entry follows the same bounded handback policy as active navigation. The FC continues navigating to the latest Guided point and orbiting it when the stream stops. An ordinary mode entry resets the Guided slew state. Local target expiry limits what the bridge sends. [Pinned modes](https://github.com/ArduPilot/ardupilot/blob/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6/ArduPlane/mode.h#L33-L48) · [Pinned Guided mode](https://github.com/ArduPilot/ardupilot/blob/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6/ArduPlane/mode_guided.cpp#L4-L27) · [Native mode command handlers](https://github.com/ArduPilot/ardupilot/blob/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6/ArduPlane/GCS_Mavlink.cpp#L1117-L1123) · [Pinned mode entry resets](https://github.com/ArduPilot/ardupilot/blob/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6/ArduPlane/mode.cpp#L53-L62)

| Event | Required behavior in the first SITL cell |
| --- | --- |
| Ordinary short helm handoff | Pause admissions, cancel the queue and leave the last verified, envelope-bounded Guided target under FC control for a bounded window. Resume only with fresh complete inputs and the same uninterrupted FC mode and authority. Require explicit new acquisition after expiry, external mode, loss of evidence or an unconfirmed previous target. |
| Release, sustained producer silence or invalid intent, with fresh eligible Guided state | Fence navigation immediately and issue one bounded native Loiter handback through the shared mode transaction. Obtain new FC mode and independent orbit evidence. The FC centers its orbit at mode entry using its loiter policy. If Loiter cannot be confirmed within the bounded grace, withhold the watchdog heartbeat and let the independently configured FC respond. Keep the bridge lease latched off. |
| Explicit RTL while bridge owns Guided | Fence navigation before requesting RTL mode 11. Confirm the FC mode and actual return toward the declared home orbit. The fixture sets `RTL_AUTOLAND=0`, checks an empty rally/landing plan and verifies `RTL_ALTITUDE` and orbit radius. RTL is a return and orbit in this cell. |
| External pilot or FC mode change | Fence navigation and all queued bridge mode/point transitions. Observe the new pilot or FC mode without overriding it. While the verified route is healthy, continue the configured watchdog keepalive in the independently confirmed external mode. Return to Loiter and explicit fresh acquire are required before the helm can send again. |
| Broken link, bridge death or uncertain Guided restart | Fence navigation and withhold new watched heartbeats. A restarted bridge must see a fresh, independently established native handback or RTL before re-enabling the watchdog keepalive for control. Restore the route and observe quiescence before granting fresh authority. Prove that a residual packet arriving while the FC is outside Guided fails to change mode or target. |

The fixture uses `SYSID_MYGCS=245`, `FS_GCS_ENABL=1`, `FS_LONG_ACTN=1` and `FS_LONG_TIMEOUT=5`. Verify the parameters before acquisition and capture a monitored bridge heartbeat on the owned wire. In pinned source the timer requires a first matching heartbeat, accepts that system ID regardless of component or heartbeat type, and uses FC monotonic time. `FS_GCS_ENABL=3` is AUTO-only. Value 1 covers Guided and native Loiter, with the relevant landing and RC-failsafe conditions in the pinned predicate. Long action 1 explicitly selects RTL from Guided, whereas the default action 0 can continue Guided. Heartbeat recovery clears the failsafe state without restoring the old mode. [Pinned heartbeat receipt](https://github.com/ArduPilot/ardupilot/blob/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6/libraries/GCS_MAVLink/GCS_Common.cpp#L4149-L4156) · [Pinned timeout predicate](https://github.com/ArduPilot/ardupilot/blob/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6/ArduPlane/system.cpp#L353-L396) · [Pinned failsafe actions](https://github.com/ArduPilot/ardupilot/blob/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6/ArduPlane/events.cpp#L171-L215)

The existing Runtime's SITL heartbeat uses `245/191` at about 1 Hz even when FC observations are invalid. Make Plane's monitored heartbeat profile-gated and recheck it at actual handoff, including fresh FC reachability and the applicable lease state. A healthy bidirectional FC link and producer loss are distinct. Allow only the bounded handback grace while Guided without fresh producer permission. A healthy verified pilot-mode takeover leaves the keepalive active, but the FC watchdog itself remains global. A subsequent real bridge loss in FBWA or Manual can force RTL despite valid RC at this pin. That is an explicit fixture policy and must be measured or listed as such. A later pilot-priority link-loss policy needs a separate firmware-owned design and qualification. No other fixture, relay, process or component may transmit system 245 heartbeats. [Runtime heartbeat](../../../src/libmavmoos_runtime/Runtime.cpp) · [Pinned external-mode failsafe behavior](https://github.com/ArduPilot/ardupilot/blob/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6/ArduPlane/events.cpp#L108-L150)

Local cancellation retires queued work. A `DO_REPOSITION` already handed to UDP carries no authority epoch or deadline on the wire. The Guided-only mode gate protects the handback interval while the FC is outside Guided. The direct, private loopback fixture must observe queue quiescence before a new epoch, and test an intentionally delayed prior target. General delayed transport can deliver an old point during a later Guided grant and needs a separate FC-side mechanism or independently qualified delivery bound.

## Native acceptance before claiming the workflow

1. Record the exact attested Plane executable, clean stock helm source/binary, active parameters, home, model and `--speedup 1`. Boot in a fresh private state directory and owned loopback-only route. The simulator owns the direct app peer socket, and a separate read-only observer records mode, target, actual airspeed, groundspeed, horizontal position and relative altitude. Use the pinned model parameter file plus a hashed fixture overlay. Set up a verified airborne, trimmed initial orbit using a separate simulator operator with a different sysid. Plane TAKEOFF mode 13 is a candidate for the fixture prelude. Confirm it and transition into native Loiter before exposing bridge navigation. Label the actual termination or landing evidence explicitly. [Pinned TAKEOFF controller](https://github.com/ArduPilot/ardupilot/blob/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6/ArduPlane/mode_takeoff.cpp#L63-L130)
2. Before native control, freeze numeric acceptance bounds for cruise airspeed and actual groundspeed, altitude, heading error, forward displacement, orbit radius/containment and transition deadlines. The pinned model values above are planning inputs. Test disarmed, wrong profile, stale airspeed, unhealthy sensor, unsafe height, rejected speed intent, duplicate requests and zero-output RX/shadow first. Then use a real stock helm mission to show acquisition, post-grant course/height targets and actual progress and tracking. Exercise bounded normal handoff with full post-pause evidence and absence of stale wire targets.
3. Demonstrate explicit release into native Loiter, orbit containment and explicit RTL with a return trajectory. Exercise pilot takeover with an owned simulated RC switch where the RC input is isolated to the test domain. A separate authorized test sender's mode change proves an external mode transition but carries separate evidence from an actual RC switch. The pilot-mode observation must persist for longer than the watchdog timeout with valid independent evidence. Audit cancellation and late packet behavior across every handoff, including a delayed target racing with the new mode.
4. Exercise stock producer silence while link telemetry continues, then a failed/unconfirmed healthy-link handback, outbound-only link loss, inbound-only link loss, bridge death and restart. Demonstrate FC RTL independently of bridge output using the last observed monitored heartbeat, later FC mode, DataFlash `MODE.Rsn=5` and the actual return path. The asymmetric native cases need direction-selective drops on the owned tuple in an isolated network domain. A transparent UDP relay in place of the directly owned peer changes the qualification boundary. Use fake I/O to cover the decision rules while native isolated fault injection is being prepared.
5. Restore the link and prove no automatic Guided recovery, no queued/replayed point, and explicit reacquisition using a new native Loiter observation and fresh stock-helm output. Archive unsuccessful attempts, raw two-direction wire witnesses, independent FC logs, exact scenario source and the private process/port teardown. Use one scoped native lease and keep earlier artifacts immutable. [Pinned mode reason enumeration](https://github.com/ArduPilot/ardupilot/blob/3fc7011a7d3dc047cbb17d8bd98ee94577d144c6/libraries/AP_Vehicle/ModeReason.h#L15-L30)

The [offline source build](arduplane-build-receipt.md) provides the pinned executable for the next bench. End-to-end readiness still requires the typed Plane manifest and owned-route support, exact airspeed/target observations, the profile-owned watchdog heartbeat, frozen geometric/timing limits and native evidence for each loss case.
