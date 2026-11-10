#!/usr/bin/env python3
"""Build a pinned headless MAVMOOS workspace, optionally with owned Copter SITL."""

import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tarfile
import tempfile


ROOT = Path(__file__).resolve().parents[1]
PINS = ROOT / "config/setup.json"
COPTER = ROOT / "config/firmware/arducopter-4.6.3-quad.json"
PYTHON_PACKAGES = (
    "empy==3.3.4", "fastcrc==0.3.5", "future==1.0.0", "lxml==6.0.2",
    "numpy==2.4.1", "pexpect==4.9.0", "ptyprocess==0.7.0", "pymavlink==2.4.49",
    "setuptools==80.9.0",
)
ALTITUDE_FILES = ("BHV_ConstantAltitude.cpp", "BHV_ConstantAltitude.h")


def check(condition, message):
    if not condition:
        raise ValueError(message)


def git(source, *args):
    return subprocess.check_output(["git", "-C", str(source), *args], text=True).strip()


def write_json(path, value):
    with tempfile.TemporaryDirectory(dir=path.parent) as temporary:
        record = Path(temporary) / path.name
        record.write_text(json.dumps(value, indent=2) + "\n")
        record.replace(path)


def sha256(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


class Setup:
    def __init__(self, args):
        self.args = args
        self.workspace = args.workspace
        self.downloads = self.workspace / "downloads"
        self.downloads.mkdir(exist_ok=True)
        logs = self.workspace / "logs"
        logs.mkdir(exist_ok=True)
        self.logs = Path(tempfile.mkdtemp(prefix="setup-", dir=logs))
        self.env = os.environ.copy()
        self.env["GIT_TERMINAL_PROMPT"] = "0"
        self.cmake_toolchain = ()
        # Match the repository's native build recipe when standalone CLT is installed.
        clt = Path("/Library/Developer/CommandLineTools")
        if sys.platform == "darwin" and (clt / "usr/bin/clang++").is_file():
            self.env.update(CC=str(clt / "usr/bin/clang"), CXX=str(clt / "usr/bin/clang++"),
                            SDKROOT=str(clt / "SDKs/MacOSX.sdk"))
            self.cmake_toolchain = (
                f"-DCMAKE_C_COMPILER={self.env['CC']}",
                f"-DCMAKE_CXX_COMPILER={self.env['CXX']}",
                f"-DCMAKE_OSX_SYSROOT={self.env['SDKROOT']}",
            )

    def run(self, label, *argv):
        logfile = self.logs / f"{label}.log"
        print(f"{label}  {logfile}", flush=True)
        with logfile.open("x") as log:
            result = subprocess.run(list(map(str, argv)), cwd=ROOT, env=self.env,
                                    stdout=log, stderr=subprocess.STDOUT)
        check(result.returncode == 0, f"{label} exited {result.returncode}. See {logfile}")

    def checkout(self, name, repository, revision, *, files=()):
        target = self.workspace / name
        if not target.exists():
            with tempfile.TemporaryDirectory(dir=self.workspace) as temporary:
                source = Path(temporary) / name
                self.run(f"{name}-init", "git", "init", "--quiet", source)
                self.run(f"{name}-remote", "git", "-C", source, "remote", "add", "origin", repository)
                filter_options = []
                if files:
                    self.run(f"{name}-sparse", "git", "-C", source, "sparse-checkout", "set",
                             "--no-cone", "--", *files)
                    if isinstance(repository, str) and repository.startswith("https://"):
                        filter_options = ["--filter=blob:none"]
                self.run(f"{name}-fetch", "git", "-C", source, "fetch", "--quiet",
                         "--no-tags", "--depth=1", *filter_options, "origin", revision)
                self.run(f"{name}-checkout", "git", "-C", source, "checkout",
                         "--quiet", "--detach", revision)
                source.rename(target)
        self.check_checkout(target, revision)
        return target

    @staticmethod
    def check_checkout(source, revision):
        check(git(source, "rev-parse", "HEAD") == revision and
              not git(source, "status", "--porcelain", "--untracked-files=no"),
              f"{source} must contain the unmodified pinned source {revision}")

    def archive(self, key, pin):
        path = self.downloads / pin["name"]
        if not path.exists():
            with tempfile.TemporaryDirectory(dir=self.downloads) as temporary:
                staging = Path(temporary) / path.name
                if self.args.download_cache:
                    original = self.args.download_cache / path.name
                    check(original.is_file(), f"copy the pinned {path.name} into {self.args.download_cache}")
                    shutil.copyfile(original, staging)
                else:
                    self.run(f"{key}-fetch", "curl", "--fail", "--silent", "--show-error",
                             "--location", "--retry", "2", "--max-time", "180",
                             "--proto", "=https", "--proto-redir", "=https",
                             "--output", staging, pin["url"])
                check(sha256(staging) == pin["sha256"], f"checksum mismatch for {path.name}")
                staging.rename(path)
        check(path.is_file() and sha256(path) == pin["sha256"], f"checksum mismatch for {path}")
        return path

    def cmake(self, label, source, build, *options, targets=()):
        self.run(f"{label}-configure", "cmake", "-G", "Ninja", "-S", source, "-B", build,
                 "-DCMAKE_POLICY_VERSION_MINIMUM=3.5", *self.cmake_toolchain, *options)
        command = ["cmake", "--build", build, "--parallel", self.args.jobs]
        if targets:
            command.extend(["--target", *targets])
        self.run(f"{label}-build", *command)

    def build_moos(self, moos, archives):
        cpm = self.workspace / "cpm/cpm"
        cpm.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(archives["cpm-file"], cpm / archives["cpm-file"].name)
        project = self.workspace / "proj-5.2.0"
        geodesy_src = moos / "MOOS/MOOSGeodesy"
        if not project.exists():
            with tempfile.TemporaryDirectory(dir=self.workspace) as temporary:
                with tarfile.open(archives["proj-archive"]) as archive:
                    archive.extractall(temporary, filter="data")
                extracted = Path(temporary) / "PROJ-5.2.0"
                self.run("proj-upstream-build-patch", "cmake",
                         f"-DPATCH_FILE={geodesy_src / 'cmake/proj-5.2.0.patch'}",
                         f"-DPATCH_WORKING_DIRECTORY={extracted}", "-P",
                         geodesy_src / "cmake/ApplyPatch.cmake")
                extracted.rename(project)
        core = moos / "build/MOOS/MOOSCore"
        geodesy = moos / "build/MOOS/MOOSGeodesy"
        core_option = f"-DMOOS_DIR={core}"
        common = ("-DCMAKE_BUILD_TYPE=Release", "-DCMAKE_POSITION_INDEPENDENT_CODE=ON")
        runtime = f"-DCMAKE_RUNTIME_OUTPUT_DIRECTORY={moos / 'bin'}"
        self.cmake("moos-core", moos / "MOOS/MOOSCore", core, *common, runtime,
                   "-DENABLE_EXPORT=ON", "-DUSE_ASYNC_COMMS=ON",
                   "-DTIME_WARP_AGGLOMERATION_CONSTANT=0.4")
        self.cmake("moos-essentials", moos / "MOOS/MOOSEssentials",
                   moos / "build/MOOS/MOOSEssentials", *common, runtime, core_option)
        self.cmake("moos-geodesy", geodesy_src, geodesy, *common, core_option,
                   f"-DCPM_SOURCE_CACHE={cpm.parent}", f"-DCPM_proj_SOURCE={project}",
                   "-DMOOSGEODESY_USE_PROJ=ON", "-DMOOSGEODESY_FETCH_PROJ=on")
        self.cmake("moos-ivp", moos / "ivp/src", moos / "build/ivp", *common, core_option,
                   f"-DMOOSGeodesy_DIR={geodesy}", f"-DIVP_LIB_DIRECTORY={moos / 'lib'}",
                   f"-DIVP_BIN_DIRECTORY={moos / 'bin'}", "-DIVP_BUILD_GUI_CODE=OFF",
                   "-DIVP_BUILD_BOT_CODE_ONLY=OFF", "-DUSE_UTM=ON",
                   targets=("pHelmIvP", "nsplug", "uProcessWatch", "uPokeDB", "uQueryDB", "apputil"))
        for tool in ("MOOSDB", "pAntler", "pHelmIvP", "nsplug", "pLogger", "uProcessWatch"):
            check((moos / "bin" / tool).is_file(), f"headless build is missing {tool}")

    def build_altitude(self, moos, uav):
        source = self.workspace / "altitude"
        source.mkdir(exist_ok=True)
        shutil.copyfile(ROOT / "packaging/linux/altitude-CMakeLists.txt", source / "CMakeLists.txt")
        for name in ALTITUDE_FILES:
            shutil.copyfile(uav / "src/lib_behaviors" / name, source / name)
        build = source / "build"
        self.cmake("altitude", source, build, f"-DMOOS_IVP_ROOT={moos}")
        return build / "lib"

    def build_copter(self):
        profile = json.loads(COPTER.read_text())
        firmware = self.workspace / "arducopter-4.6.3"
        attestation = self.workspace / "copter-native-attestation.json"
        python = self.workspace / "venv/bin/python"
        if not python.is_file():
            self.run("fixture-venv", sys.executable, "-m", "venv", python.parents[1])
        pip = [python, "-m", "pip", "--disable-pip-version-check", "--no-input",
               "--cache-dir", self.downloads / "pip", "install"]
        if self.args.wheelhouse:
            pip += ["--no-index", "--find-links", self.args.wheelhouse]
        self.run("fixture-packages", *pip, *PYTHON_PACKAGES)
        if self.args.ardupilot_source:
            if not firmware.exists():
                self.run("copter-export", sys.executable, "-B", ROOT / "scripts/firmware/export_source.py",
                         "--manifest", COPTER, "--source-root", self.args.ardupilot_source,
                         "--output", firmware)
        else:
            self.checkout(firmware.name, profile["source"]["repository"], profile["source"]["revision"])
            self.run("copter-submodules", "git", "-C", firmware, "submodule", "update",
                     "--init", "--recursive", "--depth=1", "--jobs", self.args.jobs)
        if not attestation.exists():
            out = self.logs / "copter-build"
            self.run("copter-compile", sys.executable, "-B", ROOT / "scripts/firmware/build_native.py",
                     "--manifest", COPTER, "--source", firmware, "--output", out,
                     "--python", python, "--jobs", self.args.jobs)
            write_json(attestation, json.loads((out / "native-attestation.json").read_text()))
        self.run("copter-verify", sys.executable, "-B", ROOT / "scripts/firmware/manifest.py",
                 "verify", "--manifest", COPTER, "--attestation", attestation,
                 "--ardupilot-root", firmware)
        return {"fixture-python": python, "copter-root": firmware, "copter-attestation": attestation}

    def prepare(self, pins, moos_revision):
        moos = self.checkout("moos-ivp", self.args.moos_source or pins["moos"]["repository"], moos_revision)
        uav = self.checkout("moos-ivp-uav-base", self.args.uav_source or pins["uav"]["repository"],
                            pins["uav"]["revision"],
                            files=tuple(f"/src/lib_behaviors/{name}" for name in ALTITUDE_FILES))
        archives = {key: self.archive(key, pin) for key, pin in pins["archives"].items()}
        self.build_moos(moos, archives)
        behavior = self.build_altitude(moos, uav)
        build = self.workspace / "build"
        state = {
            "schema": 1, "source-root": ROOT, "build": build, "moos-root": moos,
            "moos-source": moos, "moos-revision": moos_revision,
            "uav-source": uav, "uav-revision": pins["uav"]["revision"],
            "behavior-dir": behavior, "fixture-python": Path(sys.executable).absolute(),
            "setup-logs": self.logs, **archives,
        }
        app_options = []
        if self.args.native:
            state.update(self.build_copter())
            app_options = [f"-DMAVMOOS_ARDUPILOT_ROOT={state['copter-root']}",
                           f"-DMAVMOOS_SITL_PYTHON={state['fixture-python']}"]
        self.cmake("mavmoos", ROOT, build, "-DCMAKE_BUILD_TYPE=Debug", "-DBUILD_TESTING=ON",
                   f"-DPython3_EXECUTABLE={state['fixture-python']}", f"-DMOOS_IVP_ROOT={moos}",
                   f"-DMAVMOOS_MAVLINK_ARCHIVE={archives['mavlink-archive']}",
                   f"-DMAVMOOS_BEHAVIOR_DIR={behavior}", *app_options)
        self.check_checkout(moos, moos_revision)
        self.check_checkout(uav, pins["uav"]["revision"])
        result = self.workspace / "setup.json"
        write_json(result, {key: str(value) if isinstance(value, Path) else value
                            for key, value in state.items()})
        print(f"Workspace ready  {result}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, required=True, help="private build directory outside this checkout")
    parser.add_argument("--jobs", type=int, choices=range(1, 9), default=4)
    parser.add_argument("--native", choices=("copter",), help="build the pinned ArduCopter 4.6.3 simulator")
    parser.add_argument("--moos-source", type=Path, help="existing Git source for the selected MOOS revision")
    parser.add_argument("--historical-moos", action="store_true", help="use the exact archived source pin from --moos-source")
    parser.add_argument("--uav-source", type=Path, help="existing Git source for the pinned altitude behavior")
    parser.add_argument("--download-cache", type=Path, help="directory containing the three named, pinned downloads")
    parser.add_argument("--ardupilot-source", type=Path, help="existing Git source with pinned Copter submodule objects")
    parser.add_argument("--wheelhouse", type=Path, help="install fixture Python packages from this directory without an index")
    args = parser.parse_args()
    try:
        check(sys.version_info >= (3, 12), "setup requires Python 3.12 or newer")
        check(sys.platform in {"darwin", "linux"}, "setup requires macOS or Linux")
        check(not args.native or (sys.platform == "darwin" and platform.machine() == "arm64"),
              "this native bootstrap targets Apple Silicon macOS")
        check(not args.historical_moos or args.moos_source is not None,
              "--historical-moos requires --moos-source containing the archived pin")
        check(args.native or not (args.ardupilot_source or args.wheelhouse),
              "--ardupilot-source and --wheelhouse require --native copter")
        for tool in ("git", "cmake", "ninja", "patch", "c++", "lsof"):
            check(shutil.which(tool), f"install {tool} before running setup")
        check(args.download_cache or shutil.which("curl"), "install curl or supply --download-cache")
        args.workspace = args.workspace.resolve()
        for name in ("moos_source", "uav_source", "download_cache", "ardupilot_source", "wheelhouse"):
            value = getattr(args, name)
            if value is not None:
                value = value.resolve()
                check(value.is_dir(), f"--{name.replace('_', '-')} requires a directory")
                setattr(args, name, value)
        check(not args.workspace.is_relative_to(ROOT) and not ROOT.is_relative_to(args.workspace),
              "choose a private workspace outside and separate from this checkout")
        pins = json.loads(PINS.read_text())
        revision = pins["moos"]["qualification_revision" if args.historical_moos else "revision"]
        args.workspace.mkdir(mode=0o700, parents=True, exist_ok=True)
        lock = args.workspace / ".setup.lock"
        owner = args.workspace / "setup-owner.json"
        with os.fdopen(os.open(lock, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600), "r+") as handle:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            identity = {"schema": 1, "source-root": str(ROOT), "moos-revision": revision}
            if owner.exists():
                check(json.loads(owner.read_text()) == identity,
                      "this workspace is bound to another checkout or MOOS pin")
            else:
                check(not any(path != lock for path in args.workspace.iterdir()),
                      "choose a fresh empty directory for setup")
                write_json(owner, identity)
            state = args.workspace / "setup.json"
            if state.exists():
                current = json.loads(state.read_text())
                check(args.native or "copter-root" not in current,
                      "rerun this native workspace with --native copter")
            Setup(args).prepare(pins, revision)
    except (OSError, ValueError, subprocess.SubprocessError, tarfile.TarError) as error:
        print(f"Setup failed. {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
