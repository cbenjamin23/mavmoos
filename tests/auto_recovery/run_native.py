#!/usr/bin/env python3
"""Restart the bridge while one explicitly owned native Copter remains in AUTO."""
import argparse
import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shlex
import signal
import socket
import subprocess
import sys
import tempfile
import time
import traceback

from evidence import eastward_metres, replay_archived

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tests/mission_execution"))
from run_native import (FIXTURE_EXTRA, INTERVALS, dump, exact_peer, finalize_cleanup,
                        free_port, inventory, revision, sha, source_helpers,
                        stop_owned_processes, wait_rc)
from copter_frame_fixture import (STANDARD_COPTER_FRAMES, bind_copter_frame, copter_default_paths,
                                  one_copter_frame_log)

MODULES = ("telemetry", "parameters", "missions", "arducopter_control")
BOUNDARIES = ("restart.ready", "takeover.request", "handback.ready")
MARKER = "auto-may-continue"


def need(condition, why):
    if not condition:
        raise ValueError(why)


def load_helper(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ValueError("fixture helper unavailable: " + str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def bound_socket():
    stream = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    stream.bind(("127.0.0.1", 0))
    stream.setblocking(False)
    return stream


def discover(process, stream, decoder, end, destination, expected_type):
    while time.monotonic() < end:
        need(process.poll() is None, "owned native exited during endpoint discovery")
        try:
            data, sender = stream.recvfrom(65535)
        except BlockingIOError:
            time.sleep(.005)
            continue
        for message in decoder.parse_buffer(data) or []:
            if (message.get_type() == "HEARTBEAT" and message.get_srcSystem() == 1
                    and message.get_srcComponent() == 1 and message.type == expected_type
                    and message.autopilot == 3):
                proof = inventory(process)
                need(sender[0] == "127.0.0.1" and exact_peer(proof, sender[1], destination),
                     "native endpoint discovery lacks exact owned PID/socket")
                return sender, proof
    raise ValueError("original endpoint discovery deadline expired")


def live_bridge(process, port, seconds=6):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        need(process.poll() is None, "owned bridge exited before its receive socket was live")
        answer = subprocess.run(["lsof", "-nP", "-a", "-p", str(process.pid), "-iUDP"],
                                capture_output=True, text=True, timeout=3)
        if answer.returncode == 0 and re.search(r"UDP\s+127\.0\.0\.1:" + str(port) + r"\b", answer.stdout):
            return answer.stdout
        time.sleep(.02)
    raise ValueError("owned bridge socket not live within original startup deadline")


def live_helm(ownership, antler, mission):
    rows = []
    for record in ownership.owned_records(antler):
        try:
            argv = shlex.split(record["command"])
        except ValueError:
            continue
        if argv and Path(argv[0]).name == "pHelmIvP" and str(mission) in argv:
            rows.append(record)
    need(len(rows) == 1, "exact stock pHelmIvP process unavailable")
    return rows[0]


def write_marker_snapshot(out, state, name):
    marker = state / MARKER
    info = marker.lstat()
    need(marker.is_file() and not marker.is_symlink() and info.st_nlink == 1
         and info.st_uid == os.geteuid() and not info.st_mode & 0o077,
         "uncertainty marker is not the actual private regular entry")
    path = out / (name + ".marker.bin")
    path.write_bytes(marker.read_bytes())
    return {"file": path.name, "sha256": sha(path), "size": path.stat().st_size,
            "device": info.st_dev, "inode": info.st_ino, "mono": time.monotonic()}


def boundary(control, name):
    path = control / name
    if not path.is_file():
        return None
    raw = path.read_bytes()
    need(re.fullmatch((re.escape(name) + r":[A-Za-z0-9_.-]{1,40}\n").encode(), raw),
         "invalid probe boundary token: " + name)
    return raw


def ack(control, name, token):
    target = control / (name + ".ack")
    need(not target.exists(), "duplicate boundary acknowledgement")
    temporary = control / (name + ".ack.tmp")
    with temporary.open("xb") as stream:
        stream.write(token)
    temporary.rename(target)


def run(args):
    os.environ["MAVLINK20"] = "1"
    from pymavlink import mavutil

    source = Path(args.source_root).resolve()
    need(source == ROOT, "runner must come from the selected source")
    frame = args.copter_frame
    expected_type, expected_class = STANDARD_COPTER_FRAMES[frame]
    extended = frame != "quad"
    boundaries = (*BOUNDARIES, "guidance.active") if extended else BOUNDARIES
    if extended:
        need(not subprocess.check_output(["git", "-C", source, "status", "--porcelain"]).strip(),
             "additional frame restart requires a clean selected source")
    ap, moos, build, behavior, out, probe_path, starter_path = [
        Path(getattr(args, key)).resolve() for key in
        ("ardupilot_root", "moos_root", "build", "behavior_dir", "output", "probe", "starter_probe")]
    helpers = source_helpers(source)
    ownership, firmware, catalog = (helpers[key] for key in ("ownership", "firmware", "catalog"))
    identity_helper = load_helper(source / "tests/firmware_identity.py", "_restart_identity")
    selected = identity_helper.load_selected_firmware(
        source, ap, args.firmware_manifest, args.firmware_attestation)
    need(selected is not None, "explicit manifest and native attestation are required")
    manifest_path, attestation_path = Path(selected["manifest_path"]), Path(selected["attestation_path"])
    verified = firmware.verify_native(manifest_path, attestation_path, ap, expected_profile="arducopter")
    manifest = firmware.validate_manifest(manifest_path)
    need(manifest["fixture"]["home"] == [42.358456, -71.087589, 10, 0] and verified["frame"] == "quad",
         "attested Copter binary and standard fixture home required")
    need(not extended or verified["profile_id"] == "arducopter-4.6.3-quad",
         "standard hexa/octa restart requires the pinned Copter 4.6.3 binary")
    source_ref, ap_ref, moos_ref = revision(source), revision(ap), revision(moos)
    need(ap_ref == verified["source_revision"] and moos_ref == manifest["dependencies"]["moos_revision"],
         "native or stock MOOS source differs from selected profile")
    native, app = Path(verified["native_path"]), build / "bin/iMAVLink"
    protected = [native, app, probe_path, starter_path, moos / "bin/pAntler", moos / "bin/pHelmIvP",
                 moos / "bin/MOOSDB", moos / "bin/nsplug", manifest_path, attestation_path]
    protected.extend(build / "plugins" / ("mavmoos_" + name + ".so") for name in MODULES)
    protected.extend(Path(module.__file__) for module in helpers.values())
    protected.extend(source / name for name in (
        "tests/firmware_identity.py", "tests/copter_frame_fixture.py", "tests/mission_execution/run_native.py",
        "tests/mission_execution/MissionExecutionProbe.cpp",
        "tests/observation_load/FinishBoundary.h", "tests/protocol_faults/ProtocolSupport.h",
        "missions/copter_operations/meta_vehicle.moos", "missions/copter_operations/meta_vehicle.bhv"))
    protected.extend(p for p in (source / "tests/auto_recovery").glob("*") if p.is_file())
    protected.extend(p for p in (source / "src").rglob("*") if p.suffix in (".cpp", ".h"))
    protected.extend(ownership.firmware_source_inputs(source))
    protected.extend(p for p in behavior.glob("libBHV_ConstantAltitude.*") if p.is_file())
    defaults = copter_default_paths(ap, frame)
    protected.extend(defaults)
    protected = list(dict.fromkeys(protected))
    before = {str(path): sha(path) for path in protected}
    out.mkdir(parents=True, exist_ok=False)
    control, state = out / "control", out / "recovery-state"
    control.mkdir()
    state.mkdir(mode=0o700)
    (out / "logs").mkdir()
    firmware_inputs = {}
    for role, path in (("manifest", manifest_path), ("attestation", attestation_path)):
        copy = out / ("firmware-" + role + ".json")
        copy.write_bytes(path.read_bytes())
        need(sha(copy) == before[str(path)], "firmware input changed while copying")
        firmware_inputs[role] = {"path": copy.name, "sha256": sha(copy)}
    need(firmware.verify_native(out / "firmware-manifest.json", out / "firmware-attestation.json",
                                ap, expected_profile="arducopter") == verified,
         "copied firmware inputs differ")
    result = {
        "schema": 1, "result": "FAIL", "scenario": "auto_restart_recovery", "profile": "arducopter",
        "scope": "one owned Copter SITL and stock helm across exact bridge process restart",
        "hardware": False, "timewarp": 1, "rc_input_sent": False, "observer_writes": 0,
        "source_ref": source_ref, "ardupilot_ref": ap_ref, "moos_ref": moos_ref,
        "firmware_identity": verified, "firmware_inputs": firmware_inputs,
        "app_sha256": sha(app), "native_sha256": sha(native), "probe_sha256": sha(probe_path),
        "starter_probe_sha256": sha(starter_path), "artifacts_before": before, "events": {},
        "ports": {}, "marker_observations": {}, "processes": {},
        "egress_claim": "source-bound host admission and runtime observations; passive FC receive wire",
        "semantic_helper_sha256": sha(source / "tests/auto_recovery/evidence.py")}
    if extended:
        result.update(frame=frame, frame_identity={"expected_mav_type": expected_type,
                                                   "expected_frame_class": expected_class})
    fixture = {**manifest["fixture"]["parameters"], **FIXTURE_EXTRA, "SERIAL2_PROTOCOL": 2}
    result["fixture_parameters"] = fixture
    parameter_file = out / "fixture.parm"
    parameter_file.write_text("".join(f"{key} {value}\n" for key, value in fixture.items()))
    (out / "message-intervals-chan1.txt").write_text(INTERVALS)
    sockets, processes, logs, capture_streams, packets, positions = [], [], [], [], [], []
    receiver = None

    def launch(role, argv, log, env=None):
        handle = (out / log).open("x")
        logs.append(handle)
        process = subprocess.Popen(argv, cwd=out, env=env, stdout=handle,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        result["processes"][role] = {"pid": process.pid, "argv": argv, "started_mono": time.monotonic()}
        return process

    def record(name):
        result["events"][name] = time.monotonic()

    def newest_mode(mode, after, armed=True):
        now = time.monotonic()
        return any(p["mode"] == mode and p["armed"] is armed and after < p["mono"] <= now
                   and now - p["mono"] <= 2 for p in packets[-12:])

    try:
        for _ in range(3):
            sockets.append(bound_socket())
        bridge, monitor, operator = sockets
        bridge_port, monitor_port, operator_port = [stream.getsockname()[1] for stream in sockets]
        db, base, rc = free_port(socket.SOCK_STREAM), free_port(socket.SOCK_STREAM), free_port(socket.SOCK_DGRAM, "0.0.0.0")
        while db == base:
            base = free_port(socket.SOCK_STREAM)
        result["ports"].update(bridge=bridge_port, monitor=monitor_port, operator=operator_port,
                               db=db, base=base, rc=rc)
        command = [str(native), "--model", frame, "--home", "42.358456,-71.087589,10,0",
                   "--speedup", "1", "--wipe", "--defaults", ",".join(map(str, [*defaults, parameter_file])),
                   "--serial0", f"udpclient:127.0.0.1:{bridge_port}",
                   "--serial1", f"udpclient:127.0.0.1:{monitor_port}",
                   "--serial2", f"udpclient:127.0.0.1:{operator_port}",
                   "--serial5", "none", "--serial6", "none", "--serial7", "none", "--serial8", "none",
                   "--base-port", str(base), "--rc-in-port", str(rc)]
        sitl = launch("sitl", command, "sitl.log")
        result["sitl_pid"] = sitl.pid
        deadline = time.monotonic() + 20

        def decoder():
            value = mavutil.mavlink.MAVLink(None)
            value.robust_parsing = True
            return value

        primary_peer, primary_proof = discover(sitl, bridge, decoder(), deadline, bridge_port, expected_type)
        operator_peer, operator_proof = discover(sitl, operator, decoder(), deadline, operator_port, expected_type)
        rc_proof, rc_at = wait_rc(sitl, rc, deadline)
        result["ports"].update(peer=primary_peer[1], operator_peer=operator_peer[1])
        result["native_primary_socket_inventory"] = primary_proof
        result["operator_socket_inventory_before"] = operator_proof
        result["native_rc_socket_readiness"] = {"inventory": rc_proof, "observed_mono": rc_at,
                                                "original_deadline": deadline, "rc_input_sent": False}
        bridge.close()

        raw = (out / "fc-observer.mavlink").open("xb")
        trace = (out / "fc-heartbeats.jsonl").open("x")
        capture_streams = [raw, trace]
        parse = decoder()
        prefix = b""
        valid = False

        def verify_peer(sender):
            text = inventory(sitl)
            need(sender[0] == "127.0.0.1" and exact_peer(text, sender[1], monitor_port),
                 "passive peer not exact owned native socket")
            result["ports"]["monitor_peer"] = sender[1]
            result["native_observer_socket_inventory"] = text
            return text

        def consume(data, _sender, at):
            nonlocal prefix, valid
            raw.write(data)
            for message in parse.parse_buffer(data) or []:
                if message.get_type() == "BAD_DATA":
                    prefix += bytes(message.data)
                    need(not valid and (b"0 " * 3).startswith(prefix), "invalid passive wire bytes")
                    continue
                need(prefix in (b"", b"0 " * 3) and message.get_srcSystem() == 1
                     and message.get_srcComponent() == 1, "passive wire source/prefix differs")
                valid = True
                if message.get_type() == "HEARTBEAT":
                    need((message.type, message.autopilot) == (expected_type, 3),
                         "passive FC heartbeat identity changed")
                    row = {"mono": at, "mode": message.custom_mode,
                           "armed": bool(message.base_mode & 128), "wire_hex": bytes(message.get_msgbuf()).hex()}
                    packets.append(row)
                    trace.write(json.dumps(row, allow_nan=False) + "\n")
                elif extended and message.get_type() == "GLOBAL_POSITION_INT":
                    positions.append({"mono": at, "lat": message.lat, "lon": message.lon,
                                      "relative_alt": message.relative_alt, "vy": message.vy,
                                      "wire_hex": bytes(message.get_msgbuf()).hex()})

        receiver = helpers["receiver"].PassiveReceiver(monitor, verify_peer, consume, max_datagrams=150000)
        receiver.start()
        env = dict(os.environ)
        env["PATH"] = os.pathsep.join([str(build / "bin"), str(moos / "bin"), env.get("PATH", "")])
        env["IVP_BEHAVIOR_DIRS"] = str(behavior)
        macros = {"MOOS_PORT": db, "APP_PATH": app, "BIND_PORT": bridge_port,
                  "PEER_PORT": primary_peer[1], "LAT_ORIGIN": "42.358456", "LONG_ORIGIN": "-71.087589",
                  "SIMULATION_ONLY": "true", "SHADOW_MODE": "false", "SITL_PID": sitl.pid,
                  "PLUGIN_DIRECTORY": build / "plugins", "BEHAVIOR_DIRECTORY": behavior,
                  "HEIGHT": 5, "SPEED": 1.5, "FACING_POLICY": "course", "FACING_HEADING": 0}
        for suffix in ("moos", "bhv"):
            target = out / ("targ_copter." + suffix)
            template = source / "missions/copter_operations" / ("meta_vehicle." + suffix)
            argv = [str(moos / "bin/nsplug"), str(template), str(target), "--strict", "--force", "-x"]
            argv += [f"{key}={value}" for key, value in macros.items()]
            completed = subprocess.run(argv, capture_output=True, text=True, env=env, timeout=10)
            (out / ("nsplug-" + suffix + ".log")).write_text(completed.stdout + completed.stderr)
            need(completed.returncode == 0, "stock mission generation failed")
        if extended:
            behavior_file = out / "targ_copter.bhv"
            with behavior_file.open("a") as target:
                target.write((source / "tests/auto_recovery/restart_guidance.bhv").read_text())
        mission = out / "targ_copter.moos"
        text = bind_copter_frame(mission.read_text(), frame)
        replacements = [
            (f"  Run = {app} @ NewConsole = false ~ iMAVLink\n", ""),
            ("  plugins = telemetry,parameters,arducopter_control",
             "  plugins = telemetry,parameters,missions,arducopter_control\n"
             "  control_mission_execution = true\n  auto_recovery_directory = " + str(state)),
            ("  AppTick = 30", "  AppTick = 50"), ("  CommsTick = 30", "  CommsTick = 50")]
        for old, new in replacements:
            need(text.count(old) == 1, "stock mission generation anchor changed: " + old)
            text = text.replace(old, new, 1)
        mission.write_text(text)
        result["mission_sha256"] = sha(mission)
        result["behavior_sha256"] = sha(out / "targ_copter.bhv")
        antler = launch("antler", [str(moos / "bin/pAntler"), str(mission)], "pantler.log", env)
        bridge_argv = [str(app), str(mission), "--alias=iMAVLink"]
        bridge_one = launch("bridge_one", bridge_argv, "bridge-one.log", env)
        result["bridge_socket_before"] = live_bridge(bridge_one, bridge_port)
        starter_control = out / "starter-control"
        starter = launch("starter", [str(starter_path), str(db), str(starter_control), "auto_complete_handback"],
                         "starter.jsonl", env)
        probe_args = [str(probe_path), str(db), str(control)]
        if extended:
            probe_args.append("--verify-motion")
        probe = launch("probe", probe_args, "probe.jsonl", env)
        deadline = time.monotonic() + 310
        bridge_two = None
        drain = None
        index = 0
        while time.monotonic() < deadline:
            receiver.check()
            need(sitl.poll() is None and antler.poll() is None, "owned native/stock mission exited")
            if bridge_two is None:
                need(bridge_one.poll() is None and starter.poll() is None, "old bridge or AUTO starter exited before restart")
            else:
                need(bridge_two.poll() is None, "new bridge exited")
            if index < len(boundaries):
                need(probe.poll() is None, "recovery probe exited before the pending boundary")
                name = boundaries[index]
                token = boundary(control, name)
                # The two MOOS subscribers can observe AUTO in either order.
                if (token is not None and name == "restart.ready"
                        and (starter_control / "auto.started").is_file()
                        and newest_mode(3, result["processes"]["bridge_one"]["started_mono"])):
                    record(name)
                    result["owned_helm_before"] = live_helm(ownership, antler, mission)
                    result["marker_observations"]["before_kill"] = write_marker_snapshot(out, state, "before-kill")
                    need(probe.poll() is None, "recovery probe exited before the owned restart")
                    starter.terminate()
                    result["starter_exit"] = starter.wait(timeout=5)
                    need(result["starter_exit"] == -signal.SIGTERM, "starter did not retire at owned AUTO boundary")
                    bridge_one.kill()
                    result["bridge_one_exit"] = bridge_one.wait(timeout=5)
                    need(result["bridge_one_exit"] == -signal.SIGKILL, "old bridge death not observed")
                    record("bridge_killed")
                    result["marker_observations"]["after_kill"] = write_marker_snapshot(out, state, "after-kill")
                    bridge_two = launch("bridge_two", bridge_argv, "bridge-two.log", env)
                    result["bridge_socket_after"] = live_bridge(bridge_two, bridge_port)
                    record("bridge_restarted")
                    result["marker_observations"]["after_restart"] = write_marker_snapshot(out, state, "after-restart")
                    ack(control, name, token)
                    index += 1
                elif token is not None and name == "takeover.request" and newest_mode(3, result["events"]["bridge_restarted"]):
                    record(name)
                    proof = inventory(sitl)
                    need(exact_peer(proof, operator_peer[1], operator_port) and sha(native) == result["native_sha256"],
                         "explicit operator actor lost exact owned native endpoint")
                    result["operator_socket_inventory_at_send"] = proof
                    encoder = mavutil.mavlink.MAVLink(None, srcSystem=255, srcComponent=190)
                    packet = mavutil.mavlink.MAVLink_set_mode_message(1, 1, 4).pack(encoder)
                    (out / "operator-guided.mavlink").write_bytes(packet)
                    need(operator.sendto(packet, operator_peer) == len(packet), "operator Guided packet not sent exactly once")
                    result["operator_effect"] = {"datagrams_sent": 1, "source": [255, 190], "target": 1, "mode": 4}
                    record("operator_sent")
                    ack(control, name, token)
                    index += 1
                elif token is not None and name == "handback.ready" and newest_mode(4, result["events"]["operator_sent"]):
                    record(name)
                    result["marker_observations"]["before_handback"] = write_marker_snapshot(out, state, "before-handback")
                    ack(control, name, token)
                    index += 1
                elif token is not None and name == "guidance.active":
                    if name not in result["events"]:
                        record(name)
                    after = result["events"][name]
                    start = next((row for row in positions if row["mono"] > after), None)
                    end = positions[-1] if positions else None
                    if start and end and time.monotonic() - end["mono"] <= .5:
                        east = eastward_metres(start, end)
                        if (east >= 3 and 60 <= end["vy"] <= 220 and 3000 <= end["relative_alt"] <= 7000
                                and packets and packets[-1]["mono"] > after and
                                packets[-1]["mode"] == 4 and packets[-1]["armed"]):
                            record("guidance.moved")
                            result["fresh_helm_motion"] = {"start": start, "end": end, "east_m": east}
                            ack(control, name, token)
                            index += 1
            if probe.poll() is not None:
                need(probe.returncode == 0, "recovery probe failed")
                if drain is None:
                    drain = time.monotonic() + 2
                if time.monotonic() >= drain:
                    break
            time.sleep(.01)
        need(index == len(boundaries) and probe.poll() == 0 and drain is not None,
             "original native recovery scenario deadline expired")
        result["probe_exit"] = probe.returncode
        result["owned_helm_after"] = live_helm(ownership, antler, mission)
        result["native_primary_socket_inventory_after"] = inventory(sitl)
        need(exact_peer(result["native_primary_socket_inventory_after"], primary_peer[1], bridge_port),
             "owned Copter primary endpoint changed across restart")
        result["marker_absent_after_handback"] = not (state / MARKER).exists()
        record("capture_finished")
        receiver.close()
        result["passive_receiver"] = receiver.status()
        for stream in capture_streams:
            stream.close()
        # The two app instances have the same configured module composition.
        observed = catalog.observed_catalog(out / "starter.jsonl", set(MODULES), native=True,
            helm=True, mode="SITL_CONTROL", operations_helm=True, mission_execution=True, mission_resume=True)
        result["catalog"] = observed
        result["loaded_modules_sha256"] = catalog.loaded_module_hashes(build / "plugins", observed)
        result["probe_evidence"] = replay_archived(result, lambda name: (out / name).read_bytes(), expected_abi=10,
                                                   expected_frame=frame, expected_type=expected_type,
                                                   expected_class=expected_class)
        result["result"] = "PASS"
    except BaseException as error:
        result["error"] = str(error) or type(error).__name__
        result["traceback"] = traceback.format_exc()
    finally:
        errors = []
        if receiver:
            try:
                receiver.close()
            except BaseException as error:
                errors.append(str(error))
            result["passive_receiver"] = receiver.status()
        else:
            result["passive_receiver"] = {"thread_retired": True, "socket_closed": False, "error": None}
        if receiver is None or result["passive_receiver"]["thread_retired"]:
            for stream in capture_streams:
                stream.close()
        remaining, proven, process_errors = stop_owned_processes(ownership, processes)
        errors.extend(process_errors)
        for stream in sockets + logs:
            stream.close()
        if receiver is None:
            result["passive_receiver"]["socket_closed"] = all(stream.fileno() == -1 for stream in sockets)
        result["ports_clear"] = {}
        for role, port in result["ports"].items():
            kind = socket.SOCK_STREAM if role in ("db", "base") else socket.SOCK_DGRAM
            try:
                with socket.socket(socket.AF_INET, kind) as check:
                    if kind == socket.SOCK_STREAM:
                        check.settimeout(.2)
                        clear = check.connect_ex(("127.0.0.1", port)) != 0
                    else:
                        check.bind(("0.0.0.0" if role == "rc" else "127.0.0.1", port))
                        clear = True
            except OSError:
                clear = False
            result["ports_clear"][role] = clear
        # Shared cleanup helper checks the pre-existing primary/observer ports.
        # The extra operator endpoint is a separate required cleanup witness.
        full_ports = result["ports_clear"]
        result["ports_clear"] = {key: value for key, value in full_ports.items()
                                 if key not in ("operator", "operator_peer")}
        finalize_cleanup(result, remaining, proven, errors)
        result["ports_clear"] = full_ports
        if any(full_ports.get(key) is not True for key in ("operator", "operator_peer")):
            result["owned_processes_remaining"] = None
            result["cleanup_proven"] = False
            result["result"] = "FAIL"
        try:
            result["firmware_identity_after"] = firmware.verify_native(
                manifest_path, attestation_path, ap, expected_profile="arducopter")
            result["source_ref_after"], result["ardupilot_ref_after"], result["moos_ref_after"] = (
                revision(source), revision(ap), revision(moos))
            result["artifacts_after"] = {str(path): sha(path) for path in protected}
            result["artifact_drift"] = [path for path in before if before[path] != result["artifacts_after"][path]]
            need(result["firmware_identity_after"] == verified and not result["artifact_drift"]
                 and (result["source_ref_after"], result["ardupilot_ref_after"], result["moos_ref_after"])
                 == (source_ref, ap_ref, moos_ref),
                 "protected source/artifact/firmware identity changed")
            if extended and result["result"] == "PASS":
                result["frame_identity"]["dataflash_parameters"] = one_copter_frame_log(out / "logs", frame)
        except BaseException as error:
            result["result"] = "FAIL"
            result["postflight_error"] = str(error)
        result["files"] = {str(path.relative_to(out)): sha(path) for path in sorted(out.rglob("*"))
                           if path.is_file() and not path.is_symlink()}
        dump(out / "result.json", result)
    print(json.dumps({key: result.get(key) for key in ("result", "scenario", "error", "cleanup_proven",
          "postflight_error", "owned_processes_remaining", "probe_evidence", "ports_clear")}, indent=2))
    return 0 if result["result"] == "PASS" else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for flag in ("source-root", "ardupilot-root", "moos-root", "behavior-dir", "build", "output",
                 "probe", "starter-probe"):
        parser.add_argument("--" + flag, required=True)
    parser.add_argument("--firmware-manifest")
    parser.add_argument("--firmware-attestation")
    parser.add_argument("--copter-frame", choices=tuple(STANDARD_COPTER_FRAMES), default="quad")
    args = parser.parse_args()
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(143)))
    with (Path(tempfile.gettempdir()) / "mavmoos-native-sitl.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        raise SystemExit(run(args))
