#!/usr/bin/env python3
"""Bounded, owned pAntler + concurrent receive fixture; no detached mission."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
from control.catalog_evidence import observed_catalog, loaded_module_hashes
import time


def interrupted(signum, _frame):
    raise SystemExit(128 + signum)


def free_port(kind):
    with socket.socket(socket.AF_INET, kind) as reservation:
        reservation.bind(("127.0.0.1", 0))
        return reservation.getsockname()[1]


def main():
    parser = argparse.ArgumentParser()
    for name in ["moos-bin", "app", "probe", "scenario", "output"]:
        parser.add_argument("--" + name, required=True)
    args = parser.parse_args()
    signal.signal(signal.SIGTERM, interrupted)
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    db = free_port(socket.SOCK_STREAM)
    udp = free_port(socket.SOCK_DGRAM)
    peer = free_port(socket.SOCK_DGRAM)
    while peer == udp:
        peer = free_port(socket.SOCK_DGRAM)
    invalid = args.scenario != "normal"
    mission = output / "fixture.moos"
    target = "" if args.scenario == "invalid_config" else "target_system = 1"
    profile = {
        "invalid_streamed": "plugins = parameters,commands\n  parameter_list_mode = streamed",
        "invalid_auto_recovery": "plugins = arducopter_control\n  control_mission_execution = true",
    }.get(args.scenario, "")
    mission.write_text(f"""ServerHost = 127.0.0.1
ServerPort = {db}
Community = mavmoos_p1_{db}
MOOSTimeWarp = 1
TERM_REPORTING = false
ProcessConfig = ANTLER
{{
  MSBetweenLaunches = 50
  Run = MOOSDB @ NewConsole = false
  Run = iMAVLink @ NewConsole = false
}}
ProcessConfig = iMAVLink
{{
  AppTick = 20
  CommsTick = 20
  bind_address = 127.0.0.1
  bind_port = {udp}
  peer_address = 127.0.0.1
  peer_port = {peer}
  {target}
  {profile}
  target_component = 1
  expected_type = 2
  expected_autopilot = 3
  publish_interval = 0.1
}}
""")
    env = dict(os.environ)
    env["PATH"] = os.pathsep.join([str(Path(args.app).resolve().parent),
                                  str(Path(args.moos_bin).resolve()), env.get("PATH", "")])
    probe = None
    # pAntler and the probe must run concurrently to exercise real MOOS I/O.
    # An owned process group, bounded timeout and finally block scope cleanup.
    with (output / "pantler.log").open("w") as log:
        launch = subprocess.Popen([str(Path(args.moos_bin) / "pAntler"), str(mission)],
                                  stdout=log, stderr=subprocess.STDOUT, env=env,
                                  cwd=output, start_new_session=True)
        try:
            probe = subprocess.run([args.probe, str(db), str(udp), str(peer), args.scenario],
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   text=True, timeout=16, env=env)
            (output / "probe.log").write_text(probe.stdout)
        except subprocess.TimeoutExpired as error:
            text = error.stdout or ""
            if isinstance(text, bytes):
                text = text.decode(errors="replace")
            probe = subprocess.CompletedProcess(args.probe, 124, text + "\nFAIL probe timeout\n")
            (output / "probe.log").write_text(probe.stdout)
        finally:
            try:
                os.killpg(launch.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                launch.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
                launch.wait(timeout=2)
            # Reap remaining children in this owned group, even if pAntler exited first.
            try:
                os.killpg(launch.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
    cleared = {}
    for name, port, kind in [("db_tcp", db, socket.SOCK_STREAM),
                              ("bridge_udp", udp, socket.SOCK_DGRAM),
                              ("peer_udp", peer, socket.SOCK_DGRAM)]:
        with socket.socket(socket.AF_INET, kind) as check:
            if kind == socket.SOCK_STREAM:
                check.settimeout(0.2)
                cleared[name] = check.connect_ex(("127.0.0.1", port)) != 0
            else:
                try:
                    check.bind(("127.0.0.1", port))
                    cleared[name] = True
                except OSError:
                    cleared[name] = False
    remaining = []
    for line in subprocess.check_output(["ps", "-axo", "pid,pgid,comm"], text=True).splitlines()[1:]:
        columns = line.strip().split(None, 2)
        if len(columns) == 3 and columns[1] == str(launch.pid):
            remaining.append(line.strip())
    report = {"scenario": args.scenario, "layer": "loopback fixtures + stock MOOSDB/pAntler",
              "autopilot": None, "firmware": None, "timewarp": 1,
              "endpoints": {"db_tcp": db, "bridge_udp": udp, "peer_udp": peer},
              "probe_exit": probe.returncode if probe else None,
              "ports_clear": cleared, "owned_processes_remaining": remaining}
    if probe:
        report["observations"] = [line.split("\t", 3)[1:] for line in probe.stdout.splitlines()
                                  if line.startswith("OBS\t")]
        report["fixture_packets"] = [line.split("\t")[1:] for line in probe.stdout.splitlines()
                                     if line.startswith("FIXTURE_PACKET\t")]
        report["endpoint_capture"] = [line.split("\t")[1:] for line in probe.stdout.splitlines()
                                     if line.startswith("CAPTURE\t")]
    passed = probe is not None and probe.returncode == 0 and all(cleared.values()) and not remaining
    if passed and args.scenario == "normal":
        try:
            report["capabilities_catalog"] = observed_catalog(output / "probe.log", {"telemetry"}, mode="RX_ONLY")
            report["loaded_modules_sha256"] = loaded_module_hashes(Path(args.app).resolve().parent.parent / "plugins", report["capabilities_catalog"])
        except (RuntimeError, ValueError, KeyError) as error:
            passed = False; report["composition_error"] = str(error)
    report["result"] = "PASS" if passed else "FAIL"
    (output / "result.json").write_text(json.dumps(report, indent=2) + "\n")
    if probe:
        print(probe.stdout, end="")
    print(json.dumps({key: report[key] for key in ["scenario", "result", "ports_clear",
                                                   "owned_processes_remaining"]}))
    if not passed:
        print((output / "pantler.log").read_text()[-9000:])
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
