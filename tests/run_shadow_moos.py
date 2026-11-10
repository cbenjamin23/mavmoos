#!/usr/bin/env python3
"""Real stock-helm/bridge shadow integration with fixture-only UDP and scoped teardown."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
from control.catalog_evidence import observed_catalog, loaded_module_hashes


def free_port(kind):
    with socket.socket(socket.AF_INET, kind) as reservation:
        reservation.bind(("127.0.0.1", 0))
        return reservation.getsockname()[1]


def main():
    parser = argparse.ArgumentParser()
    for key in ["moos-bin", "app", "probe", "plugin-dir", "behavior-dir", "output"]:
        parser.add_argument("--"+key, type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    db, udp, peer = free_port(socket.SOCK_STREAM), free_port(socket.SOCK_DGRAM), free_port(socket.SOCK_DGRAM)
    while udp == peer:
        peer = free_port(socket.SOCK_DGRAM)
    root = Path(__file__).resolve().parents[1]
    source = root/"missions/copter_sitl"
    substitutions = {"MOOS_PORT": db, "BIND_PORT": udp, "PEER_PORT": peer,
                     "APP_PATH": args.app.resolve(), "PLUGIN_DIRECTORY": args.plugin_dir.resolve(),
                     "BEHAVIOR_DIRECTORY": args.behavior_dir.resolve(), "SIMULATION_ONLY": "false",
                     "SHADOW_MODE": "true", "SITL_PID": 0, "LAT_ORIGIN": 42.358456,
                     "LONG_ORIGIN": -71.087589, "SPEED": 1, "HEIGHT": 5,
                     "WAYPOINTS": "0,8:8,8:8,0:0,0"}
    for suffix in ["moos", "bhv"]:
        text = (source/("meta_vehicle."+suffix)).read_text()
        for key, value in substitutions.items():
            text = text.replace("$("+key+")", str(value))
        # Shadow has no simulator process and therefore no FC process binding.
        text = text.replace("  sitl_pid = 0\n", "")
        if "$(" in text:
            raise RuntimeError("unresolved mission substitution")
        (output/("targ_copter."+suffix)).write_text(text)
    (output/"logs").mkdir(exist_ok=True)
    env = dict(os.environ)
    env["PATH"] = str(args.moos_bin.resolve())+os.pathsep+env.get("PATH", "")
    env["IVP_BEHAVIOR_DIRS"] = str(args.behavior_dir.resolve())
    probe = None
    with (output/"pantler.log").open("w") as console:
        antler = subprocess.Popen([str(args.moos_bin/"pAntler"), str(output/"targ_copter.moos")],
                                  cwd=output, env=env, stdout=console, stderr=subprocess.STDOUT,
                                  start_new_session=True)
        try:
            probe = subprocess.run([str(args.probe.resolve()), str(db), str(udp), str(peer)], env=env,
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=16)
            (output/"probe.log").write_text(probe.stdout)
        except subprocess.TimeoutExpired as error:
            text = error.stdout or b""
            if isinstance(text, bytes):
                text = text.decode(errors="replace")
            (output/"probe.log").write_text(text+"\nFAIL probe timeout\n")
        finally:
            try:
                os.killpg(antler.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                antler.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(antler.pid, signal.SIGKILL)
                antler.wait()
            try:
                os.killpg(antler.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
    cleared = {}
    for key, port, kind in [("moos_tcp", db, socket.SOCK_STREAM), ("bridge_udp", udp, socket.SOCK_DGRAM),
                            ("fixture_udp", peer, socket.SOCK_DGRAM)]:
        with socket.socket(socket.AF_INET, kind) as check:
            if kind == socket.SOCK_STREAM:
                check.settimeout(0.2)
                cleared[key] = check.connect_ex(("127.0.0.1", port)) != 0
            else:
                try:
                    check.bind(("127.0.0.1", port))
                    cleared[key] = True
                except OSError:
                    cleared[key] = False
    remaining = []
    for line in subprocess.check_output(["ps", "-axo", "pid,pgid,comm"], text=True).splitlines()[1:]:
        columns = line.strip().split(None, 2)
        if len(columns) == 3 and columns[1] == str(antler.pid):
            remaining.append(line.strip())
    passed = probe is not None and probe.returncode == 0 and all(cleared.values()) and not remaining
    report = {"layer": "stock MOOS/helm + synthetic UDP peer; shadow only", "result": "PASS" if passed else "FAIL",
              "probe_exit": probe.returncode if probe else None, "ports_clear": cleared,
              "owned_processes_remaining": remaining,
              "endpoints": {"moos_tcp": db, "bridge_udp": udp, "fixture_udp": peer}}
    if passed:
        try:
            report["capabilities_catalog"] = observed_catalog(output / "probe.log", {"telemetry", "arducopter_control"}, helm=True, mode="SHADOW")
            report["loaded_modules_sha256"] = loaded_module_hashes(args.plugin_dir, report["capabilities_catalog"])
        except (RuntimeError, ValueError, KeyError) as error:
            passed = False; report["result"] = "FAIL"; report["composition_error"] = str(error)
    (output/"result.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report))
    if not passed:
        print((output/"probe.log").read_text()[-2500:])
        print((output/"pantler.log").read_text()[-1500:])
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
