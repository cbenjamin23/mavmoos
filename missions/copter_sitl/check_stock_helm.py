#!/usr/bin/env python3
"""Exercise unchanged pHelmIvP and the existing altitude behavior on owned ports."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--moos-bin", type=Path, default=Path.home()/"moos-ivp/bin")
    parser.add_argument("--behavior-dir", type=Path, default=Path.home()/"moos-ivp-uav-base/lib")
    args = parser.parse_args()
    work = args.output.resolve()
    work.mkdir(parents=True, exist_ok=True)
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
    bhv = work / "stock.bhv"
    template = Path(__file__).with_name("meta_vehicle.bhv").read_text()
    bhv.write_text(template.replace("$(SPEED)", "1.5").replace("$(HEIGHT)", "10")
                   .replace("$(WAYPOINTS)", "0,8:8,8:8,0:0,0"))
    mission = work / "stock.moos"
    mission.write_text(f"""ServerHost = localhost
ServerPort = {port}
Community = copter_helm_check
MOOSTimeWarp = 1
LatOrigin = -35.363261
LongOrigin = 149.165230
ProcessConfig = ANTLER
{{
  MSBetweenLaunches = 100
  Run = MOOSDB @ NewConsole = false
  Run = pHelmIvP @ NewConsole = false
}}
ProcessConfig = pHelmIvP
{{
  AppTick = 10
  CommsTick = 20
  behaviors = {bhv}
  domain = course:0:359:360
  domain = speed:0:5:51
  domain = altitude:0:50:51
  start_in_drive = true
  verbose = quiet
  ivp_behavior_dir = {args.behavior_dir.resolve()}
}}
""")
    env = dict(os.environ)
    env["PATH"] = str(args.moos_bin.resolve()) + os.pathsep + env.get("PATH", "")
    env["IVP_BEHAVIOR_DIRS"] = str(args.behavior_dir.resolve())
    result = {"layer": "stock_helm_process_test", "port": port,
              "behavior_directory": str(args.behavior_dir.resolve()), "pass": False}
    with (work / "console.log").open("w") as out:
        antler = subprocess.Popen([str(args.moos_bin/"pAntler"), str(mission)], cwd=work,
                                  env=env, stdout=out, stderr=subprocess.STDOUT,
                                  start_new_session=True)
        try:
            time.sleep(0.7)
            poke = subprocess.run([str(args.moos_bin/"uPokeDB"), str(mission),
                "NAV_X=0", "NAV_Y=0", "NAV_HEADING=0", "NAV_SPEED=0", "NAV_ALTITUDE=10",
                "DEPLOY=true", "MOOS_MANUAL_OVERRIDE=false"], cwd=work, env=env,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=12)
            (work/"poke.log").write_text(poke.stdout)
            query = subprocess.run([str(args.moos_bin/"uQueryDB"), str(mission),
                "--condition=DESIRED_ALTITUDE=10", "--condition=DESIRED_SPEED>0",
                "--condition=IVPHELM_ALLSTOP=clear", "--condition=IVPHELM_STATE=DRIVE",
                "--wait=6", "--check_var=DESIRED_HEADING", "--check_var=DESIRED_SPEED",
                "--check_var=DESIRED_ALTITUDE"], cwd=work, env=env,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=12)
            (work/"query.log").write_text(query.stdout)
            result.update(poke_returncode=poke.returncode, query_returncode=query.returncode)
            result["pass"] = poke.returncode == 0 and query.returncode == 0
        finally:
            try:
                os.killpg(antler.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                antler.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(antler.pid, signal.SIGKILL)
                antler.wait()
    # Closing this test's listeners is part of the test's acceptance boundary.
    with socket.socket() as probe:
        probe.settimeout(0.2)
        result["port_closed"] = probe.connect_ex(("127.0.0.1", port)) != 0
    result["pass"] = result["pass"] and result["port_closed"]
    (work/"result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
