#!/usr/bin/env python3
"""Exercise the exact ordinary operation behaviors with stock helm, without FC I/O."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tests'))
from owned_processes import stop_process_tree


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--moos-bin', type=Path, default=Path.home()/'moos-ivp/bin')
    parser.add_argument('--behavior-dir', type=Path, default=Path.home()/'moos-ivp-uav-base/lib')
    args = parser.parse_args()
    out = args.output.resolve(); out.mkdir(parents=True, exist_ok=False)
    with socket.socket() as reservation:
        reservation.bind(('127.0.0.1', 0)); port = reservation.getsockname()[1]
    bhv = out/'stock_operations.bhv'
    template = ROOT/'missions/copter_operations/meta_vehicle.bhv'
    bhv.write_text(template.read_text().replace('$(SPEED)', '1').replace('$(HEIGHT)', '5').replace('$(WAYPOINTS)', '0,0'))
    mission = out/'stock_operations.moos'
    mission.write_text(f'''ServerHost = localhost
ServerPort = {port}
Community = stock_copter_operations_check
MOOSTimeWarp = 1
LatOrigin = 42.358456
LongOrigin = -71.087589
ProcessConfig = ANTLER
{{
  MSBetweenLaunches = 100
  Run = MOOSDB @ NewConsole = false
  Run = pHelmIvP @ NewConsole = false
  Run = pLogger @ NewConsole = false
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
ProcessConfig = pLogger
{{
  AppTick = 10
  CommsTick = 20
  File = LOG_STOCK_OPERATIONS
  PATH = .
  AsyncLog = true
  SyncLog = false
  WildCardLogging = true
}}
''')
    env = dict(os.environ)
    env['PATH'] = str(args.moos_bin.resolve())+os.pathsep+env.get('PATH', '')
    env['IVP_BEHAVIOR_DIRS'] = str(args.behavior_dir.resolve())
    result = {'layer': 'stock pHelmIvP + ordinary operation behavior configuration; no FC/app connection',
              'hardware': False, 'pass': False, 'port': port, 'phases': [],
              'template_sha256': hashlib.sha256(template.read_bytes()).hexdigest()}
    antler = None
    started = time.monotonic()
    try:
        with (out/'console.log').open('w') as log:
            antler = subprocess.Popen([str(args.moos_bin/'pAntler'), str(mission)], cwd=out, env=env,
                                      stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            time.sleep(.7)
            def phase(name, pokes, conditions):
                if time.monotonic()-started > 26:
                    raise RuntimeError('stock-only mission deadline')
                commands = [[str(args.moos_bin/'uPokeDB'), str(mission)]+pokes,
                            [str(args.moos_bin/'uQueryDB'), str(mission), '--wait=3',
                             '--condition=IVPHELM_STATE=DRIVE', '--condition=IVPHELM_ALLSTOP=clear']+
                            ['--condition='+value for value in conditions]+
                            ['--check_var='+value for value in ('DESIRED_HEADING', 'DESIRED_SPEED',
                             'DESIRED_ALTITUDE', 'IVPHELM_ALLSTOP')]]
                returns = []
                for kind, command in zip(('poke', 'query'), commands):
                    run = subprocess.run(command, cwd=out, env=env, capture_output=True, text=True, timeout=6)
                    (out/(name+'-'+kind+'.log')).write_text(run.stdout+run.stderr)
                    returns.append(run.returncode)
                    if run.returncode:
                        raise RuntimeError(name+' '+kind+' failed')
                result['phases'].append({'name': name, 'returncodes': returns})
            phase('route', ['NAV_X=0', 'NAV_Y=0', 'NAV_HEADING=0', 'NAV_SPEED=0', 'NAV_ALTITUDE=5',
                           'MOOS_MANUAL_OVERRIDE=false',
                           'MAVMOOS_OPERATION_HOLD_UPDATE=station_pt=x=6,y=0,label=check_route#center_activate=false#inner_radius=0.5#outer_radius=2#outer_speed=1#transit_speed=1',
                           'MAVMOOS_OPERATION_ALTITUDE_UPDATE=altitude=5', 'MAVMOOS_HELM_OPERATION=route'],
                  ['DESIRED_SPEED>0.5', 'DESIRED_ALTITUDE=5'])
            phase('route_at_exact_goal', ['NAV_X=6'], ['DESIRED_SPEED=0', 'DESIRED_ALTITUDE=5'])
            phase('route_goal_stationkeep', ['MAVMOOS_HELM_OPERATION=hold'],
                  ['DESIRED_SPEED=0', 'DESIRED_ALTITUDE=5'])
            phase('hold_update', ['MAVMOOS_OPERATION_HOLD_UPDATE=station_pt=x=6,y=4,label=check_hold#center_activate=false',
                                 'MAVMOOS_OPERATION_ALTITUDE_UPDATE=altitude=7', 'MAVMOOS_HELM_OPERATION=hold'],
                  ['DESIRED_SPEED>0.5', 'DESIRED_ALTITUDE=7'])
            phase('second_route', ['MAVMOOS_OPERATION_HOLD_UPDATE=station_pt=x=0,y=0,label=check_return#center_activate=false#inner_radius=0.5#outer_radius=2#outer_speed=1#transit_speed=1',
                                  'MAVMOOS_OPERATION_ALTITUDE_UPDATE=altitude=5', 'MAVMOOS_HELM_OPERATION=route'],
                  ['DESIRED_SPEED>0.5', 'DESIRED_ALTITUDE=5'])
            result['pass'] = True
    except Exception as error:
        result['error'] = str(error)
    finally:
        result['owned_processes_remaining'] = stop_process_tree(antler)
        with socket.socket() as sock:
            sock.settimeout(.2); result['port_clear'] = sock.connect_ex(('127.0.0.1', port)) != 0
        result['elapsed_s'] = time.monotonic()-started
        result['pass'] = result['pass'] and result['port_clear'] and not result['owned_processes_remaining']
        alogs = list(out.glob('*/*.alog'))
        bad = [line for path in alogs for line in path.read_text(errors='replace').splitlines()
               if 'IVPHELM_ALLSTOP' in line and 'MissingDecVars:' in line]
        result['missing_decision_allstop_rows'] = bad
        if len(alogs) != 1 or bad:
            result.update(error='missing alog or unexpected missing-decision transition')
            result['pass'] = False
        result['artifact_sha256'] = {str(path.relative_to(out)): hashlib.sha256(path.read_bytes()).hexdigest()
                                     for path in out.rglob('*') if path.is_file() and path.name != 'result.json'}
        (out/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    return 0 if result['pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
