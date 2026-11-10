#!/usr/bin/env python3
"""Real-process fault scenarios against owned native Copter and stock MOOS."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from control.catalog_evidence import observed_catalog, loaded_module_hashes
import run_sitl as fixture
from pymavlink import mavutil

SCENARIOS = ['helm_pause', 'moosdb_restart', 'simulator_pause', 'simulator_restart', 'command_timeout', 'bridge_pause']


def owned_pid(parent, name):
    for line in subprocess.check_output(['ps', '-axo', 'pid,ppid,comm'], text=True).splitlines()[1:]:
        fields = line.strip().split(None, 2)
        if len(fields) == 3 and int(fields[1]) == parent and Path(fields[2]).name == name:
            return int(fields[0])
    raise RuntimeError(f'{name} not found under owned parent {parent}')


def mission_processes(mission):
    result = []
    for line in subprocess.check_output(['ps', '-axo', 'pid,pgid,args'], text=True).splitlines()[1:]:
        fields = line.strip().split(None, 2)
        if len(fields) != 3 or str(mission) not in fields[2]: continue
        executable = Path(fields[2].split()[0]).name
        if executable in ['MOOSDB', 'pAntler', 'pHelmIvP', 'iMAVLink', 'pLogger', 'uProcessWatch']:
            result.append({'pid': int(fields[0]), 'pgid': int(fields[1]), 'args': fields[2]})
    return result


def run(args, scenario):
    root = Path(__file__).resolve().parents[2]
    out = Path(args.output).resolve() / scenario
    out.mkdir(parents=True, exist_ok=False)
    (out / 'logs').mkdir()
    control = out / 'control'; control.mkdir()
    ap, moos, build, behavior = [Path(x).resolve() for x in [args.ardupilot_root, args.moos_root, args.build, args.behavior_dir]]
    firmware = fixture.native_firmware_profile(root, ap)
    native = ap / 'build/sitl/bin/arducopter'
    discovery = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    discovery.bind(('127.0.0.1', 0)); discovery.settimeout(.2)
    bridge_port = discovery.getsockname()[1]
    monitor = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    monitor.bind(('127.0.0.1', 0)); monitor.setblocking(False)
    monitor_port = monitor.getsockname()[1]
    db, base, rc = fixture.port(socket.SOCK_STREAM), fixture.port(socket.SOCK_STREAM), fixture.port(socket.SOCK_DGRAM)
    defaults = out / 'fixture.parm'
    defaults.write_text('SERIAL1_PROTOCOL 2\nMAV2_POSITION 10\nMAV2_EXT_STAT 2\nMAV2_EXTRA3 2\nLOG_DISARMED 1\n')
    (out / 'message-intervals-chan1.txt').write_text('33 100\n245 500\n')
    command = [str(native), '--model', 'quad', '--home', '42.358456,-71.087589,10,0', '--speedup', '1',
               '--defaults', str(ap / 'Tools/autotest/default_params/copter.parm') + ',' + str(defaults),
               '--serial0', f'udpclient:127.0.0.1:{bridge_port}', '--serial1', f'udpclient:127.0.0.1:{monitor_port}',
               '--serial2', 'none', '--serial5', 'none', '--serial6', 'none', '--serial7', 'none', '--serial8', 'none',
               '--base-port', str(base), '--rc-in-port', str(rc)]
    result = {'scenario': scenario, 'result': 'FAIL', 'layer': 'native Copter + stock MOOS process fault', 'hardware': False, 'firmware_identity': firmware,
              'app_sha256': fixture.sha(build / 'bin/iMAVLink'), 'native_sha256': fixture.sha(native),
              'probe_sha256': fixture.sha(Path(args.probe).resolve()),
              'harness_sha256': fixture.sha(Path(__file__).resolve()),
              'probe_source_sha256': fixture.sha(Path(__file__).with_name('FaultProbe.cpp')),
              'recovery_protocol': 'stock manual override true/false with source-qualified all-stop observations, then explicit acquire' if scenario == 'moosdb_restart' else 'fresh data then explicit acquire; restart requires rebind' if scenario != 'command_timeout' else 'terminal timeout retained; fresh command ID required',
              'modules_sha256': {p.name: fixture.sha(p) for p in sorted((build / 'plugins').glob('mavmoos_*')) if p.is_file()},
              'moos_revision': subprocess.check_output(['git', '-C', str(moos), 'rev-parse', 'HEAD'], text=True).strip(),
              'ardupilot_revision': subprocess.check_output(['git', '-C', str(ap), 'rev-parse', 'HEAD'], text=True).strip(),
              'command': command, 'ports': {'bridge': bridge_port, 'monitor': monitor_port, 'db': db, 'rc': rc}, 'events': []}
    processes = []; handles = []; stopped = set(); launch = probe = sitl = None; mission = out / 'targ_copter.moos'
    def spawn(cmd, logfile, env=None):
        stream = (out / logfile).open('w'); handles.append(stream)
        process = subprocess.Popen(cmd, cwd=out, env=env, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process); return process
    def event(kind, **values):
        result['events'].append({'mono': time.monotonic(), 'kind': kind, **values})
    packets = []
    try:
        sitl = spawn(command, 'sitl.log')
        decoder = mavutil.mavlink.MAVLink(None); decoder.robust_parsing = True
        deadline = time.monotonic() + 20; peer = None
        while time.monotonic() < deadline:
            if sitl.poll() is not None: raise RuntimeError('native SITL exited before heartbeat')
            try: data, address = discovery.recvfrom(4096)
            except socket.timeout: continue
            if any(m.get_type() == 'HEARTBEAT' and m.get_srcSystem() == 1 and m.get_srcComponent() == 1 for m in decoder.parse_buffer(data) or []):
                peer = address; break
        if peer is None or peer[0] != '127.0.0.1': raise RuntimeError('owned SITL peer discovery failed')
        result['ports']['peer'] = peer[1]; result['initial_sitl_pid'] = sitl.pid
        discovery.close()
        env = dict(os.environ); env['PATH'] = os.pathsep.join([str(build / 'bin'), str(moos / 'bin'), env.get('PATH', '')]); env['IVP_BEHAVIOR_DIRS'] = str(behavior)
        macros = {'MOOS_PORT': db, 'APP_PATH': build / 'bin/iMAVLink', 'BIND_PORT': bridge_port, 'PEER_PORT': peer[1],
                  'LAT_ORIGIN': '42.358456', 'LONG_ORIGIN': '-71.087589', 'SIMULATION_ONLY': 'true', 'SHADOW_MODE': 'false',
                  'SITL_PID': sitl.pid, 'PLUGIN_DIRECTORY': build / 'plugins', 'BEHAVIOR_DIRECTORY': behavior,
                  'HEIGHT': 5, 'SPEED': 1, 'WAYPOINTS': '0,20:20,20:20,0:0,0'}
        for suffix in ['moos', 'bhv']:
            subprocess.run([str(moos / 'bin/nsplug'), str(root / f'missions/copter_sitl/meta_vehicle.{suffix}'), str(out / f'targ_copter.{suffix}'),
                            '--strict', '--force', '-x'] + [f'{k}={v}' for k, v in macros.items()], check=True, env=env, capture_output=True)
        mission = out / 'targ_copter.moos'
        if scenario == 'command_timeout': mission.write_text(mission.read_text().replace('  AppTick = 30', '  command_ack_timeout = 0.4\n  AppTick = 30', 1))
        result['mission_sha256'] = fixture.sha(mission)
        result['behavior_sha256'] = fixture.sha(out / 'targ_copter.bhv')
        launch = spawn([str(moos / 'bin/pAntler'), str(mission)], 'pantler.log', env)
        probe = spawn([str(Path(args.probe).resolve()), str(db), scenario, str(control)], 'probe.jsonl', env)
        decoder = mavutil.mavlink.MAVLink(None); decoder.robust_parsing = True
        injected = restored = False; target_pid = None; drain = None
        with (out / 'fc-observer.jsonl').open('w') as trace, (out / 'fc-observer.mavlink').open('wb') as raw:
            deadline = time.monotonic() + 190
            while time.monotonic() < deadline:
                if probe.poll() is not None:
                    if drain is None: drain = time.monotonic() + 1.5
                    if time.monotonic() >= drain: break
                if not injected and (control / 'inject.request').exists():
                    if scenario in ['helm_pause', 'bridge_pause']:
                        target_name = 'iMAVLink' if scenario == 'bridge_pause' else 'pHelmIvP'
                        target_pid = owned_pid(launch.pid, target_name); os.kill(target_pid, signal.SIGSTOP); stopped.add(target_pid)
                    elif scenario == 'moosdb_restart':
                        target_pid = owned_pid(launch.pid, 'MOOSDB'); os.kill(target_pid, signal.SIGTERM)
                    elif scenario == 'simulator_restart':
                        target_pid = sitl.pid; fixture.stop(sitl)
                    else:
                        target_pid = sitl.pid; os.kill(target_pid, signal.SIGSTOP); stopped.add(target_pid)
                    injected_at = time.monotonic()
                    event('injected', target_pid=target_pid); (control / 'injected').write_text(str(injected_at)); injected = True
                # A paused bridge cannot report revocation. Resume it before the probe
                # observes the post-boundary latch, while FC/MOOS traffic continues.
                automatic_restore = scenario == 'bridge_pause' and injected and time.monotonic() - injected_at >= .8
                if injected and not restored and ((control / 'restore.request').exists() or automatic_restore):
                    if scenario == 'moosdb_restart':
                        replacement = spawn([str(moos / 'bin/MOOSDB'), str(mission)], 'moosdb-replacement.log', env)
                        event('restored', replacement_pid=replacement.pid)
                    elif scenario == 'simulator_restart':
                        sitl = spawn(command, 'sitl-replacement.log'); result['replacement_sitl_pid'] = sitl.pid
                        event('restored', replacement_pid=sitl.pid)
                    else:
                        os.kill(target_pid, signal.SIGCONT); stopped.discard(target_pid); event('restored', target_pid=target_pid)
                    (control / 'restored').write_text(str(time.monotonic())); restored = True
                try: data, address = monitor.recvfrom(65535)
                except BlockingIOError: time.sleep(.01); continue
                raw.write(data)
                for message in decoder.parse_buffer(data) or []:
                    if message.get_srcSystem() != 1 or message.get_srcComponent() != 1: continue
                    if message.get_type() in ['HEARTBEAT', 'GLOBAL_POSITION_INT', 'EXTENDED_SYS_STATE', 'STATUSTEXT']:
                        item = {'mono': time.monotonic(), 'endpoint': list(address), **message.to_dict()}; packets.append(item); trace.write(json.dumps(item) + '\n')
            if probe.poll() is None: raise RuntimeError('fault scenario exceeded190s deadline')
        result['probe_exit'] = probe.returncode
        result['injected'] = injected; result['restored'] = restored
        hearts = [m for m in packets if m['mavpackettype'] == 'HEARTBEAT']; landed = [m for m in packets if m['mavpackettype'] == 'EXTENDED_SYS_STATE']; positions = [m for m in packets if m['mavpackettype'] == 'GLOBAL_POSITION_INT']
        result['observer'] = {'writes': 0, 'heartbeat_samples': len(hearts), 'position_samples': len(positions),
            'maximum_relative_height_m': max([m['relative_alt'] / 1000 for m in positions], default=0),
            'final_disarmed': bool(hearts) and not bool(hearts[-1]['base_mode'] & 128),
            'final_on_ground': bool(landed) and landed[-1]['landed_state'] == 1,
            'observed_modes': sorted(set(m['custom_mode'] for m in hearts))}
        if probe.returncode != 0 or not injected or not restored: raise RuntimeError('fault probe or coordinator failed')
        result['capabilities_catalog'] = observed_catalog(out / 'probe.jsonl', {'telemetry', 'parameters', 'arducopter_control'}, native=True, helm=True, mode="SITL_CONTROL")
        result['loaded_modules_sha256'] = loaded_module_hashes(build / 'plugins', result['capabilities_catalog'])
        if scenario not in ['command_timeout', 'simulator_restart'] and (not result['observer']['final_disarmed'] or not result['observer']['final_on_ground'] or result['observer']['maximum_relative_height_m'] < 4): raise RuntimeError('independent observer did not corroborate flight and ground finish')
        if scenario == 'command_timeout' and any(m['base_mode'] & 128 for m in hearts): raise RuntimeError('ground timeout test unexpectedly armed simulator')
        if scenario == 'simulator_restart':
            restored_at = next(e['mono'] for e in result['events'] if e['kind'] == 'restored')
            fresh_hearts = [m for m in hearts if m['mono'] > restored_at + 1]
            result['replacement_observer'] = {'heartbeat_samples': len(fresh_hearts), 'disarmed': bool(fresh_hearts) and all(not (m['base_mode'] & 128) for m in fresh_hearts)}
            if sitl.poll() is not None or not result['replacement_observer']['disarmed']: raise RuntimeError('replacement simulator did not independently boot disarmed')
        result['result'] = 'PASS'
    except Exception as error:
        result['error'] = str(error)
    finally:
        for pid in stopped:
            try: os.kill(pid, signal.SIGCONT)
            except ProcessLookupError: pass
        for process in reversed(processes): fixture.stop(process)
        # pAntler creates a distinct process group per child. Check exact mission
        # ownership as well as launcher groups, and clean only those leftovers.
        leftover = mission_processes(mission)
        for process in leftover:
            try: os.kill(process['pid'], signal.SIGTERM)
            except ProcessLookupError: pass
        if leftover: time.sleep(.3)
        for process in mission_processes(mission):
            try: os.kill(process['pid'], signal.SIGKILL)
            except ProcessLookupError: pass
        discovery.close(); monitor.close()
        for handle in handles: handle.close()
        result['ports_clear'] = {}
        for name in ['bridge', 'monitor', 'peer', 'rc']:
            if name not in result['ports']: continue
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                try: s.bind(('127.0.0.1', result['ports'][name])); result['ports_clear'][name] = True
                except OSError: result['ports_clear'][name] = False
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(.2); result['ports_clear']['moos_tcp'] = s.connect_ex(('127.0.0.1', db)) != 0
        groups = {p.pid for p in processes}; result['owned_processes_remaining'] = []
        for line in subprocess.check_output(['ps', '-axo', 'pid,pgid,comm'], text=True).splitlines()[1:]:
            f = line.strip().split(None, 2)
            if len(f) == 3 and int(f[1]) in groups: result['owned_processes_remaining'].append(line.strip())
        result['mission_processes_remaining'] = mission_processes(mission)
        if not all(result['ports_clear'].values()) or result['owned_processes_remaining'] or result['mission_processes_remaining']: result['result'] = 'FAIL'
        (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: result.get(k) for k in ['scenario', 'result', 'error', 'observer', 'ports_clear', 'owned_processes_remaining']}, indent=2), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser()
    for key in ['ardupilot-root', 'moos-root', 'build', 'probe', 'behavior-dir', 'output']: parser.add_argument('--' + key, required=True)
    parser.add_argument('--scenario', choices=['all'] + SCENARIOS, default='all')
    args = parser.parse_args(); outcomes = []
    # The simulator has a fixed internal IRLock helper port; serialize native-host runs.
    with (Path(tempfile.gettempdir()) / 'mavmoos-native-sitl.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        for scenario in SCENARIOS if args.scenario == 'all' else [args.scenario]: outcomes.append(run(args, scenario))
    summary = {'layer': 'native Copter process faults', 'result': 'PASS' if all(r['result'] == 'PASS' for r in outcomes) else 'FAIL',
               'scenarios': [{'scenario': r['scenario'], 'result': r['result'], 'error': r.get('error')} for r in outcomes]}
    Path(args.output).resolve().joinpath('summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    return 0 if summary['result'] == 'PASS' else 1


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(143)))
    raise SystemExit(main())
