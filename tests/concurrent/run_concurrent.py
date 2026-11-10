#!/usr/bin/env python3
"""Concurrent protocol/stock-helm native simulator fixtures; no proxy or hardware."""
import argparse
import fcntl
import importlib.util
import json
import math
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
from heartbeat_audit import audit_primary_heartbeat_reset
from mission_read_evidence import mission_read_evidence
from missions.mission_evidence import mission_capabilities_match
import run_sitl as fixture
from owned_processes import owned_records
from pymavlink import mavutil, DFReader

SCENARIOS = ['bulk', 'mission_default', 'bridge_pause', 'mission_read']


def source_fixture_selector(root):
    path = root / 'scripts/firmware/scenario_fixtures.py'
    spec = importlib.util.spec_from_file_location('_concurrent_scenario_fixtures', path)
    if spec is None or spec.loader is None:
        raise RuntimeError('selected source scenario selector unavailable')
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def distance(a, b):
    return math.hypot((a['lat'] - b['lat']) / 1e7 * 111195,
                      (a['lon'] - b['lon']) / 1e7 * 111195 * math.cos(math.radians(a['lat'] / 1e7)))




def run(args, scenario):
    root = Path(__file__).resolve().parents[2]
    ap, moos, build, behavior = [Path(x).resolve() for x in [args.ardupilot_root, args.moos_root, args.build, args.behavior_dir]]
    firmware = fixture.native_firmware_profile(root, ap)
    selector = selected_fixture = fixture_file = None
    fixture_inputs_before = {}
    if scenario == 'mission_read':
        selector = source_fixture_selector(root)
        selected_fixture = selector.select(root, firmware, 'armed_wiped_mission_read')
        protected_fixture = selector.protected_paths(selected_fixture) + [Path(__file__).resolve(),
            root / 'tests/concurrent/ConcurrentMoosProbe.cpp', root / 'tests/concurrent/mission_read_evidence.py',
            root / 'tests/firmware/MissionFixtureShape.h']
        fixture_inputs_before = {str(path): fixture.sha(path) for path in protected_fixture}
    out = Path(args.output).resolve() / scenario; out.mkdir(parents=True, exist_ok=False)
    (out / 'logs').mkdir(); control = out / 'control'; control.mkdir()
    if selected_fixture:
        fixture_file = selector.copy_to_evidence(selected_fixture, out)
    probe_binary = Path(args.probe).resolve() if args.probe else build / 'bin/concurrent_moos_probe'
    native = ap / 'build/sitl/bin/arducopter'
    discovery = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); discovery.bind(('127.0.0.1', 0)); discovery.settimeout(.2)
    bridge_port = discovery.getsockname()[1]
    monitor = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); monitor.bind(('127.0.0.1', 0)); monitor.setblocking(False)
    monitor_port = monitor.getsockname()[1]
    db, base, rc = fixture.port(socket.SOCK_STREAM), fixture.port(socket.SOCK_STREAM), fixture.port(socket.SOCK_DGRAM)
    defaults = out / 'fixture.parm'; defaults.write_text('SERIAL1_PROTOCOL 2\nMAV2_POSITION 10\nMAV2_EXT_STAT 2\nMAV2_EXTRA3 2\nLOG_DISARMED 1\n')
    (out / 'message-intervals-chan1.txt').write_text('33 100\n245 500\n')
    command = [str(native), '--model', 'quad', '--home', '42.358456,-71.087589,10,0', '--speedup', '1', '--wipe',
               '--defaults', str(ap / 'Tools/autotest/default_params/copter.parm') + ',' + str(defaults),
               '--serial0', f'udpclient:127.0.0.1:{bridge_port}', '--serial1', f'udpclient:127.0.0.1:{monitor_port}',
               '--serial2', 'none', '--serial5', 'none', '--serial6', 'none', '--serial7', 'none', '--serial8', 'none',
               '--base-port', str(base), '--rc-in-port', str(rc)]
    tick, budget = (30, 32) if scenario == 'mission_default' else (50, 128)
    result = {'scenario': scenario, 'result': 'FAIL', 'layer': 'native Copter + stock pHelmIvP + concurrent protocol/navigation modules',
              'hardware': False, 'firmware_identity': firmware, 'profile': {'AppTick': tick, 'CommsTick': tick, 'max_datagrams_per_iter': budget, 'max_queue_age': .25, 'list_mode': 'paced', 'list_timeout': 60},
              'command': command, 'app_sha256': fixture.sha(build / 'bin/iMAVLink'), 'native_sha256': fixture.sha(native),
              'probe_sha256': fixture.sha(probe_binary), 'harness_sha256': fixture.sha(Path(__file__).resolve()),
              'probe_source_sha256': fixture.sha(Path(__file__).with_name('ConcurrentMoosProbe.cpp')),
              'modules_sha256': {p.name: fixture.sha(p) for p in sorted((build / 'plugins').glob('mavmoos_*')) if p.is_file()},
              'moos_revision': subprocess.check_output(['git', '-C', str(moos), 'rev-parse', 'HEAD'], text=True).strip(),
              'ardupilot_revision': subprocess.check_output(['git', '-C', str(ap), 'rev-parse', 'HEAD'], text=True).strip(),
              'ports': {'bridge': bridge_port, 'monitor': monitor_port, 'db': db, 'base': base, 'rc': rc}, 'events': []}
    if scenario == 'mission_read':
        result['profile']['read_only_services'] = {'mission': 'download_declared_wiped_mission_after_arm', 'parameter': 'read_SIM_WIND_SPD', 'operation_timeout_s': 15, 'post_read_observation_s': 2}
        result.update(scenario_fixture=selected_fixture, scenario_fixture_file=fixture_file,
                      fixture_inputs_before=fixture_inputs_before)
    processes = []; handles = []; stopped = set(); packets = []; mission = out / 'targ_copter.moos'; launch = probe = None
    def event(kind, **values): result['events'].append({'mono': time.monotonic(), 'kind': kind, **values})
    def spawn(cmd, name, env=None):
        log = (out / name).open('w'); handles.append(log)
        process = subprocess.Popen(cmd, cwd=out, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process); return process
    try:
        sitl = spawn(command, 'sitl.log'); result['sitl_pid'] = sitl.pid
        decoder = mavutil.mavlink.MAVLink(None); decoder.robust_parsing = True
        peer = None; until = time.monotonic() + 20
        while time.monotonic() < until:
            if sitl.poll() is not None: raise RuntimeError('native simulator exited at discovery')
            try: data, address = discovery.recvfrom(4096)
            except socket.timeout: continue
            if any(m.get_type() == 'HEARTBEAT' and m.get_srcSystem() == 1 and m.get_srcComponent() == 1 for m in decoder.parse_buffer(data) or []): peer = address; break
        if peer is None or peer[0] != '127.0.0.1': raise RuntimeError('owned native simulator peer not discovered')
        result['ports']['peer'] = peer[1]; discovery.close()
        env = dict(os.environ); env['PATH'] = os.pathsep.join([str(build / 'bin'), str(moos / 'bin'), env.get('PATH', '')]); env['IVP_BEHAVIOR_DIRS'] = str(behavior)
        macros = {'MOOS_PORT': db, 'APP_PATH': build / 'bin/iMAVLink', 'BIND_PORT': bridge_port, 'PEER_PORT': peer[1],
                  'LAT_ORIGIN': '42.358456', 'LONG_ORIGIN': '-71.087589', 'SIMULATION_ONLY': 'true', 'SHADOW_MODE': 'false',
                  'SITL_PID': sitl.pid, 'PLUGIN_DIRECTORY': build / 'plugins', 'BEHAVIOR_DIRECTORY': behavior,
                  'HEIGHT': 5, 'SPEED': 1, 'WAYPOINTS': '0,100:100,100:100,0:0,0'}
        for suffix in ['moos', 'bhv']:
            subprocess.run([str(moos / 'bin/nsplug'), str(root / f'missions/copter_sitl/meta_vehicle.{suffix}'), str(out / f'targ_copter.{suffix}'), '--strict', '--force', '-x'] + [f'{k}={v}' for k, v in macros.items()], check=True, env=env, capture_output=True)
        text = mission.read_text().replace('  AppTick = 30', f'  AppTick = {tick}', 1).replace('  CommsTick = 30', f'  CommsTick = {tick}', 1)
        text = text.replace('  nav_position_timeout = 0.5', f'  nav_position_timeout = 0.5\n  max_datagrams_per_iter = {budget}\n  parameter_list_mode = paced\n  parameter_list_timeout = 60', 1)
        if scenario == 'mission_read':
            text = text.replace('  nav_position_timeout = 0.5', '  nav_position_timeout = 0.5\n  plugins = telemetry,arducopter_control,parameters,missions\n  mission_timeout = 15', 1)
        mission.write_text(text); result['mission_sha256'] = fixture.sha(mission); result['behavior_sha256'] = fixture.sha(out / 'targ_copter.bhv')
        launch = spawn([str(moos / 'bin/pAntler'), str(mission)], 'pantler.log', env)
        probe_command = [str(probe_binary), str(db), scenario, str(control)]
        if selected_fixture:
            probe_command.append(selected_fixture['expectation']['shape'])
        probe = spawn(probe_command, 'probe.jsonl', env)
        decoder = mavutil.mavlink.MAVLink(None); decoder.robust_parsing = True
        injected = restored = False; target = None; markers = set(); drain = None
        with (out / 'fc-observer.jsonl').open('w') as trace, (out / 'fc-observer.mavlink').open('wb') as raw:
            until = time.monotonic() + 190
            while time.monotonic() < until:
                if probe.poll() is not None:
                    if drain is None: drain = time.monotonic() + 1.5
                    if time.monotonic() >= drain: break
                for marker in ['transfer.started', 'reads.completed', 'transfer.finished']:
                    if marker not in markers and (control / marker).exists(): event(marker); markers.add(marker)
                if scenario == 'bridge_pause' and not injected and (control / 'inject.request').exists():
                    state = json.loads((control / 'list-state.json').read_text())
                    if not state['pending'] or not 0 < state['seen_indices'] < state['expected']: raise RuntimeError('list no longer demonstrably pending at interruption boundary')
                    owned = [p for p in owned_records(launch) if Path(p['command'].split()[0]).name == 'iMAVLink']
                    if len(owned) != 1: raise RuntimeError('exact owned bridge process not found')
                    target = owned[0]['pid']; os.kill(target, signal.SIGSTOP); stopped.add(target)
                    injected_at = time.monotonic(); event('bridge_stopped', pid=target, pending_state=state)
                    (control / 'injected').write_text(str(injected_at)); injected = True
                if injected and not restored and time.monotonic() - injected_at >= .8:
                    os.kill(target, signal.SIGCONT); stopped.discard(target); event('bridge_resumed', pid=target)
                    (control / 'restored').write_text(str(time.monotonic())); restored = True
                try: data, address = monitor.recvfrom(65535)
                except BlockingIOError: time.sleep(.005); continue
                raw.write(data)
                for m in decoder.parse_buffer(data) or []:
                    if m.get_srcSystem() == 1 and m.get_srcComponent() == 1 and m.get_type() in ['HEARTBEAT', 'GLOBAL_POSITION_INT', 'EXTENDED_SYS_STATE', 'STATUSTEXT']:
                        row = {'mono': time.monotonic(), 'endpoint': list(address), **m.to_dict()}; packets.append(row); trace.write(json.dumps(row) + '\n')
            if probe.poll() is None: raise RuntimeError('concurrent fixture exceeded190s')
        result['probe_exit'] = probe.returncode
        if probe.returncode != 0: raise RuntimeError('concurrent probe failed')
        expected_modules = {'telemetry', 'parameters', 'arducopter_control'} | ({'missions'} if scenario == 'mission_read' else set())
        result['capabilities_catalog'] = observed_catalog(out / 'probe.jsonl', expected_modules, native=True, helm=True, mode="SITL_CONTROL")
        if scenario == 'mission_read':
            provider = next(m for m in result['capabilities_catalog']['composition']['modules'] if m['name'] == 'missions')
            if (provider['control_provider'] or not mission_capabilities_match(provider, result['capabilities_catalog']['composition']['plugin_abi'])
                    or provider['request_owners'] != ['MAVMOOS_MISSION_REQUEST']):
                raise RuntimeError('mission provider composition differs from bounded data-only profile')
        result['loaded_modules_sha256'] = loaded_module_hashes(build / 'plugins', result['capabilities_catalog'])
        if scenario == 'bridge_pause' and not (injected and restored): raise RuntimeError('pending-list bridge interruption missing')
        events = []
        for line in (out / 'probe.jsonl').read_text().splitlines():
            try: events.append(json.loads(line))
            except json.JSONDecodeError: pass
        result['assertions'] = [e for e in events if e.get('type') == 'assertion']
        transfer = next(e['value'] for e in result['assertions'] if e['key'] == 'transfer_outcome')
        transfer_fields = dict(part.split('=', 1) for part in transfer.split(','))
        result['capability_outcome'] = {'table_status': transfer_fields['status'],
            'navigation_continuity': 'LATCHED_REQUIRES_EXPLICIT_RECOVERY' if transfer_fields['loss_seen'] == 'true' else 'CONTINUOUS',
            'acceptance_kind': 'CHARACTERIZATION_NOT_DEFAULT_TRANSFER_SUPPORT' if scenario == 'mission_default' else 'NOMINAL_CONCURRENT_TRANSFER' if scenario == 'bulk' else 'INTERRUPTED_SESSION_RESET_AND_EXPLICIT_RECOVERY' if transfer_fields['status'] == 'CANCELLED' else 'INTERRUPTION_AND_SCOPED_RECOVERY'}
        if scenario == 'mission_read':
            expected_fixture = selector.reselect((out / fixture_file['path']).read_bytes(), firmware,
                                                'armed_wiped_mission_read', selected_fixture)
            result['mission_read'] = mission_read_evidence(events, expected_fixture)
            result['capability_outcome'] = {'mission_status': 'COMPLETED', 'parameter_read_status': 'COMPLETED',
                'navigation_continuity': 'CONTINUOUS', 'acceptance_kind': 'MISSION_AND_PARAMETER_READ_WITH_CONTINUOUS_NAVIGATION'}
        snapshots = [json.loads(e['value']) for e in events if e.get('type') == 'snapshot']
        result['parameter_results'] = [fields for e in events if e.get('type') == 'parameter_result' for fields in [dict(part.split('=', 1) for part in e['value'].split(','))]]
        if scenario == 'bridge_pause' and transfer_fields['status'] == 'CANCELLED':
            terminal = next(r for r in result['parameter_results'] if r['id'] == 'concurrent-list' and r['status'] == 'CANCELLED')
            result['primary_heartbeat_reset_audit'] = audit_primary_heartbeat_reset(out, terminal)
        if snapshots:
            if len(snapshots) != 1: raise RuntimeError('multiple list snapshots')
            snapshot = snapshots[0]; rows = snapshot['values']; count = snapshot['count']
            if not 100 <= count <= 4096 or len(rows) != count or sorted(r['index'] for r in rows) != list(range(count)) or len({r['name'] for r in rows}) != count or any(not math.isfinite(r['value']) for r in rows): raise RuntimeError('complete snapshot does not contain every distinct valid index/name/value')
            (out / 'complete-snapshot.json').write_text(json.dumps(snapshot, indent=2) + '\n')
            result['complete_snapshot'] = {'count': count, 'distinct_indices': len(rows), 'bytes': len(json.dumps(snapshot, separators=(',', ':')))}
        elif scenario == 'bulk': raise RuntimeError('bulk complete snapshot missing')
        hearts = [m for m in packets if m['mavpackettype'] == 'HEARTBEAT']; poses = [m for m in packets if m['mavpackettype'] == 'GLOBAL_POSITION_INT']; landed = [m for m in packets if m['mavpackettype'] == 'EXTENDED_SYS_STATE']
        armed_at = min([h['mono'] for h in hearts if h['base_mode'] & 128], default=None)
        flight_poses = [p for p in poses if armed_at is not None and p['mono'] >= armed_at]
        home = {'lat': 423584560, 'lon': -710875890}
        result['observer'] = {'writes': 0, 'heartbeat_samples': len(hearts), 'position_samples': len(poses), 'flight_position_samples': len(flight_poses), 'flight_window_start_python_mono': armed_at, 'prearm_position_samples_excluded': len(poses) - len(flight_poses), 'displacement_reference': {'basis': 'CONFIGURED_FIXTURE_HOME', 'latitude': 42.358456, 'longitude': -71.087589}, 'maximum_relative_height_m': max([m['relative_alt'] / 1000 for m in flight_poses], default=0), 'maximum_displacement_m': max([distance(home, p) for p in flight_poses], default=0), 'final_disarmed': bool(hearts) and not bool(hearts[-1]['base_mode'] & 128), 'final_on_ground': bool(landed) and landed[-1]['landed_state'] == 1, 'observed_modes': sorted({m['custom_mode'] for m in hearts})}
        if not result['observer']['final_disarmed'] or not result['observer']['final_on_ground'] or result['observer']['maximum_relative_height_m'] < 4 or result['observer']['maximum_displacement_m'] < 2: raise RuntimeError('independent FC observer did not corroborate flight/movement/landing')
        a = next(e['mono'] for e in result['events'] if e['kind'] == 'transfer.started'); b = next(e['mono'] for e in result['events'] if e['kind'] == 'transfer.finished')
        concurrent_poses = [p for p in flight_poses if a <= p['mono'] <= b]
        transfer_travel = sum(distance(x, y) for x, y in zip(concurrent_poses, concurrent_poses[1:]))
        transfer_displacement = max([distance(concurrent_poses[0], p) for p in concurrent_poses], default=0) if concurrent_poses else 0
        result['transfer_observer'] = {'coordinator_observed_window_s': b - a, 'position_samples': len(concurrent_poses), 'minimum_height_m': min([p['relative_alt'] / 1000 for p in concurrent_poses], default=None), 'maximum_height_m': max([p['relative_alt'] / 1000 for p in concurrent_poses], default=None), 'horizontal_path_distance_m': transfer_travel, 'maximum_horizontal_displacement_m': transfer_displacement, 'maximum_reported_groundspeed_mps': max([math.hypot(p['vx'], p['vy']) / 100 for p in concurrent_poses], default=0), 'note': 'Python coordinator marker-observation window; not subtracted from C++ steady-clock timestamps or exact packet-send timing'}
        if len(concurrent_poses) < 2 or result['transfer_observer']['minimum_height_m'] < 4: raise RuntimeError('insufficient independent airborne samples during transfer')
        if scenario in ('bulk', 'mission_read') and transfer_displacement < .05: raise RuntimeError('concurrent reads lacked independent horizontal motion during their observation window')
        if scenario == 'mission_read':
            completed_at = next(e['mono'] for e in result['events'] if e['kind'] == 'reads.completed')
            if not a <= completed_at <= b:
                raise RuntimeError('read completion marker fell outside explicit continuity observation')
            result['transfer_observer']['coordinator_observed_reads_completed_after_s'] = completed_at - a
            result['transfer_observer']['note'] += '; mission_read window explicitly includes post-completion navigation; exact short service completion is recorded separately in C++.'
        result['result'] = 'PASS'
    except Exception as error: result['error'] = str(error)
    finally:
        for pid in stopped:
            try: os.kill(pid, signal.SIGCONT)
            except ProcessLookupError: pass
        remaining = []
        for process in reversed(processes): remaining.extend(fixture.stop(process))
        discovery.close(); monitor.close()
        for h in handles: h.close()
        result['owned_processes_remaining'] = remaining; result['ports_clear'] = {}
        for name in ['bridge', 'monitor', 'peer', 'rc']:
            if name not in result['ports']: continue
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                try: s.bind(('127.0.0.1', result['ports'][name])); result['ports_clear'][name] = True
                except OSError: result['ports_clear'][name] = False
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(.2); result['ports_clear']['moos_tcp'] = s.connect_ex(('127.0.0.1', db)) != 0
        wind = []; boot = []
        try:
            for logfile in sorted((out / 'logs').glob('*.BIN')):
                reader = DFReader.DFReader_binary(str(logfile))
                while True:
                    m = reader.recv_match(type=['PARM', 'MSG'])
                    if m is None: break
                    if m.get_type() == 'PARM' and str(m.Name) == 'SIM_WIND_SPD': wind.append(float(m.Value))
                    if m.get_type() == 'MSG' and ('ArduCopter' in m.Message or 'Frame:' in m.Message): boot.append(m.Message)
        except Exception as error:
            result['dataflash_error'] = str(error)
            if result['result'] == 'PASS': result['result'] = 'FAIL'; result['error'] = 'DataFlash evidence could not be decoded'
        result['independent_dataflash'] = {'SIM_WIND_SPD_values': wind, 'boot': boot}
        if result['result'] == 'PASS' and (not wind or any(v != wind[0] for v in wind) or .25 in wind): result['result'] = 'FAIL'; result['error'] = 'DataFlash did not corroborate unchanged simulator wind'
        if remaining or not all(result['ports_clear'].values()): result['result'] = 'FAIL'; result['error'] = 'owned process/port cleanup incomplete'
        if selected_fixture:
            result['fixture_inputs_after'] = {path: fixture.sha(path) for path in fixture_inputs_before}
            result['fixture_input_drift'] = [path for path, pin in fixture_inputs_before.items()
                                             if result['fixture_inputs_after'][path] != pin]
            result['scenario_fixture_file']['observed_sha256'] = fixture.sha(out / fixture_file['path'])
            if result['fixture_input_drift'] or result['scenario_fixture_file']['observed_sha256'] != fixture_file['sha256']:
                result['result'] = 'FAIL'; result['fixture_error'] = 'source-bound scenario fixture changed'
        (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: result.get(k) for k in ['scenario', 'result', 'error', 'profile', 'capability_outcome', 'observer', 'transfer_observer', 'ports_clear', 'owned_processes_remaining']}, indent=2), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser()
    for key in ['ardupilot-root', 'moos-root', 'build', 'behavior-dir', 'output']: parser.add_argument('--' + key, required=True)
    parser.add_argument('--probe'); parser.add_argument('--scenario', choices=['all'] + SCENARIOS, default='all'); args = parser.parse_args()
    with (Path(tempfile.gettempdir()) / 'mavmoos-native-sitl.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        outcomes = [run(args, s) for s in SCENARIOS if args.scenario in ['all', s]]
    summary = {'result': 'PASS' if all(r['result'] == 'PASS' for r in outcomes) else 'FAIL', 'scenarios': [{'scenario': r['scenario'], 'result': r['result'], 'error': r.get('error')} for r in outcomes]}
    (Path(args.output).resolve() / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n'); return 0 if summary['result'] == 'PASS' else 1


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(143)))
    raise SystemExit(main())
