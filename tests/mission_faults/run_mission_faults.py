#!/usr/bin/env python3
"""Owned disarmed native partial-upload cancellation/reset fixture.

The discovery20s and active150s supervisor bounds cover home initialization,
an explicit >8s FC protocol-settlement wait and bounded readback/clear at warp1.
Scoped shutdown follows. No execution API.
"""
import argparse
import fcntl
import hashlib
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

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tests'))
from owned_processes import owned_records, stop_process_tree
from control.catalog_evidence import loaded_module_hashes
from mission_fault_evidence import validate


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def port(kind):
    with socket.socket(socket.AF_INET, kind) as stream:
        stream.bind(('127.0.0.1', 0))
        return stream.getsockname()[1]


def sockets(process):
    if process.poll() is not None:
        raise RuntimeError('owned native process exited')
    result = subprocess.run(['lsof', '-nP', '-a', '-p', str(process.pid), '-iUDP'], capture_output=True, text=True, timeout=3)
    if result.returncode:
        raise RuntimeError('owned native UDP inventory unavailable')
    return result.stdout


def exact_loopback(inventory, source, destination):
    return bool(re.search(r'UDP\s+127\.0\.0\.1:' + str(source) + r'->127\.0\.0\.1:' + str(destination) + r'\b', inventory))


def app_record(antler, mission):
    result = []
    for record in owned_records(antler):
        try:
            tokens = shlex.split(record['command'])
        except ValueError:
            continue
        if tokens and Path(tokens[0]).name == 'iMAVLink' and str(mission) in tokens:
            result.append(record)
    if len(result) != 1:
        raise RuntimeError('exact owned mission iMAVLink process not unique')
    return result[0]


def run(args):
    from pymavlink import mavutil, DFReader
    ap, moos, build, out = [Path(getattr(args, key)).resolve() for key in ('ardupilot_root', 'moos_root', 'build', 'output')]
    source = Path(args.source_root).resolve() if args.source_root else ROOT
    from owned_processes import native_firmware_profile
    firmware = native_firmware_profile(source, ap)
    probe_path = Path(args.probe).resolve() if args.probe else build / 'bin/mission_fault_probe'
    out.mkdir(parents=True, exist_ok=False)
    control = out / 'control'; control.mkdir()
    native, app, module = ap / 'build/sitl/bin/arducopter', build / 'bin/iMAVLink', build / 'plugins/mavmoos_missions.so'
    for path in (native, app, module, probe_path, moos / 'bin/pAntler'):
        if not path.is_file():
            raise RuntimeError('required artifact missing: ' + str(path))
    discovery = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); discovery.bind(('127.0.0.1', 0)); discovery.settimeout(.2)
    monitor = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); monitor.bind(('127.0.0.1', 0)); monitor.setblocking(False)
    bridge_port, monitor_port = discovery.getsockname()[1], monitor.getsockname()[1]
    db, base, rc = port(socket.SOCK_STREAM), port(socket.SOCK_STREAM), port(socket.SOCK_DGRAM)
    defaults = out / 'fixture.parm'
    defaults.write_text('LOG_DISARMED 1\nSERIAL1_PROTOCOL 2\nMAV2_POSITION 2\nMAV2_EXT_STAT 2\nMAV2_EXTRA3 2\n')
    (out / 'message-intervals-chan1.txt').write_text('33 500\n245 500\n')
    command = [str(native), '--model', 'quad', '--home', '42.358456,-71.087589,10,0', '--speedup', '1', '--wipe',
               '--defaults', str(ap / 'Tools/autotest/default_params/copter.parm') + ',' + str(defaults),
               '--serial0', f'udpclient:127.0.0.1:{bridge_port}', '--serial1', f'udpclient:127.0.0.1:{monitor_port}',
               '--serial2', 'none', '--serial5', 'none', '--serial6', 'none', '--serial7', 'none', '--serial8', 'none',
               '--base-port', str(base), '--rc-in-port', str(rc)]
    protected = [app, module, native, probe_path, Path(__file__).resolve(), Path(__file__).with_name('MissionFaultProbe.cpp'),
                 Path(__file__).with_name('mission_fault_evidence.py'), source / 'tests/mission_faults/MissionFaultProbe.cpp',
                 source / 'tests/missions/mission_evidence.py']
    protected += [source / name for name in ('src/plugins/MissionPlugin.cpp', 'src/libmavmoos_runtime/Plugin.h',
                    'src/libmavmoos_runtime/Runtime.cpp', 'src/libmavmoos_runtime/RuntimeMission.cpp')]
    before = {str(path): sha(path) for path in protected}
    result = {'result': 'FAIL', 'scenario': args.scenario, 'scope': 'owned disarmed native Copter; mission-data cancellation only',
              'hardware': False, 'firmware_identity': firmware, 'timewarp': 1, 'mission_execution_requested': False, 'observer_writes': 0,
              'command': command, 'ports': {'bridge': bridge_port, 'monitor': monitor_port, 'db': db, 'base': base, 'rc': rc},
              'source_root': str(source), 'source_ref': subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip(),
              'ardupilot_ref': subprocess.check_output(['git', '-C', str(ap), 'rev-parse', 'HEAD'], text=True).strip(),
              'moos_ref': subprocess.check_output(['git', '-C', str(moos), 'rev-parse', 'HEAD'], text=True).strip(),
              'artifacts_before': before, 'app_sha256': before[str(app)], 'module_sha256': before[str(module)],
              'native_sha256': before[str(native)], 'probe_sha256': before[str(probe_path)], 'events': []}
    processes, streams, packets, stopped = [], [], [], None
    antler = sitl = probe = None
    def launch(argv, log, env=None):
        stream = (out / log).open('w'); streams.append(stream)
        process = subprocess.Popen(argv, cwd=out, env=env, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process); return process
    try:
        sitl = launch(command, 'sitl.log'); result['sitl_pid'] = sitl.pid
        decoder = mavutil.mavlink.MAVLink(None); decoder.robust_parsing = True
        deadline, peer = time.monotonic() + 20, None
        while peer is None and time.monotonic() < deadline:
            if sitl.poll() is not None:
                raise RuntimeError('native process exited during startup')
            try:
                data, sender = discovery.recvfrom(4096)
            except socket.timeout:
                continue
            if any(m.get_type() == 'HEARTBEAT' and m.get_srcSystem() == 1 and m.get_srcComponent() == 1 and
                   m.type == 2 and m.autopilot == 3 for m in decoder.parse_buffer(data) or []):
                peer = sender
        inventory = sockets(sitl); result['initial_socket_inventory'] = inventory
        if peer is None or peer[0] != '127.0.0.1' or not exact_loopback(inventory, peer[1], bridge_port):
            raise RuntimeError('exact owned primary native endpoint not proven')
        result['ports']['peer'] = peer[1]; discovery.close()
        mission = out / 'mission_fault.moos'
        mission.write_text(f'''ServerHost = localhost
ServerPort = {db}
Community = mission_fault
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
  AppTick = 50
  CommsTick = 50
  bind_address = 127.0.0.1
  bind_port = {bridge_port}
  peer_address = 127.0.0.1
  peer_port = {peer[1]}
  target_system = 1
  target_component = 1
  expected_type = 2
  expected_autopilot = 3
  simulation_only = true
  sitl_pid = {sitl.pid}
  plugin_directory = {build / 'plugins'}
  plugins = missions
  max_datagrams_per_iter = 128
  mission_timeout = 30
}}
''')
        env = dict(os.environ); env['PATH'] = os.pathsep.join([str(build / 'bin'), str(moos / 'bin'), env.get('PATH', '')])
        antler = launch([str(moos / 'bin/pAntler'), str(mission)], 'pantler.log', env)
        probe = launch([str(probe_path), str(db), args.scenario, str(control)], 'probe.jsonl', env)
        decoder = mavutil.mavlink.MAVLink(None); decoder.robust_parsing = True
        deadline, drain, monitor_peer, injected_at = time.monotonic() + 150, None, None, None
        with (out / 'observer.jsonl').open('w') as trace, (out / 'observer.mavlink').open('wb') as raw:
            while time.monotonic() < deadline:
                if sitl.poll() is not None or antler.poll() is not None:
                    raise RuntimeError('owned native or MOOS launcher exited')
                if probe.poll() is not None:
                    if drain is None:
                        drain = time.monotonic() + 1.2
                    if time.monotonic() >= drain:
                        break
                if args.scenario == 'reset' and injected_at is None and (control / 'pause.request').is_file():
                    record = app_record(antler, mission); stopped = record
                    os.kill(record['pid'], signal.SIGSTOP); injected_at = time.monotonic()
                    result['events'].append({'kind': 'bridge_paused', 'mono': injected_at, 'owned_process': record})
                if stopped is not None and time.monotonic() - injected_at >= 2.6:
                    current = app_record(antler, mission)
                    if current['pid'] != stopped['pid'] or current['command'] != stopped['command']:
                        raise RuntimeError('paused owned app identity changed')
                    os.kill(stopped['pid'], signal.SIGCONT); stopped = None
                    result['events'].append({'kind': 'bridge_resumed', 'mono': time.monotonic(), 'pid': current['pid']})
                try:
                    data, sender = monitor.recvfrom(65535)
                except BlockingIOError:
                    time.sleep(.005); continue
                if monitor_peer is None:
                    if sender[0] != '127.0.0.1' or not exact_loopback(sockets(sitl), sender[1], monitor_port):
                        raise RuntimeError('independent observer source not owned by native process')
                    monitor_peer = sender; result['monitor_peer'] = list(sender)
                if sender != monitor_peer:
                    raise RuntimeError('observer received a different endpoint')
                raw.write(data)
                for message in decoder.parse_buffer(data) or []:
                    if message.get_srcSystem() == 1 and message.get_srcComponent() == 1:
                        row = {'received_mono': time.monotonic(), **message.to_dict()}; packets.append(row)
                        trace.write(json.dumps(row) + '\n')
            result['monitor_finished_mono'] = time.monotonic()
        if probe.poll() is None:
            raise RuntimeError('fault probe exceeded150s wrapper bound')
        result['probe_exit'] = probe.returncode
        if probe.returncode:
            raise RuntimeError('mission fault probe failed')
        events = [json.loads(line) for line in (out / 'probe.jsonl').read_text().splitlines() if line.startswith('{')]
        result['provider_evidence'] = validate(events, args.scenario)
        result['loaded_modules_sha256'] = loaded_module_hashes(build / 'plugins', result['provider_evidence']['catalog'])
        hearts = [p for p in packets if p['mavpackettype'] == 'HEARTBEAT']; landed = [p for p in packets if p['mavpackettype'] == 'EXTENDED_SYS_STATE']
        end = result['monitor_finished_mono']
        if not hearts or any(p['base_mode'] & 128 for p in hearts) or not landed or landed[-1]['landed_state'] != 1:
            raise RuntimeError('independent FC state missing or observed arming/not ground')
        if not 0 <= end - hearts[-1]['received_mono'] <= 2 or not 0 <= end - landed[-1]['received_mono'] <= 2:
            raise RuntimeError('independent final ground/disarmed state stale at monitor end')
        if args.scenario == 'reset' and ([e['kind'] for e in result['events']] != ['bridge_paused', 'bridge_resumed'] or
                result['events'][1]['mono'] - result['events'][0]['mono'] < 2.6):
            raise RuntimeError('bounded bridge reset injection not performed')
        result['observer'] = {'ever_armed': False, 'final_on_ground': True, 'heartbeat_samples': len(hearts), 'landed_samples': len(landed),
                              'observed_modes': sorted({p['custom_mode'] for p in hearts}),
                              'final_heartbeat_age_s': end - hearts[-1]['received_mono'], 'final_landed_age_s': end - landed[-1]['received_mono']}
        result['result'] = 'PASS'
    except BaseException as error:
        result['error'] = str(error) or type(error).__name__
    finally:
        if stopped is not None:
            try:
                records = owned_records(antler)
                if any(r['pid'] == stopped['pid'] and r['command'] == stopped['command'] for r in records):
                    os.kill(stopped['pid'], signal.SIGCONT)
            except (ProcessLookupError, RuntimeError):
                pass
        remaining = []
        for process in reversed(processes):
            remaining.extend(stop_process_tree(process))
        discovery.close(); monitor.close()
        for stream in streams:
            stream.close()
        cleared = {}
        for key in ('bridge', 'monitor', 'peer', 'rc'):
            if key not in result['ports']:
                continue
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as stream:
                try:
                    stream.bind(('127.0.0.1', result['ports'][key])); cleared[key] = True
                except OSError:
                    cleared[key] = False
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as stream:
            stream.settimeout(.2); cleared['db'] = stream.connect_ex(('127.0.0.1', db)) != 0
        result['ports_clear'], result['owned_processes_remaining'] = cleared, remaining
        if not all(cleared.values()) or remaining:
            result['result'] = 'FAIL'; result['cleanup_error'] = 'owned resources remain'
        try:
            arming = []; logs = []
            for path in sorted((out / 'logs').glob('*.BIN')):
                digest = sha(path); reader = DFReader.DFReader_binary(str(path))
                while True:
                    row = reader.recv_match(type=['ARM'])
                    if row is None:
                        break
                    arming.append({'log': path.name, **row.to_dict()})
                if sha(path) != digest:
                    raise RuntimeError('DataFlash changed during readback')
                logs.append({'path': str(path.relative_to(out)), 'sha256': digest})
            if not logs:
                raise RuntimeError('DataFlash evidence unavailable')
            if any(int(row['ArmState']) != 0 for row in arming):
                raise RuntimeError('DataFlash observed arming')
            result['dataflash'] = {'logs': logs, 'arming_records': arming, 'partial_prefix_claim': 'FC mission download, not assumed CMD logging of an incomplete upload'}
            (out / 'dataflash-arming.json').write_text(json.dumps(arming, indent=2) + '\n')
        except Exception as error:
            result['result'] = 'FAIL'; result['dataflash_error'] = str(error)
        after = {str(path): sha(path) for path in protected}
        result['artifacts_after'] = after; result['artifact_drift'] = [name for name in before if before[name] != after[name]]
        if result['artifact_drift']:
            result['result'] = 'FAIL'; result['artifact_error'] = 'source or artifact changed during run'
        result['files'] = {str(path.relative_to(out)): sha(path) for path in sorted(out.rglob('*')) if path.is_file() and path.name != 'result.json'}
        (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({key: result.get(key) for key in ('result', 'scenario', 'error', 'probe_exit', 'provider_evidence', 'observer', 'ports_clear', 'owned_processes_remaining', 'artifact_drift')}, indent=2))
    return 0 if result['result'] == 'PASS' else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ('ardupilot-root', 'moos-root', 'build', 'output'):
        parser.add_argument('--' + key, required=True)
    parser.add_argument('--scenario', choices=['cancel', 'reset'], required=True)
    parser.add_argument('--source-root')
    parser.add_argument('--probe')
    args = parser.parse_args()
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(143)))
    with (Path(tempfile.gettempdir()) / 'mavmoos-native-sitl.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        raise SystemExit(run(args))
