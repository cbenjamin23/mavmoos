#!/usr/bin/env python3
"""Owned disarmed native-Copter mission-data bench; no motion or execution API.

Normal runs are short. The explicit 120s supervisor bound includes FC startup/ground
readiness plus five independently bounded protocol/readback phases and retries.
All native fixtures share the existing lock. Use a new output directory.
"""
import argparse
import fcntl
import hashlib
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
from run_sitl import port, sha, stop
from pymavlink import mavutil, DFReader
from mission_evidence import validate_snapshots, observed_catalog



def run(args):
    ap, moos, build, out = [Path(getattr(args, key)).resolve() for key in ('ardupilot_root', 'moos_root', 'build', 'output')]
    from owned_processes import native_firmware_profile
    firmware = native_firmware_profile(Path(__file__).resolve().parents[2], ap)
    plugin_dir = Path(args.plugin_dir).resolve() if args.plugin_dir else build / 'plugins'
    probe_binary = Path(args.probe).resolve() if args.probe else build / 'bin/mission_moos_probe'
    out.mkdir(parents=True, exist_ok=False)
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(('127.0.0.1', 0)); receiver.settimeout(.2)
    monitor = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    monitor.bind(('127.0.0.1', 0)); monitor.setblocking(False)
    bind, observe = receiver.getsockname()[1], monitor.getsockname()[1]
    db, base, rc = port(socket.SOCK_STREAM), port(socket.SOCK_STREAM), port(socket.SOCK_DGRAM)
    defaults = out / 'fixture.parm'
    defaults.write_text('LOG_DISARMED 1\nSERIAL1_PROTOCOL 2\nMAV2_POSITION 2\nMAV2_EXT_STAT 2\nMAV2_EXTRA3 2\n')
    (out / 'message-intervals-chan1.txt').write_text('33 500\n245 500\n')
    native = ap / 'build/sitl/bin/arducopter'
    command = [str(native), '--model', 'quad', '--home', '42.358456,-71.087589,10,0', '--speedup', '1', '--wipe',
               '--defaults', str(ap / 'Tools/autotest/default_params/copter.parm') + ',' + str(defaults),
               '--serial0', f'udpclient:127.0.0.1:{bind}', '--serial1', f'udpclient:127.0.0.1:{observe}',
               '--serial2', 'none', '--serial5', 'none', '--serial6', 'none', '--serial7', 'none', '--serial8', 'none',
               '--base-port', str(base), '--rc-in-port', str(rc)]
    result = {'result': 'FAIL', 'layer': 'mission-only MOOS provider + owned disarmed native Copter', 'hardware': False, 'firmware_identity': firmware,
              'command': command, 'app_sha256': sha(build / 'bin/iMAVLink'), 'native_sha256': sha(native),
              'module_sha256': sha(plugin_dir / 'mavmoos_missions.so'), 'probe_sha256': sha(probe_binary),
              'harness_sha256': sha(Path(__file__)),
              'ports': {'bridge': bind, 'monitor': observe, 'db': db, 'base': base, 'rc': rc},
              'observer_writes': 0, 'mission_execution_requested': False}
    processes = []; handles = []; packets = []
    try:
        stream = (out / 'sitl.log').open('w'); handles.append(stream)
        sitl = subprocess.Popen(command, cwd=out, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(sitl); result['sitl_pid'] = sitl.pid
        decoder = mavutil.mavlink.MAVLink(None); decoder.robust_parsing = True
        deadline = time.monotonic() + 20; peer = None
        while peer is None and time.monotonic() < deadline:
            if sitl.poll() is not None:
                raise RuntimeError('native simulator exited before binding')
            try:
                data, address = receiver.recvfrom(4096)
            except socket.timeout:
                continue
            if any(m.get_type() == 'HEARTBEAT' and m.get_srcSystem() == 1 and m.get_srcComponent() == 1
                   for m in decoder.parse_buffer(data) or []):
                peer = address
        if peer is None or peer[0] != '127.0.0.1':
            raise RuntimeError('owned native loopback peer not discovered')
        result['ports']['peer'] = peer[1]
        result['owned_sockets'] = subprocess.run(['lsof', '-nP', '-a', '-p', str(sitl.pid), '-iUDP'], capture_output=True, text=True).stdout
        receiver.close()
        mission = out / 'missions.moos'
        mission.write_text(f'''ServerHost = localhost
ServerPort = {db}
Community = missions
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
  bind_port = {bind}
  peer_address = 127.0.0.1
  peer_port = {peer[1]}
  target_system = 1
  target_component = 1
  expected_type = 2
  expected_autopilot = 3
  simulation_only = true
  sitl_pid = {sitl.pid}
  plugins = missions
  plugin_directory = {plugin_dir}
  max_datagrams_per_iter = 128
  mission_timeout = 20
}}
''')
        env = dict(os.environ)
        env['PATH'] = os.pathsep.join([str(build / 'bin'), str(moos / 'bin'), env.get('PATH', '')])
        stream = (out / 'pantler.log').open('w'); handles.append(stream)
        antler = subprocess.Popen([str(moos / 'bin/pAntler'), str(mission)], cwd=out, env=env,
                                  stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(antler)
        stream = (out / 'probe.jsonl').open('w'); handles.append(stream)
        probe = subprocess.Popen([str(probe_binary), str(db)], cwd=out, env=env,
                                 stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(probe)
        decoder = mavutil.mavlink.MAVLink(None); decoder.robust_parsing = True
        with (out / 'observer.jsonl').open('w') as trace, (out / 'observer.mavlink').open('wb') as raw:
            until = time.monotonic() + 120
            drain = None
            while time.monotonic() < until:
                if probe.poll() is not None:
                    if drain is None:
                        drain = time.monotonic() + 1.2
                    if time.monotonic() >= drain:
                        break
                try:
                    data, address = monitor.recvfrom(65535)
                except BlockingIOError:
                    time.sleep(.005)
                    continue
                raw.write(data)
                for message in decoder.parse_buffer(data) or []:
                    if message.get_srcSystem() == 1 and message.get_srcComponent() == 1:
                        row = {'mono': time.monotonic(), 'endpoint': list(address), **message.to_dict()}
                        packets.append(row); trace.write(json.dumps(row) + '\n')
            if probe.poll() is None:
                raise RuntimeError('mission probe exceeded120s')
        result['probe_exit'] = probe.returncode
        if probe.returncode != 0:
            raise RuntimeError('mission probe failed')
        events = [json.loads(line) for line in (out / 'probe.jsonl').read_text().splitlines() if line.startswith('{')]
        result['capabilities_catalog'] = observed_catalog(events)
        snapshots = validate_snapshots(events)
        (out / 'snapshots.json').write_text(json.dumps(snapshots, indent=2) + '\n')
        results = [dict(part.split('=', 1) for part in e['value'].split(',') if '=' in part)
                   for e in events if e.get('type') == 'result']
        for operation in ('missions-initial', 'missions-upload', 'missions-download', 'missions-clear', 'missions-empty'):
            if not any(r['id'] == operation and r['status'] == 'COMPLETED' for r in results):
                raise RuntimeError('missing operation completion')
        if not any(r['id'] == 'missions-start-denied' and r['status'] == 'REJECTED' for r in results):
            raise RuntimeError('mission-data provider accepted execution request')
        assertions = {e['key']: e['value'] for e in events if e.get('type') == 'assertion'}
        if not {'download_without_control', 'bench_ready_upload', 'bench_ready_clear', 'upload_verified', 'execution_not_exposed'} <= set(assertions):
            raise RuntimeError('missing admission/verification assertions')
        hearts = [p for p in packets if p['mavpackettype'] == 'HEARTBEAT']
        landed = [p for p in packets if p['mavpackettype'] == 'EXTENDED_SYS_STATE']
        if not hearts or any(p['base_mode'] & 128 for p in hearts):
            raise RuntimeError('independent observer missing or observed arming')
        if not landed or landed[-1]['landed_state'] != 1:
            raise RuntimeError('independent observer did not confirm final ground')
        result.update({'result': 'PASS', 'complete_snapshot_counts': {s['id']: s['count'] for s in snapshots},
                       'terminal_results': [r for r in results if r['status'] in ('COMPLETED', 'FAILED', 'REJECTED', 'TIMED_OUT', 'CANCELLED')],
                       'assertions': assertions, 'observer': {'heartbeat_samples': len(hearts), 'landed_samples': len(landed),
                       'ever_armed': False, 'final_on_ground': True, 'modes': sorted({p['custom_mode'] for p in hearts})}})
    except Exception as error:
        result['error'] = str(error)
    finally:
        for process in reversed(processes):
            stop(process)
        receiver.close(); monitor.close()
        for stream in handles:
            stream.close()
        groups = {process.pid for process in processes}
        result['owned_processes_remaining'] = []
        for line in subprocess.check_output(['ps', '-axo', 'pid,pgid,comm'], text=True).splitlines()[1:]:
            fields = line.strip().split(None, 2)
            if len(fields) == 3 and int(fields[1]) in groups:
                result['owned_processes_remaining'].append(line.strip())
        cleared = {}
        for key in ('bridge', 'monitor', 'rc', 'peer'):
            if key not in result['ports']:
                continue
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as check:
                try:
                    check.bind(('127.0.0.1', result['ports'][key])); cleared[key] = True
                except OSError:
                    cleared[key] = False
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as check:
            check.settimeout(.2); cleared['db'] = check.connect_ex(('127.0.0.1', db)) != 0
        result['ports_clear'] = cleared
        if not all(cleared.values()) or result['owned_processes_remaining']:
            result['result'] = 'FAIL'; result['cleanup_error'] = 'owned resources remain'
        cmds = []; logs = []
        try:
            for path in sorted((out / 'logs').glob('*.BIN')):
                before = sha(path); log = DFReader.DFReader_binary(str(path))
                while True:
                    message = log.recv_match(type=['CMD'])
                    if message is None:
                        break
                    cmds.append({'log': path.name, **message.to_dict()})
                if sha(path) != before:
                    raise RuntimeError('owned log changed during extraction')
                logs.append({'path': str(path.relative_to(out)), 'sha256': before})
            (out / 'dataflash-mission.json').write_text(json.dumps(cmds, indent=2) + '\n')
            result['independent_dataflash'] = {'owned_logs': logs, 'mission_command_records': len(cmds),
                'uploaded_waypoint_sequences': sorted({int(c['CNum']) for c in cmds if int(c['CTot']) == 3 and int(c['CId']) == 16})}
            if result['result'] == 'PASS' and not {1, 2} <= set(result['independent_dataflash']['uploaded_waypoint_sequences']):
                raise RuntimeError('FC log lacks uploaded mission waypoint records')
        except Exception as error:
            result['result'] = 'FAIL'; result['dataflash_error'] = str(error)
        (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    if result['result'] != 'PASS':
        for name in ('probe.jsonl', 'pantler.log', 'sitl.log'):
            if (out / name).exists():
                print(name, (out / name).read_text()[-5000:])
    return int(result['result'] != 'PASS')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    for name in ('ardupilot-root', 'moos-root', 'build', 'output'):
        parser.add_argument('--' + name, required=True)
    parser.add_argument('--plugin-dir')
    parser.add_argument('--probe')
    args = parser.parse_args()
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(143)))
    with (Path(tempfile.gettempdir()) / 'mavmoos-native-sitl.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        raise SystemExit(run(args))
