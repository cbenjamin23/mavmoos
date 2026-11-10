#!/usr/bin/env python3
"""Parameter-only MOOS/native-SITL fixture, serialized with other native SITL tests."""
import argparse
import fcntl
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
from run_sitl import port, sha, stop
from pymavlink import mavutil, DFReader


def run(args):
    ap, moos, build, out = [Path(getattr(args, k)).resolve() for k in ['ardupilot_root', 'moos_root', 'build', 'output']]
    from owned_processes import native_firmware_profile
    firmware = native_firmware_profile(Path(__file__).resolve().parents[2], ap)
    plugin_dir = Path(args.plugin_dir).resolve() if args.plugin_dir else build / 'plugins'
    probe_path = Path(args.probe).resolve() if args.probe else build / 'bin/parameter_moos_probe'
    out.mkdir(parents=True, exist_ok=True)
    bridge = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    bridge.bind(('127.0.0.1', 0)); bridge.settimeout(.2)
    bind = bridge.getsockname()[1]
    db, base, rc = port(socket.SOCK_STREAM), port(socket.SOCK_STREAM), port(socket.SOCK_DGRAM)
    fixture = out / 'fixture.parm'; fixture.write_text('LOG_DISARMED 1\n')
    binary = ap / 'build/sitl/bin/arducopter'
    command = [str(binary), '--model', 'quad', '--home', '42.358456,-71.087589,10,0', '--speedup', '1', '--wipe',
               '--defaults', str(ap / 'Tools/autotest/default_params/copter.parm') + ',' + str(fixture),
               '--serial0', f'udpclient:127.0.0.1:{bind}', '--serial1', 'none', '--serial2', 'none',
               '--serial5', 'none', '--serial6', 'none', '--serial7', 'none', '--serial8', 'none',
               '--base-port', str(base), '--rc-in-port', str(rc)]
    result = {'result': 'FAIL', 'layer': 'parameter-only MOOS plugin + native disarmed Copter SITL', 'hardware': False, 'firmware_identity': firmware,
              'command': command, 'app_sha256': sha(build / 'bin/iMAVLink'), 'copter_sha256': sha(binary),
              'module_sha256': sha(plugin_dir / 'mavmoos_parameters.so'), 'probe_sha256': sha(probe_path),
              'bulk_receive_profile': {'app_tick_hz': 50, 'max_datagrams_per_iter': 128, 'list_original_timeout_s': 20},
              'ports': {'bridge': bind, 'db': db, 'base': base, 'rc': rc}, 'helm_started': False}
    sitl = antler = probe = None; handles = []
    try:
        log = (out / 'sitl.log').open('w'); handles.append(log)
        sitl = subprocess.Popen(command, cwd=out, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        decoder = mavutil.mavlink.MAVLink(None); decoder.robust_parsing = True
        deadline = time.monotonic() + 20; peer = None
        while time.monotonic() < deadline and peer is None:
            if sitl.poll() is not None: raise RuntimeError('SITL exited at startup')
            try: data, address = bridge.recvfrom(4096)
            except socket.timeout: continue
            for msg in decoder.parse_buffer(data) or []:
                if msg.get_type() == 'HEARTBEAT' and msg.get_srcSystem() == 1 and msg.get_srcComponent() == 1:
                    peer = address; break
        if peer is None or peer[0] != '127.0.0.1': raise RuntimeError('owned loopback simulator peer missing')
        result['ports']['peer'] = peer[1]; result['sitl_pid'] = sitl.pid
        result['owned_sockets'] = subprocess.run(['lsof', '-nP', '-a', '-p', str(sitl.pid), '-iUDP'], capture_output=True, text=True).stdout
        bridge.close()
        mission = out / 'parameters.moos'
        mission.write_text(f'''ServerHost = localhost
ServerPort = {db}
Community = parameters
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
  plugins = parameters
  plugin_directory = {plugin_dir}
  max_datagrams_per_iter = 128
  parameter_timeout = 8
  parameter_list_timeout = 20
  parameter_list_mode = streamed
}}
''')
        env = dict(os.environ); env['PATH'] = os.pathsep.join([str(build / 'bin'), str(moos / 'bin'), env.get('PATH', '')])
        log = (out / 'pantler.log').open('w'); handles.append(log)
        antler = subprocess.Popen([str(moos / 'bin/pAntler'), str(mission)], cwd=out, env=env,
                                  stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        log = (out / 'probe.log').open('w'); handles.append(log)
        probe = subprocess.Popen([str(probe_path), str(db)], cwd=out, env=env,
                                 stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        probe.wait(timeout=120)
        result['probe_exit'] = probe.returncode
        if probe.returncode != 0: raise RuntimeError('parameter probe failed')
        result['capabilities_catalog'] = observed_catalog(out / 'probe.log', {'parameters'}, mode='SITL_CONTROL')
        result['loaded_modules_sha256'] = loaded_module_hashes(plugin_dir, result['capabilities_catalog'])
        snapshots = []
        for line in (out / 'probe.log').read_text().splitlines():
            try: event = json.loads(line)
            except json.JSONDecodeError: continue
            if event.get('type') == 'snapshot': snapshots.append(json.loads(event['value']))
        if len(snapshots) != 1: raise RuntimeError('complete MOOS list snapshot missing or duplicated')
        snapshot = snapshots[0]; rows = snapshot['values']; count = snapshot['count']
        if snapshot['id'] != 'params-list' or snapshot['status'] != 'complete' or not 100 <= count <= 4096:
            raise RuntimeError('invalid snapshot identity/state/count')
        if len(rows) != count or sorted(row['index'] for row in rows) != list(range(count)):
            raise RuntimeError('MOOS snapshot did not deliver every distinct index')
        if len({row['name'] for row in rows}) != count or any(not math.isfinite(row['value']) for row in rows):
            raise RuntimeError('invalid names/values in complete MOOS snapshot')
        frame = next(row for row in rows if row['name'] == 'FRAME_CLASS')
        if frame['value'] != 1 or frame['type'] != 2: raise RuntimeError('snapshot frame type/value mismatch')
        result['complete_moos_snapshot'] = {'count': count, 'distinct_indices': len(rows),
                                           'bytes': len(json.dumps(snapshot, separators=(',', ':')))}
        time.sleep(.3)
        result['result'] = 'PASS'
    except Exception as error:
        result['error'] = str(error)
    finally:
        for process in [probe, antler, sitl]: stop(process)
        bridge.close()
        for handle in handles: handle.close()
        cleared = {}
        for name in ['bridge', 'rc', 'peer']:
            if name not in result['ports']: continue
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as check:
                try: check.bind(('127.0.0.1', result['ports'][name])); cleared[name] = True
                except OSError: cleared[name] = False
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as check:
            check.settimeout(.2); cleared['db'] = check.connect_ex(('127.0.0.1', db)) != 0
        groups = {p.pid for p in [probe, antler, sitl] if p is not None}
        result['owned_processes_remaining'] = []
        for line in subprocess.check_output(['ps', '-axo', 'pid,pgid,comm'], text=True).splitlines()[1:]:
            fields = line.strip().split(None, 2)
            if len(fields) == 3 and int(fields[1]) in groups: result['owned_processes_remaining'].append(line.strip())
        result['ports_clear'] = cleared
        if not all(cleared.values()) or result['owned_processes_remaining']: result['result'] = 'FAIL'
        wind = []; boot = []
        for logfile in sorted((out / 'logs').glob('*.BIN')):
            data = DFReader.DFReader_binary(str(logfile))
            while True:
                msg = data.recv_match(type=['PARM', 'MSG'])
                if msg is None: break
                if msg.get_type() == 'PARM' and str(msg.Name) == 'SIM_WIND_SPD': wind.append(float(msg.Value))
                if msg.get_type() == 'MSG' and ('ArduCopter' in msg.Message or 'Frame:' in msg.Message): boot.append(msg.Message)
        result['independent_fc_dataflash'] = {'boot': boot, 'SIM_WIND_SPD_values': wind}
        if result['result'] == 'PASS' and (not wind or .25 not in wind or wind[-1] != wind[0]):
            result['result'] = 'FAIL'; result['error'] = 'FC DataFlash did not independently corroborate write and restoration'
        (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    if result['result'] != 'PASS':
        for name in ['probe.log', 'pantler.log', 'sitl.log']:
            if (out / name).exists(): print(name, (out / name).read_text()[-5000:])
    return 0 if result['result'] == 'PASS' else 1


def main():
    parser = argparse.ArgumentParser()
    for name in ['ardupilot-root', 'moos-root', 'build', 'output']: parser.add_argument('--' + name, required=True)
    parser.add_argument('--plugin-dir')
    parser.add_argument('--probe')
    args = parser.parse_args()
    with (Path(tempfile.gettempdir()) / 'mavmoos-native-sitl.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        return run(args)


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(143)))
    raise SystemExit(main())
