#!/usr/bin/env python3
"""Owned native ground Rover, stock helm, read-only FC monitor and scoped RC pilot."""
import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from owned_processes import stop_process_tree
from control.catalog_evidence import loaded_module_hashes, observed_catalog
from pymavlink import mavutil


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def port(kind):
    with socket.socket(socket.AF_INET, kind) as stream:
        stream.bind(('127.0.0.1', 0))
        return stream.getsockname()[1]


def sockets(process):
    if process.poll() is not None:
        raise RuntimeError('owned native Rover exited')
    answer = subprocess.run(['lsof', '-nP', '-a', '-p', str(process.pid), '-iUDP'],
                            capture_output=True, text=True, timeout=3)
    if answer.returncode:
        raise RuntimeError('owned native Rover UDP inventory unavailable')
    return answer.stdout


def owns_udp(inventory, number, allow_wildcard=False):
    addresses = r'(?:\*|0\.0\.0\.0|127\.0\.0\.1)' if allow_wildcard else r'127\.0\.0\.1'
    return bool(re.search(r'UDP\s+' + addresses + ':' + str(number) + r'\b', inventory))


class Pilot:
    """Neutral RC1/RC3; switch RC8 from fixture Hold to real pilot Manual."""
    def __init__(self, process, rc, control, output):
        self.process, self.rc, self.control = process, rc, control
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('127.0.0.1', 0))
        self.trace = output.open('w')
        self.previous, self.last, self.sent = None, 0, 0
        inventory = sockets(process)
        if not owns_udp(inventory, rc, allow_wildcard=True):
            raise RuntimeError('exact owned native RC socket not verified')
        self.trace.write(json.dumps({'event': 'owned_rc_socket_verified', 'pid': process.pid,
                                     'port': rc, 'inventory': inventory}) + '\n')
        self.trace.flush()

    def requested_mode(self):
        if (self.control / 'takeover.request').is_file():
            return 1000, 'pilot Manual takeover'
        return 1500, 'fixture neutral and initial Hold'

    def update(self):
        if time.monotonic() - self.last < .1:
            return
        if self.process.poll() is not None:
            raise RuntimeError('owned native Rover exited before RC send')
        mode, reason = self.requested_mode()
        pwm = [1500, 0, 1500, 0, 0, 0, 0, mode]
        packet = struct.pack('<8H', *pwm)
        if self.socket.sendto(packet, ('127.0.0.1', self.rc)) != len(packet):
            raise RuntimeError('owned RC packet was not locally accepted')
        self.last = time.monotonic()
        self.sent += 1
        self.trace.write(json.dumps({'sent_mono': self.last, 'target': ['127.0.0.1', self.rc],
                         'pwm': pwm, 'zero_channels': 'retain', 'reason': reason}) + '\n')
        self.trace.flush()
        self.previous = mode

    def close(self):
        # The fixture exits while disarmed; no claim that silence releases RC.
        self.socket.close()
        self.trace.close()


def catalog(path):
    return observed_catalog(path, {'telemetry', 'parameters', 'ardurover_control'},
                            native=True, helm=True, mode='SITL_CONTROL')


def dataflash(path):
    reader = mavutil.mavlink_connection(str(path))
    records = {'MODE': [], 'ARM': [], 'RCIN': []}
    while True:
        message = reader.recv_match(type=list(records))
        if message is None:
            break
        records[message.get_type()].append(message.to_dict())
    armed = next(item for item in records['ARM'] if item['ArmState'] == 1)
    manual = next(item for item in records['MODE'] if item['ModeNum'] == 0 and item['TimeUS'] > armed['TimeUS'])
    handback = next((item for item in records['MODE'] if item['TimeUS'] > manual['TimeUS']), None)
    if not handback or handback['ModeNum'] != 4 or handback.get('Rsn') != 1:
        raise RuntimeError('DataFlash does not corroborate pilot selection of Hold after Manual')
    recovered = next(item for item in records['MODE'] if item['ModeNum'] == 15 and item['TimeUS'] > handback['TimeUS'])
    samples = [item for item in records['RCIN'] if manual['TimeUS'] <= item['TimeUS'] < handback['TimeUS']]
    # The RC switch input leads the mode change by the FC's debounce interval.
    release = next((index for index,item in enumerate(samples) if item['C8'] != 1000),len(samples))
    debounce = samples[release:]
    samples = samples[:release]
    if (manual.get('Rsn') != 1 or not samples or
            any(item['C1'] != 1500 or item['C3'] != 1500 for item in samples+debounce) or
            any(item['C8'] != 1500 for item in debounce) or
            (debounce and handback['TimeUS']-debounce[0]['TimeUS'] > 500000)):
        raise RuntimeError('DataFlash does not corroborate RC-command Manual takeover with neutral drive inputs')
    if armed.get('Forced', 0) != 0 or not any(item['ArmState'] == 0 and item['TimeUS'] > armed['TimeUS'] for item in records['ARM']):
        raise RuntimeError('normal arm/final disarm not corroborated in DataFlash')
    if not {4, 11, 15}.issubset({item['ModeNum'] for item in records['MODE']}):
        raise RuntimeError('Hold/RTL/Guided not corroborated in DataFlash')
    return {'armed': armed, 'pilot_manual': manual, 'pilot_hold_handback': handback, 'explicit_guided_recovery': recovered,
            'hold_switch_debounce_samples': len(debounce), 'manual_neutral_samples': len(samples), 'first_manual_rcin': samples[0], 'last_manual_rcin': samples[-1]}


def main(args):
    root = Path(__file__).resolve().parents[2]
    ap, moos, build, out = (Path(getattr(args, key)).resolve() for key in ('ardupilot_root', 'moos_root', 'build', 'output'))
    binary, probe_path = ap / 'build/sitl/bin/ardurover', build / 'bin/rover_workflow_probe'
    if not binary.is_file() or not probe_path.is_file():
        raise RuntimeError('built native ardurover and rover_workflow_probe are required')
    # A fresh directory gives this run sole ownership of logs, markers and BIN files.
    out.mkdir(parents=True, exist_ok=False)
    (out / 'logs').mkdir()
    control = out / 'control'
    control.mkdir()
    bridge, monitor = (socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(2))
    bridge.bind(('127.0.0.1', 0)); bridge.settimeout(.2)
    monitor.bind(('127.0.0.1', 0)); monitor.setblocking(False)
    bridge_port, monitor_port = bridge.getsockname()[1], monitor.getsockname()[1]
    db, base, rc = port(socket.SOCK_STREAM), port(socket.SOCK_STREAM), port(socket.SOCK_DGRAM)
    fixture = out / 'fixture.parm'
    fixture.write_text('SERIAL1_PROTOCOL 2\nMAV2_POSITION 10\nMAV2_EXT_STAT 2\nMAV2_EXTRA3 2\nLOG_DISARMED 1\nWP_RADIUS 1\nWP_SPEED 3\nMODE_CH 8\nMODE1 0\nMODE4 4\n')
    (out / 'message-intervals-chan1.txt').write_text('33 100\n193 500\n')
    defaults = ap / 'Tools/autotest/default_params/rover.parm'
    command = [str(binary), '--model', 'rover', '--home', '42.358456,-71.087589,10,0', '--speedup', '1', '--wipe',
               '--defaults', str(defaults) + ',' + str(fixture),
               '--serial0', f'udpclient:127.0.0.1:{bridge_port}', '--serial1', f'udpclient:127.0.0.1:{monitor_port}',
               '--serial2', 'none', '--serial5', 'none', '--serial6', 'none', '--serial7', 'none', '--serial8', 'none',
               '--base-port', str(base), '--rc-in-port', str(rc)]
    result = {'result': 'FAIL', 'layer': 'owned native ground ArduRover + stock pHelmIvP + dynamic iMAVLink',
              'hardware': False, 'timewarp': 1, 'command': command, 'native_sha256': sha(binary),
              'app_sha256': sha(build / 'bin/iMAVLink'), 'probe_sha256': sha(probe_path),
              'harness_sha256': sha(__file__), 'defaults_sha256': sha(defaults), 'fixture_sha256': sha(fixture),
              'ardupilot_revision': subprocess.check_output(['git', '-C', str(ap), 'rev-parse', 'HEAD'], text=True).strip(),
              'moos_revision': subprocess.check_output(['git', '-C', str(moos), 'rev-parse', 'HEAD'], text=True).strip(),
              'mavmoos_revision': subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip(),
              'mavmoos_revision_scope': 'harness and mission checkout; app build can differ; recorded app/probe/module hashes identify actual artifacts',
              'ports': {'bridge': bridge_port, 'monitor': monitor_port, 'db': db, 'base': base, 'rc': rc},
              'independent_monitor_writes': 'none; pinned fixture and message intervals set observation streams',
              'pilot_writes': 'native owned RC UDP only; neutral RC1/RC3, RC8 Hold-to-Manual switch'}
    sitl = launch = probe = pilot = None
    handles, packets = [], []
    try:
        native_log = (out / 'sitl.log').open('w'); handles.append(native_log)
        sitl = subprocess.Popen(command, cwd=out, stdout=native_log, stderr=subprocess.STDOUT, start_new_session=True)
        result['sitl_pid'] = sitl.pid
        decoder = mavutil.mavlink.MAVLink(None); decoder.robust_parsing = True
        deadline, peer = time.monotonic() + 20, None
        while time.monotonic() < deadline:
            if sitl.poll() is not None:
                raise RuntimeError('owned Rover exited during startup')
            try:
                data, address = bridge.recvfrom(4096)
            except socket.timeout:
                continue
            for message in decoder.parse_buffer(data) or []:
                if message.get_type() == 'HEARTBEAT' and message.get_srcSystem() == 1 and message.get_srcComponent() == 1:
                    if message.type != 10 or message.autopilot != 3:
                        raise RuntimeError('unexpected native heartbeat identity')
                    peer = address
            if peer:
                break
        # The first heartbeat can precede the lazily initialized native RC
        # backend. Preserve every ownership snapshot and wait boundedly for it.
        # MAVLink still needs exact loopback binding; only the native RC socket
        # may use AP_RCProtocol_UDP's source-defined 0.0.0.0 bind.
        result['discovered_peer'] = peer
        if peer is not None:
            result['ports']['peer'] = peer[1]
        result['sitl_udp_socket_attempts'] = []
        ready_by = time.monotonic() + 5
        while True:
            inventory = sockets(sitl)
            result['sitl_udp_sockets'] = inventory
            result['sitl_udp_socket_attempts'].append({'observed_mono': time.monotonic(), 'inventory': inventory})
            if peer is not None and peer[0] == '127.0.0.1' and owns_udp(inventory, peer[1]) and owns_udp(inventory, rc, allow_wildcard=True):
                break
            if time.monotonic() >= ready_by:
                break
            time.sleep(.1)
        if peer is None or peer[0] != '127.0.0.1' or not owns_udp(inventory, peer[1]) or not owns_udp(inventory, rc, allow_wildcard=True):
            raise RuntimeError('exact owned Rover loopback/RC endpoints not verified')
        bridge.close()
        pilot = Pilot(sitl, rc, control, out / 'fixture-pilot.jsonl')
        pilot.update()
        replay = out / 'native-replay'
        replay.mkdir(mode=0o700)
        macros = {'NATIVE_REPLAY_CONFIG': f'plugin.ardurover_control.native_replay_directory = {replay}',
                  'MOOS_PORT': str(db), 'APP_PATH': str(build / 'bin/iMAVLink'), 'BIND_PORT': str(bridge_port),
                  'PEER_PORT': str(peer[1]), 'LAT_ORIGIN': '42.358456', 'LONG_ORIGIN': '-71.087589',
                  'SIMULATION_ONLY': 'true', 'SHADOW_MODE': 'false', 'SITL_PID': str(sitl.pid),
                  'PLUGIN_DIRECTORY': str(build / 'plugins'), 'SPEED': '1.2', 'WAYPOINTS': '0,12:12,12:12,0:0,0'}
        env = dict(os.environ)
        env['PATH'] = os.pathsep.join([str(build / 'bin'), str(moos / 'bin'), env.get('PATH', '')])
        # This mission uses stock built-in 2D behaviors only.
        env.pop('IVP_BEHAVIOR_DIRS', None)
        for suffix in ('moos', 'bhv'):
            subprocess.run([str(moos / 'bin/nsplug'), str(root / f'missions/rover_sitl/meta_vehicle.{suffix}'),
                            str(out / f'targ_rover.{suffix}'), '--strict', '--force', '-x'] +
                           [f'{key}={value}' for key, value in macros.items()], env=env, check=True, capture_output=True)
            result[f'generated_{suffix}_sha256'] = sha(out / f'targ_rover.{suffix}')
        log = (out / 'pantler.log').open('w'); handles.append(log)
        launch = subprocess.Popen([str(moos / 'bin/pAntler'), str(out / 'targ_rover.moos')], cwd=out,
                                  env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        log = (out / 'probe-stdout.log').open('w'); handles.append(log)
        probe = subprocess.Popen([str(probe_path), str(db), str(out / 'workflow.jsonl'), str(control)],
                                 cwd=out, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        decoder = mavutil.mavlink.MAVLink(None); decoder.robust_parsing = True
        deadline, drain, monitor_peer = time.monotonic() + 200, None, None
        with (out / 'fc-monitor.jsonl').open('w') as trace, (out / 'fc-monitor.mavlink').open('wb') as raw:
            while time.monotonic() < deadline:
                if probe.poll() is not None:
                    if drain is None:
                        drain = time.monotonic() + 1.5
                    if time.monotonic() >= drain:
                        break
                else:
                    pilot.update()
                try:
                    data, address = monitor.recvfrom(65535)
                except BlockingIOError:
                    time.sleep(.02); continue
                if monitor_peer is None:
                    if address[0] != '127.0.0.1' or not owns_udp(sockets(sitl), address[1]):
                        raise RuntimeError('independent telemetry endpoint is not owned by native Rover')
                    monitor_peer = address
                    result['monitor_peer'] = address
                if address != monitor_peer:
                    raise RuntimeError('independent monitor received another endpoint')
                raw.write(data)
                for message in decoder.parse_buffer(data) or []:
                    if message.get_srcSystem() == 1 and message.get_srcComponent() == 1 and message.get_type() in {'HEARTBEAT', 'GLOBAL_POSITION_INT', 'STATUSTEXT'}:
                        row = {'received_mono': time.monotonic(), **message.to_dict()}
                        trace.write(json.dumps(row) + '\n'); packets.append(row)
            if probe.poll() is None:
                raise RuntimeError('bounded 200 second wrapper deadline exceeded')
        result['probe_exit'] = probe.returncode
        hearts = [row for row in packets if row['mavpackettype'] == 'HEARTBEAT']
        positions = [row for row in packets if row['mavpackettype'] == 'GLOBAL_POSITION_INT' and abs(row['lat']/1e7-42.358456) < .001 and abs(row['lon']/1e7+71.087589) < .001]
        def distance(row):
            return math.hypot((row['lat']/1e7-42.358456)*111195., (row['lon']/1e7+71.087589)*111195.*math.cos(math.radians(42.358456)))
        independent = {'observed_modes': sorted({row['custom_mode'] for row in hearts}),
                       'armed_observed': any(row['base_mode'] & 128 for row in hearts),
                       'final_disarmed': bool(hearts) and not bool(hearts[-1]['base_mode'] & 128),
                       'position_samples': len(positions), 'heartbeat_samples': len(hearts),
                       'maximum_home_distance_m': max(map(distance, positions), default=0),
                       'final_home_distance_m': distance(positions[-1]) if positions else -1,
                       'final_speed_m_s': math.hypot(positions[-1]['vx'], positions[-1]['vy'])/100 if positions else -1}
        result['independent_monitor'] = independent
        if probe.returncode != 0:
            raise RuntimeError('Rover MOOS workflow failed; inspect workflow.jsonl')
        result['capabilities_catalog'] = catalog(out / 'workflow.jsonl')
        result['loaded_modules_sha256'] = loaded_module_hashes(build / 'plugins', result['capabilities_catalog'])
        if not {0, 4, 11, 15}.issubset(independent['observed_modes']) or not independent['armed_observed'] or not independent['final_disarmed'] or independent['maximum_home_distance_m'] < 4 or not 0 <= independent['final_home_distance_m'] <= 2.5 or not 0 <= independent['final_speed_m_s'] <= .4:
            raise RuntimeError('independent FC monitor did not corroborate motion, takeover, return and disarmed stop')
        result['result'] = 'PASS'
    except BaseException as error:
        result['error'] = str(error) or type(error).__name__
    finally:
        if pilot is not None:
            pilot.close()
        remaining = []
        for process in (probe, launch, sitl):
            remaining.extend(stop_process_tree(process))
        bridge.close(); monitor.close()
        for handle in handles:
            handle.close()
        clear = {}
        for name in ('bridge', 'monitor', 'rc', 'peer'):
            if name in result['ports']:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as stream:
                    try:
                        stream.bind(('127.0.0.1', result['ports'][name])); clear[name] = True
                    except OSError:
                        clear[name] = False
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as stream:
            stream.settimeout(.2); clear['moos_tcp'] = stream.connect_ex(('127.0.0.1', db)) != 0
        result['ports_clear'], result['owned_processes_remaining'] = clear, remaining
        if not all(clear.values()) or remaining:
            result['result'], result['cleanup_error'] = 'FAIL', 'owned process/endpoint cleanup incomplete'
        if result['result'] == 'PASS':
            try:
                bins = list((out / 'logs').glob('*.BIN'))
                if len(bins) != 1:
                    raise RuntimeError('expected exactly one new BIN from fresh owned output')
                digest = sha(bins[0]); proof = dataflash(bins[0])
                if sha(bins[0]) != digest:
                    raise RuntimeError('selected DataFlash file changed during read')
                result['independent_dataflash'] = {'path': str(bins[0]), 'sha256': digest, **proof}
            except Exception as error:
                result['result'], result['error'] = 'FAIL', str(error)
        (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'result': result['result'], 'output': str(out), 'error': result.get('error'),
                      'independent_monitor': result.get('independent_monitor'), 'ports_clear': result['ports_clear']}, indent=2))
    return 0 if result['result'] == 'PASS' else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('ardupilot-root', 'moos-root', 'build', 'output'):
        parser.add_argument('--' + name, required=True)
    arguments = parser.parse_args()
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(143)))
    with (Path(tempfile.gettempdir()) / 'mavmoos-native-sitl.lock').open('w') as lease:
        fcntl.flock(lease, fcntl.LOCK_EX)
        raise SystemExit(main(arguments))
