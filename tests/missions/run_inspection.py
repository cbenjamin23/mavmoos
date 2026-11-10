#!/usr/bin/env python3
"""Verify mission inspection with the real module, MOOSDB and one owned peer."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tests'))
from mission_evidence import observed_catalog
from owned_processes import native_firmware_profile, stop_process_tree

LOOPBACK = '127.0.0.1'
TYPES = {'flight': 0, 'fence': 1, 'rally': 2}
SNAPSHOT_PREFIX = 'MAVMOOS_MISSION_INSPECTION_'
SEED_SYSTEM, SEED_COMPONENT = 246, 190
HOME = (423584560, -710875890)
# Native seed rows are (command, frame, x, y, z, parameter 1).
SEED = {
    'flight': [(16, 0, *HOME, 10, 0),
               (16, 3, HOME[0] + 120, HOME[1], 12, 0),
               (16, 3, HOME[0] + 160, HOME[1] + 90, 14, 1)],
    'fence': [(5001, 0, HOME[0] - 6000, HOME[1] - 6000, 0, 3),
              (5001, 0, HOME[0] + 6000, HOME[1] - 6000, 0, 3),
              (5001, 0, HOME[0], HOME[1] + 6000, 0, 3),
              (5004, 0, HOME[0] + 1800, HOME[1] + 2200, 0, 11)],
    'rally': [(5100, 3, HOME[0] + 750, HOME[1] - 1000, 21, 0),
              (5100, 0, HOME[0] + 950, HOME[1] + 1400, 36, 0)],
}


def require(ok, error):
    if not ok:
        raise RuntimeError(error)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def ref(root):
    return subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'],
                                   text=True, timeout=5).strip()


def reserve(kind):
    stream = socket.socket(socket.AF_INET, kind)
    stream.bind((LOOPBACK, 0))
    return stream


def port(kind):
    with reserve(kind) as stream:
        return stream.getsockname()[1]


def events(path):
    if not path.exists():
        return []
    raw = path.read_bytes()
    complete = raw[:raw.rfind(b'\n') + 1]
    return [json.loads(line) for line in complete.splitlines() if line.startswith(b'{')]


def fields(text):
    return dict(part.split('=', 1) for part in text.split(','))


def mission_text(db, bridge, peer, plugins, sitl_pid):
    return f'''ServerHost = 127.0.0.1
ServerPort = {db}
Community = mission_inspection_{db}
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
  bind_port = {bridge}
  peer_address = 127.0.0.1
  peer_port = {peer}
  target_system = 1
  target_component = 1
  expected_type = 2
  expected_autopilot = 3
  simulation_only = {'true' if sitl_pid else 'false'}
  sitl_pid = {sitl_pid or 0}
  shadow_mode = false
  helm_enabled = false
  heartbeat_timeout = 2
  firmware_requests = false
  plugins = missions
  plugin_directory = {plugins}
  max_datagrams_per_iter = 128
}}
'''


def run_loopback(peer, rogue, bridge, args, process, gate, report, out):
    encoded = subprocess.check_output([str(args.probe), '--frames'], text=True, timeout=3)
    (out / 'generated-frames.hex').write_text(encoded)
    frames = [bytes.fromhex(line) for line in encoded.splitlines()]
    require(len(frames) == 192 * 5, 'missing generated heartbeat and mission-count frames')
    deadline = time.monotonic() + 19
    silence_until = None
    log = out / 'probe.jsonl'
    with (out / 'app-egress.mavlink').open('wb') as trace:
        for start in range(0, len(frames), 5):
            if time.monotonic() >= deadline or process.poll() is not None:
                break
            now = time.monotonic()
            if silence_until is None and any(row['type'] == 'phase' for row in events(log)):
                silence_until = now + 2.4
            if silence_until is not None and now >= silence_until and not gate.exists():
                gate.touch()
            first, *rest = frames[start:start + 5]
            if silence_until is None or gate.exists():
                peer.sendto(first, (LOOPBACK, bridge))
            else:
                # Try to mask selected-peer heartbeat loss from the wrong socket.
                rogue.sendto(first, (LOOPBACK, bridge))
            for packet in rest:
                peer.sendto(packet, (LOOPBACK, bridge))
            time.sleep(.09)
            for receiver in (peer, rogue):
                try:
                    data, address = receiver.recvfrom(65535)
                except BlockingIOError:
                    continue
                trace.write(data)
                raise RuntimeError(f'receive-only app emitted UDP to {address}')
    require(gate.exists(), 'generated fixture did not complete its heartbeat-loss interval')
    report['bridge_egress_datagrams'] = 0


def owned_endpoint(process, local, remote):
    require(process.poll() is None, 'owned simulator stopped')
    inventory = subprocess.check_output(['lsof', '-nP', '-a', '-p', str(process.pid), '-iUDP'],
                                        text=True, timeout=3)
    expected = rf'UDP\s+127\.0\.0\.1:{local}->127\.0\.0\.1:{remote}\b'
    require(re.search(expected, inventory), 'simulator PID does not own the selected endpoint')
    return inventory


def discover(stream, process, dialect):
    parser = dialect.MAVLink(None)
    parser.robust_parsing = True
    deadline = time.monotonic() + 17
    while time.monotonic() < deadline:
        require(process.poll() is None, 'owned simulator exited during discovery')
        try:
            data, address = stream.recvfrom(65535)
        except socket.timeout:
            continue
        for msg in parser.parse_buffer(data) or []:
            if (msg.get_type() == 'HEARTBEAT' and (msg.get_srcSystem(), msg.get_srcComponent()) == (1, 1)
                    and msg.type == 2 and msg.autopilot == 3 and not msg.base_mode & 128):
                require(address[0] == LOOPBACK, 'simulator discovery left loopback')
                return address
    raise RuntimeError('disarmed owned Copter heartbeat was absent')


class SeedPeer:
    """Independent GCS for private simulator storage setup and disarmed evidence."""
    def __init__(self, dialect, stream, address, sitl, out):
        self.stream, self.address, self.sitl = stream, address, sitl
        self.inbound = dialect.MAVLink(None)
        self.inbound.robust_parsing = True
        self.outbound = dialect.MAVLink(self, srcSystem=SEED_SYSTEM, srcComponent=SEED_COMPONENT)
        self.heartbeat_at = self.landed_at = -1
        self.hearts = self.landed = self.writes = 0
        self.out = out

    def write(self, data):
        require(time.monotonic() - min(self.heartbeat_at, self.landed_at) < 2,
                'fixture writes require fresh disarmed ground evidence')
        owned_endpoint(self.sitl, self.address[1], self.stream.getsockname()[1])
        require(self.stream.sendto(data, self.address) == len(data), 'incomplete fixture seed send')
        self.writes += 1
        with (self.out / 'seed-egress.mavlink').open('ab') as trace:
            trace.write(data)

    def receive(self):
        require(self.sitl.poll() is None, 'owned simulator exited')
        try:
            data, address = self.stream.recvfrom(65535)
        except socket.timeout:
            return []
        require(address == self.address, 'unexpected simulator observer endpoint')
        with (self.out / 'seed-ingress.mavlink').open('ab') as trace:
            trace.write(data)
        messages = [m for m in self.inbound.parse_buffer(data) or []
                    if (m.get_srcSystem(), m.get_srcComponent()) == (1, 1)]
        with (self.out / 'seed-ingress.jsonl').open('a') as trace:
            for message in messages:
                kind = message.get_type()
                now = time.monotonic()
                if kind == 'HEARTBEAT':
                    require(not message.base_mode & 128 and message.custom_mode != 3,
                            'fixture observed arming or AUTO')
                    self.heartbeat_at = now
                    self.hearts += 1
                elif kind == 'EXTENDED_SYS_STATE':
                    require(message.landed_state == 1, 'fixture observed unexpected landed state')
                    self.landed_at = now
                    self.landed += 1
                if kind in {'MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK', 'HEARTBEAT', 'EXTENDED_SYS_STATE'}:
                    trace.write(json.dumps({'at': now, **message.to_dict()}) + '\n')
        return messages

    def load(self, name, rows=None):
        rows, plan_type = SEED[name] if rows is None else rows, TYPES[name]
        deadline, next_seq = time.monotonic() + 5, 0
        self.outbound.mission_count_send(1, 1, len(rows), plan_type)
        while time.monotonic() < deadline:
            for message in self.receive():
                kind = message.get_type()
                if (kind not in ('MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK') or
                        message.get_msgbuf()[0] != 0xFD or
                        (message.target_system, message.target_component, message.mission_type) !=
                        (SEED_SYSTEM, SEED_COMPONENT, plan_type)):
                    continue
                if kind == 'MISSION_ACK':
                    require(message.type == 0 and next_seq == len(rows), f'fixture upload rejected {name}')
                    return
                require(message.seq == next_seq and next_seq < len(rows), f'fixture upload sequence changed {name}')
                command, frame, x, y, z, first = rows[next_seq]
                self.outbound.mission_item_int_send(1, 1, next_seq, frame, command, 0, 1,
                                                    first, 0, 0, 0, x, y, z, plan_type)
                next_seq += 1
        raise RuntimeError(f'fixture seed ACK deadline for {name}')


def drive_native(seed, process, gate, out):
    deadline = time.monotonic() + 38
    log = out / 'probe.jsonl'
    while time.monotonic() < deadline and process.poll() is None:
        seed.receive()
        if not gate.exists() and any(row['type'] == 'phase' for row in events(log)):
            while time.monotonic() < deadline and time.monotonic() - min(seed.heartbeat_at, seed.landed_at) >= .6:
                seed.receive()
            for name in SEED:
                seed.load(name)
            gate.touch()
    require(gate.exists(), 'native fixture did not seed after the empty readback')
    require(seed.hearts >= 2 and seed.landed >= 2 and time.monotonic() - seed.landed_at < 2,
            'independent disarmed ground evidence absent or stale')
    return {'heartbeat_samples': seed.hearts, 'ground_samples': seed.landed,
            'ever_armed': False, 'final_on_ground': True, 'fixture_storage_writes': seed.writes}


def audit(rows, mode):
    native = mode == 'native'
    catalog = observed_catalog(rows, 'SITL_CONTROL' if native else 'RX_ONLY')
    require(catalog['composition']['plugin_abi'] == 10 and
            catalog['composition']['modules'][0]['request_owners'] == ['MAVMOOS_MISSION_REQUEST'],
            'wrong inspection ABI or request owner')
    require(any(r['type'] == 'workflow' and r['value'] == 'PASS' for r in rows), 'missing operator completion')
    runtime = [fields(r['value']) for r in rows if r['type'] == 'runtime']
    require(runtime and all(r['nav_tx_accepted'] == '0' and r['mission_mutations'] == '0' and
                           r['armed'] == '0' and r['permission'] == 'false' and r['control_profile'] == 'none'
                           for r in runtime), 'runtime effect/control state was not inspection-only')
    require(all(fields(r['value']).get('valid') == 'false' for r in rows
                if r['key'] == 'MAVMOOS_STORED_MISSION_PROOF'), 'inspection issued a flight execution proof')
    all_snapshots = [(r['key'], json.loads(r['value'])) for r in rows if r['type'] == 'snapshot']
    snapshots = [(key, value) for key, value in all_snapshots if value['status'] == 'complete']
    tx = [fields(r['value']) for r in rows if r['type'] == 'tx']
    results = [fields(r['value']) for r in rows if r['type'] == 'result']
    requests = [fields(r['value']) for r in rows if r['type'] == 'request']
    expected_ids = [name + '-empty' for name in TYPES]
    expected_ids += [name + '-seeded' for name in TYPES] if native else ['fence-empty', 'fence-new-session']
    require([r['id'] for r in requests] == expected_ids and
            all(r['schema'] == '1' and r['action'] == 'inspect' and r['timeout'] == '6' for r in requests),
            'operator workflow changed')
    summary = {'capabilities_catalog': catalog}
    if native:
        require(len(snapshots) == 6 and {s['id'] for _, s in snapshots} == set(expected_ids),
                'missing or conflated complete snapshots')
        for key, snap in snapshots:
            name = snap['type']
            ident, plan_type = snap['id'], TYPES[name]
            expected = SEED[name] if ident.endswith('-seeded') else []
            require(key == SNAPSHOT_PREFIX + name.upper() + '_SNAPSHOT' and ident.startswith(name + '-') and
                    snap['schema'] == 1 and snap['mission_type'] == plan_type and snap['count'] == len(expected) and
                    len(snap['items']) == len(expected) and snap['verification'] == 'observation' and
                    snap['atomic'] is False and snap['source_age'] == 'unknown', f'bad snapshot scope for {ident}')
            complete = [r for r in results if r['id'] == ident and r['status'] == 'COMPLETED']
            require(len(complete) == 1 and complete[0]['mission_type'] == str(plan_type) and
                    complete[0]['session_epoch'] == str(snap['session_epoch']) and
                    complete[0]['mutation_may_have_applied'] == complete[0]['readback_confirmed'] == 'false',
                    f'bad terminal result for {ident}')
            for seq, (item, expected_row) in enumerate(zip(snap['items'], expected)):
                command, frame, x, y, z, first = expected_row
                require((item['seq'], item['command'], item['frame']) == (seq, command, frame),
                        f'wrong item identity for {ident}')
                if not (name == 'flight' and seq == 0):
                    require((item['x'], item['y'], item['z'], item['params']) == (x, y, z, [first, 0, 0, 0]),
                            f'stored row differs from seeded coordinates/parameters for {ident}:{seq}')
            sends = [r for r in tx if r['operation_id'] == ident]
            require([r['msgid'] for r in sends] == ['43'] + ['51'] * len(expected) + ['47'] and
                    all(r['mission_type'] == str(plan_type) and r['late'] == 'false' and r['clock_valid'] == 'true' and
                        r['status'] == 'LOCAL_ACCEPTED' for r in sends),
                    f'wrong typed read sequence or local send state for {ident}')
        require(len(tx) == sum(2 + snap['count'] for _, snap in snapshots), 'unexpected bridge mission send')
        summary['snapshot_counts'] = {snap['id']: snap['count'] for _, snap in snapshots}
    else:
        require(not snapshots and not tx and all(r['tx_attempts'] == r['tx_accepted'] == '0' for r in runtime),
                'receive-only inspection advanced or sent')
        require([r['id'] for r in results if r['status'] == 'FAILED'] == [*expected_ids[:3], 'fence-new-session'] and
                any(r['status'] == 'DUPLICATE' and r['id'] == 'fence-empty' for r in results),
                'receive-only refusal/replay outcomes differ')
        first = next(r for r in results if r['id'] == 'fence-empty' and r['status'] == 'FAILED')
        later = next(r for r in results if r['id'] == 'fence-new-session' and r['status'] == 'FAILED')
        require(int(later['session_epoch']) > int(first['session_epoch']) and
                {s['type'] for _, s in all_snapshots if s.get('status') == 'invalidated' and
                 s.get('session_epoch') == int(later['session_epoch'])} == set(TYPES),
                'heartbeat loss did not clear all three observations')
        summary['reset_epochs'] = [int(first['session_epoch']), int(later['session_epoch'])]
    summary['terminal_results'] = [r for r in results if r['status'] in ('COMPLETED', 'FAILED', 'DUPLICATE')]
    return summary


def run(args):
    native = args.mode == 'native'
    out, build, moos = args.output.resolve(), args.build.resolve(), args.moos_root.resolve()
    out.mkdir(parents=True, exist_ok=False)
    report = {'result': 'FAIL', 'mode': args.mode, 'hardware': False, 'scope': 'owned loopback mission inspection'}
    processes, sockets = [], []
    protected = {}
    ports = {}

    def launch(argv, filename, env=None):
        with (out / filename).open('w') as stream:
            process = subprocess.Popen(argv, cwd=out, env=env, stdout=stream, stderr=subprocess.STDOUT,
                                       start_new_session=True)
        processes.append(process)
        return process

    def udp(name, timeout=None):
        stream = reserve(socket.SOCK_DGRAM)
        stream.settimeout(timeout)
        ports[name] = stream.getsockname()[1]
        sockets.append(stream)
        return stream

    try:
        app, module = build / 'bin/iMAVLink', build / 'plugins/mavmoos_missions.so'
        pinned = [app, module, args.probe, Path(__file__), Path(__file__).with_name('InspectionProbe.cpp'),
                  Path(__file__).with_name('mission_evidence.py')]
        ports['db'] = port(socket.SOCK_STREAM)
        report.update({'source_ref': ref(ROOT), 'moos_ref': ref(moos), 'app_sha256': sha(app),
                       'module_sha256': sha(module), 'probe_sha256': sha(args.probe)})
        sitl = None
        if native:
            from pymavlink.dialects.v20 import ardupilotmega as dialect
            ap = args.ardupilot_root.resolve()
            firmware = native_firmware_profile(ROOT, ap)
            require(firmware is not None, 'select the native Copter manifest and attestation')
            executable = ap / 'build/sitl/bin/arducopter'
            pinned.extend((executable, Path(firmware['manifest_path']), Path(firmware['attestation_path'])))
            report.update({'firmware_identity': firmware, 'native_sha256': sha(executable),
                           'ardupilot_ref': ref(ap)})
            primary, secondary = udp('bridge', .1), udp('seed', .04)
            ports['base'] = port(socket.SOCK_STREAM)
            ports['rc'] = port(socket.SOCK_DGRAM)
            defaults = out / 'fixture.parm'
            defaults.write_text('SERIAL1_PROTOCOL 2\nMAV2_POSITION 2\nMAV2_EXT_STAT 2\nMAV2_EXTRA3 2\n')
            (out / 'message-intervals-chan1.txt').write_text('33 500\n245 500\n')
            command = [str(executable), '--model', 'quad', '--home', '42.358456,-71.087589,10,0',
                       '--speedup', '1', '--wipe', '--defaults',
                       str(ap / 'Tools/autotest/default_params/copter.parm') + ',' + str(defaults),
                       '--serial0', f'udpclient:{LOOPBACK}:{ports["bridge"]}',
                       '--serial1', f'udpclient:{LOOPBACK}:{ports["seed"]}',
                       '--serial2', 'none', '--serial5', 'none', '--serial6', 'none',
                       '--serial7', 'none', '--serial8', 'none',
                       '--base-port', str(ports['base']), '--rc-in-port', str(ports['rc'])]
            report['command'] = command
            protected = {str(path): sha(path) for path in pinned}
            for role in ('manifest', 'attestation'):
                shutil.copyfile(firmware[role + '_path'], out / ('firmware-' + role + '.json'))
            sitl = launch(command, 'sitl.log')
            report['sitl_pid'] = sitl.pid
            primary_address = discover(primary, sitl, dialect)
            secondary_address = discover(secondary, sitl, dialect)
            ports.update(peer=primary_address[1], seed_peer=secondary_address[1])
            report['socket_inventory'] = owned_endpoint(sitl, ports['peer'], ports['bridge'])
            owned_endpoint(sitl, ports['seed_peer'], ports['seed'])
            primary.close()
            seed = SeedPeer(dialect, secondary, secondary_address, sitl, out)
        else:
            protected = {str(path): sha(path) for path in pinned}
            ports['bridge'] = port(socket.SOCK_DGRAM)
            peer, rogue = udp('peer', 0), udp('rogue', 0)
        mission = out / 'inspection.moos'
        mission.write_text(mission_text(ports['db'], ports['bridge'], ports['peer'], build / 'plugins',
                                        sitl.pid if sitl else None))
        env = dict(os.environ)
        env['PATH'] = os.pathsep.join([str(build / 'bin'), str(moos / 'bin'), env.get('PATH', '')])
        antler = launch([str(moos / 'bin/pAntler'), str(mission)], 'pantler.log', env)
        gate = out / 'workflow-ready'
        probe = launch([str(args.probe), str(ports['db']), args.mode, str(gate)], 'probe.jsonl', env)
        if native:
            report['observer'] = drive_native(seed, probe, gate, out)
            require(report['observer']['fixture_storage_writes'] == len(SEED) + sum(map(len, SEED.values())),
                    'fixture write count differs from its bounded seed')
        else:
            run_loopback(peer, rogue, ports['bridge'], args, probe, gate, report, out)
        report['probe_exit'] = probe.wait(timeout=1)
        require(report['probe_exit'] == 0 and antler.poll() is None, 'inspection probe or mission launcher failed')
        report.update(audit(events(out / 'probe.jsonl'), args.mode))
        report['result'] = 'PASS'
    except Exception as error:
        report['error'] = str(error)
    finally:
        report['owned_processes_remaining'] = []
        for process in reversed(processes):
            report['owned_processes_remaining'].extend(stop_process_tree(process))
        for stream in sockets:
            stream.close()
        clear = {}
        for name, number in ports.items():
            kind = socket.SOCK_STREAM if name in ('db', 'base') else socket.SOCK_DGRAM
            with socket.socket(socket.AF_INET, kind) as stream:
                if kind == socket.SOCK_STREAM:
                    stream.settimeout(.2)
                    clear[name] = stream.connect_ex((LOOPBACK, number)) != 0
                else:
                    try:
                        stream.bind((LOOPBACK, number))
                        clear[name] = True
                    except OSError:
                        clear[name] = False
        report.update({'ports': ports, 'ports_clear': clear, 'artifacts_before': protected})
        report['artifact_drift'] = [path for path, digest in protected.items()
                                    if not Path(path).is_file() or sha(Path(path)) != digest]
        if report['owned_processes_remaining'] or not all(clear.values()) or report['artifact_drift']:
            report['result'] = 'FAIL'
            report['cleanup_error'] = 'owned resources or protected file drift'
        report['files'] = {str(path.relative_to(out)): sha(path) for path in sorted(out.rglob('*'))
                           if path.is_file() and not path.is_symlink()}
        (out / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({key: report[key] for key in ('result', 'mode', 'error', 'cleanup_error',
                      'snapshot_counts', 'reset_epochs', 'ports_clear') if key in report}, indent=2))
    if report['result'] != 'PASS':
        for name in ('probe.jsonl', 'pantler.log', 'sitl.log'):
            path = out / name
            if path.is_file():
                print(name, path.read_text(errors='replace')[-2400:])
    return int(report['result'] != 'PASS')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=('native', 'loopback'), required=True)
    for option in ('output', 'moos-root', 'build', 'probe'):
        parser.add_argument('--' + option, type=Path, required=True)
    parser.add_argument('--ardupilot-root', type=Path)
    args = parser.parse_args()
    if (args.mode == 'native') != bool(args.ardupilot_root):
        parser.error('native mode requires an explicit owned ArduPilot checkout')
    args.probe = args.probe.resolve()
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(143)))
    if args.mode == 'native':
        with (Path(tempfile.gettempdir()) / 'mavmoos-native-sitl.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            raise SystemExit(run(args))
    raise SystemExit(run(args))
