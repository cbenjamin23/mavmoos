#!/usr/bin/env python3
"""Camera and gimbal operations through an exact owned disarmed Copter SITL."""

import argparse
from contextlib import ExitStack
import errno
import fcntl
import json
import os
from pathlib import Path
import re
import select
import signal
import socket
import subprocess
import sys
import tempfile
import traceback

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'tests'), str(ROOT / 'scripts/firmware')]
from manifest import validate_manifest, verify_native
from owned_processes import stop_process_tree
from payload_camera.camera_fixture import CameraFixture, mavlink
from payload_gimbal.gimbal_fixture import GimbalFixture
from plane_native.flight_fixture import (Channel, MAX_CAPTURE_BYTES, exact_peer, inventory, json_data,
                                         log, need, now, reserve, revision, sha, start)


MODULES = ('telemetry', 'camera_status', 'gimbal_status')
PROFILE = 'arducopter-4.6.3-quad'
MANIFEST = ROOT / 'config/firmware' / (PROFILE + '.json')
APP, FC, CAMERA, GIMBAL = (245, 191), (1, 1), (1, 100), (1, 154)
CAMERA_REQUEST, CAMERA_RESULT = 'MAVMOOS_CAMERA_REQUEST', 'MAVMOOS_CAMERA_RESULT'
GIMBAL_REQUEST, GIMBAL_RESULT = 'MAVMOOS_GIMBAL_REQUEST', 'MAVMOOS_GIMBAL_RESULT'
TERMINAL = {'CONFIRMED', 'CANCELLED', 'FAILED', 'REJECTED', 'EXPIRED', 'UNCERTAIN'}


def preflight(args):
    manifest = validate_manifest(MANIFEST)
    firmware = verify_native(MANIFEST, args.firmware_attestation, args.ardupilot_root)
    need(firmware['profile_id'] == PROFILE and firmware['frame'] == 'quad' and not firmware['fixture']['hardware'],
         'exact selected Copter 4.6.3 quad required')
    need(revision(ROOT) == args.expected_app_revision and revision(args.moos_root) == manifest['dependencies']['moos_revision'],
         'clean exact MAVMOOS and stock MOOS revisions required')
    build, moos = args.build, args.moos_root
    cache = build / 'CMakeCache.txt'
    for key, path in (('CMAKE_HOME_DIRECTORY', ROOT), ('MOOS_IVP_ROOT', moos)):
        values = re.findall(rf'^{key}:[^=]+=([^\n]+)$', cache.read_text(), re.M)
        need(len(values) == 1 and Path(values[0]).resolve() == path, f'build provenance differs at {key}')
    paths = {
        'cache': cache, 'native': Path(firmware['native_path']),
        'defaults': args.ardupilot_root / firmware['fixture']['default_parameter_file'],
        'app': build / 'bin/iMAVLink', 'probe': build / 'bin/payload_operations_moos_probe',
        'antler': moos / 'bin/pAntler', 'db': moos / 'bin/MOOSDB', 'helm': moos / 'bin/pHelmIvP',
        'manifest': MANIFEST, 'attestation': args.firmware_attestation,
    }
    for module in MODULES:
        paths[module] = build / 'plugins' / f'mavmoos_{module}.so'
    for name in ('tests/payload_operations/run_native.py', 'tests/payload_operations/PayloadMoosProbe.cpp',
                 'tests/payload_camera/camera_fixture.py', 'tests/payload_gimbal/gimbal_fixture.py',
                 'tests/plane_native/flight_fixture.py', 'tests/owned_processes.py', 'scripts/firmware/manifest.py'):
        paths[name] = ROOT / name
    need(all(path.is_file() and not path.is_symlink() for path in paths.values()), 'frozen regular artifact required')
    hashes = {name: sha(path) for name, path in paths.items()}
    need(hashes['native'] == firmware['native_sha256'] and hashes['manifest'] == firmware['manifest_sha256'] and
         hashes['attestation'] == firmware['attestation_sha256'], 'firmware inputs changed during preflight')
    return paths, hashes, firmware


def mission_text(ports, paths, pid):
    return f"""ServerHost = 127.0.0.1
ServerPort = {ports['db']}
Community = payload_ops_{ports['db']}
MOOSTimeWarp = 1
TERM_REPORTING = true
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
  bind_port = {ports['bridge']}
  peer_address = 127.0.0.1
  peer_port = {ports['bridge_peer']}
  target_system = 1
  target_component = 1
  expected_type = 2
  expected_autopilot = 3
  simulation_only = true
  shadow_mode = false
  sitl_pid = {pid}
  firmware_requests = false
  plugins = {','.join(MODULES)}
  plugin_directory = {paths['telemetry'].parent}
  camera_component = {CAMERA[1]}
  gimbal_component = {GIMBAL[1]}
  plugin.camera_status.capture_enabled = true
  plugin.gimbal_status.point_enabled = true
  max_datagrams_per_iter = 128
}}
"""


def fields(value):
    return dict(field.split('=', 1) for field in value.split(',') if '=' in field)


class Probe:
    def __init__(self, process, stack, out):
        self.process = process
        self.records = []
        self.latest = {}
        self.partial = b''
        os.set_blocking(process.stdout.fileno(), False)
        self.witness = stack.enter_context((out / 'moos.jsonl').open('x'))
        self.stdout = stack.enter_context((out / 'probe.stdout').open('xb'))

    def read(self):
        data = os.read(self.process.stdout.fileno(), 16384)
        need(bool(data), 'MOOS probe closed the observation pipe')
        need(self.stdout.tell() + len(data) <= MAX_CAPTURE_BYTES, 'MOOS probe byte limit')
        self.stdout.write(data)
        lines = (self.partial + data).split(b'\n')
        self.partial = lines.pop()
        need(len(self.partial) <= 16384, 'MOOS probe line limit')
        for line in lines:
            # MOOS can print a startup carriage return before the first row.
            parts = line.lstrip(b'\r').decode('utf-8', errors='replace').split('\t', 3)
            if len(parts) != 4 or parts[0] not in ('app', 'request'):
                continue
            kind, key, stamp, value = parts
            if kind == 'app' and value.startswith('{'):
                value = json.loads(value)
            elif kind == 'app' and key == GIMBAL_RESULT:
                value = fields(value)
            row = {'kind': kind, 'key': key, 'mono': float(stamp), 'value': value}
            log(self.witness, row)
            self.records.append(row)
            if kind == 'app':
                self.latest[key] = value

    def request(self, key, value):
        self.process.stdin.write(f'{key}\t{value}\n'.encode('ascii'))
        self.process.stdin.flush()

    def results(self, key, ident, after):
        return [row['value'] for row in self.records if row['kind'] == 'app' and row['key'] == key and
                row['mono'] >= after and row['value'].get('id') == ident]

    def terminal(self, key, ident, after):
        return next((row for row in reversed(self.results(key, ident, after)) if row['status'] in TERMINAL), None)

    def ready(self):
        state = fields(self.latest.get('MAVMOOS_RUNTIME_STATUS', ''))
        caps = json.dumps(self.latest.get('MAVMOOS_CAPABILITIES', {}))
        capture = self.latest.get('MAVMOOS_CAMERA_CAPTURE_STATUS', {})
        camera_info = self.latest.get('MAVMOOS_CAMERA_INFORMATION', {})
        gimbal = self.latest.get('MAVMOOS_GIMBAL_MANAGER_STATUS', {})
        attitude = self.latest.get('MAVMOOS_GIMBAL_DEVICE_ATTITUDE_STATUS', {})
        return (state.get('mode') == 'SITL_CONTROL' and state.get('heartbeat_valid') == '1' and
                state.get('landed_valid') == '1' and state.get('armed') == '0' and state.get('in_air') == '0' and
                state.get('permission') == 'false' and state.get('control_profile') == 'none' and
                'camera.capture.single' in caps and 'gimbal.control.point' in caps and
                camera_info.get('fresh') and capture.get('fresh') and capture['data']['image_count'] == 1 and
                gimbal.get('metadata', {}).get('fresh') and
                any(row['metadata'].get('fresh') for row in attitude.get('devices', ())))


class PayloadLink:
    def __init__(self, endpoint, out, stack):
        self.endpoint = endpoint
        endpoint.setblocking(False)
        self.port = endpoint.getsockname()[1]
        self.peer = None
        self.decoder = mavlink.MAVLink(None)
        self.decoder.robust_parsing = True
        self.encoders = {identity: mavlink.MAVLink(None, srcSystem=identity[0], srcComponent=identity[1])
                         for identity in (CAMERA, GIMBAL)}
        self.camera = CameraFixture(self.encoders[CAMERA], out / 'camera')
        self.gimbal = GimbalFixture(mavlink, out)
        self.started = now()
        self.latest_fc = {}
        self.heartbeats = 0
        self.last_mode = None
        self.ground_observed = False
        self.commands = []
        self.drop_camera_ack = self.takeover_after_point = False
        self.rows = stack.enter_context((out / 'payload-wire.jsonl').open('x'))
        self.raw = {direction: stack.enter_context((out / f'payload-{direction}.mavlink').open('xb'))
                    for direction in ('rx', 'tx')}

    def boot(self):
        return int((now() - self.started) * 1000)

    def capture(self, direction, data):
        stream = self.raw[direction]
        at = stream.tell()
        need(at + len(data) <= MAX_CAPTURE_BYTES, 'payload wire byte limit')
        stream.write(data)
        return at

    def record(self, direction, frame, raw_at, length):
        row = {'direction': direction, 'mono': now(), 'datagram_offset': raw_at, 'datagram_length': length,
               'system': frame.get_srcSystem(), 'component': frame.get_srcComponent(), **frame.to_dict()}
        log(self.rows, row)
        return row

    def send(self, identity, messages):
        need(self.peer is not None, 'component route has no owned peer')
        encoder = self.encoders[identity]
        for frame in messages:
            wire = frame.pack(encoder)
            encoder.seq = (encoder.seq + 1) % 256
            need(self.endpoint.sendto(wire, self.peer) == len(wire), 'component write failed')
            self.record('tx', frame, self.capture('tx', wire), len(wire))

    def read(self, simulator, socket_proofs):
        for _ in range(64):
            try:
                data, peer = self.endpoint.recvfrom(65535)
            except BlockingIOError:
                return
            if self.peer is None:
                proof = inventory(simulator)
                need(exact_peer(proof, peer, self.port), 'component socket lacks exact native ownership')
                self.peer = peer
                log(socket_proofs, {'mono': now(), 'role': 'payload', 'peer': peer, 'inventory': proof})
            need(self.peer == peer, 'component socket peer changed')
            at = self.capture('rx', data)
            for frame in self.decoder.parse_buffer(data) or ():
                if frame.get_type() == 'BAD_DATA':
                    log(self.rows, {'direction': 'rx_rejected', 'mono': now(), 'datagram_offset': at,
                                    'datagram_length': len(data), 'reason': frame.reason})
                    continue
                row = self.record('rx', frame, at, len(data))
                identity = (frame.get_srcSystem(), frame.get_srcComponent())
                if identity == FC:
                    self.observe_fc(frame, row)
                elif identity == APP and frame.get_type() != 'HEARTBEAT':
                    self.accept_operation(frame, row)

    def observe_fc(self, frame, row):
        self.latest_fc[frame.get_type()] = row
        if frame.get_type() == 'HEARTBEAT':
            need(frame.autopilot == 3 and frame.type == 2 and not frame.base_mode & 128,
                 'independent native heartbeat changed profile or armed')
            if self.last_mode is not None:
                need(frame.custom_mode == self.last_mode, 'independent native heartbeat changed flight mode')
            self.last_mode = frame.custom_mode
            self.heartbeats += 1
        elif frame.get_type() == 'EXTENDED_SYS_STATE':
            need(frame.landed_state in (0, 1) and (not self.ground_observed or frame.landed_state == 1),
                 'independent native landed state changed')
            self.ground_observed |= frame.landed_state == 1

    def accept_operation(self, frame, row):
        need(frame.get_type() == 'COMMAND_LONG' and self.ground(), 'unexpected app effect on component route')
        target = (frame.target_system, frame.target_component)
        self.commands.append(row)
        boot = self.boot()
        if target == CAMERA:
            replies = self.camera.handle(frame, boot)
            need(len(replies) == 1 and replies[0].result == mavlink.MAV_RESULT_ACCEPTED, 'camera rejected wire command')
            if self.drop_camera_ack:
                self.drop_camera_ack = False
                log(self.rows, {'direction': 'fault', 'mono': now(), 'event': 'drop_camera_ack'})
            else:
                self.send(CAMERA, replies)
        elif target == GIMBAL:
            replies = self.gimbal.handle(frame, boot)
            need(len(replies) == 1 and replies[0].result == mavlink.MAV_RESULT_ACCEPTED, 'gimbal rejected wire command')
            if self.takeover_after_point and self.gimbal.commands[-1] == 'point':
                self.takeover_after_point = False
                self.gimbal.inject_takeover(boot)
            self.send(GIMBAL, replies)
        else:
            raise RuntimeError('unexpected wire target on component route')

    def ground(self):
        heartbeat = self.latest_fc.get('HEARTBEAT')
        landed = self.latest_fc.get('EXTENDED_SYS_STATE')
        return (heartbeat and landed and landed['landed_state'] == 1 and
                now() - min(heartbeat['mono'], landed['mono']) < 2)

    def advance(self):
        if self.peer:
            boot = self.boot()
            self.send(CAMERA, self.camera.periodic(boot))
            self.send(GIMBAL, self.gimbal.periodic(boot))


class Bench:
    def __init__(self, paths, firmware, out, stack, report):
        self.out, self.stack, self.report, self.paths = out, stack, report, paths
        self.simulator = self.antler = self.probe = self.probe_process = None
        self.bridge = Channel('bridge-prelude', stack.enter_context(reserve(socket.SOCK_DGRAM)), mavlink, stack, out)
        self.payload = PayloadLink(stack.enter_context(reserve(socket.SOCK_DGRAM)), out, stack)
        self.db = stack.enter_context(reserve(socket.SOCK_STREAM))
        self.base = stack.enter_context(reserve(socket.SOCK_STREAM))
        self.rc = stack.enter_context(reserve(socket.SOCK_DGRAM, '0.0.0.0'))
        self.irlock = stack.enter_context(reserve(socket.SOCK_DGRAM))
        self.ports = report['ports'] = {'bridge': self.bridge.port, 'payload': self.payload.port}
        self.ports.update({name: getattr(self, name).getsockname()[1] for name in ('db', 'base', 'rc', 'irlock')})
        self.proofs = stack.enter_context((out / 'ownership.jsonl').open('x'))
        params = firmware['fixture']['parameters'] | {'SERIAL0_PROTOCOL': 2, 'SERIAL1_PROTOCOL': 2}
        defaults = out / 'fixture.parm'
        defaults.write_text(''.join(f'{key} {value}\n' for key, value in params.items()))
        for channel in (0, 1):
            (out / f'message-intervals-chan{channel}.txt').write_text('0 1000\n245 250\n')
        report['fixture_parameters'] = params
        report['native_command'] = [str(paths['native']), '--model', 'quad',
            '--home', ','.join(map(str, firmware['fixture']['home'])), '--speedup', '1', '--wipe',
            '--defaults', f"{paths['defaults']},{defaults}",
            '--serial0', f"udpclient:127.0.0.1:{self.ports['bridge']}",
            '--serial1', f"udpclient:127.0.0.1:{self.ports['payload']}",
            '--serial2', 'none', '--serial5', 'none', '--serial6', 'none', '--serial7', 'none', '--serial8', 'none',
            '--base-port', str(self.ports['base']), '--rc-in-port', str(self.ports['rc']),
            '--irlock-port', str(self.ports['irlock'])]

    def start(self):
        for reserved in (self.base, self.rc, self.irlock):
            reserved.close()
        self.simulator = start(self.report['native_command'], self.out, 'sitl.log')
        self.report['sitl_pid'] = self.simulator.pid
        self.wait(25, lambda: self.bridge.peer and self.bridge.latest.get('HEARTBEAT') and self.payload.ground(),
                  'owned Copter routes or ground prelude unavailable')
        self.ports['bridge_peer'] = self.bridge.peer[1]
        self.ports['payload_peer'] = self.payload.peer[1]
        self.verify_routes()
        mission = self.out / 'payload.moos'
        mission.write_text(mission_text(self.ports, self.paths, self.simulator.pid))
        env = {key: value for key, value in os.environ.items()
               if not key.startswith(('DYLD_', 'LD_PRELOAD', 'LD_LIBRARY_PATH', 'MOOS'))}
        env['PATH'] = os.pathsep.join(map(str, (self.paths['app'].parent, self.paths['db'].parent, '/usr/bin', '/bin')))
        self.bridge.endpoint.close()
        self.db.close()
        self.antler = start([self.paths['antler'], mission], self.out, 'antler.log', env)
        with (self.out / 'probe.stderr').open('x') as stderr:
            self.probe_process = subprocess.Popen([str(self.paths['probe']), str(self.ports['db'])], cwd=self.out,
                env=env, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=stderr, start_new_session=True)
        self.probe = Probe(self.probe_process, self.stack, self.out)
        self.wait(15, lambda: self.probe.ready() and self.payload.ground(), 'owned MOOS payload readiness unavailable')

    def verify_routes(self):
        proof = inventory(self.simulator)
        need(exact_peer(proof, self.bridge.peer, self.bridge.port) and exact_peer(proof, self.payload.peer, self.payload.port),
             'an exact native route was lost')
        log(self.proofs, {'mono': now(), 'role': 'both', 'inventory': proof})

    def pump(self, duration):
        need(self.simulator.poll() is None, 'owned Copter exited')
        active = {self.payload.endpoint: lambda: self.payload.read(self.simulator, self.proofs)}
        if self.bridge.endpoint.fileno() != -1:
            active[self.bridge.endpoint] = lambda: self.bridge.read(self.simulator, self.proofs)
        if self.probe:
            need(self.antler.poll() is None and self.probe_process.poll() is None, 'owned MOOS process exited')
            active[self.probe_process.stdout] = self.probe.read
        for ready in select.select(active, (), (), duration)[0]:
            active[ready]()
        self.payload.advance()

    def wait(self, timeout, condition, reason):
        deadline = now() + timeout
        while now() < deadline:
            self.pump(min(.02, max(0, deadline - now())))
            value = condition()
            if value:
                return value
        raise RuntimeError(reason)

    def close(self):
        errors = []
        if self.probe_process:
            try:
                self.probe_process.stdin.close()
                try:
                    self.probe_process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    pass
            except OSError as error:
                errors.append(str(error))
            finally:
                self.probe_process.stdout.close()
        remaining = []
        for process in (self.probe_process, self.antler, self.simulator):
            try:
                remaining.extend(stop_process_tree(process))
            except Exception as error:
                errors.append(str(error))
        self.report['owned_processes_remaining'] = remaining
        self.report['process_cleanup_errors'] = errors


def exercise(bench):
    link, probe = bench.payload, bench.probe
    bench.verify_routes()
    started = now()
    probe.request(CAMERA_REQUEST, 'schema=1,id=payload-ok,action=capture')
    probe.request(GIMBAL_REQUEST, 'schema=1,id=payload-ok,action=point,pitch_deg=-30,yaw_deg=40')
    bench.wait(10, lambda: probe.terminal(CAMERA_RESULT, 'payload-ok', started) and
               probe.terminal(GIMBAL_RESULT, 'payload-ok', started), 'paired payload operation timed out')
    camera = probe.results(CAMERA_RESULT, 'payload-ok', started)
    gimbal = probe.results(GIMBAL_RESULT, 'payload-ok', started)
    pose = link.gimbal.snapshot()
    need([row['status'] for row in camera] == ['QUEUED', 'SENT', 'ACK_OBSERVED', 'CONFIRMED'] and
         camera[-1]['observed_image_index'] == 1 and camera[-1]['ack_result'] == 0 and
         link.camera.capture_count == 2 and sha(link.camera.captured_paths[0]) != sha(link.camera.captured_paths[1]),
         'camera confirmation lacks an independently written image')
    need(gimbal[-1]['status'] == 'CONFIRMED' and any(row['status'] == 'POINTED' for row in gimbal) and
         pose['commands'] == ['claim', 'point', 'release'] and link.gimbal.primary == (0, 0) and
         abs(pose['pitch_deg'] + 30) <= 2 and abs(pose['yaw_deg'] - 40) <= 2,
         'gimbal confirmation lacks independently observed pose and release')
    bench.report['confirmed'] = {'camera': camera, 'gimbal': gimbal, 'pose': pose}

    bench.verify_routes()
    started = now()
    probe.request(CAMERA_REQUEST, 'schema=1,id=payload-ok,action=capture')
    duplicate = bench.wait(3, lambda: probe.terminal(CAMERA_RESULT, 'payload-ok', started),
                           'duplicate camera request did not resolve')
    need(duplicate['status'] == 'REJECTED' and duplicate['reason'] == 'duplicate_or_history_full',
         'duplicate camera request passed')
    bench.wait(3, lambda: (probe.latest.get('MAVMOOS_CAMERA_CAPTURE_STATUS', {}).get('data') or {}).get('image_count') == 2,
               'camera idle count did not advance')
    link.drop_camera_ack = link.takeover_after_point = True
    started = now()
    probe.request(CAMERA_REQUEST, 'schema=1,id=missing-ack,action=capture')
    probe.request(GIMBAL_REQUEST, 'schema=1,id=takeover,action=point,pitch_deg=-50,yaw_deg=70')
    bench.wait(9, lambda: probe.terminal(CAMERA_RESULT, 'missing-ack', started) and
               probe.terminal(GIMBAL_RESULT, 'takeover', started), 'payload fault evidence timed out')
    camera_fault = probe.results(CAMERA_RESULT, 'missing-ack', started)
    gimbal_fault = probe.results(GIMBAL_RESULT, 'takeover', started)
    need([row['status'] for row in camera_fault] == ['QUEUED', 'SENT', 'CONFIRMED'] and
         camera_fault[-1]['reason'] == 'new_camera_image_observed' and camera_fault[-1]['ack_result'] is None and
         camera_fault[-1]['observed_image_index'] == 2 and link.camera.capture_count == 3 and
         link.camera.accepted_sequences == [1, 2] and not link.drop_camera_ack,
         'lost camera ACK prevented image confirmation or caused a retry')
    need(gimbal_fault[-1]['status'] == 'FAILED' and gimbal_fault[-1]['reason'] == 'manager_taken_over' and
         link.gimbal.commands == ['claim', 'point', 'release', 'claim', 'point'] and
         link.gimbal.primary == (42, 190) and not link.takeover_after_point,
         'takeover did not retire gimbal work or disturbed the foreign controller')
    need(len(link.commands) == 7 and link.heartbeats >= 3 and link.ground(),
         'unexpected command count or insufficient disarmed FC evidence')
    bench.report['faults'] = {'duplicate_camera': duplicate, 'missing_camera_ack': camera_fault,
                             'gimbal_takeover': gimbal_fault, 'gimbal_after_takeover': link.gimbal.snapshot()}
    bench.report['wire_command_count'] = len(link.commands)
    bench.report['disarmed_heartbeats'] = link.heartbeats
    bench.report['image_files'] = {str(path.relative_to(bench.out)): sha(path) for path in link.camera.captured_paths}


def ports_clear(ports):
    clear = {}
    for name, port in ports.items():
        tcp = name in ('db', 'base')
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM if tcp else socket.SOCK_DGRAM) as check:
            if tcp:
                check.settimeout(.2)
                clear[name] = check.connect_ex(('127.0.0.1', port)) == errno.ECONNREFUSED
                continue
            try:
                check.bind(('0.0.0.0' if name == 'rc' else '127.0.0.1', port))
                clear[name] = True
            except OSError:
                clear[name] = False
    return clear


def run(args):
    out = args.output
    out.mkdir(mode=0o700, parents=True, exist_ok=False)
    report = {'result': 'FAIL', 'source_revision': args.expected_app_revision, 'hardware': False,
              'scope': 'disarmed Copter SITL routing to simulated camera and gimbal', 'ports': {}}
    paths, before = {}, {}
    try:
        paths, before, firmware = preflight(args)
        report['firmware'] = firmware
        report['input_sha256'] = before
        for key in ('manifest', 'attestation'):
            copy = out / f'firmware-{key}.json'
            copy.write_bytes(paths[key].read_bytes())
            need(sha(copy) == before[key], f'archived firmware {key} differs')
        with ExitStack() as stack:
            bench = Bench(paths, firmware, out, stack, report)
            try:
                bench.start()
                exercise(bench)
            finally:
                bench.close()
        report['result'] = 'PASS'
    except BaseException as error:
        report['error'] = str(error) or type(error).__name__
        (out / 'failure.log').write_text(traceback.format_exc())
    try:
        report['ports_clear'] = ports_clear(report['ports'])
        report['cleanup_proven'] = bool(report['ports_clear']) and all(report['ports_clear'].values()) and \
            report.get('owned_processes_remaining') == [] and report.get('process_cleanup_errors') == []
        need(report['cleanup_proven'], 'owned teardown proof incomplete')
        report['input_sha256_after'] = {key: sha(path) for key, path in paths.items()}
        need(report['input_sha256_after'] == before and revision(ROOT) == args.expected_app_revision,
             'frozen input or source changed during run')
    except BaseException as error:
        report['result'] = 'FAIL'
        report['postflight_error'] = str(error) or type(error).__name__
    report['files'] = {str(file.relative_to(out)): {'bytes': file.stat().st_size, 'sha256': sha(file)}
                       for file in sorted(out.rglob('*')) if file.is_file() and not file.is_symlink()}
    (out / 'result.json').write_text(json.dumps(json_data(report), indent=2, allow_nan=False) + '\n')
    print(f"{report['result']} {out / 'result.json'}")
    return 0 if report['result'] == 'PASS' else 1


def interrupted(*_):
    raise InterruptedError('owned payload qualification interrupted')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('build', 'moos-root', 'ardupilot-root', 'firmware-attestation', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--expected-app-revision', required=True)
    arguments = parser.parse_args()
    for name in ('build', 'moos_root', 'ardupilot_root', 'firmware_attestation', 'output'):
        setattr(arguments, name, getattr(arguments, name).resolve())
    signal.signal(signal.SIGTERM, interrupted)
    os.umask(0o077)
    with (Path(tempfile.gettempdir()) / 'mavmoos-native-sitl.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.exit(1, 'The native SITL lease is already held.\n')
        raise SystemExit(run(arguments))
