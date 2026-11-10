#!/usr/bin/env python3
"""Qualify the owned disarmed Copter ExternalNav workflow through the real app."""
from contextlib import ExitStack
import argparse
import fcntl
import json
import os
import re
import signal
import subprocess
import sys
import tempfile
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'tests'), str(ROOT / 'scripts/firmware')]
from manifest import validate_manifest, verify_native
from native_fixture import (CopterBench, INPUT, PROFILE, REQUEST, SOURCE, STATUS, aiding, corroborate_log,
                            ekf_trace, gps_distance, last_gps, ports_clear, pose_trace)
from owned_processes import stop_process_tree
from plane_native.flight_fixture import MAX_CAPTURE_BYTES, json_data, log, need, now, revision, sha, start

MANIFEST = ROOT / 'config/firmware' / f'{PROFILE}.json'
MODULES = ('telemetry', 'external_navigation')


def preflight(args):
    manifest = validate_manifest(MANIFEST)
    firmware = verify_native(MANIFEST, args.firmware_attestation, args.ardupilot_root)
    need(firmware['profile_id'] == PROFILE and firmware['frame'] == 'quad' and not firmware['fixture']['hardware'],
         'exact disarmed Copter 4.6.3 quad fixture required')
    need(revision(ROOT) == args.expected_app_revision and
         revision(args.moos_root) == manifest['dependencies']['moos_revision'], 'clean exact MAVMOOS and stock MOOS required')
    cache = args.build / 'CMakeCache.txt'
    for key, path in (('CMAKE_HOME_DIRECTORY', ROOT), ('MOOS_IVP_ROOT', args.moos_root)):
        values = re.findall(rf'^{key}:[^=]+=([^\n]+)$', cache.read_text(), re.M)
        need(len(values) == 1 and Path(values[0]).resolve() == path, f'build provenance differs at {key}')
    paths = {
        'cache': cache, 'native': Path(firmware['native_path']),
        'defaults': args.ardupilot_root / firmware['fixture']['default_parameter_file'],
        'app': args.build / 'bin/iMAVLink', 'probe': args.build / 'bin/external_navigation_moos_probe',
        'antler': args.moos_root / 'bin/pAntler', 'db': args.moos_root / 'bin/MOOSDB',
        'helm': args.moos_root / 'bin/pHelmIvP',
        'manifest': MANIFEST, 'attestation': args.firmware_attestation,
    }
    for module in MODULES:
        paths[module] = args.build / 'plugins' / f'mavmoos_{module}.so'
    for name in ('tests/external_navigation/run_native.py', 'tests/external_navigation/native_fixture.py',
                 'tests/external_navigation/ExternalNavigationMoosProbe.cpp', 'tests/owned_processes.py',
                 'tests/plane_native/flight_fixture.py', 'scripts/firmware/manifest.py'):
        paths[name] = ROOT / name
    need(all(path.is_file() and not path.is_symlink() for path in paths.values()), 'frozen regular artifacts required')
    hashes = {name: sha(path) for name, path in paths.items()}
    need(hashes['native'] == firmware['native_sha256'] and hashes['manifest'] == firmware['manifest_sha256'] and
         hashes['attestation'] == firmware['attestation_sha256'], 'firmware inputs changed during preflight')
    return paths, hashes, firmware


def mission_text(ports, paths, pid):
    return f"""ServerHost = 127.0.0.1
ServerPort = {ports['db']}
Community = external_navigation_{ports['db']}
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
  plugin.external_navigation.source = {SOURCE}
  max_datagrams_per_iter = 128
}}
"""


def fields(value):
    return dict(field.split('=', 1) for field in value.split(',') if '=' in field)


class Probe:
    def __init__(self, process, stack, out):
        self.process = process
        self.records = []
        self.approved = set()
        self.latest = {}
        self.partial = b''
        os.set_blocking(process.stdout.fileno(), False)
        self.witness = stack.enter_context((out / 'moos.jsonl').open('x'))
        self.stdout = stack.enter_context((out / 'probe.stdout').open('xb'))

    def read(self):
        data = os.read(self.process.stdout.fileno(), 16384)
        need(bool(data) and self.stdout.tell() + len(data) <= MAX_CAPTURE_BYTES, 'MOOS probe stopped or byte limit')
        self.stdout.write(data)
        lines = (self.partial + data).split(b'\n')
        self.partial = lines.pop()
        need(len(self.partial) <= 4096, 'MOOS probe line limit')
        for line in lines:
            parts = line.lstrip(b'\r').decode().split('\t', 3)
            if len(parts) != 4 or parts[0] not in ('app', 'request'):
                continue
            kind, key, stamp, value = parts
            row = {'kind': kind, 'key': key, 'mono': float(stamp), 'value': value}
            log(self.witness, row)
            self.records.append(row)
            if kind == 'app':
                self.latest[key] = value

    def request(self, key, value):
        self.process.stdin.write(f'{key}\t{value}\n'.encode('ascii'))
        self.process.stdin.flush()

    def action(self, name, run):
        self.request(REQUEST, f'schema=1,action={name},run={run}')

    def sample(self, run, sequence, position, velocity, *, capture='now', approved=True):
        if approved:
            self.approved.add((run, sequence))
        self.request(INPUT, f'schema=1,run={run},seq={sequence},capture_us={capture},quality=90,'
                     f'frame=local_frd_body_frd,position={position:.7f}/0/0,velocity={velocity:.7f}/0/0,rates=0/0/0,'
                     'quaternion=1/0/0/0,position_var=.04/.04/.04,angle_var=.04/.04/.04,'
                     'velocity_var=.04/.04/.04,rate_var=.04/.04/.04')

    def samples(self):
        samples = []
        for record in self.records:
            if (record['kind'], record['key']) != ('request', INPUT):
                continue
            parsed = fields(record['value'])
            if (int(parsed['run']), int(parsed['seq'])) not in self.approved:
                continue
            row = {'capture_us': int(parsed['capture_us']), 'position': list(map(float, parsed['position'].split('/'))),
                   'velocity': list(map(float, parsed['velocity'].split('/')))}
            samples.append(row)
        return samples

    def status(self):
        return fields(self.latest.get(STATUS, ''))

    def ready(self):
        runtime = fields(self.latest.get('MAVMOOS_RUNTIME_STATUS', ''))
        caps = self.latest.get('MAVMOOS_CAPABILITIES', '')
        return (runtime.get('mode') == 'SITL_CONTROL' and runtime.get('heartbeat_valid') == '1' and
                runtime.get('landed_valid') == '1' and runtime.get('armed') == '0' and runtime.get('in_air') == '0' and
                runtime.get('permission') == 'false' and runtime.get('control_profile') == 'none' and
                'external_navigation.odometry.sitl' in caps and self.status().get('state') == 'IDLE')


class AppBench:
    def __init__(self, native, paths, out, stack):
        self.native, self.paths, self.out, self.stack = native, paths, out, stack
        self.antler = self.process = self.probe = None

    def launch(self):
        ports = self.native.ports
        mission = self.out / 'external-navigation.moos'
        mission.write_text(mission_text(ports, self.paths, self.native.native.pid))
        env = {key: value for key, value in os.environ.items()
               if not key.startswith(('DYLD_', 'LD_PRELOAD', 'LD_LIBRARY_PATH', 'MOOS'))}
        env['PATH'] = os.pathsep.join(map(str, (self.paths['app'].parent, self.paths['db'].parent, '/usr/bin', '/bin')))
        self.native.bridge.endpoint.close()
        self.native.db.close()
        self.antler = start([self.paths['antler'], mission], self.out, 'antler.log', env)
        with (self.out / 'probe.stderr').open('x') as stderr:
            self.process = subprocess.Popen([str(self.paths['probe']), str(ports['db'])], cwd=self.out,
                env=env, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=stderr, start_new_session=True)
        self.probe = Probe(self.process, self.stack, self.out)
        self.wait(10, lambda: self.probe.ready() and self.native.is_ground(), 'app did not report the owned disarmed bench')

    def pump(self, duration=.02):
        need(self.antler.poll() is None and self.process.poll() is None, 'owned MOOS process exited')
        self.native.pump(duration, {self.process.stdout: self.probe.read})
        need(self.native.is_ground(), 'fixture stopped providing disarmed ground evidence')
        runtime = fields(self.probe.latest.get('MAVMOOS_RUNTIME_STATUS', ''))
        need(runtime.get('permission', 'false') == 'false' and runtime.get('control_profile', 'none') == 'none',
             'estimator input acquired control authority')

    def wait(self, timeout, condition, reason):
        return self.native.wait(timeout, condition, reason, self.pump)

    def close(self):
        if self.process:
            self.process.stdin.close()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                pass
            finally:
                self.process.stdout.close()
        return stop_process_tree(self.process) + stop_process_tree(self.antler)


def exercise(native, app, report):
    probe = app.probe
    native.verify_routes()
    probe.sample(1, 900001, 19, 0, approved=False)
    app.wait(2, lambda: probe.status().get('last_rejection') == 'stale_or_inactive_run', 'input passed before begin')
    need(probe.status().get('sent') == '0', 'native handoff before explicit begin')
    probe.action('begin', 1)
    app.wait(2, lambda: probe.status().get('run') == '1', 'begin did not open the scoped lease')
    probe.sample(1, 900002, 19, 0, capture=1, approved=False)
    app.wait(2, lambda: probe.status().get('last_rejection') == 'invalid_or_stale_sample', 'stale capture was not fenced')

    started = next_send = now()
    start_ramp = settled = None
    gps = None
    sequence = 0
    while now()-started < 21:
        app.pump()
        t = now()
        poses = pose_trace(native.monitor, started)
        if start_ramp is None and poses and any(aiding(row) for row in ekf_trace(native.monitor, started)):
            gps = last_gps(native.monitor)
            if gps:
                start_ramp = t
                report['baseline_pose'] = poses[-1]
        x = 0 if start_ramp is None else min(6., t-start_ramp)
        vx = 0 if start_ramp is None or x >= 6 else 1.
        if t >= next_send:
            sequence += 1
            probe.sample(1, sequence, x, vx)
            next_send = t+.095
        if x >= 6 and poses and poses[-1]['x']-report['baseline_pose']['x'] >= 4.8:
            if settled is None:
                settled = t
            if t-settled > 1:
                report['shifted_pose'] = poses[-1]
                break
    need('shifted_pose' in report, 'native fused pose did not shift after MOOS input')
    report['stationary_gps_delta_m'] = gps_distance(gps, last_gps(native.monitor))
    need(report['stationary_gps_delta_m'] < 2.5, 'independent GPS moved during ExternalNav shift')

    silence = now()
    report['stream_retired'] = app.wait(1.5, lambda: probe.status()
        if probe.status().get('reason') == 'source_dropout' else None, 'app did not retire a silent producer')
    report['local_silence_retirement_s'] = now()-silence
    report['native_aiding_loss'] = app.wait(13, lambda: next(
        (row for row in ekf_trace(native.monitor, silence) if not aiding(row)), None),
        'FC did not independently report loss of horizontal aiding')
    report['native_aiding_loss_s'] = report['native_aiding_loss']['mono']-silence
    sent_at_dropout = int(report['stream_retired']['sent'])
    probe.action('begin', 1)
    app.wait(2, lambda: probe.status().get('last_rejection') == 'run_unavailable', 'retired run reopened')
    probe.sample(1, 900003, 19, 0, approved=False)
    app.wait(2, lambda: probe.status().get('last_rejection') == 'stale_or_inactive_run', 'old run sample passed after dropout')
    need(int(probe.status()['sent']) == sent_at_dropout, 'dropout produced additional local handoffs')

    probe.action('begin', 2)
    app.wait(2, lambda: probe.status().get('run') == '2', 'new explicit run denied')
    for sequence in range(1, 7):
        probe.sample(2, sequence, 6., 0.)
        app.wait(.45, lambda: int(probe.status().get('last_sequence', 0)) >= sequence,
                 'new run did not accept a fresh sample')
    app.wait(.45, lambda: int(probe.status().get('sent', 0)) > sent_at_dropout, 'new run never reached local socket')
    probe.action('stop', 2)
    report['explicit_stop'] = app.wait(1, lambda: probe.status()
        if probe.status().get('reason') == 'explicit_stop' else None, 'explicit stop did not retire the run')
    probe.sample(2, 900004, 19, 0, approved=False)
    app.wait(2, lambda: probe.status().get('last_rejection') == 'stale_or_inactive_run', 'stopped run accepted a new sample')
    need(probe.status()['sent'] == report['explicit_stop']['sent'], 'explicit stop produced additional local handoffs')
    report['local_send_count'] = int(probe.status()['sent'])
    report['ground_heartbeat_count'] = sum(row['mavpackettype'] == 'HEARTBEAT' for row in native.monitor.messages)
    need(report['ground_heartbeat_count'] >= 5 and native.is_ground(), 'insufficient independent disarmed observations')


def run(args):
    out = args.output
    out.mkdir(mode=0o700, parents=True, exist_ok=False)
    report = {'result': 'FAIL', 'scope': 'owned disarmed Copter ExternalNav through iMAVLink',
              'source_revision': args.expected_app_revision, 'hardware': False, 'ports': {}}
    paths, before, samples = {}, {}, []
    try:
        paths, before, firmware = preflight(args)
        report['firmware'] = firmware
        report['input_sha256'] = before
        for key in ('manifest', 'attestation'):
            copy = out / f'firmware-{key}.json'
            copy.write_bytes(paths[key].read_bytes())
            need(sha(copy) == before[key], f'archived firmware {key} differs')
        with ExitStack() as stack:
            native = CopterBench(out, paths, firmware, stack, report)
            app = AppBench(native, paths, out, stack)
            try:
                native.launch()
                app.launch()
                exercise(native, app, report)
            finally:
                if app.probe:
                    samples = app.probe.samples()
                try:
                    remaining = app.close()
                finally:
                    native.close()
                report['owned_processes_remaining'].extend(remaining)
        corroborate_log(out, samples, report)
        need(report['native_log']['pose_matches'] >= report['local_send_count']*.95,
             'too few local socket handoffs reached the pinned estimator')
        report['result'] = 'PASS'
    except BaseException as error:
        report['error'] = str(error) or type(error).__name__
        (out / 'failure.log').write_text(traceback.format_exc())
    try:
        report['ports_clear'] = ports_clear(report['ports'])
        report['cleanup_proven'] = bool(report['ports_clear']) and all(report['ports_clear'].values()) and \
            report.get('owned_processes_remaining') == []
        need(report['cleanup_proven'], 'owned teardown proof incomplete')
        report['input_sha256_after'] = {key: sha(path) for key, path in paths.items()}
        need(report['input_sha256_after'] == before and revision(ROOT) == args.expected_app_revision,
             'frozen input or source changed during run')
    except BaseException as error:
        report['result'] = 'FAIL'
        report['postflight_error'] = str(error) or type(error).__name__
    report['files'] = {str(file.relative_to(out)): {'bytes': file.stat().st_size, 'sha256': sha(file)}
                       for file in sorted(out.rglob('*')) if file.is_file() and not file.is_symlink()}
    (out / 'result.json').write_text(json.dumps(json_data(report), indent=2, allow_nan=False)+'\n')
    print(f"{report['result']} {out / 'result.json'}")
    return 0 if report['result'] == 'PASS' else 1


def interrupted(*_):
    raise InterruptedError('owned ExternalNav qualification interrupted')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('build', 'moos-root', 'ardupilot-root', 'firmware-attestation', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--expected-app-revision', required=True)
    options = parser.parse_args()
    for name in ('build', 'moos_root', 'ardupilot_root', 'firmware_attestation', 'output'):
        setattr(options, name, getattr(options, name).resolve())
    signal.signal(signal.SIGTERM, interrupted)
    os.umask(0o077)
    with (Path(tempfile.gettempdir())/'mavmoos-native-sitl.lock').open('a') as lock:
        try:
            fcntl.flock(lock,fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.exit(1, 'The native SITL lease is already held.\n')
        raise SystemExit(run(options))
