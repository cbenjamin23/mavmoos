#!/usr/bin/env python3
"""Bounded pAntler/MOOS/native Copter DISARM bench on direct owned loopback."""
import argparse
import fcntl
import hashlib
import importlib.util
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
import threading
import time
import traceback

import evidence

ROOT = Path(__file__).resolve().parents[2]
HELPERS = {
    'mission': 'missions/external_command/mission.py',
    'ownership': 'tests/owned_processes.py',
    'passive': 'tests/observation_load/passive_receiver.py',
    'wire': 'tests/imu/wire_evidence.py',
    'manifest': 'scripts/firmware/manifest.py',
}
SDK_FILES = {
    'api_sha256': 'src/libmavmoos_runtime/Plugin.h',
    'loader_header_sha256': 'src/libmavmoos_runtime/PluginHost.h',
    'loader_sha256': 'src/libmavmoos_runtime/PluginHost.cpp',
    'backend_header_sha256': 'src/libmavmoos_runtime/ControlBackend.h',
    'module_source_sha256': 'examples/disarmed_command/DisarmedCommand.cpp',
    'contracts_source_sha256': 'examples/disarmed_command/ContractTests.cpp',
}
INTERVALS = '1 500\n33 200\n245 500\n'
KIND_FIELDS = {kind: fields for kind, _, _, fields in evidence.MESSAGE_FIELDS.values() if kind != 'COMMAND_ACK'}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def git(path, *args):
    return subprocess.check_output(['git', '-C', str(path), *args], text=True, timeout=10).strip()


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def load(source, role, relative):
    path = (source / relative).resolve()
    if not path.is_file() or source not in path.parents:
        raise ValueError('missing selected-source helper ' + relative)
    spec = importlib.util.spec_from_file_location('_external_disarm_' + role, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def preflight(args):
    source, ap, moos, build = [Path(getattr(args, key)).resolve() for key in
                               ('source_root', 'ardupilot_root', 'moos_root', 'build')]
    if Path(__file__).resolve() != source / 'tests/external_command/run_native.py':
        raise ValueError('select the executing runner source explicitly')
    ref = git(source, 'rev-parse', 'HEAD')
    if git(source, 'status', '--porcelain', '--untracked-files=no') or (args.expected_source_ref and ref != args.expected_source_ref):
        raise ValueError('expected clean selected source revision')
    helpers = {role: load(source, role, relative) for role, relative in HELPERS.items()}
    firmware = helpers['ownership'].native_firmware_profile(source, ap, args.firmware_manifest, args.firmware_attestation)
    if not firmware or firmware['qualification'] != 'not_inferred' or firmware['frame'] != 'quad':
        raise ValueError('select an explicitly attested native quad profile')
    manifest = helpers['manifest'].validate_manifest(firmware['manifest_path'])
    fixture = manifest['fixture']
    parameters = fixture['parameters']
    if fixture['home'] != [42.358456, -71.087589, 10, 0] or any(parameters.get(name) != value for name, value in
        {'LOG_DISARMED': 1, 'SERIAL0_PROTOCOL': 2, 'SERIAL1_PROTOCOL': 2, 'SIM_WIND_SPD': 0}.items()):
        raise ValueError('native home, logging, protocol or wind fixture differs')
    moos_ref = git(moos, 'rev-parse', 'HEAD')
    if moos_ref != manifest['dependencies']['moos_revision'] or git(moos, 'status', '--porcelain', '--untracked-files=no'):
        raise ValueError('stock MOOS dependency differs from the manifest')
    app, probe, module = build / 'bin/iMAVLink', Path(args.probe).resolve(), Path(args.module).resolve()
    backend = build / 'plugins' / ('mavmoos_arducopter_control' + module.suffix)
    native = Path(firmware['native_path'])
    sdk = Path(args.module_inputs).resolve() if args.module_inputs else module.parents[1] / 'disarmed-command-inputs.json'
    artifacts = [app, probe, module, backend, native, sdk, *(moos / 'bin' / tool for tool in ('MOOSDB', 'pAntler', 'nsplug'))]
    if any(not path.is_file() or path.is_symlink() for path in artifacts) or not helpers['mission'].canonical_module(module):
        raise ValueError('required regular canonical app, example, probe or native artifact missing')
    for expected, path in ((args.expected_app_sha256, app), (args.expected_probe_sha256, probe),
                           (args.expected_module_sha256, module)):
        if expected and sha(path) != expected:
            raise ValueError('expected frozen artifact differs: ' + str(path))
    sdk_inputs = evidence.parse(sdk.read_bytes())
    exact_sdk = {key: sha(source / name) for key, name in SDK_FILES.items()}
    exact_sdk.update(schema=1, mavlink_revision=manifest['dependencies']['mavlink_revision'])
    if sdk_inputs != exact_sdk:
        raise ValueError('example consumer inputs differ from the selected host ABI, loader or source')
    protected = artifacts + [source / name for name in SDK_FILES.values()] + [Path(m.__file__) for m in helpers.values()]
    protected += helpers['ownership'].firmware_source_inputs(source, firmware)
    protected += [source / name for name in ('src/iMAVLink/MAVLink.cpp', 'src/libmavmoos_rx/UdpReceiver.cpp',
        'src/libmavmoos_runtime/Runtime.cpp', 'src/plugins/ArduCopterControlPlugin.cpp', 'src/libmavmoos_control/ArduCopterBackend.cpp',
        'missions/external_command/meta_command.moos', 'tests/external_command/DisarmSitlProbe.cpp',
        'tests/external_command/run_native.py', 'tests/external_command/evidence.py',
        'examples/disarmed_command/CMakeLists.txt', 'tests/external_command/CMakeLists.txt')]
    protected += [ap / 'Tools/autotest/default_params/copter.parm', ap / 'libraries/GCS_MAVLink/GCS_Common.cpp',
                  ap / 'libraries/AP_RCProtocol/AP_RCProtocol_UDP.cpp', helpers['ownership'].arducopter_gcs_source(ap)]
    before = {str(p): sha(p) for p in dict.fromkeys(protected)}
    return (source, ap, moos, build, ref, moos_ref, helpers, firmware, fixture, app, probe, module, backend, native, sdk, before)


def free_port(kind, address='127.0.0.1'):
    with socket.socket(socket.AF_INET, kind) as stream:
        stream.bind((address, 0))
        return stream.getsockname()[1]


def inventory(pid, selector='-iUDP'):
    answer = subprocess.run(['lsof', '-nP', '-a', '-p', str(pid), selector],
                            capture_output=True, text=True, timeout=3)
    if answer.returncode or not answer.stdout.strip():
        raise RuntimeError('exact owned socket inventory unavailable')
    return answer.stdout


def udp_peer(text, port, destination):
    return bool(re.search(rf'UDP\s+127\.0\.0\.1:{port}->127\.0\.0\.1:{destination}(?:\s|$)', text))


def owned_apps(ownership, antler, mission, app, moos, bridge, db, previous=None):
    result = {}
    expected = {'iMAVLink': app, 'MOOSDB': moos / 'bin/MOOSDB'}
    for row in ownership.owned_records(antler):
        try:
            command = shlex.split(row['command'])
        except ValueError:
            continue
        if not command or str(mission) not in command:
            continue
        name = Path(command[0]).name
        if row['pid'] == antler.pid and Path(command[0]) == moos / 'bin/pAntler':
            continue  # The owned inventory contains the launcher as well as its children.
        if name not in expected:
            raise RuntimeError('unexpected app in owned mission')
        # The generated pAntler roster uses an absolute app and stock MOOSDB
        # selected by the supervisor's explicit PATH.
        if (name in result or row['parent'] != antler.pid or
            (name == 'iMAVLink' and Path(command[0]) != expected[name])):
            raise RuntimeError('duplicate or wrong owned app executable')
        result[name] = row
    if set(result) != set(expected):
        raise RuntimeError('owned MOOSDB/iMAVLink startup incomplete')
    if previous and any(result[name]['pid'] != previous[name]['pid'] or
                        result[name]['command'] != previous[name]['command'] for name in expected):
        raise RuntimeError('owned app identity changed')
    app_udp = inventory(result['iMAVLink']['pid'])
    db_tcp = inventory(result['MOOSDB']['pid'], '-iTCP')
    if not re.search(rf'UDP\s+127\.0\.0\.1:{bridge}(?:\s|$)', app_udp) or not re.search(rf':{db}\s+\(LISTEN\)', db_tcp):
        raise RuntimeError('exact owned app/DB socket not ready')
    return result, {'app_udp': app_udp, 'db_tcp': db_tcp}


def clear_ports(ports):
    result = {}
    for name, port in ports.items():
        tcp = name in {'db', 'base'}
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM if tcp else socket.SOCK_DGRAM) as stream:
            if tcp:
                stream.settimeout(.2)
                result[name] = stream.connect_ex(('127.0.0.1', port)) != 0
            else:
                try:
                    stream.bind(('0.0.0.0' if name == 'rc' else '127.0.0.1', port))
                    result[name] = True
                except OSError:
                    result[name] = False
    return result


def run(args, prepared):
    source, ap, moos, build, ref, moos_ref, helpers, firmware, fixture, app, probe, module, backend, native, sdk, before = prepared
    os.environ['MAVLINK20'] = '1'
    from pymavlink import mavutil

    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    (out / 'logs').mkdir()
    copied = {}
    for role in ('manifest', 'attestation'):
        path = out / ('firmware-source.json' if role == 'manifest' else 'firmware-attestation.json')
        path.write_bytes(Path(firmware[role + '_path']).read_bytes())
        copied[role] = {'path': path.name, 'sha256': sha(path)}
        if copied[role]['sha256'] != firmware[role + '_sha256']:
            raise ValueError('copied selected firmware identity drift')
    sdk_copy = out / 'disarmed-command-inputs.json'
    sdk_copy.write_bytes(sdk.read_bytes())
    if sha(sdk_copy) != before[str(sdk)]:
        raise ValueError('copied consumer inputs drift')
    manifest_path, attestation_path = [out / copied[role]['path'] for role in ('manifest', 'attestation')]
    selected = helpers['manifest'].verify_native(manifest_path, attestation_path, ap)
    if any(firmware.get(key) != value for key, value in selected.items()):
        raise ValueError('copied selected firmware verification differs')
    before.update({str(p): sha(p) for p in (manifest_path, attestation_path, sdk_copy)})
    config = out / 'fixture.parm'
    config.write_text(''.join(f'{key} {value}\n' for key, value in fixture['parameters'].items()))
    intervals = out / 'message-intervals-chan1.txt'
    intervals.write_text(INTERVALS)
    before.update({str(p): sha(p) for p in (config, intervals)})
    result = {
        'result': 'FAIL', 'scenario': 'disarmed_command', 'profile': 'arducopter', 'hardware': False,
        'scope': 'one explicit non-forced disarm while already disarmed on ground; exact owned loopback SITL',
        'source_ref': ref, 'ardupilot_ref': firmware['source_revision'], 'moos_ref': moos_ref,
        'firmware_identity': firmware, 'firmware_inputs': copied,
        'app_sha256': before[str(app)], 'probe_sha256': before[str(probe)], 'native_sha256': before[str(native)],
        'example_inputs': {'path': sdk_copy.name, 'sha256': before[str(sdk_copy)], 'original_path': str(sdk)},
        'example_artifact': str(module),
        'example_build_kind': 'root_integrated' if module.parent == build / 'plugins' else 'standalone_consumer',
        'observer_writes': 0, 'rc_input_sent': False, 'relay_used': False, 'timewarp': 1,
        'fixture_parameters': fixture['parameters'], 'ports': {}, 'chronology': {}, 'artifacts_before': before,
        'owned_processes_remaining': None, 'cleanup_proven': False,
    }
    ownership = helpers['ownership']
    processes, sockets, handles, captures = [], [], [], []
    capture = None
    capture_closed = False
    ports = result['ports']
    chronology = result['chronology']

    def launch(argv, name, env=None):
        handle = (out / name).open('w')
        handles.append(handle)
        process = subprocess.Popen(argv, cwd=out, env=env, stdout=handle, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes.append(process)
        result.setdefault('processes', {})[name] = {'pid': process.pid, 'argv': [str(part) for part in argv]}
        return process

    def bind_udp(name):
        stream = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sockets.append(stream)
        stream.bind(('127.0.0.1', 0))
        stream.setblocking(False)
        ports[name] = stream.getsockname()[1]
        return stream

    try:
        primary, monitor = bind_udp('bridge'), bind_udp('monitor')
        for name, kind, address in (('db', socket.SOCK_STREAM, '127.0.0.1'),
                                    ('base', socket.SOCK_STREAM, '127.0.0.1'),
                                    ('rc', socket.SOCK_DGRAM, '0.0.0.0')):
            port = free_port(kind, address)
            if port in ports.values():
                raise RuntimeError('selected fixture ports overlap')
            ports[name] = port
        defaults = str(ap / 'Tools/autotest/default_params/copter.parm') + ',' + str(config)
        command = [str(native), '--model', 'quad', '--home', '42.358456,-71.087589,10,0', '--speedup', '1',
                   '--wipe', '--defaults', defaults, '--serial0', f'udpclient:127.0.0.1:{ports["bridge"]}',
                   '--serial1', f'udpclient:127.0.0.1:{ports["monitor"]}', '--serial2', 'none', '--serial5', 'none',
                   '--serial6', 'none', '--serial7', 'none', '--serial8', 'none',
                   '--base-port', str(ports['base']), '--rc-in-port', str(ports['rc'])]
        result['command'] = command
        sitl = launch(command, 'sitl.log')
        ready = threading.Event()
        first_ready_at = []
        selected_kinds = set()
        decoder = helpers['wire'].FrameDecoder(mavutil.mavlink.MAVLink(None))
        raw = (out / 'fc-observer.mavlink').open('wb')
        trace = (out / 'fc-observer.jsonl').open('w')
        startup = (out / 'fc-observer-startup.jsonl').open('w')
        captures.extend((raw, trace, startup))

        def verify(sender):
            text = inventory(sitl.pid)
            if not udp_peer(text, sender[1], ports['monitor']):
                raise RuntimeError('exact owned passive FC peer differs')
            return text

        def consume(data, _sender, at):
            raw.write(data)
            rows, prefix = decoder.feed(data, at)
            for row in prefix:
                startup.write(json.dumps(row, allow_nan=False) + '\n')
            for row in rows:
                kind = row['mavpackettype']
                if kind not in KIND_FIELDS:
                    continue
                sample = {'mono': at, 'kind': kind, 'system': row['system'], 'component': row['component'],
                          **{field: row[field] for field in KIND_FIELDS[kind]}}
                trace.write(json.dumps(sample, allow_nan=False) + '\n')
                if (kind == 'HEARTBEAT' or
                    kind == 'EXTENDED_SYS_STATE' and row['landed_state'] == 1 or
                    kind == 'GLOBAL_POSITION_INT' and evidence.at_fixture_home(row)):
                    selected_kinds.add(kind)
                if not first_ready_at and len(selected_kinds) == len(KIND_FIELDS):
                    first_ready_at.append(at)
                    ready.set()

        capture = helpers['passive'].PassiveReceiver(monitor, verify, consume, max_datagrams=30000)
        capture.start()
        discovery = mavutil.mavlink.MAVLink(None)
        discovery.robust_parsing = True
        deadline = time.monotonic() + 20
        peer = None
        with (out / 'primary-discovery.mavlink').open('wb') as stream:
            datagrams = 0
            while time.monotonic() < deadline:
                capture.check()
                if sitl.poll() is not None:
                    raise RuntimeError('owned native startup exited')
                try:
                    data, sender = primary.recvfrom(65535)
                except BlockingIOError:
                    time.sleep(.005)
                    continue
                datagrams += 1
                if datagrams > 5000:
                    raise RuntimeError('primary discovery datagram bound exceeded')
                stream.write(data)
                if sender[0] == '127.0.0.1' and any(m.get_type() == 'HEARTBEAT' and
                    (m.get_srcSystem(), m.get_srcComponent(), m.type, m.autopilot) == (1, 1, 2, 3) and
                    not m.base_mode & 128 for m in discovery.parse_buffer(data) or ()):
                    peer = sender
                    break
        if peer is None or peer[1] in ports.values():
            raise RuntimeError('unique native primary endpoint not discovered')
        ports['peer'] = peer[1]
        while time.monotonic() < deadline:
            capture.check()
            if sitl.poll() is not None:
                raise RuntimeError('owned native exited before socket proof')
            proof = inventory(sitl.pid)
            now = time.monotonic()
            if (now < deadline and ready.is_set() and now >= first_ready_at[0] + 1.6 and
                udp_peer(proof, peer[1], ports['bridge']) and re.search(
                    rf'UDP\s+(?:\*|0\.0\.0\.0):{ports["rc"]}(?:\s|$)', proof)):
                chronology['passive_ready'] = first_ready_at[0]
                result['native_primary_inventory'] = {'at': now, 'original_deadline': deadline, 'lsof': proof}
                break
            time.sleep(.02)
        else:
            raise RuntimeError('native primary/passive/RC readiness unproven by original startup deadline')
        ports['monitor_peer'] = capture.peer[1]
        if len(set(ports.values())) != len(ports):
            raise RuntimeError('native peer ports overlap selected fixture')
        result['native_monitor_inventory'] = capture.peer_proof
        mission, stage, generation = helpers['mission'].generate(build=build, module=module, moos=moos,
            work=out / 'mission', db=ports['db'], bind=ports['bridge'], peer=ports['peer'], mode='sitl',
            pid=sitl.pid, request_source=evidence.SOURCE)
        loaded = {path.name: sha(stage / path.name) for path in (backend, module)}
        if any(loaded[path.name] != before[str(path)] for path in (backend, module)):
            raise RuntimeError('staged external module identity differs')
        result['generation_command'] = generation
        result['mission_sha256'] = sha(mission)
        before.update({str(p): sha(p) for p in (mission, stage / backend.name, stage / module.name)})
        env = dict(os.environ, PATH=os.pathsep.join((str(moos / 'bin'), os.environ.get('PATH', ''))))
        primary.close()
        antler = launch([str(moos / 'bin/pAntler'), str(mission)], 'pantler.log', env)
        deadline = time.monotonic() + 10
        startup_error = None
        while time.monotonic() < deadline:
            capture.check()
            if sitl.poll() is not None or antler.poll() is not None:
                raise RuntimeError('owned native or pAntler exited before probe')
            try:
                apps, socket_proof = owned_apps(ownership, antler, mission, app, moos, ports['bridge'], ports['db'])
            except RuntimeError as error:
                startup_error = error
                time.sleep(.05)
                continue
            if time.monotonic() >= deadline:
                raise RuntimeError('owned app readiness proof crossed startup deadline')
            result['owned_apps'] = apps
            result['app_socket_inventory'] = socket_proof
            break
        else:
            raise RuntimeError('owned MOOSDB/iMAVLink startup deadline') from startup_error
        chronology['probe_started'] = time.monotonic()
        driver = launch([str(probe), str(ports['db'])], 'probe.jsonl', env)
        deadline = chronology['probe_started'] + 55
        next_inventory = chronology['probe_started'] + .5
        with (out / 'ownership.jsonl').open('w') as receipts:
            while time.monotonic() < deadline:
                capture.check()
                if sitl.poll() is not None or antler.poll() is not None:
                    raise RuntimeError('owned native or pAntler exited during probe')
                if driver.poll() is not None:
                    if 'probe_exit_observed' not in chronology:
                        chronology['probe_exit_observed'] = time.monotonic()
                        result['probe_exit'] = driver.returncode
                    if driver.returncode or time.monotonic() >= chronology['probe_exit_observed'] + 1.5:
                        break
                if time.monotonic() >= next_inventory:
                    _, observed = owned_apps(ownership, antler, mission, app, moos, ports['bridge'], ports['db'], apps)
                    receipts.write(json.dumps({'mono': time.monotonic(), 'apps': apps, 'sockets': observed}) + '\n')
                    receipts.flush()
                    next_inventory = time.monotonic() + .5
                time.sleep(.005)
            else:
                raise RuntimeError('external command probe/capture exceeded original supervisor deadline')
        if result.get('probe_exit') != 0:
            raise RuntimeError('owned external command MOOS probe failed')
        result['final_owned_apps'], result['final_app_socket_inventory'] = owned_apps(
            ownership, antler, mission, app, moos, ports['bridge'], ports['db'], apps)
        capture.close()
        capture_closed = True
        chronology['capture_finished'] = time.monotonic()
        result['wire_capture'] = decoder.finish()
        for stream in captures:
            stream.close()
        result['command_evidence'] = evidence.audit(result, lambda name: (out / name).read_bytes())
        result['loaded_modules_sha256'] = loaded
        result['result'] = 'PASS'
    except BaseException as error:
        result['error'] = str(error) or type(error).__name__
        result['traceback'] = traceback.format_exc()
    finally:
        errors = []
        if capture is not None:
            try:
                if not capture_closed:
                    capture.close()
            except BaseException as error:
                errors.append('passive capture: ' + str(error))
            result['capture'] = capture.status()
        retired = capture is None or result['capture']['thread_retired'] is True
        if not retired:
            errors.append('passive capture has not retired')
        remaining = []
        for process in reversed(processes):
            try:
                left = ownership.stop_process_tree(process)
                if type(left) is not list:
                    raise RuntimeError('unknown owned process inventory')
                remaining.extend(left)
            except BaseException as error:
                errors.append('owned process cleanup: ' + str(error))
        for stream in sockets + handles + (captures if retired else []):
            try:
                stream.close()
            except BaseException as error:
                errors.append('owned stream cleanup: ' + str(error))
        try:
            result['ports_clear'] = clear_ports(ports)
        except BaseException as error:
            result['ports_clear'] = {}
            errors.append('owned port cleanup: ' + str(error))
        clean_capture = (capture is not None and retired and result['capture']['socket_closed'] is True
                         and result['capture']['error'] is None)
        clean_ports = set(result['ports_clear']) == evidence.PORTS and all(result['ports_clear'].values())
        result['owned_processes_remaining'] = remaining if not errors else None
        result['cleanup_proven'] = not errors and not remaining and clean_capture and clean_ports
        if not result['cleanup_proven']:
            result['result'] = 'FAIL'
            result['cleanup_errors'] = errors or ['receiver, process or port absence unproven']
        try:
            result['source_ref_after'] = git(source, 'rev-parse', 'HEAD')
            result['moos_ref_after'] = git(moos, 'rev-parse', 'HEAD')
            result['firmware_identity_after'] = ownership.native_firmware_profile(
                source, ap, args.firmware_manifest, args.firmware_attestation)
            if (result['source_ref_after'] != ref or git(source, 'status', '--porcelain', '--untracked-files=no') or
                result['moos_ref_after'] != moos_ref or git(moos, 'status', '--porcelain', '--untracked-files=no') or
                result['firmware_identity_after'] != firmware or
                helpers['manifest'].verify_native(manifest_path, attestation_path, ap) != selected):
                raise RuntimeError('source, dependency or selected/copied native identity drift')
            result['artifacts_after'] = {path: sha(path) for path in before}
            result['artifact_drift'] = sorted(path for path in before if before[path] != result['artifacts_after'][path])
            if result['artifact_drift']:
                raise RuntimeError('frozen file bytes changed')
        except BaseException as error:
            result['postflight_error'] = str(error) or type(error).__name__
            result['result'] = 'FAIL'
        unfinalized = set() if retired else {Path(stream.name) for stream in captures}
        result['files'] = {str(path.relative_to(out)): sha(path) for path in sorted(out.rglob('*'))
                           if path.is_file() and path not in unfinalized and path.name != 'result.json'}
        if result['result'] == 'PASS':
            try:
                evidence.replay_archived(result, lambda name: (out / name).read_bytes(), expected_abi=10)
            except BaseException as error:
                result['result'] = 'FAIL'
                result['offline_replay_error'] = str(error) or type(error).__name__
        dump(out / 'result.json', result)
    print(json.dumps({key: result.get(key) for key in
        ('result', 'scenario', 'error', 'probe_exit', 'postflight_error', 'offline_replay_error', 'cleanup_proven',
         'owned_processes_remaining', 'ports_clear', 'artifact_drift')}, indent=2))
    return int(result['result'] != 'PASS')


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    for key in ('ardupilot-root', 'moos-root', 'build', 'probe', 'module', 'output'):
        result.add_argument('--' + key, required=True)
    result.add_argument('--source-root', default=str(ROOT))
    for key in ('module-inputs', 'firmware-manifest', 'firmware-attestation', 'expected-source-ref',
                'expected-app-sha256', 'expected-probe-sha256', 'expected-module-sha256'):
        result.add_argument('--' + key)
    result.add_argument('--verify-only', action='store_true', help='check source/artifact identities without sockets or processes')
    return result


if __name__ == '__main__':
    options = parser().parse_args()
    try:
        prepared = preflight(options)
        if options.verify_only:
            print(json.dumps({'result': 'PASS', 'source_ref': prepared[4], 'firmware_identity': prepared[7],
                              'artifacts': prepared[-1]}, indent=2))
            raise SystemExit(0)
        signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(143)))
        with (Path(tempfile.gettempdir()) / 'mavmoos-native-sitl.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            raise SystemExit(run(options, prepared))
    except (ValueError, OSError, RuntimeError, subprocess.SubprocessError) as error:
        print('Disarm fixture: ' + str(error), file=sys.stderr)
        raise SystemExit(1)
