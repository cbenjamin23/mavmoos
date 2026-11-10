#!/usr/bin/env python3
"""One owned, disarmed Copter 4.6.3 receive-only observer case.

The native telemetry socket feeds an independent byte audit and a separate app peer.
The simulator's serial0 console has a separate receive-only socket.
The peer can send only to the proved local app socket. Its incoming bytes are
captured as egress evidence and never forwarded to the simulator.
"""
import argparse
from contextlib import ExitStack
import errno
import fcntl
import json
import os
from pathlib import Path
import re
import signal
import socket
import stat
import subprocess
import sys
import tempfile
import threading
import time
import traceback

from native_audit import audit_forwarded, need, status_evidence
from run_loopback import audit, mission_text, reserve, start, take_egress
from imu.audit_native import fields
from imu.run_native import dump, exact_peer, inventory, live_app, load_helpers, revision, sha
from status.DiagnosticsEvidence import audit_native as audit_native_diagnostics
from traffic.audit import KEY as TRAFFIC_KEY, audit_native as audit_native_traffic


ROOT = Path(__file__).resolve().parents[2]
PROFILE = 'arducopter-4.6.3-quad'
PLUGINS = ('telemetry', 'odometry', 'air_data', 'esc_status')
INTERVALS = '32 200\n74 200\n11030 200\n245 500\n'
STATUS_INTERVALS = '125 200\n193 200\n'
TRAFFIC_FIXTURE = {
    'SERIAL2_PROTOCOL': 2, 'ADSB_TYPE': 1, 'ADSB_LIST_RADIUS': 0, 'ADSB_ICAO_ID': -1,
    'SIM_ADSB_TYPES': 1, 'SIM_ADSB_COUNT': 5, 'SIM_ADSB_RADIUS': 2000, 'SIM_ADSB_ALT': 120,
    'SIM_ADSB_TX': 0, 'AVD_ENABLE': 0,
}
PROBE_SECONDS = 14
MAX_WIRE_BYTES = 8 * 1024 * 1024


def clean_source(path):
    change = subprocess.check_output(
        ['git', '-C', str(path), 'status', '--porcelain', '--untracked-files=normal'],
        text=True, timeout=10)
    need(not change.strip(), 'selected source checkout has changes: ' + str(path))
    return revision(path)


def preflight(args, plugins):
    source = args.source_root.resolve()
    need(source == ROOT, 'runner must belong to the explicitly selected source')
    imported = {'run_loopback': 'tests/observer_matrix/run_loopback.py',
                'native_audit': 'tests/observer_matrix/native_audit.py',
                'imu.run_native': 'tests/imu/run_native.py', 'imu.audit_native': 'tests/imu/audit_native.py',
                'status.DiagnosticsEvidence': 'tests/status/DiagnosticsEvidence.py',
                'traffic.audit': 'tests/traffic/audit.py'}
    for name, relative in imported.items():
        need(Path(sys.modules[name].__file__).resolve() == source / relative,
             'imported helper outside selected source: ' + name)
    helpers = load_helpers(source)
    ap, moos, build = (path.resolve() for path in (args.ardupilot_root, args.moos_root, args.build))
    firmware = helpers['ownership'].native_firmware_profile(
        source, ap, args.firmware_manifest, args.firmware_attestation)
    need(firmware and firmware['profile_id'] == PROFILE and firmware['frame'] == 'quad' and
         firmware['manifest_sha256'] == sha(source / 'config/firmware' / (PROFILE + '.json')),
         'exact pinned stable quad manifest and native attestation required')
    manifest = helpers['manifest'].validate_manifest(Path(firmware['manifest_path']))
    moos_ref, source_ref = clean_source(moos), clean_source(source)
    need(moos_ref == manifest['dependencies']['moos_revision'], 'stock MOOS dependency pin differs')
    cache = build / 'CMakeCache.txt'
    text = cache.read_text()
    for key, path in (('CMAKE_HOME_DIRECTORY', source), ('MOOS_IVP_ROOT', moos)):
        configured = re.search(r'^' + key + r':[^=]+=([^\n]+)$', text, re.M)
        need(configured and Path(configured[1]).resolve() == path,
             'configured build path differs from selected ' + key)
    paths = {
        'native': Path(firmware['native_path']),
        'app': build / 'bin/iMAVLink',
        'probe': args.probe.absolute() if args.probe else build / 'bin/observer_matrix_probe',
        'antler': moos / 'bin/pAntler',
        'db': moos / 'bin/MOOSDB',
        'helm': moos / 'bin/pHelmIvP',
        'defaults': ap / firmware['fixture']['default_parameter_file'],
    }
    modules = [build / 'plugins' / f'mavmoos_{name}.so' for name in plugins]
    need(all(path.is_file() and not path.is_symlink() for path in [*paths.values(), *modules]),
         'required regular artifact missing or symlinked')
    for role, expected in (('app', args.expected_app_sha256), ('probe', args.expected_probe_sha256)):
        if expected:
            need(re.fullmatch('[0-9a-f]{64}', expected) and sha(paths[role]) == expected,
                 'frozen ' + role + ' digest differs')
    protected = [cache, *paths.values(), *modules]
    protected.extend(source / name for name in (
        'tests/observer_matrix/Probe.cpp', 'tests/observer_matrix/run_native.py',
        'tests/observer_matrix/native_audit.py', 'tests/observer_matrix/run_loopback.py',
        'src/libmavmoos_runtime/PassiveObserver.h', 'src/plugins/OdometryPlugin.cpp',
        'src/plugins/AirDataPlugin.cpp', 'src/plugins/EscStatusPlugin.cpp',
    ))
    protected.extend(Path(module.__file__) for module in helpers.values())
    protected.extend(source / relative for relative in imported.values())
    protected.extend(helpers['ownership'].firmware_source_inputs(source, firmware))
    protected.extend(ap / name for name in (
        'libraries/AP_ESC_Telem/AP_ESC_Telem.cpp',
        'libraries/AP_ESC_Telem/AP_ESC_Telem_SITL.cpp',
        'libraries/AP_HAL_SITL/RCOutput.cpp', 'libraries/SITL/SITL.cpp',
        'libraries/GCS_MAVLink/GCS_Common.cpp',
        'libraries/AP_Vehicle/AP_Vehicle.cpp', 'libraries/AP_Vehicle/Parameters.cpp',
    ))
    if args.traffic:
        protected.append(source / 'src/plugins/TrafficPlugin.cpp')
        protected.extend(ap / name for name in (
            'ArduCopter/GCS_Mavlink.cpp', 'libraries/AP_ADSB/AP_ADSB.cpp',
            'libraries/GCS_MAVLink/MAVLink_routing.cpp', 'libraries/SITL/SIM_ADSB.cpp',
            'libraries/AP_HAL_SITL/SITL_State_common.cpp',
            'libraries/AP_HAL_SITL/UARTDriver.cpp',
        ))
    if args.status_diagnostics:
        protected.append(source / 'src/plugins/SystemStatusPlugin.cpp')
        protected.extend(ap / name for name in (
            'libraries/AP_HAL/AnalogIn.h', 'libraries/AP_HAL/AP_HAL_Boards.h',
            'libraries/AP_HAL_SITL/AnalogIn.h', 'libraries/AP_NavEKF3/AP_NavEKF3_Outputs.cpp',
        ))
    return source, ap, moos, build, helpers, firmware, source_ref, moos_ref, paths, protected


def copy_firmware(out, firmware, verifier, ap):
    copied = {}
    for key, name in (('manifest', 'firmware-source.json'), ('attestation', 'firmware-attestation.json')):
        raw = Path(firmware[key + '_path']).read_bytes()
        path = out / name
        with path.open('xb') as stream:
            stream.write(raw)
        digest = sha(path)
        need(digest == firmware[key + '_sha256'], 'verified firmware input changed before copy')
        copied[key] = {'file': name, 'sha256': digest}
    identity = verifier.verify_native(out / copied['manifest']['file'],
                                      out / copied['attestation']['file'], ap)
    need(all(firmware.get(key) == value for key, value in identity.items()), 'copied firmware differs')
    return copied, identity


def clean_ports(ports):
    result = {}
    for name, port in ports.items():
        tcp = name in ('moos', 'native_base')
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM if tcp else socket.SOCK_DGRAM) as check:
            if tcp:
                check.settimeout(.2)
                result[name] = check.connect_ex(('127.0.0.1', port)) == errno.ECONNREFUSED
            else:
                try:
                    check.bind(('0.0.0.0' if name == 'rc' else '127.0.0.1', port))
                    result[name] = True
                except OSError:
                    result[name] = False
    return result


def exercise(out, build, moos, paths, helpers, firmware, decoder_factory, report, plugins):
    processes, remaining, errors = [], [], []
    capture = None
    capture_finished = False
    packets = []
    decoder = helpers['wire'].FrameDecoder(decoder_factory())
    forward_byte = forward_row = None
    report['bridge_egress_datagrams'] = 0
    report['forwarded_datagrams'] = report['forwarded_bytes'] = 0
    report['ports'] = ports = {}
    ready = threading.Event()
    traffic = 'traffic' in plugins
    status_diagnostics = 'system_status' in plugins

    def launch(command, name, env=None):
        process = start(command, out, env, name)
        processes.append(process)
        return process

    with ExitStack() as stack:
        console_rx = stack.enter_context(reserve(socket.SOCK_DGRAM))
        console_rx.settimeout(.5)
        native_rx = stack.enter_context(reserve(socket.SOCK_DGRAM))
        app_peer = stack.enter_context(reserve(socket.SOCK_DGRAM))
        db_socket = stack.enter_context(reserve(socket.SOCK_STREAM))
        app_socket = stack.enter_context(reserve(socket.SOCK_DGRAM))
        base_socket = stack.enter_context(reserve(socket.SOCK_STREAM))
        rc_socket = stack.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
        rc_socket.bind(('0.0.0.0', 0))
        native_rx.setblocking(False)
        app_peer.setblocking(False)
        ports.update(console_rx=console_rx.getsockname()[1], native_rx=native_rx.getsockname()[1],
                     app_peer=app_peer.getsockname()[1],
                     moos=db_socket.getsockname()[1], bridge=app_socket.getsockname()[1],
                     native_base=base_socket.getsockname()[1], rc=rc_socket.getsockname()[1])
        egress = stack.enter_context((out / 'bridge-egress.mavlink').open('xb'))
        raw = stack.enter_context((out / 'native-wire.mavlink').open('xb'))
        decoded = stack.enter_context((out / 'native-wire.jsonl').open('x'))
        startup = stack.enter_context((out / 'native-startup.jsonl').open('x'))
        datagrams = stack.enter_context((out / 'native-datagrams.jsonl').open('x'))

        def drain_app(expect_empty=True):
            report['bridge_egress_datagrams'] += take_egress(app_peer, ports['bridge'], egress)
            if expect_empty:
                need(report['bridge_egress_datagrams'] == 0, 'app emitted a datagram to its configured peer')

        def prove_peer(sender):
            proof = inventory(sitl)
            need(sender[0] == '127.0.0.1' and exact_peer(proof, sender[1], ports['native_rx']),
                 'native UDP source is not the exact owned simulator')
            return proof

        def consume(data, _sender, at):
            nonlocal forward_byte, forward_row
            offset = decoder.input_bytes
            need(offset + len(data) <= MAX_WIRE_BYTES, 'native wire evidence exceeds byte bound')
            # The independently replayable app suffix starts at a complete
            # binary boundary after serial1 startup traffic.
            if ready.is_set() and forward_byte is None and decoder.frames and offset == decoder.consumed_bytes:
                forward_byte, forward_row = offset, len(packets)
            raw.write(data)
            record = {'offset': offset, 'length': len(data), 'mono': at, 'forwarded': False}
            try:
                rows, prefix = decoder.feed(data, at)
            except Exception as error:
                raw.flush()
                record['decode_error'] = str(error)
                datagrams.write(json.dumps(record) + '\n')
                datagrams.flush()
                raise
            for row in rows:
                packets.append(row)
                decoded.write(json.dumps(row, allow_nan=False) + '\n')
            for row in prefix:
                startup.write(json.dumps(row, allow_nan=False) + '\n')
            forwarded = forward_byte is not None
            if forwarded:
                sent = app_peer.sendto(data, ('127.0.0.1', ports['bridge']))
                need(sent == len(data), 'incomplete one-way app datagram handoff')
                report['forwarded_datagrams'] += 1
                report['forwarded_bytes'] += sent
            record['forwarded'] = forwarded
            datagrams.write(json.dumps(record) + '\n')

        try:
            parameters = {**firmware['fixture']['parameters'], 'SIM_ESC_TELEM': 1,
                          **(TRAFFIC_FIXTURE if traffic else {})}
            report['fixture_parameters'] = parameters
            defaults = out / 'fixture.parm'
            defaults.write_text(''.join(f'{key} {value}\n' for key, value in parameters.items()))
            # Copter's SITL UART driver reserves SERIAL1 when SIM_ADSB_COUNT
            # is nonnegative. Put the passive FC output on channel 2 then.
            channel = 2 if traffic else 1
            intervals = INTERVALS + ('246 100\n' if traffic else '') + (STATUS_INTERVALS if status_diagnostics else '')
            (out / f'message-intervals-chan{channel}.txt').write_text(intervals)
            telemetry = f"udpclient:127.0.0.1:{ports['native_rx']}"
            serial1, serial2 = ('sim:adsb', telemetry) if traffic else (telemetry, 'none')
            report['native_telemetry_channel'] = channel
            home = ','.join(map(str, firmware['fixture']['home']))
            command = [str(paths['native']), '--model', 'quad', '--home', home, '--speedup', '1',
                       '--wipe', '--defaults', str(paths['defaults']) + ',' + str(defaults),
                       '--serial0', f"udpclient:127.0.0.1:{ports['console_rx']}",
                       '--serial1', serial1, '--serial2', serial2, '--serial5', 'none',
                       '--serial6', 'none', '--serial7', 'none', '--serial8', 'none',
                       '--base-port', str(ports['native_base']), '--rc-in-port', str(ports['rc'])]
            report['native_command'] = command
            base_socket.close()
            rc_socket.close()
            sitl = launch(command, 'sitl.log')
            report['sitl_pid'] = sitl.pid
            capture = helpers['passive'].PassiveReceiver(native_rx, prove_peer, consume, max_datagrams=10000)
            capture.start()
            started = time.monotonic()
            # A GPS origin alone does not establish a valid NED estimate.
            # Wait for the actual disarmed producer before timing the app probe.
            deadline = started + 60
            required_families = {'HEARTBEAT', 'LOCAL_POSITION_NED'}
            if status_diagnostics:
                required_families.update(('POWER_STATUS', 'EKF_STATUS_REPORT'))
            while time.monotonic() < deadline:
                capture.check()
                need(sitl.poll() is None, 'native process exited during passive startup')
                families = {row['mavpackettype'] for row in packets}
                adsb_producers = {row['ICAO_address'] for row in packets if row['mavpackettype'] == 'ADSB_VEHICLE'}
                if capture.peer and required_families <= families and (
                        not traffic or len(adsb_producers) >= 2):
                    break
                time.sleep(.05)
            else:
                raise ValueError('owned disarmed native observer readiness deadline')
            report['native_stream_ready_s'] = time.monotonic() - started
            report['native_socket_inventory'] = capture.peer_proof
            ports['native_source'] = capture.peer[1]
            console, console_peer = console_rx.recvfrom(65535)
            console_inventory = inventory(sitl)
            need(console and console_peer[0] == '127.0.0.1' and
                 exact_peer(console_inventory, console_peer[1], ports['console_rx']),
                 'serial0 console is not from the exact owned simulator')
            ports['console_source'] = console_peer[1]
            report['console_socket_inventory'] = console_inventory
            (out / 'serial0-first-datagram.bin').write_bytes(console)

            mission = out / 'observers-native.moos'
            config = mission_text(ports['moos'], ports['bridge'], ports['app_peer'], 3, plugins)
            # Explicit module directory binds composition to the frozen build.
            if '  plugin_directory' not in config:
                config = config.replace('  simulation_only = false\n',
                                        f"  plugin_directory = {build / 'plugins'}\n  simulation_only = false\n")
            mission.write_text(config)
            env = dict(os.environ)
            env['PATH'] = os.pathsep.join((str(build / 'bin'), str(moos / 'bin'), env.get('PATH', '')))
            app_socket.close()
            db_socket.close()
            antler = launch([str(paths['antler']), str(mission)], 'pantler.log', env)
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                capture.check()
                drain_app()
                need(sitl.poll() is None and antler.poll() is None, 'native or app exited during app startup')
                try:
                    owned, proof = live_app(helpers['ownership'], antler, mission, ports['bridge'])
                except RuntimeError:
                    time.sleep(.05)
                else:
                    break
            else:
                raise ValueError('exact owned app socket startup deadline')
            report['owned_app'], report['app_socket_inventory'] = owned, proof
            probe_args = [str(paths['probe']), str(ports['moos']), str(PROBE_SECONDS)]
            if traffic:
                probe_args.append('--traffic')
            if status_diagnostics:
                probe_args.append('--status-diagnostics')
            probe = launch(probe_args, 'probe.log', env)
            ready.set()
            deadline = time.monotonic() + PROBE_SECONDS + 6
            next_check = time.monotonic()
            owner_log = stack.enter_context((out / 'ownership.jsonl').open('x'))
            while probe.poll() is None and time.monotonic() < deadline:
                capture.check()
                drain_app()
                need(sitl.poll() is None and antler.poll() is None, 'native or app exited during observation')
                if time.monotonic() >= next_check:
                    row, proof = live_app(helpers['ownership'], antler, mission, ports['bridge'], owned)
                    native_proof = prove_peer(capture.peer)
                    owner_log.write(json.dumps({'mono': time.monotonic(), 'app': row,
                                                'app_udp': proof, 'native_udp': native_proof}) + '\n')
                    next_check = time.monotonic() + 1
                time.sleep(.05)
            report['probe_exit'] = probe.poll()
            need(probe.returncode == 0, 'native MOOS probe failed or exceeded deadline')
            report['final_owned_app'], report['final_app_socket_inventory'] = live_app(
                helpers['ownership'], antler, mission, ports['bridge'], owned)
            report['final_native_socket_inventory'] = prove_peer(capture.peer)
            capture.close()
            capture_finished = True
            report['capture_finished_mono'] = time.monotonic()
            report['wire_decode'] = decoder.finish()
            drain_app()
        finally:
            if capture is not None:
                if not capture_finished:
                    try:
                        capture.close()
                    except Exception as error:
                        errors.append(str(error))
                report['capture'] = capture.status()
            for process in reversed(processes):
                try:
                    remaining.extend(helpers['ownership'].stop_process_tree(process))
                except Exception as error:
                    errors.append(str(error))
            try:
                drain_app(expect_empty=False)
            except Exception as error:
                errors.append(str(error))
            report['owned_processes_remaining'] = None if errors else remaining
            report['cleanup_errors'] = errors
    return packets, forward_byte, forward_row


def run(args, lease_path):
    plugins = PLUGINS + (('traffic',) if args.traffic else ())
    if args.status_diagnostics:
        plugins += ('system_status',)
    source, ap, moos, build, helpers, firmware, source_ref, moos_ref, paths, protected = preflight(args, plugins)
    os.environ['MAVLINK20'] = '1'
    from pymavlink import mavutil

    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    copied, copied_identity = copy_firmware(out, firmware, helpers['manifest'], ap)
    decoder_path = Path(mavutil.mavlink.__file__).resolve()
    protected.extend([Path(sys.executable).resolve(), decoder_path])
    protected.extend(out / item['file'] for item in copied.values())
    before = {str(path): sha(path) for path in protected}
    report = {
        'schema': 1, 'result': 'FAIL', 'scenario': 'rx_only', 'profile': 'arducopter',
        'scope': 'owned disarmed Copter observer matrix, one-way telemetry exact-byte loopback',
        'hardware': False, 'timewarp': 1, 'source_root': str(source), 'source_ref': source_ref,
        'ardupilot_ref': firmware['source_revision'], 'moos_ref': moos_ref,
        'firmware_identity': firmware, 'copied_firmware': copied, 'artifacts_before': before,
        'native_sha256': before[str(paths['native'])], 'app_sha256': before[str(paths['app'])],
        'probe_sha256': before[str(paths['probe'])], 'decoder_path': str(decoder_path),
        'native_lock': str(lease_path), 'runner_pid': os.getpid(), 'observer_writes_to_fc': 0,
        'traffic_fixture': args.traffic,
        'status_diagnostics_fixture': args.status_diagnostics,
    }
    decoder_factory = lambda: mavutil.mavlink.MAVLink(None)
    try:
        rows, byte_start, row_start = exercise(out, build, moos, paths, helpers, firmware, decoder_factory, report, plugins)
        need(byte_start is not None and row_start is not None, 'app handoff never began')
        report['bridge_egress_bytes'] = (out / 'bridge-egress.mavlink').stat().st_size
        need(report['bridge_egress_datagrams'] == report['bridge_egress_bytes'] == 0,
             'owned peer observed bridge egress')
        raw = (out / 'native-wire.mavlink').read_bytes()
        need(report['forwarded_datagrams'] > 0 and report['forwarded_bytes'] == len(raw) - byte_start,
             'one-way byte handoff evidence incomplete')
        report['forwarded_first_byte'] = byte_start
        report['forwarded_first_row'] = row_start
        report['wire_replay'] = helpers['wire'].replay(raw, rows, decoder_factory)
        report['forwarded_replay'] = helpers['wire'].replay(raw[byte_start:], rows[row_start:], decoder_factory)
        log = out / 'probe.log'
        samples, modules = audit(log, plugins, build / 'plugins')
        report['loaded_modules_sha256'] = modules
        report['ground_runtime_observations'] = status_evidence(log.read_text(), fields)
        report['observation_evidence'] = audit_forwarded(rows[row_start:], samples, report['capture_finished_mono'])
        if args.traffic:
            report['observation_evidence']['families']['ADSB_VEHICLE'] = audit_native_traffic(
                samples[TRAFFIC_KEY], rows[row_start:])
        if args.status_diagnostics:
            report['diagnostic_evidence'] = audit_native_diagnostics(samples, rows[row_start:])
            dump(out / 'diagnostic-audit.json', report['diagnostic_evidence'])
        dump(out / 'observer-audit.json', report['observation_evidence'])
        report['result'] = 'PASS'
    except BaseException as error:
        report['error'] = str(error) or type(error).__name__
        report['traceback'] = traceback.format_exc()
    finally:
        try:
            cleared = clean_ports(report.get('ports', {}))
            report['ports_clear'] = cleared
            capture = report.get('capture', {})
            report['cleanup_proven'] = (report.get('owned_processes_remaining') == [] and bool(cleared) and
                                        all(cleared.values()) and capture.get('thread_retired') is True and
                                        capture.get('socket_closed') is True and capture.get('error') is None)
            need(report['cleanup_proven'], 'owned process or port cleanup unproven')
        except Exception as error:
            report['result'] = 'FAIL'
            report['cleanup_proven'] = False
            report['cleanup_error'] = str(error)
        try:
            report['source_ref_after'] = clean_source(source)
            report['moos_ref_after'] = clean_source(moos)
            report['firmware_identity_after'] = helpers['ownership'].native_firmware_profile(
                source, ap, args.firmware_manifest, args.firmware_attestation)
            post_copy = helpers['manifest'].verify_native(out / copied['manifest']['file'],
                                                          out / copied['attestation']['file'], ap)
            after = {path: sha(path) for path in before}
            report['artifacts_after'] = after
            report['artifact_drift'] = sorted(path for path, digest in before.items() if after[path] != digest)
            need(report['source_ref_after'] == source_ref and report['moos_ref_after'] == moos_ref and
                 report['firmware_identity_after'] == firmware and post_copy == copied_identity and
                 not report['artifact_drift'], 'protected native inputs changed during execution')
        except Exception as error:
            report['result'] = 'FAIL'
            report['postflight_error'] = str(error)
        report['files'] = {str(path.relative_to(out)): sha(path) for path in sorted(out.rglob('*'))
                           if path.is_file() and path.name != 'result.json'}
        dump(out / 'result.json', report)
    print(json.dumps({key: report[key] for key in ('result', 'error', 'cleanup_proven',
                     'observation_evidence', 'diagnostic_evidence', 'postflight_error') if key in report}, indent=2))
    return int(report['result'] != 'PASS')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=['rx_only'], required=True)
    parser.add_argument('--traffic', action='store_true', help='include the native SIM_ADSB traffic fixture')
    parser.add_argument('--status-diagnostics', action='store_true',
                        help='include the native board-power and ArduPilot EKF observations')
    for name in ('source-root', 'ardupilot-root', 'moos-root', 'build', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    for name in ('firmware-manifest', 'firmware-attestation', 'probe'):
        parser.add_argument('--' + name, type=Path)
    for name in ('expected-app-sha256', 'expected-probe-sha256'):
        parser.add_argument('--' + name)
    args = parser.parse_args()
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(143))
    lease = Path(tempfile.gettempdir()) / 'mavmoos-native-sitl.lock'
    flags = os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_CLOEXEC
    with os.fdopen(os.open(lease, flags, 0o600), 'r+') as lock:
        identity = os.fstat(lock.fileno())
        need(stat.S_ISREG(identity.st_mode) and identity.st_uid == os.getuid(), 'unsafe native lease')
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return run(args, lease)


if __name__ == '__main__':
    raise SystemExit(main())
