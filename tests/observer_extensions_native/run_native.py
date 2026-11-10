#!/usr/bin/env python3
"""One owned stable Copter boot for passive optical flow, terrain and navigation."""
import argparse
from contextlib import ExitStack
import fcntl
import json
import os
from pathlib import Path
import signal
import socket
import stat
import sys
import tempfile
import threading
import time
import traceback

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'tests'), str(ROOT / 'tests/observer_matrix')]
from imu.audit_native import fields
from observer_matrix.native_audit import need, status_evidence
from observer_matrix.run_loopback import audit, mission_text, reserve, start, take_egress
from observer_matrix.run_native import (
    MAX_WIRE_BYTES, clean_ports, clean_source, copy_firmware, dump, exact_peer,
    inventory, live_app, preflight, sha,
)
from observer_extensions_native.audit import audit_forwarded

PLUGINS = ('telemetry', 'optical_flow', 'terrain', 'navigation_monitor')
INTERVALS = '33 200\n62 200\n100 100\n133 500\n136 500\n245 500\n'
FIXTURE = {'SERIAL1_PROTOCOL': 2, 'SIM_FLOW_ENABLE': 1, 'FLOW_TYPE': 10,
           'TERRAIN_ENABLE': 1, 'TERRAIN_OPTIONS': 0}
PRODUCERS = {'HEARTBEAT', 'EXTENDED_SYS_STATE', 'GLOBAL_POSITION_INT', 'OPTICAL_FLOW',
             'TERRAIN_REQUEST', 'TERRAIN_REPORT', 'NAV_CONTROLLER_OUTPUT'}
PROBE_SECONDS = 16


def wait_for(seconds, predicate, check, reason):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        check()
        result = predicate()
        if result:
            return result
        time.sleep(.05)
    raise RuntimeError(reason)


def exercise(out, build, moos, paths, helpers, firmware, decoder_factory, report):
    rows, processes, cleanup_errors = [], [], []
    capture = None
    ready = threading.Event()
    boundary = None
    decoder = helpers['wire'].FrameDecoder(decoder_factory())
    ports = report['ports'] = {}
    report['bridge_egress_datagrams'] = 0
    report['forwarded_datagrams'] = report['forwarded_bytes'] = 0

    def launch(command, label, env=None):
        process = start(command, out, env, label)
        processes.append(process)
        return process

    with ExitStack() as stack:
        console = stack.enter_context(reserve(socket.SOCK_DGRAM))
        native = stack.enter_context(reserve(socket.SOCK_DGRAM))
        app_peer = stack.enter_context(reserve(socket.SOCK_DGRAM))
        bridge = stack.enter_context(reserve(socket.SOCK_DGRAM))
        db = stack.enter_context(reserve(socket.SOCK_STREAM))
        base = stack.enter_context(reserve(socket.SOCK_STREAM))
        rc = stack.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
        rc.bind(('0.0.0.0', 0))
        console.settimeout(.5)
        native.setblocking(False)
        app_peer.setblocking(False)
        ports.update(console_rx=console.getsockname()[1], native_rx=native.getsockname()[1],
                     app_peer=app_peer.getsockname()[1], bridge=bridge.getsockname()[1],
                     moos=db.getsockname()[1], native_base=base.getsockname()[1],
                     rc=rc.getsockname()[1])
        egress = stack.enter_context((out / 'bridge-egress.mavlink').open('xb'))
        raw = stack.enter_context((out / 'native-wire.mavlink').open('xb'))
        frames = stack.enter_context((out / 'native-wire.jsonl').open('x'))
        packets = stack.enter_context((out / 'native-datagrams.jsonl').open('x'))
        startup = stack.enter_context((out / 'native-startup.jsonl').open('x'))

        def owned_peer(sender, destination):
            proof = inventory(sitl)
            need(sender[0] == '127.0.0.1' and exact_peer(proof, sender[1], destination),
                 'received an unowned native UDP route')
            return proof

        def consume(data, _sender, at):
            nonlocal boundary
            offset = decoder.input_bytes
            need(offset + len(data) <= MAX_WIRE_BYTES, 'native wire byte bound exceeded')
            if ready.is_set() and boundary is None and decoder.frames and offset == decoder.consumed_bytes:
                boundary = offset, len(rows)
            raw.write(data)
            decoded, prefix = decoder.feed(data, at)
            rows.extend(decoded)
            for row in decoded:
                frames.write(json.dumps(row, allow_nan=False) + '\n')
            for row in prefix:
                startup.write(json.dumps(row, allow_nan=False) + '\n')
            forwarded = boundary is not None
            if forwarded:
                accepted = app_peer.sendto(data, ('127.0.0.1', ports['bridge']))
                need(accepted == len(data), 'partial one-way relay datagram')
                report['forwarded_datagrams'] += 1
                report['forwarded_bytes'] += accepted
            packets.write(json.dumps({'offset': offset, 'length': len(data), 'mono': at,
                                      'forwarded': forwarded}) + '\n')

        def check():
            capture.check()
            need(all(process.poll() is None for process in processes if process is not probe),
                 'owned SITL or mission exited')
            report['bridge_egress_datagrams'] += take_egress(app_peer, ports['bridge'], egress)
            need(report['bridge_egress_datagrams'] == 0, 'receive-only app emitted a datagram')

        probe = None
        try:
            parameters = {**firmware['fixture']['parameters'], **FIXTURE}
            defaults = out / 'fixture.parm'
            intervals = out / 'message-intervals-chan1.txt'
            defaults.write_text(''.join(f'{key} {value}\n' for key, value in parameters.items()))
            intervals.write_text(INTERVALS)
            report['fixture_parameters'] = parameters
            report['fixture_sha256'] = {file.name: sha(file) for file in (defaults, intervals)}
            home = ','.join(map(str, firmware['fixture']['home']))
            command = [str(paths['native']), '--model', 'quad', '--home', home, '--speedup', '1', '--wipe',
                       '--defaults', f"{paths['defaults']},{defaults}",
                       '--serial0', f"udpclient:127.0.0.1:{ports['console_rx']}",
                       '--serial1', f"udpclient:127.0.0.1:{ports['native_rx']}",
                       '--serial2', 'none', '--serial5', 'none', '--serial6', 'none', '--serial7', 'none',
                       '--serial8', 'none', '--base-port', str(ports['native_base']), '--rc-in-port', str(ports['rc'])]
            report['native_command'] = command
            base.close()
            rc.close()
            sitl = launch(command, 'sitl.log')
            report['sitl_pid'] = sitl.pid
            capture = helpers['passive'].PassiveReceiver(
                native, lambda sender: owned_peer(sender, ports['native_rx']), consume, max_datagrams=10000)
            capture.start()
            lat, lon = firmware['fixture']['home'][:2]

            def producers_ready():
                observed = tuple(rows)
                near_home = any(row['mavpackettype'] == 'GLOBAL_POSITION_INT' and
                                abs(row['lat'] / 1e7 - lat) <= .01 and abs(row['lon'] / 1e7 - lon) <= .01
                                for row in observed)
                return capture.peer and near_home and PRODUCERS <= {row['mavpackettype'] for row in observed}

            wait_for(60, producers_ready, check, 'required native observer producers did not appear near home within 60s')
            report['native_socket_inventory'] = capture.peer_proof
            ports['native_source'] = capture.peer[1]
            console_data, console_peer = console.recvfrom(65535)
            need(console_data, 'owned serial0 console had no startup evidence')
            report['console_socket_inventory'] = owned_peer(console_peer, ports['console_rx'])
            ports['console_source'] = console_peer[1]
            (out / 'serial0-first-datagram.bin').write_bytes(console_data)

            mission = out / 'observers-native.moos'
            config = mission_text(ports['moos'], ports['bridge'], ports['app_peer'], 3, PLUGINS)
            config = config.replace('  simulation_only = false\n',
                                    f"  plugin_directory = {build / 'plugins'}\n  simulation_only = false\n")
            config = config.replace('  plugin.odometry.sample_timeout = 0.8\n', '')
            config = config.replace('  plugin.odometry.publish_interval = 0.08\n', '')
            mission.write_text(config)
            env = dict(os.environ)
            env['PATH'] = os.pathsep.join((str(build / 'bin'), str(moos / 'bin'), env.get('PATH', '')))
            bridge.close()
            db.close()
            antler = launch([str(paths['antler']), str(mission)], 'pantler.log', env)

            def prove_app():
                try:
                    return live_app(helpers['ownership'], antler, mission, ports['bridge'])
                except RuntimeError:
                    return None

            app, app_inventory = wait_for(10, prove_app, check, 'exact owned app startup deadline')
            report['owned_app'], report['app_socket_inventory'] = app, app_inventory
            probe = launch([str(paths['probe']), str(ports['moos']), str(PROBE_SECONDS)], 'probe.log', env)
            ready.set()
            until, next_inventory = time.monotonic() + PROBE_SECONDS + 6, 0
            ownership_log = stack.enter_context((out / 'ownership.jsonl').open('x'))
            while probe.poll() is None and time.monotonic() < until:
                check()
                now = time.monotonic()
                if now >= next_inventory:
                    _, current = live_app(helpers['ownership'], antler, mission, ports['bridge'], app)
                    ownership_log.write(json.dumps({'mono': now, 'app_udp': current,
                                                    'native_udp': owned_peer(capture.peer, ports['native_rx'])}) + '\n')
                    next_inventory = now + 1
                time.sleep(.05)
            report['probe_exit'] = probe.poll()
            need(probe.returncode == 0, 'native observer MOOS probe failed or exceeded deadline')
            _, report['final_app_socket_inventory'] = live_app(
                helpers['ownership'], antler, mission, ports['bridge'], app)
            report['final_native_socket_inventory'] = owned_peer(capture.peer, ports['native_rx'])
            capture.close()
            report['capture_finished_mono'] = time.monotonic()
            report['wire_decode'] = decoder.finish()
            check()
            need(boundary is not None, 'one-way relay never reached a complete MAVLink boundary')
        finally:
            if capture is not None:
                if not capture.status()['socket_closed']:
                    try:
                        capture.close()
                    except Exception as error:
                        cleanup_errors.append(str(error))
                report['capture'] = capture.status()
            remaining = []
            for process in reversed(processes):
                try:
                    remaining.extend(helpers['ownership'].stop_process_tree(process))
                except Exception as error:
                    cleanup_errors.append(str(error))
            try:
                report['bridge_egress_datagrams'] += take_egress(app_peer, ports['bridge'], egress)
            except Exception as error:
                cleanup_errors.append(str(error))
            report['cleanup_errors'] = cleanup_errors
            report['owned_processes_remaining'] = remaining
    return rows, boundary


def run(args, lease):
    args.probe = args.probe or args.build / 'bin/observer_extensions_native_probe'
    # Reuse the base passive launcher's preflight with its optional fixtures off.
    args.traffic = args.status_diagnostics = False
    source, ap, moos, build, helpers, firmware, source_ref, moos_ref, paths, protected = preflight(args, PLUGINS)
    os.environ['MAVLINK20'] = '1'
    from pymavlink import mavutil

    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    copied, copied_identity = copy_firmware(out, firmware, helpers['manifest'], ap)
    protected.extend(Path(__file__).with_name(name) for name in (
        'run_native.py', 'ObserverExtensionsProbe.cpp', 'audit.py'))
    protected.extend(source / 'src/plugins' / name for name in (
        'OpticalFlowPlugin.cpp', 'TerrainPlugin.cpp', 'NavigationMonitorPlugin.cpp'))
    protected.extend(ap / name for name in (
        'ArduCopter/GCS_Mavlink.cpp', 'libraries/AP_Terrain/TerrainGCS.cpp',
        'libraries/AP_OpticalFlow/AP_OpticalFlow_SITL.cpp'))
    protected.extend([Path(sys.executable).resolve(), Path(mavutil.mavlink.__file__).resolve(),
                      *(out / item['file'] for item in copied.values())])
    before = {str(path): sha(path) for path in protected}
    report = {'schema': 1, 'result': 'FAIL', 'scenario': 'rx_only', 'profile': 'arducopter',
              'scope': 'disarmed stable Copter optical flow, terrain, navigation observer',
              'hardware': False, 'timewarp': 1, 'source_ref': source_ref, 'moos_ref': moos_ref,
              'mavmoos_revision': source_ref, 'ardupilot_revision': firmware['source_revision'],
              'moos_revision': moos_ref, 'firmware_identity': firmware, 'copied_firmware': copied,
              'artifacts_before': before, 'native_lock': str(lease), 'observer_writes_to_fc': 0,
              **{f'{role}_sha256': before[str(paths[role])] for role in ('native', 'app', 'probe', 'helm')}}
    decoder_factory = lambda: mavutil.mavlink.MAVLink(None)
    try:
        rows, (offset, first) = exercise(out, build, moos, paths, helpers, firmware, decoder_factory, report)
        report['bridge_egress_bytes'] = (out / 'bridge-egress.mavlink').stat().st_size
        need(report['bridge_egress_datagrams'] == report['bridge_egress_bytes'] == 0,
             'bridge emitted data to the owned app peer')
        wire = (out / 'native-wire.mavlink').read_bytes()
        need(report['forwarded_datagrams'] > 0 and report['forwarded_bytes'] == len(wire) - offset,
             'one-way byte accounting differs')
        report['forwarded_first_byte'], report['forwarded_first_row'] = offset, first
        report['wire_replay'] = helpers['wire'].replay(wire, rows, decoder_factory)
        report['forwarded_replay'] = helpers['wire'].replay(wire[offset:], rows[first:], decoder_factory)
        log = out / 'probe.log'
        samples, report['loaded_modules_sha256'] = audit(log, PLUGINS, build / 'plugins')
        report['ground_runtime_observations'] = status_evidence(log.read_text(), fields)
        report['observation_evidence'] = audit_forwarded(rows[first:], wire[offset:], samples,
                                                        report['capture_finished_mono'])
        dump(out / 'observer-extensions-audit.json', report['observation_evidence'])
        report['result'] = 'PASS'
    except BaseException as error:
        report['error'] = str(error) or type(error).__name__
        report['traceback'] = traceback.format_exc()
    finally:
        try:
            cleared = clean_ports(report.get('ports', {}))
            report['ports_clear'] = cleared
            capture = report.get('capture', {})
            need(cleared and all(cleared.values()) and report.get('owned_processes_remaining') == [] and
                 not report.get('cleanup_errors') and capture.get('thread_retired') is True and
                 capture.get('socket_closed') is True and capture.get('error') is None,
                 'owned endpoints or processes not proved clean')
            report['cleanup_proven'] = True
        except Exception as error:
            report['result'], report['cleanup_proven'], report['cleanup_error'] = 'FAIL', False, str(error)
        try:
            after = {path: sha(path) for path in before}
            report['artifacts_after'] = after
            report['source_ref_after'], report['moos_ref_after'] = clean_source(source), clean_source(moos)
            report['firmware_identity_after'] = helpers['ownership'].native_firmware_profile(
                source, ap, args.firmware_manifest, args.firmware_attestation)
            need(before == after and (report['source_ref_after'], report['moos_ref_after']) == (source_ref, moos_ref)
                 and report['firmware_identity_after'] == firmware and
                 helpers['manifest'].verify_native(out / copied['manifest']['file'],
                                                   out / copied['attestation']['file'], ap) == copied_identity,
                 'protected native observer inputs changed during execution')
        except Exception as error:
            report['result'], report['postflight_error'] = 'FAIL', str(error)
        report['files'] = {str(path.relative_to(out)): sha(path) for path in sorted(out.rglob('*'))
                           if path.name != 'result.json' and path.is_file() and not path.is_symlink()}
        dump(out / 'result.json', report)
    print(json.dumps({key: report[key] for key in ('result', 'error', 'cleanup_proven',
                     'observation_evidence', 'postflight_error') if key in report}, indent=2))
    return int(report['result'] != 'PASS')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('source-root', 'ardupilot-root', 'moos-root', 'build', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    for name, variable in (('firmware-manifest', 'MAVMOOS_FIRMWARE_MANIFEST'),
                           ('firmware-attestation', 'MAVMOOS_FIRMWARE_ATTESTATION')):
        value = os.environ.get(variable)
        parser.add_argument('--' + name, type=Path, default=Path(value) if value else None, required=not value)
    parser.add_argument('--probe', type=Path)
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
