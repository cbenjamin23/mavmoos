#!/usr/bin/env python3
"""Manual owned fixed-wing AIRSPEED witness. The app and wire observer only receive from the FC."""
import argparse
from contextlib import ExitStack
import fcntl
import json
import os
from pathlib import Path
import platform
import re
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
from air_data_native.audit import KEY, SENSORS, audit_airspeed
from imu.audit_native import fields
from imu.run_native import dump, exact_peer, inventory, live_app, load_helpers, revision, sha
from observer_extensions_native.run_native import wait_for
from observer_matrix.native_audit import need, status_evidence
from observer_matrix.run_loopback import audit, mission_text, reserve, start, take_egress
from observer_matrix.run_native import MAX_WIRE_BYTES, clean_ports, clean_source

PLUGINS = ('air_data',)
PIN = '3fc7011a7d3dc047cbb17d8bd98ee94577d144c6'
BINARY_SHA = 'b977ff6cd559f6a6c3d16cfb57c1e32840291210eb593e4f7254fea701137fa5'
BUILD_ATTESTATION_SHA = '3ddd2117e19fe1238e1d05cb483946a5c9801aea4c467f2fbdcac5d9f6e34a75'
DEFAULTS_SHA = '93ba9a70c771609a90b81249d6a1d5a9df8d48bef7d149b42b2d9c7fbd06494a'
MOOS_PIN = '023fb54e00692611ecee2cd6f1138d10eec2127e'
HELM_SHA = '5b3d2e705609c37b5b5000384655b4491010d6d797b94c43830d9972e684a876'
HOME = (42.358456, -71.087589, 10, 0)
FIXTURE = {
    'SYSID_THISMAV': 1, 'SERIAL0_PROTOCOL': 2, 'SERIAL1_PROTOCOL': 2, 'Q_ENABLE': 0,
    'ARSPD_TYPE': 100, 'ARSPD2_TYPE': 100, 'ARSPD_PRIMARY': 0, 'ARSPD_USE': 1, 'ARSPD2_USE': 0,
    'SIM_WIND_SPD': 0, 'SIM_ARSPD_RND': .5, 'SIM_ARSPD2_RND': 1.25,
    'SIM_ARSPD_FAIL': 0, 'SIM_ARSPD2_FAIL': 0,
}
INTERVALS = '33 250\n245 500\n295 200\n'
PROBE_SECONDS = 10


def app_clock():
    # Darwin libc++ steady_clock uses this suspend-inclusive raw clock.
    # Python time.monotonic() instead uses mach_absolute_time() on this host.
    return time.clock_gettime(time.CLOCK_MONOTONIC_RAW)


def preflight(args):
    build, moos, plane = (path.resolve() for path in (args.build, args.moos_root, args.plane_build_root))
    app_source = args.app_source_root.resolve()
    source_ref, app_ref, moos_ref = (clean_source(path) for path in (ROOT, app_source, moos))
    need(platform.system() == 'Darwin' and platform.machine() == 'arm64'
         and hasattr(time, 'CLOCK_MONOTONIC_RAW'),
         'exact Plane executable and acquisition clock require Darwin ARM64')
    need(app_clock() > 0, 'Darwin app acquisition clock unavailable')
    need(moos_ref == MOOS_PIN, 'stock MOOS source pin differs')
    need(app_ref == args.expected_app_source_revision, 'frozen app build source pin differs')
    cache = build / 'CMakeCache.txt'
    cache_text = cache.read_text()
    for key, path in (('CMAKE_HOME_DIRECTORY', app_source), ('MOOS_IVP_ROOT', moos)):
        configured = re.search(r'^' + key + r':[^=]+=([^\n]+)$', cache_text, re.M)
        need(configured and Path(configured[1]).resolve() == path,
             'configured build provenance differs for ' + key)
    source = plane / 'source'
    paths = {
        'native': source / 'build/sitl/bin/arduplane',
        'defaults': source / 'Tools/autotest/models/plane.parm',
        'attestation': plane / 'build-attestation.json',
        'app': build / 'bin/iMAVLink',
        'probe': build / 'bin/observer_matrix_probe',
        'air_data': build / 'plugins/mavmoos_air_data.so',
        'antler': moos / 'bin/pAntler', 'db': moos / 'bin/MOOSDB', 'helm': moos / 'bin/pHelmIvP',
    }
    need(all(path.is_file() and not path.is_symlink() for path in paths.values()),
         'required regular artifact missing or symlinked')
    for role, expected in (
        ('native', BINARY_SHA), ('defaults', DEFAULTS_SHA), ('attestation', BUILD_ATTESTATION_SHA),
        ('helm', HELM_SHA), ('app', args.expected_app_sha256), ('probe', args.expected_probe_sha256),
        ('air_data', args.expected_module_sha256),
    ):
        need(re.fullmatch('[0-9a-f]{64}', expected) and sha(paths[role]) == expected, f'{role} digest differs')
    receipt = json.loads(paths['attestation'].read_text())
    need(receipt['status'] == 'OFFLINE_BUILD_PASS' and receipt['source_revision'] == PIN
         and receipt['artifact']['sha256'] == BINARY_SHA and revision(source) == PIN,
         'Plane source and build receipt differ')

    # The installed ArduPilotMega Python dialect omits the development family.
    from pymavlink.dialects.v20 import all as codec
    airspeed = codec.mavlink_map.get(295)
    need(airspeed is not None and airspeed.crc_extra == 234 and airspeed.unpacker.format == '<ffhBB',
         'independent MAVLink2 decoder AIRSPEED definition differs')
    protected = [cache, *paths.values(), Path(codec.__file__).resolve(), Path(sys.executable).resolve()]
    helpers = load_helpers(ROOT)
    protected.extend(Path(module.__file__).resolve() for module in helpers.values())
    protected.extend(app_source / name for name in (
        'src/plugins/AirDataPlugin.cpp', 'src/libmavmoos_runtime/PassiveObserver.h',
        'src/iMAVLink/MAVLink.cpp', 'src/libmavmoos_rx/Telemetry.cpp',
    ))
    protected.extend(ROOT / name for name in (
        'tests/air_data_native/run_native.py', 'tests/air_data_native/audit.py',
        'tests/observer_extensions_native/audit.py', 'tests/observer_matrix/Probe.cpp',
        'tests/observer_matrix/run_native.py', 'tests/observer_matrix/run_loopback.py',
    ))
    return paths, helpers, codec, protected, (source_ref, app_ref, moos_ref)


def exercise(out, args, paths, helpers, codec, report):
    processes, rows, cleanup_errors = [], [], []
    capture = None
    boundary = None
    ready = threading.Event()
    ports = report['ports'] = {}
    report['bridge_egress_datagrams'] = report['forwarded_datagrams'] = report['forwarded_bytes'] = 0
    decoder = helpers['wire'].FrameDecoder(codec.MAVLink(None), expected_type=1)

    def launch(command, name, env=None):
        process = start(command, out, env, name)
        processes.append(process)
        return process

    with ExitStack() as stack:
        console, native, app_peer, bridge = [stack.enter_context(reserve(socket.SOCK_DGRAM)) for _ in range(4)]
        db, base = [stack.enter_context(reserve(socket.SOCK_STREAM)) for _ in range(2)]
        rc = stack.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
        rc.bind(('0.0.0.0', 0))
        console.settimeout(.5)
        native.setblocking(False)
        app_peer.setblocking(False)
        ports.update({name: endpoint.getsockname()[1] for name, endpoint in (
            ('console_rx', console), ('native_rx', native), ('app_peer', app_peer), ('bridge', bridge),
            ('moos', db), ('native_base', base), ('rc', rc),
        )})
        egress = stack.enter_context((out / 'bridge-egress.mavlink').open('xb'))
        raw = stack.enter_context((out / 'native-wire.mavlink').open('xb'))
        frames = stack.enter_context((out / 'native-wire.jsonl').open('x'))
        packets = stack.enter_context((out / 'native-datagrams.jsonl').open('x'))
        ownership = stack.enter_context((out / 'ownership.jsonl').open('x'))

        def owned_peer(sender, destination):
            proof = inventory(sitl)
            need(sender[0] == '127.0.0.1' and exact_peer(proof, sender[1], destination),
                 'received a datagram outside the exact owned native route')
            return proof

        def consume(data, _sender, at):
            nonlocal boundary
            offset = decoder.input_bytes
            need(offset + len(data) <= MAX_WIRE_BYTES, 'native wire exceeds byte bound')
            if ready.is_set() and boundary is None and decoder.frames and offset == decoder.consumed_bytes:
                boundary = offset, len(rows)
            raw.write(data)
            decoded, _ = decoder.feed(data, at)
            rows.extend(decoded)
            for row in decoded:
                frames.write(json.dumps(row, allow_nan=False) + '\n')
            forwarded = boundary is not None
            if forwarded:
                need(app_peer.sendto(data, ('127.0.0.1', ports['bridge'])) == len(data),
                     'incomplete one-way native datagram')
                report['forwarded_datagrams'] += 1
                report['forwarded_bytes'] += len(data)
            packets.write(json.dumps({'offset': offset, 'length': len(data), 'mono': at,
                                      'forwarded': forwarded}) + '\n')

        probe = None

        def check():
            capture.check()
            need(all(process.poll() is None for process in processes if process is not probe),
                 'owned SITL or mission exited')
            report['bridge_egress_datagrams'] += take_egress(app_peer, ports['bridge'], egress)
            need(report['bridge_egress_datagrams'] == 0, 'receive-only app emitted a datagram')

        try:
            defaults = out / 'fixture.parm'
            intervals = out / 'message-intervals-chan1.txt'
            defaults.write_text(''.join(f'{key} {value}\n' for key, value in FIXTURE.items()))
            intervals.write_text(INTERVALS)
            report['fixture_parameters'] = FIXTURE
            report['fixture_sha256'] = {path.name: sha(path) for path in (defaults, intervals)}
            command = [str(paths['native']), '--model', 'plane', '--home', ','.join(map(str, HOME)),
                       '--speedup', '1', '--wipe', '--defaults', f"{paths['defaults']},{defaults}",
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
                native, lambda sender: owned_peer(sender, ports['native_rx']), consume,
                max_datagrams=10000, clock=app_clock)
            capture.start()

            def producers_ready():
                observed = tuple(rows)
                sensors = {row['id'] for row in observed if row['mavpackettype'] == 'AIRSPEED'}
                return (capture.peer and sensors == set(SENSORS)
                        and any(row['mavpackettype'] == 'HEARTBEAT' for row in observed)
                        and any(row['mavpackettype'] == 'EXTENDED_SYS_STATE' and row['landed_state'] == 1
                                for row in observed)
                        and any(row['mavpackettype'] == 'GLOBAL_POSITION_INT'
                                and abs(row['lat'] / 1e7 - HOME[0]) <= .01
                                and abs(row['lon'] / 1e7 - HOME[1]) <= .01 for row in observed))

            wait_for(60, producers_ready, check, 'both native sensor IDs and disarmed ground evidence missing at home')
            report['native_socket_inventory'] = capture.peer_proof
            ports['native_source'] = capture.peer[1]
            console_data, console_peer = console.recvfrom(65535)
            need(console_data, 'owned serial0 console did not start')
            report['console_socket_inventory'] = owned_peer(console_peer, ports['console_rx'])
            ports['console_source'] = console_peer[1]
            (out / 'serial0-first-datagram.bin').write_bytes(console_data)

            mission = out / 'plane-air-data-native.moos'
            config = mission_text(ports['moos'], ports['bridge'], ports['app_peer'], 3, PLUGINS)
            config = config.replace('  expected_type = 2\n', '  expected_type = 1\n')
            config = config.replace('  simulation_only = false\n',
                                    f"  plugin_directory = {args.build.resolve() / 'plugins'}\n"
                                    '  simulation_only = false\n  helm_enabled = false\n')
            mission.write_text('\n'.join(line for line in config.splitlines() if 'plugin.odometry.' not in line) + '\n')
            env = dict(os.environ)
            env['PATH'] = os.pathsep.join((str(paths['app'].parent), str(paths['antler'].parent), env.get('PATH', '')))
            bridge.close()
            db.close()
            antler = launch([str(paths['antler']), str(mission)], 'pantler.log', env)

            def prove_app():
                try:
                    return live_app(helpers['ownership'], antler, mission, ports['bridge'])
                except RuntimeError:
                    return None

            app, proof = wait_for(10, prove_app, check, 'exact owned iMAVLink startup deadline')
            report['owned_app'], report['app_socket_inventory'] = app, proof
            probe = launch([str(paths['probe']), str(ports['moos']), str(PROBE_SECONDS),
                            '--watch=' + KEY], 'probe.log', env)
            ready.set()
            until, next_inventory = time.monotonic() + PROBE_SECONDS + 6, 0
            while probe.poll() is None and time.monotonic() < until:
                check()
                now = time.monotonic()
                if now >= next_inventory:
                    _, app_udp = live_app(helpers['ownership'], antler, mission, ports['bridge'], app)
                    ownership.write(json.dumps({'mono': app_clock(), 'app_udp': app_udp,
                                                'native_udp': owned_peer(capture.peer, ports['native_rx'])}) + '\n')
                    next_inventory = now + 1
                time.sleep(.05)
            report['probe_exit'] = probe.poll()
            need(probe.returncode == 0, 'native Plane MOOS probe failed or exceeded deadline')
            live_app(helpers['ownership'], antler, mission, ports['bridge'], app)
            owned_peer(capture.peer, ports['native_rx'])
            capture.close()
            report['wire_decode'] = decoder.finish()
            check()
            need(boundary is not None, 'one-way stream never reached a complete frame boundary')
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


def run(args, lease, setup):
    paths, helpers, codec, protected, refs = setup
    before = {str(path): sha(path) for path in protected}
    out = args.output.resolve()
    out.mkdir(mode=0o700, parents=True, exist_ok=False)
    copied = out / 'plane-build-attestation.json'
    copied.write_bytes(paths['attestation'].read_bytes())
    need(sha(copied) == BUILD_ATTESTATION_SHA, 'native Plane build attestation copy differs')
    report = {'schema': 1, 'result': 'FAIL', 'scenario': 'plane_airspeed_rx_only', 'vehicle_profile': 'fixed_wing_observer',
              'source_revision': refs[0], 'app_source_revision': refs[1], 'moos_revision': refs[2],
              'ardupilot_revision': PIN, 'hardware': False, 'timewarp': 1, 'native_lock': str(lease),
              'model': 'plane', 'home': HOME, 'observer_writes_to_fc': 0,
              'acquisition_clock': 'clock_gettime(CLOCK_MONOTONIC_RAW)', 'artifacts_before': before,
              **{f'{role}_sha256': sha(paths[role]) for role in ('native', 'app', 'probe', 'helm')}}
    try:
        rows, (offset, first) = exercise(out, args, paths, helpers, codec, report)
        wire = (out / 'native-wire.mavlink').read_bytes()
        report['bridge_egress_bytes'] = (out / 'bridge-egress.mavlink').stat().st_size
        need(report['bridge_egress_datagrams'] == report['bridge_egress_bytes'] == 0,
             'app transmitted to the owned peer')
        need(report['forwarded_datagrams'] > 0 and report['forwarded_bytes'] == len(wire) - offset,
             'one-way byte accounting differs')
        report['forwarded_first_byte'], report['forwarded_first_row'] = offset, first
        decoder = lambda: codec.MAVLink(None)
        report['wire_replay'] = helpers['wire'].replay(wire, rows, decoder, expected_type=1)
        report['forwarded_replay'] = helpers['wire'].replay(wire[offset:], rows[first:], decoder, expected_type=1)
        probe = out / 'probe.log'
        samples, report['loaded_modules_sha256'] = audit(probe, PLUGINS, paths['air_data'].parent)
        report['ground_runtime_observations'] = status_evidence(probe.read_text(), fields)
        report['observation_evidence'] = audit_airspeed(wire[offset:], rows[first:], samples.get(KEY, []))
        report['result'] = 'PASS'
    except BaseException as error:
        report['error'] = str(error) or type(error).__name__
        report['traceback'] = traceback.format_exc()
    finally:
        try:
            ports = report['ports_clear'] = clean_ports(report.get('ports', {}))
            capture = report.get('capture', {})
            need(ports and all(ports.values()) and report.get('owned_processes_remaining') == []
                 and not report.get('cleanup_errors') and capture.get('thread_retired') is True
                 and capture.get('socket_closed') is True and capture.get('error') is None,
                 'owned endpoints and process teardown not proved')
            report['cleanup_proven'] = True
        except Exception as error:
            report['result'], report['cleanup_proven'], report['cleanup_error'] = 'FAIL', False, str(error)
        try:
            after = report['artifacts_after'] = {path: sha(path) for path in before}
            after_refs = tuple(clean_source(path) for path in (ROOT, args.app_source_root, args.moos_root))
            need(before == after and after_refs == refs and sha(copied) == BUILD_ATTESTATION_SHA
                 and all(sha(out / name) == digest for name, digest in report['fixture_sha256'].items()),
                 'protected source, build or fixture inputs changed')
        except Exception as error:
            report['result'], report['artifact_error'] = 'FAIL', str(error)
        report['files'] = {path.relative_to(out).as_posix(): sha(path) for path in sorted(out.iterdir())
                           if path.is_file() and not path.is_symlink()}
        dump(out / 'result.json', report)
    print(f"{report['result']} {out / 'result.json'}")
    return int(report['result'] != 'PASS')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('build', 'moos-root', 'plane-build-root'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--app-source-root', type=Path, default=ROOT)
    parser.add_argument('--expected-app-source-revision', required=True)
    for name in ('app', 'probe', 'module'):
        parser.add_argument('--expected-' + name + '-sha256', required=True)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--check-inputs', action='store_true')
    args = parser.parse_args()
    if not args.check_inputs and (not args.output or args.output.exists()):
        parser.error('--output must name a fresh private run directory')
    setup = preflight(args)
    if args.check_inputs:
        print(json.dumps({'preflight': 'PASS', 'native_sha256': BINARY_SHA, 'source_refs': setup[4]}))
        return 0
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(143))
    lease = Path(tempfile.gettempdir()) / 'mavmoos-native-sitl.lock'
    flags = os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_CLOEXEC
    with os.fdopen(os.open(lease, flags, 0o600), 'r+') as lock:
        identity = os.fstat(lock.fileno())
        need(stat.S_ISREG(identity.st_mode) and identity.st_uid == os.getuid(), 'unsafe native lease')
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return run(args, lease, setup)


if __name__ == '__main__':
    raise SystemExit(main())
