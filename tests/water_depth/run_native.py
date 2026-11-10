#!/usr/bin/env python3
"""One optional disarmed motorboat boot with ArduPilot's simulated depthfinder sources."""
import argparse
from contextlib import ExitStack
import fcntl
import json
import os
from pathlib import Path
import re
import signal
import socket
import stat
import sys
import tempfile
import threading
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'tests'), str(ROOT / 'tests/observer_matrix')]
from imu.audit_native import fields
from imu.run_native import dump, exact_peer, inventory, live_app, load_helpers, revision, sha
from observer_extensions_native.run_native import wait_for
from observer_matrix.run_loopback import audit, mission_text, reserve, start, take_egress
from observer_matrix.run_native import MAX_WIRE_BYTES, clean_ports, clean_source
from water_depth.audit import WATER_DEPTH, require
from water_depth.native_audit import SENSORS, audit_native

ROVER = '04cf6486e8d42ce138de816e8466244346ed77cb'
NATIVE_SHA = 'a859ab703395bbb04da523dae5414cc5fbc23b736a1c71639f2ef3e255df5fc9'
MOOS = '023fb54e00692611ecee2cd6f1138d10eec2127e'
HELM = '5b3d2e705609c37b5b5000384655b4491010d6d797b94c43830d9972e684a876'
DEFAULTS = {
    'rover.parm': '6479879d3e625854a6c32cb4d3fe231bab81d935e07904eac8a411fd6f971bc4',
    'motorboat.parm': '55a816688adc1a7df897fdc56cbdd6bd5b43c354ee823891bff1e8a861861b01',
    'rover-skid.parm': '45a572363fc0bb1459ec676eafcf379f9f7a0823a3cf2be7a3d9f54c2e2c53a9',
}
PARAMETERS = {
    'FRAME_CLASS': 2, 'SERIAL0_PROTOCOL': 2, 'SERIAL1_PROTOCOL': 2,
    'SIM_WIND_SPD': 0, 'SIM_TIDE_SPEED': 0, 'SIM_WAVE_ENABLE': 0,
    'RNGFND1_TYPE': 17, 'RNGFND1_ORIENT': 25, 'SERIAL7_PROTOCOL': 9, 'SERIAL7_BAUD': 9600,
    'RNGFND3_TYPE': 2, 'RNGFND3_ADDR': 112, 'RNGFND3_ORIENT': 0,
    'RNGFND6_TYPE': 2, 'RNGFND6_ADDR': 113, 'RNGFND6_ORIENT': 25,
}
HOME = (42.358456, -71.087589, 10, 0)
INTERVALS = '33 250\n11038 200\n'
PLUGINS = ('water_depth',)


def app_clock():
    # Darwin C++ steady_clock and Python mach_absolute_time use different epochs.
    return time.clock_gettime(time.CLOCK_MONOTONIC_RAW)


def preflight(args):
    from pymavlink.dialects.v20 import all as codec

    require(sys.platform == 'darwin' and hasattr(time, 'CLOCK_MONOTONIC_RAW') and app_clock() > 0,
            'this native artifact and acquisition clock use Darwin')
    rover, moos, build = (path.resolve() for path in (args.rover_root, args.moos_root, args.build))
    require(revision(rover) == ROVER and clean_source(moos) == MOOS, 'Rover or stock MOOS source pin differs')
    source_ref = clean_source(ROOT)
    cache = build / 'CMakeCache.txt'
    text = cache.read_text()
    app_source = Path(re.search(r'^CMAKE_HOME_DIRECTORY:INTERNAL=(.+)$', text, re.M)[1]).resolve()
    require(clean_source(app_source) == args.app_revision and
            Path(re.search(r'^MOOS_IVP_ROOT:[^=]+=(.+)$', text, re.M)[1]).resolve() == moos,
            'app build source or MOOS path differs')
    defaults = [rover / 'Tools/autotest/default_params' / name for name in DEFAULTS]
    paths = {'native': rover / 'build/sitl/bin/ardurover', 'app': build / 'bin/iMAVLink',
             'probe': build / 'bin/observer_matrix_probe', 'module': build / 'plugins/mavmoos_water_depth.so',
             'antler': moos / 'bin/pAntler', 'db': moos / 'bin/MOOSDB', 'helm': moos / 'bin/pHelmIvP'}
    require(all(path.is_file() and not path.is_symlink() for path in [*defaults, *paths.values()]),
            'required regular native, app, MOOS or defaults artifact missing')
    expected = {'native': NATIVE_SHA, 'helm': HELM, 'app': args.app_sha256,
                'probe': args.probe_sha256, 'module': args.module_sha256}
    require(all(sha(paths[role]) == digest for role, digest in expected.items())
            and all(sha(path) == DEFAULTS[path.name] for path in defaults), 'frozen fixture digest differs')
    message = codec.mavlink_map.get(11038)
    require(message is not None and message.crc_extra == 47 and message.unpacker.format == '<IiiffffffBB',
            'independent Python WATER_DEPTH dialect differs')
    helpers = load_helpers(ROOT)
    protected = [cache, *paths.values(), *defaults, Path(codec.__file__).resolve(),
                 Path(sys.executable).resolve(), Path(__file__).resolve(),
                 ROOT / 'tests/water_depth/native_audit.py', ROOT / 'tests/water_depth/audit.py',
                 app_source / 'src/plugins/WaterDepthPlugin.cpp',
                 *(Path(module.__file__).resolve() for module in helpers.values())]
    return codec, helpers, paths, defaults, protected, source_ref, app_source


def capture(out, paths, defaults, codec, helpers, report):
    processes, rows = [], []
    receiver = None
    boundary = None
    forward = threading.Event()
    decoder = helpers['wire'].FrameDecoder(codec.MAVLink(None), expected_type=11)
    ports = report['ports'] = {}
    report['egress_datagrams'] = report['forwarded_bytes'] = 0
    with ExitStack() as stack:
        console, native, peer, bridge = [stack.enter_context(reserve(socket.SOCK_DGRAM)) for _ in range(4)]
        db, base = [stack.enter_context(reserve(socket.SOCK_STREAM)) for _ in range(2)]
        rc = stack.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
        rc.bind(('0.0.0.0', 0))
        ports.update({name: sock.getsockname()[1] for name, sock in (
            ('console_rx', console), ('native_rx', native), ('app_peer', peer), ('bridge', bridge),
            ('moos', db), ('native_base', base), ('rc', rc))})
        native.setblocking(False)
        peer.setblocking(False)
        console.settimeout(.5)
        egress = stack.enter_context((out / 'bridge-egress.mavlink').open('xb'))
        raw = stack.enter_context((out / 'native-wire.mavlink').open('xb'))
        trace = stack.enter_context((out / 'native-wire.jsonl').open('x'))
        packets = stack.enter_context((out / 'native-datagrams.jsonl').open('x'))

        def launch(command, log, env=None):
            process = start(command, out, env, log)
            processes.append(process)
            return process

        def native_proof(sender, target):
            evidence = inventory(sitl)
            require(sender[0] == '127.0.0.1' and exact_peer(evidence, sender[1], target),
                    'native sender not owned by the exact Rover PID')
            return evidence

        def consume(data, _sender, at):
            nonlocal boundary
            offset = decoder.input_bytes
            require(offset + len(data) <= MAX_WIRE_BYTES, 'native wire capture bound exceeded')
            if forward.is_set() and boundary is None and decoder.frames and offset == decoder.consumed_bytes:
                boundary = offset, len(rows)
            raw.write(data)
            messages, _ = decoder.feed(data, at)
            rows.extend(messages)
            for item in messages:
                trace.write(json.dumps(item, allow_nan=False) + '\n')
            if boundary is not None:
                require(peer.sendto(data, ('127.0.0.1', ports['bridge'])) == len(data), 'short one-way relay')
                report['forwarded_bytes'] += len(data)
            packets.write(json.dumps({'offset': offset, 'length': len(data), 'mono': at,
                                      'forwarded': boundary is not None}) + '\n')

        probe = None

        def check():
            receiver.check()
            require(all(proc.poll() is None for proc in processes if proc is not probe),
                    'owned Rover or passive app exited')
            report['egress_datagrams'] += take_egress(peer, ports['bridge'], egress)
            require(report['egress_datagrams'] == 0, 'receive-only app emitted an outbound datagram')

        try:
            overlay, intervals = out / 'fixture.parm', out / 'message-intervals-chan1.txt'
            overlay.write_text(''.join(f'{key} {value}\n' for key, value in PARAMETERS.items()))
            intervals.write_text(INTERVALS)
            report['fixture_sha256'] = {path.name: sha(path) for path in (overlay, intervals)}
            command = [str(paths['native']), '--model', 'motorboat-skid', '--home', ','.join(map(str, HOME)),
                       '--speedup', '1',
                       '--wipe', '--defaults', ','.join(map(str, [*defaults, overlay])),
                       '--serial0', f"udpclient:127.0.0.1:{ports['console_rx']}",
                       '--serial1', f"udpclient:127.0.0.1:{ports['native_rx']}",
                       '--serial2', 'none', '--serial5', 'none', '--serial6', 'none',
                       '--serial7', 'sim:nmea', '--serial8', 'none', '--base-port', str(ports['native_base']),
                       '--rc-in-port', str(ports['rc'])]
            report['native_command'] = command
            base.close()
            rc.close()
            sitl = launch(command, 'sitl.log')
            report['sitl_pid'] = sitl.pid
            receiver = helpers['passive'].PassiveReceiver(
                native, lambda sender: native_proof(sender, ports['native_rx']), consume,
                max_datagrams=10000, clock=app_clock)
            receiver.start()

            def ready():
                observed = tuple(rows)
                sensors = {item['id'] for item in observed if item['mavpackettype'] == 'WATER_DEPTH'}
                return (receiver.peer and sensors == SENSORS
                        and any(item['mavpackettype'] == 'HEARTBEAT' for item in observed)
                        and any(item['mavpackettype'] == 'GLOBAL_POSITION_INT' and
                                abs(item['lat'] / 1e7 - HOME[0]) <= .01 and
                                abs(item['lon'] / 1e7 - HOME[1]) <= .01 for item in observed))

            wait_for(60, ready, check, 'native NMEA and I2C depthfinders not observed at fixture home')
            report['native_route'] = receiver.peer_proof
            ports['native_source'] = receiver.peer[1]
            console_data, address = console.recvfrom(65535)
            require(console_data, 'owned Rover console evidence missing')
            report['console_route'] = native_proof(address, ports['console_rx'])
            ports['console_source'] = address[1]
            (out / 'serial0-first-datagram.bin').write_bytes(console_data)

            mission = out / 'boat-water-depth.moos'
            config = mission_text(ports['moos'], ports['bridge'], ports['app_peer'], 3, PLUGINS)
            config = config.replace('  expected_type = 2\n',
                                    '  expected_type = 11\n  vehicle_profile = ardurover_boat\n')
            config = config.replace('  simulation_only = false\n',
                                    f"  plugin_directory = {paths['module'].parent}\n"
                                    '  simulation_only = false\n  helm_enabled = false\n')
            mission.write_text('\n'.join(line for line in config.splitlines() if 'plugin.odometry.' not in line) + '\n')
            env = dict(os.environ)
            env['PATH'] = os.pathsep.join((str(paths['app'].parent), str(paths['antler'].parent), env.get('PATH', '')))
            bridge.close()
            db.close()
            antler = launch([str(paths['antler']), str(mission)], 'pantler.log', env)

            def app_ready():
                try:
                    return live_app(helpers['ownership'], antler, mission, ports['bridge'])
                except RuntimeError:
                    return None

            app, report['app_route'] = wait_for(10, app_ready, check, 'exact passive app route missing')
            report['owned_app'] = app
            probe = launch([str(paths['probe']), str(ports['moos']), '10', '--watch=' + WATER_DEPTH],
                           'probe.log', env)
            forward.set()
            wait_for(16, lambda: probe.poll() is not None, check, 'native depth probe exceeded deadline')
            require(probe.returncode == 0, 'native depth probe failed')
            _, report['final_app_route'] = live_app(helpers['ownership'], antler, mission, ports['bridge'], app)
            report['final_native_route'] = native_proof(receiver.peer, ports['native_rx'])
            receiver.close()
            report['wire_decode'] = decoder.finish()
            check()
            require(boundary is not None, 'no complete forwarded MAVLink boundary')
        finally:
            errors, remaining = [], []
            if receiver is not None:
                try:
                    if not receiver.status()['socket_closed']:
                        receiver.close()
                except Exception as error:
                    errors.append(str(error))
                report['capture'] = receiver.status()
            for process in reversed(processes):
                try:
                    remaining.extend(helpers['ownership'].stop_process_tree(process))
                except Exception as error:
                    errors.append(str(error))
            try:
                report['egress_datagrams'] += take_egress(peer, ports['bridge'], egress)
            except Exception as error:
                errors.append(str(error))
            report['cleanup_errors'], report['owned_processes_remaining'] = errors, remaining
    return rows, boundary


def run(args, setup, lease):
    codec, helpers, paths, defaults, protected, source_ref, app_source = setup
    before = {str(path): sha(path) for path in protected}
    out = args.output.resolve()
    out.mkdir(mode=0o700, parents=True, exist_ok=False)
    report = {'result': 'FAIL', 'schema': 1, 'scenario': 'native_boat_water_depth_rx_only',
              'mavmoos_revision': args.app_revision, 'runner_revision': source_ref,
              'ardupilot_revision': ROVER, 'moos_revision': MOOS, 'vehicle_profile': 'ardurover_boat',
              'model': 'motorboat-skid', 'home': HOME, 'timewarp': 1, 'observer_writes_to_fc': 0, 'hardware': False, 'native_lock': str(lease),
              'acquisition_clock': 'clock_gettime(CLOCK_MONOTONIC_RAW)', 'artifacts_before': before,
              **{f'{role}_sha256': before[str(paths[role])] for role in ('app', 'probe', 'native', 'helm')}}
    try:
        rows, (offset, first) = capture(out, paths, defaults, codec, helpers, report)
        raw = (out / 'native-wire.mavlink').read_bytes()
        report['egress_bytes'] = (out / 'bridge-egress.mavlink').stat().st_size
        require(report['egress_datagrams'] == report['egress_bytes'] == 0 and
                report['forwarded_bytes'] == len(raw) - offset > 0, 'wrong one-way bytes or bridge egress')
        report['forwarded_first_byte'], report['forwarded_first_row'] = offset, first
        factory = lambda: codec.MAVLink(None)
        report['replay'] = helpers['wire'].replay(raw, rows, factory, expected_type=11)
        report['forwarded_replay'] = helpers['wire'].replay(raw[offset:], rows[first:], factory, expected_type=11)
        probe = out / 'probe.log'
        samples, report['loaded_modules_sha256'] = audit(probe, PLUGINS, paths['module'].parent)
        runtime = [fields(line.split('\t', 2)[2]) for line in probe.read_text().splitlines()
                   if line.startswith('CAPTURE\tMAVMOOS_RUNTIME_STATUS\t')]
        require(runtime and all(row['armed'] == '0' and row['control_profile'] == 'none' for row in runtime)
                and any(row['heartbeat_valid'] == '1' for row in runtime), 'app disarmed observer state missing')
        report['observation_evidence'] = audit_native(raw[offset:], rows[first:], samples[WATER_DEPTH])
        report['result'] = 'PASS'
    except BaseException as error:
        report['error'] = str(error) or type(error).__name__
    finally:
        try:
            ports = report['ports_clear'] = clean_ports(report.get('ports', {}))
            status = report.get('capture', {})
            require(ports and all(ports.values()) and report.get('owned_processes_remaining') == []
                    and not report.get('cleanup_errors') and status.get('thread_retired') and status.get('socket_closed')
                    and status.get('error') is None, 'native fixture teardown not proved')
            report['cleanup_proven'] = True
            after = report['artifacts_after'] = {name: sha(name) for name in before}
            require(before == after and clean_source(ROOT) == source_ref and clean_source(app_source) == args.app_revision
                    and revision(args.rover_root) == ROVER and clean_source(args.moos_root) == MOOS
                    and all(sha(out / name) == value for name, value in report['fixture_sha256'].items()),
                    'frozen observer inputs changed')
        except Exception as error:
            report['result'], report['postflight_error'] = 'FAIL', str(error)
        report['files'] = {file.name: sha(file) for file in sorted(out.iterdir())
                           if file.is_file() and not file.is_symlink()}
        dump(out / 'result.json', report)
    print(f"{report['result']} {out / 'result.json'}")
    return int(report['result'] != 'PASS')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('rover-root', 'moos-root', 'build'):
        parser.add_argument('--' + name, required=True, type=Path)
    for name in ('app-revision', 'app-sha256', 'probe-sha256', 'module-sha256'):
        parser.add_argument('--' + name, required=True)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--check-inputs', action='store_true')
    args = parser.parse_args()
    if not args.check_inputs and (args.output is None or args.output.exists()):
        parser.error('--output must name a fresh private run directory')
    setup = preflight(args)
    if args.check_inputs:
        print(f'PASS native Rover input identities at {ROVER}')
        return 0
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(143))
    lease = Path(tempfile.gettempdir()) / 'mavmoos-native-sitl.lock'
    with os.fdopen(os.open(lease, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600), 'r+') as lock:
        owner = os.fstat(lock.fileno())
        require(stat.S_ISREG(owner.st_mode) and owner.st_uid == os.getuid(), 'unsafe native lease')
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return run(args, setup, lease)


if __name__ == '__main__':
    raise SystemExit(main())
