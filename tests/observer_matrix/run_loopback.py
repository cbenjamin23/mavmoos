#!/usr/bin/env python3
"""Exercise the real app with a generated, receive-only UDP peer."""
import argparse
import errno
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from control.catalog_evidence import loaded_module_hashes, observed_catalog
from imu.audit_native import fields
from owned_processes import stop_process_tree
from status.DiagnosticsEvidence import audit_loopback as audit_diagnostics
from traffic.audit import KEY as TRAFFIC_KEY, audit_loopback as audit_traffic

LANDING_COMPONENT = 158
LANDING_KEY = 'MAVMOOS_LANDING_TARGET'


def reserve(kind):
    stream = socket.socket(socket.AF_INET, kind)
    stream.bind(('127.0.0.1', 0))
    return stream


def mission_text(db, bridge, peer, autopilot, plugins, traffic_timeout=3, status_timeout=3, plugin_directory=None):
    landing = f'  landing_target_component = {LANDING_COMPONENT}\n' if 'landing_target' in plugins else ''
    traffic = f'  plugin.traffic.sample_timeout = {traffic_timeout}\n' if 'traffic' in plugins else ''
    diagnostics = (f'  status_timeout = {status_timeout}\n  status_publish_interval = 0.08\n'
                   if 'system_status' in plugins else '')
    module_path = f'  plugin_directory = {plugin_directory}\n' if plugin_directory else ''
    return f'''ServerHost = 127.0.0.1
ServerPort = {db}
Community = observer_matrix_{db}
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
  expected_autopilot = {autopilot}
  simulation_only = false
  shadow_mode = false
  plugins = {','.join(plugins)}
  plugin.odometry.sample_timeout = 0.8
  plugin.odometry.publish_interval = 0.08
{landing}{traffic}{diagnostics}{module_path}}}
'''


def start(command, out, env, name):
    with (out / name).open('w') as stream:
        return subprocess.Popen(command, cwd=out, env=env, stdout=stream,
                                stderr=subprocess.STDOUT, start_new_session=True)


def take_egress(peer, bridge, stream):
    count = 0
    while count < 128:
        try:
            data, address = peer.recvfrom(65535)
        except BlockingIOError:
            return count
        if address != ('127.0.0.1', bridge):
            raise ValueError('unexpected datagram source on owned app peer')
        stream.write(data)
        count += 1
    raise ValueError('app egress capture exceeded datagram bound')


def audit(log, plugins, plugin_dir):
    catalog = observed_catalog(log, set(plugins), mode='RX_ONLY')
    if any(module['request_owners'] or module['control_provider']
           for module in catalog['composition']['modules']):
        raise ValueError('receive-only composition owns effects')
    samples = {}
    statuses = {'MAVMOOS_STATUS': 0, 'MAVMOOS_RUNTIME_STATUS': 0}
    for line in log.read_text().splitlines():
        if not line.startswith('CAPTURE\t'):
            continue
        _, key, value = line.split('\t', 2)
        if key == 'MAVMOOS_RUNTIME_STATUS':
            status = fields(value)
            if any(status.get(name) != '0' for name in ('tx_attempts', 'tx_accepted', 'nav_tx_accepted')) or \
                    status.get('mode') != 'RX_ONLY' or status.get('permission') != 'false':
                raise ValueError('runtime reported non-passive state')
        elif key == 'MAVMOOS_STATUS':
            status = fields(value)
            if any(status.get(name) != '0' for name in ('mavlink_tx_attempts', 'mavlink_tx_accepted')) or \
                    status.get('state') != 'RX_ONLY' or status.get('tx_enabled') != 'false':
                raise ValueError('app reported non-passive state')
        elif key != 'MAVMOOS_CAPABILITIES':
            samples.setdefault(key, []).append(json.loads(value))
        if key in statuses:
            statuses[key] += 1
    if not all(statuses.values()):
        raise ValueError('app/runtime passive counters missing')
    return samples, loaded_module_hashes(plugin_dir, catalog)


def complete_capture(log):
    raw = log.read_bytes()
    return raw[:raw.rfind(b'\n') + 1]


def wait_app_status(log, condition, error):
    prefix = b'CAPTURE\tMAVMOOS_STATUS\t'
    deadline = time.monotonic() + .5
    while time.monotonic() < deadline:
        for line in reversed(complete_capture(log).splitlines()):
            if line.startswith(prefix):
                status = fields(line[len(prefix):].decode())
                if condition(status):
                    return status
                break
        time.sleep(.025)
    raise ValueError(error)


def verify_family(key, observations, expect, autopilot, before_recovery, component=1):
    fresh = {}
    stale = None
    cleared = False
    recovered = None
    for index, entry in enumerate(observations):
        if 'groups' in entry:
            group = next(iter(entry['groups']), None)
            sample = group['metadata'] if group else {'present': False}
            data = group
        else:
            sample, data = entry, entry['data']
        if not sample['present']:
            if stale is not None and index < before_recovery:
                cleared = True
            continue
        if cleared and index < before_recovery:
            raise ValueError(f'{key}: data-only packets restored a cleared session')
        if sample['system_id'] != 1 or sample['component_id'] != component or sample['autopilot'] != autopilot or \
                sample['source_age'] != 'unknown' or sample['authoritative_for_control'] is not False or \
                not (0 < sample['effective_at'] <= sample['read_at'] and sample['age_s'] >= 0):
            raise ValueError(f'{key}: invalid source/time/provenance')
        if key == LANDING_KEY and (sample['read_only'] is not True or sample['age_basis'] != 'QUEUE_EMPTY_FLOOR'
                                  or sample['age_is_upper_bound'] is not True
                                  or sample['fresh'] != (sample['age_s'] <= sample['sample_timeout_s'])):
            raise ValueError(f'{key}: invalid original-age contract')
        identity = sample['session_epoch'], sample['observation_id']
        times = sample['effective_at'], sample['read_at']
        if identity in fresh and times != fresh[identity]:
            raise ValueError(f'{key}: repeated publication renewed original timestamps')
        if sample['fresh']:
            if not expect(data):
                raise ValueError(f'{key}: unexpected decoded payload {data}')
            if recovered is None and cleared and identity[0] > stale[0]:
                recovered = identity
            fresh[identity] = times
        elif stale is None and identity in fresh:
            stale = identity
    if recovered is None:
        raise ValueError(f'{key}: missing fresh/stale/cleared/recovered sequence')
    return {'fresh': len(fresh), 'stale': list(stale), 'recovered': list(recovered)}


def run(args):
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    px4 = args.identity == 'px4'
    autopilot = 12 if px4 else 3
    plugins = ['telemetry', 'odometry']
    if not px4:
        plugins.extend(('air_data', 'esc_status', 'imu', 'landing_target', 'traffic', 'system_status'))
    frames = [bytes.fromhex(line) for line in subprocess.check_output(
        [args.frames, args.identity], text=True, timeout=3).splitlines()]
    valid_width = 4 if px4 else 16
    batch_width = 4 if px4 else 24
    if len(frames) != 96 * batch_width:
        raise ValueError('missing generated fixture frames')
    batches = iter(frames[index:index + batch_width] for index in range(0, len(frames), batch_width))
    (out / 'fixture-frames.hex').write_text(''.join(frame.hex() + '\n' for frame in frames))
    processes = []
    report = {'result': 'FAIL', 'source_kind': 'generated_loopback_frames', 'identity': args.identity,
              'bridge_egress_datagrams': 0}
    with reserve(socket.SOCK_DGRAM) as peer, (out / 'bridge-egress.mavlink').open('wb') as egress:
        peer.setblocking(False)
        with reserve(socket.SOCK_STREAM) as tcp, reserve(socket.SOCK_DGRAM) as udp:
            db, bridge = tcp.getsockname()[1], udp.getsockname()[1]
        ports = {'moos': db, 'bridge': bridge, 'peer': peer.getsockname()[1]}
        report['ports'] = ports
        mission = out / 'observers.moos'
        explicit_dir = args.plugin_directory.resolve() if args.plugin_directory else None
        plugin_dir = explicit_dir or args.build.resolve() / 'plugins'
        mission.write_text(mission_text(db, bridge, ports['peer'], autopilot, plugins,
                                        traffic_timeout=.8, status_timeout=.8, plugin_directory=explicit_dir))
        moos_bin = args.moos_bin.resolve()
        env = dict(os.environ)
        env['PATH'] = os.pathsep.join((str(args.build.resolve() / 'bin'), str(moos_bin), env.get('PATH', '')))
        try:
            antler = start([str(moos_bin / 'pAntler'), str(mission)], out, env, 'pantler.log')
            processes.append(antler)
            probe_command = [args.probe, str(db), '14']
            if not px4:
                probe_command.extend(('--landing-target', '--traffic', '--status-diagnostics'))
            probe = start(probe_command, out, env, 'probe.log')
            processes.append(probe)
            probe_log = out / 'probe.log'
            deadline = time.monotonic() + 4
            while 'CAPTURE\tMAVMOOS_RUNTIME_STATUS\t' not in probe_log.read_text():
                if time.monotonic() >= deadline or antler.poll() is not None or probe.poll() is not None:
                    raise ValueError('actual app/probe readiness deadline')
                time.sleep(.05)
            pre_recovery = out / 'probe-pre-recovery.log'
            pre_bind = out / 'probe-pre-bind.log'
            rejected_indices = () if px4 else tuple(range(valid_width, valid_width + 7))
            phases = [] if px4 else [('prebind', .5, (12, 13, 15))]
            during_stale = () if px4 else (13, 23)
            phases.extend((('first', 2.3, range(valid_width)), ('stale', 1.4, (0, *during_stale, *rejected_indices)),
                           ('loss', 2.4, ()), ('rebind_required', .5, range(1, valid_width)),
                           ('recovery', 1.6, range(valid_width))))
            for phase, seconds, selected in phases:
                if phase == 'recovery':
                    time.sleep(.15)
                    pre_recovery.write_bytes(complete_capture(probe_log))
                started = time.monotonic()
                sent_batches = 0
                while time.monotonic() - started < seconds:
                    if antler.poll() is not None or probe.poll() is not None:
                        raise ValueError(f'app or probe stopped during {phase}')
                    if selected:
                        batch = next(batches)
                        for index in selected:
                            peer.sendto(batch[index], ('127.0.0.1', bridge))
                        sent_batches += 1
                    report['bridge_egress_datagrams'] += take_egress(peer, bridge, egress)
                    time.sleep(.08)
                if phase == 'prebind':
                    # Two single records and one six-frame diagnostics datagram.
                    status = wait_app_status(probe_log, lambda row: int(row['frames']) == sent_batches * 8
                                             and row['wrong_target'] == '0',
                                             'prebind sensor data did not reach app ingress')
                    pre_bind.write_bytes(complete_capture(probe_log))
                    report['landing_target_route'] = {'component': LANDING_COMPONENT,
                                                      'prebind_datagrams': sent_batches}
                    report['traffic_route'] = {'prebind_datagrams': sent_batches,
                                               'aggregate_prebind_decoded_frames': int(status['frames'])}
                if phase == 'stale' and not px4:
                    # Five legacy observer mismatches and two diagnostics frames
                    # fail ingress. The invalid ICAO reaches its module decoder.
                    expected = sent_batches * 7
                    status = wait_app_status(probe_log, lambda row: int(row['wrong_target']) == expected,
                                             'landing target negatives did not all fail ingress identity checks')
                    report['landing_target_route'].update({
                        'negative_datagrams_by_case': {name: sent_batches for name in
                            ('wrong_system', 'wrong_component', 'sensor_heartbeat')},
                        'wrong_target_observed': sent_batches * 3,
                    })
                    report['traffic_route'].update({'wrong_system': sent_batches, 'wrong_component': sent_batches,
                                                    'invalid_icao': sent_batches,
                                                    'aggregate_wrong_target_observed': int(status['wrong_target'])})
                    report['diagnostic_route'] = {'wrong_system': sent_batches, 'wrong_component': sent_batches}
            if probe.wait(timeout=9) != 0:
                raise ValueError('MOOS probe did not complete')
            samples, modules = audit(probe_log, plugins, plugin_dir)
            before_recovery, _ = audit(pre_recovery, plugins, plugin_dir)
            checks = {
                'MAVMOOS_LOCAL_KINEMATICS': lambda row: row['position_m'] == [1.25, -2.5, -3.75],
                'MAVMOOS_ODOMETRY': lambda row: row['frame_id'] == (1 if px4 else 20) and row['child_frame_id'] == 12
                    and row['time_usec_raw'] == '9007199254740993' and row['quaternion_wxyz'] == [.5, .5, -.5, .5]
                    and (row['pose_covariance'][1] is None if px4 else row['pose_covariance'] is None),
                'MAVMOOS_LOCAL_ORIGIN': lambda row: row['latitude_deg_e7'] == 423584560
                    and row['longitude_deg_e7'] == -710875890 and row['altitude_msl_m'] == 12.345
                    and row['time_usec_raw'] == '9007199254740993',
            }
            if not px4:
                before_bind, _ = audit(pre_bind, plugins, plugin_dir)
                if not before_bind.get(LANDING_KEY) or any(
                        row['present'] or row['fresh'] or row['data'] is not None for row in before_bind[LANDING_KEY]):
                    raise ValueError('landing target published before a flight-controller heartbeat')
                report['landing_target_route']['prebind_absent'] = True
                report['traffic_evidence'] = audit_traffic(
                    before_bind[TRAFFIC_KEY], before_recovery[TRAFFIC_KEY], samples[TRAFFIC_KEY])
                report['diagnostic_evidence'] = audit_diagnostics(before_bind, before_recovery, samples)
                checks.update({
                    'MAVMOOS_VFR_HUD': lambda row: row['airspeed_reported_m_s'] == 5.5 and row['groundspeed_m_s'] == 7.25,
                    'MAVMOOS_WIND': lambda row: row['direction_from_deg'] == -135.5,
                    'MAVMOOS_WIND_COV': lambda row: row['north_m_s'] == 3.25 and row['east_m_s'] is None,
                    'MAVMOOS_ESC_INFO': lambda row: row['first_index'] == 4 and len(row['entries']) == 2,
                    'MAVMOOS_ESC_STATUS': lambda row: row['first_index'] == 0 and len(row['entries']) == 4,
                    'MAVMOOS_ESC_TELEMETRY': lambda row: row['entries'][0]['voltage_cv'] == 1680,
                    'MAVMOOS_SCALED_PRESSURE': lambda row: row['press_abs_hpa'] == 1007.25
                        and row['press_diff_hpa'] == -1.75 and row['temperature_press_diff_cdeg_raw'] == -125,
                    'MAVMOOS_IMU_VIBRATION': lambda row: row['vibration_m_s2'] == [1.25, 2.5, 3.75]
                        and row['clipping_counts'] == [7, 4294967295, 0]
                        and row['time_usec_raw'] == '18446744073709551615',
                    LANDING_KEY: lambda row: row == {
                        'message_id': 149, 'wire_version': 2, 'payload_length': 60,
                        'time_usec_raw': '9007199254740995', 'target_num': 42, 'frame': 12,
                        'target_type_raw': 2, 'distance_m': 3.5, 'position_valid': True,
                        'angle_x_rad': .125, 'angle_y_rad': -.25, 'position_m': [.75, -1.25, 3],
                    },
                })
            report['loaded_modules_sha256'] = modules
            report['samples'] = {key: verify_family(key, samples[key], check, autopilot, len(before_recovery[key]),
                                                   LANDING_COMPONENT if key == LANDING_KEY else 1)
                                 for key, check in checks.items()}
            report['result'] = 'PASS'
        except (OSError, ValueError, RuntimeError, KeyError, TypeError, subprocess.SubprocessError) as error:
            report['error'] = str(error)
        finally:
            remaining, errors = [], []
            for process in reversed(processes):
                try:
                    remaining.extend(stop_process_tree(process))
                except (OSError, ValueError, subprocess.SubprocessError) as error:
                    errors.append(str(error))
            report['owned_processes_remaining'] = None if errors else remaining
            report['cleanup_errors'] = errors
            try:
                report['bridge_egress_datagrams'] += take_egress(peer, bridge, egress)
            except (OSError, ValueError) as error:
                report['result'] = 'FAIL'
                report.setdefault('error', str(error))
            if report['bridge_egress_datagrams'] != 0:
                report['result'] = 'FAIL'
                report.setdefault('error', 'app emitted bytes to its configured peer')
    clear = {}
    for name, port in ports.items():
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM if name == 'moos' else socket.SOCK_DGRAM) as check:
            if name == 'moos':
                check.settimeout(.2)
                clear[name] = check.connect_ex(('127.0.0.1', port)) == errno.ECONNREFUSED
            else:
                try:
                    check.bind(('127.0.0.1', port))
                except OSError:
                    clear[name] = False
                else:
                    clear[name] = True
    report['ports_clear'] = clear
    report['cleanup_proven'] = report['owned_processes_remaining'] == [] and all(clear.values())
    if not report['cleanup_proven']:
        report['result'] = 'FAIL'
    text = json.dumps(report, indent=2, allow_nan=False)
    (out / 'result.json').write_text(text + '\n')
    print(text)
    return int(report['result'] != 'PASS')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--identity', choices=('ardupilot', 'px4'), required=True)
    for name in ('moos-bin', 'build', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    for name in ('frames', 'probe'):
        parser.add_argument('--' + name, required=True)
    parser.add_argument('--plugin-directory', type=Path, help='Set an explicit path for a relocated module layout')
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(143))
    raise SystemExit(run(parser.parse_args()))
