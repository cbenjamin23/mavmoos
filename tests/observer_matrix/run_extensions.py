#!/usr/bin/env python3
"""Group optional observers behind one real app and a generated loopback peer."""
import argparse
import errno
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

from run_loopback import audit, complete_capture, reserve, start, take_egress, verify_family, wait_app_status
from owned_processes import stop_process_tree
from air_data.wire_fixture import KEYS as AIR_DATA_KEYS, audit_loopback as audit_air_data
from clock_status.wire_fixture import KEYS as CLOCK_KEYS, audit_loopback as audit_clock
from optical_flow.wire_fixture import KEYS as FLOW_KEYS, audit_loopback as audit_flow
from link_status.wire_fixture import KEYS as LINK_KEYS, audit_loopback as audit_link
from debug_values.wire_fixture import KEYS as DEBUG_KEYS, audit_loopback as audit_debug
from obstacle_observations.audit import OBSTACLE, obstacle_matches
from terrain.audit import REPORT, REQUEST, report_matches, request_matches
from wheel_odometry.audit import WHEELS, wheel_matches
from event_timeline.wire_fixture import KEYS as EVENT_KEYS, audit_loopback as audit_events
from status.power_fixture import KEYS as POWER_KEYS, audit_loopback as audit_power
from water_depth.audit import KEYS as DEPTH_KEYS, audit_loopback as audit_depth

MODULES = ('telemetry', 'optical_flow', 'terrain', 'navigation_monitor', 'link_status', 'odometry', 'clock_status', 'debug_values', 'event_timeline')
CHECKS = {
    'MAVMOOS_NAV_CONTROLLER_OUTPUT': lambda d: d['nav_roll_reported_deg'] == 4.25
        and d['nav_pitch_reported_deg'] == -8.5 and d['nav_bearing_reported_deg'] == -115
        and d['target_bearing_reported_deg'] == 295 and d['waypoint_distance_reported_m'] == 513
        and d['altitude_error_reported_m'] == -4.25 and d['airspeed_error_reported_raw'] == 2.5
        and d['cross_track_error_reported_m'] == -2.75,
    'MAVMOOS_POSITION_TARGET_LOCAL_NED': lambda d: d['time_boot_ms_raw'] == 4294967295
        and d['type_mask_raw'] == 0x0BA2
        and d['coordinate_frame'] == 'local_ned' and d['position_xyz_m'] == [1.25, None, -3.5]
        and d['velocity_xyz_m_s'] == [5, -1.25, None] and d['acceleration_or_force'] == 'force_n'
        and d['acceleration_or_force_xyz'] == [-.5, None, None]
        and d['yaw_rad'] == .25 and d['yaw_rate_rad_s'] is None,
    'MAVMOOS_POSITION_TARGET_GLOBAL_INT': lambda d: d['time_boot_ms_raw'] == 121
        and d['type_mask_raw'] == 0xFDF8
        and d['coordinate_frame'] == 'global_msl' and d['mask_unassigned_bits'] == 0xF000
        and d['latitude_e7_raw'] == 423584560 and d['longitude_e7_raw'] == -710875890
        and d['latitude_deg'] == 42.358456 and d['longitude_deg'] == -71.087589
        and d['altitude_reported_m'] == 65.5 and d['velocity_xyz_m_s'] == [None, None, None]
        and d['yaw_rad'] is None,
    REPORT: report_matches,
    REQUEST: request_matches,
    WHEELS: wheel_matches,
}
GNSS = {
    'MAVMOOS_GPS2_STATUS': lambda d: d['remote_time_usec'] == '18446744073709551000'
        and d['latitude_deg']['known'] and abs(d['latitude_deg']['value'] - 42.3584561) < 1e-8
        and d['longitude_deg']['known'] and abs(d['longitude_deg']['value'] + 71.0875891) < 1e-8
        and d['yaw_deg']['value'] == 0 and d['horizontal_accuracy_m']['value'] == 1.2
        and d['dgps_satellites_reported_raw'] == 7,
    'MAVMOOS_GPS_RTK': lambda d: d['baseline_axes'] == 'ECEF_XYZ'
        and d['baseline_mm_raw'] == [-1500, 2750, -990] and d['accuracy_unit'] == 'unspecified',
    'MAVMOOS_GPS2_RTK': lambda d: d['baseline_axes'] == 'NORTH_EAST_DOWN'
        and d['baseline_mm_raw'] == [3100, 2750, -990] and d['rtk_receiver_id_raw'] == 9,
}


def require(ok, message):
    if not ok:
        raise ValueError(message)


def mission(db, bridge, peer, autopilot, plugin_dir, modules, expected_type=2):
    setup = ''.join(f'  plugin.{key}.sample_timeout = .65\n  plugin.{key}.publish_interval = .08\n'
                    for key in modules if key not in ('telemetry', 'distance_sensor', 'system_status'))
    if 'distance_sensor' in modules:
        setup += '  distance_sensor_timeout = .65\n  distance_sensor_publish_interval = .08\n'
    if 'system_status' in modules:
        setup += '  status_timeout = .65\n  status_publish_interval = .08\n'
    return f'''ServerHost = 127.0.0.1
ServerPort = {db}
Community = observer_extensions_{db}
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
  expected_type = {expected_type}
  expected_autopilot = {autopilot}
  simulation_only = false
  shadow_mode = false
  plugin_directory = {plugin_dir}
  plugins = {','.join(modules)}
{setup}}}
'''


def verify_sample(key, before_bind, before_recovery, samples, autopilot, check):
    require(before_bind.get(key) and all(not row['present'] and row['data'] is None
                                        for row in before_bind[key]), f'{key}: prebind observation admitted')
    for row in samples[key]:
        require(row['read_only'] is True and row['authoritative_for_control'] is False,
                f'{key}: observation claims authority')
        if row['present']:
            timeout = .65 if key in GNSS else row['sample_timeout_s']
            require(row['age_basis'] == 'QUEUE_EMPTY_FLOOR' and row['age_is_upper_bound'] is True
                    and row['fresh'] == (row['age_s'] <= timeout),
                    f'{key}: changed original age semantics')
    return verify_family(key, samples[key], check, autopilot, len(before_recovery[key]))


def run(args):
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    autopilot = 12 if args.identity == 'px4' else 3
    plane = args.identity == 'plane'
    boat = args.identity == 'boat'
    if plane:
        modules, checks, watch = ('air_data',), {}, AIR_DATA_KEYS
        prebind_per_batch, probe_seconds = 4, '8'
        phases = [('prebind', .4, (1,)), ('first', .95, (0, 1)), ('stale', .9, (0, 2)),
                  ('loss', 2.25, ()), ('unbound', .4, (1,)), ('recover', .9, (0, 1))]
    elif boat:
        modules, checks, watch = ('water_depth',), {}, DEPTH_KEYS
        prebind_per_batch, probe_seconds = 2, '9'
        phases = [('prebind', .48, (1,)), ('first', 1.15, (0, 1)), ('stale', 1., (0, 2)),
                  ('loss', 2.3, ()), ('unbound', .48, (1,)), ('recover', 1.1, (0, 1))]
    else:
        modules = MODULES + (('distance_sensor', 'system_status') if autopilot == 3 else ())
        checks = {**CHECKS, **({OBSTACLE: obstacle_matches, **GNSS} if autopilot == 3 else {})}
        watch = (*checks, *FLOW_KEYS, *LINK_KEYS, *CLOCK_KEYS, *DEBUG_KEYS, *EVENT_KEYS,
                 *(POWER_KEYS if autopilot == 3 else ()))
        prebind_per_batch, probe_seconds = (36 if autopilot == 3 else 30), '14'
        phases = [('prebind', .64, (1,)), ('first', 2.1, (0, 1)), ('stale', 1.2, (0, 2)),
                  ('loss', 2.35, ()), ('unbound', .64, (1,)), ('recover', 1.6, (0, 1))]
    report = {'result': 'FAIL', 'identity': args.identity, 'source_kind': 'generated_loopback_frames',
              'hardware': False, 'bridge_egress_datagrams': 0}
    moos_bin, build = args.moos_bin.resolve(), args.build.resolve()
    plugin_dir = build / 'plugins'
    env = {**os.environ, 'PATH': os.pathsep.join(map(str, (build / 'bin', moos_bin, os.environ.get('PATH', ''))))}
    processes = []
    with reserve(socket.SOCK_DGRAM) as peer, (out / 'bridge-egress.mavlink').open('wb') as egress:
        peer.setblocking(False)
        with reserve(socket.SOCK_STREAM) as tcp, reserve(socket.SOCK_DGRAM) as udp:
            db, bridge = tcp.getsockname()[1], udp.getsockname()[1]
        ports = {'moos': db, 'bridge': bridge, 'peer': peer.getsockname()[1]}
        report['ports'] = ports
        try:
            hex_rows = subprocess.check_output([args.frames, args.identity], text=True, timeout=4).splitlines()
            require(len(hex_rows) == 96 * 3, 'incomplete fixture vector batches')
            frames = [bytes.fromhex(row) for row in hex_rows]
            batches = iter(frames[i:i + 3] for i in range(0, len(frames), 3))
            (out / 'fixture-frames.hex').write_text('\n'.join(hex_rows) + '\n')
            config = out / 'observers.moos'
            config.write_text(mission(db, bridge, ports['peer'], autopilot, plugin_dir, modules,
                                      expected_type=1 if plane else 11 if boat else 2))
            antler = start([str(moos_bin / 'pAntler'), str(config)], out, env, 'pantler.log')
            processes.append(antler)
            probe = start([args.probe, str(db), probe_seconds, *('--watch=' + key for key in watch)],
                          out, env, 'probe.log')
            processes.append(probe)
            probe_log = out / 'probe.log'
            deadline = time.monotonic() + 4
            while b'CAPTURE\tMAVMOOS_RUNTIME_STATUS\t' not in complete_capture(probe_log):
                require(time.monotonic() < deadline and antler.poll() is None and probe.poll() is None,
                        'real app or probe did not become ready')
                time.sleep(.04)
            pre_bind, pre_recovery = out / 'probe-pre-bind.log', out / 'probe-pre-recovery.log'
            invalid_high_latency = 0
            for name, seconds, selected in phases:
                if name == 'recover':
                    time.sleep(.12)
                    pre_recovery.write_bytes(complete_capture(probe_log))
                until, sent = time.monotonic() + seconds, 0
                while time.monotonic() < until:
                    require(antler.poll() is None and probe.poll() is None, f'process stopped in {name}')
                    if selected:
                        batch = next(batches)
                        for index in selected:
                            peer.sendto(batch[index], ('127.0.0.1', bridge))
                        sent += 1
                    report['bridge_egress_datagrams'] += take_egress(peer, bridge, egress)
                    time.sleep(.08)
                if name == 'prebind':
                    status = wait_app_status(probe_log, lambda s: int(s['frames']) == sent * prebind_per_batch,
                                             'prebind frames did not reach real app ingress')
                    report['prebind_frames'] = int(status['frames'])
                    pre_bind.write_bytes(complete_capture(probe_log))
                if name == 'stale':
                    status = wait_app_status(probe_log, lambda s: int(s['wrong_target']) == sent * (2 if plane or boat else 8 if autopilot == 3 else 6),
                                             'wrong system/component frames were not all rejected')
                    report['wrong_target_frames'] = int(status['wrong_target'])
                    if plane:
                        invalid_high_latency = sent * 2
            require(probe.wait(timeout=8) == 0, 'MOOS probe failed')
            final_capture = complete_capture(probe_log)
            require(final_capture.startswith(pre_recovery.read_bytes()) and
                    pre_recovery.read_bytes().startswith(pre_bind.read_bytes()),
                    'prebind or prerecovery capture differs from the recorded app trace')
            samples, loaded = audit(probe_log, modules, plugin_dir)
            before_bind, _ = audit(pre_bind, modules, plugin_dir)
            before_recovery, _ = audit(pre_recovery, modules, plugin_dir)
            report['loaded_modules_sha256'] = loaded
            if plane:
                report['air_data'] = audit_air_data(before_bind, before_recovery, samples, invalid_high_latency)
                report['vfr_hud'] = verify_sample('MAVMOOS_VFR_HUD', before_bind, before_recovery, samples, autopilot,
                                                  lambda d: d['airspeed_reported_m_s'] == 5.5
                                                  and d['airspeed_basis'] == 'fc_reported_unspecified')
            elif boat:
                report['water_depth'] = audit_depth(before_bind, before_recovery, samples, autopilot)
            else:
                report['samples'] = {key: verify_sample(key, before_bind, before_recovery, samples, autopilot, check)
                                     for key, check in checks.items()}
                report['optical_flow'] = audit_flow(before_bind, before_recovery, samples, autopilot)
                report['link_status'] = audit_link(before_bind, before_recovery, samples, autopilot)
                report['clock_status'] = audit_clock(before_bind, before_recovery, samples, autopilot)
                report['debug_values'] = audit_debug(before_bind, before_recovery, samples, autopilot)
                report['event_timeline'] = audit_events(before_bind, before_recovery, samples, autopilot)
                if autopilot == 3:
                    report['power_diagnostics'] = audit_power(before_bind, before_recovery, samples, autopilot)
            report['result'] = 'PASS'
        except (OSError, ValueError, RuntimeError, KeyError, TypeError, StopIteration, subprocess.SubprocessError) as error:
            report['error'] = str(error) or type(error).__name__
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
    if not all(clear.values()) or report['owned_processes_remaining'] != [] or report['bridge_egress_datagrams'] != 0:
        report['result'] = 'FAIL'
        report.setdefault('error', 'unexpected app egress or cleanup uncertainty')
    report['app_sha256'] = hashlib.sha256((build / 'bin/iMAVLink').read_bytes()).hexdigest()
    report['files'] = {name.name: hashlib.sha256(name.read_bytes()).hexdigest()
                       for name in out.iterdir() if name.is_file() and not name.is_symlink()}
    (out / 'result.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps(report, indent=2, allow_nan=False))
    return int(report['result'] != 'PASS')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--identity', choices=('ardupilot', 'px4', 'plane', 'boat'), required=True)
    for arg in ('moos-bin', 'build', 'output'):
        parser.add_argument('--' + arg, type=Path, required=True)
    for arg in ('frames', 'probe'):
        parser.add_argument('--' + arg, required=True)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(143))
    raise SystemExit(run(parser.parse_args()))
