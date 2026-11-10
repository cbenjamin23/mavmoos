#!/usr/bin/env python3
"""Check selected camera and gimbal routes in a real receive-only app."""
import argparse
from collections import defaultdict
from contextlib import ExitStack
import hashlib
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tests'))
from imu.audit_native import fields
from observer_matrix.run_loopback import audit, complete_capture, mission_text, reserve, start, take_egress, wait_app_status
from camera_status import audit as camera
from owned_processes import stop_process_tree

MODULES = ('telemetry', 'gimbal_status')
INFORMATION = 'MAVMOOS_GIMBAL_DEVICE_INFORMATION'
ATTITUDE = 'MAVMOOS_GIMBAL_DEVICE_ATTITUDE_STATUS'
CAMERA_CHECKS = {camera.INFORMATION: camera.information_matches, camera.SETTINGS: camera.settings_matches,
                 camera.CAPTURE: camera.capture_matches, camera.STORAGE: camera.storage_matches}
CAMERA_VALID = ('camera_information', 'camera_settings', 'camera_capture', 'camera_storage')
CAMERA_NEGATIVE = ('camera_other_component', 'camera_other_system', 'camera_from_primary', 'camera_image_excluded')


def need(condition, reason):
    if not condition:
        raise RuntimeError(reason)


def read_latest(capture, observed_keys):
    selected = {}
    for line in capture.decode().splitlines():
        if line.startswith('CAPTURE\t'):
            _, key, value = line.split('\t', 2)
            selected[key] = fields(value) if key == 'MAVMOOS_STATUS' else json.loads(value) if key in observed_keys else value
    need(all(key in selected for key in (*observed_keys, 'MAVMOOS_STATUS')),
         'app probe omitted selected payload or app status')
    return selected


def frame_sets(driver, options, expected_count):
    output = subprocess.check_output([str(driver), *options], text=True, timeout=4)
    frames = defaultdict(list)
    for line in output.splitlines():
        name, raw = line.split()
        frames[name].append(bytes.fromhex(raw))
    need(len(frames) == expected_count and all(len(rows) == 100 for rows in frames.values()),
         'generated payload frame fixture shape differs')
    return frames


def verify_bound(snapshot):
    for key in (INFORMATION, ATTITUDE):
        record = snapshot[key]
        need(record['present'] and len(record['devices']) == 1, f'missing selected gimbal observation {key}')
        sample = record['devices'][0]
        need(sample['device_id'] == 0 and sample['metadata']['fresh'] and
             (sample['metadata']['system_id'], sample['metadata']['component_id']) == (1, 154),
             'gimbal observation source or age differs')
    need(snapshot[INFORMATION]['devices'][0]['data']['uid_raw'] == '18446744073709551000',
         'wrong gimbal identity reached published records')
    need(snapshot[ATTITUDE]['devices'][0]['data']['quaternion_wxyz'] == [.5, -.5, .5, -.5],
         'selected attitude differs from encoded frame')


def verify_camera(snapshot):
    for key, check in CAMERA_CHECKS.items():
        record = snapshot[key]
        need(record['present'] and record['fresh'] and
             (record['system_id'], record['component_id']) == (1, 100) and check(record['data']),
             'camera payload, source or age differs: ' + key)


def stamps(snapshot, keys):
    values = {}
    for key in keys:
        row = snapshot[key]
        sample = row['devices'][0]['metadata'] if key in (INFORMATION, ATTITUDE) else row
        values[key] = (sample['session_epoch'], sample['observation_id'], sample['effective_at'], sample['read_at'])
    return values


def exercise(args, out, report):
    frames = frame_sets(args.driver, ('--frames',), 7)
    camera_enabled = args.camera_frames is not None
    modules = MODULES + (('camera_status',) if camera_enabled else ())
    keys = (INFORMATION, ATTITUDE, *(CAMERA_CHECKS if camera_enabled else ()))
    camera_valid = CAMERA_VALID if camera_enabled else ()
    camera_negative = CAMERA_NEGATIVE if camera_enabled else ()
    camera_heartbeat = ('camera_heartbeat',) if camera_enabled else ()
    if camera_enabled:
        extra = frame_sets(args.camera_frames, (), 9)
        need(not (frames.keys() & extra.keys()), 'duplicate generated vector name')
        frames.update(extra)
    report['modules'] = modules
    (out / 'fixture-frames.json').write_text(json.dumps(
        {name: [frame.hex() for frame in packets] for name, packets in frames.items()}, sort_keys=True) + '\n')
    launched = []
    egress_count = 0
    output = out / 'probe.log'
    ports = report['ports'] = {}
    with ExitStack() as stack:
        peer = stack.enter_context(reserve(socket.SOCK_DGRAM))
        db = stack.enter_context(reserve(socket.SOCK_STREAM))
        bridge = stack.enter_context(reserve(socket.SOCK_DGRAM))
        peer.setblocking(False)
        ports.update(peer=peer.getsockname()[1], db=db.getsockname()[1], bridge=bridge.getsockname()[1])
        capture = stack.enter_context((out / 'unexpected-egress.bin').open('xb'))
        sent_trace = stack.enter_context((out / 'fixture-sent.jsonl').open('x'))
        sent_total = defaultdict(int)
        mission = out / 'gimbal-observers.moos'
        text = mission_text(ports['db'], ports['bridge'], ports['peer'], 3, modules)
        settings = ('  camera_component = 100\n  plugin.camera_status.sample_timeout = .65\n'
                    if camera_enabled else '')
        text = text.replace('  simulation_only = false\n', f"  plugin_directory = {args.plugin_directory}\n"
                            '  gimbal_component = 154\n  plugin.gimbal_status.sample_timeout = .65\n' +
                            settings + '  simulation_only = false\n')
        text = text.replace('  plugin.odometry.sample_timeout = 0.8\n', '')
        text = text.replace('  plugin.odometry.publish_interval = 0.08\n', '')
        text = text.replace('Run = iMAVLink @ NewConsole = false',
                            f'Run = {args.app} @ NewConsole = false ~ iMAVLink')
        mission.write_text(text)
        env = dict(os.environ)
        env['PATH'] = os.pathsep.join((str(args.app.parent), str(args.moos_root / 'bin'), env.get('PATH', '')))
        db.close()
        bridge.close()

        def send(stage, seconds, *names):
            nonlocal egress_count
            begun = time.monotonic()
            deadline = begun + seconds
            stage_counts = defaultdict(int)
            while time.monotonic() < deadline:
                need(all(process.poll() is None for process in launched), 'owned loopback process exited')
                for name in names:
                    packet = frames[name].pop(0)
                    need(peer.sendto(packet, ('127.0.0.1', ports['bridge'])) == len(packet),
                         'generated frame handoff was partial')
                    sent_trace.write(json.dumps({'stage':stage, 'vector':name,
                                                 'ordinal':sent_total[name], 'sent_at':time.monotonic()}) + '\n')
                    sent_total[name] += 1
                    stage_counts[name] += 1
                egress_count += take_egress(peer, ports['bridge'], capture)
                need(egress_count == 0, 'app emitted a datagram in receive-only gimbal fixture')
                time.sleep(.1)
            cutoff = complete_capture(output)
            snapshot = read_latest(cutoff, keys)
            report['stages'][stage] = snapshot
            report['phase_evidence'][stage] = {'started':begun, 'ended':time.monotonic(),
                                              'frames':dict(stage_counts), 'probe_cutoff_bytes':len(cutoff)}
            sent_trace.flush()
            return snapshot

        def only_rejected(stage, name, base):
            send(stage, .75, 'heartbeat', name)
            attempted = report['phase_evidence'][stage]['frames'][name]
            expected = base + attempted
            wait_app_status(output, lambda row: int(row['wrong_target']) == expected,
                            'unrelated payload ID did not fail its selected route: ' + name)
            report['phase_evidence'][stage]['wrong_target_before'] = base
            report['phase_evidence'][stage]['wrong_target_after'] = expected
            return expected

        try:
            launched.append(start([str(args.moos_root / 'bin/pAntler'), str(mission)], out, env, 'pantler.log'))
            launched.append(start([str(args.probe), str(ports['db']), '18'], out, env, 'probe.log'))
            before = send('payload_without_fc', 2, 'gimbal_heartbeat', 'information', 'attitude',
                          *camera_heartbeat, *camera_valid)
            need(before['MAVMOOS_STATUS']['heartbeat_fresh'] == 'false' and
                 all(not before[key]['present'] for key in keys),
                 'a payload heartbeat admitted observations without the selected FC')
            wrong = send('foreign_messages', 1.6, 'heartbeat', 'wrong_component', 'wrong_system', 'wrong_family',
                         'gimbal_heartbeat', *camera_negative, *camera_heartbeat)
            need(wrong['MAVMOOS_STATUS']['identity'] == 'EXPECTED' and all(not wrong[key]['present'] for key in keys)
                 and int(wrong['MAVMOOS_STATUS']['wrong_target']) > 0,
                 'foreign payload data or an unrelated message reached the selected source')
            settled_foreign = send('foreign_drained', .25, 'heartbeat')
            count = int(settled_foreign['MAVMOOS_STATUS']['wrong_target'])
            count = only_rejected('excluded_gimbal_id', 'wrong_family', count)
            if camera_enabled:
                only_rejected('excluded_camera_id', 'camera_image_excluded', count)
            bound = send('bound', 1.5, 'heartbeat', 'gimbal_heartbeat', 'information', 'attitude',
                         *camera_heartbeat, *camera_valid)
            verify_bound(bound)
            if camera_enabled:
                verify_camera(bound)
            quiet = send('quiet_stale', .9, 'heartbeat')
            stale = send('payload_stale', .8, 'heartbeat', 'wrong_component', *camera_negative)
            for checkpoint in (quiet, stale):
                need(checkpoint['MAVMOOS_STATUS']['heartbeat_fresh'] == 'true' and
                     all(checkpoint[key]['present'] and not (checkpoint[key]['devices'][0]['metadata'] if key in (
                         INFORMATION, ATTITUDE) else checkpoint[key])['fresh'] for key in keys),
                     'payload samples did not become stale independently of FC heartbeat')
            need(stamps(stale, keys) == stamps(quiet, keys), 'foreign data renewed an old payload sample')
            expired = send('fc_expired', 2.7, 'gimbal_heartbeat', 'information', 'attitude',
                           *camera_heartbeat, *camera_valid)
            need(expired['MAVMOOS_STATUS']['heartbeat_fresh'] == 'false' and
                 int(expired['MAVMOOS_STATUS']['unbound_reports']) > int(stale['MAVMOOS_STATUS']['unbound_reports'])
                 and all(not expired[key]['present'] for key in keys),
                 'payload data or heartbeat renewed the expired FC session')
            restored = send('rebound', 1.5, 'heartbeat', 'information', 'attitude', *camera_valid)
            verify_bound(restored)
            if camera_enabled:
                verify_camera(restored)
            need(all(stamps(restored, keys)[key][0] > stamps(bound, keys)[key][0] for key in keys),
                 'selected FC did not start a fresh payload session')
            closed_prefix = out / 'probe-captured.log'
            closed_prefix.write_bytes(complete_capture(output))
            _, report['loaded_modules_sha256'] = audit(closed_prefix, modules, args.plugin_directory)
        finally:
            errors = []
            remaining = []
            for process in reversed(launched):
                try:
                    remaining.extend(stop_process_tree(process))
                except Exception as error:
                    errors.append(str(error))
            try:
                egress_count += take_egress(peer, ports['bridge'], capture)
            except Exception as error:
                errors.append(str(error))
            report['cleanup_errors'], report['owned_processes_remaining'] = errors, remaining
            report['egress_datagrams'] = egress_count


def cleared(ports):
    result = {}
    for name, port in ports.items():
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM if name == 'db' else socket.SOCK_DGRAM) as check:
            if name == 'db':
                check.settimeout(.2)
                result[name] = check.connect_ex(('127.0.0.1', port)) != 0
            else:
                try:
                    check.bind(('127.0.0.1', port))
                    result[name] = True
                except OSError:
                    result[name] = False
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('app', 'probe', 'driver', 'moos-root', 'plugin-directory', 'output-parent'):
        parser.add_argument('--' + name, type=lambda value: Path(value).resolve(), required=True)
    parser.add_argument('--camera-frames', type=lambda value: Path(value).resolve())
    args = parser.parse_args()
    args.output_parent.mkdir(parents=True, exist_ok=True)
    out = Path(tempfile.mkdtemp(prefix='payload-', dir=args.output_parent))
    report = {'result': 'FAIL', 'mode': 'RX_ONLY', 'hardware': False, 'stages': {}, 'phase_evidence': {}}
    try:
        exercise(args, out, report)
        report['result'] = 'PASS'
    except BaseException as error:
        report['error'] = str(error) or type(error).__name__
    finally:
        report['ports_clear'] = cleared(report.get('ports', {}))
        if (not report['ports_clear'] or not all(report['ports_clear'].values()) or
                report.get('cleanup_errors') != [] or report.get('owned_processes_remaining') != [] or
                report.get('egress_datagrams') != 0):
            report['result'] = 'FAIL'
        report['app_sha256'] = hashlib.sha256(args.app.read_bytes()).hexdigest()
        report['files'] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                           for p in out.iterdir() if p.is_file() and not p.is_symlink()}
        (out / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'result': report['result'], 'error': report.get('error'), 'output': str(out)}))
    return int(report['result'] != 'PASS')


if __name__ == '__main__':
    raise SystemExit(main())
