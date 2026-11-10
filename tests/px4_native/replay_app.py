#!/usr/bin/env python3
"""Replay exact native PX4 onboard datagrams through the real RX_ONLY MOOS app."""

import argparse
from contextlib import ExitStack
import hashlib
import json
import os
from pathlib import Path
import re
import socket
import subprocess
import sys
import time

from audit import audit_capture, dump, need, sha

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tests'))
from imu.audit_native import fields
from imu.run_native import live_app
from observer_matrix.native_audit import app_value, wire_value
from observer_matrix.run_loopback import audit, complete_capture, mission_text, reserve, start, take_egress
import owned_processes

PLUGINS = ('telemetry', 'odometry')
MOOS_PIN = '023fb54e00692611ecee2cd6f1138d10eec2127e'


def clean_revision(path):
    args = ['git', '-C', str(path)]
    need(not subprocess.check_output([*args, 'status', '--porcelain'], timeout=10).strip(),
         f'source tree has changes: {path}')
    return subprocess.check_output([*args, 'rev-parse', 'HEAD'], text=True, timeout=10).strip()


def odometry_value(payload):
    return (
        str(payload['time_usec_raw']), payload['frame_id'], payload['child_frame_id'],
        tuple(payload['position_m']), tuple(payload['velocity_m_s']), tuple(payload['quaternion_wxyz']),
        tuple(payload['angular_velocity_rad_s']), tuple(payload['pose_covariance']),
        tuple(payload['velocity_covariance']), payload['reset_counter'], payload['estimator_type'], payload['quality'],
    )


def native_odometry(frame):
    return odometry_value({
        'time_usec_raw': frame.time_usec, 'frame_id': frame.frame_id, 'child_frame_id': frame.child_frame_id,
        'position_m': [frame.x, frame.y, frame.z], 'velocity_m_s': [frame.vx, frame.vy, frame.vz],
        'quaternion_wxyz': frame.q, 'angular_velocity_rad_s': [frame.rollspeed, frame.pitchspeed, frame.yawspeed],
        'pose_covariance': frame.pose_covariance, 'velocity_covariance': frame.velocity_covariance,
        'reset_counter': frame.reset_counter, 'estimator_type': frame.estimator_type, 'quality': frame.quality or None,
    })


def verify_publication(rows, native, value, family):
    observed, fresh = {}, set()
    for record in rows:
        if not record['present']:
            continue
        need((record['system_id'], record['component_id'], record['autopilot']) == (1, 1, 12) and
             record['read_only'] and not record['authoritative_for_control'] and record['source_age'] == 'unknown',
             f'invalid {family} source or observation provenance')
        effective, read, age = (record[key] for key in ('effective_at', 'read_at', 'age_s'))
        need(0 < effective <= read and age >= 0 and age + 1e-6 >= read - effective and
             record['fresh'] == (age <= record['sample_timeout_s']), f'invalid {family} observation age')
        identity = (record['session_epoch'], record['observation_id'])
        decoded = value(record['data'])
        need(decoded in native, f'{family} MOOS fields do not match native wire')
        original = effective, read, decoded
        need(observed.setdefault(identity, original) == original, f'republication changed {family} sample')
        if record['fresh']:
            fresh.add(identity)
    need(len(fresh) >= 3, f'fewer than three advancing fresh {family} publications')
    need(rows[-1]['present'] is False, f'{family} remained present after the heartbeat loss')
    return {'fresh_distinct': len(fresh), 'observed_distinct': len(observed), 'cleared_after_silence': True}


def audit_replay(log, plugin_dir, frames):
    samples, modules = audit(log, PLUGINS, plugin_dir)
    healthy, expired = False, False
    for row in log.read_text().splitlines():
        if row.startswith('CAPTURE\tMAVMOOS_STATUS\t'):
            need(fields(row.split('\t', 2)[-1]).get('helm') == 'disabled', 'helm enabled in RX_ONLY replay')
        if row.startswith('CAPTURE\tMAVMOOS_RUNTIME_STATUS\t'):
            status = fields(row.split('\t', 2)[-1])
            need(status.get('configuration_valid') == 'true' and status.get('control_profile') == 'none' and
                 status.get('armed') == status.get('in_air') == '0', 'non-passive runtime state during replay')
            live = status.get('heartbeat_valid') == '1'
            healthy |= live and status.get('landed_valid') == '1' and status.get('position_valid') == '1'
            expired |= healthy and not live
    need(healthy and expired, 'runtime did not see and expire live PX4 heartbeat/landed/position evidence')
    local = {wire_value('LOCAL_POSITION_NED', frame.to_dict())
             for frame in frames if frame.get_type() == 'LOCAL_POSITION_NED'}
    odometry = {native_odometry(frame) for frame in frames if frame.get_type() == 'ODOMETRY'}
    return {
        'loaded_modules': modules,
        'rx_only_live_and_expired': True,
        'local_kinematics': verify_publication(
            samples['MAVMOOS_LOCAL_KINEMATICS'], local, lambda row: app_value('LOCAL_POSITION_NED', row),
            'LOCAL_POSITION_NED'),
        'odometry': verify_publication(samples['MAVMOOS_ODOMETRY'], odometry, odometry_value, 'ODOMETRY'),
    }


def exercise(out, build, moos, datagrams, report):
    children = []
    try:
        with ExitStack() as stack:
            db = stack.enter_context(reserve(socket.SOCK_STREAM))
            bridge = stack.enter_context(reserve(socket.SOCK_DGRAM))
            app_peer = stack.enter_context(reserve(socket.SOCK_DGRAM))
            app_peer.setblocking(False)
            egress = stack.enter_context((out / 'app-egress.mavlink').open('xb'))
            db_port, bridge_port, peer_port = (item.getsockname()[1] for item in (db, bridge, app_peer))
            report['loopback_ports'] = {'moos': db_port, 'app': bridge_port, 'replay_peer': peer_port}
            report['app_egress_datagrams'] = report['replayed_datagrams'] = 0
            mission = out / 'px4-native-wire-replay.moos'
            text = mission_text(db_port, bridge_port, peer_port, 12, PLUGINS)
            mission.write_text(text.replace('  simulation_only = false',
                                            f'  plugin_directory = {build / "plugins"}\n  simulation_only = false'))
            env = dict(os.environ)
            env['PATH'] = os.pathsep.join((str(build / 'bin'), str(moos / 'bin'), env.get('PATH', '')))
            db.close()
            bridge.close()
            antler = start([str(moos / 'bin/pAntler'), str(mission)], out, env, 'pantler.log')
            children.append(antler)
            probe_log = out / 'probe.log'
            probe = start([str(build / 'bin/observer_matrix_probe'), str(db_port), '23'], out, env, probe_log.name)
            children.append(probe)

            def drain():
                report['app_egress_datagrams'] += take_egress(app_peer, bridge_port, egress)
                need(antler.poll() is None, 'owned replay mission exited early')

            deadline = time.monotonic() + 4
            while b'CAPTURE\tMAVMOOS_RUNTIME_STATUS\t' not in complete_capture(probe_log):
                drain()
                need(time.monotonic() < deadline and probe.poll() is None, 'MOOS replay probe startup timeout')
                time.sleep(.02)
            app, inventory = live_app(owned_processes, antler, mission, bridge_port)
            report['owned_app'] = app
            (out / 'app-udp.txt').write_text(inventory)
            first = datagrams[0].nanoseconds
            started = time.monotonic()
            for datagram in datagrams:
                next_at = started + (datagram.nanoseconds - first) / 1e9
                time.sleep(max(0, next_at - time.monotonic()))
                accepted = app_peer.sendto(datagram.payload, ('127.0.0.1', bridge_port))
                need(accepted == len(datagram.payload), 'partial onboard UDP replay')
                report['replayed_datagrams'] += 1
                drain()
            report['replay_duration_s'] = time.monotonic() - started
            until = time.monotonic() + 10
            while probe.poll() is None:
                drain()
                need(time.monotonic() < until, 'MOOS replay probe did not finish')
                time.sleep(.02)
            drain()
            live_app(owned_processes, antler, mission, bridge_port, app)
            need(probe.returncode == 0 and report['app_egress_datagrams'] == 0,
                 'MOOS replay probe failed or app emitted UDP')
    finally:
        report['remaining_owned_processes'] = [
            row for child in reversed(children) for row in owned_processes.stop_process_tree(child)]
    need(not report['remaining_owned_processes'], 'owned replay processes remain')
    return probe_log


def run(args):
    capture = args.capture.resolve()
    out = args.output.resolve()
    build = args.build.resolve()
    moos = args.moos_root.resolve()
    out.mkdir(parents=True, exist_ok=False, mode=0o700)
    report = {'schema': 1, 'result': 'FAIL',
              'scope': 'exact native PX4 datagram replay into the actual receive-only app on host',
              'hardware': False, 'app_attached_to_live_simulator': False}
    try:
        native, datagrams, frames = audit_capture(capture / 'native')
        saved = json.loads((capture / 'capture-result.json').read_text())
        need(json.loads(json.dumps(native, allow_nan=False)) == saved, 'saved native capture result differs')
        container = json.loads((capture / 'capture-identity.json').read_text())
        need(container['container_removed'] and container['container_policy']['network_mode'] == 'none',
             'native capture identity or cleanup missing')
        report['native_capture'] = {'path': str(capture), 'result_sha256': sha(capture / 'capture-result.json'),
                                    'identity_sha256': sha(capture / 'capture-identity.json')}
        report['replayed_onboard_stream_sha256'] = hashlib.sha256(
            b''.join(item.payload for item in datagrams)).hexdigest()
        cache = (build / 'CMakeCache.txt').read_text()
        source = Path(re.search(r'^CMAKE_HOME_DIRECTORY:INTERNAL=(.+)$', cache, re.M)[1]).resolve()
        need(Path(re.search(r'^MOOS_IVP_ROOT:PATH=(.+)$', cache, re.M)[1]).resolve() == moos,
             'build selected a different MOOS source')
        report['app_source_revision'] = clean_revision(source)
        report['moos_source_revision'] = clean_revision(moos)
        need(report['moos_source_revision'] == MOOS_PIN, 'stock MOOS source revision differs')
        artifacts = [build / 'bin/iMAVLink', build / 'bin/observer_matrix_probe',
                     *(build / 'plugins' / f'mavmoos_{name}.so' for name in PLUGINS),
                     *(moos / 'bin' / name for name in ('MOOSDB', 'pAntler', 'pHelmIvP'))]
        need(all(item.is_file() and not item.is_symlink() for item in artifacts), 'missing regular app artifact')
        hashes = {str(item): sha(item) for item in artifacts}
        report['artifact_sha256'] = hashes
        log = exercise(out, build, moos, datagrams, report)
        report.update(audit_replay(log, build / 'plugins', frames))
        need(hashes == {str(item): sha(item) for item in artifacts}, 'app artifact changed during replay')
        report['result'] = 'PASS'
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
        raise
    finally:
        dump(out / 'app-replay-result.json', report)
    print(f"PASS: {report['replayed_datagrams']} exact datagrams and no app UDP egress in {out}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--moos-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    run(parser.parse_args())
