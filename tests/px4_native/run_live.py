#!/usr/bin/env python3
"""Exercise stock PX4 and the actual MOOS app together in one isolated container."""

import argparse
from collections import Counter
import json
from pathlib import Path
import re
import shutil
import subprocess

from audit import FIRMWARE_REVISION, datagrams, decode_datagram, dump, need, sha
from replay_app import ROOT, fields
from run_capture import CONTAINER_POLICY, FIXTURE, TOOLS_IMAGE, docker, image, mount


def source_receipt(snapshot, revision):
    git = ['git', '-C', str(ROOT)]
    revision = subprocess.check_output([*git, 'rev-parse', '--verify', revision + '^{commit}'], text=True).strip()
    files = sorted(subprocess.check_output(
        [*git, 'ls-tree', '-r', '--name-only', revision, 'src', 'CMakeLists.txt'], text=True).splitlines())
    supplied = sorted(str(path.relative_to(snapshot)) for path in snapshot.rglob('*') if path.is_file())
    need(supplied == files, 'passive source snapshot file set differs from selected commit')
    for name in files:
        expected = subprocess.check_output([*git, 'show', revision + ':' + name])
        need((snapshot / name).read_bytes() == expected, 'passive source file differs from selected commit: ' + name)
    return {'revision': revision, 'snapshot': str(snapshot), 'sha256': {name: sha(snapshot / name) for name in files}}


def verify_routes(evidence):
    checks = {'app': (14540, '0100007F'), 'px4': (14580, '00000000')}
    rows = [line.split() for line in (evidence / 'udp-sockets').read_text().splitlines()[1:]]
    for role, (port, address) in checks.items():
        descriptors = (evidence / f'{role}-descriptors').read_text().splitlines()
        matches = [row for row in rows if row[1] == f'{address}:{port:04X}']
        need(len(matches) == 1 and f'socket:[{matches[0][9]}]' in descriptors, f'unowned live {role} UDP socket')
    maps = (evidence / 'app-module-maps').read_text().splitlines()
    modules = {row.split()[-1] for row in maps}
    need(modules == {'/work/mavmoos-passive-build/plugins/mavmoos_telemetry.so',
                     '/work/mavmoos-passive-build/plugins/mavmoos_odometry.so'}, 'loaded live module paths differ')
    rates = {}
    for block in re.split(r'^instance #\d+:', (evidence / 'px4-startup.log').read_text(), flags=re.M):
        route = re.search(r'transport protocol: UDP \((18570|14580), remote port: (14550|14540)\)', block)
        if not route:
            continue
        peer = int(route[2])
        rx = float(re.search(r'^\s+rx: ([0-9.]+) B/s$', block, re.M)[1])
        tx = float(re.search(r'^\s+tx: ([0-9.]+) B/s$', block, re.M)[1])
        need(rx == 0 and tx > 0, 'live PX4 link had unexpected traffic direction')
        rates[peer] = {'rx_bytes_per_s': rx, 'tx_bytes_per_s': tx}
    need(rates.keys() == {14550, 14540}, 'stock PX4 link statistics missing')
    return {'px4_reported_link_rates': rates, 'loaded_modules': sorted(modules)}


def verify_observations(evidence):
    good, cleared = False, False
    records = {'MAVMOOS_LOCAL_KINEMATICS': {}, 'MAVMOOS_ODOMETRY': {}}
    counts = Counter()
    for line in (evidence / 'probe.log').read_text().splitlines():
        if not line.startswith('CAPTURE\t'):
            continue
        _, key, text = line.split('\t', 2)
        counts[key] += 1
        if key == 'MAVMOOS_RUNTIME_STATUS':
            state = fields(text)
            need(state.get('mode') == 'RX_ONLY' and state.get('control_profile') == 'none' and
                 state.get('permission') == 'false' and state.get('configuration_valid') == 'true' and
                 all(state.get(field) == '0' for field in ('armed', 'in_air', 'tx_attempts', 'tx_accepted',
                                                          'nav_tx_accepted')), 'live runtime was not passive')
            heartbeat = state.get('heartbeat_valid') == '1'
            good |= heartbeat and state.get('position_valid') == state.get('landed_valid') == '1'
            cleared |= good and not heartbeat
        elif key == 'MAVMOOS_STATUS':
            status = fields(text)
            need(status.get('state') == 'RX_ONLY' and status.get('helm') == 'disabled' and
                 status.get('loaded_modules') == '2' and status.get('tx_enabled') == 'false' and
                 status.get('mavlink_tx_attempts') == status.get('mavlink_tx_accepted') == '0',
                 'live app was not passive')
        elif key in records:
            row = json.loads(text)
            if not row['present']:
                continue
            need((row['system_id'], row['component_id'], row['autopilot']) == (1, 1, 12) and
                 row['read_only'] and not row['authoritative_for_control'] and
                 0 < row['effective_at'] <= row['read_at'], 'live observation source or age differs')
            if row['fresh']:
                records[key][(row['session_epoch'], row['observation_id'])] = row['data']
    need(good and cleared and counts['MAVMOOS_STATUS'] and all(len(rows) >= 3 for rows in records.values()),
         'live app heartbeat, cache loss or fresh publications missing')
    return {'moos_records': counts, 'fresh_observations': {key: len(rows) for key, rows in records.items()},
            'rx_only_live_and_expired': True}


def verify_gcs(evidence):
    from pymavlink.dialects.v20 import ardupilotmega as wire
    decoder = wire.MAVLink(None)
    counts = Counter()
    for datagram in datagrams(evidence):
        need(datagram.port == 14550, 'unexpected capture on the live app route')
        for message in decode_datagram(decoder, datagram):
            family = message.get_type()
            counts[family] += 1
            if family == 'HEARTBEAT':
                need((message.type, message.autopilot) == (2, 12) and not message.base_mode & 128,
                     'GCS wire heartbeat armed or wrong identity')
            if family == 'EXTENDED_SYS_STATE':
                need(message.landed_state == 1, 'GCS wire showed an airborne simulation')
    need(all(counts[key] >= 3 for key in ('HEARTBEAT', 'EXTENDED_SYS_STATE', 'LOCAL_POSITION_NED')),
         'live independent GCS wire capture incomplete')
    return {'frames': counts, 'wire_sha256': sha(evidence / 'px4-wire.mavlink'),
            'datagram_index_sha256': sha(evidence / 'px4-datagrams.tsv')}


def run(args):
    need(re.fullmatch(r'[a-f0-9]{64}', args.app_sha256) and
         all(re.fullmatch(r'[a-zA-Z0-9][a-zA-Z0-9_.-]+', name) for name in (args.build_volume, args.px4_volume)),
         'an exact app hash and two local named Docker volumes are required')
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False, mode=0o700)
    fixture, evidence = out / 'fixture', out / 'evidence'
    fixture.mkdir(mode=0o700)
    evidence.mkdir(mode=0o700)
    for file in (FIXTURE / 'run_live.sh', FIXTURE / 'capture_udp.c', ROOT / 'tests/observer_matrix/Probe.cpp'):
        shutil.copyfile(file, fixture / file.name)
    app_source = source_receipt(args.app_source.resolve(), args.app_revision)
    dump(out / 'app-source.json', app_source)
    image(TOOLS_IMAGE, TOOLS_IMAGE)
    command = [
        'create', *CONTAINER_POLICY, '--label=mavmoos.scope=px4-passive-live',
        '--mount', f'type=volume,src={args.build_volume},dst=/work,readonly',
        '--mount', f'type=volume,src={args.px4_volume},dst=/opt/px4,readonly',
        '--mount', mount(fixture, '/fixture', True), '--mount', mount(evidence, '/evidence', False),
        '--env', f'EXPECTED_APP_SHA256={args.app_sha256}',
        '--entrypoint=/bin/sh', TOOLS_IMAGE, '/fixture/run_live.sh',
    ]
    container = docker(*command).stdout.strip()
    need(re.fullmatch(r'[a-f0-9]{64}', container), 'missing exact owned PX4 live container ID')
    try:
        info, = json.loads(docker('container', 'inspect', container).stdout)
        host = info['HostConfig']
        mounts = {item['Destination']: (item['Type'], item['RW']) for item in info['Mounts']}
        need(info['Image'] == TOOLS_IMAGE and host['NetworkMode'] == 'none' and host['ReadonlyRootfs'] and
             host['CapDrop'] == ['ALL'] and 'no-new-privileges' in host['SecurityOpt'] and
             not host['Privileged'] and not host['PortBindings'] and mounts ==
             {'/work': ('volume', False), '/opt/px4': ('volume', False),
              '/fixture': ('bind', False), '/evidence': ('bind', True)}, 'live container policy differs')
        dump(out / 'container-before.json', info)
        live = docker('start', '-a', container, timeout=55, check=False)
        (out / 'container.log').write_text(live.stdout + live.stderr)
        live.check_returncode()
        state = json.loads(docker('container', 'inspect', '--format', '{{json .State}}', container).stdout)
        need(not state['Running'] and state['ExitCode'] == 0, 'live PX4 container did not exit cleanly')
        dump(out / 'container-after.json', state)
    finally:
        docker('container', 'rm', '-f', container)
        need(docker('container', 'inspect', container, check=False).returncode != 0, 'live container remains')
        (out / 'container-removal.txt').write_text(container + '\n')
    artifacts = dict(line.split()[::-1] for line in (evidence / 'artifacts-before.sha256').read_text().splitlines())
    need(artifacts['/work/mavmoos-passive-build/bin/iMAVLink'] == args.app_sha256, 'frozen Linux app differs')
    need(f'PX4 git-hash: {FIRMWARE_REVISION}\n' in (evidence / 'version.log').read_text(), 'live PX4 revision differs')
    report = {'schema': 1, 'result': 'PASS', 'scope': 'direct receive-only PX4 SIH and MOOS in offline Linux',
              'hardware': False, 'container_id': container, 'container_removed': True,
              'app_source_revision': app_source['revision'], 'app_artifacts': artifacts,
              'firmware_source_revision': FIRMWARE_REVISION, 'gcs_capture': verify_gcs(evidence),
              **verify_routes(evidence), **verify_observations(evidence)}
    dump(out / 'live-result.json', report)
    print(f'PASS: live Linux RX_ONLY PX4 SIH in {out}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-volume', required=True)
    parser.add_argument('--px4-volume', required=True)
    parser.add_argument('--app-source', type=Path, required=True, help='exact production source snapshot used to build')
    parser.add_argument('--app-revision', required=True)
    parser.add_argument('--app-sha256', required=True)
    parser.add_argument('--output', type=Path, required=True)
    run(parser.parse_args())
