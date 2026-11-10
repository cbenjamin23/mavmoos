#!/usr/bin/env python3
"""Build and qualify a fresh MAVMOOS revision with the pinned PX4 Ackermann rover."""
import argparse
from hashlib import sha256
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys

from run_ackermann import EXTRA_PINS, HOLD, MISSION, OFFBOARD, PX4_REVISION
from run_ackermann_app import ARTIFACT_FILES, BUILD_TARGETS, MODEL, PROFILE, STOCK_FILES, AckermannWorkflow, run_candidate
from qualify_fixed_wing import (DEPENDENCIES, FIRMWARE, MAVLINK_SHA, audit_container, build_worker, container, mount)
from run_px4_offboard import STABLE_PIN, fields
from audit_aircraft_wire import WIRE, normalized, wire_rows
from profiles import (CIInterrupted, LinuxDomainLease, LocalDocker, MOOS_REVISION, ROOT, TOOLS_IMAGE, clean_revision,
                      dump, git, interrupted, native_files, need, parse, sha, volume)

FIXTURE_INPUTS = (
    'tests/px4_rover/qualify_ackermann.py', 'tests/px4_rover/run_ackermann_app.py',
    'tests/px4_rover/run_ackermann.py', 'tests/px4_rover/meta_ackermann.moos',
    'tests/px4_rover/meta_ackermann.bhv', 'tests/px4_offboard/Px4MoosProbe.cpp',
    'tests/px4_offboard/run_px4_offboard.py', 'tests/px4_offboard/qualify_fixed_wing.py',
    'tests/px4_offboard/run_fixed_wing.py', 'tests/px4_offboard/firmware-v1.17.0.json',
    'tests/owned_processes.py', 'scripts/ci/isolation.py', 'scripts/ci/run.py',
    'scripts/release/local_docker.py', 'scripts/release/profiles.py',
)
RUNTIME_FIXTURE_INPUTS = frozenset((
    'tests/px4_rover/qualify_ackermann.py',
    'tests/px4_rover/run_ackermann_app.py', 'tests/px4_rover/run_ackermann.py',
    'tests/px4_rover/meta_ackermann.moos', 'tests/px4_rover/meta_ackermann.bhv',
))
BUILD_CONTAINER_PROOFS = tuple(f'build-container-{name}.json' for name in ('before', 'after', 'cleanup'))
CASES = ('full', 'restart', 'abort', 'gps-loss', 'autonomy', 'stress')
EXTRA_FIXTURES = {
    'autonomy': 'tests/px4_rover/meta_ackermann_autonomy.bhv',
    'stress': 'tests/px4_offboard/audit_aircraft_wire.py',
}


def owned_container(endpoint, out, stage, mounts, options, timeout):
    container(endpoint, out, stage, mounts, options, timeout,
              entry='/source/tests/px4_rover/qualify_ackermann.py',
              scope_prefix='px4-ackermann', name_prefix='mavmoos-px4-rover')


def verify_build_revision(build_revision, fixture_revision):
    need(re.fullmatch(r'[0-9a-f]{40}', build_revision) and re.fullmatch(r'[0-9a-f]{40}', fixture_revision),
         'missing exact build or fixture revision')
    if build_revision != fixture_revision:
        diff = subprocess.check_output(['git', '-C', ROOT, 'diff', '--name-only', '--no-renames', '-z',
                                        build_revision, fixture_revision, '--'], timeout=15)
        changed = {os.fsdecode(name) for name in diff.split(b'\0') if name}
        need(changed <= RUNTIME_FIXTURE_INPUTS, 'sealed build reuse requires unchanged build inputs')


def audit_build_container(out, revision):
    audit_container(out, 'build')
    before = parse((out / BUILD_CONTAINER_PROOFS[0]).read_text())
    need(before['Config']['Cmd'] == ['-B', '/source/tests/px4_rover/qualify_ackermann.py',
                                     '--worker', 'build', '--source-revision', revision],
         'sealed build container revision differs')
    build = parse((out / 'build-manifest.json').read_text())
    need(build['source_revision'] == revision and build['configure'] == [
        'cmake', '-S', '/source', '-B', '/candidate', '-G', 'Ninja', '-DCMAKE_BUILD_TYPE=Debug',
        '-DBUILD_TESTING=ON', '-DMOOS_IVP_ROOT=/work/moos-ivp',
        '-DMAVMOOS_MAVLINK_ARCHIVE=/input/mavlink.tar.gz',
    ] and build['build'] == ['cmake', '--build', '/candidate', '--parallel', '2', '--target', *BUILD_TARGETS],
        'sealed build commands differ')


def verify_stop(proof, intended):
    mode = proof['mode']
    need(mode['mode'] == HOLD and mode['intended_mode'] == intended and
         proof['firmware_dwell_s'] >= 1.5 and len(proof['samples']) >= 5 and
         0 <= proof['max_truth_speed_m_s'] <= .05, 'independent parked dwell evidence differs')

def compound_proof(directory, receipt):
    route = receipt['compound']
    before, detour, final = route['before_mission'], route['detour'], route['terminal_stop']
    pause, restart = receipt['pause_resume'], receipt['restart']
    layouts = {key: value for key, value in WIRE.items() if value[0] != 'VFR_HUD'}
    layouts[42] = ('MISSION_CURRENT', 28, '<HHBBI', ('sequence', 'total', 'state', 'mission_id'))
    keys = {entry[0]: entry[3] for entry in layouts.values()}
    with (directory / 'moos-wire.jsonl').open() as stream:
        rows = [json.loads(line) for line in stream if line.startswith('{')]
    observed = [(row['message'], tuple(row[key] for key in keys[row['message']]))
                for row in rows if row.get('message') in keys]
    wire = list(wire_rows((directory / 'gcs.mavlink').read_bytes(), layouts))
    need(observed and observed == wire, 'independent rover observations differ from captured MAVLink frames')
    records = {normalized(row) for row in rows}
    need(all(normalized(row) in records for row in (
         route['first']['start'], route['first_end'], pause['before'], pause['after'], pause['cursor'],
         pause['same_cursor'], pause['denied'], restart['continued'], restart['continued_mode'],
         restart['blocked_result'], restart['released'], restart['wrong_session_denied'], detour['start_fix'],
         detour['origin'], detour['allstop'], detour['prepared']['ready'], detour['index'], detour['end'],
         *detour['hits'], *detour['prepared']['stock'].values())),
         'stock, mission cursor or restart witnesses are absent from capture')

    def check_fence(start, end, counts):
        states = [row for row in rows if row.get('key') == 'MAVMOOS_RUNTIME_STATUS' and
                  row.get('source') == 'iMAVLink' and start <= row['mono'] < end]
        need(len(states) >= 5 and all(fields(row).get('permission') == 'false' and
             all(fields(row).get(key) == value for key, value in counts.items()) for row in states),
             'motion or prestream traffic crossed a retired stock grant')
        return len(states)

    original = check_fence(before['native_hold']['mode']['mono'], restart['crash'], pause['fenced_counts'])
    recovered = check_fence(restart['launched_at'], detour['acquired']['acquired_at'],
                            {'nav_tx_accepted': '0', 'px4_hold_tx_accepted': '0'})
    first_m = AckermannWorkflow.moved(route['first']['start'], route['first_end'])
    resume_m = AckermannWorkflow.moved(pause['before'], pause['after'])
    crash_m = AckermannWorkflow.moved(pause['after'], restart['continued'])
    detour_m = AckermannWorkflow.moved(detour['acquired']['start'], detour['end'])
    need(first_m >= 3 and resume_m >= 1 and crash_m >= 1 and detour_m >= 14 and
         pause['cursor']['mission_id'] == pause['same_cursor']['mission_id'] ==
         int(receipt['verified_upload']['proof']['remote_id']) and
         pause['cursor']['sequence'] == pause['same_cursor']['sequence'] and
         fields(pause['denied'])['reason'] == fields(restart['blocked_result'])['reason'] == 'acquisition_rejected' and
         restart['continued_mode']['mode'] == MISSION and
         before['native_hold']['mode']['mono'] < restart['mission']['mono'] < pause['denied']['mono'] <
         pause['same_cursor']['mono'] <= restart['crash'] < restart['continued']['mono'] <
         restart['launched_at'] <= restart['blocked_result']['mono'] < restart['released']['mono'] <
         detour['allstop']['mono'] <= detour['updated_at'] < detour['acquired']['acquired_at'] < detour['index']['mono'] <
         final['released_at'] < receipt['disarmed']['mono'], 'compound authority handoff order or travel differs')
    lat, lon = (detour['origin'][key] * 1e-7 for key in ('latitude_e7', 'longitude_e7'))
    need(detour['index']['value'] == 2 and detour['index']['source'] == 'pHelmIvP' and
         detour['allstop']['source'] == 'pHelmIvP' and detour['allstop']['value'] in ('NothingToDo', 'NoGoalBehavior') and
         fields(detour['prepared']['ready']).get('acquire_preparation') == 'READY' and
         all(row['source'] == 'pHelmIvP' for row in detour['prepared']['stock'].values()) and
         len(detour['hits']) == 2 and len(detour['points']) == 3 and
         detour['acquired']['acquired_at'] <= detour['hits'][0]['mono'] < detour['hits'][1]['mono'] <=
         detour['index']['mono'] + .25 and
         all(math.dist(((row['longitude_e7'] * 1e-7 - lon) * 111195 * math.cos(math.radians(lat)),
                        (row['latitude_e7'] * 1e-7 - lat) * 111195), goal) <= 3
             for row, goal in zip(detour['hits'], detour['points'])),
         'updated stock route lacks two independently observed waypoint gates')
    update = [row for row in rows if row.get('kind') == 'request' and row['key'] == 'STRESS_ROUTE_UPDATE']
    need(len(update) == 1 and abs(update[0]['mono'] - detour['updated_at']) <= .2 and
         update[0]['value'] == detour['update'] == 'points=' +
         ':'.join(f'{x:.3f},{y:.3f}' for x, y in detour['points']), 'route update did not match the captured request')
    for stop in (before, final):
        verify_stop(stop['fallback'], OFFBOARD)
        verify_stop(stop['native_hold'], HOLD)
    verify_stop(pause['stopped'], HOLD)
    return {'stock_legs_m': [first_m, detour_m], 'resume_motion_m': resume_m,
            'after_restart_motion_m': crash_m, 'revised_waypoint_gates': len(detour['hits']),
            'fenced_runtime_samples': [original, recovered], 'crc_checked_frames': len(wire)}


def native_manifest(out, revision, build_revision, case):
    build = parse((out / 'build-manifest.json').read_text())
    receipt = parse((out / 'native/result.json').read_text())
    pin = parse(STABLE_PIN.read_text())
    artifacts = build['artifacts']
    need(build['source_revision'] == receipt.get('build_source_revision', revision) == build_revision and
         receipt['source_revision'] == revision and
         build['tools_image'] == TOOLS_IMAGE and build['moos_revision'] == MOOS_REVISION and
         build['mavlink_sha256'] == MAVLINK_SHA and set(artifacts) == set(ARTIFACT_FILES) and
         receipt['result'] == 'PASS' and receipt['firmware_revision'] == PX4_REVISION and
         receipt['profile'] == PROFILE and receipt['model'] == MODEL and receipt['case'] == case and
         receipt['artifact_drift'] == receipt['owned_processes_remaining'] == [] and receipt['resume_error'] is None,
         'source, artifacts, profile or native result differ')
    need(set(receipt['ports_clear']) == {'14540', '14550', '14580', '18570', '14280', '13030', '19450', 'db'} and
         all(cleared is True for cleared in receipt['ports_clear'].values()), 'owned endpoints were not released')
    expected = {f'/candidate/{name}': digest for name, digest in artifacts.items()}
    expected.update({f'/opt/px4/{name}': digest for name, digest in {**pin['files'], **EXTRA_PINS}.items()})
    expected.update({f'/work/{name}': pin['retained_work_files'][name] for name in STOCK_FILES})
    need(all(receipt['artifacts_before'].get(name) == digest for name, digest in expected.items()),
         'native protected input readback differs')
    for proof in (build['namespace'], receipt['namespace']):
        need(proof['cap_eff'] == '0000000000000000' and proof['ipv4_routes'] == [] and
             proof['no_new_privileges'] is proof['root_read_only'] is True and
             [name for name, flags in proof['interface_flags'].items() if flags & 1] == ['lo'],
             'namespace evidence differs')
    native_files(out / 'native', receipt['files'],
                 ('gcs.mavlink', 'moos-wire.jsonl', 'startup.log', 'fixture-cli.log', 'targ_px4.moos', 'targ_px4.bhv'))
    need(receipt['disarmed']['mode'] == HOLD and receipt['disarmed']['armed'] is False,
         'independent final disarm proof differs')
    verify_stop(receipt['armed_hold'], HOLD)
    summary = {'firmware_revision': PX4_REVISION, 'model': MODEL, 'profile': PROFILE, 'case': case,
               'app_sha256': artifacts['bin/iMAVLink'], 'helm_sha256': build['helm_sha256'],
               'mission_remote_id': receipt['verified_upload']['proof']['remote_id'],
               'native_result_sha256': sha(out / 'native/result.json')}
    if case in ('restart', 'stress'):
        restart = receipt['restart']
        verify_stop(restart['stopped_hold'], HOLD)
        need(restart['old_session']['session'] != restart['new_session']['session'] and
             restart['new_session']['effects_blocked'] == 'true' and
             'status=REJECTED' in restart['wrong_session_denied']['value'].split(',') and
             'permission=false' in restart['released']['value'].split(','),
             'independent retained mission and handback fence proof differs')
        result = {**summary, 'restart_retained_mission': True, 'handback_without_grant': True}
        if case == 'stress':
            result['compound'] = compound_proof(out / 'native', receipt)
        return result
    if case == 'abort':
        abort = receipt['abort']
        verify_stop(abort['stopped_hold'], HOLD)
        release, denied = fields(abort['lease_retired']), fields(abort['resume_denied'])
        need(abort['motion_before_abort_m'] >= 3 and abort['active']['mode'] == MISSION and
             abort['active']['intended_mode'] == MISSION and
             fields(abort['result']).get('status') == 'COMPLETED' and
             fields(abort['mission_cancelled']).get('status') == 'CANCELLED' and
             all(release.get(key) == value for key, value in (
                 ('mission_execution_held', 'false'), ('permission', 'false'),
                 ('nav_tx_accepted', '0'), ('px4_hold_tx_accepted', '0'))) and
             denied.get('status') == 'REJECTED' and denied.get('reason') == 'target_not_retained',
             'stop-only native abort, released lease or resume fence differs')
        return {**summary, 'mission_motion_before_abort_m': abort['motion_before_abort_m'],
                'aborted_stopped_hold': True, 'aborted_resume_denied': True}
    mission, helm = receipt['mission'], receipt['helm']
    need(mission['motion_before_pause_m'] >= 3 and mission['motion_after_resume_m'] >= 8 and
         mission['completion']['reached']['sequence'] == 1 and helm['straight_motion_m'] >= 2 and
         14 <= helm['turn']['heading_change_deg'] <= 65 and
         helm['first_acquire']['prestream_tx'] >= 8 and helm['fresh_acquire']['prestream_tx'] >= 8,
         'independent Mission, steering or reacquisition proof differs')
    for stopped, intended in ((mission['hold_dwell'], HOLD), (helm['fallback']['stopped'], OFFBOARD),
                              (helm['release_stopped'], OFFBOARD), (receipt['pre_disarm_handback'], HOLD)):
        verify_stop(stopped, intended)
    if case == 'autonomy':
        route = receipt['autonomy_route']
        need(route['corner']['source'] == route['exit_gate']['source'] == 'pHelmIvP' and
             route['corner']['value'] == 'turn' and route['exit_gate']['value'] == 2 and
             route['corner_to_exit_m'] >= 13, 'stock Ackermann corner and exit evidence differs')
        summary.update(autonomy_route=route)
    if case == 'gps-loss':
        fallback = helm['fallback']
        degraded = fallback['degraded_unconfirmed_stop']
        verify_stop(degraded, OFFBOARD)
        need(all(fields(row['status'])['stop_confirmed'] == 'false' and
                 row['independent_gps']['message'] == 'GPS_RAW_INT' for row in degraded['samples']) and
             fields(fallback['denied_while_degraded']['runtime'])['permission'] == 'false' and
             fields(fallback['denied_while_degraded']['helm'])['reason'] == 'acquisition_rejected' and
             fields(fallback['denied_while_degraded']['rover'])['hold_ready'] == 'false' and
             all(fields(row['status'])['stop_confirmed'] == 'true' for row in fallback['stopped']['samples']),
             'physical stop, degraded bridge observation, or restored confirmation differs')
        summary.update(degraded_stop_dwell_s=degraded['firmware_dwell_s'],
                       degraded_stop_max_speed_m_s=degraded['max_truth_speed_m_s'])
    return {**summary, 'mission_motion_m': [mission['motion_before_pause_m'], mission['motion_after_resume_m']],
            'helm_motion_m': helm['straight_motion_m'], 'helm_turn_deg': helm['turn']['heading_change_deg'],
            'fallback_truth_speed_m_s': helm['fallback']['stopped']['max_truth_speed_m_s']}


def audit(out):
    result = parse((out / 'qualification.json').read_text())
    revision = result['source_revision']
    need(result['result'] == 'PASS' and result['scope'] == 'owned_sitl' and
         result['hardware'] is False and re.fullmatch(r'[0-9a-f]{40}', revision) and
         result['case'] in CASES and set(result['fixture_sources']) ==
         (set(FIXTURE_INPUTS) | ({EXTRA_FIXTURES[result['case']]} if result['case'] in EXTRA_FIXTURES else set())),
         'archived qualification identity differs')
    for name, expected in result['fixture_sources'].items():
        blob = subprocess.check_output(['git', '-C', ROOT, 'show', f'{revision}:{name}'], timeout=15)
        need(sha256(blob).hexdigest() == expected, f'archived fixture source differs: {name}')
    need(git(ROOT, 'rev-parse', f'{revision}^{{tree}}') == result['source_tree'], 'archived source tree differs')
    audit_container(out, 'native')
    build_revision = result.get('build_source_revision', revision)
    verify_build_revision(build_revision, revision)
    if result['built_in_this_run'] or 'build_source_revision' in result:
        audit_build_container(out, build_revision)
    if 'build_source_revision' in result:
        native = parse((out / 'native-container-before.json').read_text())
        need(native['Config']['Cmd'] == ['-B', '/source/tests/px4_rover/qualify_ackermann.py',
                                        '--worker', 'native', '--source-revision', revision,
                                        '--build-source-revision', build_revision, '--case', result['case']],
             'native container fixture or build revision differs')
    lease = parse((out / 'domain-lease.json').read_text())
    need(lease['source_ref'] == revision and lease['acquired'] is lease['released'] is True and
         lease['release_errors'] == [], 'archived Linux-domain lease proof differs')
    proof = native_manifest(out, revision, build_revision, result['case'])
    need(proof == result['proof'], 'archived proof readback differs')
    return proof


def qualify(args):
    out = args.output.resolve()
    out.mkdir(mode=0o700, parents=True)
    report = {'schema': 1, 'result': 'FAIL', 'scope': 'owned_sitl', 'hardware': False,
              'tools_image': TOOLS_IMAGE, 'built_in_this_run': args.built is None, 'case': args.case}
    endpoint = None
    try:
        revision = clean_revision(ROOT)
        sources = (*FIXTURE_INPUTS, EXTRA_FIXTURES[args.case]) if args.case in EXTRA_FIXTURES else FIXTURE_INPUTS
        report.update(source_revision=revision, source_tree=git(ROOT, 'rev-parse', 'HEAD^{tree}'),
                      fixture_sources={name: sha(ROOT / name) for name in sources})
        endpoint = LocalDocker()
        report['docker_endpoint'] = endpoint.evidence()
        image, = parse(endpoint.capture('image', 'inspect', TOOLS_IMAGE))
        need((image['Id'], image['Os'], image['Architecture']) == (TOOLS_IMAGE, 'linux', 'arm64'),
             'pinned local Linux ARM64 tools image differs')
        dependencies = volume(endpoint.capture, args.stock_volume)
        mounts = [mount('bind', ROOT, '/source'), mount('volume', dependencies, '/work'),
                  mount('bind', out, '/evidence', True)]
        build = out / 'build'
        build_revision = revision
        if args.built:
            previous = args.built.resolve(strict=True)
            build = previous / 'build'
            manifest = parse((previous / 'build-manifest.json').read_text())
            build_revision = manifest['source_revision']
            verify_build_revision(build_revision, revision)
            need(manifest['tools_image'] == TOOLS_IMAGE and
                 set(manifest['artifacts']) == set(ARTIFACT_FILES) and
                 all(sha(build / name) == digest for name, digest in manifest['artifacts'].items()),
                 'previous sealed build artifacts differ')
            audit_build_container(previous, build_revision)
            dump(out / 'build-manifest.json', manifest)
            for name in BUILD_CONTAINER_PROOFS:
                (out / name).write_bytes((previous / name).read_bytes())
        else:
            archive = args.mavlink_archive.resolve(strict=True)
            need(sha(archive) == MAVLINK_SHA, 'local MAVLink archive differs from the pin')
            build.mkdir(mode=0o700)
            owned_container(endpoint, out, 'build', [
                *mounts, mount('bind', build, '/candidate', True),
                mount('bind', archive, '/input/mavlink.tar.gz')],
                ['--worker', 'build', '--source-revision', revision], 450)
        report['build_source_revision'] = build_revision
        need(clean_revision(ROOT) == revision, 'source changed during the offline build')
        if args.build_only:
            report.update(result='PASS', scope='offline_linux_build')
        else:
            firmware = volume(endpoint.capture, args.px4_volume)
            need(firmware != dependencies, 'firmware and stock dependency volumes alias')
            lease = LinuxDomainLease(out / 'domain-lease.json', revision)
            lease.acquire()
            try:
                owned_container(endpoint, out, 'native', [
                    *mounts, mount('bind', build, '/candidate'), mount('volume', firmware, '/opt/px4')],
                    ['--worker', 'native', '--source-revision', revision, '--build-source-revision',
                     build_revision, '--case', args.case], 245)
            finally:
                lease.release()
            need(clean_revision(ROOT) == revision, 'source changed during the owned Ackermann test')
            report.update(result='PASS', proof=native_manifest(out, revision, build_revision, args.case))
    except (CIInterrupted, KeyboardInterrupt, OSError, ValueError, RuntimeError, LookupError, TypeError,
            subprocess.SubprocessError) as error:
        report['error'] = f'{type(error).__name__}: {error}'
    finally:
        if endpoint is not None:
            endpoint.close()
        dump(out / 'qualification.json', report)
    print(f"{report['result']} PX4 Ackermann: {out / 'qualification.json'}", flush=True)
    return report['result'] != 'PASS'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument('--output', type=Path)
    modes.add_argument('--audit', type=Path, help='verify archived results without starting the simulator')
    modes.add_argument('--worker', choices=('build', 'native'), help=argparse.SUPPRESS)
    parser.add_argument('--source-revision', help=argparse.SUPPRESS)
    parser.add_argument('--build-source-revision', help=argparse.SUPPRESS)
    parser.add_argument('--case', choices=CASES, default='full')
    parser.add_argument('--build-only', action='store_true')
    parser.add_argument('--built', type=Path, help='sealed build from an invocation with identical build inputs')
    parser.add_argument('--mavlink-archive', type=Path, help='hash-pinned local MAVLink archive')
    parser.add_argument('--stock-volume', default=DEPENDENCIES)
    parser.add_argument('--px4-volume', default=FIRMWARE)
    args = parser.parse_args()
    if args.worker:
        if not re.fullmatch(r'[a-f0-9]{40}', args.source_revision or ''):
            parser.error('worker requires an exact source revision')
        if args.worker == 'build':
            return build_worker(args.source_revision, targets=BUILD_TARGETS, artifact_files=ARTIFACT_FILES)
        if not re.fullmatch(r'[0-9a-f]{40}', args.build_source_revision or ''):
            parser.error('native worker requires an exact build revision')
        return run_candidate(args.source_revision, args.build_source_revision, Path('/evidence/native'),
                             Path('/evidence/build-manifest.json'), args.case)
    if args.audit:
        print(json.dumps(audit(args.audit.resolve()), indent=2))
        return 0
    if bool(args.mavlink_archive) == bool(args.built) or (args.build_only and args.built):
        parser.error('select an offline MAVLink archive for a fresh build or an existing sealed build')
    signal.signal(signal.SIGTERM, interrupted)
    os.umask(0o077)
    return qualify(args)


if __name__ == '__main__':
    raise SystemExit(main())
