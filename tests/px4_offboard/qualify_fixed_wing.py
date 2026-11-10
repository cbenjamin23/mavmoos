#!/usr/bin/env python3
"""Build and qualify a fresh MAVMOOS revision with pinned PX4 and stock MOOS."""
import argparse
from hashlib import sha256
import json
import os
from pathlib import Path
import platform
import re
import signal
import subprocess
import sys
from uuid import uuid4

from run_fixed_wing import AIRFRAME_PINS, FRESH_BUILD_FILES, MODEL, SITE_ALTITUDE_M, run_candidate
from run_px4_offboard import HOLD, OFFBOARD, STABLE_PIN, namespace

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts/release'))
from profiles import (CIInterrupted, CONTAINER_ID, LinuxDomainLease, LocalDocker, MOOS_REVISION, OWNER_LABEL,
                      TOOLS_IMAGE, accepted, bounded, cache_path, clean_revision, dump, git, interrupted, native_files,
                      need, owned_container, parse, remove_owned_container, sha, volume)

DEPENDENCIES = 'mavmoos-release-017-4eb4e29'
FIRMWARE = 'mavmoos-stable-px4-4b2eac609afc45619159fc755701a8a3-package'
MAVLINK_SHA = '3fc41a3cd46717b1aa226b958aed0829fbe7b7c9793bc8ffdcca76345a9dfde3'
FIXTURE_INPUTS = ('tests/px4_offboard/qualify_fixed_wing.py', 'tests/px4_offboard/run_fixed_wing.py',
                  'tests/px4_offboard/run_px4_offboard.py', 'tests/px4_offboard/Px4MoosProbe.cpp',
                  'tests/px4_offboard/meta_fixed_wing.moos', 'tests/px4_offboard/meta_fixed_wing.bhv',
                  'tests/px4_offboard/meta_vehicle.moos', 'tests/px4_offboard/meta_vehicle.bhv',
                  'tests/px4_offboard/firmware-v1.17.0.json', 'tests/owned_processes.py',
                  'scripts/ci/isolation.py', 'scripts/ci/run.py', 'scripts/release/local_docker.py',
                  'scripts/release/profiles.py')


def mount(kind, source, destination, writable=False):
    return kind, str(source), destination, writable


def mount_arguments(spec):
    kind, source, destination, writable = spec
    need(',' not in source, 'comma in Docker mount source')
    options = [f'type={kind}', f'src={source}', f'dst={destination}']
    if not writable:
        options.append('readonly')
    if kind == 'volume':
        options.append('volume-nocopy')
    return ['--mount', ','.join(options)]


def check_container(info, name, identity, scope, expected_mounts):
    owned_container(info, name, name, identity)
    host = info['HostConfig']
    need(info['Image'] == TOOLS_IMAGE and info['Config']['Labels']['mavmoos.scope'] == scope and
         host['NetworkMode'] == 'none' and host['ReadonlyRootfs'] is True and host['Privileged'] is False and
         host['CapDrop'] == ['ALL'] and 'no-new-privileges' in host['SecurityOpt'] and
         host['PidMode'] in ('', 'private') and host['IpcMode'] == host['CgroupnsMode'] == 'private' and
         host['UTSMode'] in ('', 'private') and all(not host.get(key) for key in
         ('CapAdd', 'Devices', 'DeviceRequests', 'PortBindings', 'VolumesFrom')),
         'candidate container violates the owned Linux isolation policy')
    actual = set()
    for entry in info['Mounts']:
        source = entry.get('Name') or entry['Source']
        if entry['Type'] == 'bind' and platform.system() == 'Darwin' and source.startswith('/host_mnt/'):
            source = source.removeprefix('/host_mnt')
        actual.add(mount(entry['Type'], source, entry['Destination'], entry['RW']))
    need(len(actual) == len(info['Mounts']) and actual == set(expected_mounts),
         'candidate container has unexpected source, dependency, firmware or evidence mounts')


def container(endpoint, out, stage, mounts, options, timeout, *,
              entry='/source/tests/px4_offboard/qualify_fixed_wing.py',
              scope_prefix='px4-fixed-wing', name_prefix='mavmoos-px4-fw'):
    docker = endpoint.capture
    name = name_prefix + '-' + stage + '-' + uuid4().hex
    scope = scope_prefix + '-' + stage
    command = ['create', '--pull=never', '--name', name, f'--label=mavmoos.scope={scope}',
               f'--label={OWNER_LABEL}={name}', '--network=none', '--ipc=private',
               '--cgroupns=private', '--read-only', '--cap-drop=ALL',
               '--security-opt=no-new-privileges', '--pids-limit=128', '--cpus=2',
               '--memory=2048m' if stage == 'build' else '--memory=768m',
               '--tmpfs=/tmp:rw,nosuid,nodev,size=96m']
    for spec in mounts:
        command.extend(mount_arguments(spec))
    command.extend(['--workdir=/evidence', '--entrypoint=/usr/bin/python3', TOOLS_IMAGE, '-B',
                    entry, *options])
    dump(out / f'{stage}-container-command.json', endpoint.argv(*command))
    identity = None
    try:
        created = docker(*command)
        need(CONTAINER_ID.fullmatch(created), 'missing exact candidate container ID')
        identity = created
        before, = parse(docker('container', 'inspect', identity))
        dump(out / f'{stage}-container-before.json', before)
        check_container(before, name, identity, scope, mounts)
        execution = bounded(endpoint.argv('start', '-a', identity), ROOT, out / f'{stage}-launcher.log',
                            timeout, env=endpoint.env, inventory_timeout=10)
        dump(out / f'{stage}-launcher-result.json', execution)
        after, = parse(docker('container', 'inspect', identity))
        dump(out / f'{stage}-container-after.json', after)
        check_container(after, name, identity, scope, mounts)
        need(accepted(execution) and not after['State']['Running'] and after['State']['ExitCode'] == 0,
             f'owned {stage} container did not exit successfully')
    finally:
        removed = remove_owned_container(docker, name, name, identity)
        dump(out / f'{stage}-container-cleanup.json', {'container': removed or identity, 'removed': True,
                                                      'create_reply_received': identity is not None})


def build_worker(revision, *,
                 targets=('iMAVLink', 'mavmoos_telemetry', 'mavmoos_px4_control', 'px4_offboard_moos_probe'),
                 artifact_files=FRESH_BUILD_FILES):
    proof = namespace()
    build, source, moos = Path('/candidate'), Path('/source'), Path('/work/moos-ivp')
    archive = Path('/input/mavlink.tar.gz')
    pin = parse(STABLE_PIN.read_text())
    helm = moos / 'bin/pHelmIvP'
    helm_hash = pin['retained_work_files']['moos-ivp/bin/pHelmIvP']
    need(all(os.statvfs(path).f_flag & os.ST_RDONLY for path in (source, moos, archive)) and
         sha(archive) == MAVLINK_SHA and sha(helm) == helm_hash and not any(build.iterdir()),
         'build requires read-only pinned dependencies and an empty output')
    configure = ['cmake', '-S', str(source), '-B', str(build), '-G', 'Ninja',
                 '-DCMAKE_BUILD_TYPE=Debug', '-DBUILD_TESTING=ON',
                 f'-DMOOS_IVP_ROOT={moos}', f'-DMAVMOOS_MAVLINK_ARCHIVE={archive}']
    compile_command = ['cmake', '--build', str(build), '--parallel', '2', '--target',
                       *targets]
    subprocess.run(configure, check=True, timeout=60)
    subprocess.run(compile_command, check=True, timeout=360)
    need(cache_path(build / 'CMakeCache.txt', 'CMAKE_HOME_DIRECTORY') == source and
         cache_path(build / 'CMakeCache.txt', 'MOOS_IVP_ROOT') == moos and sha(helm) == helm_hash and
         sha(archive) == MAVLINK_SHA, 'candidate build provenance or input bytes changed')
    dump('/evidence/build-manifest.json', {'schema': 1, 'source_revision': revision, 'tools_image': TOOLS_IMAGE,
                                         'platform': 'linux-arm64', 'moos_revision': MOOS_REVISION,
                                         'helm_sha256': helm_hash, 'mavlink_sha256': MAVLINK_SHA,
                                         'namespace': proof, 'configure': configure, 'build': compile_command,
                                         'artifacts': {name: sha(build / name) for name in artifact_files}})
    return 0


def native_manifest(out, revision, vehicle):
    build = parse((out / 'build-manifest.json').read_text())
    receipt = parse((out / 'native/result.json').read_text())
    pin = parse(STABLE_PIN.read_text())
    artifacts = build['artifacts']
    need(build['source_revision'] == receipt['source_revision'] == revision and build['tools_image'] == TOOLS_IMAGE and
         build['moos_revision'] == MOOS_REVISION and build['mavlink_sha256'] == MAVLINK_SHA and
         set(artifacts) == set(FRESH_BUILD_FILES) and receipt['result'] == 'PASS' and
         receipt['artifact_drift'] == [] and receipt['resume_error'] is None and
         receipt['firmware_revision'] == pin['revision'] and receipt['owned_processes_remaining'] == [] and
         set(receipt['ports_clear']) == {'14540', '14550', '14580', '18570', 'db'} and
         all(flag is True for flag in receipt['ports_clear'].values()), 'candidate build, native receipt or cleanup differs')
    inputs = {f'/candidate/{name}': digest for name, digest in artifacts.items()}
    inputs.update({f'/opt/px4/{name}': digest for name, digest in pin['files'].items()})
    inputs.update({f'/work/{name}': digest for name, digest in pin['retained_work_files'].items()
                   if not name.startswith('mavmoos-build/')})
    if vehicle == 'fixed-wing':
        inputs.update({f'/opt/px4/{name}': digest for name, digest in AIRFRAME_PINS.items()})
    need(all(receipt['artifacts_before'].get(name) == digest for name, digest in inputs.items()),
         'native artifact readback differs')
    for isolation in (build['namespace'], receipt['namespace']):
        need(isolation['ipv4_routes'] == [] and isolation['cap_eff'] == '0' * 16 and
             isolation['no_new_privileges'] is isolation['root_read_only'] is True and
             [name for name, flags in isolation['interface_flags'].items() if flags & 1] == ['lo'],
             'candidate effective namespace proof is incomplete')
    need(receipt['landed_disarmed'] is True and receipt['independent_loss_hold']['mode'] == HOLD and
         receipt['independent_release_hold']['mode'] == HOLD and
         receipt['failsafe_current_mode']['intended_mode'] == OFFBOARD and
         receipt['handback_current_mode']['intended_mode'] == HOLD and
         'reason=acquisition_rejected' in receipt['denied_failsafe_acquire']['value'].split(','),
         'candidate loss, denial, reacquisition or landing proof is missing')
    settled = receipt['stable_home_settle']
    need(settled['processes_before'] == settled['processes_after'] and
         settled['samples'][-1]['flight_time_us'] >= pin['home_correction_window_us'] and
         settled['samples'][-1]['mono'] - settled['samples'][0]['mono'] >= 6 and
         len({(row['home_timestamp_us'], row['home_update_count']) for row in settled['samples']}) == 1,
         'stable home-reference proof differs')
    first = receipt if vehicle == 'quadrotor' else receipt['first_acquisition']
    second = receipt['second_acquisition']
    minimum_east = 4 if vehicle == 'quadrotor' else 65
    need(all(leg['travel']['east_m'] >= minimum_east and leg['prestream_count'] >= 10 and
             leg['independent_offboard']['mode'] == OFFBOARD for leg in (first, second)),
         'measured stock-helm flight proof differs')
    proof = {'east_m': [leg['travel']['east_m'] for leg in (first, second)],
             'prestream_count': [leg['prestream_count'] for leg in (first, second)],
             'firmware_revision': receipt['firmware_revision'], 'firmware_sha256': pin['files']['bin/px4'],
             'stock_helm_sha256': receipt['artifacts_before']['/work/moos-ivp/bin/pHelmIvP'],
             'app_sha256': artifacts['bin/iMAVLink'], 'native_result_sha256': sha(out / 'native/result.json'),
             'vehicle': 'sihsim_quadx' if vehicle == 'quadrotor' else MODEL}
    if vehicle == 'fixed-wing':
        site = receipt['simulator_site']
        need(site['configured_msl_m'] == SITE_ALTITUDE_M and
             all(10 <= site[key] <= 20 for key in ('home_before_takeoff_msl_m', 'home_after_settle_msl_m')),
             'fixed-wing SIH site and nonzero HOME altitude proof differ')
        need(receipt['model'] == MODEL and receipt['profile'] == 'px4_fixed_wing' and
             first['independent_offboard']['mav_type'] == second['independent_offboard']['mav_type'] == 1 and
             receipt['normal_handoff']['pause']['offboard']['mode'] == OFFBOARD and
             receipt['normal_handoff']['pause']['stock_allstop']['source'] == 'pHelmIvP',
             'independent fixed-wing model or normal handoff proof differs')
        for leg in (first, second):
            travel = leg['travel']
            need(10 <= travel['airspeed_range_mps'][0] <= travel['airspeed_range_mps'][1] <= 35 and
                 8 <= travel['groundspeed_range_mps'][0] <= travel['groundspeed_range_mps'][1] <= 35 and
                 20 <= travel['altitude_range_m'][0] <= travel['altitude_range_m'][1] <= 130 and
                 min(travel['air_speed_samples'], travel['position_samples'], travel['altitude_samples']) >= 5,
                 'fixed-wing independent motion envelope differs')
        takeover = receipt['native_takeover']
        third = receipt['third_acquisition']
        observed = takeover['observed']
        need(observed['current_mode']['mode'] == observed['current_mode']['intended_mode'] == HOLD and
             observed['heartbeat']['mode'] == HOLD and 'state=LATCHED' in observed['helm']['value'].split(',') and
             'permission=true' in takeover['before']['value'].split(',') and
             'permission=false' in takeover['fence']['value'].split(',') and
             observed['runtime']['mono'] < takeover['fence']['mono'] < third['acquire_at'] and
             third['independent_offboard']['mode'] == OFFBOARD and third['prestream_count'] >= 10 and
             third['first_motion']['runtime']['mono'] < receipt['independent_release_hold']['mono'],
             'fixed-wing native takeover, fenced mail or third acquisition proof differs')
        proof['native_takeover'] = True
        proof['airspeed_range_mps'] = [leg['travel']['airspeed_range_mps'] for leg in (first, second)]
    native_files(out / 'native', receipt['files'],
                 ('gcs.mavlink', 'moos-wire.jsonl', 'preflight-gcs.mavlink', 'startup.log', 'fixture-cli.log'))
    return proof


def audit_container(out, stage):
    before = parse((out / f'{stage}-container-before.json').read_text())
    after = parse((out / f'{stage}-container-after.json').read_text())
    cleanup = parse((out / f'{stage}-container-cleanup.json').read_text())
    need(before['Id'] == after['Id'] == cleanup['container'] and
         before['Image'] == after['Image'] == TOOLS_IMAGE and not after['State']['Running'] and
         after['State']['ExitCode'] == 0 and cleanup['removed'] is True,
         f'{stage} exact container teardown proof differs')


def audit(out):
    report = parse((out / 'qualification.json').read_text())
    revision, vehicle = report['source_revision'], report['vehicle']
    need(report['result'] == 'PASS' and report['scope'] == 'owned_sitl' and report['hardware'] is False and
         re.fullmatch(r'[0-9a-f]{40}', revision) and vehicle in ('fixed-wing', 'quadrotor') and
         set(report['fixture_sources']) == set(FIXTURE_INPUTS), 'invalid archived candidate identity')
    for name, expected in report['fixture_sources'].items():
        blob = subprocess.check_output(['git', '-C', ROOT, 'show', f'{revision}:{name}'], timeout=15)
        need(sha256(blob).hexdigest() == expected, f'archive source differs: {name}')
    need(git(ROOT, 'rev-parse', f'{revision}^{{tree}}') == report['source_tree'],
         'archived source tree differs')
    audit_container(out, 'native')
    if report['built_in_this_run']:
        audit_container(out, 'build')
    lease = parse((out / 'domain-lease.json').read_text())
    need(lease['source_ref'] == revision and lease['acquired'] is lease['released'] is True and
         lease['release_errors'] == [], 'archived Linux-domain teardown proof differs')
    proof = native_manifest(out, revision, vehicle)
    need(proof == report['proof'], 'archived summary differs from the native readback')
    return proof


def qualify(args):
    out = args.output.resolve()
    out.mkdir(mode=0o700, parents=True, exist_ok=False)
    report = {'schema': 1, 'result': 'FAIL', 'scope': 'owned_sitl', 'hardware': False,
              'vehicle': args.vehicle, 'tools_image': TOOLS_IMAGE, 'built_in_this_run': args.built is None}
    endpoint = None
    try:
        revision = clean_revision(ROOT)
        report.update(source_revision=revision, source_tree=git(ROOT, 'rev-parse', 'HEAD^{tree}'),
                      fixture_sources={name: sha(ROOT / name) for name in FIXTURE_INPUTS})
        endpoint = LocalDocker()
        report['docker_endpoint'] = endpoint.evidence()
        image, = parse(endpoint.capture('image', 'inspect', TOOLS_IMAGE))
        need((image['Id'], image['Os'], image['Architecture']) == (TOOLS_IMAGE, 'linux', 'arm64'),
             'pinned offline Linux ARM64 tools image differs')
        dependencies = volume(endpoint.capture, args.stock_volume)
        mounts = [mount('bind', ROOT, '/source'), mount('volume', dependencies, '/work'),
                  mount('bind', out, '/evidence', True)]
        build = out / 'build'
        if args.built:
            built = args.built.resolve(strict=True)
            build = built / 'build'
            build_manifest = parse((built / 'build-manifest.json').read_text())
            need(build_manifest['source_revision'] == revision and build_manifest['tools_image'] == TOOLS_IMAGE and
                 set(build_manifest['artifacts']) == set(FRESH_BUILD_FILES) and
                 all(sha(build / name) == digest for name, digest in build_manifest['artifacts'].items()),
                 'previous candidate build differs from the clean checkout or artifact manifest')
            audit_container(built, 'build')
            dump(out / 'build-manifest.json', build_manifest)
        else:
            archive = args.mavlink_archive.resolve(strict=True)
            need(sha(archive) == MAVLINK_SHA, 'offline MAVLink archive differs from its immutable pin')
            build.mkdir(mode=0o700)
            container(endpoint, out, 'build',
                      [*mounts, mount('bind', build, '/candidate', True),
                       mount('bind', archive, '/input/mavlink.tar.gz')],
                      ['--worker', 'build', '--source-revision', revision], 450)
        need(clean_revision(ROOT) == revision, 'source changed during the offline candidate build')
        if args.build_only:
            report.update(result='PASS', scope='offline_linux_build')
        else:
            firmware = volume(endpoint.capture, args.px4_volume)
            need(firmware != dependencies, 'candidate firmware and dependencies volumes alias')
            lease = LinuxDomainLease(out / 'domain-lease.json', revision)
            lease.acquire()
            try:
                container(endpoint, out, 'native',
                          [*mounts, mount('bind', build, '/candidate'), mount('volume', firmware, '/opt/px4')],
                          ['--worker', args.vehicle, '--source-revision', revision], 480)
            finally:
                lease.release()
            need(clean_revision(ROOT) == revision, 'source changed during owned SITL qualification')
            report.update(result='PASS', proof=native_manifest(out, revision, args.vehicle))
    except (CIInterrupted, KeyboardInterrupt, OSError, ValueError, RuntimeError,
            LookupError, AttributeError, TypeError, subprocess.SubprocessError) as error:
        report['error'] = f'{type(error).__name__}: {error}'
    finally:
        if endpoint is not None:
            endpoint.close()
        dump(out / 'qualification.json', report)
    print(f"{report['result']} PX4 {args.vehicle}: {out / 'qualification.json'}", flush=True)
    return report['result'] != 'PASS'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument('--output', type=Path, help='fresh evidence directory')
    modes.add_argument('--audit', type=Path, help='verify archived results without starting a simulator')
    modes.add_argument('--worker', choices=('build', 'fixed-wing', 'quadrotor'), help=argparse.SUPPRESS)
    parser.add_argument('--vehicle', choices=('fixed-wing', 'quadrotor'), default='fixed-wing')
    parser.add_argument('--build-only', action='store_true', help='compile without reserving the Linux SITL domain')
    parser.add_argument('--built', type=Path, help='sealed build from an earlier invocation at this source revision')
    parser.add_argument('--mavlink-archive', type=Path, help='local immutable MAVLink archive used for a fresh build')
    parser.add_argument('--stock-volume', default=DEPENDENCIES)
    parser.add_argument('--px4-volume', default=FIRMWARE)
    parser.add_argument('--source-revision', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.worker:
        if not re.fullmatch(r'[0-9a-f]{40}', args.source_revision or ''):
            parser.error('worker requires an exact source revision')
        if args.worker == 'build':
            return build_worker(args.source_revision)
        return run_candidate(args.worker, args.source_revision, Path('/evidence/native'),
                             Path('/evidence/build-manifest.json'))
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
