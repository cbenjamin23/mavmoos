#!/usr/bin/env python3
"""Build and qualify a relocated ARM64 Linux kit from exact local sources."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import tarfile
import tempfile
from uuid import uuid4

from kit import ARCHIVE, MODULE, V1_ARCHIVE, sha, standard_modules, write_json

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts/release'))
from local_docker import (CONTAINER_ID, OWNER_LABEL, LocalDocker, owned_container, owned_volume,
                          remove_owned_container, remove_owned_volume)

IMAGE = 'sha256:644154814d3e5c162392b229f26bd68e3ad55c44acf863e78ee83e7568cbed85'
MOOS_REVISION = '023fb54e00692611ecee2cd6f1138d10eec2127e'
UAV_REVISION = 'e34a62ef49c2be42215a69c351850cbd85881486'
SOURCE_PATHS = ('CMakeLists.txt', 'src', 'tests', 'scripts', 'config', 'examples', 'missions', 'packaging', 'README.md')
STOCK_FILES = ('altitude-CMakeLists.txt', 'build-stock.sh', 'check_sources.py', 'record_elf.py')
BUILDER_EVIDENCE = ('package-manifest.json', 'build-witness-artifacts.json', 'final-source-check.json',
                    'stock-source-check.json', 'linux-stock-artifacts.json', 'packages.tsv',
                    'uname.txt', 'cmake-version.txt', 'compiler-version.txt')
WITNESSES = {
    'pAntler': '/work/moos-ivp/bin/pAntler',
    'MOOSDB': '/work/moos-ivp/bin/MOOSDB',
    'observer_matrix_frames': '/work/mavmoos-build/bin/observer_matrix_frames',
    'observer_matrix_probe': '/work/mavmoos-build/bin/observer_matrix_probe',
}


def need(condition, message):
    if not condition:
        raise ValueError(message)


def snapshot(directory, revision, destination, paths=()):
    with destination.open('xb') as stream:
        subprocess.run(['git', '-C', str(directory), 'archive', '--format=tar', revision, *paths],
                       stdout=stream, check=True, timeout=90)


def git_file(directory, revision, path):
    return subprocess.check_output(['git', '-C', str(directory), 'show', f'{revision}:{path}'], timeout=15)


def prepare(args, inputs, moos_revision, *, v1=False):
    revision = args.source_ref
    snapshot(ROOT, revision, inputs / 'mavmoos.tar', SOURCE_PATHS)
    snapshot(args.moos_source, moos_revision, inputs / 'moos-ivp.tar')
    with tarfile.open(inputs / 'altitude.tar', 'w') as archive:
        for name in ('BHV_ConstantAltitude.cpp', 'BHV_ConstantAltitude.h'):
            data = git_file(args.uav_source, UAV_REVISION, 'src/lib_behaviors/' + name)
            info = tarfile.TarInfo(name)
            info.size = len(data)
            archive.addfile(info, io.BytesIO(data))
    if v1:
        (inputs / 'UAV-BASE-LICENSE').write_bytes(git_file(args.uav_source, UAV_REVISION, 'LICENSE'))
    for name in STOCK_FILES:
        (inputs / name).write_bytes(git_file(ROOT, revision, 'packaging/linux/' + name))
    for name in ('build_stock.sh', 'build_app.sh'):
        (inputs / name).write_bytes(git_file(ROOT, revision, 'packaging/release/' + name))
    for original, name, expected in (
        (args.cpm_file, 'CPM_0.43.1.cmake', '1c40fc102ce9625d7de7eb14f541cab30cc3138dca627f0b0ec40293ce6c2934'),
        (args.proj_archive, 'proj-5.2.0.tar.gz', '11958654f92314ba4fd72fb697d975d2a9c22cd7eb3545798019b099f8285d91'),
        (args.mavlink_archive, 'mavlink.tar.gz', '3fc41a3cd46717b1aa226b958aed0829fbe7b7c9793bc8ffdcca76345a9dfde3'),
    ):
        need(original.is_file() and sha(original) == expected, f'input differs from pin: {name}')
        shutil.copyfile(original, inputs / name)
    for path in inputs.iterdir():
        path.chmod(0o644)
    return {path.name: sha(path) for path in sorted(inputs.iterdir())}


def build(args, *, v1=False):
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    owner = 'mavmoos-release-' + uuid4().hex
    report = {'schema': 1, 'result': 'FAIL', 'source_revision': args.source_ref, 'platform': 'linux-arm64',
              'uav_revision': UAV_REVISION, 'image': args.image,
              'container_removed': False, 'work_volume': args.work_volume, 'volume_retained': False}
    archive = V1_ARCHIVE if v1 else ARCHIVE
    containers = {}
    volume_attempted = False
    endpoint = None

    def docker(*values, timeout=25, log=None, check=True):
        command = endpoint.argv(*values)
        with (out / 'commands.jsonl').open('a') as stream:
            stream.write(json.dumps(command) + '\n')
        if log:
            with (out / log).open('x') as stream:
                result = subprocess.run(command, env=endpoint.env, stdout=stream,
                                        stderr=subprocess.STDOUT, timeout=timeout)
        else:
            result = subprocess.run(command, env=endpoint.env, text=True, capture_output=True, timeout=timeout)
        if check and result.returncode:
            detail = f' (see {log})' if log else ': ' + result.stderr.strip()
            raise ValueError(f'Docker {values[0]} failed{detail}')
        return result

    def capture(*values):
        return docker(*values).stdout.strip()

    def record_ownership():
        write_json(out / 'owned-containers.json',
                   {'owner': owner, 'containers': containers,
                    'volume_attempt': args.work_volume if volume_attempted else None})

    def prove_volume():
        rows = json.loads(capture('volume', 'inspect', args.work_volume))
        need(len(rows) == 1, 'Docker did not return one release work volume')
        owned_volume(rows[0], args.work_volume, owner)
        need(rows[0]['Driver'] == 'local' and not rows[0].get('Options'),
             'release work volume is not a plain local volume')

    def inspect(role, identity, volume=None):
        rows = json.loads(docker('container', 'inspect', identity).stdout)
        need(len(rows) == 1, 'Docker did not return one container')
        row = rows[0]
        config, host = row['Config'], row['HostConfig']
        owned_container(row, owner + '-' + role, owner, identity)
        need(row['Image'] == args.image and host['NetworkMode'] == 'none' and config['User'] == '65534:65534' and
             host['Privileged'] is False and host['CapDrop'] == ['ALL'] and not host.get('CapAdd') and
             'no-new-privileges' in host['SecurityOpt'] and
             all(not host.get(key) for key in ('Devices', 'DeviceRequests', 'Binds', 'VolumesFrom', 'PortBindings')),
             'container image, network or privilege policy differs')
        mounts = row['Mounts']
        need((volume is None and not mounts) or
             (len(mounts) == 1 and mounts[0]['Type'] == 'volume' and mounts[0]['Name'] == volume and
              mounts[0]['Destination'] == '/work' and mounts[0]['RW']), 'unexpected Docker mounts')
        return {'id': identity, 'network': host['NetworkMode'], 'user': config['User'], 'mounts': mounts,
                'image': row['Image'], 'privileged': host['Privileged'], 'cap_drop': host['CapDrop']}

    def launch(role, inputs, volume=None):
        mount = ['--mount', f'type=volume,source={volume},target=/work'] if volume else []
        containers[role] = None
        record_ownership()
        identity = docker('create', '--pull', 'never', '--name', owner + '-' + role, '--label',
                          f'{OWNER_LABEL}={owner}', '--network', 'none', '--cap-drop', 'ALL',
                          '--security-opt', 'no-new-privileges', '--user', '65534:65534',
                          '--cpus', args.jobs, '--memory', '4g', '--memory-swap', '4g',
                          '--pids-limit', '256', '--env', f'JOBS={args.jobs}', *mount,
                          args.image, 'sleep', 'infinity').stdout.strip()
        need(CONTAINER_ID.fullmatch(identity), 'Docker create did not return the full container ID')
        containers[role] = identity
        record_ownership()
        write_json(out / (role + '-container.json'), inspect(role, identity, volume))
        docker('cp', inputs, identity + ':/inputs', timeout=90)
        docker('start', identity)
        docker('exec', '--user', '0', identity, 'chmod', '0777', '/work')
        return identity

    try:
        moos_revision = (json.loads(git_file(ROOT, args.source_ref, 'config/setup.json'))['moos']['revision']
                         if v1 else MOOS_REVISION)
        report['moos_revision'] = moos_revision
        controller = Path(__file__).read_bytes()
        need(controller == git_file(ROOT, args.source_ref, 'packaging/release/build_linux.py'),
             'release controller differs from the selected commit')
        report['controller_sha256'] = hashlib.sha256(controller).hexdigest()
        endpoint = LocalDocker(args.docker)
        report['docker_endpoint'] = endpoint.evidence()
        local = json.loads(docker('image', 'inspect', args.image).stdout)
        need(len(local) == 1 and local[0]['Id'] == args.image and
             local[0]['Os'] == 'linux' and local[0]['Architecture'] == 'arm64', 'require an exact local Linux ARM64 image')
        if args.work_volume:
            volumes = docker('volume', 'ls', '--format', '{{.Name}}').stdout.splitlines()
            need(args.work_volume not in volumes, 'work volume already exists')
            volume_attempted = True
            record_ownership()
            created = capture('volume', 'create', '--driver', 'local', '--label',
                              f'{OWNER_LABEL}={owner}', args.work_volume)
            need(created == args.work_volume, 'unexpected Docker volume identity')
            prove_volume()

        with tempfile.TemporaryDirectory(prefix='.release-inputs-', dir=out) as temporary:
            stage = Path(temporary)
            inputs = stage / 'builder'
            inputs.mkdir(mode=0o755)
            report['input_sha256'] = prepare(args, inputs, moos_revision, v1=v1)
            builder = launch('builder', inputs, args.work_volume)
            shutil.rmtree(inputs)
            stock_env = ('--env', 'MAVMOOS_V1_RUNTIME=1') if v1 else ()
            stock_pins = ('--stock-revisions', moos_revision, UAV_REVISION) if v1 else ()
            docker('exec', *stock_env, builder, 'sh', '/inputs/build_stock.sh', timeout=1200, log='stock-build.log')
            docker('exec', builder, 'sh', '/inputs/build_app.sh', args.source_ref, *stock_pins,
                   timeout=900, log='app-build.log')
            write_json(out / 'builder-postflight.json', inspect('builder', builder, args.work_volume))
            for name in (*BUILDER_EVIDENCE, archive):
                docker('cp', f'{builder}:/work/evidence/{name}', out / name, timeout=90)
            manifest = json.loads((out / 'package-manifest.json').read_text())
            modules = standard_modules(git_file(ROOT, args.source_ref, 'src/plugins/CMakeLists.txt').decode())
            need(manifest['source_revision'] == args.source_ref and manifest['module_count'] == len(modules) and
                 manifest['schema'] == (2 if v1 else 1) and
                 {name for name in manifest['files'] if MODULE.fullmatch(name)} == modules,
                 'unexpected package build manifest')
            artifact_rows = json.loads((out / 'build-witness-artifacts.json').read_text())
            report['build_witness_sha256'] = {row['path']: row['sha256'] for row in artifact_rows}
            report['helm_sha256'] = report['build_witness_sha256']['/work/moos-ivp/bin/pHelmIvP']
            report['app_sha256'] = manifest['files']['bin/iMAVLink']['sha256']
            report['module_count'] = manifest['module_count']
            report['package'] = {'path': archive, 'sha256': sha(out / archive), 'bytes': (out / archive).stat().st_size}
            if v1:
                need(manifest['moos_revision'] == moos_revision and manifest['uav_revision'] == UAV_REVISION and
                     manifest['files']['bin/pHelmIvP']['sha256'] == report['helm_sha256'],
                     'bundle stock provenance differs from the builder')
                (out / (archive + '.sha256')).write_text(f"{report['package']['sha256']}  {archive}\n")
                probe = '/work/mavmoos-build/bin/px4_offboard_moos_probe'
                destination = out / 'px4_offboard_moos_probe'
                docker('cp', f'{builder}:{probe}', destination)
                need(sha(destination) == report['build_witness_sha256'][probe], 'SITL probe differs from the builder')

            fixtures = stage / 'consumer'
            fixtures.mkdir(mode=0o755)
            witness = fixtures / 'witness'
            witness.mkdir(mode=0o755)
            shutil.copyfile(out / archive, fixtures / archive)
            snapshot(ROOT, args.source_ref, fixtures / 'fixtures.tar', ('tests',))
            for name in ('consume.py', 'kit.py'):
                (fixtures / name).write_bytes(git_file(ROOT, args.source_ref, 'packaging/release/' + name))
            for name, source in WITNESSES.items():
                if v1 and name in ('MOOSDB', 'pAntler'):
                    continue
                destination = witness / name
                docker('cp', f'{builder}:{source}', destination)
                need(sha(destination) == report['build_witness_sha256'][source], 'consumer witness artifact differs')
                destination.chmod(0o755)
            consumer = launch('consumer', fixtures)
            completed = docker('exec', consumer, 'python3', '-B', '/inputs/consume.py', args.source_ref,
                               *(['--v1'] if v1 else []),
                               timeout=70, log='consumer.log', check=False)
            (out / 'consumer').mkdir()
            docker('cp', consumer + ':/work/evidence/.', out / 'consumer', check=False)
            need(completed.returncode == 0, 'installed consumer failed (see consumer evidence)')
            result = json.loads((out / 'consumer/consumer-result.json').read_text())
            need(result['result'] == 'PASS' and result['source_revision'] == args.source_ref and
                 result['package_sha256'] == report['package']['sha256'] and result['app_sha256'] == report['app_sha256'] and
                 result['module_count'] == len(modules) and result['rx_only_udp_egress'] == 0 and
                 result['missing_module_app_rejected'] is True, 'relocated consumer receipt differs')
            write_json(out / 'consumer-postflight.json', inspect('consumer', consumer))
            report['consumer_result'] = 'consumer/consumer-result.json'
            report['consumer_result_sha256'] = sha(out / report['consumer_result'])
            report['result'] = 'PASS'
    except (OSError, ValueError, KeyError, subprocess.SubprocessError, KeyboardInterrupt) as error:
        report['error'] = str(error) or type(error).__name__
    finally:
        cleanup_errors = []
        removed = {}
        for role, identity in reversed(containers.items()):
            try:
                removed[role] = remove_owned_container(capture, owner + '-' + role, owner, identity)
            except (OSError, ValueError, KeyError, subprocess.SubprocessError) as error:
                cleanup_errors.append(str(error))
        report['container_removed'] = bool(containers) and not cleanup_errors
        report['removed_container_ids'] = removed
        if cleanup_errors:
            report.update(result='FAIL', cleanup_errors=cleanup_errors)
        if volume_attempted and report['result'] == 'PASS':
            try:
                prove_volume()
                report['volume_retained'] = True
            except (OSError, ValueError, KeyError, subprocess.SubprocessError) as error:
                report.update(result='FAIL', volume_postflight_error=str(error))
        if volume_attempted and report['result'] == 'FAIL' and not cleanup_errors:
            try:
                report['work_volume_removed'] = remove_owned_volume(capture, args.work_volume, owner)
            except (OSError, ValueError, KeyError, subprocess.SubprocessError) as error:
                report['volume_cleanup_error'] = str(error)
        if endpoint is not None:
            endpoint.close()
        write_json(out / 'result.json', report)
    print(json.dumps(report, indent=2, sort_keys=True))
    return int(report['result'] != 'PASS')


def interrupted(signum, _frame):
    raise InterruptedError(f'release build interrupted by signal {signum}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-ref', required=True, help='Full immutable Git commit')
    for option in ('moos-source', 'uav-source', 'cpm-file', 'proj-archive', 'mavlink-archive', 'output'):
        parser.add_argument('--' + option, required=True, type=Path)
    parser.add_argument('--docker', default='docker')
    parser.add_argument('--image', default=IMAGE, help='An existing immutable Linux ARM64 Docker image ID')
    parser.add_argument('--jobs', type=int, choices=(1, 2, 3, 4), default=3)
    parser.add_argument('--v1', action='store_true', help='include the public stock headless MOOS runtime')
    parser.add_argument('--work-volume', help='Retain one newly created named build volume after PASS')
    args = parser.parse_args()
    if not re.fullmatch(r'[0-9a-f]{40}', args.source_ref) or not re.fullmatch(r'sha256:[0-9a-f]{64}', args.image):
        parser.error('source and image must be full immutable IDs')
    if args.work_volume and not re.fullmatch(r'mavmoos-release-[a-zA-Z0-9][a-zA-Z0-9_.-]{1,80}', args.work_volume):
        parser.error('the new work volume must start with mavmoos-release-')
    signal.signal(signal.SIGTERM, interrupted)
    return build(args, v1=args.v1)


if __name__ == '__main__':
    raise SystemExit(main())
