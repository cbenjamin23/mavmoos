#!/usr/bin/env python3
"""Source-only clean Linux reproduction in an owned local Docker container."""
import argparse
import hashlib
import io
import json
import os
import re
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tarfile
import time
from linux_export_ardupilot import export as export_ardupilot, ARDUPILOT_PIN
from ci.isolation import (DOMAIN, PROBE, RESOURCE_ARGS, owned_container,
                          validate_container, validate_namespace)

MOOS_PIN = '023fb54e00692611ecee2cd6f1138d10eec2127e'
UAV_PIN = 'e34a62ef49c2be42215a69c351850cbd85881486'
IMAGE = 'mavmoos-clean-linux:20261103'


class RecipeInterrupted(BaseException):
    def __init__(self, number):
        self.number = number
        super().__init__('source-built CI interrupted by signal ' + str(number))


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def snapshot_file(archive_path, name):
    """Read a regular member of the selected source snapshot, not the worktree."""
    relative = Path(name)
    if relative.is_absolute() or '..' in relative.parts or str(relative) != name:
        raise ValueError('unsafe snapshot path')
    with tarfile.open(archive_path) as archive:
        matches = [member for member in archive if member.name == name]
        if len(matches) != 1 or not matches[0].isfile():
            raise ValueError('missing/nonregular selected snapshot input: ' + name)
        return archive.extractfile(matches[0]).read()


def metadata_archive(source, revision, destination, archive_path):
    """Exact local commit objects for native fixtures that read Git identity."""
    subprocess.run(['git', 'clone', '--bare', '--no-local', '--depth=1',
                    Path(source).resolve().as_uri(), str(destination)], check=True)
    subprocess.run(['git', '--git-dir', str(destination), 'fetch', '--depth=1', 'origin', revision], check=True)
    subprocess.run(['git', '--git-dir', str(destination), 'update-ref', 'HEAD', revision], check=True)
    with tarfile.open(archive_path, 'w') as archive:
        for item in sorted(destination.iterdir()):
            archive.add(item, arcname=item.name)


def disconnect_owned_bridge(docker, owner, run, inspect_container):
    """Fail before app execution if fresh identity/disconnection proof fails."""
    inspect_container('isolation-before-disconnect')
    run([docker, 'network', 'disconnect', 'bridge', owner['container_id']])
    return inspect_container('isolation-before', True, True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--moos-source', required=True)
    parser.add_argument('--uav-source', required=True)
    parser.add_argument('--output', default='build-linux/reproduction')
    parser.add_argument('--docker', default='docker')
    parser.add_argument('--source-ref', help='Immutable MAVMOOS commit; omit to snapshot the current tracked/untracked source worktree')
    parser.add_argument('--keep-container', action='store_true')
    parser.add_argument('--image', help='Existing immutable sha256 image ID; skip rebuilding the tooling image')
    parser.add_argument('--mavlink-archive', help='Optional local immutable MAVLink archive (SHA-256 checked)')
    parser.add_argument('--ardupilot-source', help='Optional local source checkout; build pinned native Linux Copter and run the SITL tests too')
    parser.add_argument('--ci-tier', choices=['software', 'smoke', 'full'], help='Use the explicit local CI runner instead of legacy direct CTest')
    parser.add_argument('--ci-firmware-manifest', help='Source-snapshot-relative firmware manifest for native CI')
    parser.add_argument('--isolated-native-domain', action='store_true',
                        help='Fixed-budget owned Linux domain; disconnect and prove namespace before offline app/native execution')
    args = parser.parse_args()
    if args.image and not re.fullmatch(r'sha256:[0-9a-f]{64}',args.image):
        parser.error('--image must be an immutable sha256 image ID')
    if args.ci_firmware_manifest and not args.ci_tier:
        parser.error('--ci-firmware-manifest requires --ci-tier')
    if args.ci_tier in ('smoke', 'full') and not (args.ardupilot_source and args.source_ref and args.ci_firmware_manifest):
        parser.error('native CI requires --ardupilot-source, --source-ref and --ci-firmware-manifest')
    if args.isolated_native_domain and (args.ci_tier not in ('smoke', 'full') or not args.mavlink_archive or
            not args.source_ref or not re.fullmatch(r'[0-9a-f]{40}', args.source_ref) or not args.image or args.keep_container):
        parser.error('isolated Linux domain requires native tier, full immutable source/image, offline MAVLink archive and removal')
    root = Path(__file__).resolve().parents[1]
    output = Path(args.output).resolve()
    output.mkdir(parents=True,exist_ok=False)
    inputs, evidence = output / 'inputs', output / 'evidence'
    inputs.mkdir()
    evidence.mkdir()
    name = 'mavmoos-linux-' + str(time.time_ns())
    commands = []
    command_results = []

    def run(argv, log=None, check=True):
        commands.append(argv)
        if log:
            with (evidence / log).open('w') as handle:
                result = subprocess.run(argv, stdout=handle, stderr=subprocess.STDOUT)
        else:
            result = subprocess.run(argv, capture_output=True, text=True)
            log='command-'+str(len(commands)).zfill(2)+'.log'
            (evidence/log).write_text(result.stdout+result.stderr)
        command_results.append({'argv':argv,'exit':result.returncode,'log':log})
        if check and result.returncode:
            raise RuntimeError(f'command failed ({result.returncode}); see {log or argv[0]}')
        return result

    for source, pin, target in [(args.moos_source, MOOS_PIN, 'moos-ivp.tar')]:
        with (inputs / target).open('wb') as handle:
            subprocess.run(['git', '-C', source, 'archive', '--format=tar', pin], stdout=handle, check=True)
    with tarfile.open(inputs / 'altitude.tar', 'w') as archive:
        for filename in ['BHV_ConstantAltitude.cpp', 'BHV_ConstantAltitude.h']:
            data = subprocess.check_output(['git', '-C', args.uav_source, 'show', UAV_PIN + ':src/lib_behaviors/' + filename])
            info = tarfile.TarInfo(filename)
            info.size = len(data)
            archive.addfile(info, io.BytesIO(data))
    if args.source_ref:
        with (inputs / 'mavmoos.tar').open('wb') as handle:
            subprocess.run(['git', '-C', str(root), 'archive', '--format=tar', args.source_ref], stdout=handle, check=True)
    else:
        names = subprocess.check_output(['git', '-C', str(root), 'ls-files', '--cached', '--others', '--exclude-standard', '-z']).split(b'\0')
        with tarfile.open(inputs / 'mavmoos.tar', 'w') as archive:
            for raw in sorted(set(names)):
                if not raw:
                    continue
                rel = Path(os.fsdecode(raw))
                path = root / rel
                if path.is_file():
                    archive.add(path, arcname=str(rel), recursive=False)
    with tarfile.open(inputs / 'mavmoos.tar') as archive:
        source_files = [{'path': member.name,
                         'sha256': hashlib.sha256(archive.extractfile(member).read()).hexdigest()}
                        for member in archive if member.isfile()]
    (evidence / 'mavmoos-source.json').write_text(json.dumps({
        'snapshot': args.source_ref or 'current worktree',
        'tar_sha256': sha(inputs / 'mavmoos.tar'), 'files': source_files}, indent=2) + '\n')
    if args.isolated_native_domain:
        for helper in ('scripts/linux_reproduce.py', 'scripts/ci/linux.py', 'scripts/ci/isolation.py',
                       'scripts/ci/run.py', 'tests/owned_processes.py'):
            if (root / helper).read_bytes() != snapshot_file(inputs / 'mavmoos.tar', helper):
                raise RuntimeError('selected host Linux controller differs from frozen source: ' + helper)
    ci_manifest = None
    if args.ci_tier:
        if args.ci_firmware_manifest:
            data = snapshot_file(inputs / 'mavmoos.tar', args.ci_firmware_manifest)
            (inputs / 'firmware-manifest.json').write_bytes(data)
            # Host export code must be the exact selected implementation too.
            for helper in ('manifest.py', 'export_source.py'):
                relative = 'scripts/firmware/' + helper
                if (root / relative).read_bytes() != snapshot_file(inputs / 'mavmoos.tar', relative):
                    raise RuntimeError('selected firmware helper differs from host export code: ' + relative)
            validated = subprocess.check_output([sys.executable, '-B', str(root / 'scripts/firmware/manifest.py'),
                'validate', '--manifest', str(inputs / 'firmware-manifest.json')], text=True)
            ci_manifest = json.loads(validated)['manifest']
            if ci_manifest['dependencies']['moos_revision'] != MOOS_PIN or ci_manifest['dependencies']['uav_revision'] != UAV_PIN:
                raise RuntimeError('selected profile dependencies differ from Linux recipe pins')
            if args.ci_tier == 'full' and ci_manifest['source']['revision'] != ARDUPILOT_PIN:
                raise RuntimeError('full Linux CI currently requires baseline Copter/Rover source; stable candidate uses smoke')
        (inputs / 'ci-config.json').write_text(json.dumps({'schema': 1, 'tier': args.ci_tier,
            'firmware_manifest': args.ci_firmware_manifest,
            'manifest_sha256': sha(inputs / 'firmware-manifest.json') if ci_manifest else None,
            'native_source_revision': ci_manifest['source']['revision'] if ci_manifest else None,
            'execution_domain': DOMAIN if args.isolated_native_domain else 'legacy-local',
            'scope': 'local source-built CI; no hosted activation or physical route'}, indent=2) + '\n')
        if args.source_ref:
            source_revision = subprocess.check_output(['git', '-C', str(root), 'rev-parse', args.source_ref], text=True).strip()
            metadata_archive(root, source_revision, output / 'mavmoos-metadata.git', inputs / 'mavmoos-git.tar')
    if args.ardupilot_source:
        ap_export = output / 'ardupilot-source'
        if ci_manifest:
            report_ap = json.loads(subprocess.check_output([sys.executable, '-B', str(root / 'scripts/firmware/export_source.py'),
                '--manifest', str(inputs / 'firmware-manifest.json'), '--source-root', str(Path(args.ardupilot_source).resolve()),
                '--output', str(ap_export)], text=True))
        else:
            report_ap = export_ardupilot(args.ardupilot_source, ap_export)
        with tarfile.open(inputs / 'ardupilot.tar', 'w') as archive:
            for item in sorted(ap_export.iterdir()):
                archive.add(item, arcname=item.name)
        (evidence / 'ardupilot-source.json').write_text(json.dumps(report_ap, indent=2) + '\n')
        # The native harness records git revisions. Include source-only metadata,
        # separately from the immutable source archive, rather than inventing a revision.
        metadata = output / 'moos-metadata.git'
        subprocess.run(['git', 'clone', '--bare', '--no-local', '--depth=1',
                        Path(args.moos_source).resolve().as_uri(), str(metadata)], check=True)
        subprocess.run(['git', '--git-dir', str(metadata), 'fetch', '--depth=1', 'origin', MOOS_PIN], check=True)
        subprocess.run(['git', '--git-dir', str(metadata), 'update-ref', 'HEAD', MOOS_PIN], check=True)
        with tarfile.open(inputs / 'moos-git.tar', 'w') as archive:
            for item in sorted(metadata.iterdir()):
                archive.add(item, arcname=item.name)
    for filename in ['altitude-CMakeLists.txt', 'build-stock.sh', 'build-app.sh', 'build-sitl.sh', 'record_elf.py', 'check_sources.py']:
        if args.ci_tier:
            (inputs / filename).write_bytes(snapshot_file(inputs / 'mavmoos.tar', 'packaging/linux/' + filename))
        else:
            shutil.copy2(root / 'packaging/linux' / filename, inputs / filename)
    if args.mavlink_archive:
        cached=Path(args.mavlink_archive).resolve()
        if sha(cached)!='3fc41a3cd46717b1aa226b958aed0829fbe7b7c9793bc8ffdcca76345a9dfde3':
            raise RuntimeError('local MAVLink archive differs from immutable dependency pin')
        shutil.copyfile(cached,inputs/'mavlink.tar.gz')
    # Inputs are source archives and build recipes, never Mac build products.
    for filename in ['moos-ivp.tar', 'mavmoos.tar']:
        with tarfile.open(inputs / filename) as archive:
            for member in archive:
                if member.isfile() and member.name.endswith(('.dylib', '.a', '.so')):
                    raise RuntimeError('prebuilt library in source archive: ' + member.name)
    # Docker's copied inputs belong to the configured nonroot container user.
    # Make these owned host-side snapshots readable before copying them;
    # dropped root capabilities intentionally prevent chmod of another UID.
    for path in inputs.iterdir():
        if path.is_file():
            path.chmod(0o444)
    inputs.chmod(0o555)
    report = {'result': 'FAIL', 'layer': 'clean Linux source build + localhost software tests',
              'container': name, 'moos_revision': MOOS_PIN, 'uav_revision': UAV_PIN,
              'mavmoos_ref': args.source_ref or 'worktree snapshot',
              'ardupilot_revision': (ci_manifest['source']['revision'] if ci_manifest else ARDUPILOT_PIN) if args.ardupilot_source else None,
              'inputs': {p.name: {'sha256': sha(p), 'bytes': p.stat().st_size} for p in inputs.iterdir() if p.is_file()}}
    created = False
    owner = None
    if args.ci_tier:
        report['ci_tier'] = args.ci_tier
        report['layer'] = 'clean Linux source build + explicit local CI ' + args.ci_tier
        owner = {'schema': 1, 'name': name, 'label_key': 'mavmoos.ci.owner',
                 'label_value': name, 'container_id': None}
        (evidence / 'owned-container.json').write_text(json.dumps(owner, indent=2) + '\n')
        def interrupted(number, _frame):
            raise RecipeInterrupted(number)
        signal.signal(signal.SIGTERM, interrupted)
    if args.isolated_native_domain:
        report['execution_domain'] = DOMAIN
    previous_namespace = None

    def inspect_container(label, disconnected=False, namespace=False):
        require_owner = owner is not None and owner.get('container_id')
        if not require_owner:
            raise RuntimeError('full owned container ID is absent')
        inspection_file = label + '-inspect.json'
        run([args.docker, 'inspect', owner['container_id']], inspection_file)
        inspection = json.loads((evidence / inspection_file).read_text())
        identity = validate_container(owner, inspection, report['image'], disconnected)
        if not namespace:
            return inspection
        namespace_file = label + '-namespace.json'
        run([args.docker, 'exec', identity, 'python3', '-B', '-c', PROBE], namespace_file)
        observed = json.loads((evidence / namespace_file).read_text())
        proof = validate_namespace(observed, previous_namespace)
        proof.update({'container_id': identity, 'name': name,
                      'inspection_file': inspection_file, 'inspection_sha256': sha(evidence / inspection_file),
                      'namespace_file': namespace_file, 'namespace_sha256': sha(evidence / namespace_file)})
        return proof, observed

    try:
        if not args.image:
            context = root / 'packaging/linux'
            if args.ci_tier:
                context = output / 'tooling-context'
                context.mkdir()
                (context / 'Dockerfile').write_bytes(snapshot_file(inputs / 'mavmoos.tar', 'packaging/linux/Dockerfile'))
            run([args.docker, 'build', '--progress', 'plain', '-t', IMAGE, str(context)], 'image-build.log')
        report['image'] = json.loads(run([args.docker, 'image', 'inspect', args.image or IMAGE]).stdout)[0]['Id']
        create = [args.docker, 'create', '--name', name, '--label', 'mavmoos.scope=clean-linux-20261103',
             '--cap-drop', 'ALL', '--security-opt', 'no-new-privileges', '--user', '65534:65534',
             '--env', 'HOME=/tmp/mavmoos-home']
        if owner:
            create += ['--label', owner['label_key'] + '=' + owner['label_value']]
        if args.isolated_native_domain:
            create += RESOURCE_ARGS
        created_result = run(create + [report['image'], 'sleep', 'infinity'])
        created = True
        if owner:
            owner['container_id'] = created_result.stdout.strip()
            if not re.fullmatch(r'[0-9a-f]{64}', owner['container_id']):
                raise RuntimeError('Docker create did not return the full owned container ID')
            (evidence / 'owned-container.json').write_text(json.dumps(owner, indent=2) + '\n')
        run([args.docker, 'start', name])
        if args.isolated_native_domain:
            inspect_container('isolation-created')
        run([args.docker, 'cp', str(inputs), name + ':/inputs'])
        run([args.docker, 'exec', '--user', '0', name, 'sh', '-c',
             'chmod 777 /work'])
        run([args.docker, 'exec', name, 'id'], 'build-user.log')
        run([args.docker, 'exec', name, 'sh', '/inputs/build-stock.sh'], 'stock-build.log')
        if args.ardupilot_source:
            run([args.docker, 'exec', name, 'sh', '/inputs/build-sitl.sh'], 'sitl-build.log')
        if args.isolated_native_domain:
            # Source dependency setup may use the owned bridge. All app/FC
            # execution happens only after this fresh full-ID/label/resource
            # inspection and effective network disconnection proof.
            report['isolation_before'], previous_namespace = disconnect_owned_bridge(
                args.docker, owner, run, inspect_container)
        run([args.docker, 'exec', name, 'sh', '/inputs/build-app.sh'], 'mavmoos-build-test.log')
        if args.isolated_native_domain:
            report['isolation_after'], _ = inspect_container('isolation-after', True, True)
        report['result'] = 'PASS'
    except RecipeInterrupted as error:
        report['interrupted'] = True
        report['error'] = str(error)
        raise
    except Exception as error:
        report['error'] = str(error)
    finally:
        if created and args.isolated_native_domain and previous_namespace is not None and 'isolation_after' not in report:
            try:
                report['isolation_after'], _ = inspect_container('isolation-after', True, True)
            except Exception as error:
                report['result'] = 'FAIL'
                report['isolation_postflight_error'] = str(error)
                report.setdefault('error', 'isolated Linux postflight not proven')
        if created:
            # Retain failure evidence too; a failed build must not erase its logs.
            run([args.docker, 'cp', name + ':/work/evidence/.', str(evidence)], check=False)
            if not args.ci_tier:
                run([args.docker, 'cp', name + ':/work/mavmoos-build/evidence', str(evidence / 'tests')], check=False)
            else:
                ci_result = evidence / 'ci/result.json'
                if not ci_result.is_file() or json.loads(ci_result.read_text()).get('result') != 'PASS':
                    report['result'] = 'FAIL'
                    report.setdefault('error', 'source-built CI result absent or failed')
        if created and not args.keep_container:
            try:
                identity = name
                if owner:
                    inspected = run([args.docker, 'inspect', owner['container_id']])
                    identity = owned_container(owner, json.loads(inspected.stdout))
                run([args.docker, 'stop', '--time', '3', identity], check=False)
                run([args.docker, 'rm', identity], check=False)
                absent=run([args.docker,'ps','--no-trunc','-aq','--filter','name=^/'+name+'$'],check=False)
                report['container_removed']=absent.returncode==0 and not absent.stdout.strip()
                if not report['container_removed']:
                    raise RuntimeError('owned container remains after cleanup')
            except Exception as error:
                report['result'] = 'FAIL'
                report['container_removed'] = False
                report['container_cleanup_error'] = str(error)
                report.setdefault('error', 'owned_container_cleanup_failed')
        report['retained_container'] = bool(created and args.keep_container)
        (evidence / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
        (evidence / 'command-results.json').write_text(json.dumps(command_results, indent=2) + '\n')
        (evidence / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    return 0 if report['result'] == 'PASS' else 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except RecipeInterrupted as error:
        print(json.dumps({'result': 'FAIL', 'error': str(error), 'interrupted': True}), file=sys.stderr)
        raise SystemExit(128 + error.number)
