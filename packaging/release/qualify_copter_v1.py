#!/usr/bin/env python3
"""Run the full owned Copter starter from the archived ARM64 v1 release."""
import argparse
from io import BytesIO
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tarfile
import tempfile
from types import SimpleNamespace

from consume import unpack
from kit import sha, verify, write_json

INSTALL = Path('/work/mavmoos')
PROBE = Path('/test-probe/sitl_workflow_probe')
FIRMWARE = Path('/firmware')
ATTESTATION = Path('/firmware-attestation.json')
PYTHON_PACKAGES = Path('/fixture-python')
FIXTURE_PATHS = (
    'config/setup.json', 'config/firmware/arducopter-4.6.3-quad.json', 'scripts/firmware/manifest.py',
    'tests/run_sitl.py', 'tests/copter_release.py', 'tests/SitlWorkflowProbe.cpp', 'tests/owned_processes.py',
    'tests/firmware_identity.py', 'tests/plane_native/flight_fixture.py', 'tests/operations/run_operations.py',
    'tests/copter_frame_fixture.py',
    'tests/control/catalog_evidence.py', 'missions/copter_release/launch.sh',
    'missions/copter_release/meta_vehicle.bhv', 'missions/copter_sitl/meta_vehicle.moos',
    'tests/px4_offboard/run_px4_offboard.py',
)
MODULES = ('telemetry', 'parameters', 'arducopter_control')


def need(condition, message):
    if not condition:
        raise ValueError(message)


def load(path):
    return json.loads(path.read_text())


def worker():
    sys.path[:0] = ['/work/fixture/tests', '/work/fixture/tests/px4_offboard', str(PYTHON_PACKAGES)]
    from run_px4_offboard import namespace
    from run_sitl import main

    launch = load(Path('/work/launch.json'))
    revision = launch['source_revision']
    manifest = verify(INSTALL, revision)
    need(manifest['schema'] == 2 and sha(INSTALL / 'manifest.json') == launch['manifest_sha256'] and
         sha(PROBE) == launch['probe_sha256'] and sha(Path(__file__)) == launch['worker_sha256'] and
         sha(ATTESTATION) == launch['firmware_attestation_sha256'] and
         all(sha(Path('/work/fixture') / name) == digest for name, digest in launch['fixture_sources'].items()) and
         all(os.statvfs(path).f_flag & os.ST_RDONLY for path in
             (INSTALL, PROBE, '/work/fixture', FIRMWARE, PYTHON_PACKAGES, ATTESTATION)),
         'Copter starter requires the exact read-only runtime, probe and fixture mounts')
    write_json(Path('/evidence/namespace.json'), namespace())
    os.environ.update(GIT_CONFIG_COUNT='1', GIT_CONFIG_KEY_0='safe.directory',
                      GIT_CONFIG_VALUE_0=str(FIRMWARE))
    sys.argv[1:] = ['--release-demo', '--public-stock', '--expected-app-revision', revision,
                    '--build', str(INSTALL), '--moos-root', str(INSTALL), '--behavior-dir', str(INSTALL / 'lib'),
                    '--ardupilot-root', str(FIRMWARE), '--firmware-attestation', str(ATTESTATION),
                    '--output', '/evidence/native']
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(143)))
    status = main(installed=SimpleNamespace(root=INSTALL, probe=PROBE, manifest=manifest,
                                           fixture_revision=launch['fixture_revision']))
    need(verify(INSTALL, revision) == manifest and sha(INSTALL / 'manifest.json') == launch['manifest_sha256'],
         'installed payload changed during the Copter starter')
    return status


def qualify(args):
    repo = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(repo / 'tests/px4_offboard'))
    from qualify_fixed_wing import (CIInterrupted, LinuxDomainLease, LocalDocker, TOOLS_IMAGE, container,
                                   interrupted, mount, native_files)

    out = args.output.resolve()
    out.mkdir(mode=0o700, parents=True, exist_ok=False)
    signal.signal(signal.SIGTERM, interrupted)
    report = {'schema': 1, 'result': 'FAIL', 'scope': 'installed Linux ARM64 v1 Copter starter', 'hardware': False}
    endpoint = None
    try:
        def git(*command):
            return subprocess.check_output(['git', '-C', str(repo), *command], timeout=35)

        attestation = args.firmware_attestation.resolve(strict=True)
        firmware = args.firmware.resolve(strict=True)
        probe = args.probe.resolve(strict=True)
        python_packages = args.python_packages.resolve(strict=True)
        archive = args.archive.resolve(strict=True)
        receipt = json.loads(git('show', '73499b1:docs/implementation/2026-11-07-linux-arm64-v1/readback.json'))
        revision = receipt['source_revision']
        fixture_revision = git('rev-parse', 'HEAD').decode().strip()
        need(receipt['result'] == 'PASS' and receipt['package']['sha256'] == sha(archive) and
             receipt['package']['bytes'] == archive.stat().st_size and
             not git('diff', '--name-only', revision, fixture_revision, '--',
                     'tests/SitlWorkflowProbe.cpp', 'missions/copter_release', 'missions/copter_sitl',
                     'config/firmware/arducopter-4.6.3-quad.json') and
             git('show', f'{fixture_revision}:packaging/release/qualify_copter_v1.py') == Path(__file__).read_bytes(),
             'release bundle, original mission/probe source or installed qualification controller differs')
        with tempfile.TemporaryDirectory(prefix='.installed-', dir=out.parent) as directory:
            stage = Path(directory).resolve()
            unpack(archive, stage)
            manifest = verify(stage / 'mavmoos', revision)
            need(manifest['files']['bin/pHelmIvP']['sha256'] == receipt['stock_helm_sha256'],
                 'stock helm differs from the release receipt')
            fixture_tar = git('archive', fixture_revision, *FIXTURE_PATHS)
            with tarfile.open(fileobj=BytesIO(fixture_tar)) as source:
                source.extractall(stage / 'fixture', filter='data')
            for name in ('qualify_copter_v1.py', 'kit.py', 'consume.py'):
                (stage / name).write_bytes(git('show', f'{fixture_revision}:packaging/release/{name}'))
            launch = {'source_revision': revision, 'fixture_revision': fixture_revision,
                      'archive_sha256': receipt['package']['sha256'], 'probe_sha256': sha(probe),
                      'manifest_sha256': sha(stage / 'mavmoos/manifest.json'),
                      'firmware_attestation_sha256': sha(attestation),
                      'worker_sha256': sha(stage / Path(__file__).name),
                      'fixture_sources': {name: sha(stage / 'fixture' / name) for name in FIXTURE_PATHS}}
            write_json(stage / 'launch.json', launch)
            write_json(out / 'launch.json', launch)
            report.update(launch)
            endpoint = LocalDocker()
            image, = json.loads(endpoint.capture('image', 'inspect', TOOLS_IMAGE))
            need((image['Id'], image['Os'], image['Architecture']) == (TOOLS_IMAGE, 'linux', 'arm64'),
                 'pinned Linux ARM64 tools image differs')
            mounts = [mount('bind', stage, '/work'), mount('bind', firmware, str(FIRMWARE)),
                      mount('bind', attestation, str(ATTESTATION)), mount('bind', probe, str(PROBE)),
                      mount('bind', python_packages, str(PYTHON_PACKAGES)), mount('bind', out, '/evidence', True)]
            lease = LinuxDomainLease(out / 'domain-lease.json', revision)
            lease.acquire()
            try:
                container(endpoint, out, 'native', mounts, ['--worker'], 320,
                          entry='/work/qualify_copter_v1.py', scope_prefix='release-v1-copter',
                          name_prefix='mavmoos-v1-copter')
            finally:
                lease.release()
        raw = out / 'native/result.json'
        proof = load(raw)
        modules = [f'mavmoos_{name}.so' for name in MODULES]
        expected = {name: manifest['files']['plugins/' + name]['sha256'] for name in modules}
        need(proof['result'] == 'PASS' and proof['source_revision'] == revision and
             proof['fixture_revision'] == fixture_revision and proof['moos_revision'] == manifest['moos_revision'] and
             proof['helm_sha256'] == receipt['stock_helm_sha256'] and
             proof['app_sha256'] == manifest['files']['bin/iMAVLink']['sha256'] and
             proof['inputs_sha256'] == proof['inputs_sha256_after'] and
             proof['inputs_sha256']['probe'] == launch['probe_sha256'] and
             proof['inputs_sha256']['firmware_attestation'] == launch['firmware_attestation_sha256'] and
             proof['loaded_modules_sha256'] == expected and proof['cleanup_proven'] is True,
             'installed Copter flight or artifact readback differs')
        native_files(out / 'native', proof['files'], ('fc-monitor.mavlink', 'fc-monitor.jsonl',
                     'workflow.log', 'targ_copter.moos', 'targ_copter.bhv'))
        report.update(result='PASS', firmware_revision=proof['ardupilot_revision'],
                      firmware_sha256=proof['copter_sha256'], stock_helm_sha256=proof['helm_sha256'],
                      installed_processes=proof['installed_processes'], proofs=proof['proofs'],
                      ports_clear=proof['ports_clear'], native_receipt_sha256=sha(raw))
    except (CIInterrupted, KeyboardInterrupt, OSError, ValueError, LookupError, RuntimeError,
            subprocess.SubprocessError) as error:
        report['error'] = f'{type(error).__name__}: {error}'
    finally:
        if endpoint is not None:
            endpoint.close()
        write_json(out / 'qualification.json', report)
    print(f"{report['result']} installed Copter starter. Evidence at {out}")
    return int(report['result'] != 'PASS')


if __name__ == '__main__':
    if sys.argv[1:] == ['--worker']:
        raise SystemExit(worker())
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('archive', 'firmware', 'firmware-attestation', 'probe', 'python-packages', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    raise SystemExit(qualify(parser.parse_args()))
