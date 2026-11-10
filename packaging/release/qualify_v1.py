#!/usr/bin/env python3
"""Fly the extracted Linux ARM64 v1 bundle in owned, pinned PX4 SITL."""
import argparse
from io import BytesIO
import json
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tarfile
import tempfile

from consume import unpack
from kit import V1_ARCHIVE, sha, verify, write_json

INSTALL = Path('/work/mavmoos')
PROBE = Path('/work/px4_offboard_moos_probe')
FLIGHT_FILES = (
    'bin/iMAVLink', 'bin/pHelmIvP', 'bin/MOOSDB', 'bin/pAntler', 'bin/pLogger',
    'bin/nsplug', 'lib/libBHV_ConstantAltitude.so', 'plugins/mavmoos_telemetry.so',
    'plugins/mavmoos_px4_control.so',
)


def need(condition, message):
    if not condition:
        raise ValueError(message)


def worker():
    sys.path.insert(0, '/work/fixtures/tests/px4_offboard')
    from run_aircraft import AircraftWorkflow, STABLE_PIN, main
    from run_px4_offboard import STABLE_PX4_REVISION

    launch = json.loads(Path('/work/launch.json').read_text())
    revision = launch['source_revision']
    manifest_path = INSTALL / 'manifest.json'
    manifest = verify(INSTALL, revision)
    pins = {INSTALL / name: manifest['files'][name]['sha256'] for name in FLIGHT_FILES}
    pins[PROBE] = launch['probe_sha256']
    pin = json.loads(STABLE_PIN.read_text())
    need(pin['revision'] == STABLE_PX4_REVISION and sha(manifest_path) == launch['manifest_sha256'] and
         sha(Path(__file__)) == launch['worker_sha256'],
         'installed manifest, worker or stable firmware pin differs')

    class InstalledAircraft(AircraftWorkflow):
        def stable_work_pins(self, stable):
            return pins

    args = argparse.Namespace(vehicle='quadrotor', output=Path('/evidence/native'), source_revision=revision,
                              db_port=15300, px4_root=Path('/opt/px4'), moos_root=INSTALL,
                              behavior_dir=INSTALL / 'lib', build=INSTALL, probe=PROBE, nsplug=None,
                              stable_pin=pin, build_manifest=manifest_path)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(143)))
    result = main(args, InstalledAircraft)
    need(verify(INSTALL, revision) == manifest and sha(manifest_path) == launch['manifest_sha256'],
         'runtime bundle changed during native SITL')
    return result


def stage_kit(kit, stage, repo):
    receipt = json.loads((kit / 'result.json').read_text())
    archive = kit / V1_ARCHIVE
    package = receipt['package']
    need(receipt['result'] == 'PASS' and receipt['container_removed'] is True and
         receipt['platform'] == 'linux-arm64' and package['path'] == V1_ARCHIVE and
         package['bytes'] == archive.stat().st_size and package['sha256'] == sha(archive),
         'kit build receipt does not identify the verified Linux ARM64 archive')
    revision = receipt['source_revision']
    unpack(archive, stage)
    root = stage / 'mavmoos'
    manifest = verify(root, revision)
    probe = kit / PROBE.name
    probe_digest = receipt['build_witness_sha256']['/work/mavmoos-build/bin/' + PROBE.name]
    need(manifest['schema'] == 2 and manifest == json.loads((kit / 'package-manifest.json').read_text()) and
         manifest['files']['bin/pHelmIvP']['sha256'] == receipt['helm_sha256'] and sha(probe) == probe_digest,
         'packaged stock helm, runtime manifest or test-only probe differs')
    shutil.copy2(probe, stage / PROBE.name)
    fixture_sources = subprocess.check_output(
        ['git', '-C', str(repo), 'archive', revision, 'tests/px4_offboard', 'tests/owned_processes.py',
         'packaging/release/kit.py', 'packaging/release/consume.py'], timeout=30)
    with tarfile.open(fileobj=BytesIO(fixture_sources)) as source:
        source.extractall(stage / 'fixtures', filter='data')
    for name in ('kit.py', 'consume.py'):
        shutil.copyfile(stage / 'fixtures/packaging/release' / name, stage / name)
    shutil.copyfile(__file__, stage / Path(__file__).name)
    launch = {'source_revision': revision, 'archive_sha256': package['sha256'], 'probe_sha256': probe_digest,
              'manifest_sha256': sha(root / 'manifest.json'), 'worker_sha256': sha(Path(__file__))}
    write_json(stage / 'launch.json', launch)
    return launch, manifest


def qualify(args):
    repo = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(repo / 'tests/px4_offboard'))
    from qualify_fixed_wing import (CIInterrupted, LinuxDomainLease, LocalDocker, TOOLS_IMAGE, container,
                                   interrupted, mount, native_files, volume)
    signal.signal(signal.SIGTERM, interrupted)
    out = args.output.resolve()
    out.mkdir(mode=0o700, parents=True, exist_ok=False)
    report = {'schema': 1, 'result': 'FAIL', 'scope': 'installed Linux ARM64 runtime in owned SITL', 'hardware': False}
    endpoint = None
    try:
        with tempfile.TemporaryDirectory(prefix='.install-', dir=out.parent) as directory:
            stage = Path(directory).resolve()
            launch, manifest = stage_kit(args.kit.resolve(strict=True), stage, repo)
            report.update(launch)
            endpoint = LocalDocker()
            image, = json.loads(endpoint.capture('image', 'inspect', TOOLS_IMAGE))
            need((image['Id'], image['Os'], image['Architecture']) == (TOOLS_IMAGE, 'linux', 'arm64'),
                 'pinned offline Linux ARM64 tools image differs')
            firmware = volume(endpoint.capture, args.px4_package_volume)
            pin = json.loads((stage / 'fixtures/tests/px4_offboard/firmware-v1.17.0.json').read_text())
            mounts = [mount('bind', stage, '/work'), mount('volume', firmware, '/opt/px4'),
                      mount('bind', out, '/evidence', True)]
            lease = LinuxDomainLease(out / 'domain-lease.json', launch['source_revision'])
            lease.acquire()
            try:
                container(endpoint, out, 'native', mounts, ['--worker'], 540,
                          entry=f'/work/{Path(__file__).name}', scope_prefix='release-v1',
                          name_prefix='mavmoos-release-v1')
            finally:
                lease.release()

        raw = out / 'native/result.json'
        proof = json.loads(raw.read_text())
        expected = {str(INSTALL / name): manifest['files'][name]['sha256'] for name in FLIGHT_FILES}
        expected.update({str(PROBE): launch['probe_sha256'],
                         str(INSTALL / 'manifest.json'): launch['manifest_sha256']})
        expected.update({f'/opt/px4/{name}': digest for name, digest in pin['files'].items()})
        need(proof['result'] == 'PASS' and proof['source_revision'] == launch['source_revision'] and
             proof['firmware_revision'] == pin['revision'] and
             proof['stock_helm_process']['command'].split()[0] == str(INSTALL / 'bin/pHelmIvP') and
             all(proof['artifacts_before'].get(name) == digest for name, digest in expected.items()) and
             not proof['artifact_drift'] and not proof['owned_processes_remaining'] and
             all(proof['ports_clear'].values()), 'flight or installed artifact readback differs')
        native_files(out / 'native', proof['files'], ('gcs.mavlink', 'moos-wire.jsonl',
                     'preflight-gcs.mavlink', 'fixture-cli.log', 'targ_px4.moos', 'targ_px4.bhv'))
        report.update(result='PASS', model=proof['model'], firmware_revision=proof['firmware_revision'],
                      eastward_legs_m=[leg['east_m'] for leg in proof['legs']],
                      homeward_m=proof['return_motion']['homeward_m'],
                      landed_disarmed=proof['landed_disarmed'], ports_clear=proof['ports_clear'],
                      native_receipt_sha256=sha(raw), native_file_count=len(proof['files']))
    except (CIInterrupted, KeyboardInterrupt, OSError, ValueError, LookupError, RuntimeError,
            subprocess.SubprocessError) as error:
        report['error'] = f'{type(error).__name__}: {error}'
    finally:
        if endpoint is not None:
            endpoint.close()
        write_json(out / 'qualification.json', report)
    print(f"{report['result']} installed runtime SITL. Evidence at {out}")
    return int(report['result'] != 'PASS')


if __name__ == '__main__':
    if sys.argv[1:] == ['--worker']:
        raise SystemExit(worker())
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--kit', required=True, type=Path)
    parser.add_argument('--px4-package-volume', required=True)
    parser.add_argument('--output', required=True, type=Path)
    raise SystemExit(qualify(parser.parse_args()))
