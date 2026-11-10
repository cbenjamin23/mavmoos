#!/usr/bin/env python3
"""Install and exercise the kit in an independent offline Linux consumer."""
import json
from pathlib import Path
import socket
import subprocess
import sys
import tarfile

from kit import ARCHIVE, LIBRARY, STOCK_RUNTIME, V1_ARCHIVE, sha, verify, write_json

INPUTS = Path('/inputs')
WORK = Path('/work')


def need(condition, message):
    if not condition:
        raise ValueError(message)


def unpack(archive, destination):
    with tarfile.open(archive, 'r:gz') as source:
        members = source.getmembers()
        names = {member.name for member in members}
        need(len(members) == len(names) <= 80 and sum(member.size for member in members) <= 128 * 1024 * 1024
             and all((member.isfile() or member.isdir()) and
                     (member.name == 'mavmoos' or member.name.startswith('mavmoos/'))
                     for member in members), 'archive exceeds the runtime kit layout bounds')
        source.extractall(destination, members=members, filter='data')


def expect_invalid(root, revision, message):
    try:
        verify(root, revision)
    except (FileNotFoundError, ValueError) as error:
        return str(error)
    raise ValueError(message)


def observe(root, destination, *, v1=False):
    witness = INPUTS / 'witness'
    command = [
        sys.executable, '-B', str(WORK / 'fixtures/tests/observer_matrix/run_loopback.py'),
        '--identity', 'ardupilot', '--moos-bin', str(root / 'bin' if v1 else witness),
        '--build', str(root), '--plugin-directory', str(root / 'plugins'),
        '--frames', str(witness / 'observer_matrix_frames'),
        '--probe', str(witness / 'observer_matrix_probe'), '--output', str(destination),
    ]
    with (WORK / 'evidence' / (destination.name + '.log')).open('w') as stream:
        result = subprocess.run(command, cwd=WORK, stdout=stream, stderr=subprocess.STDOUT, timeout=30)
    evidence = json.loads((destination / 'result.json').read_text())
    need(evidence['cleanup_proven'] and evidence['bridge_egress_datagrams'] == 0, 'consumer witness cleanup or egress failed')
    return result.returncode, evidence


def main(revision, *, v1=False):
    out = WORK / 'evidence'
    out.mkdir(exist_ok=False)
    report = {'result': 'FAIL', 'source_revision': revision}
    try:
        routes = Path('/proc/net/route').read_text().splitlines()[1:]
        routes6 = Path('/proc/net/ipv6_route').read_text().splitlines()
        interfaces = {name: int(Path('/sys/class/net', name, 'flags').read_text(), 16)
                      for _, name in socket.if_nameindex()}
        need({name for name, flags in interfaces.items() if flags & 1} == {'lo'} and
             not routes and all(row.split()[-1] == 'lo' for row in routes6) and
             not (WORK / 'mavmoos-build').exists() and
             not (WORK / 'moos-ivp').exists(), 'consumer is not independent and loopback-only')
        report['interfaces'] = interfaces
        relocated = WORK / 'installed-elsewhere'
        relocated.mkdir()
        archive = INPUTS / (V1_ARCHIVE if v1 else ARCHIVE)
        report['package_sha256'] = sha(archive)
        unpack(archive, relocated)
        root = relocated / 'mavmoos'
        manifest = verify(root, revision)
        need(manifest['schema'] == (2 if v1 else 1), 'consumer selected the wrong package format')
        report['prefix'] = str(root)
        report['module_count'] = manifest['module_count']
        report['app_sha256'] = manifest['files']['bin/iMAVLink']['sha256']
        executable = root / 'bin/iMAVLink'
        if v1:
            with (out / 'sha256sum.log').open('w') as log:
                subprocess.run(['sha256sum', '--strict', '--check', 'SHA256SUMS'],
                               cwd=root, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=10)
        with (out / 'interface.txt').open('w') as log:
            subprocess.run([executable, '--interface'], cwd=WORK, stdout=log, stderr=subprocess.STDOUT,
                           check=True, timeout=5)
        with (out / 'dynamic-dependencies.txt').open('w') as log:
            binaries = {executable, root / LIBRARY, *(root / 'plugins').glob('*.so')}
            if v1:
                binaries.update(root / name for name in STOCK_RUNTIME)
            for path in sorted(binaries):
                result = subprocess.check_output(['ldd', path], cwd=WORK, stderr=subprocess.STDOUT,
                                                 text=True, timeout=5)
                log.write(f'{path.name}\n{result}')
                need('not found' not in result and '/work/mavmoos-build/' not in result and
                     '/work/moos-ivp/' not in result, f'unresolved or build-tree dependency in {path.name}')
                if path.name == 'mavmoos_telemetry.so':
                    geodesy = [row.split()[2] for row in result.splitlines()
                               if row.split()[:2] == ['libMOOSGeodesy.so', '=>']]
                    need(len(geodesy) == 1 and Path(geodesy[0]).resolve() == root / LIBRARY,
                         'telemetry did not resolve geodesy inside the relocated kit')
                    report['geodesy_sha256'] = manifest['files'][LIBRARY]['sha256']
        with tarfile.open(INPUTS / 'fixtures.tar') as archive:
            archive.extractall(WORK / 'fixtures', filter='data')

        status, positive = observe(root, out / 'relocated', v1=v1)
        need(status == 0 and positive['result'] == 'PASS', 'relocated app did not complete the receive-only fixture')
        module_hashes = positive['loaded_modules_sha256']
        need(len(module_hashes) == 8 and all(manifest['files']['plugins/' + name]['sha256'] == checksum
                                             for name, checksum in module_hashes.items()),
             'observed module composition differs from packaged artifacts')
        report['loaded_module_hashes'] = module_hashes
        report['rx_only_udp_egress'] = positive['bridge_egress_datagrams']
        if v1:
            report['stock_runtime_sha256'] = {name: manifest['files'][name]['sha256'] for name in sorted(STOCK_RUNTIME)}

        module = root / 'plugins/mavmoos_odometry.so'
        withheld = WORK / module.name
        module.rename(withheld)
        try:
            report['missing_module_manifest_rejected'] = expect_invalid(
                root, revision, 'kit verification accepted a missing module')
            status, negative = observe(root, out / 'missing-module', v1=v1)
            log = (out / 'missing-module/probe.log').read_text()
            need(status == 1 and negative['result'] == 'FAIL' and
                 'state=CONFIG_ERROR,reason=plugin_configuration_failed' in log and
                 'CAPTURE\tMAVMOOS_CAPABILITIES\t' not in log, 'app did not fail closed with a missing module')
            report['missing_module_app_rejected'] = True
        finally:
            withheld.rename(module)
        original = module.read_bytes()
        try:
            module.write_bytes(original + b'changed')
            report['changed_module_rejected'] = expect_invalid(
                root, revision, 'kit verification accepted altered bytes')
        finally:
            module.write_bytes(original)
        other = ('1' if revision[0] == '0' else '0') + revision[1:]
        report['wrong_revision_rejected'] = expect_invalid(root, other, 'kit verification accepted a stale revision')
        need(verify(root, revision) == manifest, 'consumer fixture did not restore its private installation')
        report['result'] = 'PASS'
    except (OSError, ValueError, KeyError, subprocess.SubprocessError) as error:
        report['error'] = str(error)
    write_json(out / 'consumer-result.json', report)
    return int(report['result'] != 'PASS')


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1], v1=sys.argv[2:] == ['--v1']))
