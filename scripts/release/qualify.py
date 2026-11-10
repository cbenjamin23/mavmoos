#!/usr/bin/env python3
"""Verify the package and owned SITL receipts for one local release candidate."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import tarfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts/ci'))
sys.path.insert(0, str(ROOT / 'packaging/release'))
from run import accepted, audit_junit, parse, require, sha
from kit import CHECKSUMS, checksums, standard_modules
from matrix import MANIFEST, host_observations, load_manifest, receipt_paths
from profiles import KIT_PROBE, audit_aircraft, kit_witnesses, native_files, native_observations
from receipts import RECEIPT_LIMIT, local, receipt
import stress

REV = re.compile(r'[0-9a-f]{40}\Z')
DIGEST = re.compile(r'[0-9a-f]{64}\Z')
PLATFORM = re.compile(r'(darwin|linux)-(arm64|x86_64)\Z')
REGISTRY = 'config/capabilities/copter.json'
STOCK_MANIFEST = 'config/firmware/arducopter-4.6.3-quad.json'
HOST_CASES = ('copter_operator', 'rover_ground', 'rover_boat',
              'plane_normal', 'plane_takeover', 'sub')
NATIVE_CASES = (*HOST_CASES, 'px4_offboard')
COPTER_PROOFS = ('lifecycle', 'short_handoff', 'expired_handoff', 'stationkeeping',
                 'pilot_takeover', 'in_flight_parameter_read')
COPTER_FILES = ('fc-monitor.jsonl', 'fc-monitor.mavlink', 'operator-rx.jsonl', 'operator-rx.mavlink',
                'operator-ownership.jsonl', 'release-phases.jsonl', 'workflow.log', 'fixture-pilot.jsonl',
                'firmware-manifest.json', 'firmware-attestation.json')
KIT_CONSUMER_REJECTIONS = {
    'missing_module_manifest_rejected': 'expected the app, {count} standard modules, geodesy library and operator files',
    'changed_module_rejected': 'kit payload differs from its manifest',
    'wrong_revision_rejected': 'kit source revision differs',
}
MODULE = re.compile(r'mavmoos_[a-z0-9_]+\.so\Z')
RAW_REVISION = {
    'copter_operator': 'source_revision', 'rover_ground': 'mavmoos_revision',
    'rover_boat': 'mavmoos_revision', 'plane_normal': 'runner_revision',
    'plane_takeover': 'runner_revision', 'sub': 'app_source', 'px4_offboard': 'source_revision',
}
RECEIPTS = {
    'software': 'software/result.json',
    'linux_kit': 'linux/result.json',
    'copter_operator': 'copter/profile-result.json',
    **{case: f'profiles/{case}/profile-result.json' for case in NATIVE_CASES[1:]},
}


def identity(value, pattern, label):
    require(isinstance(value, str) and pattern.fullmatch(value), f'invalid {label}')
    return value


def referenced(root, reference):
    require(isinstance(reference, dict), 'missing referenced artifact')
    path = local(root, reference.get('path'))
    require(sha(path) == identity(reference.get('sha256'), DIGEST, 'artifact SHA-256'),
            f'artifact bytes differ from receipt: {path.name}')
    return path


def file_record(directory, path):
    return {'path': path.relative_to(directory).as_posix(), 'sha256': sha(path)}


def bound(report, revision):
    require(type(report.get('schema')) is int and report['schema'] == 1, 'unknown receipt schema')
    require(report.get('source_revision') == revision, 'receipt refers to a different MAVMOOS commit')
    require(report.get('result') == 'PASS', 'receipt did not pass')


def supervision(record, directory):
    require(record.get('cleanup_proven') is True and record.get('timeout') is False and
            accepted(record), 'runner process ownership or cleanup failed')
    referenced(directory, {'path': Path(record['log']).name, 'sha256': record['log_sha256']})
    processes = receipt(referenced(directory, {'path': Path(record['process_evidence']).name,
                                               'sha256': record['process_evidence_sha256']}))
    require(processes.get('schema') == 1 and processes.get('cleanup_proven') is True and
            processes.get('remaining') == processes.get('unexpected_live_descendants') == [] and
            processes.get('cleanup_failure') is None, 'raw process ownership or cleanup failed')


def build_modules(paths):
    modules = [(Path(name).name, digest) for name, digest in paths.items()
               if Path(name).parent.name == 'plugins' and MODULE.fullmatch(Path(name).name)]
    require(len(modules) == len(dict(modules)), 'build module names are ambiguous')
    return dict(modules)


def software(report, directory, revision, registry_bytes):
    bound(report, revision)
    require(report.get('tier') == 'software' and report.get('hosted') is False
            and report.get('native_evidence') == [], 'expected the local software tier')
    require(report.get('source_revision_after') == revision and report.get('input_drift') == []
            and report.get('artifact_drift') == [], 'software source or artifacts drifted')
    registry_sha = hashlib.sha256(registry_bytes).hexdigest()
    require(report.get('registry_sha256') == registry_sha, 'software used a different registry')
    tests = {test['id']: test for test in parse(registry_bytes)['tests'] if 'software' in test['tiers']}
    required = {name for name, test in tests.items() if test['required']}
    selected = report.get('selected_ids', [])
    require(required and len(selected) == len(set(selected)) and required <= set(selected) <= tests.keys(),
            'software tier did not select every required test exactly once')
    executed = audit_junit(local(directory, 'junit.xml'), [tests[name] for name in selected])
    require(report.get('cases') == executed, 'software case summary differs from JUnit')
    require(report.get('commands'), 'software runner supervision is missing')
    for command in report['commands']:
        supervision(command, directory)
    before, after = report.get('artifacts_before'), report.get('artifacts_after')
    require(isinstance(before, dict) and before == after, 'software artifact identities differ')
    apps = [digest for path, digest in before.items() if Path(path).parts[-2:] == ('bin', 'iMAVLink')]
    require(len(apps) == 1, 'software must identify one built iMAVLink artifact')
    return {'app_sha256': identity(apps[0], DIGEST, 'software app SHA-256'),
            'modules_sha256': build_modules(before),
            'required_cases': len(required), 'executed_cases': len(executed), 'registry_sha256': registry_sha,
            'junit_sha256': sha(directory / 'junit.xml')}


def app_identity(report, moos_revision):
    require(report.get('moos_revision') == moos_revision, 'stock MOOS revision differs from the pin')
    return {'platform': identity(report.get('platform'), PLATFORM, 'platform'),
            'moos_revision': moos_revision,
            'helm_sha256': identity(report.get('helm_sha256'), DIGEST, 'stock helm SHA-256'),
            'app_sha256': identity(report.get('app_sha256'), DIGEST, 'app SHA-256')}


def consumer_witnesses(directory, loaded):
    paths = (local(directory, 'consumer/relocated/result.json'),
             local(directory, 'consumer/missing-module/result.json'))
    for path, expected in zip(paths, ('PASS', 'FAIL')):
        raw = receipt(path)
        ports = raw.get('ports_clear')
        require(raw.get('result') == expected and raw.get('cleanup_proven') is True and
                raw.get('owned_processes_remaining') == raw.get('cleanup_errors') == [] and
                isinstance(ports, dict) and ports and all(value is True for value in ports.values()) and
                type(raw.get('bridge_egress_datagrams')) is int and raw['bridge_egress_datagrams'] == 0,
                f'relocated consumer witness failed: {path.parent.name}')
        if expected == 'PASS':
            require(raw.get('loaded_modules_sha256') == loaded, 'relocated modules differ from consumer receipt')
    probe = local(directory, 'consumer/missing-module/probe.log')
    require(probe.stat().st_size <= RECEIPT_LIMIT, 'missing-module probe log exceeds size limit')
    text = probe.read_text()
    require('state=CONFIG_ERROR,reason=plugin_configuration_failed' in text and
            'CAPTURE\tMAVMOOS_CAPABILITIES\t' not in text, 'missing module did not fail closed in the app')
    return [file_record(directory, path) for path in (*paths, probe)]


def kit(report, directory, revision, moos_revision, module_count, expected_modules=None):
    bound(report, revision)
    context = app_identity(report, moos_revision)
    require(context['platform'] == 'linux-arm64', 'runtime kit requires a Linux ARM64 build')
    require(type(report.get('module_count')) is int and report['module_count'] == module_count
            and report.get('container_removed') is True, 'runtime kit module roster or cleanup failed')
    package = report.get('package')
    archive = referenced(directory, package)
    require(type(package.get('bytes')) is int and 0 < package['bytes'] == archive.stat().st_size,
            'package length differs from receipt')
    require(report.get('consumer_result') == 'consumer/consumer-result.json', 'missing relocated consumer receipt')
    consumer_path = referenced(directory, {'path': report['consumer_result'],
                                          'sha256': report.get('consumer_result_sha256')})
    consumer = receipt(consumer_path)
    interfaces = consumer.get('interfaces')
    require(consumer.get('result') == 'PASS' and consumer.get('source_revision') == revision and
            consumer.get('app_sha256') == context['app_sha256'] and
            consumer.get('module_count') == module_count and consumer.get('package_sha256') == package['sha256'] and
            consumer.get('prefix') == '/work/installed-elsewhere/mavmoos' and
            isinstance(interfaces, dict) and all(type(flags) is int for flags in interfaces.values()) and
            {name for name, flags in interfaces.items() if flags & 1} == {'lo'} and
            type(consumer.get('rx_only_udp_egress')) is int and consumer['rx_only_udp_egress'] == 0 and
            consumer.get('missing_module_app_rejected') is True and
            all(consumer.get(key) == reason.format(count=module_count)
                for key, reason in KIT_CONSUMER_REJECTIONS.items()),
            'consumer receipt lacks tested kit identity, relocation, zero egress or fail-closed proof')
    manifest_path = local(directory, 'package-manifest.json')
    manifest = receipt(manifest_path)
    require(manifest.get('schema') == 1 and manifest.get('source_revision') == revision and
            manifest.get('platform') == context['platform'] and manifest.get('module_count') == module_count,
            'package manifest identity differs')
    files = manifest['files']
    modules = build_modules({name: entry['sha256'] for name, entry in files.items()})
    require(expected_modules is None or set(modules) == expected_modules,
            'package modules differ from the selected source roster')
    loaded = consumer.get('loaded_module_hashes')
    require(len(modules) == module_count and isinstance(loaded, dict) and len(loaded) == 8 and
            all(modules.get(name) == identity(digest, DIGEST, 'loaded module SHA-256') for name, digest in loaded.items())
            and files['bin/iMAVLink']['sha256'] == context['app_sha256']
            and files['lib/libMOOSGeodesy.so']['sha256'] == consumer.get('geodesy_sha256')
            and report['build_witness_sha256']['/work/moos-ivp/bin/pHelmIvP'] == context['helm_sha256'],
            'package app, module, geodesy or helm identity differs')
    with tarfile.open(archive, 'r:gz') as bundle:
        members = bundle.getmembers()
        require(len(members) == len({member.name for member in members}) <= 64 and
                sum(member.size for member in members) <= 128 * 1024 * 1024,
                'runtime package exceeds its layout bounds')
        record = bundle.getmember('mavmoos/manifest.json')
        require(record.isfile() and record.size <= RECEIPT_LIMIT, 'invalid manifest in package')
        with bundle.extractfile(record) as stream:
            require(parse(stream.read()) == manifest, 'embedded manifest differs from verified package manifest')
        app = bundle.getmember('mavmoos/bin/iMAVLink')
        require(app.isfile() and 0 < app.size == files['bin/iMAVLink']['bytes'], 'package app length differs')
        with bundle.extractfile(app) as stream:
            require(hashlib.file_digest(stream, 'sha256').hexdigest() == context['app_sha256'],
                    'package app differs from tested artifact')
        if expected_modules is not None:
            payload = {'mavmoos/' + name: entry for name, entry in files.items()}
            checksum_name = 'mavmoos/' + CHECKSUMS
            require(all(member.isdir() or member.isfile() for member in members) and
                    {member.name for member in members if member.isfile()} ==
                    {*payload, 'mavmoos/manifest.json', checksum_name}, 'package payload roster differs')
            for name, entry in payload.items():
                member = bundle.getmember(name)
                require(member.size == entry['bytes'], f'package payload length differs: {name}')
                with bundle.extractfile(member) as stream:
                    require(hashlib.file_digest(stream, 'sha256').hexdigest() == entry['sha256'],
                            f'package payload digest differs: {name}')
            with bundle.extractfile(checksum_name) as stream:
                require(stream.read() == checksums(files).encode(), 'package checksum list differs')
    witnesses = consumer_witnesses(directory, loaded)
    return {**context, 'module_count': module_count, 'package': package,
            'modules_sha256': modules,
            'package_manifest': file_record(directory, manifest_path),
            'consumer_receipt': file_record(directory, consumer_path), 'consumer_witnesses': witnesses}


def copter_observations(raw, directory, context):
    firmware = raw['firmware_identity']
    platform = firmware['platform']
    inputs = raw['inputs_sha256']
    ports = raw['ports_clear']
    require(raw.get('release_case') == 'copter_operator' and raw.get('hardware') is False and
            raw.get('timewarp') == 1 and raw.get('probe_exit') == 0 and raw.get('cleanup_proven') is True and
            raw.get('owned_processes_remaining') == [] and not raw.get('postflight_error') and
            not raw.get('pilot_cleanup_error') and
            set(ports) == {'bridge', 'monitor', 'rc', 'peer', 'operator', 'operator_peer', 'monitor_peer',
                           'irlock', 'moos_tcp', 'base_tcp'} and all(value is True for value in ports.values()),
            'Copter raw workflow or teardown differs')
    require(firmware['profile_id'] == 'arducopter-4.6.3-quad' and firmware['frame'] == 'quad' and
            firmware['fixture']['hardware'] is False and raw['ardupilot_revision'] == firmware['source_revision'] and
            raw['copter_sha256'] == inputs['native'] == firmware['native_sha256'] and
            raw['moos_revision'] == context['moos_revision'] and
            raw['app_sha256'] == inputs['app'] == context['app_sha256'] and
            raw['helm_sha256'] == inputs['pHelmIvP'] == context['helm_sha256'] and
            context['platform'] == f"{platform['system'].lower()}-{platform['machine']}" and
            inputs == raw['inputs_sha256_after'], 'Copter raw firmware, app, helm or source inputs differ')
    proofs = raw['proofs']
    require(isinstance(proofs, dict) and all(proofs.get(name) for name in COPTER_PROOFS) and
            proofs['lifecycle'] == raw['independent_monitor'] and
            proofs['lifecycle']['final_disarmed'] is proofs['lifecycle']['final_on_ground'] is True and
            proofs['pilot_takeover']['stale_navigation_fenced'] is proofs['pilot_takeover']['explicit_reacquire'] is True,
            'Copter raw observations lack workflow, takeover or ground proof')
    files = raw['files']
    native_files(directory, files, COPTER_FILES)
    for name in ('manifest', 'attestation'):
        require(files[f'firmware-{name}.json']['sha256'] == firmware[name + '_sha256'] == inputs['firmware_' + name],
                'Copter archived firmware input differs')
    onboard = raw['independent_pilot_rcin']
    log = 'logs/' + Path(onboard['input_path']).name
    require(log.endswith('.BIN') and files[log]['sha256'] == onboard['input_sha256'],
            'Copter DataFlash identity differs from its index')
    return {'revision': firmware['source_revision'], 'native_sha256': firmware['native_sha256'], 'frame': 'quad'}, proofs


def observed_artifacts(raw, case, context):
    if case == 'copter_operator' or case.startswith('rover_'):
        modules = raw['loaded_modules_sha256']
    elif case.startswith('plane_'):
        modules = {'mavmoos_' + name.removeprefix('module_') + '.so': digest
                   for name, digest in raw['input_sha256'].items() if name.startswith('module_')}
    elif case == 'sub':
        modules = {'mavmoos_ardusub_control.so': raw['inputs']['sub_plugin']['sha256'],
                   'mavmoos_telemetry.so': raw['inputs']['telemetry_plugin']['sha256']}
    else:
        modules = build_modules(raw['artifacts_before'])
    require(isinstance(modules, dict) and modules and
            all(MODULE.fullmatch(name) and identity(digest, DIGEST, 'native module SHA-256')
                for name, digest in modules.items()), 'invalid observed module digests')
    return {'app': context['app_sha256'], 'helm': context['helm_sha256'], 'modules': modules}


def px4_container(directory):
    cleanup_path = local(directory, 'container-cleanup.json')
    before_path = local(directory, 'container-before.json')
    after_path = local(directory, 'container-after.json')
    cleanup = receipt(cleanup_path)
    before = receipt(before_path)
    after = receipt(after_path)
    container = identity(cleanup.get('container'), DIGEST, 'PX4 container ID')
    require(cleanup.get('removed') is True and before['Id'] == after['Id'] == container and
            before['HostConfig']['NetworkMode'] == 'none' and after['State']['Running'] is False and
            after['State']['ExitCode'] == 0, 'owned PX4 container identity or teardown differs')
    return [file_record(directory, path) for path in (before_path, after_path, cleanup_path)]


def native(report, directory, case, revision, moos_revision):
    bound(report, revision)
    require(report.get('case') == case and report.get('cleanup_proven') is True,
            'native case identity or cleanup failed')
    context = app_identity(report, moos_revision)
    firmware = report.get('firmware')
    require(isinstance(firmware, dict) and isinstance(firmware.get('frame'), str)
            and firmware['frame'].strip(), 'missing native frame')
    identity(firmware.get('revision'), REV, 'firmware revision')
    identity(firmware.get('native_sha256'), DIGEST, 'firmware SHA-256')
    proof = report.get('proofs')
    require(isinstance(proof, dict) and proof, 'native observation proofs are missing')
    if case == 'copter_operator':
        require(all(proof.get(name) for name in COPTER_PROOFS), 'missing Copter operator proof')
    raw = report.get('native_receipt')
    raw_path = referenced(directory, raw)
    require(raw['path'] == ('result.json' if case == 'copter_operator' else 'native/result.json'),
            'unexpected underlying native receipt path')
    observed = receipt(raw_path)
    require(observed.get('result') == 'PASS' and observed.get(RAW_REVISION[case]) == revision,
            'underlying native receipt failed or refers to a different source')
    artifacts = observed_artifacts(observed, case, context)
    supporting = {}
    if case == 'copter_operator':
        observed_firmware, observations = copter_observations(observed, raw_path.parent, context)
    else:
        require(context['platform'] == ('linux-arm64' if case == 'px4_offboard' else 'darwin-arm64'),
                'native platform differs from the selected profile')
        observed_firmware, observations = native_observations(observed, raw_path.parent, case, revision, artifacts)
        supervisor = local(directory, 'launcher-result.json')
        supervision(receipt(supervisor), directory)
        supporting['launcher_receipt'] = file_record(directory, supervisor)
        if case == 'px4_offboard':
            supporting['container_cleanup'] = px4_container(directory)
    require(firmware == observed_firmware, 'normalized firmware differs from raw observation')
    require(proof == observations, 'normalized workflow proofs differ from raw observations')
    return {**context, 'modules_sha256': artifacts['modules'], 'firmware': firmware, 'proofs': proof,
            'native_receipt': raw, **supporting}


def aircraft(report, directory, cell, revision, moos_revision, kit):
    bound(report, revision)
    context = app_identity(report, moos_revision)
    require(context['platform'] == 'linux-arm64' and report['case'] == 'px4_offboard' and
            report['aircraft'] == cell['scenario'] and report['hardware'] is False and
            report['cleanup_proven'] is True, 'PX4 release cell identity or cleanup differs')
    raw_path = referenced(directory, report['native_receipt'])
    require(report['native_receipt']['path'] == 'native/result.json', 'unexpected aircraft raw receipt path')
    local(directory, 'build-manifest.json')
    require(kit, 'PX4 aircraft requires the verified Linux kit receipt')
    observed = audit_aircraft(directory, revision, cell['scenario'],
                              {'app': context['app_sha256'], 'helm': context['helm_sha256'],
                               'stock_witness_sha256': kit['stock_witness_sha256'],
                               'probe_sha256': kit['probe_sha256']})
    require(all(observed[key] == report[key] for key in observed),
            'aircraft normalization differs from its raw receipt')
    raw = receipt(raw_path)
    artifacts = raw['artifacts_before']
    launcher = local(directory, 'launcher-result.json')
    supervision(receipt(launcher), directory)
    lease = receipt(local(directory, 'domain-lease.json'))
    require(lease['source_ref'] == revision and lease['acquired'] is lease['released'] is True and
            lease['release_errors'] == [], 'aircraft Linux-domain lease did not retire')
    return {**context, 'modules_sha256': build_modules(artifacts), 'profile': cell['profile'],
            'probe_sha256': artifacts[KIT_PROBE],
            'stock_witness_sha256': {name: artifacts[name] for name in kit['stock_witness_sha256']},
            'firmware': observed['firmware'], 'proofs': observed['proofs'],
            'native_receipt': report['native_receipt'], 'launcher_receipt': file_record(directory, launcher),
            'container_cleanup': px4_container(directory)}


def check_cohorts(cells):
    host = [cell for cell in cells.values() if cell.get('platform') == 'darwin-arm64']
    for field in ('app_sha256', 'helm_sha256', 'platform'):
        require(len({cell[field] for cell in host}) <= 1, f'host native {field} differs across cells')
    if host and 'software' in cells:
        require(host[0]['app_sha256'] == cells['software']['app_sha256'],
                'native host app differs from the software-qualified build')
        require(all(cells['software']['modules_sha256'].get(name) == digest
                    for cell in host for name, digest in cell['modules_sha256'].items()),
                'native host module differs from the software-qualified build')
    linux = [cell for name, cell in cells.items() if name != 'linux_kit' and cell.get('platform') == 'linux-arm64']
    for cell in linux if 'linux_kit' in cells else ():
        for field in ('app_sha256', 'helm_sha256', 'platform'):
            require(cells['linux_kit'][field] == cell[field],
                    f'PX4 native {field} differs from the Linux kit build')
        require(all(cells['linux_kit']['modules_sha256'].get(name) == digest
                    for name, digest in cell['modules_sha256'].items()),
                'PX4 native module differs from the Linux kit build')
        if 'probe_sha256' in cell:
            require(cells['linux_kit']['probe_sha256'] == cell['probe_sha256'],
                    'PX4 native probe differs from the Linux kit build')
            require(cells['linux_kit']['stock_witness_sha256'] == cell['stock_witness_sha256'],
                    'PX4 native stock files differ from the Linux kit build')
    for pair in (('rover_ground', 'rover_boat'), ('plane_normal', 'plane_takeover')):
        if all(name in cells for name in pair):
            a, b = (cells[name]['firmware'] for name in pair)
            require(all(a[key] == b[key] for key in ('revision', 'native_sha256')),
                    f'{pair[0]} and {pair[1]} used different firmware')
            require((a['frame'] != b['frame']) == (pair[0] == 'rover_ground'),
                    f'{pair[0]} and {pair[1]} frame pairing is invalid')


def stress_observations(root, name, path, data, revision):
    case = stress.CASES[name]
    native_run = stress.check_run(path.parent, data, case, revision)
    require(data.get('result') == native_run.get('result') == 'PASS', 'stress mission did not pass')
    replay = f'{name}.replay'
    require(local(root, f'supervisor/{replay}.started').read_text() == revision + '\n',
            'stress replay was started against a different commit')
    supervisor = receipt(local(root, f'supervisor/{replay}.json'))
    log = local(root, f'supervisor/{replay}.log')
    require(Path(supervisor['log']).name == log.name, 'stress replay log does not match the case')
    supervision(supervisor, log.parent)
    receipt(log)
    return {'profile': case.profile, 'replay': file_record(root, log)}


def qualify(root, revision, registry_bytes, moos_revision, module_count, *, matrix=None, expected_modules=None):
    root = root.resolve()
    require(root.is_dir(), 'evidence directory is absent')
    cells, missing, errors = {}, [], {}
    stress_cases = matrix.get('stress_cases', ()) if matrix is not None else ()
    for case, name in receipt_paths(matrix, RECEIPTS).items():
        path = root / name
        supervisor = root / 'supervisor' / (case + '.started')
        if not path.exists() and not path.is_symlink():
            if matrix is not None and (supervisor.exists() or supervisor.is_symlink()):
                errors[case] = 'started stage lacks a finished receipt'
            else:
                missing.append(case)
            continue
        try:
            path = local(root, name)
            data = receipt(path)
            if matrix is not None:
                require(local(root, f'supervisor/{case}.started').read_text() == revision + '\n',
                        'stage was started against a different commit')
                supervision(receipt(local(root, f'supervisor/{case}.json')), supervisor.parent)
            cell = matrix['cases'].get(case) if matrix is not None else None
            if case == 'software':
                result = software(data, path.parent, revision, registry_bytes)
            elif case == 'linux_kit':
                result = kit(data, path.parent, revision, moos_revision, module_count, expected_modules)
                if matrix is not None:
                    result.update(kit_witnesses(data, path.parent, revision))
            elif case in stress_cases:
                result = stress_observations(root, case, path, data, revision)
            elif cell and cell['fixture'] == 'px4_aircraft':
                result = aircraft(data, path.parent, cell, revision, moos_revision, cells.get('linux_kit'))
            elif cell:
                result = host_observations(data, path.parent, cell, revision)
            else:
                result = native(data, path.parent, case, revision, moos_revision)
            cells[case] = {**result, 'receipt': {'path': name, 'sha256': sha(path)}}
        except (OSError, ValueError, RuntimeError, TypeError, LookupError, AttributeError, EOFError, tarfile.TarError,
                ET.ParseError) as error:
            errors[case] = str(error)
    try:
        check_cohorts(cells)
    except ValueError as error:
        errors['cohort'] = str(error)
    report = {'schema': 1, 'result': 'FAIL' if errors else 'INCOMPLETE' if missing else 'PASS',
              'scope': 'local SITL release candidate', 'source_revision': revision, 'cells': cells,
              'missing': missing, 'errors': errors}
    if matrix is not None:
        report.update(required_cases=list(receipt_paths(matrix, RECEIPTS)),
                      outside_release_matrix=matrix['outside_release_matrix'])
    return report


def frozen_file(revision, path):
    return subprocess.check_output(['git', '-C', str(ROOT), 'show', f'{revision}:{path}'])


def frozen_matrix(revision):
    manifest = subprocess.check_output(['git', '-C', str(ROOT), 'ls-tree', revision, '--', MANIFEST], text=True)
    return load_manifest(frozen_file(revision, MANIFEST), RECEIPTS) if manifest else None


def qualify_revision(evidence, revision):
    revision = identity(revision, REV, 'release source revision')
    require(subprocess.check_output(['git', '-C', str(ROOT), 'cat-file', '-t', revision],
                                    text=True).strip() == 'commit', 'release identity must be a commit')
    registry = frozen_file(revision, REGISTRY)
    manifest = parse(frozen_file(revision, STOCK_MANIFEST))
    moos = identity(manifest['dependencies']['moos_revision'], REV, 'stock MOOS pin')
    # The declared standard provider roster excludes example plugins.
    modules = {Path(name).name for name in standard_modules(frozen_file(revision, 'src/plugins/CMakeLists.txt').decode())}
    matrix = frozen_matrix(revision)
    return qualify(evidence, revision, registry, moos, len(modules),
                   matrix=matrix, expected_modules=modules if matrix is not None else None)


def save_report(report, destination):
    with destination.open('x') as output:
        json.dump(report, output, indent=2)
        output.write('\n')
    print(json.dumps({'result': report['result'], 'source_revision': report['source_revision'],
                      'passed': len(report['cells']), 'missing': report['missing'],
                      'errors': report['errors']}))
    return {'PASS': 0, 'FAIL': 1, 'INCOMPLETE': 2}[report['result']]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--revision', required=True, help='full Git commit of the built release')
    parser.add_argument('--evidence', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path, help='new report JSON file')
    args = parser.parse_args()
    return save_report(qualify_revision(args.evidence, args.revision), args.output)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError, TypeError, subprocess.CalledProcessError) as error:
        print(json.dumps({'result': 'FAIL', 'error': str(error)}), file=sys.stderr)
        raise SystemExit(1)
