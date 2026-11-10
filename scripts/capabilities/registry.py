#!/usr/bin/env python3
"""Validate capability declarations and exact archived qualification cells.

This reader does not load modules, execute tests, open sockets or promote a
configured capability into a claim of qualified behavior.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path, PurePosixPath
import re
import stat
import xml.etree.ElementTree as ET
import zipfile

TIERS = {'software', 'smoke', 'full'}
HEX = re.compile(r'[a-f0-9]{64}\Z')
REV = re.compile(r'[a-f0-9]{40}\Z')
IDENT = re.compile(r'[a-zA-Z0-9_.-]+\Z')

# These are case-specific publication scopes, not all capabilities incidentally
# exercised by a flight. Callers must still select a nonempty supported subset.
NATIVE_CASE_CAPS = {
    'native.copter.route_hold_return': {'copter.hold.guided', 'copter.hold.helm'},
    'native.copter.wind_repeated_cycle': {'copter.hold.guided', 'copter.hold.helm'},
    'native.copter.auto_complete_handback': {'copter.mission.execute'},
    'native.copter.auto_pause_abort': {'copter.mission.execute'},
    'native.imu.rx_only': {'observations.imu'},
}
NATIVE_CASE_PROBES = {
    'native.copter.route_hold_return': 'copter_scenario_probe',
    'native.copter.wind_repeated_cycle': 'copter_scenario_probe',
    'native.copter.auto_complete_handback': 'mission_execution_probe',
    'native.copter.auto_pause_abort': 'mission_execution_probe',
    'native.imu.rx_only': 'imu_moos_probe',
}
FIRMWARE_FIELDS = ('schema', 'profile_id', 'manifest_sha256', 'attestation_sha256',
                   'source_revision', 'native_path', 'native_sha256', 'platform',
                   'frame', 'fixture', 'build_provenance', 'qualification')


def require(value, message):
    if not value:
        raise ValueError(message)


def fields(value, required, optional=(), label='record'):
    require(isinstance(value, dict), label + ': object required')
    require(set(required) <= set(value) <= set(required) | set(optional), label + ': missing/extra fields')


def parse(data):
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, 'duplicate JSON key: ' + key)
            result[key] = value
        return result
    def bad(value):
        raise ValueError('nonfinite JSON number: ' + value)
    result = json.loads(data, object_pairs_hook=pairs, parse_constant=bad)
    def finite(value):
        if isinstance(value, float):
            require(math.isfinite(value), 'nonfinite JSON number')
        elif isinstance(value, dict):
            for item in value.values():
                finite(item)
        elif isinstance(value, list):
            for item in value:
                finite(item)
    finite(result)
    return result


def relative(value):
    require(isinstance(value, str) and value and '\\' not in value and '\0' not in value, 'invalid relative path')
    path = PurePosixPath(value)
    require(not path.is_absolute() and '..' not in path.parts and str(path) == value, 'noncanonical/escaping relative path')
    return path


def identifiers(values, label):
    require(isinstance(values, list) and len(values) == len(set(values)), label + ': missing/duplicate list')
    require(all(isinstance(v, str) and IDENT.fullmatch(v) for v in values), label + ': invalid ID')
    return set(values)


def native_case_shape(cell):
    """The additive variant has no operation_id/terminal fiction."""
    evidence = cell['evidence']
    fields(evidence, ('kind', 'matrix', 'readback', 'job', 'case', 'raw'), label='native case evidence')
    require(evidence['kind'] == 'native_case', 'unknown evidence kind')
    require(evidence['case'] == cell['test_id'] and evidence['case'] in NATIVE_CASE_CAPS,
            'unrecognized or mixed native qualification case')
    require(set(cell['capability_ids']) <= NATIVE_CASE_CAPS[evidence['case']],
            'case does not establish the selected capability scope')
    require(isinstance(evidence['job'], str) and IDENT.fullmatch(evidence['job']), 'invalid full job ID')
    for label in ('matrix', 'readback'):
        ref = evidence[label]
        fields(ref, ('path', 'sha256'), ('reviewer_sha256',) if label == 'readback' else (), label)
        relative(ref['path'])
        require(isinstance(ref['sha256'], str) and HEX.fullmatch(ref['sha256']), label + ': missing digest')
    require(isinstance(evidence['readback'].get('reviewer_sha256'), str) and
            HEX.fullmatch(evidence['readback']['reviewer_sha256']), 'readback: missing exact reviewer digest')
    fields(evidence['raw'], ('archive', 'member', 'sha256'), label='raw native case')
    relative(evidence['raw']['archive']); relative(evidence['raw']['member'])
    require(isinstance(evidence['raw']['sha256'], str) and HEX.fullmatch(evidence['raw']['sha256']),
            'raw native case: missing digest')


def _file_sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


class NativeCaseEvidence:
    """Stdlib-only publication checks over an already deeply reviewed archive.

    The SHA-bound readback is the explicit semantic-review receipt. This class
    checks its provenance and the referenced full job/case metadata, which may
    be retained as a checksum-verified subset of the original archive. It does not
    decode MAVLink, replay flight criteria, authenticate a signer or establish a
    new simulator qualification. No archived code is imported or executed.
    """
    def __init__(self, root, matrix_ref, readback_ref):
        self.root = Path(root).resolve()
        self.matrix_path = self.local(matrix_ref['path'])
        require(_file_sha(self.matrix_path) == matrix_ref['sha256'], 'matrix bytes changed')
        self.matrix = parse(self.matrix_path.read_bytes())
        self.readback_path = self.local(readback_ref['path'])
        require(_file_sha(self.readback_path) == readback_ref['sha256'], 'readback bytes changed')
        self.readback = parse(self.readback_path.read_bytes())
        require(type(self.matrix.get('schema')) is int and self.matrix['schema'] == 1 and isinstance(self.matrix.get('source_revision'), str) and
                REV.fullmatch(self.matrix['source_revision']), 'unsupported native matrix identity')
        require(type(self.readback.get('schema')) is int and self.readback['schema'] == 1 and self.readback.get('result') == 'PASS' and
                self.readback.get('git_readback') is True and
                self.readback.get('source_revision') == self.matrix['source_revision'] and
                self.readback.get('matrix_sha256') == matrix_ref['sha256'] and
                self.readback.get('reviewer_sha256') == readback_ref['reviewer_sha256'],
                'deep readback failed or is not bound to this exact matrix/source/reviewer')
        self.archives = self.index(self.matrix.get('archives'), 'file', 'archives')
        self.jobs = self.index(self.matrix.get('jobs'), 'id', 'matrix jobs')
        self.reviewed_jobs = self.index(self.readback.get('jobs'), 'id', 'readback jobs')
        require(set(self.jobs) == set(self.reviewed_jobs), 'readback job inventory differs')
        self.checked_archives = set()
        self.job_cache = {}
        source = self.matrix.get('source')
        fields(source, ('archive', 'prefix'), label='selected source')
        self.source_archive, self.source_prefix = source['archive'], source['prefix']
        self.frozen_registry_data = self.member(self.source_archive, self.join(self.source_prefix, 'config/capabilities/copter.json'))
        self.frozen_registry = parse(self.frozen_registry_data)
        self.frozen = validate_registry(self.frozen_registry)

    @staticmethod
    def index(rows, key, label):
        require(isinstance(rows, list) and rows, label + ': nonempty list required')
        result = {}
        for row in rows:
            require(isinstance(row, dict) and isinstance(row.get(key), str) and row[key] and row[key] not in result,
                    label + ': missing/duplicate identity')
            result[row[key]] = row
        return result

    def local(self, name, parent=None):
        relative(name)
        path = ((parent or self.root) / name).resolve()
        require(path.is_relative_to(self.root) and path.is_file(), 'evidence path escapes publication root or is missing')
        return path

    @staticmethod
    def join(prefix, name):
        if prefix:
            relative(prefix)
        relative(name)
        return str(PurePosixPath(prefix) / name)

    def member(self, archive, member):
        relative(member)
        require(archive in self.archives, 'unknown evidence archive')
        descriptor = self.archives[archive]
        fields(descriptor, ('file', 'sha256', 'members'), label='indexed archive')
        original = self.matrix_path.parent / relative(archive)
        metadata_only = not original.exists() and not original.is_symlink()
        candidate = str(relative(archive).with_suffix('.metadata.zip')) if metadata_only else archive
        path = self.local(candidate, self.matrix_path.parent)
        require(isinstance(descriptor['members'], dict) and member in descriptor['members'], 'unindexed evidence member')
        with zipfile.ZipFile(path) as zipped:
            if archive not in self.checked_archives:
                if not metadata_only:
                    require(_file_sha(path) == descriptor['sha256'], 'archive bytes changed')
                infos = zipped.infolist()
                names = {info.filename for info in infos}
                expected = set(descriptor['members'])
                require(len(names) == len(infos) and names and
                        (names <= expected if metadata_only else names == expected),
                        'archive has duplicate/unindexed/missing members')
                if metadata_only:
                    require(sum(info.file_size for info in infos) <= 16 * 1024 * 1024,
                            'metadata archive exceeds bound')
                for info in infos:
                    relative(info.filename)
                    require(not info.is_dir() and not info.flag_bits & 1 and
                            not stat.S_ISLNK(info.external_attr >> 16), 'unsafe archive member')
                    require(isinstance(descriptor['members'][info.filename], str) and
                            HEX.fullmatch(descriptor['members'][info.filename]), 'invalid indexed member digest')
                    if metadata_only:
                        require(hashlib.sha256(zipped.read(info)).hexdigest() == descriptor['members'][info.filename],
                                'archived metadata bytes changed')
                self.checked_archives.add(archive)
            require(member in zipped.namelist(), 'required archive member is missing')
            require(zipped.getinfo(member).file_size <= 16 * 1024 * 1024, 'metadata member exceeds bound')
            data = zipped.read(member)
        require(hashlib.sha256(data).hexdigest() == descriptor['members'][member], 'evidence member bytes changed')
        return data

    def json(self, archive, member):
        return parse(self.member(archive, member))

    def job_member(self, job, name):
        return self.join(job['prefix'], name)

    def recorded_member(self, job, path):
        require(isinstance(path, str) and '\\' not in path and '\0' not in path, 'invalid recorded evidence path')
        base = PurePosixPath(job['recorded_output_root'])
        require(base.is_absolute() and '..' not in base.parts, 'invalid recorded output root')
        value = PurePosixPath(path)
        require(value.is_absolute() and str(value) == path and '..' not in value.parts and value.is_relative_to(base),
                'recorded evidence path escapes exact full job')
        return self.job_member(job, str(value.relative_to(base)))

    def full_job(self, ident):
        if ident in self.job_cache:
            return self.job_cache[ident]
        require(ident in self.jobs, 'full qualification job is absent')
        job, reviewed = self.jobs[ident], self.reviewed_jobs[ident]
        require(job.get('qualification') == 'current' and job.get('tier') == 'full' and
                reviewed.get('qualification') == 'current' and reviewed.get('tier') == 'full' and
                reviewed.get('result') == 'PASS', 'reference, targeted, failed or non-full job cannot qualify')
        report = self.json(job['archive'], self.job_member(job, 'result.json'))
        require(type(report.get('schema')) is int and report['schema'] == 1 and report.get('result') == 'PASS' and report.get('tier') == 'full' and
                report.get('hosted') is False and report.get('source_revision') == self.matrix['source_revision'] and
                report.get('source_revision_after') == self.matrix['source_revision'], 'full CI source/result mismatch')
        for before, after in (('inputs_before', 'inputs_after'), ('artifacts_before', 'artifacts_after'),
                              ('firmware', 'firmware_after'), ('moos_dependency_before', 'moos_dependency_after')):
            require(isinstance(report.get(before), dict) and report[before] and report[before] == report.get(after),
                    'full CI input/artifact/firmware/MOOS preservation failed')
        require(report.get('input_drift') == [] and report.get('artifact_drift') == [] and
                report.get('moos_dependency_drift') is False, 'full CI dependency drift')
        require(report.get('registry_sha256') == hashlib.sha256(self.frozen_registry_data).hexdigest(),
                'full CI used a different capability/test registry')
        plan = self.index(self.json(job['archive'], self.job_member(job, 'selected-tests.json')), 'id', 'selected tests')
        expected = {t['id'] for t in self.frozen_registry['tests'] if 'full' in t['tiers']}
        absent = identifiers(report.get('absent_optional'), 'absent optional tests')
        require(absent <= {t['id'] for t in self.frozen_registry['tests'] if not t['required']} and
                set(plan) == expected - absent and identifiers(report.get('selected_ids'), 'full selected IDs') == set(plan) and
                identifiers(report.get('scope_excluded_ids'), 'excluded IDs') == absent,
                'targeted/missing/extra full CI selection')
        for key, row in plan.items():
            declared = self.frozen['tests'][key]
            require(row.get('kind') == declared['kind'] and type(row.get('native')) is bool and
                    row['native'] == (declared['scope'] == 'native_sitl'), 'planned test role differs from frozen declaration')
        cases = self.index(report.get('cases'), 'id', 'executed cases')
        require(set(cases) == set(plan) and all(c.get('result') == 'PASS' for c in cases.values()),
                'full CI case failure/missing execution')
        for case in cases.values():
            elapsed = case.get('duration_s')
            require(type(elapsed) in (int, float) and math.isfinite(elapsed) and elapsed >= 0, 'invalid case duration')
        suite = ET.fromstring(self.member(job['archive'], self.job_member(job, 'junit.xml')))
        junit = list(suite.iter('testcase'))
        names = [row.get('name') for row in junit]
        require(names and len(names) == len(set(names)) and set(names) == set(plan), 'JUnit missing/duplicate/extra executed case')
        for row in junit:
            require(not any(node.tag in ('failure', 'error', 'skipped') for node in row) and
                    row.get('status', 'run') == 'run', 'JUnit failed/skipped/disabled case')
            require(float(row.get('time', 'nan')) == cases[row.get('name')]['duration_s'], 'JUnit/case duration mismatch')
        require(suite.get('tests') == str(len(plan)) and
                all(suite.get(key, '0') == '0' for key in ('failures', 'errors', 'disabled', 'skipped')),
                'JUnit full case counters disagree')
        native = self.index(report.get('native_evidence'), 'id', 'native evidence')
        native_ids = {key for key, row in plan.items() if row.get('native') is True}
        require(native_ids and set(native) == native_ids and all(row.get('result') == 'PASS' for row in native.values()) and
                reviewed.get('cases') == len(plan) and reviewed.get('native_cases') == len(native_ids) and
                reviewed.get('cleanup_reports') == len(native_ids), 'full/readback native inventory mismatch')
        self.job_cache[ident] = (job, reviewed, report, plan, native)
        return self.job_cache[ident]

    def verify(self, cell, current_caps, current_tests):
        evidence = cell['evidence']
        require(cell['source_revision'] == self.matrix['source_revision'], 'mixed qualified source revisions')
        job, reviewed, report, plan, native = self.full_job(evidence['job'])
        ident = evidence['case']
        require(ident in plan and plan[ident].get('native') is True and ident in native and
                plan[ident].get('kind') == 'native', 'unknown/non-native selected case')
        frozen_test = self.frozen['tests'][ident]
        require(frozen_test == current_tests[ident], 'test declaration changed since qualified source')
        for capid in cell['capability_ids']:
            frozen_cap = self.frozen['capabilities'].get(capid)
            require(frozen_cap is not None and frozen_cap['implementation_status'] == 'implemented' and
                    ident in frozen_cap['test_ids'] and ident in current_caps[capid]['test_ids'] and
                    frozen_cap['owner'] == current_caps[capid]['owner'] and
                    frozen_cap['catalog_capability'] == current_caps[capid]['catalog_capability'],
                    'capability/test/provider differs from frozen source')
        raw_ref = evidence['raw']
        member = self.job_member(job, 'cases/' + ident + '/evidence/result.json')
        require(raw_ref['archive'] == job['archive'] and raw_ref['member'] == member,
                'raw result belongs to a different job/case/archive')
        raw_data = self.member(raw_ref['archive'], member)
        require(hashlib.sha256(raw_data).hexdigest() == raw_ref['sha256'], 'raw native result digest mismatch')
        raw = parse(raw_data)
        reports = native[ident].get('cleanup_reports')
        require(isinstance(reports, list) and len(reports) == 1 and
                reports[0].get('sha256') == raw_ref['sha256'] and
                self.recorded_member(job, reports[0].get('path')) == member and
                native[ident].get('firmware_binding') == 'selected_copter_manifest', 'raw/full CI reference differs')
        semantic = [row for row in reviewed.get('semantic_replays', []) if row.get('id') == ident]
        require(len(semantic) == 1 and semantic[0].get('member') == member and isinstance(semantic[0].get('proof'), dict) and
                semantic[0]['proof'], 'qualified deep semantic case replay is absent/ambiguous')
        require(raw.get('result') == 'PASS' and raw.get('hardware') is False and type(raw.get('probe_exit')) is int and raw['probe_exit'] == 0 and
                raw.get('profile') == cell['vehicle_profile'] and raw.get('scenario') == frozen_test['scenario'] and
                raw.get('source_ref') == cell['source_revision'] and raw.get('ardupilot_ref') == cell['firmware_revision'] and
                raw.get('app_sha256') == cell['app_sha256'] and raw.get('native_sha256') == cell['native_sha256'],
                'raw source/profile/artifact/result mismatch')
        require(raw.get('artifact_drift') == [] and raw.get('owned_processes_remaining') == [] and
                isinstance(raw.get('ports_clear'), dict) and raw['ports_clear'] and
                all(value is True for value in raw['ports_clear'].values()) and
                ('cleanup_proven' not in raw or raw['cleanup_proven'] is True), 'raw cleanup/drift is unproven')
        for key, value in (('source_ref_after', cell['source_revision']), ('revision', cell['source_revision']),
                            ('ardupilot_revision', cell['firmware_revision']), ('copter_sha256', cell['native_sha256'])):
            require(key not in raw or raw[key] == value, 'contradictory raw identity alias: ' + key)
        expected = plan[ident].get('expected_identity', {})
        firmware = report['firmware']
        for key in FIRMWARE_FIELDS:
            require(key in firmware and expected.get(key) == firmware[key] and
                    raw.get('firmware_identity', {}).get(key) == firmware[key] and
                    raw.get('firmware_identity_after', {}).get(key) == firmware[key],
                    'raw/planned/full firmware identity differs: ' + key)
        require(expected.get('vehicle_profile') == cell['vehicle_profile'] and
                expected.get('mavmoos_revision') == cell['source_revision'] and
                expected.get('app_sha256') == cell['app_sha256'] and raw.get('probe_sha256') == expected.get('probe_sha256') and
                HEX.fullmatch(str(raw.get('probe_sha256', ''))) and reviewed.get('app_sha256') == cell['app_sha256'] and
                reviewed.get('native_profile_id') == firmware['profile_id'] and
                reviewed.get('native_platform') == firmware['platform'] and
                firmware['source_revision'] == cell['firmware_revision'] and firmware['native_sha256'] == cell['native_sha256'] and
                firmware['frame'] == cell['frame'] and firmware['fixture'].get('hardware') is False,
                'case/qualification firmware or artifact identity differs')
        build = PurePosixPath(report.get('build', ''))
        require(build.is_absolute() and '..' not in build.parts, 'full CI build path missing')
        artifacts = report['artifacts_before']
        require(artifacts.get(str(build / 'bin/iMAVLink')) == cell['app_sha256'] and
                artifacts.get(firmware['native_path']) == cell['native_sha256'] and
                artifacts.get(str(build / 'bin' / NATIVE_CASE_PROBES[ident])) == raw['probe_sha256'],
                'raw app/native/probe differs from frozen full-job artifacts')
        if ident == 'native.imu.rx_only':
            modules = {'telemetry', 'imu'}
        elif ident in ('native.copter.route_hold_return', 'native.copter.wind_repeated_cycle'):
            modules = {'telemetry', 'parameters', 'arducopter_control'}
        else:
            modules = {'telemetry', 'parameters', 'missions', 'arducopter_control'}
        module_names = {'mavmoos_' + name + '.so' for name in modules}
        loaded = raw.get('loaded_modules_sha256')
        require(isinstance(loaded, dict) and set(loaded) == module_names and
                all(HEX.fullmatch(str(value)) and expected.get('module_sha256', {}).get(name) == value and
                    artifacts.get(str(build / 'plugins' / name)) == value for name, value in loaded.items()),
                'loaded capability module bytes differ from frozen plan/artifacts')
        platform = firmware['platform']
        require(cell['platform'] == platform['system'].lower() + '-' + platform['machine'].lower(), 'platform cell differs')
        copied = raw.get('copied_firmware') if ident == 'native.imu.rx_only' else raw.get('firmware_inputs')
        fields(copied, ('manifest', 'attestation'), label='copied firmware')
        raw_parent = str(PurePosixPath(member).parent)
        for role in ('manifest', 'attestation'):
            ref = copied[role]
            key = 'file' if ident == 'native.imu.rx_only' else 'path'
            fields(ref, (key, 'sha256'), label='copied ' + role)
            require(ref['sha256'] == firmware[role + '_sha256'], 'copied firmware role digest differs')
            data = self.member(job['archive'], self.join(raw_parent, ref[key]))
            require(hashlib.sha256(data).hexdigest() == ref['sha256'], 'copied firmware bytes differ')
            if role == 'manifest':
                manifest = parse(data)
                require(manifest.get('profile_id') == firmware['profile_id'] and
                        manifest.get('vehicle_profile') == cell['vehicle_profile'] and
                        manifest.get('source', {}).get('revision') == cell['firmware_revision'] and
                        manifest.get('source', {}).get('release_label') == cell['firmware_label'], 'firmware label/profile differs')


def validate_registry(value, source_root=None, verify_evidence=False):
    fields(value, ('schema', 'name', 'scope', 'source_baseline', 'tests', 'capabilities', 'qualifications',
                   'pending_qualification_tests', 'status_definitions'), label='registry')
    require(type(value['schema']) is int and value['schema'] == 1, 'unsupported registry schema')
    require(value['tests'] and value['capabilities'], 'empty registry')
    tests = {}
    for test in value['tests']:
        fields(test, ('id', 'kind', 'tiers', 'timeout_s', 'required', 'scope'),
               ('ctest_name', 'runner', 'scenario', 'vehicle_profile'), 'test')
        ident = test['id']
        require(isinstance(ident, str) and IDENT.fullmatch(ident) and ident not in tests, 'invalid/duplicate test ID')
        tiers = identifiers(test['tiers'], ident + ' tiers')
        require(tiers and tiers <= TIERS, ident + ': invalid tier')
        require(type(test['timeout_s']) is int and 1 <= test['timeout_s'] <= 1800, ident + ': invalid timeout')
        require(type(test['required']) is bool, ident + ': required must be bool')
        if test['kind'] == 'ctest':
            require(set(test) & {'runner', 'scenario'} == set(), ident + ': mixed CTest/native binding')
            require(isinstance(test.get('ctest_name'), str) and IDENT.fullmatch(test['ctest_name']), ident + ': missing CTest name')
        else:
            require(test['kind'] == 'native' and 'ctest_name' not in test and
                    isinstance(test.get('runner'), str) and IDENT.fullmatch(test['runner']) and
                    isinstance(test.get('scenario'), str) and IDENT.fullmatch(test['scenario']), ident + ': invalid native binding')
            require(test['scope'] == 'native_sitl' and test.get('vehicle_profile') == 'arducopter', ident + ': unbounded native profile')
        require(test['scope'] in ('software', 'native_sitl', 'optional_software_extra'), ident + ': invalid scope')
        require(test['required'] or test['scope'] == 'optional_software_extra', ident + ': native/ordinary test cannot be silently optional')
        tests[ident] = test
    bindings = [(t['kind'], t.get('ctest_name'), t.get('runner'), t.get('scenario')) for t in tests.values()]
    require(len(bindings) == len(set(bindings)), 'two IDs alias one test/scenario')
    caps = {}
    for cap in value['capabilities']:
        fields(cap, ('id', 'owner', 'catalog_capability', 'implementation_status', 'interface', 'wire',
                     'preconditions', 'completion', 'cancellation', 'test_ids', 'source_references',
                     'qualification_ids'), label='capability')
        ident = cap['id']
        require(isinstance(ident, str) and IDENT.fullmatch(ident) and ident not in caps, 'invalid/duplicate capability ID')
        require(cap['implementation_status'] in ('implemented', 'deferred', 'unsupported'), ident + ': invalid implementation status')
        require(isinstance(cap['owner'], str) and cap['owner'], ident + ': missing owner')
        fields(cap['interface'], ('variable', 'request', 'strategy'), label=ident + ' interface')
        fields(cap['completion'], ('admission', 'ack', 'observed'), label=ident + ' completion')
        require(all(isinstance(v, str) and v for v in cap['completion'].values()), ident + ': incomplete outcome contract')
        ids = identifiers(cap['test_ids'], ident + ' tests')
        require(ids <= set(tests), ident + ': unknown test ID')
        if cap['implementation_status'] == 'implemented':
            require(ids and cap['source_references'], ident + ': implemented capability lacks tests/source')
        if cap['implementation_status'] != 'implemented':
            require(not cap['qualification_ids'], ident + ': unimplemented/rejected capability claims positive qualification')
        identifiers(cap['qualification_ids'], ident + ' qualifications')
        require(isinstance(cap['preconditions'], list) and cap['preconditions'] and
                all(isinstance(x, str) and x for x in cap['preconditions']), ident + ': missing preconditions')
        require(isinstance(cap['wire'], list), ident + ': wire list required')
        for wire in cap['wire']:
            fields(wire, ('message', 'message_id', 'command', 'command_id', 'parameters'), label=ident + ' wire')
            require(wire['message_id'] is None or type(wire['message_id']) is int and 0 <= wire['message_id'] <= 0xffffff,
                    ident + ': invalid MAVLink message ID')
            require(wire['command_id'] is None or type(wire['command_id']) is int and 0 <= wire['command_id'] <= 65535,
                    ident + ': invalid MAV_CMD ID')
        for source in cap['source_references']:
            fields(source, ('path', 'revision'), label=ident + ' source')
            relative(source['path'])
            require(REV.fullmatch(source['revision']), ident + ': source revision must be immutable')
            if source_root is not None:
                require((Path(source_root) / source['path']).is_file(), ident + ': source path missing')
        caps[ident] = cap
    qualifications = {}
    for cell in value['qualifications']:
        fields(cell, ('id', 'level', 'test_id', 'capability_ids', 'source_revision', 'firmware_revision',
                     'firmware_label', 'vehicle_profile', 'frame', 'platform', 'app_sha256', 'native_sha256',
                     'evidence', 'scope'), label='qualification')
        ident = cell['id']
        require(IDENT.fullmatch(ident) and ident not in qualifications, 'invalid/duplicate qualification ID')
        require(cell['level'] == 'native_sitl' and cell['vehicle_profile'] == 'arducopter' and cell['frame'] == 'quad',
                ident + ': qualification scope expanded')
        require(cell['test_id'] in tests and tests[cell['test_id']]['scope'] == 'native_sitl', ident + ': native evidence test missing')
        require(REV.fullmatch(cell['source_revision']) and REV.fullmatch(cell['firmware_revision']), ident + ': immutable revisions required')
        require(HEX.fullmatch(cell['app_sha256']) and HEX.fullmatch(cell['native_sha256']), ident + ': artifact identity missing')
        capids = identifiers(cell['capability_ids'], ident + ' capabilities')
        require(capids and capids <= set(caps), ident + ': unknown/empty capability set')
        for capid in capids:
            cap = caps[capid]
            require(cap['implementation_status'] == 'implemented' and ident in cap['qualification_ids'] and
                    cell['test_id'] in cap['test_ids'], ident + ': qualification not mutually bound')
        require(isinstance(cell['evidence'], dict), ident + ': evidence object required')
        if cell['evidence'].get('kind') == 'native_case':
            native_case_shape(cell)
        else:
            fields(cell['evidence'], ('path', 'sha256', 'case', 'operation_id', 'terminal'), label=ident + ' evidence')
            relative(cell['evidence']['path'])
            require(HEX.fullmatch(cell['evidence']['sha256']), ident + ': evidence hash missing')
            require(cell['evidence']['terminal'] in ('COMPLETED', 'CANCELLED'), ident + ': invalid positive-operation terminal')
        qualifications[ident] = cell
    for cap in caps.values():
        require(set(cap['qualification_ids']) <= set(qualifications), cap['id'] + ': missing qualification cell')
    require(identifiers(value['pending_qualification_tests'], 'pending tests') <= set(tests), 'unknown pending test')
    if verify_evidence:
        require(source_root is not None, 'source root required for evidence readback')
        cache = {}
        native_cache = {}
        for cell in qualifications.values():
            if cell['evidence'].get('kind') == 'native_case':
                evidence = cell['evidence']
                key = tuple((role, tuple(sorted(evidence[role].items()))) for role in ('matrix', 'readback'))
                if key not in native_cache:
                    native_cache[key] = NativeCaseEvidence(source_root, evidence['matrix'], evidence['readback'])
                native_cache[key].verify(cell, caps, tests)
                continue
            path = Path(source_root) / cell['evidence']['path']
            if str(path) not in cache:
                data = path.read_bytes()
                cache[str(path)] = (hashlib.sha256(data).hexdigest(), parse(data))
            digest, matrix = cache[str(path)]
            require(digest == cell['evidence']['sha256'], cell['id'] + ': evidence bytes changed')
            require(matrix['result'] == 'PASS' and matrix['artifact_drift'] == [] and matrix['app_sha256'] == cell['app_sha256'],
                    cell['id'] + ': matrix failed or artifact mismatch')
            selected = [case for case in matrix['cases'] if case['name'] == cell['evidence']['case']]
            require(len(selected) == 1 and selected[0]['result'] == 'PASS', cell['id'] + ': evidence case absent/duplicated/failed')
            reports = selected[0]['cleanup_reports']
            require(len(reports) == 1, cell['id'] + ': ambiguous operation evidence report')
            report = reports[0]
            require(report['result'] == 'PASS' and report['hardware'] is False and report['probe_exit'] == 0 and
                    report['revision'] == cell['source_revision'] and report['ardupilot_revision'] == cell['firmware_revision'] and
                    report['app_sha256'] == cell['app_sha256'] and report['copter_sha256'] == cell['native_sha256'],
                    cell['id'] + ': raw scope/source/artifact/result mismatch')
            require(report['ports_clear'] and all(v is True for v in report['ports_clear'].values()) and
                    report['owned_processes_remaining'] == [], cell['id'] + ': cleanup failed')
            operation = report['operations'].get(cell['evidence']['operation_id'])
            require(operation is not None and operation['terminal'] == cell['evidence']['terminal'] and
                    operation['terminal'] in operation['statuses'] and type(operation['original_expires']) in (int, float) and
                    math.isfinite(operation['original_expires']) and operation['original_expires'] > 0,
                    cell['id'] + ': operation confirmation missing/contradictory')
    return {'tests': tests, 'capabilities': caps, 'qualifications': qualifications}


def select_tests(registry, tier):
    require(tier in TIERS, 'invalid CI tier')
    validate_registry(registry)
    return [test for test in registry['tests'] if tier in test['tiers']]


def validate_inventory(registry, inventory, tier):
    index = validate_registry(registry)
    require(isinstance(inventory, dict) and isinstance(inventory.get('tests'), list), 'invalid CTest inventory')
    names = [test.get('name') for test in inventory['tests']]
    require(names and all(isinstance(n, str) and n for n in names) and len(names) == len(set(names)), 'empty/duplicate CTest inventory')
    known = {test['ctest_name'] for test in index['tests'].values() if test['kind'] == 'ctest'}
    require(set(names) <= known, 'configured CTest lacks registry entry: ' + ','.join(sorted(set(names) - known)))
    selected = select_tests(registry, tier)
    missing = [test['id'] for test in selected if test['kind'] == 'ctest' and test['required'] and test['ctest_name'] not in names]
    require(not missing, 'required CTests absent: ' + ','.join(missing))
    return {'result': 'PASS', 'tier': tier, 'registered_ctests': len(names),
            'selected': [test['id'] for test in selected],
            'optional_absent': [test['id'] for test in selected if test['kind'] == 'ctest' and not test['required'] and test['ctest_name'] not in names]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--registry', type=Path, required=True)
    parser.add_argument('--source-root', type=Path)
    parser.add_argument('--verify-evidence', action='store_true')
    parser.add_argument('--inventory', type=Path)
    parser.add_argument('--tier', choices=sorted(TIERS), default='full')
    args = parser.parse_args()
    registry = parse(args.registry.read_bytes())
    index = validate_registry(registry, args.source_root, args.verify_evidence)
    report = {'result': 'PASS', 'tests': len(index['tests']), 'capabilities': len(index['capabilities']),
              'qualification_cells': len(index['qualifications']), 'evidence_verified': args.verify_evidence,
              'scope': 'declarations and exact referenced evidence only, no test/native execution'}
    if args.inventory:
        report['inventory'] = validate_inventory(registry, parse(args.inventory.read_bytes()), args.tier)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, KeyError, TypeError, zipfile.BadZipFile, ET.ParseError) as error:
        print(json.dumps({'result': 'FAIL', 'error': str(error)}))
        raise SystemExit(1)
