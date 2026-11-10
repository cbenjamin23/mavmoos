#!/usr/bin/env python3
"""Inert publication controls; synthetic receipts establish no qualification."""
import argparse
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
import zipfile

ROOT = Path(__file__).resolve().parents[2]
API = REGISTRY = INDEX = None
REV, FWREV, APP, NATIVE, PROBE = '1' * 40, '2' * 40, '3' * 64, '4' * 64, '5' * 64


def encoded(value):
    return json.dumps(value, indent=2).encode()


def digest(data):
    return hashlib.sha256(data).hexdigest()


def initialize(root):
    global ROOT, API, REGISTRY, INDEX
    ROOT = Path(root).resolve()
    spec = importlib.util.spec_from_file_location('_native_publication_registry', ROOT / 'scripts/capabilities/registry.py')
    API = importlib.util.module_from_spec(spec); spec.loader.exec_module(API)
    REGISTRY = API.parse((ROOT / 'config/capabilities/copter.json').read_bytes())
    INDEX = API.validate_registry(REGISTRY)


class Fixture:
    """Small fabricated metadata graph used only to exercise the reader."""
    def __init__(self, directory, ident='native.imu.rx_only'):
        self.root, self.ident = Path(directory), ident
        self.prefix = 'inert/ci'
        self.frozen = copy.deepcopy(REGISTRY)
        self.manifest = {'profile_id': 'inert-quad', 'vehicle_profile': 'arducopter',
                         'source': {'revision': FWREV, 'release_label': 'inert fixture'}}
        self.attestation = {'scope': 'inert metadata fixture, not a build attestation'}
        firmware = {'schema': 1, 'profile_id': 'inert-quad', 'manifest_sha256': digest(encoded(self.manifest)),
                    'attestation_sha256': digest(encoded(self.attestation)), 'source_revision': FWREV,
                    'native_path': '/inert/ardupilot/build/sitl/bin/arducopter', 'native_sha256': NATIVE,
                    'platform': {'system': 'Linux', 'machine': 'arm64'}, 'frame': 'quad',
                    'fixture': {'hardware': False}, 'build_provenance': 'inert_fixture', 'qualification': 'not_inferred'}
        selected = [t for t in self.frozen['tests'] if 'full' in t['tiers']]
        modules = {'mavmoos_' + name + '.so': digest(('inert ' + name).encode())
                   for name in ('telemetry', 'parameters', 'missions', 'arducopter_control', 'imu')}
        self.plan = [{'id': t['id'], 'kind': t['kind'], 'native': t['scope'] == 'native_sitl',
                      'expected_identity': dict(firmware, vehicle_profile='arducopter', mavmoos_revision=REV,
                                                app_sha256=APP, probe_sha256=PROBE, module_sha256=modules)} for t in selected]
        self.raw = {}
        for name in API.NATIVE_CASE_CAPS:
            self.raw[name] = {'result': 'PASS', 'hardware': False, 'probe_exit': 0, 'profile': 'arducopter',
                              'scenario': INDEX['tests'][name]['scenario'], 'source_ref': REV, 'ardupilot_ref': FWREV,
                              'app_sha256': APP, 'native_sha256': NATIVE, 'probe_sha256': PROBE,
                              'artifact_drift': [], 'owned_processes_remaining': [], 'ports_clear': {'inert': True},
                              'cleanup_proven': True, 'firmware_identity': copy.deepcopy(firmware),
                              'firmware_identity_after': copy.deepcopy(firmware)}
            if name == 'native.imu.rx_only':
                names = ('telemetry', 'imu')
            elif name in ('native.copter.route_hold_return', 'native.copter.wind_repeated_cycle'):
                names = ('telemetry', 'parameters', 'arducopter_control')
            else:
                names = ('telemetry', 'parameters', 'missions', 'arducopter_control')
            self.raw[name]['loaded_modules_sha256'] = {'mavmoos_' + n + '.so': modules['mavmoos_' + n + '.so'] for n in names}
            key, field = ('copied_firmware', 'file') if name == 'native.imu.rx_only' else ('firmware_inputs', 'path')
            self.raw[name][key] = {role: {field: role + '.json', 'sha256': firmware[role + '_sha256']}
                                   for role in ('manifest', 'attestation')}
        self.report = {'schema': 1, 'result': 'PASS', 'tier': 'full', 'hosted': False,
                       'source_revision': REV, 'source_revision_after': REV,
                       'inputs_before': {'inert': '6' * 64}, 'inputs_after': {'inert': '6' * 64},
                       'artifacts_before': {'inert': APP}, 'artifacts_after': {'inert': APP},
                       'firmware': firmware, 'firmware_after': copy.deepcopy(firmware),
                       'moos_dependency_before': {'inert': True}, 'moos_dependency_after': {'inert': True},
                       'input_drift': [], 'artifact_drift': [], 'moos_dependency_drift': False,
                       'registry_sha256': digest(encoded(self.frozen)), 'absent_optional': [],
                       'selected_ids': [t['id'] for t in selected], 'scope_excluded_ids': [],
                       'cases': [{'id': t['id'], 'result': 'PASS', 'duration_s': .1} for t in selected],
                       'native_evidence': [{'id': p['id'], 'result': 'PASS', 'cleanup_reports': [],
                                            'firmware_binding': 'selected_copter_manifest'} for p in self.plan if p['native']]}
        self.report['build'] = '/inert/build'
        artifacts = {'/inert/build/bin/iMAVLink': APP, firmware['native_path']: NATIVE}
        artifacts.update({'/inert/build/bin/' + probe: PROBE for probe in API.NATIVE_CASE_PROBES.values()})
        artifacts.update({'/inert/build/plugins/' + name: value for name, value in modules.items()})
        self.report['artifacts_before'] = artifacts; self.report['artifacts_after'] = copy.deepcopy(artifacts)
        self.suite = ET.Element('testsuite', tests=str(len(selected)), failures='0', errors='0', skipped='0')
        for test in selected:
            ET.SubElement(self.suite, 'testcase', name=test['id'], time='0.1', status='run')
        self.job = {'id': 'inert-full', 'archive': 'job.zip', 'prefix': self.prefix,
                    'recorded_output_root': '/inert/output', 'tier': 'full', 'qualification': 'current'}
        self.reviewed = {'id': 'inert-full', 'qualification': 'current', 'result': 'PASS', 'tier': 'full',
                         'cases': len(selected), 'native_cases': len(self.report['native_evidence']),
                         'cleanup_reports': len(self.report['native_evidence']), 'app_sha256': APP,
                         'native_profile_id': firmware['profile_id'], 'native_platform': firmware['platform'],
                         'semantic_replays': [{'id': name, 'member': self.raw_member(name),
                                               'proof': {'inert': True}} for name in API.NATIVE_CASE_CAPS]}
        self.matrix = {'schema': 1, 'source_revision': REV, 'archives': [],
                       'source': {'archive': 'source.zip', 'prefix': ''}, 'jobs': [self.job]}
        self.readback = {'schema': 1, 'result': 'PASS', 'git_readback': True, 'source_revision': REV,
                         'reviewer_sha256': '7' * 64, 'jobs': [self.reviewed]}
        self.cell = {'id': 'inert-native', 'level': 'native_sitl', 'test_id': ident,
                     'capability_ids': sorted(API.NATIVE_CASE_CAPS[ident]), 'source_revision': REV,
                     'firmware_revision': FWREV, 'firmware_label': 'inert fixture', 'vehicle_profile': 'arducopter',
                     'frame': 'quad', 'platform': 'linux-arm64', 'app_sha256': APP, 'native_sha256': NATIVE,
                     'scope': 'inert metadata test; no qualification claim',
                     'evidence': {'kind': 'native_case', 'matrix': {}, 'readback': {}, 'job': 'inert-full',
                                  'case': ident, 'raw': {}}}
        self.extra = {}

    def raw_member(self, ident):
        return self.prefix + '/cases/' + ident + '/evidence/result.json'

    def archive(self, filename, members):
        path = self.root / filename
        with zipfile.ZipFile(path, 'w') as zipped:
            for member, data in members.items():
                zipped.writestr(member, data)
        return {'file': filename, 'sha256': digest(path.read_bytes()),
                'members': {name: digest(data) for name, data in members.items()}}

    def publish(self):
        self.root.mkdir(parents=True, exist_ok=True)
        members = {self.prefix + '/selected-tests.json': encoded(self.plan),
                   self.prefix + '/junit.xml': ET.tostring(self.suite)}
        for name, raw in self.raw.items():
            member = self.raw_member(name); data = encoded(raw)
            members[member] = data
            parent = str(Path(member).parent)
            members[parent + '/manifest.json'] = encoded(self.manifest)
            members[parent + '/attestation.json'] = encoded(self.attestation)
            for row in self.report['native_evidence']:
                if row['id'] == name and row['cleanup_reports'] == []:
                    row['cleanup_reports'] = [{'path': '/inert/output/cases/' + name + '/evidence/result.json',
                                                'sha256': digest(data)}]
        members[self.prefix + '/result.json'] = encoded(self.report)
        members.update(self.extra)
        self.matrix['archives'] = [self.archive('source.zip', {'config/capabilities/copter.json': encoded(self.frozen)}),
                                   self.archive('job.zip', members)]
        matrix_data = encoded(self.matrix)
        (self.root / 'matrix.json').write_bytes(matrix_data)
        self.readback['matrix_sha256'] = digest(matrix_data)
        readback_data = encoded(self.readback)
        (self.root / 'readback.json').write_bytes(readback_data)
        evidence = self.cell['evidence']
        evidence['matrix'] = {'path': 'matrix.json', 'sha256': digest(matrix_data)}
        evidence['readback'] = {'path': 'readback.json', 'sha256': digest(readback_data), 'reviewer_sha256': '7' * 64}
        evidence['raw'] = {'archive': 'job.zip', 'member': self.raw_member(self.ident),
                           'sha256': digest(encoded(self.raw[self.ident]))}
        return self.cell

    def check(self):
        API.native_case_shape(self.cell)
        evidence = self.cell['evidence']
        context = API.NativeCaseEvidence(self.root, evidence['matrix'], evidence['readback'])
        context.verify(self.cell, INDEX['capabilities'], INDEX['tests'])


class NativeCasePublicationTests(unittest.TestCase):
    def control(self, change, reject=True):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory); change(fixture); fixture.publish()
            if reject:
                with self.assertRaises((ValueError, KeyError, TypeError, ET.ParseError, zipfile.BadZipFile)):
                    fixture.check()
            else:
                fixture.check()

    def test_cold_archive_metadata_is_verified_and_never_hides_a_bad_original(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Fixture(directory); fixture.publish()
            original = fixture.root / 'job.zip'
            metadata = fixture.root / 'job.metadata.zip'
            with zipfile.ZipFile(original) as src:
                members = {name: src.read(name) for name in src.namelist()}
            members.pop(fixture.raw_member('native.copter.route_hold_return'))
            fixture.archive(metadata.name, members)
            original.unlink()
            fixture.check()

            member = fixture.raw_member(fixture.ident)
            fixture.archive(metadata.name, members | {member: b'changed metadata'})
            with self.assertRaisesRegex(ValueError, 'archived metadata bytes changed'):
                fixture.check()

            fixture.archive(metadata.name, members)
            original.write_bytes(b'invalid original')
            with self.assertRaises(zipfile.BadZipFile):
                fixture.check()

    def test_five_explicit_cases_and_capability_subsets(self):
        with tempfile.TemporaryDirectory() as directory:
            for ident, caps in API.NATIVE_CASE_CAPS.items():
                for cap in caps:
                    with self.subTest(case=ident, capability=cap):
                        fixture = Fixture(Path(directory) / ident, ident); fixture.cell['capability_ids'] = [cap]
                        fixture.publish(); fixture.check()

    def test_no_terminal_operation_fields(self):
        self.control(lambda f: f.cell['evidence'].update(operation_id='invented', terminal='COMPLETED'))

    def test_unknown_kind_and_case(self):
        for key, value in (('kind', 'arbitrary'), ('case', 'native.unknown')):
            with self.subTest(key=key): self.control(lambda f: f.cell['evidence'].update({key: value}))

    def test_boolean_schema_and_probe_exit_are_not_integers(self):
        for record in ('matrix', 'readback', 'report'):
            with self.subTest(record=record): self.control(lambda f: getattr(f, record).update(schema=True))
        self.control(lambda f: f.raw[f.ident].update(probe_exit=False))

    def test_incidental_capabilities_not_promoted(self):
        self.control(lambda f: f.cell.update(capability_ids=['copter.disarm']))

    def test_nonfull_or_reference_jobs(self):
        for record, key, value in (('job', 'tier', 'smoke'), ('job', 'qualification', 'reference_only'),
                                   ('reviewed', 'tier', 'software'), ('reviewed', 'qualification', 'reference_only'),
                                   ('reviewed', 'result', 'FAIL')):
            with self.subTest(record=record, key=key): self.control(lambda f: getattr(f, record).update({key: value}))

    def test_failed_or_unbound_deep_readback(self):
        for key, value in (('result', 'FAIL'), ('git_readback', False), ('source_revision', '9' * 40),
                            ('reviewer_sha256', '9' * 64)):
            with self.subTest(key=key): self.control(lambda f: f.readback.update({key: value}))

    def test_targeted_selection_cannot_be_labelled_full(self):
        def change(f):
            f.plan[:] = [p for p in f.plan if p['id'] == f.ident]
            f.report['selected_ids'] = [f.ident]
            f.report['cases'] = [c for c in f.report['cases'] if c['id'] == f.ident]
            f.suite[:] = [r for r in f.suite if r.get('name') == f.ident]
            f.suite.set('tests', '1'); f.reviewed['cases'] = 1
        self.control(change)

    def test_missing_selected_or_executed_case(self):
        for key in ('selected_ids', 'cases', 'native_evidence'):
            with self.subTest(key=key): self.control(lambda f: f.report[key].pop())

    def test_rehashed_plan_cannot_change_native_or_ctest_roles(self):
        self.control(lambda f: f.plan[0].update(kind='native'))
        self.control(lambda f: f.plan[0].update(native=not f.plan[0]['native']))

    def test_failed_full_result(self): self.control(lambda f: f.report.update(result='FAIL'))
    def test_failed_executed_case(self): self.control(lambda f: f.report['cases'][0].update(result='FAIL'))
    def test_duplicate_executed_case(self): self.control(lambda f: f.report['cases'].append(copy.deepcopy(f.report['cases'][0])))
    def test_dependency_drift(self):
        for key, value in (('artifact_drift', ['changed']), ('input_drift', ['changed']), ('moos_dependency_drift', True),
                            ('source_revision_after', '9' * 40), ('firmware_after', {}), ('artifacts_after', {})):
            with self.subTest(key=key): self.control(lambda f: f.report.update({key: value}))

    def test_junit_failure_skip_missing_duplicate_or_counter(self):
        changes = [lambda f: ET.SubElement(f.suite[0], 'failure'), lambda f: ET.SubElement(f.suite[0], 'skipped'),
                   lambda f: f.suite.remove(f.suite[0]), lambda f: f.suite.append(copy.deepcopy(f.suite[0])),
                   lambda f: f.suite.set('tests', '999'), lambda f: f.suite[0].set('time', '9'),
                   lambda f: f.suite[0].set('status', 'notrun')]
        for number, change in enumerate(changes):
            with self.subTest(number=number): self.control(change)

    def test_source_profile_platform_and_artifact_cell_mismatch(self):
        for key, value in (('source_revision', '9' * 40), ('firmware_revision', '9' * 40),
                            ('platform', 'darwin-arm64'), ('app_sha256', '9' * 64), ('native_sha256', '9' * 64),
                            ('firmware_label', 'latest'), ('frame', 'other')):
            with self.subTest(key=key): self.control(lambda f: f.cell.update({key: value}))

    def test_raw_scope_identity_or_cleanup_mismatch(self):
        for key, value in (('result', 'FAIL'), ('hardware', True), ('probe_exit', 1), ('profile', 'ardurover'),
                            ('source_ref', '9' * 40), ('ardupilot_ref', '9' * 40), ('app_sha256', '9' * 64),
                            ('native_sha256', '9' * 64), ('probe_sha256', '9' * 64), ('scenario', 'other'),
                            ('cleanup_proven', False), ('owned_processes_remaining', None),
                            ('ports_clear', {'inert': False}), ('artifact_drift', ['changed'])):
            with self.subTest(key=key): self.control(lambda f: f.raw[f.ident].update({key: value}))

    def test_every_firmware_field_before_after_is_required(self):
        for role in ('firmware_identity', 'firmware_identity_after'):
            for key in API.FIRMWARE_FIELDS:
                with self.subTest(role=role, key=key): self.control(lambda f: f.raw[f.ident][role].pop(key))

    def test_semantic_receipt_cannot_be_empty_or_for_other_case(self):
        for change in (lambda f: f.reviewed.update(semantic_replays=[]),
                       lambda f: next(r for r in f.reviewed['semantic_replays'] if r['id'] == f.ident).update(proof={}),
                       lambda f: next(r for r in f.reviewed['semantic_replays'] if r['id'] == f.ident).update(member='other/result.json')):
            self.control(change)

    def test_current_and_frozen_test_binding_required(self):
        def change(f):
            next(t for t in f.frozen['tests'] if t['id'] == f.ident)['timeout_s'] += 1
            f.report['registry_sha256'] = digest(encoded(f.frozen))
        self.control(change)

    def test_frozen_capability_mapping_required(self):
        def change(f):
            cap = next(c for c in f.frozen['capabilities'] if c['id'] == 'observations.imu')
            cap['test_ids'].remove(f.ident)
            f.report['registry_sha256'] = digest(encoded(f.frozen))
        self.control(change)

    def test_copied_firmware_roles_and_bytes_bound(self):
        self.control(lambda f: f.raw[f.ident]['copied_firmware']['manifest'].update(sha256='9' * 64))
        self.control(lambda f: f.attestation.update(tampered=True))

    def test_frozen_app_probe_native_and_module_bytes_bound(self):
        for path in ('/inert/build/bin/iMAVLink', '/inert/build/bin/imu_moos_probe',
                     '/inert/ardupilot/build/sitl/bin/arducopter', '/inert/build/plugins/mavmoos_imu.so'):
            def change(f):
                f.report['artifacts_before'][path] = '9' * 64
                f.report['artifacts_after'][path] = '9' * 64
            with self.subTest(path=path): self.control(change)
        self.control(lambda f: f.raw[f.ident]['loaded_modules_sha256'].pop('mavmoos_imu.so'))
        self.control(lambda f: f.raw[f.ident]['loaded_modules_sha256'].update(unexpected='9' * 64))

    def test_contradictory_raw_aliases_reject(self):
        for key in ('source_ref_after', 'revision', 'ardupilot_revision', 'copter_sha256'):
            with self.subTest(key=key): self.control(lambda f: f.raw[f.ident].update({key: 'wrong'}))

    def test_archive_path_escape(self): self.control(lambda f: f.extra.update({'../escape': b'no'}))

    def test_references_are_hash_bound(self):
        for ref in ('matrix', 'readback', 'raw'):
            with self.subTest(ref=ref), tempfile.TemporaryDirectory() as directory:
                fixture = Fixture(directory); fixture.publish(); fixture.cell['evidence'][ref]['sha256'] = '9' * 64
                with self.assertRaises(ValueError): fixture.check()

    def test_no_legacy_cell_or_registry_byte_changes(self):
        current = API.parse((ROOT / 'config/capabilities/copter.json').read_bytes())
        self.assertEqual(current['qualifications'], REGISTRY['qualifications'])
        legacy = [c for c in current['qualifications'] if 'kind' not in c['evidence']]
        self.assertEqual(len(legacy), 11)
        self.assertTrue(all(set(c['evidence']) == {'path', 'sha256', 'case', 'operation_id', 'terminal'} for c in legacy))

    def test_integrated_additive_readback_retains_all_legacy_cells(self):
        with tempfile.TemporaryDirectory(dir=ROOT, prefix='build-publication-inert-') as directory:
            fixture = Fixture(directory); fixture.publish()
            cell = copy.deepcopy(fixture.cell)
            for key in ('matrix', 'readback'):
                cell['evidence'][key]['path'] = Path(directory).name + '/' + cell['evidence'][key]['path']
            registry = copy.deepcopy(REGISTRY); registry['qualifications'].append(cell)
            next(c for c in registry['capabilities'] if c['id'] == 'observations.imu')['qualification_ids'].append(cell['id'])
            checked = API.validate_registry(registry, ROOT, True)
            self.assertEqual(len(checked['qualifications']), len(REGISTRY['qualifications']) + 1)
            self.assertEqual(registry['qualifications'][:len(REGISTRY['qualifications'])], REGISTRY['qualifications'])
            spec = importlib.util.spec_from_file_location('_registry_case_with_addition', ROOT / 'tests/copter_scenarios/RegistryTests.py')
            case_module = importlib.util.module_from_spec(spec); spec.loader.exec_module(case_module)
            case_module.API, case_module.REGISTRY, case_module.ROOT = API, registry, ROOT
            case_module.RegistryTests('test_exact_archived_evidence').test_exact_archived_evidence()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path, default=ROOT)
    args = parser.parse_args(); initialize(args.source_root)
    result = unittest.TextTestRunner().run(unittest.defaultTestLoader.loadTestsFromTestCase(NativeCasePublicationTests))
    print(json.dumps({'result': 'PASS' if result.wasSuccessful() else 'FAIL', 'tests_run': result.testsRun,
                      'scope': 'inert metadata graphs and rehashed negative controls; no native/semantic qualification'}, indent=2))
    return int(not result.wasSuccessful())


if __name__ == '__main__':
    raise SystemExit(main())
