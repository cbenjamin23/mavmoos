#!/usr/bin/env python3
"""Capability registry structural/evidence controls; never execute native I/O."""
import argparse
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
API = None
REGISTRY = None


class RegistryTests(unittest.TestCase):
    def reject(self, change):
        value = copy.deepcopy(REGISTRY)
        change(value)
        with self.assertRaises((ValueError, KeyError, TypeError)):
            API.validate_registry(value)

    def test_valid_declarations(self):
        result = API.validate_registry(REGISTRY, ROOT)
        self.assertTrue({'copter.arm', 'copter.hold.native', 'copter.hold.guided', 'copter.hold.helm', 'copter.mission.execute'} <= set(result['capabilities']))
        self.assertEqual(result['capabilities']['copter.mission.execute']['implementation_status'], 'implemented')

    def test_exact_archived_evidence(self):
        result = API.validate_registry(REGISTRY, ROOT, True)
        self.assertEqual(len([cell for cell in result['qualifications'].values()
                              if 'kind' not in cell['evidence']]), 11)

    def test_control_capabilities_have_required_software_witnesses(self):
        result = API.validate_registry(REGISTRY, ROOT)
        controls = {'arducopter_control', 'ardurover_control', 'arduplane_control',
                    'ardusub_control', 'px4_control', 'px4_rover_control', 'arduquadplane_control'}
        capabilities = [cap for cap in result['capabilities'].values()
                        if cap['implementation_status'] == 'implemented' and
                        (cap['owner'] in controls or cap['id'] in
                         {'copter.auto.recovery_handback', 'px4.rover.recovery.handback'})]
        self.assertEqual({cap['owner'] for cap in capabilities}, controls | {'runtime'})
        required = {ident for ident, test in result['tests'].items()
                    if test['required'] and test['scope'] == 'software' and 'software' in test['tiers']}
        for cap in capabilities:
            with self.subTest(capability=cap['id']):
                self.assertTrue(required.intersection(cap['test_ids']), 'no required software witness')

    def test_synthetic_complete_inventory(self):
        inventory = {'tests': [{'name': t['ctest_name']} for t in REGISTRY['tests'] if t['kind'] == 'ctest']}
        self.assertEqual(API.validate_inventory(REGISTRY, inventory, 'full')['result'], 'PASS')

    def test_optional_rover_auditor_absence(self):
        inventory = {'tests': [{'name': t['ctest_name']} for t in REGISTRY['tests'] if t['kind'] == 'ctest' and t['required']]}
        report = API.validate_inventory(REGISTRY, inventory, 'full')
        self.assertCountEqual(report['optional_absent'],
                              ['ctest.rover_harness_evidence', 'ctest.rover_passive_profiles', 'ctest.imu_wire_codec'])

    def test_schema(self): self.reject(lambda v: v.update(schema=2))
    def test_unknown_field(self): self.reject(lambda v: v.update(fabricated=True))
    def test_empty_test_registry(self): self.reject(lambda v: v.update(tests=[]))
    def test_duplicate_test_id(self): self.reject(lambda v: v['tests'].append(copy.deepcopy(v['tests'][0])))
    def test_aliased_binding(self):
        def change(v):
            item = copy.deepcopy(v['tests'][0]);item['id'] = 'alias';v['tests'].append(item)
        self.reject(change)
    def test_mixed_ctest_native(self): self.reject(lambda v: v['tests'][0].update(runner='fake'))
    def test_unknown_tier(self): self.reject(lambda v: v['tests'][0].update(tiers=['fake']))
    def test_empty_tier(self): self.reject(lambda v: v['tests'][0].update(tiers=[]))
    def test_duplicate_tier(self): self.reject(lambda v: v['tests'][0].update(tiers=['full', 'full']))
    def test_boolean_timeout(self): self.reject(lambda v: v['tests'][0].update(timeout_s=True))
    def test_excess_timeout(self): self.reject(lambda v: v['tests'][0].update(timeout_s=100000))
    def test_native_optional(self): self.reject(lambda v: next(t for t in v['tests'] if t['kind'] == 'native').update(required=False))
    def test_wrong_native_profile(self): self.reject(lambda v: next(t for t in v['tests'] if t['kind'] == 'native').update(vehicle_profile='physical'))
    def test_duplicate_capability(self): self.reject(lambda v: v['capabilities'].append(copy.deepcopy(v['capabilities'][0])))
    def test_unknown_test_reference(self): self.reject(lambda v: v['capabilities'][0]['test_ids'].append('missing'))
    def test_implemented_without_test(self): self.reject(lambda v: v['capabilities'][0].update(test_ids=[]))
    def test_implemented_without_source(self): self.reject(lambda v: v['capabilities'][0].update(source_references=[]))
    def test_moving_source_ref(self): self.reject(lambda v: v['capabilities'][0]['source_references'][0].update(revision='main'))
    def test_source_path_escape(self): self.reject(lambda v: v['capabilities'][0]['source_references'][0].update(path='../other'))
    def test_deferred_claims_qualification(self): self.reject(lambda v: v['capabilities'][0].update(implementation_status='deferred'))
    def test_missing_qualification_cell(self): self.reject(lambda v: v['qualifications'].pop())
    def test_mutual_capability_binding(self): self.reject(lambda v: v['qualifications'][0].update(capability_ids=['missing']))
    def test_moving_firmware_ref(self): self.reject(lambda v: v['qualifications'][0].update(firmware_revision='latest'))
    def test_qualification_scope(self): self.reject(lambda v: v['qualifications'][0].update(level='field'))
    def test_artifact_identity_missing(self): self.reject(lambda v: v['qualifications'][0].update(app_sha256='unknown'))
    def test_evidence_escape(self): self.reject(lambda v: v['qualifications'][0]['evidence'].update(path='/tmp/other'))
    def test_evidence_hash_missing(self): self.reject(lambda v: v['qualifications'][0]['evidence'].update(sha256=''))
    def test_unknown_pending_test(self): self.reject(lambda v: v['pending_qualification_tests'].append('missing'))
    def test_missing_required_inventory(self):
        with self.assertRaises(ValueError): API.validate_inventory(REGISTRY, {'tests': [{'name': 'plugin_host'}]}, 'full')
    def test_empty_inventory(self):
        with self.assertRaises(ValueError): API.validate_inventory(REGISTRY, {'tests': []}, 'software')
    def test_duplicate_inventory(self):
        with self.assertRaises(ValueError): API.validate_inventory(REGISTRY, {'tests': [{'name': 'plugin_host'}, {'name': 'plugin_host'}]}, 'software')
    def test_unknown_inventory(self):
        with self.assertRaises(ValueError): API.validate_inventory(REGISTRY, {'tests': [{'name': 'not_registered'}]}, 'software')
    def test_wrong_evidence_hash(self):
        value = copy.deepcopy(REGISTRY);value['qualifications'][0]['evidence']['sha256'] = '0' * 64
        with self.assertRaises(ValueError): API.validate_registry(value, ROOT, True)

    def test_changed_semantic_report_even_with_updated_hash(self):
        value = copy.deepcopy(REGISTRY)
        target = ROOT / value['qualifications'][0]['evidence']['path']
        original = target.read_bytes();report = API.parse(original)
        case = next(c for c in report['cases'] if c['name'] == 'copter_native_operations')
        case['cleanup_reports'][0]['ports_clear']['fabricated'] = False
        changed = json.dumps(report).encode()
        for cell in value['qualifications']:
            if cell['evidence'].get('path') == value['qualifications'][0]['evidence']['path']:
                cell['evidence']['sha256'] = hashlib.sha256(changed).hexdigest()
        real = Path.read_bytes
        def reader(path):
            return changed if path == target else real(path)
        with patch.object(Path, 'read_bytes', reader), self.assertRaises(ValueError):
            API.validate_registry(value, ROOT, True)

    def test_wrong_source_revision(self):
        value = copy.deepcopy(REGISTRY);value['qualifications'][0]['source_revision'] = '0' * 40
        with self.assertRaises(ValueError): API.validate_registry(value, ROOT, True)

    def test_wrong_operation_terminal(self):
        value = copy.deepcopy(REGISTRY);value['qualifications'][0]['evidence']['terminal'] = 'CANCELLED'
        with self.assertRaises(ValueError): API.validate_registry(value, ROOT, True)

    def test_duplicate_json(self):
        with self.assertRaises(ValueError): API.parse('{"x":1,"x":2}')
    def test_nonfinite_json(self):
        with self.assertRaises(ValueError): API.parse('{"x":NaN}')


def main():
    global API, REGISTRY, ROOT
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path, default=ROOT)
    args = parser.parse_args();ROOT = args.source_root.resolve()
    path = ROOT / 'scripts/capabilities/registry.py'
    spec = importlib.util.spec_from_file_location('_capability_registry', path)
    API = importlib.util.module_from_spec(spec);spec.loader.exec_module(API)
    REGISTRY = API.parse((ROOT / 'config/capabilities/copter.json').read_bytes())
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(RegistryTests)
    spec = importlib.util.spec_from_file_location('_additive_qualification_tests', ROOT / 'tests/copter_scenarios/QualificationTests.py')
    additive = importlib.util.module_from_spec(spec);spec.loader.exec_module(additive);additive.initialize(ROOT)
    suite.addTests(unittest.defaultTestLoader.loadTestsFromTestCase(additive.QualificationTests))
    spec = importlib.util.spec_from_file_location('_native_case_publication_tests', ROOT / 'tests/copter_scenarios/NativeCasePublicationTests.py')
    native = importlib.util.module_from_spec(spec);spec.loader.exec_module(native);native.initialize(ROOT)
    suite.addTests(unittest.defaultTestLoader.loadTestsFromTestCase(native.NativeCasePublicationTests))
    result = unittest.TextTestRunner().run(suite)
    print(json.dumps({'result': 'PASS' if result.wasSuccessful() else 'FAIL', 'tests_run': result.testsRun,
                      'scope': 'registry structure, exact archived evidence, synthetic inventory and in-memory controls only'}, indent=2))
    return int(not result.wasSuccessful())


if __name__ == '__main__':
    raise SystemExit(main())
