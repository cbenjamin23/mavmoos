#!/usr/bin/env python3
"""CI controller tests using real CTest and inert temporary Python projects.

No MAVLink/socket/native simulator is launched. Native report tests below are
explicit synthetic evidence-controller tests, not vehicle qualification.
"""
import copy
import contextlib
import importlib.util
import inspect
import io
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('_mavmoos_ci', ROOT / 'scripts/ci/run.py')
ci = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ci)


def entry(name='alpha', **overrides):
    value = {'id': 'ctest.' + name, 'kind': 'ctest', 'ctest_name': name,
             'required': True, 'scope': 'software', 'tiers': ['software', 'smoke', 'full'], 'timeout_s': 5}
    value.update(overrides)
    return value


def native_entry(name, tiers=('full',)):
    return entry(name, scope='native_sitl', tiers=list(tiers),
                 vehicle_profile=ci.ROVER_NATIVE_PROFILES.get(name, 'arducopter'))


def manual_entries():
    return [{'id': name, 'kind': 'native', 'runner': runner, 'scenario': scenario,
             'required': True, 'scope': 'native_sitl', 'tiers': ['full'],
             'timeout_s': {'mission_execution': 360, 'imu_observations': 120}.get(runner, 30),
             'vehicle_profile': 'arducopter'}
            for name, (runner, scenario) in ci.MANUAL_CONTRACT.items()]


class SchemaAndEvidenceTests(unittest.TestCase):
    def load(self, value):
        with tempfile.TemporaryDirectory(prefix='mavmoos-ci-schema-') as folder:
            path = Path(folder) / 'registry.json'
            path.write_text(json.dumps(value))
            return ci.read_registry(path)

    def test_valid_registry(self):
        self.assertEqual(self.load({'schema': 1, 'tests': [entry()]})['tests'][0]['id'], 'ctest.alpha')

    def test_duplicate_keys_and_nonfinite_numbers(self):
        for text in ('{"schema":1,"schema":1}', '{"a":NaN}', '{"a":Infinity}', '{"a":1e309}'):
            with self.subTest(text=text), self.assertRaises(ci.Failure):
                ci.parse(text)

    def test_invalid_registry_mutations(self):
        mutations = [lambda r: r.update(schema=2), lambda r: r.update(tests=[]),
                     lambda r: r['tests'].append(copy.deepcopy(r['tests'][0])),
                     lambda r: r['tests'][0].update(required='true'),
                     lambda r: r['tests'][0].update(tiers=[]),
                     lambda r: r['tests'][0].update(tiers=['software', 'software']),
                     lambda r: r['tests'][0].update(tiers=['unqualified']),
                     lambda r: r['tests'][0].update(tiers=['software', 'smoke']),
                     lambda r: r['tests'][0].update(scope='native_sitl', vehicle_profile='arducopter'),
                     lambda r: r['tests'][0].update(vehicle_profile='arducopter'),
                     lambda r: r['tests'][0].update(ctest_name='copter_sitl_workflow'),
                     lambda r: r['tests'][0].update(timeout_s=0),
                     lambda r: r['tests'][0].update(timeout_s=True),
                     lambda r: r['tests'][0].update(id='../escape'),
                     lambda r: r['tests'][0].update(kind='shell'),
                     lambda r: r['tests'].append(entry('beta', ctest_name='alpha')),
                     lambda r: r['tests'][0].update(kind='native', runner='arbitrary_vehicle')]
        for mutate in mutations:
            value = {'schema': 1, 'tests': [entry()]}
            mutate(value)
            with self.subTest(value=value), self.assertRaises(ci.Failure):
                self.load(value)

    def test_required_missing_or_unregistered_ctest_is_rejected(self):
        registry = {'schema': 1, 'tests': [entry()]}
        with self.assertRaises(ci.Failure):
            ci.select_tests(registry, {}, 'software')
        with self.assertRaises(ci.Failure):
            ci.select_tests(registry, {'alpha': {}, 'new_untested': {}}, 'software')

    def test_optional_absence_is_explicit(self):
        registry = {'schema': 1, 'tests': [entry(), entry('optional', required=False, scope='optional_software_extra')]}
        selected, missing = ci.select_tests(registry, {'alpha': {}}, 'software')
        self.assertEqual([r['id'] for r in selected], ['ctest.alpha'])
        self.assertEqual(missing, ['ctest.optional'])

    def test_zero_required_tier_is_rejected(self):
        with self.assertRaises(ci.Failure):
            ci.select_tests({'tests': [entry(required=False)]}, {'alpha': {}}, 'software')

    def test_full_requires_native_additions_and_existing_matrix(self):
        registry = {'tests': [entry()]}
        with self.assertRaises(ci.Failure):
            ci.select_tests(registry, {'alpha': {}}, 'full')

    def test_arbitrary_sitl_cases_cannot_replace_the_required_matrix(self):
        tests = [native_entry('arbitrary' + str(index)) for index in range(len(ci.BASELINE_NATIVE))]
        graph = {test['ctest_name']: {'properties': [{'name': 'LABELS', 'value': ['sitl']}]} for test in tests}
        tests += manual_entries()
        with self.assertRaises(ci.Failure):
            ci.select_tests({'tests': tests}, graph, 'full')

    def test_full_requires_exact_required_runner_and_case_bindings(self):
        tests = [native_entry(name) for name in ci.BASELINE_NATIVE]
        graph = {test['ctest_name']: {'properties': [{'name': 'LABELS', 'value': ['sitl']}]} for test in tests}
        tests += manual_entries()
        self.assertEqual(len(ci.select_tests({'tests': tests}, graph, 'full')[0]), len(tests))
        for mutate in (lambda v: v.pop(), lambda v: v[0].update(required=False),
                       lambda v: v[-1].update(runner='distance_flight'),
                       lambda v: v[-1].update(scenario='unrelated')):
            changed = copy.deepcopy(tests)
            mutate(changed)
            with self.assertRaises(ci.Failure):
                ci.select_tests({'tests': changed}, graph, 'full')

    def test_full_keeps_all_prior_native_cases_and_requires_both_auto_cases(self):
        tests = [native_entry(name) for name in ci.BASELINE_NATIVE] + manual_entries()
        graph = {test['ctest_name']: {'properties': [{'name': 'LABELS', 'value': ['sitl']}]}
                 for test in tests if test['kind'] == 'ctest'}
        for name in ci.NATIVE_REQUIRED:
            changed = [test for test in tests if test['id'] != name]
            with self.subTest(missing=name), self.assertRaises(ci.Failure):
                ci.select_tests({'tests': changed}, graph, 'full')
        for field, value in (('vehicle_profile', 'ardurover'), ('timeout_s', 361), ('timeout_s', 359)):
            changed = copy.deepcopy(tests)
            next(t for t in changed if t['id'] == 'native.copter.auto_complete_handback')[field] = value
            with self.subTest(field=field, value=value), self.assertRaises(ci.Failure):
                ci.select_tests({'tests': changed}, graph, 'full')

    def test_required_imu_observer_cannot_use_another_profile_or_supervisor(self):
        tests = [native_entry(name) for name in ci.BASELINE_NATIVE] + manual_entries()
        graph = {test['ctest_name']: {'properties': [{'name': 'LABELS', 'value': ['sitl']}]}
                 for test in tests if test['kind'] == 'ctest'}
        for field, value in (('vehicle_profile', 'ardurover'), ('timeout_s', 119), ('timeout_s', 121)):
            changed = copy.deepcopy(tests)
            next(t for t in changed if t['id'] == 'native.imu.rx_only')[field] = value
            with self.subTest(field=field, value=value), self.assertRaises(ci.Failure):
                ci.select_tests({'tests': changed}, graph, 'full')

    def test_actual_registry_full_selection(self):
        # The isolated CI worktree can select the integrated checkout's registry
        # explicitly; ordinary aggregate/CTest runs use their own source tree.
        path = Path(os.environ.get('MAVMOOS_CI_TEST_REGISTRY',
                                   ROOT / 'config/capabilities/copter.json'))
        registry = ci.read_registry(path)
        graph = {test['ctest_name']: {'properties': [{'name': 'LABELS',
                  'value': ['sitl'] if test['scope'] == 'native_sitl' else []}]}
                 for test in registry['tests'] if test['kind'] == 'ctest'}
        smoke, absent = ci.select_tests(registry, graph, 'smoke')
        self.assertFalse(absent)
        self.assertEqual([test['id'] for test in smoke if ci.native(test, graph)],
                         ['ctest.copter_sitl_workflow'])
        weakened = copy.deepcopy(registry)
        next(test for test in weakened['tests'] if test['id'] == 'ctest.copter_sitl_workflow')['tiers'] = ['full']
        with self.assertRaisesRegex(ci.Failure, 'smoke requires exactly'):
            ci.select_tests(weakened, graph, 'smoke')
        selected, absent = ci.select_tests(registry, graph, 'full')
        self.assertFalse(absent)
        self.assertTrue(ci.NATIVE_REQUIRED.issubset({test['id'] for test in selected}))
        self.assertTrue({'ctest.' + name for name in ci.BASELINE_NATIVE}.issubset(
            {test['id'] for test in selected}))
        distance = next(test for test in selected if test['id'] == 'native.distance_flight')
        self.assertEqual((distance['runner'], distance['scenario']),
                         ('distance_flight', 'distance'))
        auto = [test for test in selected if test.get('runner') == 'mission_execution']
        self.assertEqual({test['scenario'] for test in auto}, ci.MISSION_EXECUTION_SCENARIOS)
        self.assertTrue(all(test['vehicle_profile'] == 'arducopter' and test['timeout_s'] == 360 for test in auto))
        imu = [test for test in selected if test.get('runner') == 'imu_observations']
        self.assertEqual(len(imu), 1)
        self.assertEqual((imu[0]['scenario'], imu[0]['vehicle_profile'], imu[0]['timeout_s']),
                         ('rx_only', 'arducopter', 120))

    def test_junit_requires_actual_complete_unique_passes(self):
        with tempfile.TemporaryDirectory(prefix='mavmoos-ci-junit-') as folder:
            path = Path(folder) / 'junit.xml'
            valid = '<testsuite><testcase name="ctest.alpha" time="0.1" status="run"/></testsuite>'
            path.write_text(valid)
            self.assertEqual(ci.audit_junit(path, [entry()])[0]['result'], 'PASS')
            malformed = [valid.replace('ctest.alpha', 'other'), '<testsuite/>',
                         valid.replace('</testsuite>', '<testcase name="ctest.alpha" time="0"/></testsuite>'),
                         valid.replace('time="0.1"', 'time="nan"'),
                         valid.replace('status="run"', 'status="notrun"'),
                         valid.replace('/>', '><skipped/></testcase>'),
                         valid.replace('/>', '><failure/></testcase>'),
                         valid.replace('/>', '><error/></testcase>')]
            for value in malformed:
                path.write_text(value)
                with self.subTest(value=value), self.assertRaises(ci.Failure):
                    ci.audit_junit(path, [entry()])

    def test_synthetic_native_cleanup_gate(self):
        with tempfile.TemporaryDirectory(prefix='mavmoos-ci-cleanup-') as folder:
            base = Path(folder)
            output = base / 'result.json'
            expected = {'vehicle_profile': 'arducopter', 'profile_id': 'synthetic_quad',
                'source_revision': 'd' * 40, 'mavmoos_revision': 'c' * 40,
                'native_sha256': 'b' * 64, 'app_sha256': 'a' * 64,
                'manifest_sha256': 'f' * 64, 'driver_sha256': None,
                'module_sha256': {'mavmoos_parameters.so': 'e' * 64},
                'single_module': 'mavmoos_parameters.so', 'probe_sha256': None}
            item = {'id': 'synthetic.native', 'outputs': [str(base)], 'vehicle_profile': 'arducopter',
                    'case_directory': str(base), 'expected_identity': expected}
            (base / 'native-launch-result.json').write_text(json.dumps({'result': 'PASS',
                'id': item['id'], 'firmware': {key: expected[key] for key in
                    ('native_sha256', 'source_revision', 'profile_id', 'manifest_sha256')}}))
            valid = {'result': 'PASS', 'hardware': False, 'ports_clear': {'owned_udp': True},
                     'owned_processes_remaining': [], 'artifact_drift': [],
                     'app_sha256': expected['app_sha256'], 'native_sha256': expected['native_sha256'],
                     'ardupilot_revision': expected['source_revision'],
                     'loaded_modules_sha256': expected['module_sha256']}
            output.write_text(json.dumps(valid))
            self.assertEqual(ci.audit_native(item)['firmware_binding'], 'selected_copter_manifest')
            mutations = [lambda r: r.update(result='FAIL'), lambda r: r.update(hardware=True),
                         lambda r: r.update(ports_clear={}),
                         lambda r: r['ports_clear'].update(owned_udp=False),
                         lambda r: r.update(owned_processes_remaining=[123]),
                         lambda r: r.pop('owned_processes_remaining'),
                         lambda r: r.update(artifact_drift=['changed.so']),
                         lambda r: r.update(app_sha256='WRONG_APP'),
                         lambda r: r.update(native_sha256='WRONG_NATIVE'),
                         lambda r: r.update(copter_sha256='INCONSISTENT_ALIAS'),
                         lambda r: r.update(ardupilot_revision='WRONG_COMMIT'),
                         lambda r: r.update(source_ref='WRONG_MAVMOOS_COMMIT'),
                         lambda r: r.update(profile='ardurover'),
                         lambda r: r['loaded_modules_sha256'].update({'mavmoos_parameters.so': 'WRONG_MODULE'}),
                         lambda r: r.update(modules_sha256={'unknown.so': 'WRONG_MODULE'}),
                         lambda r: r.pop('loaded_modules_sha256')]
            for mutate in mutations:
                value = copy.deepcopy(valid)
                mutate(value)
                output.write_text(json.dumps(value))
                with self.subTest(value=value), self.assertRaises(ci.Failure):
                    ci.audit_native(item)
            expected['copter_operations_parity'] = True
            output.write_text(json.dumps(valid))
            with self.assertRaisesRegex(ci.Failure, 'stable Copter parity evidence is missing'):
                ci.audit_native(item)
            output.unlink()
            with self.assertRaises(ci.Failure):
                ci.audit_native(item)

    def test_bracket_quote_cannot_terminate_on_supplied_delimiters(self):
        value = 'path spaces;]=] ]==] ${UNCHANGED}'
        quoted = ci.bracket(value)
        self.assertTrue(quoted.startswith('[===['))
        self.assertIn(value, quoted)

    def test_selected_copter_root_changes_but_rover_keeps_its_own_binding(self):
        command = ['python', 'runner.py', '--ardupilot-root', '/old/baseline', '--output', '/fresh']
        selected, profile = ci.selected_native_command(entry('copter', vehicle_profile='arducopter'), command, Path('/selected/stable'))
        self.assertEqual(selected[3], '/selected/stable')
        self.assertEqual(profile, 'arducopter')
        self.assertEqual(command[3], '/old/baseline')
        selected, profile = ci.selected_native_command(entry('native_rover', vehicle_profile='ardurover'), command, Path('/selected/stable'))
        self.assertEqual(selected, command)
        self.assertEqual(profile, 'ardurover')
        boat_command = command + ['--profile', 'boat']
        selected, profile = ci.selected_native_command(native_entry('native_rover_boat'), boat_command, Path('/selected/stable'))
        self.assertEqual(selected, boat_command)
        self.assertEqual(profile, 'ardurover_boat')
        with self.assertRaisesRegex(ci.Failure, 'selected profile disagree'):
            ci.selected_native_command(native_entry('native_rover_boat'), command + ['--profile', 'ground'], Path('/selected/stable'))
        for malformed in (['python', 'runner.py'], command + ['--ardupilot-root', '/ambiguous']):
            with self.assertRaises(ci.Failure):
                ci.selected_native_command(entry(), malformed, Path('/selected/stable'))

    def test_stable_copter_operations_plan_selects_the_parity_variant(self):
        command = ['python', 'run_operations.py', '--ardupilot-root', '/configured']
        test = native_entry('copter_native_operations')
        stable = {'profile_id': ci.COPTER_PARITY_PROFILE, 'source_revision': ci.COPTER_PARITY_REVISION}
        selected, _ = ci.selected_native_command(test, command, Path('/selected'), stable)
        self.assertEqual(selected, ['python', 'run_operations.py', '--ardupilot-root', '/selected', '--parity'])
        baseline, _ = ci.selected_native_command(test, command, Path('/selected'), {'profile_id': 'dev'})
        self.assertNotIn('--parity', baseline)
        with self.assertRaisesRegex(ci.Failure, 'exact 4.6.3 source'):
            ci.selected_native_command(test, command, Path('/selected'), {**stable, 'source_revision': '0' * 40})
        with self.assertRaisesRegex(ci.Failure, 'selected by the verified firmware'):
            ci.selected_native_command(test, command + ['--parity'], Path('/selected'), stable)

    def test_focused_native_keeps_the_exact_manual_binding(self):
        expected = next(test for test in manual_entries() if test['id'] == 'native.copter.auto_pause_abort')
        case = [expected['id']]
        self.assertEqual(ci.select_tests({'tests': [entry(), expected]}, {}, 'focused-native', case),
                         ([expected], []))
        for bad in ({'scenario': 'auto_complete_handback'}, {'timeout_s': 30}):
            with self.subTest(bad=bad), self.assertRaises(ci.Failure):
                ci.select_tests({'tests': [{**expected, **bad}]}, {}, 'focused-native', case)
        with self.assertRaisesRegex(ci.Failure, 'only required native cases'):
            ci.select_tests({'tests': [entry()]}, {}, 'focused-native', ['ctest.alpha'])

    def test_venv_interpreter_context_is_retained(self):
        with tempfile.TemporaryDirectory(prefix='mavmoos-ci-venv-') as name:
            root = Path(name).resolve()
            subprocess.run([sys.executable, '-m', 'venv', '--without-pip', str(root / 'venv')], check=True)
            interpreter = ci.input_path('sitl_python', root / 'venv/bin/python')
            prefix = subprocess.check_output([str(interpreter), '-c', 'import sys;print(sys.prefix)'], text=True).strip()
            self.assertEqual(Path(prefix).resolve(), root / 'venv')


class NativeEvidenceFixture:
    """Shared synthetic metadata setup; fake app/module files are never executed."""
    def prepare(self, runner):
        self.temp = tempfile.TemporaryDirectory(prefix='mavmoos-ci-auto-metadata-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.source, self.build, self.output = (self.root / n for n in ('source', 'build', 'output'))
        relative = 'tests/mission_execution/run_native.py' if runner == 'mission_execution' else 'tests/imu/run_native.py'
        runner_file = self.source / relative
        runner_file.parent.mkdir(parents=True); runner_file.write_text('# inert metadata fixture; never execute\n')
        (self.build / 'bin').mkdir(parents=True); (self.build / 'plugins').mkdir()
        for name in ('iMAVLink', 'mission_execution_probe', 'imu_moos_probe'):
            (self.build / 'bin' / name).write_text('inert nonexecutable bytes: ' + name)
        for name in ci.MISSION_EXECUTION_MODULES | ci.IMU_MODULES | {'mavmoos_other_observer.so'}:
            (self.build / 'plugins' / name).write_text('inert nonlibrary bytes: ' + name)
        self.manifest, self.attestation = self.root / 'manifest.json', self.root / 'attestation.json'
        self.manifest.write_text('synthetic identity-only manifest fixture')
        self.attestation.write_text('synthetic identity-only attestation fixture')
        self.args = SimpleNamespace(source_root=self.source, build=self.build,
            ardupilot_root=self.root / 'not-launched-ardupilot', moos_root=self.root / 'not-launched-moos',
            behavior_dir=self.root / 'not-loaded-behavior', sitl_python=Path(sys.executable),
            firmware_manifest=self.manifest, firmware_attestation=self.attestation)
        self.args.moos_root.mkdir()
        for command in (['git', 'init', '-q', str(self.args.moos_root)],
                        ['git', '-C', str(self.args.moos_root), '-c', 'user.name=Inert metadata fixture',
                         '-c', 'user.email=inert@example.invalid', 'commit', '-q', '--allow-empty', '-m', 'inert source identity']):
            subprocess.run(command, check=True, capture_output=True)
        self.test = next(t for t in manual_entries() if t['runner'] == runner)
        self.firmware = {'schema': 1, 'profile_id': 'synthetic_quad_not_qualification', 'source_revision': 'd' * 40,
            'native_sha256': 'b' * 64, 'manifest_sha256': ci.sha(self.manifest),
            'attestation_sha256': ci.sha(self.attestation), 'platform': {'system': 'Synthetic', 'machine': 'none'},
            'native_path': str(self.root / 'not-launched-ardupilot/build/sitl/bin/arducopter'),
            'frame': 'synthetic', 'fixture': {'model': 'synthetic'},
            'build_provenance': 'existing_artifact_identity', 'qualification': 'not_inferred'}
        self.output.mkdir()
        _, planned = ci.plan([self.test], {}, self.args, self.output, self.firmware, 'c' * 40)
        self.item, self.expected = planned[0], planned[0]['expected_identity']
        self.evidence = Path(self.item['outputs'][0]); self.evidence.mkdir()
        self.files = {}
        copied = {}
        for role, source in (('manifest', self.manifest), ('attestation', self.attestation)):
            name = 'firmware/' + source.name
            target = self.evidence / name; target.parent.mkdir(exist_ok=True); target.write_bytes(source.read_bytes())
            self.files[name] = ci.sha(target)
            copied[role] = {'path' if runner == 'mission_execution' else 'file': name, 'sha256': self.files[name]}
        identity = {k: self.expected[k] for k in ci.FIRMWARE_IDENTITY_FIELDS}
        self.valid = {'result': 'PASS', 'profile': 'arducopter', 'scenario': self.test['scenario'],
            'hardware': False, 'probe_exit': 0, 'source_ref': 'c' * 40, 'ardupilot_ref': 'd' * 40,
            'moos_ref': self.expected['moos_revision'],
            'app_sha256': self.expected['app_sha256'], 'native_sha256': self.expected['native_sha256'],
            'probe_sha256': self.expected['probe_sha256'],
            'loaded_modules_sha256': {n: self.expected['module_sha256'][n] for n in
                                     (ci.MISSION_EXECUTION_MODULES if runner == 'mission_execution' else ci.IMU_MODULES)},
            'firmware_identity': identity, 'firmware_identity_after': copy.deepcopy(identity),
            'firmware_inputs' if runner == 'mission_execution' else 'copied_firmware': copied,
            'files': self.files, 'artifact_drift': [],
            'ports_clear': {'owned_loopback': True}, 'owned_processes_remaining': []}
        if runner == 'imu_observations':
            self.valid.update(source_ref_after='c' * 40, moos_ref_after=self.expected['moos_revision'],
                imu_module_sha256=self.expected['module_sha256']['mavmoos_imu.so'],
                observer_writes=0, arming_requested=False, rc_input_sent=False, relay_used=False,
                cleanup_proven=True, capture={'thread_retired': True, 'socket_closed': True, 'error': None},
                provider_evidence={'zero_tx_counters_observed': True, 'authority_granted': False,
                    'loaded_modules': ['imu', 'telemetry'], 'io_status_records': 2, 'runtime_status_records': 2})
        Path(self.item['case_directory'], 'native-launch-result.json').write_text(json.dumps({
            'result': 'PASS', 'id': self.item['id'], 'firmware': copy.deepcopy(identity)}))
        self.raw = self.evidence / 'result.json'

    def audit(self, value=None):
        self.raw.write_text(json.dumps(self.valid if value is None else value))
        return ci.audit_native(self.item)


class MissionExecutionEvidenceTests(NativeEvidenceFixture, unittest.TestCase):
    def setUp(self): self.prepare('mission_execution')

    def test_runner_both_scenarios_and_exact_probe_are_explicit(self):
        for scenario in ci.MISSION_EXECUTION_SCENARIOS:
            test = {**self.test, 'scenario': scenario}
            command, outputs = ci.manual_command(test, self.args, self.root / ('case-' + scenario))
            self.assertEqual(command[2], str(self.source / 'tests/mission_execution/run_native.py'))
            for option, expected in (('--scenario', scenario), ('--probe', str(self.build / 'bin/mission_execution_probe')),
                    ('--expected-app-sha256', ci.sha(self.build / 'bin/iMAVLink')),
                    ('--expected-probe-sha256', ci.sha(self.build / 'bin/mission_execution_probe')),
                    ('--firmware-manifest', str(self.manifest)), ('--firmware-attestation', str(self.attestation))):
                self.assertEqual(command[command.index(option) + 1], expected)
            self.assertEqual(outputs, [str(self.root / ('case-' + scenario) / 'evidence')])
        for changed in ({'runner': 'unknown'}, {'scenario': 'unrecognized'}):
            with self.subTest(changed=changed), self.assertRaises(ci.Failure):
                ci.manual_command({**self.test, **changed}, self.args, self.root / 'never-created')

    def test_plan_pins_probe_modules_contract_and_original_supervisor(self):
        self.assertEqual(self.expected['probe_sha256'], ci.sha(self.build / 'bin/mission_execution_probe'))
        self.assertEqual(self.expected['raw_contract'], ci.MISSION_EXECUTION_CONTRACT)
        self.assertEqual(self.expected['scenario'], self.test['scenario'])
        self.assertTrue(ci.MISSION_EXECUTION_MODULES < set(self.expected['module_sha256']))
        launch = json.loads(Path(self.item['case_directory'], 'launch.json').read_bytes())
        self.assertEqual(launch['timeout_s'], 360)
        self.assertIn('--probe', launch['command'])
        self.assertIn('435', (self.output / 'plan/CTestTestfile.cmake').read_text())

    def test_complete_identity_and_cleanup_schema_is_accepted_only_as_synthetic_metadata(self):
        value = self.audit()
        self.assertEqual(value['firmware_binding'], 'selected_copter_manifest')
        self.assertEqual(len(value['cleanup_reports']), 1)

    def test_missing_or_mismatched_auto_identity_is_rejected(self):
        mutations = [lambda r: r.pop('probe_sha256'), lambda r: r.update(probe_sha256='0' * 64),
            lambda r: r.pop('source_ref'), lambda r: r.pop('ardupilot_ref'),
            lambda r: r.update(source_ref='e' * 40), lambda r: r.update(ardupilot_ref='e' * 40),
            lambda r: r.pop('profile'), lambda r: r.update(profile='ardurover'),
            lambda r: r.update(scenario='auto_pause_abort'), lambda r: r.pop('scenario'),
            lambda r: r.pop('hardware'), lambda r: r.update(hardware=True),
            lambda r: r.pop('probe_exit'), lambda r: r.update(probe_exit=False), lambda r: r.update(probe_exit=1),
            lambda r: r.pop('firmware_identity'), lambda r: r.pop('firmware_identity_after'),
            lambda r: r['firmware_identity_after'].update(native_sha256='0' * 64),
            lambda r: r['firmware_identity_after'].pop('platform'),
            lambda r: r['firmware_identity'].update(platform={'system': 'wrong', 'machine': 'none'}),
            lambda r: r['firmware_identity_after'].update(manifest_sha256='0' * 64),
            lambda r: r['firmware_identity_after'].update(attestation_sha256='0' * 64),
            lambda r: r['firmware_identity_after'].update(frame='other'),
            lambda r: r['firmware_identity_after'].update(qualification='supported'),
            lambda r: r.pop('owned_processes_remaining'), lambda r: r.update(owned_processes_remaining=None),
            lambda r: r.pop('artifact_drift'), lambda r: r.update(artifact_drift=None)]
        for mutate in mutations:
            value = copy.deepcopy(self.valid); mutate(value)
            with self.subTest(value=value), self.assertRaises(ci.Failure):
                self.audit(value)

    def test_loaded_four_modules_cannot_be_replaced_by_a_partial_or_delivered_alias(self):
        for mutate in (lambda r: r['loaded_modules_sha256'].pop('mavmoos_missions.so'),
                lambda r: r.pop('loaded_modules_sha256'),
                lambda r: r['loaded_modules_sha256'].update(mavmoos_other_observer=self.expected['module_sha256']['mavmoos_other_observer.so']),
                lambda r: r['loaded_modules_sha256'].update({'mavmoos_other_observer.so': self.expected['module_sha256']['mavmoos_other_observer.so']})):
            value = copy.deepcopy(self.valid); value['modules_sha256'] = copy.deepcopy(self.expected['module_sha256']); mutate(value)
            with self.subTest(value=value), self.assertRaises(ci.Failure):
                self.audit(value)

    def test_copied_firmware_roles_hashes_and_file_inventory_are_bound(self):
        for mutate in (lambda r: r.pop('firmware_inputs'), lambda r: r.pop('files'),
                lambda r: r.update(files=None), lambda r: r.update(files=[]),
                lambda r: r['firmware_inputs'].pop('attestation'),
                lambda r: r['firmware_inputs']['attestation'].update(sha256='0' * 64),
                lambda r: r['firmware_inputs']['manifest'].update(path='missing.json'),
                lambda r: r['files'].pop('firmware/manifest.json')):
            value = copy.deepcopy(self.valid); mutate(value)
            with self.subTest(value=value), self.assertRaises(ci.Failure):
                self.audit(value)
        (self.evidence / 'firmware/manifest.json').write_text('changed after report')
        with self.assertRaises(ci.Failure): self.audit()

    def test_top_level_result_and_planned_contract_cannot_be_downgraded(self):
        for field in ('raw_contract', 'scenario'):
            original = self.expected.pop(field)
            try:
                with self.subTest(field=field), self.assertRaises(ci.Failure): self.audit()
            finally: self.expected[field] = original
        self.item['vehicle_profile'] = 'ardurover'
        with self.assertRaises(ci.Failure): self.audit()
        self.item['vehicle_profile'] = 'arducopter'
        for field in ('mavmoos_revision', 'probe_sha256', 'platform'):
            original = self.expected[field]; self.expected[field] = None
            try:
                value = copy.deepcopy(self.valid)
                value.pop('source_ref' if field == 'mavmoos_revision' else
                          'probe_sha256' if field == 'probe_sha256' else 'firmware_identity_after')
                with self.subTest(planned_missing=field), self.assertRaises(ci.Failure): self.audit(value)
            finally: self.expected[field] = original
        self.audit(); nested = self.evidence / 'nested'; nested.mkdir()
        (nested / 'result.json').write_text(json.dumps(self.valid))
        with self.assertRaises(ci.Failure): ci.audit_native(self.item)
        self.raw.unlink()
        with self.assertRaises(ci.Failure): ci.audit_native(self.item)
        (nested / 'result.json').unlink(); nested.rmdir()
        value = copy.deepcopy(self.valid); value.pop('ports_clear')
        with self.assertRaises(ci.Failure): self.audit(value)


class ImuObservationEvidenceTests(NativeEvidenceFixture, unittest.TestCase):
    def setUp(self): self.prepare('imu_observations')

    def test_imu_command_pins_direct_receiver_probe_without_behavior_argument(self):
        command, _ = ci.manual_command(self.test, self.args, self.root / 'new-case')
        self.assertEqual(command[2], str(self.source / 'tests/imu/run_native.py'))
        self.assertEqual(command[command.index('--scenario') + 1], 'rx_only')
        self.assertEqual(command[command.index('--probe') + 1], str(self.build / 'bin/imu_moos_probe'))
        self.assertEqual(command[command.index('--expected-probe-sha256') + 1], ci.sha(self.build / 'bin/imu_moos_probe'))
        self.assertNotIn('--behavior-dir', command)
        self.assertEqual(self.expected['raw_contract'], ci.IMU_CONTRACT)
        self.assertEqual(json.loads(Path(self.item['case_directory'], 'launch.json').read_bytes())['timeout_s'], 120)

    def test_observed_counter_proof_accepts_without_invented_primary_wire_capture(self):
        self.assertNotIn('app_wire_tx_count', self.valid)
        self.assertEqual(self.audit()['firmware_binding'], 'selected_copter_manifest')
        self.assertEqual(set(self.valid['loaded_modules_sha256']), {'mavmoos_imu.so', 'mavmoos_telemetry.so'})

    def test_missing_or_effectful_observation_counter_and_authority_proof_rejects(self):
        mutations = [lambda r: r.pop('provider_evidence'),
            lambda r: r['provider_evidence'].update(zero_tx_counters_observed=False),
            lambda r: r['provider_evidence'].update(authority_granted=True),
            lambda r: r['provider_evidence'].update(io_status_records=0),
            lambda r: r['provider_evidence'].update(runtime_status_records=False),
            lambda r: r['provider_evidence']['loaded_modules'].append('arducopter_control'),
            lambda r: r.update(observer_writes=1), lambda r: r.update(observer_writes=False),
            lambda r: r.update(arming_requested=True), lambda r: r.update(rc_input_sent=True),
            lambda r: r.update(relay_used=True)]
        for mutate in mutations:
            value = copy.deepcopy(self.valid); mutate(value)
            with self.subTest(value=value), self.assertRaises(ci.Failure): self.audit(value)

    def test_capture_or_cleanup_uncertainty_never_means_absence(self):
        for mutate in (lambda r: r.pop('cleanup_proven'), lambda r: r.update(cleanup_proven=False),
                lambda r: r.update(owned_processes_remaining=None), lambda r: r.pop('capture'),
                lambda r: r['capture'].update(error='receiver failed'), lambda r: r['capture'].pop('error'),
                lambda r: r['capture'].update(socket_closed=False), lambda r: r['capture'].update(thread_retired=False)):
            value = copy.deepcopy(self.valid); mutate(value)
            with self.subTest(value=value), self.assertRaises(ci.Failure): self.audit(value)

    def test_imu_source_module_firmware_copy_and_contract_are_exact(self):
        for mutate in (lambda r: r.update(source_ref_after='e' * 40), lambda r: r.update(moos_ref_after='e' * 40),
                lambda r: r.update(moos_ref='e' * 40), lambda r: r.update(imu_module_sha256='0' * 64),
                lambda r: r.pop('copied_firmware'),
                lambda r: r['copied_firmware']['manifest'].update(file='missing.json'),
                lambda r: r['firmware_identity_after'].update(attestation_sha256='0' * 64),
                lambda r: r['loaded_modules_sha256'].pop('mavmoos_imu.so')):
            value = copy.deepcopy(self.valid); mutate(value)
            with self.subTest(value=value), self.assertRaises(ci.Failure): self.audit(value)
        original = self.expected['raw_contract']; self.expected['raw_contract'] = ci.MISSION_EXECUTION_CONTRACT
        try:
            with self.assertRaises(ci.Failure): self.audit()
        finally: self.expected['raw_contract'] = original


PAYLOAD = '''import json,os,sys,time
from pathlib import Path
mode=sys.argv[1]
if mode=='fail':sys.exit(7)
if mode=='skip':sys.exit(77)
if mode=='timeout':time.sleep(10)
if mode=='signal_first':
 import signal,subprocess
 signal.signal(signal.SIGTERM,signal.SIG_IGN)
 path=Path(sys.argv[2]);path.parent.mkdir(parents=True,exist_ok=True)
 code='import os,signal,sys,time;from pathlib import Path;signal.signal(signal.SIGTERM,signal.SIG_IGN);Path(sys.argv[1]).write_text(str(os.getpid()));time.sleep(15) # mavmoos_inert_sigterm_child'
 subprocess.Popen([sys.executable,'-c',code,str(path.with_suffix('.pid'))],start_new_session=True)
 time.sleep(15)
if mode in ('source_drift','example_drift'):Path(sys.argv[3]).write_text('changed')
if mode=='artifact_drift':Path(sys.argv[3]).write_text('changed')
if mode=='ok' or mode.endswith('_drift'):
 path=Path(sys.argv[2]);path.parent.mkdir(parents=True,exist_ok=True)
 path.write_text(json.dumps({'result':'PASS','cwd':str(Path.cwd()),'value':os.environ.get('CI_FIXTURE_VALUE')}))
print('inert CI fixture completed')
'''


class MoosDependencyTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='mavmoos-ci-moos-pin-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.moos, self.behavior = self.root / 'stock', self.root / 'behavior'
        (self.moos / 'bin').mkdir(parents=True)
        self.behavior.mkdir()
        self.git('init', '-q')
        self.git('-c', 'user.name=Inert CI fixture', '-c', 'user.email=inert@example.invalid',
                 'commit', '-q', '--allow-empty', '-m', 'exact stock source fixture')
        self.revision = ci.source_revision(self.moos)
        for name in ('pAntler', 'pHelmIvP', 'MOOSDB', 'nsplug'):
            (self.moos / 'bin' / name).write_text('inert bytes; never executed: ' + name)
        self.library = self.behavior / 'libBHV_ConstantAltitude.so'
        self.library.write_text('inert behavior bytes; never loaded')
        self.manifest = self.root / 'manifest.json'
        self.manifest.write_text(json.dumps({'dependencies': {'moos_revision': self.revision}}))
        source, build = self.root / 'source', self.root / 'build'
        source.mkdir(); build.mkdir()
        (source / 'CMakeLists.txt').write_text('# inert negative preflight fixture\n')
        self.cache = ('CMAKE_HOME_DIRECTORY:INTERNAL=' + str(source) + '\n' +
                      'MOOS_IVP_ROOT:PATH=' + str(self.moos) + '\n' +
                      'MAVMOOS_BEHAVIOR_DIR:PATH=' + str(self.behavior) + '\n').encode()
        (build / 'CMakeCache.txt').write_bytes(self.cache)
        registry = self.root / 'registry.json'
        registry.write_text(json.dumps({'schema': 1, 'tests': [entry()]}))
        self.args = SimpleNamespace(moos_root=self.moos, behavior_dir=self.behavior,
            firmware_manifest=self.manifest, firmware_attestation=None,
            source_root=source, build=build, registry=registry, tier='smoke',
            output=self.root / 'output', jobs=1, build_timeout=1,
            cmake='/this-inert-build-command-must-never-be-called')

    def git(self, *args):
        subprocess.run(['git', '-C', str(self.moos), *args], check=True,
                       capture_output=True, text=True)

    def test_matching_source_pin_and_runtime_bytes_are_bound(self):
        report = ci.moos_dependency(self.args, self.cache)
        self.assertEqual(report['source_revision'], self.revision)
        self.assertEqual(report['expected_revision'], self.revision)
        self.assertEqual(len(report['executables']), 4)
        self.assertEqual(report['behavior_files'][str(self.library)]['sha256'], ci.sha(self.library))

    def test_changed_stock_head_fails_before_any_build_or_plan(self):
        self.git('-c', 'user.name=Inert CI fixture', '-c', 'user.email=inert@example.invalid',
                 'commit', '-q', '--allow-empty', '-m', 'external checkout revision change')
        with contextlib.redirect_stdout(io.StringIO()):
            result = ci.run(self.args)
        self.assertEqual(result, 1)
        report = json.loads((self.args.output / 'result.json').read_text())
        self.assertIn('MOOS source revision differs', report['error'])
        self.assertEqual(report['commands'], [])
        self.assertFalse((self.args.output / 'plan').exists())
        self.assertFalse((self.args.output / 'junit.xml').exists())

    def test_configured_stock_and_behavior_roots_must_match_cli(self):
        for selected in (self.moos, self.behavior):
            changed = self.cache.replace(str(selected).encode(), b'/wrong/configured/path')
            with self.subTest(root=selected), self.assertRaises(ci.Failure):
                ci.moos_dependency(self.args, changed)

    def test_selected_software_and_native_commands_cannot_keep_other_roots(self):
        selected = [entry()]
        for option, path in (('--moos-root', self.moos), ('--moos-bin', self.moos / 'bin'),
                             ('--behavior-dir', self.behavior)):
            graph = {'alpha': {'command': ['inert.py', option, str(path)]}}
            ci.verify_selected_moos_paths(selected, graph, self.args)
            for command in (['inert.py', option, '/wrong/root'],
                            ['inert.py', option, str(path), option, str(path)], ['inert.py', option]):
                with self.subTest(command=command), self.assertRaises(ci.Failure):
                    ci.verify_selected_moos_paths(selected, {'alpha': {'command': command}}, self.args)

    def test_runtime_changes_additions_and_symlink_rebinding_change_identity(self):
        before = ci.moos_dependency(self.args, self.cache)
        tool = self.moos / 'bin/pHelmIvP'
        initial = tool.read_bytes(); tool.write_bytes(b'rebuilt executable')
        self.assertNotEqual(ci.moos_dependency(self.args, self.cache), before)
        tool.write_bytes(initial)
        self.library.write_bytes(b'rebuilt behavior')
        self.assertNotEqual(ci.moos_dependency(self.args, self.cache), before)
        self.library.write_text('inert behavior bytes; never loaded')
        added = self.behavior / 'additional.so'; added.write_text('added behavior')
        self.assertNotEqual(ci.moos_dependency(self.args, self.cache), before)
        added.unlink()
        first, second = self.root / 'first', self.root / 'second'
        first.write_bytes(initial); second.write_bytes(initial)
        tool.unlink(); tool.symlink_to(first)
        linked = ci.moos_dependency(self.args, self.cache)
        tool.unlink(); tool.symlink_to(second)
        self.assertNotEqual(ci.moos_dependency(self.args, self.cache), linked)

    def test_missing_runtime_file_or_empty_behavior_directory_fails(self):
        self.library.unlink()
        with self.assertRaises(ci.Failure):
            ci.moos_dependency(self.args, self.cache)
        self.library.write_text('inert')
        (self.moos / 'bin/nsplug').unlink()
        with self.assertRaises(ci.Failure):
            ci.moos_dependency(self.args, self.cache)


class RealCTestTests(unittest.TestCase):
    def setUp(self):
        for command in ('cmake', 'ctest', 'ninja'):
            self.assertIsNotNone(shutil.which(command), 'required test tool absent: ' + command)
        self.temp = tempfile.TemporaryDirectory(prefix='mavmoos-ci-inert project-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.source, self.build = self.root / 'source', self.root / 'build'
        (self.source / 'scripts').mkdir(parents=True)
        (self.source / 'scripts/payload.py').write_text(PAYLOAD)
        (self.source / 'scripts/protected.txt').write_text('original')
        (self.source / 'examples').mkdir()
        (self.source / 'examples/provider.cpp').write_text('original external provider source')
        self.registry = self.root / 'registry.json'
        self.counter = 0

    def configure(self, cases, build_fails=False):
        lines = ['cmake_minimum_required(VERSION 3.24)', 'project(CI_Inert LANGUAGES NONE)', 'enable_testing()']
        for name, mode, props in cases:
            arg = (self.source / 'scripts/protected.txt' if mode == 'source_drift' else
                   self.source / 'examples/provider.cpp' if mode == 'example_drift' else self.build / 'bin/inert')
            command = [name, sys.executable, self.source / 'scripts/payload.py', mode,
                       self.build / 'evidence' / (name + '.json'), arg]
            lines.append('add_test(' + ' '.join(ci.bracket(v) for v in command) + ')')
            if props:
                values = []
                for key, value in props.items():
                    values += [ci.bracket(key), ci.bracket(value)]
                lines.append('set_tests_properties(' + ci.bracket(name) + ' PROPERTIES ' + ' '.join(values) + ')')
        if build_fails:
            lines.append('add_custom_target(inert_failure ALL COMMAND ' + ci.bracket(sys.executable) +
                         ' -c ' + ci.bracket('import sys;sys.exit(9)') + ')')
        (self.source / 'CMakeLists.txt').write_text('\n'.join(lines) + '\n')
        outcome = subprocess.run(['cmake', '-G', 'Ninja', '-S', str(self.source), '-B', str(self.build)],
                                 capture_output=True, text=True)
        self.assertEqual(outcome.returncode, 0, outcome.stdout + outcome.stderr)
        (self.build / 'bin').mkdir(exist_ok=True)
        (self.build / 'bin/inert').write_text('frozen inert artifact')
        (self.build / 'Testing/Temporary').mkdir(parents=True, exist_ok=True)
        self.historical = self.build / 'Testing/Temporary/LastTest.log'
        self.historical.write_bytes(b'preserved historical detail\n')

    def execute(self, tests, tier='software', expected=0, cases=()):
        self.counter += 1
        out = self.root / ('run-' + str(self.counter))
        self.registry.write_text(json.dumps({'schema': 1, 'tests': tests}))
        command = [sys.executable, '-B', str(ROOT / 'scripts/ci/run.py'),
            '--source-root', str(self.source), '--build', str(self.build), '--tier', tier,
            '--registry', str(self.registry), '--output', str(out), '--jobs', '1',
            '--max-time', '20', '--build-timeout', '15']
        for case_id in cases:
            command.extend(('--case', case_id))
        outcome = subprocess.run(command, capture_output=True, text=True)
        self.assertEqual(outcome.returncode, expected, outcome.stdout + outcome.stderr)
        self.assertTrue((out / 'result.json').is_file(), outcome.stdout + outcome.stderr)
        value = json.loads((out / 'result.json').read_text())
        self.assertEqual(value['result'], 'PASS' if expected == 0 else 'FAIL')
        self.assertEqual(self.historical.read_bytes(), b'preserved historical detail\n')
        return value, out

    def test_real_ctest_pass_expected_negative_environment_and_fresh_replay(self):
        self.configure([('alpha', 'ok', {'ENVIRONMENT': 'CI_FIXTURE_VALUE=spaces and brackets ]=]'}),
                        ('beta', 'fail', {'WILL_FAIL': 'TRUE'})])
        entries = [entry(), entry('beta')]
        first, path = self.execute(entries)
        self.assertEqual(len(first['cases']), 2)
        artifact = next((path / 'cases/ctest.alpha/artifacts').rglob('alpha.json'))
        observed = json.loads(artifact.read_text())
        self.assertEqual(observed['cwd'], str(self.build.resolve()))
        self.assertEqual(observed['value'], 'spaces and brackets ]=]')
        self.assertFalse((self.build / 'evidence/alpha.json').exists())
        second, path2 = self.execute(entries)
        self.assertEqual(first['registry_sha256'], second['registry_sha256'])
        self.assertNotEqual(path, path2)

    def test_unregistered_configured_case_fails_before_test_execution(self):
        self.configure([('alpha', 'ok', {}), ('new_case', 'ok', {})])
        report, output = self.execute([entry()], expected=1)
        self.assertIn('unregistered', report['error'])
        self.assertFalse((output / 'junit.xml').exists())

    def test_required_case_absent_fails_before_test_execution(self):
        self.configure([('alpha', 'ok', {})])
        report, _ = self.execute([entry(), entry('missing')], expected=1)
        self.assertIn('not configured', report['error'])

    def test_ctest_failure_is_not_a_pass(self):
        self.configure([('alpha', 'fail', {})])
        report, out = self.execute([entry()], expected=1)
        self.assertIn('failed', report['error'])
        self.assertTrue((out / 'ctest.log').is_file())

    def test_ctest_skip_cannot_make_required_case_pass(self):
        self.configure([('alpha', 'skip', {'SKIP_RETURN_CODE': '77'})])
        report, out = self.execute([entry()], expected=1)
        self.assertIn('skipped', report['error'])
        self.assertTrue((out / 'junit.xml').is_file())

    def test_disabled_required_case_is_rejected(self):
        self.configure([('alpha', 'ok', {'DISABLED': 'TRUE'})])
        report, _ = self.execute([entry()], expected=1)
        self.assertIn('skipped', report['error'])

    def test_required_native_prerequisites_fail_before_launch(self):
        self.configure([('copter_sitl_workflow', 'ok', {'LABELS': 'sitl'})])
        report, out = self.execute([native_entry('copter_sitl_workflow', ('smoke', 'full'))],
                                   tier='smoke', expected=1)
        self.assertIn('requires --firmware-manifest', report['error'])
        self.assertFalse((out / 'junit.xml').exists())
        report, out = self.execute([native_entry('copter_sitl_workflow')], tier='focused-native',
                                   cases=['ctest.copter_sitl_workflow'], expected=1)
        self.assertIn('requires --firmware-manifest', report['error'])
        self.assertEqual(report['requested_ids'], ['ctest.copter_sitl_workflow'])
        self.assertFalse((out / 'junit.xml').exists())

    def test_software_tier_does_not_claim_full_or_run_native(self):
        self.configure([('alpha', 'ok', {}), ('native_deferred', 'fail', {'LABELS': 'sitl'})])
        report, _ = self.execute([entry(), native_entry('native_deferred')])
        self.assertEqual(report['tier'], 'software')
        self.assertEqual(report['scope_excluded_ids'], ['ctest.native_deferred'])

    def test_focused_executes_only_exact_requested_software_cases(self):
        self.configure([('alpha', 'ok', {}), ('alpha_extra', 'fail', {}),
                        ('optional', 'ok', {}), ('native_deferred', 'fail', {'LABELS': 'sitl'})])
        entries = [entry(), entry('alpha_extra'), native_entry('native_deferred'),
                   entry('optional', required=False, scope='optional_software_extra')]
        cases = ['ctest.alpha', 'ctest.optional']
        report, _ = self.execute(entries, tier='focused', cases=cases)
        self.assertEqual(report['tier'], 'focused')
        self.assertEqual(report['requested_ids'], cases)
        self.assertEqual(report['selected_ids'], cases)
        self.assertEqual([case['id'] for case in report['cases']], cases)
        self.assertEqual(report['scope_excluded_ids'], ['ctest.alpha_extra', 'ctest.native_deferred'])
        self.assertEqual(report['native_evidence'], [])

    def test_focused_invalid_case_lists_fail_before_ctest_execution(self):
        self.configure([('alpha', 'ok', {}), ('native_deferred', 'fail', {'LABELS': 'sitl'})])
        entries = [entry(), native_entry('native_deferred'), entry('absent')]
        for cases, error in ((['ctest.alpha', 'ctest.alpha'], 'unique --case IDs'),
                             (['ctest.alph'], 'unknown case ID'),
                             (['ctest.native_deferred'], 'only software cases'),
                             (['ctest.absent'], 'not configured')):
            with self.subTest(cases=cases):
                report, out = self.execute(entries, tier='focused', cases=cases, expected=1)
                self.assertIn(error, report['error'])
                self.assertFalse((out / 'junit.xml').exists())

    def test_native_label_drift_rejects_even_an_excluded_case(self):
        self.configure([('alpha', 'ok', {}), ('native_deferred', 'fail', {})])
        report, out = self.execute([entry(), native_entry('native_deferred')], expected=1)
        self.assertIn('scope disagrees with CTest sitl label', report['error'])
        self.assertFalse((out / 'junit.xml').exists())

    def test_focused_does_not_drop_ctest_dependencies(self):
        self.configure([('alpha', 'ok', {'DEPENDS': 'beta'}), ('beta', 'fail', {})])
        report, out = self.execute([entry(), entry('beta')], tier='focused',
                                   cases=['ctest.alpha'], expected=1)
        self.assertIn('unsupported dependency/fixture graph', report['error'])
        self.assertFalse((out / 'junit.xml').exists())

    def test_original_timeout_is_enforced_by_private_plan(self):
        self.configure([('alpha', 'timeout', {})])
        report, _ = self.execute([entry(timeout_s=.2)], expected=1)
        self.assertIn('failed', report['error'])

    def test_build_failure_prevents_case_execution(self):
        self.configure([('alpha', 'ok', {})], build_fails=True)
        report, out = self.execute([entry()], expected=1)
        self.assertIn('build failed', report['error'])
        self.assertFalse((out / 'junit.xml').exists())

    def test_source_drift_invalidates_even_successful_test(self):
        self.configure([('alpha', 'source_drift', {})])
        report, _ = self.execute([entry()], expected=1)
        self.assertIn(str((self.source / 'scripts/protected.txt').resolve()), report['input_drift'])

    def test_external_example_drift_invalidates_even_successful_test(self):
        self.configure([('alpha', 'example_drift', {})])
        report, _ = self.execute([entry()], expected=1)
        self.assertIn(str((self.source / 'examples/provider.cpp').resolve()), report['input_drift'])

    def test_postflight_drift_does_not_replace_the_original_case_failure(self):
        self.configure([('alpha', 'source_drift', {'WILL_FAIL': 'TRUE'})])
        report, _ = self.execute([entry()], expected=1)
        self.assertIn('CTest plan failed', report['error'])
        self.assertIn('protected source/input/artifact changed', report['postflight_error'])
        self.assertTrue(report['input_drift'])

    def test_artifact_drift_invalidates_even_successful_test(self):
        self.configure([('alpha', 'artifact_drift', {})])
        report, _ = self.execute([entry()], expected=1)
        self.assertIn(str((self.build / 'bin/inert').resolve()), report['artifact_drift'])

    def test_existing_output_is_never_replaced(self):
        self.configure([('alpha', 'ok', {})])
        _, out = self.execute([entry()])
        before = (out / 'result.json').read_bytes()
        result = subprocess.run([sys.executable, '-B', str(ROOT / 'scripts/ci/run.py'),
            '--source-root', str(self.source), '--build', str(self.build), '--tier', 'software',
            '--registry', str(self.registry), '--output', str(out)], capture_output=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual((out / 'result.json').read_bytes(), before)

    def test_build_regeneration_precedes_inventory(self):
        self.configure([('alpha', 'ok', {})])
        path = self.source / 'CMakeLists.txt'
        path.write_text(path.read_text() + 'add_test(' + ' '.join(ci.bracket(v) for v in
            ['beta', sys.executable, self.source / 'scripts/payload.py', 'ok',
             self.build / 'evidence/beta.json']) + ')\n')
        report, _ = self.execute([entry(), entry('beta')])
        self.assertEqual(len(report['cases']), 2)

    def test_actual_sigterm_stops_the_plan_without_starting_second_case(self):
        self.configure([('alpha', 'signal_first', {}), ('beta', 'ok', {})])
        self.registry.write_text(json.dumps({'schema': 1,
            'tests': [entry(timeout_s=20), entry('beta')]}))
        output = self.root / 'cancelled-plan'
        argv = [sys.executable, '-B', str(ROOT / 'scripts/ci/run.py'),
                '--source-root', str(self.source), '--build', str(self.build),
                '--tier', 'software', '--registry', str(self.registry),
                '--output', str(output), '--max-time', '30']
        with (self.root / 'owner.log').open('wb') as log:
            process = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            owner = ci.ProcessOwner(process)
            try:
                deadline, pidfiles = time.monotonic() + 8, []
                while time.monotonic() < deadline and process.poll() is None:
                    owner.capture()
                    pidfiles = list(output.rglob('alpha.pid'))
                    if pidfiles:
                        break
                    time.sleep(.05)
                self.assertTrue(pidfiles, (self.root / 'owner.log').read_text())
                time.sleep(.3)  # Let the controller record the separately grouped child.
                owner.capture()
                os.kill(process.pid, signal.SIGTERM)
                self.assertEqual(process.wait(timeout=12), 128 + signal.SIGTERM)
                report = json.loads((output / 'result.json').read_text())
                self.assertEqual(report['result'], 'FAIL')
                self.assertTrue(report['interrupted'])
                self.assertFalse(list(output.rglob('beta.json')))
                evidence = json.loads((output / 'ctest.log.processes.json').read_text())
                self.assertEqual(evidence['remaining'], [])
                self.assertTrue(any('mavmoos_inert_sigterm_child' in row['command']
                                    for row in evidence['observed']))
                self.assertEqual(owner.living(), [])
            finally:
                owner.stop()  # Exact observed inert identities only, including on failed assertions.


class ProcessOwnershipTests(unittest.TestCase):
    def deadline_inventory(self, folder, error, expired, proof, inventory_timeout=3):
        """Deterministic supervisor boundary; no actual child or clock wait."""
        clock, stopped, requested, polled = [10.0], [], [], [False]
        process = SimpleNamespace(pid=1234567, returncode=None)
        def poll():
            if not polled[0]:
                polled[0] = True
                if expired:
                    clock[0] = 19.9  # Only 0.1s of the original budget remains.
            return process.returncode
        process.poll = poll
        process.wait = lambda timeout=None: process.returncode
        original_timeout = 10.0
        def inventory(pids=None, timeout=3):
            requested.append(timeout)
            clock[0] = 20.0 if expired else 13.0
            raise error
        def stop(owner):
            # Moving beyond the deadline during cleanup must not retroactively
            # reclassify an inventory error that occurred before the deadline.
            clock[0] = 25.0
            stopped.append(True); process.returncode = -signal.SIGTERM
            owner.cleanup_proven = proof
            return [] if proof else None
        fake_time = SimpleNamespace(monotonic=lambda: clock[0], sleep=lambda _: None)
        with patch.object(ci, 'time', fake_time), \
             patch.object(ci.subprocess, 'Popen', return_value=process), \
             patch.object(ci._owner_module, 'snapshot', side_effect=inventory), \
             patch.object(ci.ProcessOwner, 'stop', stop):
            if expired and isinstance(error, subprocess.TimeoutExpired):
                result = ci.bounded(['inert-never-launched'], folder, folder / 'deadline.log',
                                    original_timeout, inventory_timeout=inventory_timeout)
                self.assertTrue(result['timeout'])
                self.assertIs(result['cleanup_proven'], proof)
                self.assertFalse(ci.accepted(result))
            else:
                with self.assertRaises(type(error)) as caught:
                    ci.bounded(['inert-never-launched'], folder, folder / 'deadline.log',
                               original_timeout, inventory_timeout=inventory_timeout)
                self.assertIs(caught.exception, error)
        self.assertEqual(stopped, [True], 'owned stop must run exactly once')
        self.assertEqual(len(requested), 1, 'inventory must not be retried')
        self.assertAlmostEqual(requested[0], .1 if expired else min(inventory_timeout, original_timeout))
        report = json.loads((folder / 'deadline.log.processes.json').read_text())
        expected_inventory_errors = 0 if isinstance(error, ci.CIInterrupted) else 1
        self.assertEqual(len(report['inventory_errors']), expected_inventory_errors)
        if expected_inventory_errors:
            self.assertEqual(report['inventory_errors'][0]['phase'], 'capture')
        self.assertEqual(report['remaining'], [] if proof else None)
        self.assertIs(report['cleanup_proven'], proof)
        return report

    def test_command_deadline_expiring_during_inventory_is_a_failed_timeout(self):
        for proof in (True, False):
            with self.subTest(cleanup_proven=proof), tempfile.TemporaryDirectory(prefix='mavmoos-ci-deadline-') as name:
                report = self.deadline_inventory(Path(name), subprocess.TimeoutExpired(['ps'], 3), True, proof)
                self.assertIsNone(report['original_failure'])
                self.assertEqual(report['inventory_errors'][0]['type'], 'TimeoutExpired')

    def test_larger_inventory_window_preserves_original_command_deadline(self):
        for expired in (False, True):
            with self.subTest(expired=expired), tempfile.TemporaryDirectory(prefix='mavmoos-ci-inventory-') as name:
                error = subprocess.TimeoutExpired(['ps'], 15)
                report = self.deadline_inventory(Path(name), error, expired, True, inventory_timeout=15)
                expected = None if expired else {'type': 'TimeoutExpired', 'error': str(error)}
                self.assertEqual(report['original_failure'], expected)

    def test_process_snapshot_enforces_the_extended_host_bound(self):
        reply = SimpleNamespace(returncode=0, stdout='11 1 11 /inert/process\n', stderr='')
        with patch.object(ci._owner_module.subprocess, 'run', return_value=reply) as query:
            self.assertIn(11, ci._owner_module.snapshot(timeout=15))
            self.assertEqual(query.call_args.kwargs['timeout'], 15)
            with self.assertRaisesRegex(ValueError, 'invalid process snapshot timeout'):
                ci._owner_module.snapshot(timeout=15.01)
            self.assertEqual(query.call_count, 1)

    def test_predeadline_inventory_failure_and_other_errors_remain_original(self):
        for proof in (True, False):
            for error, expired in ((subprocess.TimeoutExpired(['ps'], 3), False),
                                   (ValueError('independent inventory failure'), True),
                                   (ci.CIInterrupted(signal.SIGTERM, 'inert cancellation'), True)):
                with self.subTest(error=type(error).__name__, cleanup_proven=proof), \
                     tempfile.TemporaryDirectory(prefix='mavmoos-ci-before-deadline-') as name:
                    report = self.deadline_inventory(Path(name), error, expired, proof)
                    self.assertEqual(report['original_failure']['type'], type(error).__name__)

    def test_history_size_does_not_trigger_per_record_snapshots(self):
        process = SimpleNamespace(pid=100, poll=lambda: 0)
        owner = ci.ProcessOwner(process)
        owner.records = {pid: {'pid': pid, 'parent': 100, 'group': pid,
                         'command': '/already/exited/' + str(pid)} for pid in range(1000, 3616)}
        unrelated = {999: {'pid': 999, 'parent': 1, 'group': 999, 'command': '/unrelated/process'}}
        with patch.object(ci._owner_module, 'snapshot', return_value=unrelated) as query, \
             patch.object(ci.os, 'kill') as kill, patch.object(ci.os, 'killpg') as killpg:
            self.assertEqual(owner.stop(), [])
        self.assertTrue(owner.cleanup_proven)
        self.assertLessEqual(query.call_count, 5, 'snapshot work grew with 2616 exited identities')
        self.assertTrue(all(call.kwargs.get('pids') is None for call in query.call_args_list),
                        'freshly proven exited IDs were still queried')
        kill.assert_not_called(); killpg.assert_not_called()

    def test_only_exact_inventory_ps_is_omitted_and_unknown_descendants_stay(self):
        def row(pid, parent, command):
            return {'pid': pid, 'parent': parent, 'group': pid, 'command': command}
        rows = {100: row(100, 1, '/owned/controller'),
                101: row(101, 100, 'ps -axww -o pid=,ppid=,pgid=,command='),
                102: row(102, 100, 'ps -p 1'), 103: row(103, 100, '(ps)'),
                104: row(104, 101, '/unknown/descendant')}
        owner = ci.ProcessOwner(SimpleNamespace(pid=100, poll=lambda: None))
        with patch.object(ci._owner_module, 'snapshot', return_value=rows):
            owner.capture()
        self.assertEqual(set(owner.records), {100, 102, 103, 104})
        self.assertEqual(len(owner.observer_ps_omitted), 1)

    def test_snapshot_api_targets_known_ids_and_distinguishes_empty_from_failure(self):
        completed = subprocess.CompletedProcess([], 0, '11 1 11 /owned/process\n', '')
        with patch.object(ci._owner_module.subprocess, 'run', return_value=completed) as run:
            self.assertEqual(set(ci._owner_module.snapshot(pids=[17, 11, 17], timeout=.25)), {11})
            self.assertEqual(run.call_args.args[0],
                ['ps', '-ww', '-p', '11,17', '-o', 'pid=,ppid=,pgid=,command='])
        empty = subprocess.CompletedProcess([], 1, '', '')
        with patch.object(ci._owner_module.subprocess, 'run', return_value=empty):
            self.assertEqual(ci._owner_module.snapshot(pids=[11]), {})
            with self.assertRaises(subprocess.CalledProcessError):
                ci._owner_module.snapshot()
        error = subprocess.CompletedProcess([], 1, '', 'denied')
        with patch.object(ci._owner_module.subprocess, 'run', return_value=error):
            with self.assertRaises(subprocess.CalledProcessError):
                ci._owner_module.snapshot(pids=[11])
        for stdout in ('', 'unparseable output'):
            invalid = subprocess.CompletedProcess([], 0, stdout, '')
            with patch.object(ci._owner_module.subprocess, 'run', return_value=invalid):
                with self.assertRaises(ValueError):
                    ci._owner_module.snapshot()
        for kwargs in ({'pids': [False]}, {'pids': [-1]}, {'timeout': 0}, {'timeout': 16}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                ci._owner_module.snapshot(**kwargs)

    def test_actual_targeted_snapshot_reads_self_and_completed_empty_pid_set(self):
        self.assertIn(os.getpid(), ci._owner_module.snapshot(pids=[os.getpid()]))
        completed = subprocess.Popen([sys.executable, '-c', 'pass'])
        completed.wait(timeout=3)
        self.assertEqual(ci._owner_module.snapshot(pids=[completed.pid]), {})

    def test_inventory_failure_stops_scheduler_and_keeps_unknown_cleanup_explicit(self):
        real_snapshot, owners = ci._owner_module.snapshot, []
        with tempfile.TemporaryDirectory(prefix='mavmoos-ci-inventory-failure-') as name:
            folder = Path(name)
            command, pidfile = self.fixture(folder, parent_delay=1.2)
            next_case = folder / 'next-case-started'
            parent = Path(command[1])
            parent.write_text(parent.read_text() + '\nfrom pathlib import Path\n' +
                              'Path(' + repr(str(next_case)) + ').write_text("must not run")\n')
            original = subprocess.TimeoutExpired(['ps', 'original-capture'], 3)
            cleanup = subprocess.TimeoutExpired(['ps', 'cleanup-inventory'], 1)
            armed = [False]
            def inventory(pids=None, timeout=3):
                if armed[0]:
                    raise original if pids is None and len(owners[0].inventory_errors) == 0 else cleanup
                rows = real_snapshot(pids=pids, timeout=timeout)
                if pidfile.exists():
                    child = rows.get(int(pidfile.read_text()))
                    armed[0] = bool(child and 'separate_child.py' in child['command'])
                return rows
            constructor = ci.ProcessOwner
            def retain(process):
                owner = constructor(process); owners.append(owner); return owner
            try:
                with patch.object(ci, 'ProcessOwner', side_effect=retain), \
                     patch.object(ci._owner_module, 'snapshot', side_effect=inventory):
                    with self.assertRaises(subprocess.TimeoutExpired) as caught:
                        ci.bounded(command, folder, folder / 'inventory-failure.log', 10)
                self.assertIs(caught.exception, original, 'cleanup failure masked the original inventory error')
                self.assertIsNotNone(owners[0].process.poll(), 'scheduler/direct child survived failed inventory')
                self.assertFalse(next_case.exists(), 'scheduler advanced after inventory failure')
                report = json.loads((folder / 'inventory-failure.log.processes.json').read_text())
                self.assertIsNone(report['remaining'])
                self.assertIs(report['cleanup_proven'], False)
                self.assertEqual(report['original_failure']['type'], 'TimeoutExpired')
                self.assertEqual(report['cleanup_failure']['type'], 'CleanupUnproven')
                self.assertTrue(any(row['phase'] == 'cleanup.initial' for row in report['inventory_errors']))
                self.assertTrue(any(row['signal'] == signal.SIGTERM for row in report['direct_actions']))
            finally:
                # Inventory is restored: cleanup only the exact created inert
                # identity. This manual fixture step does not upgrade the
                # controller's deliberately UNKNOWN cleanup report above.
                self.ensure_gone(pidfile)
                if owners and owners[0].process.poll() is None:
                    owners[0].process.kill(); owners[0].process.wait(timeout=3)

    def test_targeted_fallback_cleans_known_child_without_claiming_complete_global_inventory(self):
        real_snapshot, owners = ci._owner_module.snapshot, []
        with tempfile.TemporaryDirectory(prefix='mavmoos-ci-targeted-fallback-') as name:
            folder = Path(name)
            command, pidfile = self.fixture(folder)
            original = subprocess.TimeoutExpired(['ps', 'injected-global-failure'], 3)
            armed = [False]
            def inventory(pids=None, timeout=3):
                if pids is None and armed[0]:
                    raise original
                rows = real_snapshot(pids=pids, timeout=timeout)
                if pidfile.exists():
                    child = rows.get(int(pidfile.read_text()))
                    armed[0] = bool(child and 'separate_child.py' in child['command'])
                return rows
            constructor = ci.ProcessOwner
            def retain(process):
                owner = constructor(process); owners.append(owner); return owner
            try:
                with patch.object(ci, 'ProcessOwner', side_effect=retain), \
                     patch.object(ci._owner_module, 'snapshot', side_effect=inventory):
                    with self.assertRaises(subprocess.TimeoutExpired) as caught:
                        ci.bounded(command, folder, folder / 'targeted.log', 10)
                self.assertIs(caught.exception, original)
                self.assertIsNotNone(owners[0].process.poll())
                row = real_snapshot(pids=[int(pidfile.read_text())]).get(int(pidfile.read_text()))
                self.assertFalse(row and 'separate_child.py' in row['command'])
                report = json.loads((folder / 'targeted.log.processes.json').read_text())
                self.assertIsNone(report['remaining']); self.assertIs(report['cleanup_proven'], False)
                self.assertTrue(any(entry['phase'] == 'cleanup.initial' for entry in report['inventory_errors']))
            finally:
                self.ensure_gone(pidfile)

    def test_cleanup_exception_cannot_replace_original_exception(self):
        owners, constructor = [], ci.ProcessOwner
        original = RuntimeError('original supervision failure')
        def retain(process):
            owner = constructor(process); owners.append(owner); return owner
        with tempfile.TemporaryDirectory(prefix='mavmoos-ci-original-failure-') as name:
            folder = Path(name)
            try:
                with patch.object(ci, 'ProcessOwner', side_effect=retain), \
                     patch.object(constructor, 'capture', side_effect=original), \
                     patch.object(constructor, 'stop', side_effect=ValueError('nested cleanup failure')):
                    with self.assertRaises(RuntimeError) as caught:
                        ci.bounded([sys.executable, '-c', 'import time;time.sleep(15)'],
                                   folder, folder / 'original.log', 10)
                self.assertIs(caught.exception, original)
                report = json.loads((folder / 'original.log.processes.json').read_text())
                self.assertIsNone(report['remaining']); self.assertIs(report['cleanup_proven'], False)
                self.assertEqual(report['original_failure']['type'], 'RuntimeError')
                self.assertEqual(report['cleanup_failure']['type'], 'ValueError')
            finally:
                if owners:
                    owners[0].stop()

    def test_denied_pause_is_recorded_but_term_is_still_attempted(self):
        process = subprocess.Popen([sys.executable, '-c', 'import time;time.sleep(15)'], start_new_session=True)
        owner, killpg = ci.ProcessOwner(process), os.killpg
        def deny_pause(group, sig):
            if group == process.pid and sig == signal.SIGSTOP:
                raise PermissionError('inert denied pause')
            return killpg(group, sig)
        try:
            with patch.object(ci.os, 'killpg', side_effect=deny_pause):
                self.assertIsNone(owner.stop())
            self.assertIsNotNone(process.poll())
            self.assertFalse(owner.cleanup_proven)
            self.assertEqual(owner.cleanup_errors[0]['type'], 'PermissionError')
            self.assertTrue(any(row['signal'] == signal.SIGTERM for row in owner.direct_actions))
        finally:
            if process.poll() is None:
                process.kill(); process.wait(timeout=3)

    def test_descendant_signal_error_does_not_skip_other_owned_processes(self):
        def row(pid, command):
            return {'pid': pid, 'parent': 1, 'group': pid, 'command': command}
        children = {101: row(101, '/owned/first'), 102: row(102, '/owned/second')}
        unrelated = {999: row(999, '/unrelated/process')}
        owner = ci.ProcessOwner(SimpleNamespace(pid=100, poll=lambda: 0))
        owner.records = dict(children)
        calls = [0]
        def snapshot(pids=None, timeout=3):
            calls[0] += 1
            return {**unrelated, **children} if calls[0] < 3 else unrelated
        def signal_owned(pid, sig):
            if pid == 101 and sig == signal.SIGTERM:
                raise PermissionError('inert descendant denied TERM')
        with patch.object(ci._owner_module, 'snapshot', side_effect=snapshot), \
             patch.object(ci.os, 'kill', side_effect=signal_owned) as kill:
            self.assertIsNone(owner.stop())
        self.assertFalse(owner.cleanup_proven)
        self.assertTrue(any(item.args == (102, signal.SIGTERM) for item in kill.call_args_list))
        self.assertEqual(owner.cleanup_errors[0]['pid'], 101)

    def test_native_postflight_drift_keeps_original_supervision_error(self):
        spec = importlib.util.spec_from_file_location('_ci_native_wrapper_test', ROOT / 'scripts/ci/native_case.py')
        wrapper = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {'run': ci}):
            spec.loader.exec_module(wrapper)
        with tempfile.TemporaryDirectory(prefix='mavmoos-ci-native-postflight-') as name:
            folder = Path(name)
            manifest, attestation = folder / 'manifest.json', folder / 'attestation.json'
            manifest.write_text('frozen manifest'); attestation.write_text('frozen attestation')
            config = {'schema': 1, 'id': 'inert.postflight', 'output': str(folder),
                      'source_root': str(ROOT), 'manifest': str(manifest), 'attestation': str(attestation),
                      'vehicle_profile': 'ardurover', 'command': ['never-executed'],
                      'working_directory': str(folder), 'timeout_s': 1}
            path = folder / 'launch.json'; path.write_text(json.dumps(config))
            def failed(_argv, _cwd, _output, timeout, *, inventory_timeout):
                self.assertEqual(timeout, config['timeout_s'])
                self.assertEqual(inventory_timeout, ci.NATIVE_INVENTORY_TIMEOUT)
                manifest.write_text('changed manifest')
                raise subprocess.TimeoutExpired(['original-supervision-error'], 1)
            previous = signal.getsignal(signal.SIGTERM)
            try:
                with patch.object(wrapper, 'bounded', side_effect=failed), \
                     patch.object(sys, 'argv', ['native_case.py', '--config', str(path)]):
                    self.assertEqual(wrapper.main(), 1)
            finally:
                signal.signal(signal.SIGTERM, previous)
            report = json.loads((folder / 'native-launch-result.json').read_text())
            self.assertIn('original-supervision-error', report['error'])
            self.assertIn('manifest/attestation changed', report['postflight_error'])

    def test_sigterm_base_exception_survives_subprocess_poll_oserror_handler(self):
        process = subprocess.Popen([sys.executable, '-c', 'import time;time.sleep(15)'],
                                   start_new_session=True)
        original_handler = signal.getsignal(signal.SIGTERM)
        poll = subprocess.Popen._internal_poll
        parameters = inspect.signature(poll).parameters
        # CPython 3.12 binds _waitpid directly; 3.14 binds a _del_safe holder.
        # Inject only the waitpid dependency, preserving the real poll handler.
        if '_del_safe' in parameters:
            holder = parameters['_del_safe'].default
            original_waitpid = holder.waitpid
            def replace_waitpid(function):
                return patch.object(holder, 'waitpid', function)
        else:
            self.assertIn('_waitpid', parameters, 'unsupported POSIX Popen poll implementation')
            original_waitpid = parameters['_waitpid'].default
            defaults = poll.__defaults__
            index = list(parameters).index('_waitpid') - (len(parameters) - len(defaults))
            def replace_waitpid(function):
                selected = list(defaults); selected[index] = function
                return patch.object(poll, '__defaults__', tuple(selected))
        try:
            for error, swallowed in ((InterruptedError('old EINTR-style signal'), True),
                                     (ci.CIInterrupted(signal.SIGTERM), False)):
                armed = [True]
                def handler(_number, _frame):
                    raise error
                def deliver_inside_poll(pid, flags):
                    if pid == process.pid and armed[0]:
                        armed[0] = False
                        os.kill(os.getpid(), signal.SIGTERM)
                    return original_waitpid(pid, flags)
                signal.signal(signal.SIGTERM, handler)
                with replace_waitpid(deliver_inside_poll):
                    if swallowed:
                        self.assertIsNone(process.poll())
                    else:
                        with self.assertRaises(ci.CIInterrupted):
                            process.poll()
                self.assertFalse(armed[0], 'actual SIGTERM was not delivered inside Popen.poll')
        finally:
            signal.signal(signal.SIGTERM, original_handler)
            process.terminate()
            process.wait(timeout=3)

    def fixture(self, folder, parent_delay=15):
        child, parent, pidfile = folder / 'separate_child.py', folder / 'parent.py', folder / 'child.pid'
        child.write_text('import os,signal,sys,time\nfrom pathlib import Path\n'
            'signal.signal(signal.SIGTERM,signal.SIG_IGN)\n'
            'pidfile=Path(sys.argv[1])\npending=pidfile.with_suffix(".pending")\n'
            'pending.write_text(str(os.getpid()))\npending.replace(pidfile)\ntime.sleep(30)\n')
        parent.write_text('import subprocess,sys,time\n'
            'subprocess.Popen([sys.executable,sys.argv[1],sys.argv[2]],start_new_session=True)\n'
            'time.sleep(' + str(parent_delay) + ')\n')
        return [sys.executable, str(parent), str(child), str(pidfile)], pidfile

    def ensure_gone(self, pidfile):
        if not pidfile.exists():
            return
        pid = int(pidfile.read_text())
        row = ci._owner_module.snapshot().get(pid)
        if row and 'separate_child.py' in row['command']:
            # This is solely an emergency fixture cleanup of the exact created
            # PID/argv; a controller leak still fails the preceding assertion.
            os.kill(pid, signal.SIGKILL)

    def test_timeout_stops_separate_session_child(self):
        with tempfile.TemporaryDirectory(prefix='mavmoos-ci-process-') as name:
            folder = Path(name)
            command, pidfile = self.fixture(folder)
            try:
                report = ci.bounded(command, folder, folder / 'timeout.log', .7)
                self.assertTrue(report['timeout'])
                self.assertTrue(report['cleanup_proven'])
                process_report = json.loads((folder / 'timeout.log.processes.json').read_text())
                self.assertEqual(process_report['remaining'], [])
                self.assertTrue(any('separate_child.py' in row['command'] for row in process_report['observed']))
                row = ci._owner_module.snapshot().get(int(pidfile.read_text()))
                self.assertFalse(row and 'separate_child.py' in row['command'])
            finally:
                self.ensure_gone(pidfile)

    def test_successful_parent_leak_is_cleaned_but_not_accepted(self):
        with tempfile.TemporaryDirectory(prefix='mavmoos-ci-process-') as name:
            folder = Path(name)
            command, pidfile = self.fixture(folder, parent_delay=.5)
            try:
                report = ci.bounded(command, folder, folder / 'leak.log', 5)
                self.assertEqual(report['exit_code'], 0)
                self.assertGreater(report['unexpected_live_descendants'], 0)
                self.assertFalse(ci.accepted(report))
                self.assertTrue(report['cleanup_proven'])
            finally:
                self.ensure_gone(pidfile)

    def test_exception_stops_separate_session_child(self):
        with tempfile.TemporaryDirectory(prefix='mavmoos-ci-process-') as name:
            folder = Path(name)
            command, pidfile = self.fixture(folder)
            previous = signal.getsignal(signal.SIGALRM)
            def alarm(_number, _frame):
                raise ci.CIInterrupted(signal.SIGALRM, 'inert test')
            signal.signal(signal.SIGALRM, alarm)
            signal.setitimer(signal.ITIMER_REAL, .7)
            try:
                with self.assertRaises(ci.CIInterrupted):
                    ci.bounded(command, folder, folder / 'interrupt.log', 10)
                report = json.loads((folder / 'interrupt.log.processes.json').read_text())
                self.assertEqual(report['remaining'], [])
                row = ci._owner_module.snapshot().get(int(pidfile.read_text()))
                self.assertFalse(row and 'separate_child.py' in row['command'])
            finally:
                signal.setitimer(signal.ITIMER_REAL, 0)
                signal.signal(signal.SIGALRM, previous)
                self.ensure_gone(pidfile)


if __name__ == '__main__':
    unittest.main(verbosity=2)
