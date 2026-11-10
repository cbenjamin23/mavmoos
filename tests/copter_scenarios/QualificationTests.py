#!/usr/bin/env python3
"""In-memory additive-qualification controls; no native qualification is run."""
import argparse
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import subprocess
import tempfile
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[2]
RECORDER = REGISTRY = SYNTHETIC = CHECKER = BOUNDARY = None


def initialize(root):
    global ROOT, RECORDER, REGISTRY, SYNTHETIC, CHECKER, BOUNDARY
    ROOT = Path(root).resolve()
    sys.path.insert(0, str(ROOT / 'scripts/capabilities'))
    import record_result
    RECORDER = record_result
    REGISTRY = record_result.parse((ROOT / 'config/capabilities/copter.json').read_bytes())
    def module(name, relative):
        spec = importlib.util.spec_from_file_location(name, ROOT / relative)
        value = importlib.util.module_from_spec(spec);spec.loader.exec_module(value);return value
    CHECKER = module('_qualification_checker', 'tests/copter_scenarios/scenario_evidence.py')
    BOUNDARY = module('_qualification_boundary', 'tests/copter_scenarios/hold_boundary.py')
    SYNTHETIC = module('_qualification_fixture', 'tests/copter_scenarios/ScenarioEvidenceTests.py')
    SYNTHETIC.CHECKER = CHECKER


def evidence(auto_disarm=False):
    events, packets, times, finished, arms, modes, parameters = SYNTHETIC.fixture()
    manifest, attestation, firmware, inputs = SYNTHETIC.firmware_fixture()
    if auto_disarm:
        events = [row for row in events if not (row['type'] in ('request', 'operation_result') and 'id=c1-disarm,' in row['value'])]
    proof = CHECKER.audit_probe(events, 'route_hold_return', inputs)
    files = {'probe.jsonl': ('\n'.join(json.dumps(row) for row in events) + '\n').encode(),
             'fc-observer.jsonl': ('\n'.join(json.dumps(row) for row in packets) + '\n').encode(),
             'dataflash-observations.json': json.dumps({'arms': arms, 'modes': modes, 'fixture_parameters': parameters}).encode(),
             CHECKER.FIRMWARE_FILES['manifest']: manifest, CHECKER.FIRMWARE_FILES['attestation']: attestation}
    receipts = []
    for name, at in sorted(times.items(), key=lambda item: item[1]):
        receipts.append({'kind': name, 'python_mono': at})
        if name in proof['boundary_tokens']:
            token = proof['boundary_tokens'][name].encode('ascii')
            files['control/' + name] = files['control/' + name + '.ack'] = token
            receipts.append({'kind': name + '.ack', 'python_mono': at + .01,
                             'token_sha256': hashlib.sha256(token).hexdigest()})
    native = firmware['native_sha256']
    result = {'result': 'PASS', 'hardware': False, 'probe_exit': 0, 'profile': 'arducopter', 'observer_writes': 0,
              'rc_input_sent': False, 'mission_execution_requested': False, 'parameter_write_requested': False,
              'ports': {'monitor_peer': 12345}, 'ports_clear': {'synthetic': True},
              'owned_processes_remaining': [], 'artifact_drift': [], 'cleanup_proven': True,
              'source_ref': '1' * 40, 'ardupilot_ref': firmware['source_revision'], 'app_sha256': '4' * 64,
              'native_sha256': native, 'firmware_identity': firmware, 'firmware_identity_after': copy.deepcopy(firmware),
              'firmware_inputs': {role: {'path': name, 'sha256': hashlib.sha256(files[name]).hexdigest()} for role, name in CHECKER.FIRMWARE_FILES.items()},
              'firmware_discovery': proof['firmware_discovery'],
              'artifacts_before': {str(ROOT / name): hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
                                   for name in ('tests/copter_scenarios/scenario_evidence.py',
                                                'tests/copter_scenarios/hold_boundary.py',
                                                'tests/observation_load/passive_receiver.py')},
              'files': {name: hashlib.sha256(data).hexdigest() for name, data in files.items()},
              'scenario': 'route_hold_return', 'events': receipts,
              'hold_boundaries': BOUNDARY.audit_boundary_bytes(receipts, proof, lambda key: files['control/' + key]),
              'passive_receiver': {'read_only': True, 'thread_retired': True, 'socket_closed': True, 'error': None,
                                   'maximum_datagrams': 150000, 'datagrams': len(packets), 'accepted_datagrams': len(packets),
                                   'first_read_at': packets[0]['mono'], 'last_read_at': packets[-1]['mono'], 'peer': ['127.0.0.1', 12345]},
              'native_observer_socket_inventory': 'synthetic exact-owned endpoint witness; not native evidence',
              'monitor_finished_python_mono': finished, 'probe_evidence': proof,
              'independent_fc': CHECKER.audit_fc(packets, times, finished, proof),
              'dataflash_evidence': CHECKER.audit_dataflash(arms, modes, parameters, 'route_hold_return')}
    return result, files


def change_input(result, files, role, change):
    """Rehash deliberately forged copied bytes, so binding gates get exercised."""
    name = CHECKER.FIRMWARE_FILES[role]
    value = json.loads(files[name]);change(value);files[name] = json.dumps(value, sort_keys=True).encode()
    digest = hashlib.sha256(files[name]).hexdigest()
    result['files'][name] = digest;result['firmware_inputs'][role]['sha256'] = digest


class QualificationTests(unittest.TestCase):
    def reject(self, mutate):
        result, files = evidence()
        mutate(result, files)
        with self.assertRaises((ValueError, KeyError, TypeError)):
            RECORDER.evidence_cells(REGISTRY, result, lambda name: files[name], ROOT)

    def test_explicit_operation_cells_positive(self):
        result, files = evidence()
        cells, identity = RECORDER.evidence_cells(REGISTRY, result, lambda name: files[name], ROOT)
        self.assertEqual(len(cells), 10)
        self.assertEqual(identity['cycles'], 1)
        self.assertEqual(result['firmware_identity']['qualification'], 'not_inferred')

    def test_auto_disarm_is_not_an_explicit_disarm_claim(self):
        result, files = evidence(auto_disarm=True)
        cells, _ = RECORDER.evidence_cells(REGISTRY, result, lambda name: files[name], ROOT)
        self.assertNotIn('copter.disarm', {cell['capability_id'] for cell in cells})

    def test_failed_result(self): self.reject(lambda r, _: r.update(result='FAIL'))
    def test_physical_scope(self): self.reject(lambda r, _: r.update(hardware=True))
    def test_observer_write(self): self.reject(lambda r, _: r.update(observer_writes=1))
    def test_rc_input(self): self.reject(lambda r, _: r.update(rc_input_sent=True))
    def test_mission_execution(self): self.reject(lambda r, _: r.update(mission_execution_requested=True))
    def test_parameter_write(self): self.reject(lambda r, _: r.update(parameter_write_requested=True))
    def test_retained_process(self): self.reject(lambda r, _: r.update(owned_processes_remaining=[123]))
    def test_uncleared_port(self): self.reject(lambda r, _: r['ports_clear'].update(synthetic=False))
    def test_native_drift(self): self.reject(lambda r, _: r['artifact_drift'].append('native'))
    def test_source_missing(self): self.reject(lambda r, _: r.update(source_ref='main'))
    def test_firmware_missing(self): self.reject(lambda r, _: r.update(ardupilot_ref='stable'))
    def test_native_hash_missing(self): self.reject(lambda r, _: r.update(native_sha256='unknown'))
    def test_manifest_is_not_qualification(self): self.reject(lambda r, _: r['firmware_identity'].update(qualification='supported'))
    def test_changed_post_firmware_identity(self): self.reject(lambda r, _: r['firmware_identity_after'].update(frame='other'))
    def test_missing_copied_manifest(self): self.reject(lambda r, _: r['files'].pop(CHECKER.FIRMWARE_FILES['manifest']))
    def test_missing_copied_attestation(self): self.reject(lambda r, _: r['files'].pop(CHECKER.FIRMWARE_FILES['attestation']))
    def test_firmware_input_hash_binding(self): self.reject(lambda r, _: r['firmware_inputs']['manifest'].update(sha256='0' * 64))
    def test_firmware_input_name_binding(self): self.reject(lambda r, _: r['firmware_inputs']['manifest'].update(path='unbound.json'))
    def test_forged_copied_manifest(self): self.reject(lambda r, f: change_input(r, f, 'manifest', lambda m: m['source']['version'].update(minor=6)))
    def test_expected_version_comes_from_bound_copy(self):
        def change(result, files):
            change_input(result, files, 'manifest', lambda m: m['source']['version'].update(minor=6))
            digest = result['files'][CHECKER.FIRMWARE_FILES['manifest']]
            result['firmware_identity']['manifest_sha256'] = digest
            result['firmware_identity_after']['manifest_sha256'] = digest
            change_input(result, files, 'attestation', lambda a: a.update(manifest_sha256=digest))
            # Even self-consistently rebound input bytes cannot turn a 4.7.0
            # observation into evidence for a 4.6.0 declaration.
        self.reject(change)
    def test_unbound_expected_version_is_not_used(self):
        result, files = evidence();result['expected_version'] = {'major': 99}
        cells, identity = RECORDER.evidence_cells(REGISTRY, result, lambda name: files[name], ROOT)
        self.assertEqual(len(cells), 10);self.assertEqual(identity['observed_version']['major'], 4)
    def test_forged_attestation_native(self): self.reject(lambda r, f: change_input(r, f, 'attestation', lambda a: a['artifact'].update(sha256='0' * 64)))
    def test_forged_attestation_profile(self): self.reject(lambda r, f: change_input(r, f, 'attestation', lambda a: a.update(profile_id='different')))
    def test_forged_attestation_source(self): self.reject(lambda r, f: change_input(r, f, 'attestation', lambda a: a['source_identity'].update(revision='0' * 40)))
    def test_forged_attestation_platform(self): self.reject(lambda r, f: change_input(r, f, 'attestation', lambda a: a['platform'].update(machine='other')))
    def test_forged_discovery_summary(self): self.reject(lambda r, _: r['firmware_discovery'].update(unique_observations=4))
    def test_checker_not_the_executed_source(self): self.reject(lambda r, _: r.update(artifacts_before={}))
    def test_missing_raw(self): self.reject(lambda r, _: r['files'].pop('probe.jsonl'))
    def test_changed_raw(self): self.reject(lambda _, files: files.update({'probe.jsonl': b'{}\n'}))
    def test_forged_probe_summary(self): self.reject(lambda r, _: r['probe_evidence'].update(cycles=2))
    def test_repeated_phase_marker(self): self.reject(lambda r, _: r['events'].append(r['events'][0]))

    def test_ack_is_replayed_but_not_a_flight_phase(self):
        result, files = evidence()
        times = RECORDER.scenario_times(result, result['probe_evidence'], lambda name: files[name], ROOT, CHECKER)
        self.assertEqual(set(times), set(CHECKER.markers(result['scenario'])))
        self.assertEqual(len(result['events']) - len(times), 4)

    def test_missing_ack_receipt(self):
        self.reject(lambda r, _: r['events'].remove(next(x for x in r['events'] if x['kind'].endswith('.ack'))))

    def test_foreign_ack_receipt(self):
        self.reject(lambda r, _: r['events'].append({'kind': 'foreign.ack', 'python_mono': 151.1}))

    def test_ack_before_original_in_array(self):
        def change(result, _):
            index = next(i for i, row in enumerate(result['events']) if row['kind'].endswith('.ack'))
            result['events'][index - 1], result['events'][index] = result['events'][index], result['events'][index - 1]
        self.reject(change)

    def test_changed_ack_bytes_even_when_rehashed(self):
        def change(result, files):
            name = next(name for name in files if name.startswith('control/') and name.endswith('.ack'))
            files[name] = b'foreign\n';result['files'][name] = hashlib.sha256(files[name]).hexdigest()
        self.reject(change)

    def test_ack_bytes_must_be_in_protected_inventory(self):
        self.reject(lambda r, _: r['files'].pop(next(name for name in r['files'] if name.endswith('.ack'))))

    def test_forged_boundary_summary(self):
        self.reject(lambda r, _: r['hold_boundaries']['boundaries'].clear())

    def test_boundary_helper_not_executed_source(self):
        self.reject(lambda r, _: r['artifacts_before'].pop(str(ROOT / 'tests/copter_scenarios/hold_boundary.py')))

    def test_receiver_helper_not_executed_source(self):
        self.reject(lambda r, _: r['artifacts_before'].update({str(ROOT / 'tests/observation_load/passive_receiver.py'): '0' * 64}))

    def test_unretired_capture(self): self.reject(lambda r, _: r['passive_receiver'].update(thread_retired=False))
    def test_unclosed_capture(self): self.reject(lambda r, _: r['passive_receiver'].update(socket_closed=False))
    def test_capture_error(self): self.reject(lambda r, _: r['passive_receiver'].update(error='lost peer'))
    def test_unknown_cleanup(self): self.reject(lambda r, _: r.update(cleanup_proven=False))
    def test_capture_bound_changed(self): self.reject(lambda r, _: r['passive_receiver'].update(maximum_datagrams=150001))
    def test_capture_over_bound(self): self.reject(lambda r, _: r['passive_receiver'].update(datagrams=150001))
    def test_capture_count_boolean(self): self.reject(lambda r, _: r['passive_receiver'].update(accepted_datagrams=True))
    def test_capture_read_after_finished(self): self.reject(lambda r, _: r['passive_receiver'].update(last_read_at=r['monitor_finished_python_mono'] + .001))
    def test_capture_foreign_peer(self): self.reject(lambda r, _: r['passive_receiver'].update(peer=['127.0.0.2', 12345]))
    def test_capture_endpoint_unproven(self): self.reject(lambda r, _: r.update(native_observer_socket_inventory=''))

    def test_new_contract_cannot_downgrade_by_omitting_helper(self):
        result, files = evidence()
        result.pop('hold_boundaries');result['events'] = [row for row in result['events'] if not row['kind'].endswith('.ack')]
        with tempfile.TemporaryDirectory() as empty_source:
            with self.assertRaisesRegex(ValueError, 'legacy independent phase contract'):
                RECORDER.scenario_times(result, result['probe_evidence'], lambda name: files[name], empty_source, CHECKER)

    def test_legacy_source_keeps_exact_original_marker_contract(self):
        result, files = evidence()
        result.pop('hold_boundaries');result['events'] = [row for row in result['events'] if not row['kind'].endswith('.ack')]
        legacy_proof = copy.deepcopy(result['probe_evidence']);legacy_proof.pop('boundary_tokens')
        with tempfile.TemporaryDirectory() as empty_source:
            times = RECORDER.scenario_times(result, legacy_proof, lambda name: files[name], empty_source, CHECKER)
            self.assertEqual(set(times), set(CHECKER.markers(result['scenario'])))

    def test_actual_cli_generate_verify_preserves_registry_and_rejects_overwrite(self):
        result, files = evidence()
        with tempfile.TemporaryDirectory(prefix='mavmoos-qualification-synthetic-') as temp:
            root = Path(temp)
            checker = root / 'tests/copter_scenarios/scenario_evidence.py';checker.parent.mkdir(parents=True)
            checker.write_bytes((ROOT / 'tests/copter_scenarios/scenario_evidence.py').read_bytes())
            for name in ('tests/copter_scenarios/hold_boundary.py', 'tests/observation_load/passive_receiver.py'):
                target = root / name;target.parent.mkdir(parents=True, exist_ok=True);target.write_bytes((ROOT / name).read_bytes())
            raw = root / 'docs/synthetic-case';raw.mkdir(parents=True)
            for name, data in files.items():
                target = raw / name;target.parent.mkdir(parents=True, exist_ok=True);target.write_bytes(data)
            (raw / 'result.json').write_text(json.dumps(result))
            archive = root / 'docs/synthetic-raw.zip'
            with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as bundle:
                for path in raw.rglob('*'):
                    if path.is_file(): bundle.writestr('case/' + path.relative_to(raw).as_posix(), path.read_bytes())
            registry = root / 'registry.json';registry.write_text(json.dumps(REGISTRY))
            original = registry.read_bytes()
            output = root / 'docs/additive-cells.json'
            command = [sys.executable, '-B', str(ROOT / 'scripts/capabilities/record_result.py'), '--source-root', str(root),
                       '--registry', str(registry), '--result', str(raw / 'result.json'), '--raw-root', str(raw),
                       '--raw-archive', str(archive), '--archive-prefix', 'case', '--output', str(output)]
            first = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(first.returncode, 0, first.stdout + first.stderr)
            self.assertEqual(json.loads(first.stdout)['result'], 'PASS')
            self.assertEqual(registry.read_bytes(), original)
            verify = [sys.executable, '-B', str(ROOT / 'scripts/capabilities/record_result.py'), '--source-root', str(root),
                      '--registry', str(registry), '--verify', str(output)]
            self.assertEqual(subprocess.run(verify, capture_output=True).returncode, 0)
            self.assertNotEqual(subprocess.run(command, capture_output=True).returncode, 0)
            archive.write_bytes(archive.read_bytes() + b'changed')
            self.assertNotEqual(subprocess.run(verify, capture_output=True).returncode, 0)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--source-root', type=Path, default=ROOT)
    args = parser.parse_args();initialize(args.source_root)
    result = unittest.TextTestRunner().run(unittest.defaultTestLoader.loadTestsFromTestCase(QualificationTests))
    print(json.dumps({'result': 'PASS' if result.wasSuccessful() else 'FAIL', 'tests_run': result.testsRun,
                      'scope': 'in-memory synthetic qualification controls, not native/vehicle execution'}, indent=2))
    return int(not result.wasSuccessful())


if __name__ == '__main__':
    raise SystemExit(main())
