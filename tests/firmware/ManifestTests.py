#!/usr/bin/env python3
"""Manifest/attestation rejection tests. Fake binaries are headers, never run."""
import copy
import importlib.util
import json
from pathlib import Path
import platform
import struct
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('_firmware_manifest', ROOT / 'scripts/firmware/manifest.py')
m = importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
sys.path.insert(0, str(ROOT / 'scripts/firmware'))
from export_source import export_source


class ManifestTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='mavmoos-firmware-schema-')
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name);self.source = self.base / 'source';self.source.mkdir()
        files = {'ArduCopter/version.h': '#define THISFIRMWARE "ArduCopter V4.7.0-dev"\n#define FIRMWARE_VERSION 4,7,0,FIRMWARE_VERSION_TYPE_DEV\n',
                 'Tools/autotest/default_params/copter.parm': 'FRAME_CLASS 1\n',
                 'libraries/GCS_MAVLink/GCS_Common.cpp': '// inert source witness\n',
                 'extra.cpp': '// separately tracked source\n', '.gitignore': 'build/\n'}
        for name, data in files.items():
            p = self.source / name;p.parent.mkdir(parents=True, exist_ok=True);p.write_text(data)
        def git(*args):
            return subprocess.check_output(['git', '-C', str(self.source), *args], text=True, stderr=subprocess.DEVNULL).strip()
        git('init', '-q');git('add', '.')
        git('-c', 'user.name=MAVMOOS Test', '-c', 'user.email=fixture@example.invalid', 'commit', '-qm', 'inert fixture')
        value = m.load(ROOT / 'config/firmware/arducopter-dev-04cf6486-quad.json')
        value['profile_id'] = 'inert-firmware-identity-test'
        value['source']['revision'] = git('rev-parse', 'HEAD');value['source']['submodules'] = []
        value['source']['protected_files'] = {k: m.file_sha(self.source / k) for k in value['source']['protected_files']}
        self.manifest = self.base / 'manifest.json';self.write(self.manifest, value)
        self.native = self.source / value['artifact_path'];self.native.parent.mkdir(parents=True)
        data = bytearray(128);machine = platform.machine().lower()
        if platform.system() == 'Darwin':
            data[:4] = b'\xcf\xfa\xed\xfe';struct.pack_into('<I', data, 4, 0x100000c if machine == 'arm64' else 0x1000007)
        else:
            data[:6] = b'\x7fELF\x02\x01';struct.pack_into('<H', data, 18, 183 if machine in ('arm64', 'aarch64') else 62)
        self.native.write_bytes(data);self.native.chmod(0o755)
        self.attestation = self.base / 'attestation.json'
        self.write(self.attestation, m.make_attestation(self.manifest, self.source))

    @staticmethod
    def write(path, value):
        path.write_text(json.dumps(value, indent=2) + '\n')

    def verify(self):
        return m.verify_native(self.manifest, self.attestation, self.source)

    def test_both_real_source_profiles_validate(self):
        for name in ('arducopter-dev-04cf6486-quad', 'arducopter-4.6.3-quad'):
            value = m.validate_manifest(ROOT / 'config/firmware' / (name + '.json'))
            self.assertEqual(value['profile_id'], name)

    def test_positive_identity_never_claims_build_or_qualification(self):
        value = self.verify()
        self.assertEqual(value['native_path'], str(self.native.resolve()))
        self.assertEqual(value['qualification'], 'not_inferred')
        self.assertEqual(value['build_provenance'], 'existing_artifact_identity')

    def test_manifest_mutations_rejected(self):
        mutations = {
            'extra_field': lambda v: v.update(extra=True),
            'boolean_schema': lambda v: v.update(schema=True),
            'floating_revision': lambda v: v['source'].update(revision='master'),
            'different_repository': lambda v: v['source'].update(repository='https://example.invalid/fork.git'),
            'path_profile_id': lambda v: v.update(profile_id='../escape'),
            'repeated_dots_id': lambda v: v.update(profile_id='x..x'),
            'unknown_role': lambda v: v.update(role='qualified'),
            'invented_qualification': lambda v: v.update(qualification='PASS'),
            'rover_as_copter': lambda v: v.update(vehicle_profile='ardurover'),
            'executable_override': lambda v: v.update(artifact_path='/bin/sh'),
            'target_broadcast': lambda v: v['target'].update(system_id=0),
            'target_boolean': lambda v: v['target'].update(component_id=True),
            'physical_scope': lambda v: v['fixture'].update(hardware=True),
            'timewarp': lambda v: v['fixture'].update(timewarp=10),
            'home_bounds': lambda v: v['fixture'].update(home=[100, 0, 0, 0]),
            'model_override': lambda v: v['fixture'].update(model='plane'),
            'missing_source_witness': lambda v: v['source']['protected_files'].pop('ArduCopter/version.h'),
            'default_parameter_override': lambda v: v['fixture'].update(default_parameter_file='../other.parm'),
            'nonfinite_parameter': lambda v: v['fixture']['parameters'].update(SIM_WIND_SPD=1e999),
            'boolean_parameter': lambda v: v['fixture']['parameters'].update(LOG_DISARMED=True),
            'dependency_alias': lambda v: v['dependencies'].update(mavlink_revision='latest'),
            'source_path_escape': lambda v: v['source']['protected_files'].update({'../escape': '0' * 64}),
            'submodule_escape': lambda v: v['source']['submodules'].append({'path': '../module', 'revision': '0' * 40}),
            'submodule_dot': lambda v: v['source']['submodules'].append({'path': '.', 'revision': '0' * 40}),
        }
        original = m.load(self.manifest)
        for name, change in mutations.items():
            with self.subTest(name=name):
                value = copy.deepcopy(original);change(value);self.write(self.manifest, value)
                with self.assertRaises((ValueError, TypeError)):
                    m.validate_manifest(self.manifest)

    def test_attestation_mutations_rejected(self):
        mutations = {
            'boolean_schema': lambda v: v.update(schema=True),
            'extra_field': lambda v: v.update(extra=True),
            'different_profile': lambda v: v.update(profile_id='other'),
            'different_manifest': lambda v: v.update(manifest_sha256='0' * 64),
            'different_semantic_manifest': lambda v: v.update(manifest_identity_sha256='0' * 64),
            'different_source': lambda v: v['source_identity'].update(revision='0' * 40),
            'different_machine': lambda v: v['platform'].update(machine='other'),
            'different_os': lambda v: v['platform'].update(system='other'),
            'different_artifact': lambda v: v['artifact'].update(sha256='0' * 64),
            'different_size': lambda v: v['artifact'].update(bytes=1),
            'different_format': lambda v: v['artifact'].update(format='script'),
            'different_path': lambda v: v['artifact'].update(path='some/other/path'),
            'qualification_from_identity': lambda v: v.update(qualification='supported'),
            'unrecorded_build': lambda v: v['build'].update(kind='recorded_source_build'),
            'unknown_provenance': lambda v: v['build'].update(kind='trusted'),
            'invented_existing_record': lambda v: v['build'].update(record={}),
        }
        original = m.load(self.attestation)
        for name, change in mutations.items():
            with self.subTest(name=name):
                value = copy.deepcopy(original);change(value);self.write(self.attestation, value)
                with self.assertRaises((ValueError, TypeError, KeyError)):
                    self.verify()

    def test_duplicate_json_keys_rejected(self):
        self.manifest.write_text('{"schema":1,"schema":1}')
        with self.assertRaises(ValueError):m.load(self.manifest)

    def test_nonfinite_json_rejected(self):
        for token in ('NaN', 'Infinity', '-Infinity', '1e999'):
            self.manifest.write_text('{"x":' + token + '}')
            with self.assertRaises(ValueError):m.load(self.manifest)

    def test_dirty_other_tracked_source_rejected(self):
        (self.source / 'extra.cpp').write_text('// modified source\n')
        with self.assertRaises(ValueError):self.verify()

    def test_protected_source_change_rejected(self):
        (self.source / 'libraries/GCS_MAVLink/GCS_Common.cpp').write_text('// changed\n')
        with self.assertRaises(ValueError):self.verify()

    def test_declared_version_must_match_source(self):
        value = m.load(self.manifest);value['source']['version']['minor'] = 99;self.write(self.manifest, value)
        with self.assertRaises(ValueError):m.make_attestation(self.manifest, self.source)

    def test_declared_label_must_match_source(self):
        value = m.load(self.manifest);value['source']['release_label'] = 'invented';self.write(self.manifest, value)
        with self.assertRaises(ValueError):m.make_attestation(self.manifest, self.source)

    def test_binary_mutation_rejected(self):
        data = self.native.read_bytes();self.native.write_bytes(data[:-1] + b'1')
        with self.assertRaises(ValueError):self.verify()

    def test_nonexecutable_artifact_rejected(self):
        self.native.chmod(0o600)
        with self.assertRaises(ValueError):self.verify()

    def test_symlink_artifact_rejected(self):
        data = self.native.read_bytes();self.native.unlink();outside = self.base / 'outside';outside.write_bytes(data);outside.chmod(0o755);self.native.symlink_to(outside)
        with self.assertRaises(ValueError):self.verify()

    def test_parent_directory_escape_rejected(self):
        binary = self.native.read_bytes();self.native.unlink();self.native.parent.rmdir()
        outside = self.base / 'outside-bin';outside.mkdir();(outside / 'arducopter').write_bytes(binary);(outside / 'arducopter').chmod(0o755)
        self.native.parent.symlink_to(outside, target_is_directory=True)
        with self.assertRaises(ValueError):self.verify()

    def test_wrong_expected_profile_rejected(self):
        with self.assertRaises(ValueError):m.verify_native(self.manifest, self.attestation, self.source, 'ardurover')

    def test_exact_manifest_bytes_attested(self):
        self.manifest.write_text(self.manifest.read_text() + '\n')
        with self.assertRaises(ValueError):self.verify()

    def test_recorded_build_receipt_binding(self):
        source = m.source_identity(m.validate_manifest(self.manifest), self.source)
        record = {'schema': 1, 'result': 'PASS', 'manifest_sha256': m.file_sha(self.manifest), 'source_identity': source,
                  'commands': [{'argv': ['inert-compiler-fixture'], 'exit': 0, 'log': 'compiler.log', 'log_sha256': '0' * 64}],
                  'environment': {}, 'builder_sha256': '1' * 64, 'scope': 'synthetic receipt only, no compiler execution',
                  'native_sha256': m.file_sha(self.native)}
        receipt = self.base / 'receipt.json';self.write(receipt, record)
        self.write(self.attestation, m.make_attestation(self.manifest, self.source, receipt))
        self.assertEqual(self.verify()['build_provenance'], 'recorded_source_build')
        changed = m.load(self.attestation);changed['build']['record']['commands'][0]['exit'] = 1;self.write(self.attestation, changed)
        with self.assertRaises(ValueError):self.verify()

    def test_source_export_original_files_verified(self):
        target = self.base / 'export'
        report = export_source(self.manifest, self.source, target)
        self.assertEqual(report['result'], 'PASS')
        self.assertEqual(report['source_identity']['mode'], 'git_archive_export')
        native = target / 'build/sitl/bin/arducopter';native.parent.mkdir(parents=True)
        native.write_bytes(self.native.read_bytes());native.chmod(0o755)
        self.write(self.attestation, m.make_attestation(self.manifest, target))
        self.assertEqual(m.verify_native(self.manifest, self.attestation, target)['qualification'], 'not_inferred')
        (target / 'extra.cpp').write_text('// changed exported source\n')
        with self.assertRaises(ValueError):m.verify_native(self.manifest, self.attestation, target)

    def test_source_export_refuses_existing_output(self):
        target = self.base / 'export';target.mkdir()
        with self.assertRaises(ValueError):export_source(self.manifest, self.source, target)

    def test_export_report_tampering_rejected(self):
        target = self.base / 'export';export_source(self.manifest, self.source, target)
        path = target / 'mavmoos-firmware-source.json';value = m.load(path);value['source_files_sha256'] = '0' * 64;self.write(path, value)
        with self.assertRaises(ValueError):m.source_identity(m.validate_manifest(self.manifest), target)


if __name__ == '__main__':
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(ManifestTests)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    print(json.dumps({'result': 'PASS' if result.wasSuccessful() else 'FAIL', 'tests': result.testsRun,
                      'manifest_mutations': 24, 'attestation_mutations': 16,
                      'scope': 'schema/identity tests with inert file headers; no native/socket or source-build qualification'}, indent=2))
    raise SystemExit(not result.wasSuccessful())
