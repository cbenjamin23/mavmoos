"""Offline fixture identity and parameter-policy regression tests."""

from copy import deepcopy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from plane_firmware import (AUTOLAND_MANIFEST_PATH, MANIFEST_PATH, SOURCE_ROOT, validate_manifest,
                            verify_attestation, verify_parameters)


class PlaneFirmwareTests(unittest.TestCase):
    def setUp(self):
        self.manifest = validate_manifest(MANIFEST_PATH)
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.fixture = Path(temporary.name) / 'fixture.json'

    def test_checked_manifest_binds_archived_plane_build(self):
        archive = json.loads((SOURCE_ROOT / 'docs/implementation/2026-11-06-plane-build/manifest.json').read_text())
        self.assertEqual(self.manifest['source']['revision'], archive['plane_revision'])
        self.assertEqual(self.manifest['native']['artifact']['sha256'], archive['executable']['sha256'])
        self.assertEqual(self.manifest['native']['artifact']['bytes'], archive['executable']['bytes'])
        self.assertEqual(self.manifest['native']['build_attestation_sha256'],
                         archive['files']['build-attestation.json']['sha256'])
        self.assertEqual(self.manifest['qualification'], 'not_inferred')

    def test_reject_changed_watchdog_quadplane_and_test_sender(self):
        for key, value in (('SYSID_MYGCS', 249), ('FS_GCS_ENABL', 0), ('FS_LONG_ACTN', 0),
                           ('FS_LONG_TIMEOUT', 6), ('RTL_AUTOLAND', 1), ('Q_ENABLE', 1), ('ARSPD_PRIMARY', 1)):
            with self.subTest(parameter=key):
                changed = deepcopy(self.manifest)
                changed['fixture']['parameters'][key] = value
                self.fixture.write_text(json.dumps(changed))
                with self.assertRaisesRegex(ValueError, 'fixture safety policy differs'):
                    validate_manifest(self.fixture)
        changed = deepcopy(self.manifest)
        changed['fixture']['fixture_operator']['system_id'] = 245
        self.fixture.write_text(json.dumps(changed))
        with self.assertRaisesRegex(ValueError, 'test sender identities differ'):
            validate_manifest(self.fixture)

    def test_native_autoland_requires_separate_exact_configuration(self):
        native = validate_manifest(AUTOLAND_MANIFEST_PATH)
        self.assertNotEqual(native['source']['revision'], self.manifest['source']['revision'])
        for key, value in (('MAV_GCS_SYSID_HI', 250), ('ARMING_SKIPCHK', -1), ('MIS_RESTART', 1),
                           ('AUTOLAND_OPTIONS', 0), ('AUTOLAND_WP_DIST', 400)):
            with self.subTest(parameter=key):
                changed = deepcopy(native)
                changed['fixture']['parameters'][key] = value
                self.fixture.write_text(json.dumps(changed))
                with self.assertRaisesRegex(ValueError, 'fixture safety policy differs'):
                    validate_manifest(self.fixture)

    def test_readback_requires_every_exact_finite_numeric_value(self):
        parameters = {key: float(value) for key, value in self.manifest['fixture']['parameters'].items()}
        self.assertEqual(verify_parameters(self.manifest, parameters), parameters)
        for value in (None, False, '1', float('nan'), float('inf'), .99):
            with self.subTest(value=value):
                changed = dict(parameters, FS_GCS_ENABL=value)
                with self.assertRaisesRegex(ValueError, 'Plane active parameter FS_GCS_ENABL'):
                    verify_parameters(self.manifest, changed)
        del parameters['WP_LOITER_RAD']
        with self.assertRaisesRegex(ValueError, 'Plane active parameter WP_LOITER_RAD'):
            verify_parameters(self.manifest, parameters)

    def test_attestation_hash_and_identity_are_both_required(self):
        source, native = self.manifest['source'], self.manifest['native']
        attestation = {'schema': 1, 'status': 'OFFLINE_BUILD_PASS', 'qualification': 'build_only',
                       'source_snapshot_verified': True, 'source_revision': source['revision'],
                       'source_tree': source['tree'], 'source_tag': source['tag'],
                       'release_banner_bytes': source['release_label'],
                       'embedded_short_source_revision': source['revision'][:8],
                       'submodules': [[item['path'], item['revision']] for item in source['submodules']],
                       'native_format': native['format'], 'artifact': native['artifact'],
                       'model_default': self.manifest['fixture']['model_defaults']}
        self.fixture.write_text(json.dumps(attestation))
        with self.assertRaisesRegex(ValueError, 'attestation bytes differ'):
            verify_attestation(self.manifest, self.fixture)
        changed = deepcopy(self.manifest)
        changed['native']['build_attestation_sha256'] = hashlib.sha256(self.fixture.read_bytes()).hexdigest()
        self.assertEqual(verify_attestation(changed, self.fixture), attestation)
        attestation['source_revision'] = '0' * 40
        self.fixture.write_text(json.dumps(attestation))
        changed['native']['build_attestation_sha256'] = hashlib.sha256(self.fixture.read_bytes()).hexdigest()
        with self.assertRaisesRegex(ValueError, 'attested source identity differs'):
            verify_attestation(changed, self.fixture)


if __name__ == '__main__':
    unittest.main()
