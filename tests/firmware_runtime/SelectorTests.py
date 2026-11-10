#!/usr/bin/env python3
"""Test profile selection boundaries; no native process, socket or build."""
import importlib.util
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

PATH = Path(__file__).resolve().parents[1] / 'firmware_identity.py'
SPEC = importlib.util.spec_from_file_location('_firmware_selector_under_test', PATH)
SELECTOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SELECTOR)


class SelectionTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='mavmoos-selector-')
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name).resolve()
        self.manifest = self.root / 'profile.json'
        self.attestation = self.root / 'attestation.json'
        self.environment = patch.dict(os.environ, {'MAVMOOS_FIRMWARE_MANIFEST': '',
                                                  'MAVMOOS_FIRMWARE_ATTESTATION': ''})
        self.environment.start()
        self.addCleanup(self.environment.stop)
        path = self.root / 'scripts/firmware/manifest.py'
        path.parent.mkdir(parents=True)
        path.write_text('def verify_native(manifest, attestation, root, expected_profile):\n'
                        '    return {"profile_id": expected_profile, "native_path": str(root / "build/sitl/bin/arducopter"),\n'
                        '            "received_manifest": str(manifest), "received_attestation": str(attestation),\n'
                        '            "qualification": "not_inferred"}\n')

    def select(self, manifest=None, attestation=None, profile='arducopter'):
        return SELECTOR.selected_firmware(self.root / 'native-source', self.root,
                                          manifest, attestation, profile)

    def test_no_selection_keeps_legacy_check_path(self):
        self.assertIsNone(self.select())

    def test_partial_explicit_selection_rejects(self):
        for manifest, attestation in [(self.manifest, None), (None, self.attestation)]:
            with self.assertRaises(ValueError):
                self.select(manifest, attestation)

    def test_partial_inherited_selection_rejects(self):
        for key, value in [('MAVMOOS_FIRMWARE_MANIFEST', self.manifest),
                           ('MAVMOOS_FIRMWARE_ATTESTATION', self.attestation)]:
            with patch.dict(os.environ, {key: str(value)}), self.assertRaises(ValueError):
                self.select()

    def test_conflicting_explicit_and_inherited_selection_rejects(self):
        with patch.dict(os.environ, {'MAVMOOS_FIRMWARE_MANIFEST': str(self.root / 'other.json')}):
            with self.assertRaises(ValueError):
                self.select(self.manifest, self.attestation)

    def test_verified_identity_and_provenance_are_forwarded(self):
        result = self.select(self.manifest, self.attestation)
        self.assertEqual(result['received_manifest'], str(self.manifest))
        self.assertEqual(result['received_attestation'], str(self.attestation))
        self.assertEqual(result['qualification'], 'not_inferred')
        self.assertEqual(result['verifier_path'], str(self.root / 'scripts/firmware/manifest.py'))

    def test_same_explicit_and_inherited_pair_is_unambiguous(self):
        with patch.dict(os.environ, {'MAVMOOS_FIRMWARE_MANIFEST': str(self.manifest),
                                     'MAVMOOS_FIRMWARE_ATTESTATION': str(self.attestation)}):
            self.assertEqual(self.select(self.manifest, self.attestation)['profile_id'], 'arducopter')

    def test_legacy_inherited_pair_uses_same_verifier(self):
        with patch.dict(os.environ, {'MAVMOOS_FIRMWARE_MANIFEST': str(self.manifest),
                                     'MAVMOOS_FIRMWARE_ATTESTATION': str(self.attestation)}):
            self.assertEqual(self.select()['received_manifest'], str(self.manifest))

    def test_profile_is_explicit_not_coerced(self):
        self.assertEqual(self.select(self.manifest, self.attestation, 'ardurover')['profile_id'], 'ardurover')

    def test_verifier_rejection_never_falls_back(self):
        (self.root / 'scripts/firmware/manifest.py').write_text('def verify_native(*args, **kwargs):\n    raise ValueError("identity mismatch")\n')
        with self.assertRaisesRegex(ValueError, 'identity mismatch'):
            self.select(self.manifest, self.attestation)

    def test_missing_verifier_never_falls_back(self):
        (self.root / 'scripts/firmware/manifest.py').unlink()
        with self.assertRaises(OSError):
            self.select(self.manifest, self.attestation)


class CopterSourcePathTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='mavmoos-gcs-source-')
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name).resolve()
        (self.root / 'ArduCopter').mkdir()
        spec = importlib.util.spec_from_file_location('_owned_source_paths', PATH.with_name('owned_processes.py'))
        self.owner = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.owner)

    def test_exactly_one_known_source_layout(self):
        for name in ('GCS_MAVLink_Copter.cpp', 'GCS_Mavlink.cpp'):
            path = self.root / 'ArduCopter' / name
            path.write_text('source fixture')
            self.assertEqual(self.owner.arducopter_gcs_source(self.root), path)
            path.unlink()

    def test_missing_layout_rejects(self):
        with self.assertRaises(ValueError):
            self.owner.arducopter_gcs_source(self.root)

    def test_ambiguous_layout_rejects(self):
        for name in ('GCS_MAVLink_Copter.cpp', 'GCS_Mavlink.cpp'):
            (self.root / 'ArduCopter' / name).write_text('source fixture')
        with self.assertRaises(ValueError):
            self.owner.arducopter_gcs_source(self.root)

    def test_symlink_does_not_replace_regular_source(self):
        target = self.root / 'other.cpp'
        target.write_text('outside layout')
        (self.root / 'ArduCopter/GCS_Mavlink.cpp').symlink_to(target)
        with self.assertRaises(ValueError):
            self.owner.arducopter_gcs_source(self.root)


if __name__ == '__main__':
    unittest.main()
