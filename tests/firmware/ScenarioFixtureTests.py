#!/usr/bin/env python3
"""Pure profile/fixture controls; no vehicle, process, socket or runtime I/O."""
import copy
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
PATH = ROOT / 'scripts/firmware/scenario_fixtures.py'
spec = importlib.util.spec_from_file_location('_scenario_fixture_tests', PATH)
F = importlib.util.module_from_spec(spec);spec.loader.exec_module(F)


def identity(profile):
    pin = F.PROFILES[profile]
    return {'schema': 1, 'profile_id': profile, 'manifest_sha256': pin['firmware_manifest_sha256'],
            'source_revision': pin['source_revision'], 'frame': 'quad', 'native_sha256': '1' * 64,
            'attestation_sha256': '2' * 64, 'qualification': 'not_inferred'}


def declaration(profile):
    return (ROOT / 'config/scenario-fixtures' / (profile + '.json')).read_bytes()


class ScenarioFixtureTests(unittest.TestCase):
    def test_both_pinned_profiles_keep_distinct_mission_shapes(self):
        for profile, count, ids in [('arducopter-dev-04cf6486-quad', 1, [43, 51, 47]),
                                    ('arducopter-4.6.3-quad', 0, [43, 47])]:
            with self.subTest(profile=profile):
                value = F.select(ROOT, identity(profile), 'armed_wiped_mission_read')
                self.assertEqual(value['expectation']['expected_count'], count)
                self.assertEqual(value['expectation']['required_host_message_ids'], ids)
                self.assertEqual(value['qualification'], 'not_inferred')
                self.assertEqual(value['binding']['native_sha256'], '1' * 64)

    def test_range_names_and_units_preserve_physical_limits(self):
        for profile, parameters in [('arducopter-dev-04cf6486-quad', {'RNGFND1_MIN': .1, 'RNGFND1_MAX': 30}),
                                    ('arducopter-4.6.3-quad', {'RNGFND1_MIN_CM': 10, 'RNGFND1_MAX_CM': 3000})]:
            value = F.select(ROOT, identity(profile), 'downward_distance')['expectation']
            self.assertEqual(value['preboot_parameters'], parameters)
            self.assertEqual((value['physical_min_m'], value['physical_max_m']), (.1, 30))
            self.assertEqual((value['reported_min_cm'], value['reported_max_cm']), (10, 3000))

    def test_closed_range_readback_checks_raw_names_values_and_physical_units(self):
        for profile in F.PROFILES:
            selected = F.select(ROOT, identity(profile), 'downward_distance')
            proof = F.range_readback(selected, selected['expectation']['preboot_parameters'])
            self.assertEqual((proof['physical_min_m'], proof['physical_max_m']), (.1, 30))
            self.assertEqual(proof['raw_parameters'], selected['expectation']['preboot_parameters'])

    def test_old_defaults_or_wrong_unit_values_are_not_equivalent_limits(self):
        selected = F.select(ROOT, identity('arducopter-4.6.3-quad'), 'downward_distance')
        for parameters in ({'RNGFND1_MIN_CM': 20, 'RNGFND1_MAX_CM': 700},
                           {'RNGFND1_MIN_CM': .1, 'RNGFND1_MAX_CM': 30},
                           {'RNGFND1_MIN': .1, 'RNGFND1_MAX': 30},
                           {'RNGFND1_MIN_CM': True, 'RNGFND1_MAX_CM': 3000},
                           {'RNGFND1_MIN_CM': 10, 'RNGFND1_MAX_CM': float('nan')}):
            with self.subTest(parameters=parameters), self.assertRaises(ValueError):F.range_readback(selected, parameters)

    def test_unknown_profile_identity_or_scenario_cannot_select(self):
        profile = next(iter(F.PROFILES)); valid = identity(profile)
        for value in (None, {}, {**valid, 'profile_id': 'unknown'}, {**valid, 'schema': True},
                      {**valid, 'frame': 'rover'}, {**valid, 'qualification': 'PASS'},
                      {**valid, 'native_sha256': None}, {**valid, 'attestation_sha256': 'bad'}):
            with self.subTest(value=value), self.assertRaises(ValueError):F.select(ROOT, value, 'armed_wiped_mission_read')
        with self.assertRaises(ValueError):F.select(ROOT, valid, 'derive_from_traffic')

    def test_manifest_and_source_cross_binding_rejected(self):
        first, second = list(F.PROFILES)
        for field in ('manifest_sha256', 'source_revision'):
            value = identity(first);value[field] = identity(second)[field]
            with self.assertRaises(ValueError):F.select(ROOT, value, 'armed_wiped_mission_read')
        with self.assertRaises(ValueError):F.from_declaration(declaration(second), identity(first), 'armed_wiped_mission_read')

    def test_changed_shape_cannot_be_selected_even_if_internally_consistent(self):
        profile = 'arducopter-dev-04cf6486-quad';value = json.loads(declaration(profile))
        value['fixtures']['armed_wiped_mission_read'] = json.loads(declaration('arducopter-4.6.3-quad'))['fixtures']['armed_wiped_mission_read']
        with self.assertRaisesRegex(ValueError, 'pinned source declaration'):
            F.from_declaration(json.dumps(value).encode(), identity(profile), 'armed_wiped_mission_read')

    def test_remote_advertisement_never_selects_expectation(self):
        profile = 'arducopter-dev-04cf6486-quad'
        value = identity(profile);value['reported_version'] = {'major': 4, 'minor': 6, 'patch': 3}
        value['observed_count'] = 0
        self.assertEqual(F.select(ROOT, value, 'armed_wiped_mission_read')['expectation']['expected_count'], 1)

    def test_offline_reselection_rejects_modified_recorded_shape(self):
        profile = 'arducopter-4.6.3-quad';selected = F.select(ROOT, identity(profile), 'armed_wiped_mission_read')
        self.assertEqual(F.reselect(declaration(profile), identity(profile), 'armed_wiped_mission_read', selected)['expectation']['expected_count'], 0)
        for change in (lambda v: v['expectation'].update(expected_count=1),
                       lambda v: v['binding'].update(source_revision='0' * 40),
                       lambda v: v.update(declaration_sha256='0' * 64)):
            changed = copy.deepcopy(selected);change(changed)
            with self.assertRaisesRegex(ValueError, 'recorded scenario fixture differs'):
                F.reselect(declaration(profile), identity(profile), 'armed_wiped_mission_read', changed)

    def test_copy_is_exact_and_existing_evidence_is_not_replaced(self):
        profile = 'arducopter-4.6.3-quad';selected = F.select(ROOT, identity(profile), 'downward_distance')
        with tempfile.TemporaryDirectory() as folder:
            receipt = F.copy_to_evidence(selected, folder)
            self.assertEqual((Path(folder) / receipt['path']).read_bytes(), declaration(profile))
            self.assertEqual(receipt['sha256'], selected['declaration_sha256'])
            with self.assertRaises(FileExistsError):F.copy_to_evidence(selected, folder)

    def test_protected_chain_and_original_manifests_remain_exact(self):
        for profile, pin in F.PROFILES.items():
            selected = F.select(ROOT, identity(profile), 'armed_wiped_mission_read')
            self.assertEqual(len(F.protected_paths(selected)), 3)
            self.assertEqual(F.digest(Path(selected['firmware_manifest_path']).read_bytes()), pin['firmware_manifest_sha256'])
            self.assertEqual(F.digest(Path(selected['selector_path']).read_bytes()), selected['selector_sha256'])

    def test_missing_changed_or_symlinked_source_input_fails_before_copy(self):
        profile = 'arducopter-4.6.3-quad'
        for mode in ('missing', 'changed', 'symlink'):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as folder:
                root = Path(folder)
                for name in ('scripts/firmware', 'config/firmware', 'config/scenario-fixtures'):(root / name).mkdir(parents=True)
                helper = root / 'scripts/firmware/scenario_fixtures.py';helper.write_bytes(PATH.read_bytes())
                (root / 'config/firmware' / (profile + '.json')).write_bytes((ROOT / 'config/firmware' / (profile + '.json')).read_bytes())
                target = root / 'config/scenario-fixtures' / (profile + '.json')
                if mode == 'changed':target.write_bytes(declaration(profile) + b' ')
                if mode == 'symlink':target.symlink_to(ROOT / 'config/scenario-fixtures' / (profile + '.json'))
                spec = importlib.util.spec_from_file_location('_temporary_scenario_selector', helper)
                module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
                with self.assertRaises(ValueError):module.select(root, identity(profile), 'armed_wiped_mission_read')

    def test_stale_imported_selector_cannot_claim_another_source_root(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaises(ValueError):F.select(folder, identity('arducopter-4.6.3-quad'), 'armed_wiped_mission_read')

    def test_strict_json_rejects_duplicates_and_nonfinite_values(self):
        for text in ('{"a":1,"a":2}', '{"a":NaN}', '{"a":Infinity}', '{"a":1e400}'):
            with self.assertRaises(ValueError):F.parse(text)

    def test_all_three_runners_reject_unknown_profile_before_socket_or_launch(self):
        old_path=list(sys.path)
        try:
            sys.path[:0]=[str(ROOT/'tests'),str(ROOT/'tests/concurrent'),
                          str(ROOT/'tests/observation_load'),str(ROOT/'tests/distance_native')]
            with patch.dict(sys.modules,{'pymavlink':SimpleNamespace(mavutil=None,DFReader=None)}),patch.dict(os.environ):
                for relative in ('tests/concurrent/run_concurrent.py','tests/observation_load/run_observation_load.py',
                                 'tests/distance_native/run_distance.py'):
                    with self.subTest(runner=relative),tempfile.TemporaryDirectory() as folder:
                        spec=importlib.util.spec_from_file_location('_preflight_'+Path(relative).stem,ROOT/relative)
                        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
                        output=Path(folder)/'not-created'
                        args=SimpleNamespace(ardupilot_root=str(Path(folder)/'ap'),moos_root=str(Path(folder)/'moos'),
                            build=str(Path(folder)/'build'),behavior_dir=str(Path(folder)/'behaviors'),
                            source_root=str(ROOT),output=str(output),probe=None,firmware_manifest=None,firmware_attestation=None)
                        bad={**identity('arducopter-4.6.3-quad'),'profile_id':'unknown'}
                        if 'concurrent/' in relative:
                            context=patch.object(module.fixture,'native_firmware_profile',return_value=bad)
                        else:
                            ownership=SimpleNamespace(native_firmware_profile=lambda *_args:bad)
                            context=patch.object(module,'source_helpers',return_value={
                                'ownership':ownership,'catalog':None,'scenario_fixtures':F})
                        with context,patch.object(module.socket,'socket') as socket_call,patch.object(module.subprocess,'Popen') as launch:
                            with self.assertRaisesRegex(ValueError,'unknown exact scenario profile'):
                                module.run(args,'mission_read') if 'concurrent/' in relative else module.run(args)
                            socket_call.assert_not_called();launch.assert_not_called()
                            self.assertFalse(output.exists())
        finally:
            sys.path[:]=old_path


if __name__ == '__main__':
    unittest.main()
