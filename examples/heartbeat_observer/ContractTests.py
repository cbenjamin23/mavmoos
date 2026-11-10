#!/usr/bin/env python3
"""Inspect actual external-module/PluginHost output; fake Services, no sockets."""
import argparse
import json
import subprocess
import sys
import unittest


class Contracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        result = subprocess.run([ARGS.driver, ARGS.module, ARGS.wrong_abi], capture_output=True,
                                text=True, timeout=10)
        if result.returncode:
            raise AssertionError(result.stderr + result.stdout)
        # CTest's LastTest.log retains the actual module/guard trace for replay.
        print(result.stdout, end='')
        parsed = [json.loads(line) for line in result.stdout.splitlines()]
        cls.rows = {row['phase']: row for row in parsed}
        if len(cls.rows) != len(parsed):
            raise AssertionError('duplicate phases')
        cls.samples = {name: row['observation'] for name, row in cls.rows.items() if 'observation' in row}

    def test_startup_has_no_sample(self):
        value = self.samples['startup']
        self.assertFalse(value['present'])
        self.assertFalse(value['fresh'])
        self.assertIsNone(value['age_s'])

    def test_original_metadata_and_unmapped_source_age(self):
        value = self.samples['fresh']
        self.assertTrue(value['present'] and value['fresh'])
        self.assertEqual((value['effective_at'], value['read_at']), (100.1, 100.2))
        self.assertAlmostEqual(value['age_s'], .15)
        self.assertEqual(value['age_basis'], 'QUEUE_EMPTY_FLOOR')
        self.assertEqual(value['source_age'], 'unknown')
        self.assertEqual(value['source_quality'], 'unassessed')
        self.assertEqual((value['system_id'], value['component_id'], value['custom_mode_raw']), (1, 1, 4))

    def test_cached_publication_does_not_renew_sample(self):
        before, after = self.samples['fresh'], self.samples['cached']
        for field in ('effective_at', 'read_at', 'observation_id'):
            self.assertEqual(before[field], after[field])
        self.assertAlmostEqual(after['age_s'], .9)

    def test_future_and_unknown_metadata_rejected(self):
        self.assertEqual(self.samples['future_rejected']['metadata_rejected'], 1)
        self.assertEqual(self.samples['basis_rejected']['metadata_rejected'], 2)
        self.assertEqual(self.samples['basis_rejected']['observation_id'], 1)

    def test_foreign_source_and_older_observation_rejected(self):
        self.assertEqual(self.samples['source_rejected']['source_rejected'], 1)
        self.assertEqual(self.samples['order_rejected']['order_rejected'], 1)
        self.assertEqual(self.samples['order_rejected']['observation_id'], 1)

    def test_expiry_preserves_stale_provenance(self):
        value = self.samples['stale']
        self.assertTrue(value['present'])
        self.assertFalse(value['fresh'])
        self.assertEqual(value['effective_at'], 100.1)
        self.assertAlmostEqual(value['age_s'], 2.4)

    def test_reset_invalidates_and_requires_post_reset_metadata(self):
        before, after = self.samples['reset'], self.samples['pre_reset_rejected']
        self.assertFalse(before['present'] or after['present'])
        self.assertEqual(before['session_epoch'], 2)
        self.assertEqual(after['metadata_rejected'], 3)
        fresh = self.samples['new_session']
        self.assertTrue(fresh['fresh'])
        self.assertEqual((fresh['session_epoch'], fresh['observation_id']), (2, 2))

    def test_clock_fault_clears_and_latches(self):
        for phase in ('clock_fault', 'clock_latched'):
            self.assertTrue(self.samples[phase]['clock_fault'])
            self.assertFalse(self.samples[phase]['present'] or self.samples[phase]['fresh'])
        self.assertFalse(self.samples['clock_reset']['clock_fault'])
        self.assertFalse(self.samples['clock_reset']['present'])

    def test_exact_read_acquisition_is_allowed(self):
        value = self.samples['read_acquisition']
        self.assertTrue(value['fresh'])
        self.assertEqual(value['effective_at'], value['read_at'])
        self.assertEqual(value['age_basis'], 'READ_ACQUISITION')

    def test_malformed_wire_does_not_replace_observation(self):
        value = self.samples['wire_rejected']
        self.assertEqual(value['wire_rejected'], 1)
        self.assertEqual(value['observation_id'], self.samples['read_acquisition']['observation_id'])

    def test_actual_factory_and_loader_reject_invalid_composition(self):
        value = self.rows['complete']
        self.assertTrue(value['wrong_abi_rejected'] and value['null_context_rejected'] and value['duplicate_rejected'])
        self.assertGreater(value['plugin_abi'], 0)

    def test_defaults_and_default_cadence(self):
        for phase in ('startup', 'fresh', 'stale', 'reset', 'new_session'):
            self.assertEqual((self.samples[phase]['sample_timeout_s'], self.samples[phase]['publish_interval_s']), (2, .25))
        self.assertEqual(self.rows['startup']['publications'], 1)
        self.assertEqual(self.rows['default_not_early']['publications'], 1)
        self.assertEqual(self.rows['fresh']['publications'], 2)

    def test_configured_expiry_cadence_and_original_metadata(self):
        expected_counts = {'startup': 1, 'before_due': 1, 'fresh': 2, 'not_early': 2,
                           'cached': 3, 'exact_expiry': 4, 'stale': 5, 'reset': 6, 'new_session': 7}
        for phase, count in expected_counts.items():
            self.assertEqual(self.rows['configured.' + phase]['publications'], count)
            value = self.samples['configured.' + phase]
            self.assertEqual((value['sample_timeout_s'], value['publish_interval_s']), (.5, .125))
        for phase in ('fresh', 'cached', 'exact_expiry', 'stale'):
            value = self.samples['configured.' + phase]
            self.assertEqual((value['effective_at'], value['read_at'], value['observation_id']), (100.0625, 100.0625, 1))
        self.assertTrue(self.samples['configured.exact_expiry']['fresh'])
        self.assertAlmostEqual(self.samples['configured.exact_expiry']['age_s'], .5)
        self.assertFalse(self.samples['configured.stale']['fresh'])
        self.assertAlmostEqual(self.samples['configured.stale']['age_s'], .625)

    def test_configured_reset_preserves_settings_but_clears_sample(self):
        value = self.samples['configured.reset']
        self.assertFalse(value['present'] or value['fresh'])
        self.assertEqual(value['session_epoch'], 2)
        value = self.samples['configured.new_session']
        self.assertTrue(value['present'] and value['fresh'])
        self.assertEqual((value['session_epoch'], value['observation_id']), (2, 2))

    def test_valid_complete_batches_ranges_equality_and_omitted_defaults(self):
        cases = {'defaults': (2, .25), 'timeout_only': (4, .25), 'interval_only': (2, .125),
                 'minimum': (.25, .02), 'maximum': (10, 2), 'equal': (.5, .5),
                 'scientific': (1, .1), 'empty_replaces_previous': (2, .25)}
        for name, expected in cases.items():
            with self.subTest(name=name):
                result = self.rows['config.' + name + '.result']
                self.assertEqual((result['status'], result['key'], result['reason']), ('Accepted', '', ''))
                value = self.samples['config.' + name + '.observation']
                self.assertEqual((value['sample_timeout_s'], value['publish_interval_s']), expected)

    def test_unknown_duplicate_and_cross_field_batches_are_atomic(self):
        for name in ('unknown', 'valid_then_unknown', 'duplicate_timeout', 'duplicate_interval',
                     'cross_field', 'cross_field_reversed', 'interval_below', 'interval_above'):
            with self.subTest(name=name):
                result = self.rows['config.' + name + '.result']
                self.assertEqual(result['status'], 'UnknownKey' if 'unknown' in name else 'Invalid')
                self.assertTrue(result['key'] and result['reason'])
                value = self.samples['config.' + name + '.observation']
                self.assertEqual((value['sample_timeout_s'], value['publish_interval_s']), (1, .1))
                self.assertEqual(value['session_epoch'], 1)
                self.assertFalse(value['clock_fault'] or value['present'])

    def test_strict_finite_full_value_parsing_and_no_partial_commit(self):
        names = ['config.invalid_timeout_' + str(i) for i in range(18)]
        for name in names:
            with self.subTest(name=name):
                result = self.rows[name + '.result']
                self.assertEqual((result['status'], result['key']), ('Invalid', 'sample_timeout'))
                self.assertTrue(result['reason'])
                value = self.samples[name + '.observation']
                self.assertEqual((value['sample_timeout_s'], value['publish_interval_s']), (1, .1))

    def test_interval_parser_rejects_the_same_nonfinite_or_partial_values(self):
        for i in range(16):
            with self.subTest(i=i):
                name = 'config.invalid_interval_' + str(i)
                result = self.rows[name + '.result']
                self.assertEqual((result['status'], result['key']), ('Invalid', 'publish_interval'))
                self.assertTrue(result['reason'])
                value = self.samples[name + '.observation']
                self.assertEqual((value['sample_timeout_s'], value['publish_interval_s']), (1, .1))

    def test_real_host_checked_batch_failure_stops_before_dispatch(self):
        value = self.rows['host_configuration_rejected']
        self.assertFalse(value['accepted'])
        self.assertEqual((value['configuration_service_calls'], value['publications'], value['effects']), (0, 0, 0))
        self.assertIn('example_heartbeat_observer', value['error'])
        self.assertIn('publish_interval', value['error'])

    def test_configuration_calls_no_services_and_produces_no_publications(self):
        results = [row for phase, row in self.rows.items() if phase.startswith('config.') and phase.endswith('.result')]
        self.assertEqual(len(results), 50)
        self.assertTrue(all(row['configuration_service_calls'] == 0 and row['effects'] == 0 for row in results))

    def test_every_current_mutating_service_guard_is_exercised(self):
        expected = {'sendCommand', 'acquireNavigation', 'revokeNavigation', 'navigation', 'parameter', 'mission',
                    'missionFinished', 'recordStoredMissionProof', 'invalidateStoredMissionProof', 'beginMissionExecution',
                    'beginMissionExecutionAbort', 'finishMissionExecution', 'suspendNavigation', 'resumeNavigation',
                    'commandFinished', 'cancelCommand', 'flightLog', 'flightLogFinished'}
        value = self.rows['effect_guards']
        self.assertEqual(set(value['detected']), expected)
        self.assertEqual(value['intentional_guard_attempts'], len(expected))

    def test_read_only_and_zero_effect_calls_through_all_phases(self):
        self.assertTrue(all(row['effects'] == 0 for row in self.rows.values()))
        for value in self.samples.values():
            self.assertTrue(value['read_only'] and value['example'])
            self.assertFalse(value['authoritative_for_control'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('driver', 'module', 'wrong-abi'):
        parser.add_argument('--' + name, required=True)
    ARGS, remaining = parser.parse_known_args()
    unittest.main(argv=[sys.argv[0], *remaining], verbosity=2)
