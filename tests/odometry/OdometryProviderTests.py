#!/usr/bin/env python3
"""Validate the JSON emitted by the actual odometry module and fake Services."""

import argparse
import json
import subprocess
import unittest


LOCAL = 'MAVMOOS_LOCAL_KINEMATICS'
RAW = 'MAVMOOS_ODOMETRY'
ORIGIN = 'MAVMOOS_LOCAL_ORIGIN'
WHEELS = 'MAVMOOS_WHEEL_DISTANCE'


def reject_nonfinite(value):
    raise ValueError('nonfinite JSON value ' + value)


class ProviderTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        result = subprocess.run([args.driver, args.plugin], capture_output=True, text=True, timeout=10)
        if result.returncode:
            raise RuntimeError(result.stderr)
        cls.cases = {}
        for line in result.stdout.splitlines():
            row = json.loads(line, parse_constant=reject_nonfinite)
            case = cls.cases.setdefault(row['case'], {})
            if row['key'] in case:
                raise ValueError('duplicate ' + row['case'] + '/' + row['key'])
            case[row['key']] = row['value']

    def test_only_observation_outputs(self):
        self.assertEqual(len(self.cases), 27)
        for values in self.cases.values():
            self.assertEqual(set(values), {LOCAL, RAW, ORIGIN, WHEELS})
            self.assertFalse(values[WHEELS]['present'])
            self.assertIsNone(values[WHEELS]['data'])
            for record in values.values():
                self.assertEqual(record['schema'], 1)
                self.assertIs(record['read_only'], True)
                self.assertIs(record['authoritative_for_control'], False)
                self.assertEqual(record['source_age'], 'unknown')

    def test_local_ned_truncated_v2_and_mavlink_1(self):
        local = self.cases['local'][LOCAL]
        data = local['data']
        self.assertTrue(local['fresh'])
        self.assertEqual((data['message_id'], data['wire_version'], data['payload_length']), (32, 2, 24))
        self.assertEqual((data['frame'], data['origin']), ('LOCAL_NED', 'FC_LOCAL_ORIGIN'))
        self.assertEqual(data['time_boot_ms'], 0x12345678)
        self.assertEqual(data['position_m'], [1.25, -2.5, -3.75])
        self.assertEqual(data['velocity_m_s'], [-0.5, 0.25, 0])
        self.assertEqual(self.cases['valid_before_bad_frames'][LOCAL]['data'],
                         dict(data, wire_version=1, payload_length=28))

    def test_raw_pose_and_twist_preserve_wire_data(self):
        data = self.cases['both'][RAW]['data']
        self.assertEqual((data['message_id'], data['wire_version'], data['payload_length']), (331, 2, 233))
        self.assertEqual(data['time_usec_raw'], '9007199254740993')
        self.assertEqual((data['frame_id'], data['child_frame_id']), (20, 12))
        self.assertEqual(data['position_m'], [8, -9, 10])
        self.assertEqual(data['quaternion_wxyz'], [0.5, 0.5, -0.5, 0.5])
        self.assertEqual(data['velocity_m_s'], [1.25, 0, -1])
        self.assertEqual(data['angular_velocity_rad_s'], [0.125, -0.25, 0.375])
        self.assertIsNone(data['pose_covariance'])
        self.assertEqual(data['velocity_covariance'], [0.25 + index * 0.125 for index in range(21)])
        self.assertEqual((data['reset_counter'], data['estimator_type'], data['quality']), (7, 2, 87))

    def test_failed_quality_and_omitted_extensions_are_distinct(self):
        failed = self.cases['explicit_failed_estimate'][RAW]
        self.assertTrue(failed['fresh'])
        self.assertEqual(failed['data']['quality'], -1)
        self.assertEqual(failed['data']['time_usec_raw'], '100')
        self.assertEqual(failed['data']['reset_counter'], 8)
        self.assertEqual(failed['data']['pose_covariance'], [0.5] + [0] * 20)
        omitted = self.cases['absent_extensions'][RAW]['data']
        self.assertEqual(omitted['payload_length'], 230)
        self.assertEqual((omitted['reset_counter'], omitted['estimator_type']), (0, 0))
        self.assertIsNone(omitted['quality'])
        self.assertIsNone(omitted['velocity_covariance'])

    def test_independent_age_and_unchanged_original_times(self):
        local = self.cases['local'][LOCAL]
        self.assertEqual((local['effective_at'], local['read_at']), (100.14, 100.3))
        self.assertEqual(local['age_basis'], 'QUEUE_EMPTY_FLOOR')
        self.assertIs(local['age_is_upper_bound'], True)
        self.assertAlmostEqual(local['age_s'], 0.17)
        self.assertIsNone(self.cases['local'][RAW]['data'])
        stale = self.cases['local_stale']
        self.assertFalse(stale[LOCAL]['fresh'])
        self.assertTrue(stale[RAW]['fresh'])
        self.assertEqual(stale[LOCAL]['observation_id'], local['observation_id'])
        self.assertEqual(stale[LOCAL]['effective_at'], local['effective_at'])
        self.assertEqual(stale[RAW]['age_basis'], 'READ_ACQUISITION')
        self.assertEqual(stale[RAW]['effective_at'], stale[RAW]['read_at'])

    def test_bad_source_time_order_and_shapes_preserve_last_valid_observation(self):
        rejected = self.cases['rejected_sources_and_order'][LOCAL]
        self.assertGreaterEqual(rejected['source_rejected'], 4)
        self.assertEqual(rejected['order_rejected'], 1)
        self.assertEqual(rejected['read_at'], 100.3)
        before = self.cases['valid_before_bad_frames']
        after = self.cases['after_bad_frames']
        for key in (LOCAL, RAW):
            self.assertEqual(after[key]['wire_rejected'], 13)
            for field in ('data', 'read_at', 'effective_at', 'observation_id'):
                self.assertEqual(after[key][field], before[key][field])

    def test_source_binding_loss_explicit_reset_and_configuration(self):
        for name in ('absent', 'before_bound_source', 'heartbeat_expired',
                     'old_floor_after_rebind', 'explicit_reset', 'unsupported_autopilot'):
            for key in (LOCAL, RAW, ORIGIN):
                self.assertFalse(self.cases[name][key]['present'], name)
                self.assertIsNone(self.cases[name][key]['data'], name)
        self.assertNotIn('interval_suppressed', self.cases)
        short = self.cases['short_fresh'][LOCAL]
        self.assertEqual((short['sample_timeout_s'], short['publish_interval_s']), (0.25, 0.1))
        self.assertTrue(short['fresh'])
        self.assertFalse(self.cases['short_stale'][LOCAL]['fresh'])
        rebound = self.cases['new_after_rebind'][LOCAL]
        self.assertGreater(rebound['session_epoch'], short['session_epoch'])
        self.assertTrue(rebound['fresh'])
        self.assertIsNone(self.cases['new_raw_session'][LOCAL]['data'])
        self.assertTrue(self.cases['new_raw_session'][RAW]['fresh'])
        self.assertGreater(self.cases['new_raw_session'][RAW]['session_epoch'], rebound['session_epoch'])

    def test_px4_common_message_fixture(self):
        for key, message_id in ((LOCAL, 32), (RAW, 331), (ORIGIN, 49)):
            record = self.cases['px4_fixture'][key]
            self.assertTrue(record['fresh'])
            self.assertEqual((record['autopilot'], record['system_id'], record['component_id']), (12, 1, 1))
            self.assertEqual(record['data']['message_id'], message_id)
        raw = self.cases['px4_fixture'][RAW]['data']
        self.assertEqual(raw['frame_id'], 1)
        pose_diagonal = {0: 0.25, 6: 0.5, 11: 0.75, 15: 0.125, 18: 0.25, 20: 0.375}
        velocity_diagonal = {0: 0.125, 6: 0.25, 11: 0.375}
        self.assertEqual(raw['pose_covariance'], [pose_diagonal.get(index) for index in range(21)])
        self.assertEqual(raw['velocity_covariance'], [velocity_diagonal.get(index) for index in range(21)])

    def test_origin_values_and_wire_extensions(self):
        data = self.cases['origin_v2'][ORIGIN]['data']
        self.assertEqual(data, {'message_id': 49, 'wire_version': 2, 'payload_length': 19,
                                'latitude_deg_e7': 423584560, 'longitude_deg_e7': -710875890,
                                'altitude_msl_mm': 12345, 'latitude_deg': 42.358456,
                                'longitude_deg': -71.087589, 'altitude_msl_m': 12.345,
                                'time_usec_raw': '9007199254740993'})
        legacy = self.cases['origin_v1'][ORIGIN]['data']
        self.assertEqual((legacy['wire_version'], legacy['payload_length']), (1, 12))
        self.assertEqual((legacy['latitude_deg'], legacy['longitude_deg']), (-90, 180))
        self.assertEqual((legacy['altitude_msl_mm'], legacy['altitude_msl_m']), (-3456, -3.456))
        self.assertIsNone(legacy['time_usec_raw'])
        zero = self.cases['origin_zero'][ORIGIN]['data']
        self.assertEqual((zero['wire_version'], zero['payload_length']), (2, 1))
        self.assertEqual((zero['latitude_deg'], zero['longitude_deg'], zero['altitude_msl_m']), (0, 0, 0))
        self.assertIsNone(zero['time_usec_raw'])

    def test_origin_source_order_and_lifecycle(self):
        first = self.cases['origin_v2'][ORIGIN]
        self.assertTrue(first['fresh'])
        self.assertEqual((first['effective_at'], first['read_at']), (100.13, 100.15))
        self.assertIs(first['age_is_upper_bound'], True)
        rejected = self.cases['origin_rejected'][ORIGIN]
        self.assertEqual((rejected['wire_rejected'], rejected['source_rejected'], rejected['order_rejected']), (4, 2, 1))
        stale = self.cases['origin_stale']
        self.assertTrue(stale[LOCAL]['fresh'])
        self.assertFalse(stale[ORIGIN]['fresh'])
        for field in ('data', 'effective_at', 'read_at', 'observation_id'):
            self.assertEqual(rejected[field], first[field])
            self.assertEqual(stale[ORIGIN][field], first[field])
        rebound = self.cases['origin_rebound'][ORIGIN]
        self.assertTrue(rebound['fresh'])
        self.assertGreater(rebound['session_epoch'], first['session_epoch'])
        self.assertIsNone(self.cases['origin_rebound'][LOCAL]['data'])
        for name in ('origin_lost', 'origin_reset'):
            self.assertFalse(self.cases[name][ORIGIN]['present'])
            self.assertIsNone(self.cases[name][ORIGIN]['data'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--driver', required=True)
    parser.add_argument('--plugin', required=True)
    args = parser.parse_args()
    unittest.main(argv=[__file__], verbosity=2)
