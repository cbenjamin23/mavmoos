#!/usr/bin/env python3
"""Exercise the actual gimbal observer module with encoded MAVLink vectors."""
import argparse
import json
import subprocess
import sys
import unittest

INFORMATION = 'MAVMOOS_GIMBAL_DEVICE_INFORMATION'
ATTITUDE = 'MAVMOOS_GIMBAL_DEVICE_ATTITUDE_STATUS'


class GimbalTests(unittest.TestCase):
    def record(self, case, family):
        return self.records[case, family]

    def device(self, case, family, ident=0):
        return next(row for row in self.record(case, family)['devices'] if row['device_id'] == ident)

    def test_only_selected_component_on_fc_session(self):
        for case in ('component_only', 'foreign_or_old', 'cleared', 'reset'):
            for family in (INFORMATION, ATTITUDE):
                record = self.record(case, family)
                self.assertFalse(record['present'])
                self.assertEqual(record['devices'], [])
        for family in (INFORMATION, ATTITUDE):
            record = self.record('bound', family)
            self.assertTrue(record['read_only'])
            self.assertFalse(record['authoritative_for_control'])
            self.assertFalse(record['atomic'])
            stamp = self.device('bound', family)['metadata']
            self.assertEqual((stamp['system_id'], stamp['component_id'], stamp['autopilot']), (1, 154, 3))
            self.assertTrue(stamp['fresh'])
            self.assertEqual(stamp['source_age'], 'unknown')
        self.assertGreater(self.device('rebound', ATTITUDE)['metadata']['session_epoch'],
                           self.device('bound', ATTITUDE)['metadata']['session_epoch'])
        self.assertFalse(self.record('rebound', INFORMATION)['present'])

    def test_raw_device_information_and_capabilities(self):
        data = self.device('bound', INFORMATION)['data']
        self.assertEqual(data['uid_raw'], '18446744073709551000')
        self.assertTrue(data['uid_known'])
        self.assertEqual(data['vendor'], {'text': 'Gimbal"Co', 'raw_hex': '47696d62616c22436f'})
        self.assertEqual(data['model']['text'], 'X' * 32)
        self.assertEqual(data['custom_name']['raw_hex'], '43ff')
        self.assertEqual(data['firmware_version'], {'raw': 0x04030201, 'major': 1, 'minor': 2, 'patch': 3, 'dev': 4})
        self.assertEqual(data['capability_basis'], 'cap_flags2')
        self.assertTrue(data['cap_flags_lower_bits_match'])
        self.assertIn('point_global', data['capabilities']['asserted'])
        self.assertEqual(data['capabilities']['unknown_mask'], 0x80000000)
        self.assertIsNone(data['angular_limits_rad']['pitch']['min']['value'])
        self.assertIsNone(data['angular_limits_rad']['pitch']['ordered'])
        self.assertFalse(data['angular_limits_rad']['yaw']['ordered'])
        secondary = self.device('two_devices', INFORMATION, 2)['data']
        self.assertFalse(secondary['uid_known'])
        self.assertFalse(secondary['cap_flags_lower_bits_match'])

    def test_quaternion_rates_yaw_frames_and_unavailable_extensions(self):
        data = self.device('bound', ATTITUDE)['data']
        self.assertEqual(data['quaternion_wxyz'], [.5, -.5, .5, -.5])
        self.assertTrue(data['quaternion_unit_within_tolerance'])
        self.assertEqual((data['yaw_frame'], data['yaw_frame_basis']), ('earth_north', 'explicit_flag'))
        self.assertEqual(data['flags']['unknown_mask'], 0x8000)
        self.assertEqual(data['failure_flags']['unknown_mask'], 0x80000000)
        self.assertIn('calibrating', data['failure_flags']['asserted'])
        self.assertIsNone(data['angular_velocity_rad_s']['pitch_y']['value'])
        self.assertEqual(data['angular_velocity_rad_s']['pitch_y']['state'], 'wire_nan')
        self.assertEqual(data['delta_yaw_rad']['value'], -.25)
        self.assertEqual(data['delta_yaw_velocity_rad_s']['state'], 'wire_nan')
        legacy = self.device('two_devices', ATTITUDE, 2)['data']
        self.assertEqual(legacy['yaw_frame_basis'], 'legacy_yaw_lock')
        self.assertEqual(legacy['delta_yaw_rad']['raw'], 1)
        self.assertIsNone(legacy['delta_yaw_rad']['value'])
        conflict = self.device('conflicting', ATTITUDE, 2)['data']
        self.assertEqual(conflict['yaw_frame'], 'conflicting')
        self.assertIsNone(conflict['quaternion_wxyz'][2])
        self.assertEqual(conflict['quaternion_component_states'][2], 'invalid_positive_infinity')
        self.assertIsNone(conflict['quaternion_unit_within_tolerance'])
        absent = self.device('expiry_and_malformed', ATTITUDE)['data']
        self.assertEqual(absent['yaw_frame'], 'vehicle_heading')
        self.assertIsNone(absent['gimbal_device_id_raw'])
        self.assertEqual(absent['delta_yaw_rad']['state'], 'extension_not_on_wire')

    def test_family_device_age_and_malformed_wire(self):
        self.assertEqual([row['device_id'] for row in self.record('two_devices', INFORMATION)['devices']], [0, 2])
        before = self.device('bound', ATTITUDE)['metadata']
        repeated = self.device('republish', ATTITUDE)['metadata']
        self.assertEqual(before['observation_id'], repeated['observation_id'])
        self.assertEqual(before['effective_at'], repeated['effective_at'])
        self.assertEqual(before['read_at'], repeated['read_at'])
        self.assertGreater(repeated['age_s'], before['age_s'])
        self.assertTrue(before['age_is_upper_bound'])
        self.assertFalse(self.device('expiry_and_malformed', INFORMATION)['metadata']['fresh'])
        self.assertTrue(self.device('expiry_and_malformed', ATTITUDE)['metadata']['fresh'])
        status = self.record('expiry_and_malformed', ATTITUDE)['status']
        self.assertEqual(status['wire_rejected'], 2)
        self.assertGreaterEqual(status['order_rejected'], 1)
        self.assertGreaterEqual(status['source_rejected'], 5)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--driver', required=True)
    parser.add_argument('--module', required=True)
    args = parser.parse_args()
    result = subprocess.run([args.driver, args.module], check=True, capture_output=True, text=True, timeout=15)
    GimbalTests.records = {(row['case'], row['key']): row['value'] for row in map(json.loads, result.stdout.splitlines())}
    unittest.main(argv=[sys.argv[0]])
