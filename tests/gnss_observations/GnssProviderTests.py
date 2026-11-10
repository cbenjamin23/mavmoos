#!/usr/bin/env python3
"""Check real system_status DSO publications from the focused GNSS vectors."""
import argparse
import json
import subprocess
import sys
import unittest

PRIMARY = 'MAVMOOS_GPS_STATUS'
SECONDARY = 'MAVMOOS_GPS2_STATUS'
RTK1 = 'MAVMOOS_GPS_RTK'
RTK2 = 'MAVMOOS_GPS2_RTK'
SUMMARY = 'MAVMOOS_SYSTEM_STATUS'


class GnssTests(unittest.TestCase):
    def sample(self, case, key):
        return self.records[case, key]

    def test_sources_and_primary_receiver_are_independent(self):
        for key in (PRIMARY, SECONDARY, RTK1, RTK2):
            self.assertFalse(self.sample('gnss_absent', key)['present'])
            self.assertFalse(self.sample('gnss_prebind', key)['present'])
        primary = self.sample('gnss_bound', PRIMARY)
        self.assertEqual(primary['data']['receiver'], 'GPS_RAW_INT_primary')
        self.assertEqual(primary['data']['remote_time_usec'], '42')
        self.assertNotIn('dgps_satellites_reported_raw', primary['data'])
        secondary = self.sample('gnss_bound', SECONDARY)
        rtk1 = self.sample('gnss_bound', RTK1)
        rtk2 = self.sample('gnss_bound', RTK2)
        aggregate = self.sample('gnss_bound', SUMMARY)
        self.assertEqual(aggregate['gps'], primary)
        self.assertEqual((aggregate['gps2'], aggregate['gps_rtk'], aggregate['gps2_rtk']), (secondary, rtk1, rtk2))
        self.assertEqual(len({sample['observation_id'] for sample in (primary, secondary, rtk1, rtk2)}), 4)
        for sample in (secondary, rtk1, rtk2):
            self.assertTrue(sample['read_only'])
            self.assertFalse(sample['authoritative_for_control'])
            self.assertEqual((sample['system_id'], sample['component_id'], sample['autopilot']), (1, 1, 3))
            self.assertEqual(sample['source_age'], 'unknown')

    def test_secondary_mavlink_units_and_unmapped_clock(self):
        data = self.sample('gnss_bound', SECONDARY)['data']
        self.assertEqual(data['remote_time_usec'], '18446744073709551000')
        self.assertEqual(data['remote_clock'], 'unmapped_boot_or_unix')
        self.assertEqual((data['dgps_satellites_reported_raw'], data['dgps_age_reported_ms']), (7, 351))
        self.assertEqual(data['dgps_availability'], 'reported_unassessed')
        for key, value, unit in (
            ('latitude_deg', 42.3584561, 'deg'), ('longitude_deg', -71.0875891, 'deg'),
            ('altitude_msl_m', -1.234, 'm'), ('hdop', 1.23, 'ratio'), ('vdop', 2.34, 'ratio'),
            ('groundspeed_m_s', 3.45, 'm/s'), ('course_deg', 359.99, 'deg'), ('yaw_deg', 0, 'deg'),
            ('horizontal_accuracy_m', 1.2, 'm'), ('ellipsoid_altitude_m', 35, 'm'),
            ('track_accuracy_deg', 3.45, 'deg'),
        ):
            with self.subTest(field=key):
                self.assertTrue(data[key]['known'])
                self.assertAlmostEqual(data[key]['value'], value)
                self.assertEqual(data[key]['unit'], unit)

    def test_absent_extensions_and_sentinels_stay_unknown(self):
        unknown = self.sample('gnss_sentinels', SECONDARY)['data']
        for key in ('latitude_deg', 'longitude_deg', 'altitude_msl_m', 'hdop', 'vdop',
                    'groundspeed_m_s', 'course_deg', 'yaw_deg', 'satellites', 'horizontal_accuracy_m'):
            self.assertIsNone(unknown[key]['value'])
        self.assertEqual(unknown['yaw_deg']['reason'], 'configured_but_unavailable')
        self.assertEqual(unknown['dgps_availability'], 'zero_or_unreported')
        legacy = self.sample('gnss_mavlink1', SECONDARY)['data']
        for key in ('yaw_deg', 'horizontal_accuracy_m', 'ellipsoid_altitude_m'):
            self.assertEqual(legacy[key]['reason'], 'extension_not_on_wire')
        trimmed = self.sample('gnss_mavlink2_trimmed', SECONDARY)['data']
        self.assertEqual(trimmed['yaw_deg']['value'], .9)
        self.assertEqual(trimmed['ellipsoid_altitude_m']['reason'], 'extension_not_on_wire')
        for key in ('latitude_deg', 'longitude_deg', 'altitude_msl_m', 'groundspeed_m_s', 'course_deg'):
            self.assertEqual(trimmed[key]['value'], 0)
        malformed = self.sample('gnss_invalid_fields', SECONDARY)
        self.assertFalse(malformed['encoding_valid'])
        self.assertIsNone(malformed['data']['latitude_deg']['value'])

    def test_signed_baselines_are_relative_and_units_remain_explicit(self):
        first = self.sample('gnss_bound', RTK1)['data']
        second = self.sample('gnss_bound', RTK2)['data']
        self.assertEqual(first['baseline_axes'], 'ECEF_XYZ')
        self.assertEqual(first['baseline_mm_raw'], [-1500, 2750, -990])
        self.assertEqual(first['baseline_m'], [-1.5, 2.75, -.99])
        self.assertEqual(second['baseline_axes'], 'NORTH_EAST_DOWN')
        self.assertEqual(second['baseline_m'], [3.1, 2.75, -.99])
        self.assertEqual(first['accuracy_raw'], 2**32 - 1)
        self.assertEqual(first['accuracy_unit'], 'unspecified')
        self.assertEqual(first['iar_num_hypotheses_raw'], -1)
        self.assertEqual(first['receiver_metadata'], 'zero_can_be_unimplemented')
        self.assertEqual((first['time_last_baseline_ms_raw'], first['rtk_receiver_id_raw'], first['rtk_health_raw']), (0, 0, 0))
        self.assertEqual((second['gps_week_raw'], second['gps_time_of_week_ms_raw']), (2365, 456789123))
        self.assertEqual((second['time_last_baseline_ms_raw'], second['rtk_receiver_id_raw'], second['rtk_health_raw']), (1500, 9, 11))
        invalid = self.sample('gnss_invalid_fields', RTK1)
        self.assertFalse(invalid['encoding_valid'])
        self.assertEqual(invalid['data']['baseline_coords_type_raw'], 7)
        self.assertEqual(invalid['data']['baseline_mm_raw'], [-1500, 2750, -990])
        self.assertIsNone(invalid['data']['baseline_axes'])
        self.assertIsNone(invalid['data']['baseline_m'])

    def test_ages_reordering_and_sessions(self):
        first = self.sample('gnss_bound', SECONDARY)
        repeated = self.sample('gnss_republish', SECONDARY)
        self.assertEqual(first['effective_at'], 100.3)
        self.assertEqual(first['read_at'], 100.4)
        self.assertEqual(repeated['observation_id'], first['observation_id'])
        self.assertGreater(repeated['age_s'], first['age_s'])
        self.assertTrue(first['age_is_upper_bound'])
        self.assertEqual(self.sample('gnss_bound', RTK2)['age_basis'], 'READ_ACQUISITION')
        for key in (PRIMARY, SECONDARY, RTK1):
            self.assertFalse(self.sample('gnss_independent_age', key)['fresh'])
        self.assertTrue(self.sample('gnss_independent_age', RTK2)['fresh'])
        for case in ('gnss_lapse', 'gnss_after_lapse_data', 'gnss_rebind_old', 'gnss_reset'):
            for key in (SECONDARY, RTK1, RTK2):
                self.assertFalse(self.sample(case, key)['present'])
                self.assertIsNone(self.sample(case, key)['data'])
        rebound = self.sample('gnss_rebind_fresh', SECONDARY)
        self.assertTrue(rebound['fresh'])
        self.assertGreater(rebound['session_epoch'], first['session_epoch'])
        self.assertFalse(self.sample('gnss_rebind_fresh', RTK1)['present'])
        before = self.sample('gnss_before_rejection', SECONDARY)
        after = self.sample('gnss_after_rejection', SECONDARY)
        self.assertEqual(after['data'], before['data'])
        self.assertEqual(after['observation_id'], before['observation_id'])
        counts = self.sample('gnss_after_rejection', SUMMARY)
        self.assertEqual((counts['out_of_order_rejected'], counts['source_rejected'], counts['wire_shape_rejected']), (1, 1, 2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--driver', required=True)
    parser.add_argument('--module', required=True)
    args = parser.parse_args()
    result = subprocess.run([args.driver, args.module], capture_output=True, text=True, check=True, timeout=15)
    GnssTests.records = {(row['case'], row['key']): row['value'] for row in map(json.loads, result.stdout.splitlines())
                         if row['case'].startswith('gnss_')}
    unittest.main(argv=[sys.argv[0]])
