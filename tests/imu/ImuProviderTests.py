#!/usr/bin/env python3
"""Semantic checks against the actual startup-loaded read-only IMU module."""
import argparse
import collections
import json
import math
from pathlib import Path
import subprocess
import unittest

parser=argparse.ArgumentParser()
parser.add_argument('--driver',required=True)
parser.add_argument('--plugin',required=True)
parser.add_argument('--trace')
args=parser.parse_args()
run=subprocess.run([args.driver,args.plugin],capture_output=True,text=True,timeout=30)
if run.returncode:raise RuntimeError(f'IMU driver exit{run.returncode}: {run.stderr}')
if args.trace:
    with Path(args.trace).open('x') as stream:stream.write(run.stdout)
def invalid(value):raise ValueError('nonfinite JSON: '+value)
def floating(value):
    result=float(value)
    if not math.isfinite(result):raise ValueError('nonfinite float: '+value)
    return result
rows=[json.loads(line,parse_constant=invalid,parse_float=floating) for line in run.stdout.splitlines()]
cases=collections.defaultdict(list)
for row in rows:cases[row['case']].append(row)
def records(case,key):return [r['value'] for r in cases[case] if r['key']==key]
def one(case,key):
    result=records(case,key)
    if len(result)!=1:raise AssertionError(f'{case}/{key}: expected1, got{len(result)}')
    return result[0]
def status(case):return one(case,'MAVMOOS_IMU_STATUS')
def sensors(case):return one(case,'MAVMOOS_IMU_SENSORS')['sensors']
def sensor(case):
    result=sensors(case)
    if len(result)!=1:raise AssertionError(f'{case}: expected1 sensor, got{len(result)}')
    return result[0]
def data(case):return sensor(case)['data']
def vibration(case):return one(case,'MAVMOOS_IMU_VIBRATION')

class ImuProviderTests(unittest.TestCase):
    def test_actual_factory_and_read_only_descriptor(self):
        self.assertEqual(one('driver_complete','summary')['effects'],0)
        self.assertGreater(one('driver_complete','summary')['abi'],0)
        self.assertGreater(len(rows),100)
        self.assertTrue(all(r['key'] in ('MAVMOOS_IMU_SENSORS','MAVMOOS_IMU_STATUS','MAVMOOS_IMU_VIBRATION','summary') for r in rows))
        s=status('raw');self.assertTrue(s['read_only']);self.assertFalse(s['authoritative_for_control'])

    def test_absence_is_not_zero_sensor_data(self):
        for case in ('absent','unbound','foreign_first_heartbeat','preconstruction_heartbeat'):
            self.assertEqual(sensors(case),[])
            self.assertFalse(one(case,'MAVMOOS_IMU_SENSORS')['present'])
        self.assertEqual(status('absent')['state'],'WAITING_HEARTBEAT')
        self.assertTrue(sensor('postfloor_bind')['fresh'])

    def test_original_local_times_and_cached_age(self):
        first=sensor('raw');cached=sensor('cached')
        self.assertAlmostEqual(first['effective_at'],100.12);self.assertAlmostEqual(first['read_at'],100.3)
        self.assertAlmostEqual(first['age_s'],.18);self.assertTrue(first['age_is_upper_bound'])
        self.assertEqual(first['age_basis'],'QUEUE_EMPTY_FLOOR')
        self.assertEqual(first['source_age'],'unknown');self.assertEqual(first['source_quality'],'unassessed')
        for field in ('observation_id','effective_at','read_at','data'):self.assertEqual(first[field],cached[field])
        self.assertAlmostEqual(cached['age_s'],.78);self.assertFalse(sensor('stale')['fresh'])

    def test_ardupilot_raw_acceleration_and_gyro_scaling(self):
        d=data('raw')
        self.assertEqual(d['raw_acceleration'],[1000,-2000,0])
        self.assertEqual(d['raw_angular_velocity'],[1000,-2000,125])
        self.assertEqual(d['raw_units'],{'acceleration':'milli-g','angular_velocity':'mrad/s','magnetic_field':'milligauss'})
        self.assertEqual(d['acceleration']['unit'],'m/s^2')
        self.assertEqual(d['angular_velocity']['unit'],'rad/s')
        for got,want in zip(d['acceleration']['value'],[9.80665,-19.6133,0]):self.assertAlmostEqual(got,want)
        self.assertEqual(d['angular_velocity']['value'],[1,-2,.125])
        self.assertEqual(d['unit_basis'],'ardupilot_raw_imu_scaled_sender_convention')

    def test_magnetic_milligauss_to_tesla_is_not_mavros_legacy_coefficient(self):
        d=data('raw');self.assertEqual(d['magnetic_field']['unit'],'T')
        self.assertEqual(d['raw_magnetic_field'],[1000,-250,0])
        for got,want in zip(d['magnetic_field']['value'],[.0001,-.000025,0]):self.assertAlmostEqual(got,want,places=14)

    def test_sensor_report_does_not_infer_orientation_presence_health(self):
        d=data('raw')
        self.assertEqual(d['body_frame'],'BODY_FRD');self.assertEqual(d['estimate'],'sensor_report_not_fused_attitude')
        self.assertEqual(d['sensor_presence'],'unknown');self.assertEqual(d['sensor_health'],'unassessed')
        self.assertEqual(d['physical_accuracy'],'unassessed');self.assertIsNone(d['covariance'])
        self.assertFalse(d['gravity_removed']);self.assertNotIn('orientation',d)

    def test_families_are_separate_even_when_they_may_share_hardware(self):
        values=sensors('four_families');self.assertEqual([v['data']['message_id'] for v in values],[26,27,116,129])
        self.assertEqual([v['data']['sensor_instance'] for v in values],[0,0,1,2])
        self.assertEqual(status('four_families')['family_selection'],'independent_no_precedence_or_fusion')
        self.assertFalse(status('four_families')['atomic'])
        self.assertEqual([v['fresh'] for v in sensors('independent_ages')],[False,False,True,True])
        for record in values:
            self.assertEqual(record['data']['physical_sensor_identity'],'unverified')
            self.assertEqual(record['data']['acceleration']['value'],[9.80665,-19.6133,0])

    def test_raw_ids_do_not_claim_distinct_physical_devices(self):
        values=sensors('raw_identities');self.assertEqual([v['data']['sensor_instance'] for v in values],[0,3])
        self.assertEqual(values[0]['data']['identity_basis'],'zero_default_or_omitted_extension_id')
        self.assertEqual(values[1]['data']['identity_basis'],'reported_extension_id')

    def test_temperature_unknown_zero_and_protocol_one_ambiguity(self):
        for family in (27,26,116,129):
            zero=data(f'temperature_{family}_0')['temperature'];self.assertFalse(zero['known']);self.assertIsNone(zero['value'])
            one_degree=data(f'temperature_{family}_1')['temperature'];self.assertTrue(one_degree['known'])
            self.assertEqual(one_degree['value'],.01);self.assertIn('ambiguous',one_degree['reason'])
            self.assertEqual(data(f'temperature_{family}_-4000')['temperature']['value'],-40)
            self.assertEqual(data(f'temperature_{family}_32767')['temperature']['value'],327.67)

    def test_mavlink1_has_no_temperature_or_raw_sensor_id_extension(self):
        for family in (27,26,116,129):
            d=data(f'v1_{family}');self.assertEqual(d['wire_version'],1)
            self.assertFalse(d['temperature']['known']);self.assertIsNone(d['temperature']['raw_cdegC'])
        self.assertEqual(data('v1_27')['sensor_instance'],0)
        self.assertEqual(data('v1_27')['identity_basis'],'mavlink1_no_id_extension')

    def test_mavlink2_trailing_zero_payload_is_valid_wire_encoding(self):
        d=data('v2_zero_tail');self.assertEqual(d['payload_length'],1)
        self.assertEqual(d['raw_acceleration'],[0,0,0]);self.assertEqual(d['raw_magnetic_field'],[0,0,0])
        self.assertFalse(d['temperature']['known']);self.assertTrue(sensor('v2_zero_tail')['encoding_valid'])
        self.assertEqual(d['sensor_presence'],'unknown')

    def test_remote_clock_is_not_a_local_freshness_clock(self):
        d=data('max_remote_stamp');self.assertEqual(d['remote_timestamp'],2**64-1)
        self.assertEqual(d['remote_clock'],'ardupilot_boot_us_unmapped_uint64')
        for case in ('remote_clock_regression','remote_ms_wrap'):
            self.assertTrue(data(case)['remote_timestamp_regressed']);self.assertTrue(sensor(case)['fresh'])
            self.assertEqual(sensor(case)['source_age'],'unknown')

    def test_out_of_order_effective_or_read_time_cannot_replace(self):
        before=sensor('before_order_reject');after=sensor('order_reject')
        for field in ('observation_id','data','effective_at','read_at'):self.assertEqual(before[field],after[field])
        self.assertEqual(status('order_reject')['out_of_order_rejected'],2)

    def test_invalid_metadata_wire_shape_and_source_are_rejected(self):
        self.assertEqual(sensors('metadata_reject'),[]);self.assertEqual(status('metadata_reject')['metadata_rejected'],7)
        self.assertEqual(sensors('wire_reject'),[]);self.assertEqual(status('wire_reject')['wire_shape_rejected'],5)
        self.assertEqual(len(sensors('foreign_sources')),1);self.assertEqual(status('foreign_sources')['source_rejected'],6)
        self.assertEqual(data('foreign_sources')['remote_timestamp'],123456789)

    def test_reset_requires_post_floor_heartbeat_and_new_sensor(self):
        for case in ('reset','old_floor_after_reset','rebound_empty'):self.assertEqual(sensors(case),[])
        self.assertEqual(status('reset')['reset_reason'],'quotes" newline\n reset')
        self.assertTrue(sensor('new_after_reset')['fresh'])
        self.assertGreater(status('new_after_reset')['session_epoch'],status('raw')['session_epoch'])

    def test_heartbeat_loss_clears_instead_of_freshening_cached_sensor(self):
        for case in ('heartbeat_lapse','lapse_rebound','host_heartbeat_invalid'):self.assertEqual(sensors(case),[])
        self.assertEqual(status('heartbeat_lapse')['reset_reason'],'heartbeat_expired')

    def test_clock_fault_retains_high_water_and_requires_valid_reset(self):
        for case in ('clock_latched','clock_not_auto_recovered'):
            self.assertEqual(sensors(case),[]);self.assertEqual(status(case)['state'],'CLOCK_FAULT')
        self.assertTrue(sensor('clock_explicit_recovery')['fresh'])
        self.assertAlmostEqual(status('clock_explicit_recovery')['session_boundary'],101.5)
        for s in records('invalid_reset','MAVMOOS_IMU_STATUS'):self.assertEqual(s['state'],'CLOCK_FAULT')

    def test_nonfinite_and_invalid_initial_clocks_do_not_become_valid_by_waiting(self):
        for case in ('invalid_initial_clock','nonfinite_owner_clock'):
            for s in records(case,'MAVMOOS_IMU_STATUS'):
                self.assertEqual(s['state'],'CLOCK_FAULT');self.assertEqual(s['sensors']['sensors'],[])
        self.assertEqual(len(records('initial_clock_explicit_recovery','MAVMOOS_IMU_SENSORS')),3)
        for s in records('initial_clock_explicit_recovery','MAVMOOS_IMU_SENSORS'):self.assertTrue(s['sensors'][0]['fresh'])

    def test_capacity_rejects_new_keys_without_eviction_or_blocking_known_key(self):
        before=sensors('capacity_eight');after=sensors('capacity_reject');updated=sensors('capacity_existing_update')
        self.assertEqual(len(before),8);self.assertEqual(len(after),8);self.assertEqual(before[0]['data'],after[0]['data'])
        self.assertEqual(status('capacity_reject')['sensor_capacity_rejected'],2)
        self.assertEqual(updated[0]['data']['remote_timestamp'],999)
        self.assertGreater(updated[0]['observation_id'],before[0]['observation_id'])

    def test_publish_cadence_and_configured_age_remain_bounded(self):
        self.assertEqual(cases['interval_suppressed'],[]);self.assertEqual(len(sensors('interval_later')),2)
        self.assertFalse(sensor('short_timeout')['fresh'])
        self.assertTrue(sensor('age_exact_limit')['fresh']);self.assertFalse(sensor('age_over_limit')['fresh'])

    def test_rover_source_and_deferred_highres_behavior(self):
        self.assertTrue(sensor('rover_binding')['fresh'])
        self.assertEqual(sensors('deferred_highres_ignored'),[])
        self.assertEqual(status('deferred_highres_ignored')['metadata_rejected'],0)

    def test_bounded_adversarial_frames_stay_finite_json(self):
        self.assertLessEqual(len(sensors('bounded_random_frames')),8)
        self.assertGreater(status('bounded_random_frames')['wire_shape_rejected'],0)
        self.assertGreater(status('bounded_random_frames')['sensor_capacity_rejected'],0)

    def test_vibration_wire_fields_and_read_only_source(self):
        report=vibration('vibration_valid')
        self.assertTrue(report['present'] and report['fresh'] and report['read_only'])
        self.assertFalse(report['authoritative_for_control'])
        self.assertEqual((report['system_id'],report['component_id'],report['autopilot']),(1,1,3))
        self.assertEqual(report['source_age'],'unknown')
        self.assertEqual(report['age_basis'],'QUEUE_EMPTY_FLOOR')
        self.assertTrue(report['age_is_upper_bound'])
        d=report['data']
        self.assertEqual(d['message_id'],241)
        self.assertEqual(d['time_usec_raw'],str(2**64-1))
        self.assertEqual(d['vibration_m_s2'],[1.25,2.5,3.75])
        self.assertEqual(d['clipping_counts'],[7,2**32-1,0])
        self.assertEqual(d['clipping_sensor_presence'],'unknown')
        self.assertEqual(d['sensor_health'],'unassessed')

    def test_vibration_retains_original_time_and_expires_independently(self):
        first=vibration('vibration_valid');cached=vibration('vibration_cached');stale=vibration('vibration_stale')
        self.assertEqual(first['effective_at'],100.2);self.assertEqual(first['read_at'],100.35)
        self.assertAlmostEqual(cached['age_s'],.7)
        for field in ('observation_id','data','effective_at','read_at'):
            self.assertEqual(first[field],cached[field]);self.assertEqual(first[field],stale[field])
        self.assertFalse(stale['fresh']);self.assertTrue(sensor('vibration_stale')['fresh'])
        regressed=vibration('vibration_remote_regression')
        self.assertTrue(regressed['fresh']);self.assertTrue(regressed['data']['remote_timestamp_regressed'])
        self.assertEqual(regressed['data']['time_usec_raw'],'1')
        self.assertEqual(regressed['session_epoch'],first['session_epoch'])
        old_imu=sensor('vibration_stale');new_imu=sensor('vibration_remote_regression')
        for field in ('observation_id','data','effective_at','read_at'):
            self.assertEqual(old_imu[field],new_imu[field])
        self.assertGreater(new_imu['age_s'],old_imu['age_s'])
        self.assertEqual(sensor('vibration_stale')['observation_id'],2)
        self.assertEqual(regressed['observation_id'],2)

    def test_vibration_rejects_malformed_values_and_foreign_or_reordered_frames(self):
        before=vibration('vibration_before_rejected');after=vibration('vibration_after_rejected')
        for field in ('observation_id','data','effective_at','read_at'):
            self.assertEqual(before[field],after[field])
        rejected=status('vibration_after_rejected')
        self.assertEqual(rejected['source_rejected'],2)
        self.assertEqual(rejected['out_of_order_rejected'],3)
        self.assertEqual(rejected['wire_shape_rejected'],7)

    def test_vibration_wire_versions_and_new_session(self):
        v1=vibration('vibration_v1')['data'];trimmed=vibration('vibration_trimmed_v2')['data']
        self.assertEqual((v1['wire_version'],v1['payload_length']),(1,32))
        self.assertEqual((trimmed['wire_version'],trimmed['payload_length']),(2,1))
        self.assertEqual(trimmed['vibration_m_s2'],[0,0,0]);self.assertEqual(trimmed['clipping_counts'],[0,0,0])
        for case in ('vibration_unbound','vibration_heartbeat_lapse','vibration_reset','vibration_rebind_required'):
            report=vibration(case)
            self.assertFalse(report['present'] or report['fresh'])
            for field in ('data','effective_at','read_at','age_s','system_id','component_id','autopilot'):
                self.assertIsNone(report[field])
        rebound=vibration('vibration_rebound');recovered=vibration('vibration_recovered')
        self.assertTrue(rebound['fresh'] and recovered['fresh'])
        self.assertGreater(rebound['session_epoch'],vibration('vibration_v1')['session_epoch'])
        self.assertGreater(recovered['session_epoch'],rebound['session_epoch'])

if __name__=='__main__':unittest.main(argv=['ImuProviderTests.py'],verbosity=2)
