#!/usr/bin/env python3
"""Semantic assertions for an actual distance_sensor MODULE and fake Services."""
import argparse
import collections
import json
import math
from pathlib import Path
import subprocess
import unittest

parser=argparse.ArgumentParser()
parser.add_argument('--driver',required=True)
parser.add_argument('--module',required=True)
parser.add_argument('--trace')
args,remaining=parser.parse_known_args()
run=subprocess.run([args.driver,args.module],capture_output=True,text=True,timeout=15)
if run.returncode:raise RuntimeError('real MODULE scenario driver failed: '+run.stderr)
if args.trace:
    path=Path(args.trace)
    if path.exists():raise RuntimeError('refuse existing trace: '+str(path))
    path.write_text(run.stdout)
def reject(value):raise ValueError('nonfinite JSON constant '+value)
def floating(value):
    number=float(value)
    if not math.isfinite(number):raise ValueError('nonfinite JSON float '+value)
    return number
def unique(pairs):
    result={}
    for key,value in pairs:
        if key in result:raise ValueError('duplicate JSON key '+key)
        result[key]=value
    return result
rows=[json.loads(line,parse_constant=reject,parse_float=floating,object_pairs_hook=unique) for line in run.stdout.splitlines()]
cases=collections.defaultdict(list)
for row in rows:cases[row['case']].append(row)
def records(case,key):return [r['value'] for r in cases[case] if r['key']==key]
def one(case,key):
    values=records(case,key)
    if len(values)!=1:raise AssertionError(f'{case}/{key}: expected one, got {len(values)}')
    return values[0]
def aggregate(case):return one(case,'MAVMOOS_DISTANCE_SENSORS')
def status(case):return one(case,'MAVMOOS_DISTANCE_STATUS')
def sensor(case,id=0):
    values=[r for r in aggregate(case)['sensors'] if r['data']['id']==id]
    if len(values)!=1:raise AssertionError(f'{case}/sensor {id}: expected one, got {len(values)}')
    return values[0]
def data(case,id=0):return sensor(case,id)['data']

class DistanceTests(unittest.TestCase):
    def test_actual_module_abi_descriptor_and_zero_effects(self):
        self.assertEqual(one('driver_complete','summary'),{'effects':0,'abi':10,'random_frames':2048})
        self.assertTrue(all(r['key'] in ('MAVMOOS_DISTANCE_SENSORS','MAVMOOS_DISTANCE_STATUS','MAVMOOS_OBSTACLE_DISTANCE','summary') for r in rows))
        obstacle = [r['value'] for r in rows if r['key'] == 'MAVMOOS_OBSTACLE_DISTANCE']
        self.assertTrue(obstacle and all(not r['present'] and r['data'] is None for r in obstacle))
        self.assertTrue(status('normal')['read_only']);self.assertFalse(status('normal')['authoritative_for_control'])

    def test_absent_sensor_is_explicit_not_zero_distance(self):
        for name in ('absent','unbound'):
            a=aggregate(name);self.assertFalse(a['present']);self.assertEqual(a['sensors'],[])
        self.assertEqual(status('absent')['state'],'WAITING_HEARTBEAT')
        self.assertEqual(status('unbound')['source_rejected'],1)

    def test_raw_centimetres_and_metre_quantities_are_distinct(self):
        d=data('normal')
        for key,raw in [('min_distance',20),('max_distance',1000),('current_distance',432)]:
            self.assertEqual(d[key+'_cm'],raw);self.assertEqual(d[key+'_m']['unit'],'m')
            self.assertTrue(d[key+'_m']['known']);self.assertAlmostEqual(d[key+'_m']['value'],raw/100)
        self.assertTrue(d['bounds_consistent']);self.assertTrue(d['current_within_reported_limits'])

    def test_distance_zero_and_uint16max_do_not_acquire_undocumented_sentinels(self):
        self.assertEqual(data('zero_distance')['current_distance_cm'],0)
        self.assertEqual(data('zero_distance')['current_distance_m']['value'],0)
        self.assertEqual(data('max_distance')['current_distance_cm'],65535)
        self.assertEqual(data('max_distance')['current_distance_m']['value'],655.35)

    def test_range_consistency_does_not_destroy_reported_values(self):
        self.assertFalse(data('reversed_bounds')['bounds_consistent'])
        for name,expected in [('reversed_bounds',75),('below_min',10),('above_max',2000)]:
            d=data(name);self.assertFalse(d['current_within_reported_limits'])
            self.assertEqual(d['current_distance_cm'],expected);self.assertTrue(d['current_distance_m']['known'])
            self.assertTrue(sensor(name)['fresh']);self.assertTrue(sensor(name)['encoding_valid'])

    def test_sensor_report_is_not_height_or_physical_accuracy(self):
        d=data('normal')
        self.assertEqual(d['distance_interpretation'],'sensor_report_no_vertical_or_terrain_projection')
        self.assertEqual(d['physical_accuracy'],'unassessed')
        self.assertEqual(d['orientation_frame'],'BODY_FRD')
        self.assertNotIn('height_above_home',d);self.assertNotIn('terrain_clearance',d)

    def test_ardupilot_zero_covariance_and_wire_unknown_are_not_perfect_variance(self):
        for raw,reason in [(0,'ardupilot_zero_unknown_or_invalid'),(255,'wire_unknown')]:
            value=data(f'covariance_{raw}')['covariance']
            self.assertEqual(value['raw_cm2'],raw);self.assertFalse(value['quantity']['known'])
            self.assertIsNone(value['quantity']['value']);self.assertEqual(value['quantity']['reason'],reason)

    def test_covariance_square_centimetres_convert_to_square_metres(self):
        for raw in (1,25,254):
            q=data(f'covariance_{raw}')['covariance']['quantity']
            self.assertTrue(q['known']);self.assertEqual(q['unit'],'m^2');self.assertAlmostEqual(q['value'],raw/10000)

    def test_signal_quality_unknown_invalid_and_outside_percentage_range(self):
        for raw,reason in [(0,'unknown_unset_or_omitted'),(1,'invalid_signal'),(101,'outside_percentage_range'),(255,'outside_percentage_range')]:
            q=data(f'quality_{raw}')['signal_quality']
            self.assertEqual(q['raw'],raw);self.assertFalse(q['quantity']['known'])
            self.assertIsNone(q['quantity']['value']);self.assertEqual(q['quantity']['reason'],reason)

    def test_reported_quality_percentage_is_preserved_without_readiness(self):
        for raw in (2,100):
            q=data(f'quality_{raw}')['signal_quality']['quantity']
            self.assertTrue(q['known']);self.assertEqual(q['value'],raw);self.assertEqual(q['unit'],'percent')
        self.assertTrue(sensor('quality_1')['fresh'])

    def test_known_enum_and_unknown_technology_are_distinct(self):
        self.assertEqual(data('normal')['type_name'],'LASER');self.assertTrue(data('normal')['type_known'])
        self.assertEqual(data('custom_unit')['type_name'],'UNKNOWN');self.assertTrue(data('custom_unit')['type_known'])
        self.assertEqual(data('unknown_enums')['type_raw'],255);self.assertFalse(data('unknown_enums')['type_known'])

    def test_orientation_ids_and_custom_are_preserved(self):
        for value,name in [(0,'NONE'),(24,'PITCH_90'),(25,'PITCH_270'),(40,'ROLL_90_PITCH_315'),(100,'CUSTOM')]:
            d=data(f'orientation_{value}');self.assertEqual(d['orientation_raw'],value)
            self.assertTrue(d['orientation_known']);self.assertEqual(d['orientation_name'],name)
        for case,value in [('orientation_41',41),('unknown_enums',255)]:
            d=data(case);self.assertFalse(d['orientation_known']);self.assertEqual(d['orientation_raw'],value)

    def test_standard_orientation_does_not_apply_a_quaternion(self):
        q=data('normal')['quaternion'];self.assertTrue(q['unit_norm'])
        self.assertFalse(q['required_for_custom_orientation']);self.assertFalse(q['usable_for_custom_orientation'])
        self.assertEqual(q['reason'],'not_applied_for_standard_orientation')

    def test_custom_orientation_requires_finite_unit_quaternion(self):
        self.assertTrue(data('custom_unit')['quaternion']['usable_for_custom_orientation'])
        self.assertTrue(data('custom_nearunit')['quaternion']['usable_for_custom_orientation'])
        for case in ('custom_zero','custom_nonunit','custom_nan'):
            q=data(case)['quaternion'];self.assertTrue(q['required_for_custom_orientation'])
            self.assertFalse(q['unit_norm']);self.assertFalse(q['usable_for_custom_orientation'])
        self.assertIsNone(data('custom_nan')['quaternion']['raw_wxyz'][0])
        self.assertAlmostEqual(data('custom_nearunit')['quaternion']['raw_wxyz'][0],1.0005,places=6)

    def test_fov_unknown_nonfinite_and_geometric_bounds(self):
        for case,key,reason in [('fov_zero_negative','horizontal_fov','unknown_unset_or_omitted'),('fov_zero_negative','vertical_fov','outside_geometric_range'),('fov_nonfinite','horizontal_fov','nonfinite'),('fov_nonfinite','vertical_fov','nonfinite'),('fov_geometric','horizontal_fov','outside_geometric_range')]:
            item=data(case)[key];self.assertFalse(item['quantity']['known']);self.assertEqual(item['quantity']['reason'],reason)
        self.assertTrue(data('fov_geometric')['vertical_fov']['quantity']['known'])
        self.assertAlmostEqual(data('normal')['horizontal_fov']['quantity']['value'],.5)

    def test_mavlink1_extensions_are_absent_despite_dirty_memory(self):
        d=data('v1_dirty_extensions');self.assertEqual(d['wire_version'],1);self.assertFalse(d['extension_bytes_observed'])
        for key in ('horizontal_fov','vertical_fov'):
            self.assertIsNone(d[key]['raw_rad']);self.assertFalse(d[key]['quantity']['known'])
        self.assertEqual(d['quaternion']['raw_wxyz'],[None]*4)
        self.assertIsNone(d['signal_quality']['raw']);self.assertFalse(d['signal_quality']['extension_available'])
        self.assertFalse(d['quaternion']['usable_for_custom_orientation'])

    def test_mavlink2_omitted_extensions_are_unknown(self):
        d=data('v2_omitted_extensions');self.assertEqual(d['wire_version'],2);self.assertFalse(d['extension_bytes_observed'])
        self.assertEqual(d['horizontal_fov']['raw_rad'],0);self.assertFalse(d['horizontal_fov']['quantity']['known'])
        self.assertEqual(d['quaternion']['raw_wxyz'],[0]*4);self.assertFalse(d['quaternion']['unit_norm'])
        self.assertEqual(d['signal_quality']['raw'],0);self.assertFalse(d['signal_quality']['quantity']['known'])

    def test_partial_extensions_and_zero_trim_decode_without_support_inference(self):
        d=data('partial_extension',1);self.assertTrue(d['extension_bytes_observed'])
        self.assertGreater(d['horizontal_fov']['raw_rad'],0);self.assertFalse(d['vertical_fov']['quantity']['known'])
        d=data('v2_zero_trim');self.assertEqual(d['payload_length'],1);self.assertEqual(d['time_boot_ms'],9)
        self.assertEqual(d['id'],0);self.assertEqual(d['current_distance_cm'],0)

    def test_independent_ids_and_bounded_cache(self):
        self.assertEqual([r['data']['id'] for r in aggregate('ids_sorted')['sensors']],[0,7,255])
        self.assertEqual(aggregate('capacity')['cache_capacity'],8)
        self.assertEqual([r['data']['id'] for r in aggregate('capacity')['sensors']],list(range(8)))
        self.assertEqual(status('capacity')['sensor_capacity_rejected'],2)
        self.assertEqual(data('retained_update',3)['current_distance_cm'],999)
        self.assertEqual([r['data']['id'] for r in aggregate('capacity_reset')['sensors']],[255])

    def test_one_id_update_does_not_renew_another(self):
        for id in (7,255):
            a,b=sensor('ids_sorted',id),sensor('id_update',id)
            self.assertEqual(a['observation_id'],b['observation_id']);self.assertEqual(a['effective_at'],b['effective_at'])
        self.assertGreater(sensor('id_update')['observation_id'],sensor('ids_sorted')['observation_id'])

    def test_local_acquisition_metadata_and_republication_keep_original_stamps(self):
        a,b=sensor('normal'),sensor('cached')
        self.assertAlmostEqual(a['effective_at'],100.12);self.assertAlmostEqual(a['read_at'],100.3)
        self.assertAlmostEqual(a['age_s'],.18);self.assertAlmostEqual(b['age_s'],.78)
        self.assertEqual(a['observation_id'],b['observation_id']);self.assertEqual(a['effective_at'],b['effective_at'])
        self.assertTrue(a['age_is_upper_bound']);self.assertEqual(a['age_basis'],'QUEUE_EMPTY_FLOOR')
        self.assertEqual(a['source_age'],'unknown');self.assertEqual(a['source_quality'],'unassessed')

    def test_samples_expire_independently(self):
        self.assertFalse(sensor('independent_ages')['fresh']);self.assertTrue(sensor('independent_ages',1)['fresh'])
        self.assertFalse(aggregate('independent_ages')['atomic']);self.assertFalse(status('independent_ages')['atomic'])

    def test_remote_uint32_wrap_does_not_reset_session_or_establish_measurement_age(self):
        a,b,c=[sensor(name) for name in ('remote_max','remote_wrap','remote_repeat')]
        self.assertEqual(a['data']['time_boot_ms'],2**32-1);self.assertEqual(b['data']['time_boot_ms'],0)
        self.assertEqual(a['session_epoch'],b['session_epoch']);self.assertGreater(c['observation_id'],b['observation_id'])
        self.assertEqual(b['data']['remote_clock'],'boot_ms_unmapped_uint32')
        self.assertEqual(b['data']['timestamp_semantics'],'ardupilot_message_send_time_not_certified_measurement_time')

    def test_older_metadata_is_rejected_but_distinct_coalesced_frames_update(self):
        a,b,c=[sensor(name) for name in ('order_before','order_after','coalesced')]
        self.assertEqual(a['observation_id'],b['observation_id']);self.assertEqual(status('order_after')['out_of_order_rejected'],2)
        self.assertGreater(c['observation_id'],b['observation_id']);self.assertEqual(c['effective_at'],a['effective_at'])
        self.assertEqual(c['read_at'],a['read_at']);self.assertEqual(c['data']['current_distance_cm'],600)

    def test_impossible_metadata_is_rejected_and_actual_read_basis_is_honest(self):
        self.assertEqual(status('bad_metadata')['metadata_rejected'],8);self.assertEqual(aggregate('bad_metadata')['sensors'],[])
        r=sensor('read_acquisition');self.assertFalse(r['age_is_upper_bound']);self.assertEqual(r['age_basis'],'READ_ACQUISITION')

    def test_foreign_ids_and_identity_changes_cannot_rebind(self):
        self.assertEqual(status('foreign_sources')['source_rejected'],6)
        self.assertEqual(status('foreign_sources')['system_id'],1)
        self.assertEqual(sensor('source_before')['observation_id'],sensor('foreign_sources')['observation_id'])

    def test_heartbeat_lapse_and_shared_invalidity_clear_all_sensors(self):
        for case in ('heartbeat_lapse','heartbeat_only','host_heartbeat_invalid'):
            self.assertFalse(aggregate(case)['present']);self.assertEqual(aggregate(case)['sensors'],[])
        self.assertEqual(status('heartbeat_only')['state'],'BOUND')

    def test_reset_floor_and_reason_are_retained(self):
        for case in ('reset','old_floor','rebound_empty'):self.assertEqual(aggregate(case)['sensors'],[])
        self.assertEqual(status('reset')['reset_reason'],'quotes" newline\n reset')
        self.assertTrue(sensor('after_reset',255)['fresh']);self.assertAlmostEqual(status('after_reset')['session_boundary'],100.4)

    def test_clock_rollback_and_premature_reset_stay_latched(self):
        self.assertTrue(sensor('highwater_before')['present'])
        for case in ('rollback','premature_reset','caught_up_latched'):
            self.assertEqual(status(case)['state'],'CLOCK_FAULT');self.assertEqual(aggregate(case)['sensors'],[])
        self.assertTrue(sensor('clock_recovered')['fresh']);self.assertAlmostEqual(status('clock_recovered')['session_boundary'],101.2)

    def test_nonfinite_negative_reset_and_invalid_construction_fail_closed(self):
        values=records('invalid_reset','MAVMOOS_DISTANCE_STATUS');self.assertEqual(len(values),3)
        self.assertTrue(all(v['state']=='CLOCK_FAULT' and not v['sensors']['sensors'] for v in values))
        for case in ('invalid_start','invalid_start_latched'):self.assertEqual(status(case)['state'],'CLOCK_FAULT')
        self.assertTrue(sensor('invalid_reset_recovered')['fresh']);self.assertTrue(sensor('invalid_start_recovered')['fresh'])

    def test_wire_shape_validation_and_legacy_message_exclusion(self):
        self.assertEqual(status('bad_wire')['wire_shape_rejected'],5);self.assertEqual(status('bad_wire')['metadata_rejected'],0)
        self.assertEqual(aggregate('bad_wire')['sensors'],[])

    def test_local_pacing_and_configured_expiry(self):
        self.assertEqual(cases['pacing_suppressed'],[])
        self.assertEqual(len(aggregate('pacing_later')['sensors']),2)
        self.assertFalse(sensor('short_timeout',1)['fresh'])

    def test_random_decoded_frames_remain_bounded_finite_and_read_only(self):
        values=records('random_frames','MAVMOOS_DISTANCE_STATUS');self.assertTrue(values)
        for v in values:
            self.assertTrue(v['read_only']);self.assertFalse(v['authoritative_for_control'])
            self.assertLessEqual(len(v['sensors']['sensors']),8)
            self.assertEqual([x['data']['id'] for x in v['sensors']['sensors']],sorted(x['data']['id'] for x in v['sensors']['sensors']))

unittest.main(argv=[__file__]+remaining)
