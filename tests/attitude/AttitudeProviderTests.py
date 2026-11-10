#!/usr/bin/env python3
"""Semantic checks of an actual attitude MODULE through fake host Services."""
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
result=subprocess.run([args.driver,args.module],capture_output=True,text=True,timeout=15)
if result.returncode:raise RuntimeError('real MODULE scenario driver failed: '+result.stderr)
if args.trace:
    path=Path(args.trace)
    if path.exists():raise RuntimeError('refuse existing trace: '+str(path))
    path.write_text(result.stdout)
def reject(value):raise ValueError('nonfinite JSON constant '+value)
def floating(value):
    number=float(value)
    if not math.isfinite(number):raise ValueError('nonfinite JSON float '+value)
    return number
rows=[json.loads(line,parse_constant=reject,parse_float=floating) for line in result.stdout.splitlines()]
by_case=collections.defaultdict(list)
for row in rows:by_case[row['case']].append(row)
def records(case,key):return [r['value'] for r in by_case[case] if r['key']==key]
def one(case,key):
    values=records(case,key)
    if len(values)!=1:raise AssertionError(f'{case}/{key}: expected1, got{len(values)}')
    return values[0]
def euler(case):return one(case,'MAVMOOS_ATTITUDE_EULER')
def quaternion(case):return one(case,'MAVMOOS_ATTITUDE_QUATERNION')
def status(case):return one(case,'MAVMOOS_ATTITUDE_STATUS')
def value(record,*fields):
    for field in fields:record=record[field]
    return record

class AttitudeTests(unittest.TestCase):
    def test_actual_factory_rejects_old_abi_and_has_no_effects(self):
        self.assertEqual(one('driver_complete','summary'),{'effects':0,'abi':10})
        self.assertGreater(len(rows),100)
        self.assertTrue(all(r['key'].startswith('MAVMOOS_ATTITUDE_') or r['key']=='summary' for r in rows))

    def test_absent_is_not_a_zero_pose(self):
        for record in (euler('absent'),quaternion('absent')):
            self.assertFalse(record['present']);self.assertFalse(record['fresh']);self.assertFalse(record['encoding_valid'])
            self.assertIsNone(record['data']);self.assertIsNone(record['effective_at']);self.assertEqual(record['observation_id'],0)
        self.assertEqual(status('absent')['state'],'WAITING_HEARTBEAT')

    def test_post_construction_accepted_heartbeat_is_required(self):
        self.assertFalse(euler('unbound')['present'])
        self.assertFalse(euler('preconstruction_floor')['present'])
        self.assertTrue(euler('after_boundary')['fresh'])
        self.assertEqual(status('unbound')['source_rejected'],1)

    def test_original_effective_read_times_are_distinct(self):
        record=euler('euler')
        self.assertAlmostEqual(record['effective_at'],100.12);self.assertAlmostEqual(record['read_at'],100.3)
        self.assertAlmostEqual(record['age_s'],.18);self.assertEqual(record['age_basis'],'QUEUE_EMPTY_FLOOR')
        self.assertTrue(record['age_is_upper_bound']);self.assertEqual(record['source_age'],'unknown');self.assertEqual(record['source_quality'],'unassessed')

    def test_republication_does_not_renew_sample(self):
        first=euler('euler');cached=euler('cached')
        self.assertEqual(first['observation_id'],cached['observation_id']);self.assertEqual(first['effective_at'],cached['effective_at'])
        self.assertAlmostEqual(cached['age_s'],.78);self.assertFalse(euler('stale_euler')['fresh']);self.assertEqual(status('stale_euler')['state'],'BOUND')

    def test_source_frames_rates_and_fused_estimate_are_explicit(self):
        data=euler('euler')['data']
        self.assertEqual((data['reference_frame'],data['body_frame'],data['estimate']),('LOCAL_NED','BODY_FRD','fused_fc_attitude'))
        rates=data['angular_velocity_body_frd_rad_s']
        for axis,expected in [('x',1.25),('y',-2.5),('z',.125)]:
            self.assertTrue(rates[axis]['known']);self.assertEqual(rates[axis]['unit'],'rad/s');self.assertAlmostEqual(rates[axis]['value'],expected)
        self.assertNotIn('covariance',data);self.assertNotIn('linear_acceleration',data)

    def test_euler_keeps_wire_angles_and_has_north_clockwise_heading(self):
        data=euler('euler')['data'];reported=data['reported_rpy']
        self.assertAlmostEqual(reported['roll_rad']['value'],.2,places=6);self.assertAlmostEqual(reported['pitch_rad']['value'],-.3,places=6)
        self.assertAlmostEqual(reported['yaw_rad']['value'],-math.pi/2,places=6)
        self.assertAlmostEqual(reported['source_yaw_heading_deg']['value'],270,places=5)
        self.assertEqual(data['raw_angles_rad'],[reported[k]['value'] for k in ('roll_rad','pitch_rad','yaw_rad')])

    def test_out_of_declared_yaw_range_is_retained_but_not_valid(self):
        data=euler('ardupilot_yaw')['data']
        self.assertFalse(data['orientation_valid']);self.assertFalse(euler('ardupilot_yaw')['encoding_valid'])
        self.assertFalse(data['wire_declared_angle_range_conformant'])
        self.assertAlmostEqual(data['raw_angles_rad'][2],3*math.pi/2,places=5)
        self.assertFalse(data['reported_rpy']['source_yaw_heading_deg']['known'])

    def test_zero_and_pi_are_known_but_twopi_does_not_silently_wrap(self):
        for case,expected in [('zero_yaw',0),('pi_yaw',180)]:
            heading=euler(case)['data']['reported_rpy']['source_yaw_heading_deg']
            self.assertTrue(heading['known']);self.assertLess(abs((heading['value']-expected+180)%360-180),.00002)
        data=euler('twopi_yaw')['data'];self.assertFalse(data['orientation_valid'])
        self.assertAlmostEqual(data['raw_angles_rad'][2],2*math.pi,places=5)
        self.assertIsNone(data['reported_rpy']['source_yaw_heading_deg']['value'])

    def test_tiny_negative_and_signed_zero_heading_are_canonical_north(self):
        for case,fn,rpy_key in [('tiny_negative_euler',euler,'reported_rpy'),('tiny_negative_quaternion',quaternion,'derived_rpy'),('negative_zero_heading',euler,'reported_rpy')]:
            item=fn(case)['data'][rpy_key]['source_yaw_heading_deg']
            self.assertTrue(item['known']);self.assertEqual(item['value'],0)
            self.assertTrue(0<=item['value']<360)
        for case,fn,rpy_key in [('tiny_positive_euler',euler,'reported_rpy'),('tiny_positive_quaternion',quaternion,'derived_rpy')]:
            item=fn(case)['data'][rpy_key]['source_yaw_heading_deg']
            self.assertTrue(item['known']);self.assertTrue(0<item['value']<1e-12)

    def test_quaternion_identity_is_wxyz(self):
        data=quaternion('q_identity')['data']
        self.assertEqual(data['quaternion_wxyz'],[1,0,0,0]);self.assertEqual(data['quaternion_norm'],1)
        self.assertTrue(data['orientation_valid']);self.assertTrue(data['derived_normalized_within_tolerance'])
        self.assertEqual(data['derived_rpy']['source_yaw_heading_deg']['value'],0)

    def test_known_east_west_and_roll_rotations(self):
        for case,heading in [('q_east',90),('q_west',270)]:
            data=quaternion(case)['data'];self.assertTrue(data['orientation_valid'])
            self.assertAlmostEqual(data['derived_rpy']['source_yaw_heading_deg']['value'],heading,places=5)
            self.assertAlmostEqual(data['derived_rpy']['roll_rad']['value'],0,places=6)
        self.assertAlmostEqual(quaternion('q_roll')['data']['derived_rpy']['roll_rad']['value'],math.pi/2,places=6)

    def test_mixed_rotation_preserves_source_rpy(self):
        data=quaternion('q_mixed')['data']['derived_rpy']
        for field,expected in [('roll_rad',.2),('pitch_rad',-.3),('yaw_rad',2.2)]:self.assertAlmostEqual(data[field]['value'],expected,places=6)

    def test_q_and_negative_q_have_same_attitude(self):
        first=quaternion('q_mixed')['data'];negative=quaternion('q_negative')['data']
        for a,b in zip(first['quaternion_wxyz'],negative['quaternion_wxyz']):self.assertAlmostEqual(a,-b)
        self.assertEqual(first['derived_rpy'],negative['derived_rpy'])

    def test_unit_norm_tolerance_is_visible_and_raw_q_not_changed(self):
        data=quaternion('q_near_unit')['data']
        self.assertEqual(data['unit_norm_tolerance'],.001);self.assertAlmostEqual(data['quaternion_norm'],.9995,places=6)
        self.assertAlmostEqual(data['quaternion_wxyz'][0],.9995,places=6);self.assertTrue(data['orientation_valid'])
        self.assertEqual(data['derived_rpy']['pitch_rad']['value'],0)

    def test_nonunit_zero_and_huge_quaternions_cannot_be_valid_pose(self):
        for case in ('q_nonunit','q_zero','q_huge'):
            record=quaternion(case);self.assertTrue(record['fresh']);self.assertFalse(record['encoding_valid'])
            self.assertFalse(record['data']['orientation_valid']);self.assertFalse(record['data']['derived_normalized_within_tolerance'])
            self.assertIsNone(record['data']['derived_rpy']['yaw_rad']['value'])

    def test_quaternion_singularity_does_not_invent_yaw_heading(self):
        data=quaternion('q_singular')['data'];self.assertTrue(data['orientation_valid'])
        self.assertAlmostEqual(data['derived_rpy']['pitch_rad']['value'],math.pi/2,places=6)
        self.assertFalse(data['derived_rpy']['rpy_unique'])
        for field in ('yaw_rad','roll_rad','source_yaw_heading_deg'):
            self.assertFalse(data['derived_rpy'][field]['known']);self.assertIsNone(data['derived_rpy'][field]['value'])

    def test_euler_singularity_retains_reported_triple_only(self):
        data=euler('euler_singular')['data']['reported_rpy']
        self.assertTrue(data['yaw_rad']['known']);self.assertFalse(data['rpy_unique'])
        self.assertFalse(data['source_yaw_heading_deg']['known'])

    def test_families_never_replace_or_freshen_each_other(self):
        initial=status('both_families');later=status('independent_ages')
        self.assertFalse(later['atomic']);self.assertEqual(later['family_selection'],'independent_no_preference')
        self.assertEqual(later['euler']['observation_id'],initial['euler']['observation_id'])
        self.assertFalse(later['euler']['fresh']);self.assertTrue(later['quaternion']['fresh'])
        self.assertNotEqual(later['quaternion']['observation_id'],initial['quaternion']['observation_id'])

    def test_nonfinite_euler_and_out_of_profile_range_are_invalid(self):
        for case in ('euler_nan','euler_inf','euler_range','yaw_range'):
            record=euler(case);self.assertFalse(record['encoding_valid']);self.assertFalse(record['data']['orientation_valid'])
            self.assertIsNone(record['data']['reported_rpy']['yaw_rad']['value'])
        self.assertIsNone(euler('euler_nan')['data']['raw_angles_rad'][0])
        self.assertEqual(euler('yaw_range')['data']['raw_angles_rad'][2],1000)

    def test_invalid_latest_observation_does_not_republish_old_valid_pose(self):
        a=quaternion('q_near_unit');b=quaternion('q_nonunit')
        self.assertGreater(b['observation_id'],a['observation_id']);self.assertGreater(b['effective_at'],a['effective_at'])
        self.assertFalse(b['data']['orientation_valid'])

    def test_unknown_rate_invalidates_record_but_not_finite_orientation_fields(self):
        for case,fn,axis in [('euler_bad_rate',euler,'y'),('q_bad_rate',quaternion,'z')]:
            record=fn(case);data=record['data'];self.assertFalse(record['encoding_valid']);self.assertTrue(data['orientation_valid'])
            self.assertFalse(data['angular_velocity_valid']);self.assertIsNone(data['angular_velocity_body_frd_rad_s'][axis]['value'])

    def test_nonfinite_quaternion_components_stay_unknown(self):
        for case,index in [('q_nan',1),('q_inf',3)]:
            data=quaternion(case)['data'];self.assertFalse(data['quaternion_components_finite']);self.assertIsNone(data['quaternion_wxyz'][index])
            self.assertIsNone(data['quaternion_norm']);self.assertFalse(data['orientation_valid'])

    def test_all_zero_offset_means_unsupported_not_null_rotation(self):
        offset=quaternion('offset_unsupported')['data']['representation_offset']
        self.assertEqual(offset['raw_wxyz'],[0,0,0,0]);self.assertEqual(offset['status'],'unsupported_or_omitted')
        self.assertTrue(offset['encoding_valid']);self.assertFalse(offset['display_adjustment_applied'])

    def test_nonzero_display_offset_is_preserved_unapplied(self):
        for case in ('offset_identity','offset_unapplied'):
            data=quaternion(case)['data'];offset=data['representation_offset']
            self.assertEqual(offset['status'],'unit_offset_unapplied');self.assertTrue(offset['extension_bytes_observed'])
            self.assertFalse(offset['display_adjustment_applied']);self.assertTrue(quaternion(case)['encoding_valid'])
            self.assertEqual(data['derived_rpy']['roll_rad']['value'],0)

    def test_invalid_offset_never_manufactures_a_valid_display_attitude(self):
        for case in ('offset_invalid','offset_nan'):
            record=quaternion(case);offset=record['data']['representation_offset']
            self.assertFalse(record['encoding_valid']);self.assertEqual(offset['status'],'invalid_offset_unapplied')
            self.assertFalse(offset['encoding_valid']);self.assertFalse(offset['display_adjustment_applied'])
        self.assertIsNone(quaternion('offset_nan')['data']['representation_offset']['raw_wxyz'][0])

    def test_remote_boot_clock_wrap_does_not_renew_or_reset_local_clock(self):
        first=euler('boot_max');wrapped=euler('boot_wrap');repeated=euler('same_remote_new_local')
        self.assertEqual(first['data']['time_boot_ms'],2**32-1);self.assertEqual(wrapped['data']['time_boot_ms'],0)
        self.assertEqual(first['session_epoch'],wrapped['session_epoch']);self.assertGreater(wrapped['observation_id'],first['observation_id'])
        self.assertGreater(repeated['observation_id'],wrapped['observation_id']);self.assertEqual(wrapped['data']['remote_clock'],'boot_ms_unmapped_uint32')

    def test_old_effective_and_backwards_read_times_are_rejected(self):
        first=euler('order_before');later=euler('order_after')
        self.assertEqual(first['observation_id'],later['observation_id']);self.assertEqual(first['data'],later['data'])
        self.assertEqual(status('order_after')['out_of_order_rejected'],2)

    def test_coalesced_distinct_frames_share_stamps_without_age_renewal(self):
        first=euler('coalesced_before');later=euler('coalesced_after')
        self.assertGreater(later['observation_id'],first['observation_id'])
        self.assertEqual(later['effective_at'],first['effective_at']);self.assertEqual(later['read_at'],first['read_at'])
        self.assertEqual(later['data']['raw_angles_rad'][2],1);self.assertAlmostEqual(later['age_s'],.6)
        self.assertEqual(status('coalesced_after')['out_of_order_rejected'],0)

    def test_impossible_or_unlabelled_metadata_cannot_create_sample(self):
        self.assertFalse(euler('metadata_invalid')['present']);self.assertEqual(status('metadata_invalid')['metadata_rejected'],8)
        record=euler('read_acquisition');self.assertTrue(record['fresh']);self.assertEqual(record['age_basis'],'READ_ACQUISITION')
        self.assertFalse(record['age_is_upper_bound']);self.assertEqual(record['effective_at'],record['read_at'])

    def test_foreign_source_and_wrong_vehicle_heartbeat_do_not_rebind(self):
        self.assertEqual(euler('source_before')['data'],euler('foreign_source')['data'])
        self.assertEqual(status('foreign_source')['source_rejected'],2)
        self.assertEqual(status('foreign_heartbeats')['source_rejected'],5)
        self.assertEqual((status('foreign_heartbeats')['system_id'],status('foreign_heartbeats')['component_id']),(1,1))

    def test_heartbeat_lapse_clears_samples_and_new_heartbeat_does_not_revive(self):
        self.assertFalse(euler('heartbeat_lapse')['present']);self.assertEqual(status('heartbeat_lapse')['state'],'WAITING_HEARTBEAT')
        self.assertEqual(status('heartbeat_only')['state'],'BOUND');self.assertFalse(euler('heartbeat_only')['present'])
        self.assertFalse(quaternion('heartbeat_only')['present'])

    def test_host_invalid_heartbeat_is_independent_reset_boundary(self):
        self.assertFalse(euler('host_heartbeat_invalid')['present']);self.assertEqual(status('host_heartbeat_invalid')['reset_reason'],'heartbeat_expired')

    def test_reset_invalidates_both_families_and_rejects_old_floor(self):
        for case in ('reset','old_floor_after_reset','rebound_empty'):
            self.assertFalse(euler(case)['present']);self.assertFalse(quaternion(case)['present'])
        self.assertTrue(euler('new_after_reset')['fresh'])
        self.assertEqual(status('reset')['reset_reason'],'quotes" newline\n reset')

    def test_clock_rollback_latches_until_valid_explicit_reset(self):
        for case in ('clock_fault','clock_still_latched','clock_nan_latched','clock_bad_reset_latched'):
            self.assertEqual(status(case)['state'],'CLOCK_FAULT');self.assertFalse(euler(case)['present'])
        self.assertTrue(euler('clock_recovery')['fresh']);self.assertEqual(status('clock_recovery')['state'],'BOUND')

    def test_wire_shape_rejection_is_distinct_from_metadata(self):
        self.assertEqual(status('wire_rejects')['wire_shape_rejected'],5)
        self.assertEqual(status('wire_rejects')['metadata_rejected'],0)

    def test_legitimate_mavlink2_zero_tail_and_v1_are_decoded(self):
        self.assertTrue(euler('v2_zero_tail')['encoding_valid']);self.assertEqual(euler('v2_zero_tail')['data']['raw_angles_rad'],[0,0,0])
        self.assertTrue(euler('v1_euler')['encoding_valid']);self.assertTrue(quaternion('v1_quaternion')['encoding_valid'])
        self.assertFalse(quaternion('v1_quaternion')['data']['representation_offset']['extension_bytes_observed'])
        self.assertFalse(quaternion('q_v2_zero_tail')['encoding_valid'])

    def test_publication_throttle_does_not_change_input_age(self):
        self.assertEqual(records('interval_suppressed','MAVMOOS_ATTITUDE_EULER'),[])
        record=euler('interval_later');self.assertAlmostEqual(record['effective_at'],100.3);self.assertAlmostEqual(record['age_s'],.5)
        self.assertFalse(euler('short_timeout')['fresh'])

    def test_aggregate_cannot_assert_authority_or_atomic_generation(self):
        for row in rows:
            if row['key']!='MAVMOOS_ATTITUDE_STATUS':continue
            record=row['value'];self.assertTrue(record['read_only']);self.assertFalse(record['authoritative_for_control']);self.assertFalse(record['atomic'])
            self.assertNotIn('safe',record);self.assertNotIn('healthy',record)

    def test_bounded_random_decoded_frames_remain_finite_read_only_and_two_cache(self):
        self.assertGreaterEqual(len(records('bounded_random_frames','MAVMOOS_ATTITUDE_STATUS')),8)
        for record in records('bounded_random_frames','MAVMOOS_ATTITUDE_STATUS'):
            self.assertEqual({k for k in record if k in ('euler','quaternion')},{'euler','quaternion'})
            self.assertFalse(record['authoritative_for_control'])

    def test_reset_before_retained_highwater_stays_faulted(self):
        self.assertTrue(euler('hardening_reliable')['fresh'])
        state=status('hardening_early_reset')
        self.assertEqual(state['state'],'CLOCK_FAULT')
        self.assertEqual(state['reset_reason'],'behind_highwater')
        self.assertFalse(euler('hardening_early_reset')['present'])

    def test_clock_catchup_does_not_implicitly_recover_after_bad_reset(self):
        for case in ('hardening_input_before_highwater','hardening_catchup_without_reset'):
            self.assertEqual(status(case)['state'],'CLOCK_FAULT',case)
            self.assertFalse(euler(case)['present'],case)
            self.assertFalse(euler(case)['fresh'],case)

    def test_valid_explicit_reset_retains_new_floor_before_fresh_recovery(self):
        state=status('hardening_valid_reset')
        self.assertEqual(state['state'],'WAITING_HEARTBEAT')
        self.assertAlmostEqual(state['session_boundary'],102.1)
        self.assertFalse(euler('hardening_valid_reset')['present'])
        self.assertEqual(status('hardening_old_floor')['state'],'WAITING_HEARTBEAT')
        self.assertFalse(euler('hardening_old_floor')['present'])
        current=euler('hardening_recovered')
        self.assertEqual(status('hardening_recovered')['state'],'BOUND')
        self.assertTrue(current['fresh']);self.assertTrue(current['present'])
        self.assertAlmostEqual(current['effective_at'],102.45)
        self.assertAlmostEqual(current['read_at'],102.45)
        self.assertGreater(current['observation_id'],euler('hardening_reliable')['observation_id'])

    def test_negative_nonfinite_and_behind_resets_latch_without_prior_clock_fault(self):
        for index in range(4):
            case='hardening_invalid_reset_'+str(index)
            self.assertEqual(status(case)['state'],'CLOCK_FAULT',case)
            self.assertFalse(euler(case)['present'],case)
            self.assertFalse(euler(case)['fresh'],case)

    def test_reset_at_equal_reliable_time_can_recover(self):
        state=status('hardening_equal_reset')
        self.assertEqual(state['state'],'BOUND')
        self.assertAlmostEqual(state['session_boundary'],101)
        self.assertTrue(euler('hardening_equal_reset')['fresh'])
        self.assertAlmostEqual(euler('hardening_equal_reset')['effective_at'],101.2)

if __name__=='__main__':unittest.main(argv=[__file__]+remaining)
