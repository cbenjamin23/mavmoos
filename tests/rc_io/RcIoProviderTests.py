#!/usr/bin/env python3
"""Semantic checks of a real rc_io MODULE through fake Services. No sockets."""
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
def input_record(case):return one(case,'MAVMOOS_RC_INPUT')
def outputs(case):return one(case,'MAVMOOS_SERVO_OUTPUT')
def port(case,number=0):
    values=[item for item in outputs(case)['ports'] if item['data']['port']==number]
    if len(values)!=1:raise AssertionError(f'{case}/port{number}: expected1, got{len(values)}')
    return values[0]
def status(case):return one(case,'MAVMOOS_RC_IO_STATUS')

class RcIoTests(unittest.TestCase):
    def test_actual_factory_abi_descriptor_and_zero_effects(self):
        self.assertEqual(one('driver_complete','summary'),{'effects':0,'abi':10,'random_frames':2048})
        self.assertGreater(len(rows),150)
        self.assertTrue(all(r['key'] in ('MAVMOOS_RC_INPUT','MAVMOOS_SERVO_OUTPUT','MAVMOOS_RC_IO_STATUS','summary') for r in rows))

    def test_absent_is_not_zero_input_or_zero_servo(self):
        item=input_record('absent')
        self.assertFalse(item['present']);self.assertFalse(item['encoding_valid']);self.assertFalse(item['fresh'])
        self.assertIsNone(item['data']);self.assertIsNone(item['effective_at']);self.assertEqual(item['observation_id'],0)
        self.assertFalse(outputs('absent')['present']);self.assertEqual(outputs('absent')['ports'],[])
        self.assertEqual(status('absent')['state'],'WAITING_HEARTBEAT')

    def test_post_boundary_heartbeat_is_required(self):
        for case in ('unbound','preconstruction_floor'):
            self.assertFalse(input_record(case)['present']);self.assertEqual(outputs(case)['ports'],[])
        self.assertEqual(status('unbound')['source_rejected'],2)
        self.assertTrue(input_record('after_boundary')['fresh'])

    def test_all_eighteen_input_slots_preserve_raw_us(self):
        data=input_record('both')['data'];channels=data['channels']
        self.assertEqual(len(channels),18);self.assertEqual([x['slot'] for x in channels],list(range(1,19)))
        self.assertEqual([x['raw_us'] for x in channels],[1001+n*101 for n in range(18)])
        self.assertTrue(all(x['reported_by_count'] and x['pwm_us']['known'] and x['pwm_us']['unit']=='us' for x in channels))
        self.assertEqual(data['channel_roles'],'unmapped');self.assertEqual(data['pulse_validity'],'unassessed')

    def test_zero_count_retains_wire_but_has_no_available_channels(self):
        item=input_record('count_0');data=item['data']
        self.assertTrue(item['present']);self.assertTrue(item['fresh']);self.assertTrue(item['encoding_valid'])
        self.assertEqual(data['reported_channel_count'],0);self.assertEqual(data['visible_channel_count'],0)
        self.assertEqual(data['channels'][0]['raw_us'],1001)
        self.assertTrue(all(not x['reported_by_count'] and not x['pwm_us']['known'] and x['pwm_us']['value'] is None for x in data['channels']))

    def test_count_one_eight_eighteen_and_overflow_do_not_invent_slots(self):
        for count in (1,8,18,19,255):
            data=input_record(f'count_{count}')['data']
            self.assertEqual(data['reported_channel_count'],count);self.assertEqual(data['visible_channel_count'],min(count,18))
            self.assertEqual(data['channels_beyond_message'],count>18);self.assertEqual(len(data['channels']),18)
            self.assertEqual(sum(x['reported_by_count'] for x in data['channels']),min(count,18))

    def test_input_zero_and_uint16max_are_raw_not_invented_sentinels(self):
        channels=input_record('input_extremes')['data']['channels']
        self.assertEqual([x['raw_us'] for x in channels[:2]],[0,65535])
        self.assertEqual([x['pwm_us']['value'] for x in channels[:2]],[0,65535])
        self.assertTrue(all(x['pwm_us']['known'] for x in channels[:2]))
        self.assertFalse(channels[2]['pwm_us']['known'])

    def test_rssi_invalid_is_unknown_and_other_values_are_device_dependent(self):
        for case,expected in [('input_extremes',255),('rssi_zero',0),('both',254)]:
            item=input_record(case)['data']['rssi'];self.assertEqual(item['raw'],expected)
            self.assertEqual(item['quantity']['known'],expected!=255)
            self.assertEqual(item['quantity']['value'],None if expected==255 else expected)
            self.assertEqual(item['quantity']['unit'],'device_dependent')

    def test_original_stamps_and_ages_are_preserved(self):
        first=input_record('both');cached=input_record('cached')
        self.assertAlmostEqual(first['effective_at'],100.12);self.assertAlmostEqual(first['read_at'],100.3)
        self.assertAlmostEqual(first['age_s'],.19);self.assertAlmostEqual(cached['age_s'],.78)
        self.assertEqual(first['observation_id'],cached['observation_id']);self.assertEqual(first['effective_at'],cached['effective_at'])
        self.assertEqual(first['age_basis'],'QUEUE_EMPTY_FLOOR');self.assertTrue(first['age_is_upper_bound'])
        self.assertEqual(first['source_age'],'unknown');self.assertEqual(first['source_quality'],'unassessed')

    def test_input_and_each_port_age_independently(self):
        self.assertFalse(input_record('independent_ages')['fresh'])
        self.assertFalse(port('independent_ages',0)['fresh']);self.assertTrue(port('independent_ages',1)['fresh'])
        self.assertEqual(port('both')['observation_id'],port('independent_ages',0)['observation_id'])
        self.assertFalse(outputs('independent_ages')['atomic']);self.assertFalse(status('independent_ages')['atomic'])

    def test_port_identity_is_retained_without_universal_flattening(self):
        records=outputs('ports_sorted')['ports']
        self.assertEqual([r['data']['port'] for r in records],[0,1,255])
        self.assertEqual(len(records[0]['data']['channels']),16)
        self.assertTrue(all(r['data']['actuator_mapping']=='unmapped_port_and_slot' for r in records))
        self.assertTrue(all(r['data']['physical_feedback'] is False for r in records))
        self.assertTrue(all(r['data']['output_kind']=='fc_command_observation' for r in records))
        self.assertNotIn('actuator_index',records[1]['data']['channels'][0])

    def test_per_port_updates_preserve_other_cache_records(self):
        for number in (0,255):
            before=port('ports_sorted',number);after=port('port_update',number)
            self.assertEqual(before['observation_id'],after['observation_id']);self.assertEqual(before['effective_at'],after['effective_at'])
        self.assertGreater(port('port_update',1)['observation_id'],port('ports_sorted',1)['observation_id'])

    def test_cache_is_bounded_rejects_new_ids_and_keeps_existing(self):
        self.assertEqual(outputs('ports_full')['cache_capacity'],8)
        self.assertEqual([x['data']['port'] for x in outputs('port_capacity')['ports']],list(range(8)))
        self.assertEqual(status('port_capacity')['port_capacity_rejected'],2)
        self.assertEqual(port('retained_port_update',3)['data']['channels'][0]['raw_us'],1999)
        self.assertEqual([x['data']['port'] for x in outputs('port_reset_capacity')['ports']],[255])

    def test_output_zero_is_explicit_ambiguity_and_uint16max_retained(self):
        channels=port('output_extremes')['data']['channels']
        self.assertEqual(channels[0]['raw_us'],0);self.assertFalse(channels[0]['pwm_us']['known'])
        self.assertEqual(channels[1]['raw_us'],65535);self.assertEqual(channels[1]['pwm_us']['value'],65535)
        self.assertEqual(channels[8]['raw_us'],0);self.assertFalse(channels[8]['pwm_us']['known'])
        self.assertEqual(channels[8]['pwm_us']['reason'],'zero_extension_unsupported_or_omitted_or_zero')
        self.assertEqual(port('output_extremes')['data']['pulse_validity'],'unassessed')

    def test_all_zero_servo_is_fresh_observation_not_actuator_feedback(self):
        item=port('output_zero',1);self.assertTrue(item['fresh']);self.assertTrue(item['encoding_valid'])
        self.assertFalse(item['data']['physical_feedback'])
        self.assertTrue(all(x['raw_us']==0 and not x['pwm_us']['known'] for x in item['data']['channels']))

    def test_mavlink1_excludes_extension_values_even_with_dirty_tail(self):
        data=port('servo_v1')['data'];self.assertEqual(data['wire_version'],1);self.assertFalse(data['extension_bytes_observed'])
        self.assertEqual([x['raw_us'] for x in data['channels'][:8]],[1001+n*101 for n in range(8)])
        for x in data['channels'][8:]:
            self.assertIsNone(x['raw_us']);self.assertFalse(x['pwm_us']['known']);self.assertEqual(x['pwm_us']['reason'],'mavlink1_extension_absent')
        self.assertEqual(input_record('input_v1')['data']['wire_version'],1)

    def test_partial_mavlink2_extension_byte_is_zero_filled(self):
        data=port('partial_extension')['data'];self.assertTrue(data['extension_bytes_observed'])
        self.assertEqual(data['channels'][8]['raw_us'],7);self.assertEqual(data['channels'][8]['pwm_us']['value'],7)
        self.assertTrue(all(x['raw_us']==0 and not x['pwm_us']['known'] for x in data['channels'][9:]))

    def test_omitted_extensions_do_not_turn_into_known_zero_outputs(self):
        data=port('omitted_extension')['data'];self.assertFalse(data['extension_bytes_observed'])
        for x in data['channels'][8:]:self.assertEqual(x['raw_us'],0);self.assertFalse(x['pwm_us']['known'])
        data=port('v2_zero_tail')['data'];self.assertEqual(data['port'],0)
        self.assertTrue(all(x['raw_us']==0 and not x['pwm_us']['known'] for x in data['channels']))
        self.assertEqual(input_record('input_zero_tail')['data']['reported_channel_count'],0)

    def test_remote_uint32_wrap_is_not_an_epoch_or_local_freshness_clock(self):
        for getter,key,clock in [(input_record,'time_boot_ms','boot_ms_unmapped_uint32'),(port,'time_usec','unmapped_uint32_us_no_epoch_inference')]:
            first=getter('remote_max');wrapped=getter('remote_wrap');repeated=getter('remote_repeat')
            self.assertEqual(first['data'][key],2**32-1);self.assertEqual(wrapped['data'][key],0)
            self.assertEqual(first['session_epoch'],wrapped['session_epoch']);self.assertEqual(wrapped['data']['remote_clock'],clock)
            self.assertGreater(repeated['observation_id'],wrapped['observation_id'])

    def test_older_effective_and_read_metadata_do_not_replace_records(self):
        self.assertEqual(status('order_after')['out_of_order_rejected'],2)
        for getter in (input_record,port):
            self.assertEqual(getter('order_before')['observation_id'],getter('order_after')['observation_id'])

    def test_same_datagram_distinct_frames_can_share_exact_stamps(self):
        for getter in (input_record,port):
            before=getter('order_before');after=getter('coalesced_frames')
            self.assertGreater(after['observation_id'],before['observation_id'])
            self.assertEqual(after['effective_at'],before['effective_at']);self.assertEqual(after['read_at'],before['read_at'])
            self.assertAlmostEqual(after['age_s'],.7);self.assertEqual(after['data']['channels'][0]['raw_us'],1999)

    def test_invalid_metadata_is_rejected_and_read_acquisition_label_is_truthful(self):
        self.assertEqual(status('metadata_invalid')['metadata_rejected'],8)
        self.assertFalse(input_record('metadata_invalid')['present'])
        item=input_record('read_acquisition');self.assertTrue(item['fresh']);self.assertFalse(item['age_is_upper_bound'])
        self.assertEqual(item['age_basis'],'READ_ACQUISITION')

    def test_foreign_sources_and_heartbeats_cannot_rebind_or_update(self):
        self.assertEqual(status('foreign_sources')['source_rejected'],6)
        self.assertEqual(status('foreign_sources')['system_id'],1)
        for getter in (input_record,port):self.assertEqual(getter('source_before')['observation_id'],getter('foreign_sources')['observation_id'])

    def test_heartbeat_lapse_clears_all_ports_and_input_and_rebind_is_empty(self):
        for case in ('heartbeat_lapse','heartbeat_only','host_heartbeat_invalid'):
            self.assertFalse(input_record(case)['present']);self.assertEqual(outputs(case)['ports'],[])
        self.assertEqual(status('heartbeat_only')['state'],'BOUND')

    def test_reset_invalidates_cache_floors_and_escapes_reason(self):
        for case in ('reset','old_floor_after_reset','rebound_empty'):
            self.assertFalse(input_record(case)['present']);self.assertEqual(outputs(case)['ports'],[])
        self.assertEqual(status('reset')['reset_reason'],'quotes" newline\n reset')
        self.assertTrue(input_record('new_after_reset')['fresh']);self.assertTrue(port('new_after_reset',255)['fresh'])

    def test_clock_fault_latches_until_valid_explicit_reset(self):
        for case in ('clock_fault','clock_still_latched','clock_nan_latched','clock_bad_reset_latched'):
            self.assertEqual(status(case)['state'],'CLOCK_FAULT');self.assertFalse(input_record(case)['present']);self.assertEqual(outputs(case)['ports'],[])
        self.assertTrue(input_record('clock_recovery')['fresh'])

    def test_explicit_reset_cannot_rewind_last_reliable_owner_time(self):
        self.assertTrue(input_record('highwater_before')['present'])
        for case in ('highwater_rollback','highwater_bad_reset','highwater_catchup_still_latched'):
            self.assertEqual(status(case)['state'],'CLOCK_FAULT')
            self.assertFalse(input_record(case)['present']);self.assertEqual(outputs(case)['ports'],[])
        self.assertTrue(input_record('highwater_recovery')['fresh']);self.assertTrue(port('highwater_recovery')['fresh'])
        self.assertAlmostEqual(status('highwater_recovery')['session_boundary'],101.2)

    def test_wire_shape_validation_does_not_consume_unrelated_messages(self):
        self.assertEqual(status('wire_rejects')['wire_shape_rejected'],5)
        self.assertEqual(status('wire_rejects')['metadata_rejected'],0)
        self.assertFalse(input_record('wire_rejects')['present'])

    def test_configured_pacing_and_short_ttl_are_local_policies(self):
        self.assertEqual(by_case['interval_suppressed'],[])
        self.assertEqual(input_record('interval_first')['data']['reported_channel_count'],18)
        self.assertEqual(input_record('interval_later')['data']['reported_channel_count'],1)
        self.assertFalse(input_record('short_timeout')['fresh']);self.assertFalse(port('short_timeout')['fresh'])

    def test_bounded_random_decoded_frames_are_finite_read_only_records(self):
        self.assertGreaterEqual(len(records('bounded_random_frames','MAVMOOS_RC_IO_STATUS')),3)
        for item in records('bounded_random_frames','MAVMOOS_RC_IO_STATUS'):
            self.assertTrue(item['read_only']);self.assertFalse(item['authoritative_for_control'])
            self.assertLessEqual(len(item['servo_output']['ports']),8)
            for record in item['servo_output']['ports']:
                self.assertEqual(len(record['data']['channels']),16)
        self.assertGreater(status('both')['input']['observation_id'],0)

unittest.main(argv=[__file__]+remaining)
