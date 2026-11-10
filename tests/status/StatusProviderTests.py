#!/usr/bin/env python3
"""Semantic checks of actual factory/MODULE publications under fake Services."""
import argparse
import collections
import json
import math
from pathlib import Path
import subprocess
import sys
import unittest

parser=argparse.ArgumentParser()
parser.add_argument('--driver',required=True)
parser.add_argument('--module',required=True)
parser.add_argument('--trace')
args,remaining=parser.parse_known_args()
result=subprocess.run([args.driver,args.module],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,timeout=15)
if result.returncode:
    raise RuntimeError(f'actual status MODULE scenario driver failed: {result.stderr}')
if args.trace:
    Path(args.trace).write_text(result.stdout)
rows=[]
def reject_constant(value):
    raise ValueError('nonfinite JSON: '+value)
for line in result.stdout.splitlines():
    rows.append(json.loads(line,parse_constant=reject_constant))
by_case=collections.defaultdict(list)
for row in rows:
    by_case[row['case']].append(row)

def values(case,key):
    return [row['value'] for row in by_case[case] if row['key']==key]
def one(case,key):
    data=values(case,key)
    if len(data)!=1:
        raise AssertionError(f'{case}/{key}: expected one record, got {len(data)}')
    return data[0]
def status(case):return one(case,'MAVMOOS_SYSTEM_STATUS')
def system(case):return one(case,'MAVMOOS_SYS_STATUS')
def gps(case):return one(case,'MAVMOOS_GPS_STATUS')
def text(case):return values(case,'MAVMOOS_FC_TEXT')
def identified(case,ident=0):
    return next(r for r in values(case,'MAVMOOS_BATTERY_STATUS') if r['data']['battery_id']==ident)

def finite_tree(value):
    if isinstance(value,float):
        return math.isfinite(value)
    if isinstance(value,dict):return all(finite_tree(v) for v in value.values())
    if isinstance(value,list):return all(finite_tree(v) for v in value)
    return True

class StatusTests(unittest.TestCase):
    def test_factory_has_no_effects_and_exact_development_abi(self):
        self.assertEqual(one('driver_complete','summary'),{'effects':0,'abi':10})
        self.assertTrue(all(finite_tree(row) for row in rows))

    def test_power_and_mcu_are_independent_bounded_records(self):
        aggregate=status('diagnostics')
        power=one('diagnostics','MAVMOOS_POWER_STATUS')
        mcus=one('diagnostics','MAVMOOS_MCU_STATUS')
        self.assertEqual((aggregate['power'],aggregate['mcu']),(power,mcus))
        self.assertEqual((aggregate['source_rejected'],aggregate['wire_shape_rejected'],
                          aggregate['out_of_order_rejected']),(1,1,1))
        data=power['data']
        self.assertEqual((data['board_rail_mv'],data['board_rail_v']['value']),(5025,5.025))
        self.assertEqual((data['servo_rail_mv'],data['servo_rail_v']['value']),(0,None))
        self.assertTrue(data['flags']['reported']['servo_valid'])
        self.assertTrue(data['flags']['reported']['high_power_peripheral_overcurrent'])
        self.assertEqual(data['flags']['unknown_mask'],0x8000)
        self.assertEqual(mcus['capacity_rejected'],1)
        self.assertEqual([row['data']['id'] for row in mcus['instances']],[2,6,10,20,30,40,50,60])
        first,second=mcus['instances'][:2]
        self.assertEqual(first['data']['temperature_c']['value'],-1.25)
        self.assertTrue(first['data']['reading_within_reported_range'])
        self.assertIsNone(second['data']['voltage_min_v']['value'])
        self.assertIsNone(second['data']['reading_within_reported_range'])
        self.assertFalse(status('diagnostics_field_fault')['mcu']['instances'][0]['data']['reading_within_reported_range'])

    def test_battery_metadata_and_generator_are_independent(self):
        def pack(case): return one(case,'MAVMOOS_BATTERY_INFO')
        def gen(case): return one(case,'MAVMOOS_GENERATOR_STATUS')
        self.assertFalse(pack('power_metadata_unbound')['present'])
        row=pack('power_metadata')
        self.assertFalse(row['atomic'])
        self.assertFalse(row['authoritative_for_control'])
        self.assertEqual(row['capacity_rejected'],1)
        self.assertEqual([r['data']['battery_id'] for r in row['instances']],[0,1,2,3,4,5,6,9])
        full,unknown=(row['instances'][i]['data'] for i in (4,7))
        self.assertEqual((full['design_capacity_ah']['value'],full['full_charge_capacity_ah']['value'],
                          full['state_of_health_pct']['value']),(10.25,8.5,83))
        self.assertEqual(full['device_name']['value'],'MAVMOOS_test_pack')
        self.assertEqual(full['serial_number']['value'],'unit-4')
        self.assertEqual(full['manufacture_date']['value'],'03112026')
        self.assertLess(full['payload_length'],140)
        self.assertEqual(unknown['device_name'],{'raw_hex':'ff','state':'non_ascii','value':None})
        self.assertEqual(unknown['serial_number'],
                         {'raw_hex':'58'*32,'state':'unterminated','value':None})
        self.assertIsNone(unknown['cycles']['value'])
        self.assertEqual(unknown['full_charge_capacity_ah']['reason'],'wire_nan')
        self.assertEqual(identified('power_metadata',4)['data']['battery_id'],4)
        initial=gen('power_metadata')
        data=initial['data']
        self.assertEqual(data['flags']['unknown_decimal'],str(1<<60))
        self.assertTrue(data['flags']['reported']['generating'])
        self.assertTrue(data['flags']['reported']['maintenance_required'])
        self.assertEqual(data['battery_current_a']['value'],-3.25)
        self.assertEqual(data['maintenance_in_s']['value'],-90)
        self.assertEqual(data['rectifier_temperature_c']['value'],-12)
        self.assertIsNone(data['runtime_s']['value'])
        self.assertIsNone(data['generator_temperature_c']['value'])
        self.assertEqual(gen('power_metadata_rejected')['observation_id'],initial['observation_id'])
        self.assertEqual(pack('power_metadata_rejected')['instances'][4]['observation_id'],
                         row['instances'][4]['observation_id'])
        split=pack('power_metadata_age_split')['instances']
        stale=split[4]
        self.assertFalse(stale['fresh'])
        self.assertTrue(split[7]['fresh'])
        self.assertEqual(split[7]['data']['battery_id'],9)
        self.assertEqual(stale['effective_at'],row['instances'][4]['effective_at'])
        self.assertTrue(gen('power_metadata_age_split')['fresh'])
        self.assertEqual(gen('power_metadata_age_split')['data']['bus_voltage_v']['reason'],'out_of_range')
        self.assertEqual(gen('power_metadata_age_split')['data']['power_generated_w']['reason'],'invalid_infinity')
        self.assertEqual(gen('power_metadata_age_split')['data']['generator_temperature_c']['value'],0)
        self.assertLess(gen('power_metadata_age_split')['data']['payload_length'],42)
        self.assertFalse(pack('power_metadata_reset')['present'])
        self.assertFalse(gen('power_metadata_reset')['present'])

    def test_estimator_bits_metrics_and_freshness_preserve_family(self):
        initial=status('diagnostics')
        ekf,estimator=initial['ekf']['data'],initial['estimator']['data']
        self.assertTrue(ekf['flags']['reported']['uninitialized'])
        self.assertTrue(ekf['flags']['reported']['gps_glitch'])
        self.assertNotIn('accel_error',ekf['flags']['reported'])
        self.assertEqual(ekf['flags']['unknown_mask'],0x1000)
        self.assertEqual(ekf['airspeed']['reason'],'mavlink1_extension_absent')
        self.assertEqual(ekf['terrain_altitude']['reason'],'zero_or_unused')
        self.assertEqual(estimator['time_usec_raw'],'9007199254740997')
        self.assertTrue(estimator['flags']['reported']['gps_glitch'])
        self.assertTrue(estimator['flags']['reported']['accel_error'])
        self.assertNotIn('uninitialized',estimator['flags']['reported'])
        self.assertEqual(estimator['flags']['unknown_mask'],0x8000)
        self.assertEqual(estimator['horizontal_accuracy_m']['value'],2.5)
        self.assertEqual(estimator['height_above_ground_ratio']['value'],0)
        broken=status('diagnostics_field_fault')['ekf']
        self.assertFalse(broken['encoding_valid'])
        self.assertEqual(broken['data']['compass']['reason'],'nonfinite_metric')
        self.assertEqual(broken['data']['horizontal_position']['reason'],'negative_metric')
        self.assertEqual(broken['data']['airspeed']['value'],1.5)
        split=status('diagnostics_age_split')
        self.assertTrue(split['power']['fresh'])
        self.assertFalse(split['ekf']['fresh'])
        self.assertFalse(split['estimator']['fresh'])
        self.assertEqual(split['estimator']['effective_at'],initial['estimator']['effective_at'])
        self.assertTrue(all(not row['fresh'] for row in split['mcu']['instances']))

    def test_diagnostics_clear_and_rebind_by_family(self):
        for case in ('absent','diagnostics_cleared','diagnostics_data_only'):
            state=status(case)
            self.assertTrue(all(not state[family]['present'] for family in ('power','mcu','ekf','estimator')))
            self.assertEqual(state['mcu']['instances'],[])
        recovered=status('diagnostics_rebound')
        self.assertGreater(recovered['session_epoch'],status('diagnostics')['session_epoch'])
        self.assertTrue(recovered['power']['fresh'])
        self.assertTrue(all(not recovered[family]['present'] for family in ('mcu','ekf','estimator')))

    def test_absent_values_are_not_zero_or_healthy(self):
        record=system('absent')
        self.assertFalse(record['present']);self.assertFalse(record['fresh'])
        self.assertIsNone(record['data']);self.assertIsNone(record['effective_at'])
        self.assertEqual(status('absent')['state'],'WAITING_HEARTBEAT')
        self.assertFalse(status('absent')['authoritative_for_control'])
        battery=one('absent','MAVMOOS_BATTERY_STATUS')
        self.assertFalse(battery['present']);self.assertIsNone(battery['data'])

    def test_no_status_admitted_without_post_boundary_heartbeat(self):
        self.assertFalse(system('unbound')['present'])
        self.assertEqual(status('unbound')['source_rejected'],1)
        self.assertFalse(system('preconstruction_floor')['present'])
        self.assertTrue(system('post_boundary')['fresh'])

    def test_conservative_acquisition_age_and_read_are_distinct(self):
        record=system('system')
        self.assertAlmostEqual(record['effective_at'],100.12)
        self.assertAlmostEqual(record['read_at'],100.30)
        self.assertAlmostEqual(record['age_s'],.18)
        self.assertEqual(record['age_basis'],'QUEUE_EMPTY_FLOOR')
        self.assertTrue(record['age_is_upper_bound'])
        self.assertEqual(record['source_age'],'unknown')
        self.assertEqual(record['source_quality'],'unassessed')

    def test_republication_never_renews_acquisition_or_observation(self):
        first=system('system');later=system('same_sample')
        self.assertEqual(first['observation_id'],later['observation_id'])
        self.assertEqual(first['effective_at'],later['effective_at'])
        self.assertAlmostEqual(later['age_s'],.78)
        self.assertFalse(system('stale_family')['fresh'])
        self.assertEqual(status('stale_family')['state'],'BOUND')

    def test_sensor_masks_do_not_mark_absent_sensors_unhealthy(self):
        data=system('system')['data'];masks=data['sensor_masks']
        self.assertEqual(masks,{'present':15,'enabled':7,'health':5,'unhealthy_enabled_present':2,'enabled_without_present':0,'healthy_without_present':0})
        self.assertAlmostEqual(data['load_pct']['value'],12.5)
        self.assertAlmostEqual(data['drop_rate_pct']['value'],1.23)
        self.assertEqual(data['errors_comm'],4)
        self.assertEqual(data['zero_extension_support'],'not_distinguishable')
        self.assertNotIn('healthy',data)

    def test_sys_status_unknowns_and_invalid_values_are_independent(self):
        self.assertFalse(system('system_unknown')['encoding_valid'])
        self.assertFalse(system('system_unknown')['data']['load_pct']['known'])
        fallback=one('system_unknown','MAVMOOS_BATTERY_STATUS')['data']
        for field in ('voltage_v','current_a','remaining_pct'):
            self.assertFalse(fallback[field]['known']);self.assertIsNone(fallback[field]['value'])

    def test_prearm_check_is_an_observation_and_can_be_unknown(self):
        for phase, present, enabled, healthy in (
            ('system', False, False, None),
            ('prearm_unhealthy', True, True, False),
            ('prearm_healthy', True, True, True),
            ('prearm_disabled', True, False, None),
        ):
            self.assertEqual(system(phase)['data']['prearm_check'], {
                'present': present, 'enabled': enabled, 'reported_healthy': healthy,
            })

    def test_sys_status_fallback_never_impersonates_identified_battery_zero(self):
        battery=values('multi_battery','MAVMOOS_BATTERY_STATUS')
        self.assertEqual(len(battery),3)
        fallback=next(r for r in battery if r['data']['origin']=='SYS_STATUS')
        self.assertIsNone(fallback['data']['battery_id'])
        self.assertEqual(fallback['data']['identity'],'ambiguous_sys_status_battery')
        self.assertEqual({r['data']['battery_id'] for r in battery if r['data']['origin']=='BATTERY_STATUS'},{0,4})
        self.assertAlmostEqual(fallback['data']['voltage_v']['value'],12.345)
        self.assertAlmostEqual(fallback['data']['current_a']['value'],2.5)

    def test_battery_units_and_negative_charging_current(self):
        data=identified('multi_battery')['data']
        self.assertAlmostEqual(data['temperature_c']['value'],25)
        self.assertAlmostEqual(data['current_a']['value'],-1)
        self.assertAlmostEqual(data['consumed_charge_mah']['value'],1234)
        self.assertAlmostEqual(data['consumed_energy_wh']['value'],20)
        self.assertAlmostEqual(data['reported_voltage_sum_v']['value'],11.4)
        self.assertEqual(data['reported_voltage_slots'],3)
        self.assertEqual(data['time_remaining_s']['value'],600)
        self.assertEqual(data['voltage_interpretation'],'cells_or_split_pack_voltage')

    def test_battery_unknowns_are_not_zero_measurements(self):
        data=identified('battery_unknown')['data']
        for key in ('temperature_c','current_a','consumed_charge_mah','consumed_energy_wh','remaining_pct','time_remaining_s','reported_voltage_sum_v'):
            self.assertFalse(data[key]['known'],key)
            self.assertIsNone(data[key]['value'])
        self.assertEqual(data['reported_voltage_slots'],0)

    def test_invalid_battery_layout_and_values_remain_explicit(self):
        item=identified('battery_invalid');data=item['data']
        self.assertFalse(item['encoding_valid']);self.assertFalse(data['voltage_layout_valid'])
        for key in ('remaining_pct','consumed_charge_mah','consumed_energy_wh','time_remaining_s','reported_voltage_sum_v'):
            self.assertFalse(data[key]['known'])

    def test_extended_cell_zero_and_fault_support_are_not_inferred(self):
        data=identified('battery_extended')['data']
        self.assertEqual(len(data['voltage_slots']),14)
        self.assertTrue(data['voltage_slots'][10]['voltage_v']['known'])
        self.assertAlmostEqual(data['voltage_slots'][10]['voltage_v']['value'],.001)
        self.assertFalse(data['voltage_slots'][11]['voltage_v']['known'])
        self.assertTrue(data['fault_reporting_supported']);self.assertEqual(data['fault_bitmask_raw'],3)

    def test_ardupilot_unused_extended_slots_do_not_inflate_pack_voltage(self):
        data=identified('battery_ardupilot_unused_extension')['data']
        self.assertAlmostEqual(data['reported_voltage_sum_v']['value'],11.4)
        self.assertEqual(data['extended_65535_note'],'ardupilot_unused_sentinel')
        for slot in data['voltage_slots'][10:]:
            self.assertEqual(slot['raw_mv'],65535)
            self.assertFalse(slot['voltage_v']['known'])
        split=identified('battery_split_pack')['data']
        self.assertAlmostEqual(split['reported_voltage_sum_v']['value'],77.879)
        self.assertEqual(split['reported_voltage_slots'],2)

    def test_battery_capacity_is_bounded_and_reset_can_bind_new_ids(self):
        self.assertEqual(status('battery_capacity')['battery_ids'],8)
        self.assertEqual(status('battery_capacity')['battery_capacity_rejected'],1)
        self.assertEqual(status('battery_reset')['battery_ids'],0)
        reset=one('battery_reset','MAVMOOS_BATTERY_STATUS')
        self.assertFalse(reset['present']);self.assertFalse(reset['fresh'])
        self.assertIsNone(reset['data'])
        self.assertEqual(status('battery_after_reset')['battery_ids'],1)
        self.assertTrue(identified('battery_after_reset',9)['fresh'])

    def test_gps_reference_units_and_large_remote_timestamp(self):
        data=gps('gps')['data']
        self.assertAlmostEqual(data['latitude_deg']['value'],42.358456)
        self.assertAlmostEqual(data['longitude_deg']['value'],-71.087589)
        self.assertAlmostEqual(data['altitude_msl_m']['value'],10.083)
        self.assertAlmostEqual(data['ellipsoid_altitude_m']['value'],35)
        self.assertAlmostEqual(data['groundspeed_m_s']['value'],3.45)
        self.assertEqual(data['course_deg']['value'],90)
        self.assertEqual(data['yaw_deg']['value'],0)
        self.assertAlmostEqual(data['hdop']['value'],1.23)
        self.assertAlmostEqual(data['horizontal_accuracy_m']['value'],1.2)
        self.assertAlmostEqual(data['speed_accuracy_m_s']['value'],.123)
        self.assertAlmostEqual(data['track_accuracy_deg']['value'],3.45)
        self.assertEqual(data['remote_time_usec'],'18446744073709551000')
        self.assertEqual(data['remote_clock'],'unmapped_boot_or_unix')

    def test_gps_fix_and_sentinels_do_not_claim_position(self):
        data=gps('gps_unknown')['data']
        for key in ('latitude_deg','longitude_deg','altitude_msl_m','satellites','hdop','vdop','groundspeed_m_s','course_deg','yaw_deg','ellipsoid_altitude_m','horizontal_accuracy_m'):
            self.assertFalse(data[key]['known'],key)
        self.assertEqual(data['yaw_deg']['reason'],'configured_but_unavailable')

    def test_gps_zero_coordinates_and_zero_speed_are_legitimate_with_fix(self):
        data=gps('gps_zero')['data']
        self.assertTrue(data['latitude_deg']['known']);self.assertEqual(data['latitude_deg']['value'],0)
        self.assertTrue(data['longitude_deg']['known']);self.assertEqual(data['longitude_deg']['value'],0)
        self.assertTrue(data['groundspeed_m_s']['known']);self.assertEqual(data['groundspeed_m_s']['value'],0)
        self.assertTrue(data['course_deg']['known']);self.assertEqual(data['course_deg']['value'],0)
        self.assertFalse(data['yaw_deg']['known'])

    def test_gps_invalid_coordinates_course_yaw_do_not_become_valid(self):
        item=gps('gps_invalid');self.assertFalse(item['encoding_valid'])
        for key in ('latitude_deg','longitude_deg','course_deg','yaw_deg'):
            self.assertFalse(item['data'][key]['known'])

    def test_out_of_order_observation_cannot_replace_newer_sample(self):
        before=system('order_before');after=system('order_after')
        self.assertEqual(before['effective_at'],after['effective_at'])
        self.assertEqual(before['observation_id'],after['observation_id'])
        self.assertAlmostEqual(after['data']['load_pct']['value'],12.5)
        self.assertEqual(status('order_after')['out_of_order_rejected'],1)

    def test_older_heartbeat_cannot_shorten_a_live_session(self):
        self.assertEqual(status('older_heartbeat')['state'], 'BOUND')
        self.assertEqual(status('older_heartbeat')['out_of_order_rejected'], 1)
        self.assertTrue(system('older_heartbeat')['fresh'])

    def test_metadata_rejects_nan_future_reverse_basis_and_preboundary(self):
        self.assertFalse(system('invalid_metadata')['present'])
        self.assertEqual(status('invalid_metadata')['metadata_rejected'],7)

    def test_impossible_shapes_are_rejected_but_unrelated_messages_are_ignored(self):
        self.assertFalse(system('wire_shapes')['present'])
        self.assertEqual(status('wire_shapes')['wire_shape_rejected'],5)
        self.assertEqual(status('wire_shapes')['metadata_rejected'],0)

    def test_v2_truncation_and_v1_base_fields_remain_compatible(self):
        self.assertTrue(system('v2_zero_truncation')['present'])
        self.assertTrue(system('v2_zero_truncation')['fresh'])
        data=system('v1_status')['data']
        self.assertAlmostEqual(data['load_pct']['value'],12.5)
        self.assertFalse(data['extension_bytes_observed'])

    def test_foreign_source_and_heartbeat_lapse_do_not_refresh_cached_status(self):
        before=system('source_before');after=system('foreign_source')
        self.assertEqual(before['observation_id'],after['observation_id'])
        self.assertEqual(status('foreign_source')['source_rejected'],1)
        self.assertFalse(system('heartbeat_lapse')['present'])
        self.assertEqual(status('heartbeat_lapse')['state'],'WAITING_HEARTBEAT')
        self.assertFalse(system('heartbeat_only')['present'])
        self.assertEqual(status('heartbeat_only')['state'],'BOUND')

    def test_clock_rollback_latches_until_explicit_reset(self):
        self.assertEqual(status('clock_fault')['state'],'CLOCK_FAULT')
        self.assertFalse(system('clock_fault')['fresh'])
        self.assertEqual(status('clock_recovery')['state'],'BOUND')
        self.assertTrue(system('clock_recovery')['fresh'])

    def test_ascii_text_is_json_escaped_and_raw_reconstructable(self):
        event=text('text_ascii')[0]
        self.assertEqual(event['status'],'complete')
        self.assertEqual(event['text'],'quotes "slash\\ newline\n\t')
        self.assertEqual(bytes.fromhex(event['raw_hex']).decode(),event['text'])
        self.assertTrue(event['utf8_valid'])

    def test_all_controls_and_invalid_unicode_sequences_preserve_raw_bytes(self):
        event=text('text_controls')[0]
        self.assertEqual(bytes.fromhex(event['raw_hex']),bytes(range(1,32))+b'"\\')
        self.assertEqual(event['text'].encode(),bytes.fromhex(event['raw_hex']))
        invalid=text('text_invalid_sequences')[0]
        self.assertFalse(invalid['utf8_valid'])
        self.assertEqual(bytes.fromhex(invalid['raw_hex']),bytes.fromhex('c080eda080f4908080'))

    def test_full_single_chunk_and_empty_terminator_have_explicit_completion(self):
        event=text('text_full_single')[0]
        self.assertEqual(event['status'],'complete');self.assertEqual(event['byte_count'],50)
        self.assertFalse(event['severity_valid'])
        terminated=text('text_empty_terminator')[0]
        self.assertEqual(terminated['status'],'complete');self.assertEqual(terminated['byte_count'],50)

    def test_old_acquisition_cannot_start_a_new_chunk_deadline(self):
        event=text('text_acquisition_expired')[0]
        self.assertEqual(event['status'],'incomplete')
        self.assertEqual(event['reason'],'acquisition_expired')
        self.assertAlmostEqual(event['original_expires'],100.45)
        self.assertEqual(status('text_acquisition_expired')['text_assemblies'],0)

    def test_deterministic_malformed_frames_remain_bounded_without_effects(self):
        states=values('bounded_random_frames','MAVMOOS_SYSTEM_STATUS')
        self.assertGreaterEqual(len(states),4)
        for state in states:
            self.assertLessEqual(state['battery_ids'],8)
            self.assertLessEqual(state['text_assemblies'],4)
        self.assertGreater(states[-1]['wire_shape_rejected'],1000)
        for event in text('bounded_random_frames'):
            if 'byte_count' in event:
                self.assertLessEqual(event['byte_count'],800)

    def test_valid_utf8_and_invalid_octets_are_distinguished(self):
        good=text('text_utf8')[0];bad=text('text_bad_utf8')[0]
        self.assertEqual(good['text'],'café');self.assertTrue(good['utf8_valid'])
        self.assertFalse(bad['utf8_valid']);self.assertEqual(bytes.fromhex(bad['raw_hex']),b'bad\xff')

    def test_utf8_can_cross_chunk_boundary_and_is_emitted_once(self):
        event=text('text_assembled_utf8')[0]
        self.assertEqual(event['status'],'complete');self.assertTrue(event['utf8_valid'])
        self.assertEqual(event['text'],'a'*49+'édone')
        self.assertAlmostEqual(event['first_effective_at'],100.2)
        self.assertAlmostEqual(event['last_read_at'],100.3)
        self.assertEqual(text('text_not_republished'),[])

    def test_duplicate_chunk_does_not_duplicate_text_or_renew_deadline(self):
        event=text('text_duplicate')[0]
        self.assertEqual(event['text'],'a'*50+'end')
        self.assertAlmostEqual(event['original_expires'],102.2)
        self.assertEqual(status('text_duplicate')['text_duplicates'],1)

    def test_gap_missing_initial_and_conflicting_chunks_are_incomplete(self):
        for case in ('text_gap','text_missing','text_conflicting','text_severity'):
            events=text(case);self.assertEqual(len(events),1,case)
            self.assertEqual(events[0]['status'],'incomplete',case)

    def test_original_assembly_timeout_survives_duplicates(self):
        event=text('text_expired')[0]
        self.assertEqual(event['status'],'incomplete')
        self.assertEqual(event['reason'],'original_chunk_deadline')
        self.assertAlmostEqual(event['original_expires'],100.7)
        self.assertEqual(text('text_late_continuation')[0]['status'],'incomplete')

    def test_text_capacity_and_reset_retire_assemblies(self):
        self.assertEqual(status('text_capacity')['text_assemblies'],4)
        self.assertEqual(status('text_capacity')['text_capacity_rejected'],1)
        self.assertEqual(status('text_reset')['text_assemblies'],0)
        self.assertEqual(len([e for e in text('text_reset') if e['status']=='incomplete']),4)
        self.assertEqual(len([e for e in text('text_reset') if e['status']=='invalidated']),1)

    def test_chunk_and_event_capacity_are_fixed(self):
        event=text('text_chunk_bound')[0]
        self.assertEqual(event['status'],'incomplete');self.assertEqual(event['byte_count'],800)
        self.assertEqual(len(text('event_capacity')),16)
        self.assertEqual(status('event_capacity')['text_events_dropped'],9)

    def test_publish_interval_caches_new_sample_without_unbounded_republication(self):
        self.assertEqual(values('interval_suppressed','MAVMOOS_SYS_STATUS'),[])
        later=system('interval_later')
        self.assertAlmostEqual(later['effective_at'],100.3)
        self.assertAlmostEqual(later['age_s'],.5)

    def test_configuration_rejections_never_cause_effects(self):
        self.assertTrue(status('config_rejections')['read_only'])
        self.assertFalse(status('config_rejections')['authoritative_for_control'])

    def test_reset_before_retained_highwater_stays_faulted(self):
        self.assertTrue(system('hardening_reliable')['fresh'])
        state=status('hardening_early_reset')
        self.assertEqual(state['state'],'CLOCK_FAULT')
        self.assertEqual(state['reset_reason'],'behind_highwater')
        self.assertFalse(system('hardening_early_reset')['present'])

    def test_clock_catchup_does_not_implicitly_recover_after_bad_reset(self):
        for case in ('hardening_input_before_highwater','hardening_catchup_without_reset'):
            self.assertEqual(status(case)['state'],'CLOCK_FAULT',case)
            self.assertFalse(system(case)['present'],case)
            self.assertFalse(system(case)['fresh'],case)

    def test_valid_explicit_reset_retains_new_floor_before_fresh_recovery(self):
        state=status('hardening_valid_reset')
        self.assertEqual(state['state'],'WAITING_HEARTBEAT')
        self.assertAlmostEqual(state['session_boundary'],102.1)
        self.assertFalse(system('hardening_valid_reset')['present'])
        self.assertEqual(status('hardening_old_floor')['state'],'WAITING_HEARTBEAT')
        self.assertFalse(system('hardening_old_floor')['present'])
        current=system('hardening_recovered')
        self.assertEqual(status('hardening_recovered')['state'],'BOUND')
        self.assertTrue(current['fresh']);self.assertTrue(current['present'])
        self.assertAlmostEqual(current['effective_at'],102.45)
        self.assertAlmostEqual(current['read_at'],102.45)
        self.assertGreater(current['observation_id'],system('hardening_reliable')['observation_id'])

    def test_negative_nonfinite_and_behind_resets_latch_without_prior_clock_fault(self):
        for index in range(4):
            case='hardening_invalid_reset_'+str(index)
            self.assertEqual(status(case)['state'],'CLOCK_FAULT',case)
            self.assertFalse(system(case)['present'],case)
            self.assertFalse(system(case)['fresh'],case)

    def test_reset_at_equal_reliable_time_can_recover(self):
        state=status('hardening_equal_reset')
        self.assertEqual(state['state'],'BOUND')
        self.assertAlmostEqual(state['session_boundary'],101)
        self.assertTrue(system('hardening_equal_reset')['fresh'])
        self.assertAlmostEqual(system('hardening_equal_reset')['effective_at'],101.2)

if __name__=='__main__':
    unittest.main(argv=[sys.argv[0],*remaining])
