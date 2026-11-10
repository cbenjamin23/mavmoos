#!/usr/bin/env python3
import argparse
import json
import subprocess

from audit import REPORT, REQUEST, REPORT_DATA, REQUEST_DATA


def check(value, reason):
    if not value:
        raise AssertionError(reason)


def verify(rows):
    for stage, samples in rows.items():
        check(set(samples) == {REPORT, REQUEST}, f'{stage}: unexpected provider output')
        timeout = 3 if stage == 'px4_shaped_common_report' else .6
        for key, expected_id in ((REPORT, 136), (REQUEST, 133)):
            sample = samples[key]
            check(sample['schema'] == 1 and sample['message_id'] == expected_id and sample['read_only'] is True
                  and sample['authoritative_for_control'] is False and sample['source_age'] == 'unknown'
                  and sample['sample_timeout_s'] == timeout and sample['publish_interval_s'] == .1,
                  f'{stage}: invalid passive metadata or a failed batch changed timing')
    for stage in ('boot', 'pre_heartbeat', 'session_expired', 'data_cannot_rebind', 'explicit_reset', 'clock_regression'):
        check(all(not s['present'] and s['data'] is None and s['effective_at'] is None
                  for s in rows[stage].values()), f'{stage}: stale session retained observations')

    fresh = rows['fresh']
    check(fresh[REPORT]['data'] == REPORT_DATA and fresh[REQUEST]['data'] == REQUEST_DATA,
          'generated MAVLink report/request did not preserve units, datums, signed height or the full mask')
    for key, effective, read in ((REPORT, 100.28, 100.31), (REQUEST, 100.35, 100.36)):
        sample = fresh[key]
        check(sample['effective_at'] == effective and sample['read_at'] == read and sample['fresh']
              and sample['age_basis'] == 'QUEUE_EMPTY_FLOOR' and sample['age_is_upper_bound']
              and sample['autopilot'] == 3 and sample['system_id'] == sample['component_id'] == 1,
              'wrong observation provenance')
        retained = rows['rejected'][key]
        check(all(retained[field] == sample[field] for field in ('effective_at', 'read_at', 'observation_id', 'data'))
              and retained['invalid_samples'] == 4 and retained['wire_rejected'] == 8
              and retained['source_rejected'] >= 3 and retained['order_rejected'] == 1,
              'foreign, malformed or older messages replaced original accepted evidence')

    unavailable = rows['unavailable'][REPORT]
    check(unavailable['fresh'] and unavailable['data'] == {
        **REPORT_DATA, 'spacing_m': 0, 'terrain_available': False, 'terrain_height_msl_m': None,
        'terrain_height_reported_msl_m': 123.5, 'current_height_reported_m': 0,
    }, 'zero spacing incorrectly certified extrapolated terrain or zero current height')
    check(rows['unavailable'][REQUEST]['observation_id'] == fresh[REQUEST]['observation_id'],
          'report renewed independent request')
    for key, prior in ((REPORT, unavailable), (REQUEST, fresh[REQUEST])):
        sample = rows['stale'][key]
        check(sample['present'] and not sample['fresh'] and sample['data'] == prior['data']
              and sample['effective_at'] == prior['effective_at'] and sample['age_s'] > .6,
              'heartbeat or publication refreshed original observation age')

    zero = rows['v1_zero_at_equator'][REPORT]
    check(zero['fresh'] and zero['session_epoch'] > fresh[REPORT]['session_epoch'] and zero['data'] == {
        **REPORT_DATA, 'latitude_deg': 0, 'longitude_deg': 0, 'spacing_m': 30,
        'terrain_height_msl_m': 0, 'terrain_height_reported_msl_m': 0, 'current_height_reported_m': 0,
    }, 'valid zero elevations or coordinates lost on MAVLink1 recovery')
    check(rows['v1_zero_at_equator'][REQUEST]['data'] == {
        **REQUEST_DATA, 'requested_blocks': 0, 'grid_mask_hex': '0x00000000000000',
    }, 'zero request mask incorrectly inferred data fulfillment')
    check(rows['explicit_reset'][REPORT]['metadata_rejected'] >= 1 and
          rows['ready_after_reset'][REPORT]['session_epoch'] > zero['session_epoch'] and
          rows['clock_regression'][REPORT]['clock_fault'],
          'reset or clock fault did not retire old observations')
    px4 = rows['px4_shaped_common_report'][REPORT]
    check(px4['data'] == REPORT_DATA and px4['autopilot'] == 12 and px4['sample_timeout_s'] == 3
          and not rows['px4_shaped_common_report'][REQUEST]['present'],
          'host-bound common report failed for a generated PX4 identity')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--driver', required=True)
    parser.add_argument('--module', required=True)
    args = parser.parse_args()
    output = subprocess.check_output([args.driver, args.module], text=True, timeout=5)
    rows = {item['phase']: item['posts'] for item in map(json.loads, output.splitlines())}
    verify(rows)
    print(f'PASS {len(rows)} terrain DSO phases')


if __name__ == '__main__':
    main()
