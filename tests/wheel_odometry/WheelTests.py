#!/usr/bin/env python3
import argparse
import json
import subprocess

from audit import WHEELS, wheel_matches

LOCAL = 'MAVMOOS_LOCAL_KINEMATICS'


def check(value, reason):
    if not value:
        raise AssertionError(reason)


def verify(rows):
    for stage, samples in rows.items():
        check(set(samples) == {LOCAL, 'MAVMOOS_ODOMETRY', 'MAVMOOS_LOCAL_ORIGIN', WHEELS},
              f'{stage}: unexpected observation output')
        sample = samples[WHEELS]
        check(sample['schema'] == 1 and sample['read_only'] is True and sample['authoritative_for_control'] is False
              and sample['source_age'] == 'unknown' and sample['publish_interval_s'] == .1
              and sample['sample_timeout_s'] == (1 if stage == 'px4_shaped' else .6),
              f'{stage}: invalid passive metadata')
    for stage in ('unbound', 'lost', 'data_cannot_rebind', 'reset', 'clock_rollback'):
        sample = rows[stage][WHEELS]
        check(not sample['present'] and not sample['fresh'] and sample['data'] is None
              and sample['effective_at'] is None and sample['observation_id'] == 0, f'{stage}: retired wheel data survived')

    fresh = rows['fresh'][WHEELS]
    check(wheel_matches(fresh['data']) and fresh['fresh'] and fresh['system_id'] == fresh['component_id'] == 1
          and fresh['autopilot'] == 3 and fresh['effective_at'] == 100.24 and fresh['read_at'] == 100.28
          and fresh['age_basis'] == 'QUEUE_EMPTY_FLOOR' and fresh['age_is_upper_bound'],
          'wheel sign, encoder order, NaN or acquisition metadata changed')
    retained = rows['rejected'][WHEELS]
    check(all(retained[key] == fresh[key] for key in ('data', 'effective_at', 'read_at', 'observation_id'))
          and retained['wire_rejected'] == 5 and retained['order_rejected'] == 1 and retained['source_rejected'] >= 3
          and rows['rejected'][LOCAL]['fresh'], 'foreign, malformed, older or independent messages overwrote the wheel frame')
    check(wheel_matches(rows['unused_tail'][WHEELS]['data']), 'unreported encoder slots affected validation')
    full = rows['full'][WHEELS]['data']
    check(full['wheel_count'] == 16 and full['distance_m'] == [(-i if i % 2 else i) for i in range(16)]
          and full['time_usec_raw'] == '42' and full['unknown_wheel_indices'] == [],
          'bounded full count, remote timestamp or raw order changed')
    empty = rows['zero_count'][WHEELS]
    check(empty['present'] and empty['data']['wheel_count'] == 0 and empty['data']['payload_length'] == 1
          and empty['data']['distance_m'] == [] and empty['data']['unknown_wheel_indices'] == [],
          'explicit no-wheels report became zero-valued measurements')
    stale = rows['stale'][WHEELS]
    check(stale['present'] and not stale['fresh'] and stale['data'] == empty['data']
          and stale['effective_at'] == empty['effective_at'] and stale['age_s'] > .6,
          'heartbeat or republication renewed original wheel age')
    recovered = rows['recovered'][WHEELS]
    after_reset = rows['after_reset'][WHEELS]
    check(recovered['session_epoch'] > fresh['session_epoch'] and after_reset['session_epoch'] > recovered['session_epoch']
          and after_reset['fresh'] and wheel_matches(after_reset['data']) and rows['reset'][WHEELS]['metadata_rejected'] >= 1
          and rows['clock_rollback'][WHEELS]['clock_fault'], 'session and clock reset behavior changed')
    check(rows['px4_shaped'][WHEELS]['autopilot'] == 12 and wheel_matches(rows['px4_shaped'][WHEELS]['data']),
          'common wire decoding failed with a selected generated PX4 identity')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--driver', required=True)
    parser.add_argument('--module', required=True)
    args = parser.parse_args()
    output = subprocess.check_output([args.driver, args.module], text=True, timeout=5)
    rows = {item['phase']: item['posts'] for item in map(json.loads, output.splitlines())}
    verify(rows)
    print(f'PASS {len(rows)} wheel odometry DSO phases')


if __name__ == '__main__':
    main()
