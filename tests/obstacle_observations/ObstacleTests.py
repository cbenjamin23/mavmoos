#!/usr/bin/env python3
import argparse
import json
import subprocess

from audit import OBSTACLE, expected_sector, obstacle_matches


def check(value, reason):
    if not value:
        raise AssertionError(reason)


def verify(rows):
    for stage, posts in rows.items():
        check(set(posts) == {'MAVMOOS_DISTANCE_SENSORS', 'MAVMOOS_DISTANCE_STATUS', OBSTACLE},
              f'{stage}: publication roster drift')
        sample = posts[OBSTACLE]
        check(sample['schema'] == 1 and sample['message_id'] == 330 and sample['family'] == 'OBSTACLE_DISTANCE'
              and sample['read_only'] is True and sample['authoritative_for_control'] is False
              and sample['source_age'] == 'unknown' and sample['sample_timeout_s'] == .6
              and sample['publish_interval_s'] == .1, f'{stage}: invalid passive provenance')
    for stage in ('unbound', 'expired_session', 'data_cannot_rebind', 'reset', 'clock_rollback'):
        sample = rows[stage][OBSTACLE]
        check(not sample['present'] and not sample['fresh'] and sample['data'] is None
              and sample['effective_at'] is None and sample['observation_id'] == 0,
              f'{stage}: retired obstacle data leaked into a new session')

    body = rows['body'][OBSTACLE]
    check(obstacle_matches(body['data']) and body['fresh'] and body['system_id'] == body['component_id'] == 1
          and body['autopilot'] == 3 and body['effective_at'] == 100.2 and body['read_at'] == 100.24
          and body['age_basis'] == 'QUEUE_EMPTY_FLOOR' and body['age_is_upper_bound'],
          'signed bearing metadata, 72 raw bins, sentinels or original times changed')
    kept = rows['rejected'][OBSTACLE]
    check(all(kept[key] == body[key] for key in ('data', 'effective_at', 'read_at', 'observation_id'))
          and kept['wire_shape_rejected'] == 2 and kept['invalid_samples'] == 5
          and kept['out_of_order_rejected'] == 2 and kept['source_rejected'] >= 3,
          'old, foreign, malformed or scalar messages overwrote the sector frame')
    scalar = rows['rejected']['MAVMOOS_DISTANCE_SENSORS']['sensors']
    check(len(scalar) == 1 and scalar[0]['data']['id'] == 7 and scalar[0]['data']['current_distance_cm'] == 120,
          'sector traffic changed the existing scalar observation')

    default = rows['default_frame'][OBSTACLE]['data']
    check(default['frame_raw'] == 0 and default['frame_name'] == 'GLOBAL' and not default['frame_explicit']
          and default['increment_source'] == 'increment' and default['increment_deg'] == 5
          and default['angle_offset_deg'] == 0 and default['payload_length'] == 158,
          'omitted extensions did not follow MAVLink2 defaults')
    unknown_frame = rows['uninterpreted_frame'][OBSTACLE]['data']
    check(unknown_frame['frame_raw'] == 1 and unknown_frame['frame_name'] == 'UNINTERPRETED'
          and unknown_frame['sensor_type_raw'] == 255 and unknown_frame['sensor_type_name'] == 'UNRECOGNIZED'
          and unknown_frame['no_hit_sentinel_cm'] is None
          and unknown_frame['sectors'][3] == expected_sector(65535, 'unknown')
          and unknown_frame['sectors'][6] == expected_sector(65534, 'in_range', 655.34),
          'uninterpreted frame or no-hit sentinel collision was promoted into a clearance')

    stale = rows['stale'][OBSTACLE]
    check(stale['present'] and not stale['fresh'] and stale['data'] == unknown_frame
          and stale['effective_at'] == rows['uninterpreted_frame'][OBSTACLE]['effective_at']
          and stale['age_s'] > .6, 'heartbeat or republication renewed the sector age')
    new = rows['new_session'][OBSTACLE]
    recovered = rows['after_reset'][OBSTACLE]
    check(new['session_epoch'] > body['session_epoch'] and recovered['session_epoch'] > new['session_epoch']
          and recovered['fresh'] and obstacle_matches(recovered['data'])
          and rows['reset'][OBSTACLE]['metadata_rejected'] >= 1
          and rows['clock_rollback'][OBSTACLE]['clock_fault'], 'session or clock recovery reused retired data')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--driver', required=True)
    parser.add_argument('--module', required=True)
    args = parser.parse_args()
    output = subprocess.check_output([args.driver, args.module], text=True, timeout=5)
    rows = {item['phase']: item['posts'] for item in map(json.loads, output.splitlines())}
    verify(rows)
    print(f'PASS {len(rows)} obstacle DSO phases')


if __name__ == '__main__':
    main()
