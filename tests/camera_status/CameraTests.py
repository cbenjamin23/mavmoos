#!/usr/bin/env python3
import argparse
import json
import subprocess

from audit import (CAPTURE, INFORMATION, SETTINGS, STORAGE, capture_matches,
                   information_matches, settings_matches, storage_matches)


def check(value, reason):
    if not value:
        raise AssertionError(reason)


def verify(rows):
    for stage, samples in rows.items():
        check(set(samples) == {INFORMATION, SETTINGS, CAPTURE, STORAGE}, f'{stage}: unexpected camera output')
        for sample in samples.values():
            check(sample['schema'] == 1 and sample['read_only'] is True and sample['authoritative_for_control'] is False
                  and sample['source_age'] == 'unknown' and sample['sample_timeout_s'] == .6
                  and sample['publish_interval_s'] == .1, f'{stage}: passive metadata changed')
    for stage in ('no_fc_heartbeat', 'fc_lost', 'camera_cannot_rebind', 'reset', 'clock_rollback'):
        check(all(not sample['present'] and not sample['fresh'] and sample['data'] is None
                  for sample in rows[stage].values()), f'{stage}: camera data survived source/session loss')

    fresh = rows['fresh']
    for key, expected in ((INFORMATION, information_matches), (SETTINGS, settings_matches),
                          (CAPTURE, capture_matches), (STORAGE, storage_matches)):
        sample = fresh[key]
        check(sample['fresh'] and sample['autopilot'] == 3 and sample['system_id'] == 1
              and sample['component_id'] == 100 and expected(sample['data']), f'{key}: decoded sample changed')
        retained = rows['rejected'][key]
        identity = ('observation_id', 'effective_at', 'read_at')
        if key == STORAGE:
            payload_unchanged = (len(retained['data']) == len(sample['data']) and
                                 all(all(before[field] == after[field] for field in (*identity, 'data'))
                                     for before, after in zip(sample['data'], retained['data'])))
        else:
            payload_unchanged = retained['data'] == sample['data']
        check(all(retained[field] == sample[field] for field in identity) and payload_unchanged
              and retained['source_rejected'] >= 6 and retained['wire_rejected'] == 6 and retained['order_rejected'] == 1,
              f'{key}: invalid, old or foreign message replaced accepted evidence')
    info = fresh[INFORMATION]
    check(info['effective_at'] == 100.24 and info['read_at'] == 100.25 and info['age_basis'] == 'QUEUE_EMPTY_FLOOR'
          and info['age_is_upper_bound'], 'camera original acquisition age changed')

    unknown = rows['unavailable_fields']
    settings = unknown[SETTINGS]['data']
    check(settings['payload_length'] == 5 and settings['mode'] == 'IMAGE_SURVEY'
          and settings['focus_pct'] is None and settings['zoom_pct'] is None,
          'absent camera extensions became zero-valued settings')
    capture = unknown[CAPTURE]['data']
    check(capture['recording_time_ms'] is None and capture['image_count'] is None and capture['image_count_raw'] == -1,
          'unavailable recording time or image count became a measured zero')
    second = unknown[STORAGE]['data'][1]['data']
    check(second['storage_id'] == 2 and second['status'] == 'NOT_SUPPORTED' and
          all(second[key] is None for key in ('total_capacity_mib', 'used_capacity_mib', 'available_capacity_mib')),
          'nonready storage capacities became measurements')

    first, second = rows['independent_storage_ages'][STORAGE]['data']
    check(not first['fresh'] and second['fresh'] and first['effective_at'] == fresh[STORAGE]['effective_at']
          and first['data']['storage_id'] == 1 and second['data']['storage_id'] == 2,
          'updating one storage ID renewed another')
    bounded = rows['bounded_storage'][STORAGE]
    check(not bounded['atomic'] and bounded['cache_capacity'] == 8 and bounded['capacity_rejected'] == 1 and
          [entry['data']['storage_id'] for entry in bounded['data']] == list(range(1, 9))
          and bounded['data'][0]['data']['time_boot_ms'] == 999,
          'storage cache exceeded its bound or lost a retained ID update')

    rebound = rows['fc_rebound'][INFORMATION]
    check(rebound['session_epoch'] > info['session_epoch'] and rebound['fresh'] and information_matches(rebound['data'])
          and not rows['fc_rebound'][STORAGE]['present']
          and rows['reset'][STORAGE]['metadata_rejected'] >= 1 and rows['clock_rollback'][INFORMATION]['clock_fault'],
          'new FC session or clock reset reused retired camera records')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--driver', required=True)
    parser.add_argument('--module', required=True)
    args = parser.parse_args()
    output = subprocess.check_output([args.driver, args.module], text=True, timeout=5)
    check('fixture.invalid' not in output and 'ignored-image.jpg' not in output, 'camera URI escaped into MOOS text')
    rows = {item['phase']: item['posts'] for item in map(json.loads, output.splitlines())}
    verify(rows)
    print(f'PASS {len(rows)} camera DSO phases')


if __name__ == '__main__':
    main()
