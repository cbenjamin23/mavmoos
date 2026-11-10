#!/usr/bin/env python3
import argparse
import json
import subprocess

from audit import MESSAGE_ID, PRIMARY, SECONDARY, depth_matches


def require(condition, reason):
    if not condition:
        raise AssertionError(reason)


def sensors(post):
    return {sample['sensor_id']: sample for sample in post['samples']}


def verify(rows):
    for stage, post in rows.items():
        require(post['schema'] == 1 and post['family'] == 'WATER_DEPTH' and post['message_id'] == MESSAGE_ID and
                post['sensor_limit'] == 10 and post['read_only'] is True and post['atomic'] is False and
                post['authoritative_for_control'] is False and post['present'] is bool(post['samples']) and
                len(post['samples']) <= 10, f'{stage}: incorrect read-only envelope')
    for stage in ('boot', 'pre_heartbeat', 'session_expired', 'data_cannot_rebind', 'explicit_reset',
                  'vehicle_changed', 'ground_rover', 'ardusub', 'px4_boat'):
        require(rows[stage]['samples'] == [], f'{stage}: stale or unsupported producer admitted')

    require(depth_matches(rows['fresh']), 'generated healthy and unhealthy sensor values or provenance differ')
    fresh = sensors(rows['fresh'])
    primary = fresh[0]['metadata']
    secondary = fresh[5]['metadata']
    require(primary['effective_at'] == 100.27 and primary['read_at'] == 100.30 and
            primary['age_basis'] == 'QUEUE_EMPTY_FLOOR' and primary['age_is_upper_bound'] is True and
            secondary['effective_at'] == secondary['read_at'] == 100.33 and
            secondary['age_basis'] == 'READ_ACQUISITION' and secondary['age_is_upper_bound'] is False and
            primary['sample_timeout_s'] == .6 and secondary['publish_interval_s'] == .1,
            'wrong original age or failed configuration altered retained timing')

    rejected = sensors(rows['rejected'])
    for sensor, original in fresh.items():
        after = rejected[sensor]
        require(after['data'] == original['data'] and
                all(after['metadata'][name] == original['metadata'][name] for name in
                    ('effective_at', 'read_at', 'observation_id')) and
                after['metadata']['source_rejected'] >= 3 and after['metadata']['wire_rejected'] == 6 and
                after['metadata']['order_rejected'] == 1 and rows['rejected']['invalid_samples'] == 6,
                'foreign, malformed, over-capacity or older reading replaced accepted evidence')

    unknown = sensors(rows['unknown'])[9]['data']
    require(unknown == {
        **PRIMARY, 'health_raw': 7, 'health': 'unknown', 'distance_m_raw': None,
        'bottom_distance_uncorrected_m': None, 'distance_status': 'unavailable_nonfinite',
        'latitude_e7_raw': -900000001, 'longitude_e7_raw': 1800000000, 'position_status': 'out_of_bounds',
        'latitude_deg': None, 'longitude_deg': None, 'vehicle_altitude_msl_m_reported': None,
        'vehicle_roll_rad_reported': None, 'vehicle_yaw_rad_reported': None,
        'temperature_c_raw': None, 'temperature_c': None, 'temperature_status': 'unavailable_nonfinite',
    }, 'unknown health, nonfinite values or coordinate limits were collapsed')
    negative = sensors(rows['negative'])[0]['data']
    require(negative == {
        **PRIMARY, 'distance_m_raw': -3.25, 'bottom_distance_uncorrected_m': None,
        'distance_status': 'invalid_negative', 'latitude_e7_raw': 0, 'longitude_e7_raw': 0,
        'position_status': 'zero_ambiguous', 'latitude_deg': None, 'longitude_deg': None,
    }, 'negative range or ambiguous zero position was interpreted as a depth or fix')

    zero = sensors(rows['zero'])[5]['data']
    require(zero == {
        **SECONDARY, 'time_boot_ms_raw': PRIMARY['time_boot_ms_raw'], 'health_raw': 1, 'health': 'healthy',
        'distance_m_raw': 0, 'bottom_distance_uncorrected_m': 0, 'distance_status': 'reported_healthy',
        'latitude_e7_raw': 0, 'latitude_deg': 0,
    }, 'valid zero beam range or a real equatorial coordinate was lost')
    stale = sensors(rows['independent_staleness'])
    require(not stale[0]['metadata']['fresh'] and stale[0]['data'] == negative and
            stale[0]['metadata']['effective_at'] == 100.66 and stale[5]['metadata']['fresh'] and
            stale[5]['metadata']['effective_at'] == 100.82,
            'unrelated sensor or heartbeat changed an original acquisition age')

    trimmed = sensors(rows['trimmed_zero'])[0]
    require(trimmed['payload_length'] == 1 and trimmed['metadata']['fresh'] is True and
            trimmed['data']['health'] == 'unhealthy' and trimmed['data']['bottom_distance_uncorrected_m'] is None and
            trimmed['data']['position_status'] == 'zero_ambiguous' and trimmed['data']['time_boot_ms_raw'] == 0,
            'legal MAVLink2 tail trim invented a healthy reading or lost the raw timestamp')
    recovered = sensors(rows['ready_after_reset'])[0]
    require(recovered['metadata']['session_epoch'] > trimmed['metadata']['session_epoch'] >
            primary['session_epoch'] and recovered['metadata']['metadata_rejected'] >= 1 and
            recovered['data'] == PRIMARY, 'session or explicit reset failed to fence original observations')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--driver', required=True)
    parser.add_argument('--module', required=True)
    args = parser.parse_args()
    data = subprocess.check_output([args.driver, args.module], text=True, timeout=5)
    rows = {item['phase']: item['sample'] for item in map(json.loads, data.splitlines())}
    verify(rows)
    print(f'PASS {len(rows)} water_depth DSO phases')


if __name__ == '__main__':
    main()
