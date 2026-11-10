"""Generated boat frames shared by the DSO and grouped real-app witnesses."""

WATER_DEPTH = 'MAVMOOS_WATER_DEPTH'
KEYS = (WATER_DEPTH,)
MESSAGE_ID = 11038
PRIMARY = {
    'time_boot_ms_raw': 4294967295,
    'health_raw': 1,
    'health': 'healthy',
    'distance_m_raw': 4.75,
    'bottom_distance_uncorrected_m': 4.75,
    'distance_status': 'reported_healthy',
    'latitude_e7_raw': 423584567,
    'longitude_e7_raw': -710875891,
    'position_status': 'reported_unconfirmed',
    'latitude_deg': 42.3584567,
    'longitude_deg': -71.0875891,
    'vehicle_altitude_msl_m_reported': -1.25,
    'vehicle_roll_rad_reported': -.25,
    'vehicle_pitch_rad_reported': .125,
    'vehicle_yaw_rad_reported': 1.75,
    'temperature_c_raw': -2.5,
    'temperature_c': -2.5,
    'temperature_status': 'reported_nonzero',
}
SECONDARY = {
    **PRIMARY, 'time_boot_ms_raw': 2, 'health_raw': 0, 'health': 'unhealthy',
    'distance_m_raw': 31.25, 'bottom_distance_uncorrected_m': None, 'distance_status': 'unavailable_health',
    'temperature_c_raw': 0, 'temperature_c': None, 'temperature_status': 'zero_ambiguous',
}


def depth_matches(post):
    if not isinstance(post, dict) or not (
            post.get('schema') == 1 and post.get('family') == 'WATER_DEPTH' and
            post.get('message_id') == MESSAGE_ID and post.get('sensor_limit') == 10 and
            post.get('present') is True and post.get('read_only') is True and
            post.get('atomic') is False and post.get('authoritative_for_control') is False):
        return False
    samples = post.get('samples', [])
    if [sample['sensor_id'] for sample in samples] != [0, 5]:
        return False
    observations = set()
    for sample, expected, length in zip(samples, (PRIMARY, SECONDARY), (38, 37)):
        metadata = sample['metadata']
        if not (sample['wire_version'] == 2 and sample['payload_length'] == length and
                sample['data'] == expected and metadata['present'] is True and metadata['fresh'] is True and
                metadata['read_only'] is True and metadata['authoritative_for_control'] is False and
                metadata['source_age'] == 'unknown' and metadata['source_quality'] == 'unassessed' and
                (metadata['system_id'], metadata['component_id'], metadata['autopilot']) == (1, 1, 3) and
                0 < metadata['effective_at'] <= metadata['read_at'] and metadata['age_s'] >= 0 and
                metadata['observation_id'] > 0):
            return False
        observations.add(metadata['observation_id'])
    return len(observations) == 2


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def audit_loopback(before_bind, before_recovery, samples, autopilot):
    initial = before_bind.get(WATER_DEPTH, [])
    prefix = before_recovery.get(WATER_DEPTH, [])
    records = samples.get(WATER_DEPTH, [])
    require(autopilot == 3 and initial and prefix[:len(initial)] == initial and records[:len(prefix)] == prefix
            and all(not row['present'] and not row['samples'] for row in initial)
            and not prefix[-1]['present'] and not prefix[-1]['samples'],
            'WATER_DEPTH absent prebind or clear phase differs')
    expected = {0: (38, PRIMARY), 5: (37, SECONDARY)}
    first, earlier, stale, recovered = {}, {}, set(), set()
    cleared = False
    for index, post in enumerate(records):
        require(post['schema'] == 1 and post['family'] == 'WATER_DEPTH' and post['message_id'] == MESSAGE_ID
                and post['read_only'] is True and post['authoritative_for_control'] is False
                and post['atomic'] is False and post['sensor_limit'] == 10 and
                post['present'] is bool(post['samples']) and post['invalid_samples'] == 0,
                'WATER_DEPTH observation envelope differs')
        if not post['samples'] and stale == expected.keys() and index < len(prefix):
            cleared = True
        require(not cleared or not post['samples'] or index >= len(prefix),
                'WATER_DEPTH data restored an unbound session')
        ids = [sample['sensor_id'] for sample in post['samples']]
        require(ids == sorted(set(ids)) and set(ids) <= expected.keys(),
                'WATER_DEPTH foreign, duplicate or unexpected sensor admitted')
        for sample in post['samples']:
            sensor = sample['sensor_id']
            meta = sample['metadata']
            require(sample['wire_version'] == 2 and (sample['payload_length'], sample['data']) == expected[sensor]
                    and meta['schema'] == 1 and meta['present'] is True and meta['read_only'] is True
                    and meta['authoritative_for_control'] is False and meta['clock_fault'] is False
                    and (meta['system_id'], meta['component_id'], meta['autopilot']) == (1, 1, 3)
                    and meta['source_age'] == 'unknown' and meta['source_quality'] == 'unassessed'
                    and meta['age_basis'] == 'QUEUE_EMPTY_FLOOR' and meta['age_is_upper_bound'] is True
                    and 0 < meta['effective_at'] <= meta['read_at']
                    and meta['age_s'] + 1e-6 >= meta['read_at'] - meta['effective_at']
                    and meta['fresh'] is (meta['age_s'] <= meta['sample_timeout_s'])
                    and all(type(meta[key]) is int and meta[key] > 0 for key in ('session_epoch', 'observation_id')),
                    'WATER_DEPTH wire values, bound source or original age differs')
            identity = sensor, meta['session_epoch'], meta['observation_id']
            times = meta['effective_at'], meta['read_at']
            if identity in earlier:
                old_times, old_age = earlier[identity]
                require(times == old_times and meta['age_s'] >= old_age,
                        'WATER_DEPTH publication renewed an original sample')
            earlier[identity] = times, meta['age_s']
            if index < len(prefix):
                if meta['fresh']:
                    first.setdefault(sensor, set()).add(identity)
                elif identity in first.get(sensor, ()):
                    stale.add(sensor)
            elif cleared and meta['fresh'] and all(identity[1] > old[1] for old in first[sensor]):
                recovered.add(sensor)
    require(cleared and first.keys() == stale == recovered == expected.keys(),
            'WATER_DEPTH incomplete fresh, stale, clear or rebound evidence')
    return {WATER_DEPTH: {'first_session_fresh': {str(sensor): len(items) for sensor, items in first.items()},
                         'stale': sorted(stale), 'recovered': sorted(recovered)}}
