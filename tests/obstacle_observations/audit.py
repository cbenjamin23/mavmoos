"""Expected output for the generated OBSTACLE_DISTANCE fixture."""
OBSTACLE = 'MAVMOOS_OBSTACLE_DISTANCE'


def expected_sector(raw, kind, meters=None):
    return {'distance_cm_raw': raw, 'classification': kind, 'distance_m': meters}


def obstacle_matches(data):
    if not isinstance(data, dict):
        return False
    scalars = {
        'time_usec_raw': '9007199254740997',
        'remote_clock': 'unix_or_boot_unmapped_uint64',
        'frame_raw': 12,
        'frame_name': 'BODY_FRD',
        'frame_explicit': True,
        'increment_deg_raw': 5,
        'increment_f_deg_raw': -2.5,
        'increment_deg': -2.5,
        'increment_source': 'increment_f',
        'angle_offset_deg': 12.25,
        'payload_length': 167,
        'sensor_type_raw': 0,
        'sensor_type_name': 'LASER',
        'min_distance_cm': 20,
        'max_distance_cm': 600,
        'no_hit_sentinel_cm': 601,
    }
    if any(data.get(key) != value for key, value in scalars.items()):
        return False
    sectors = [expected_sector(65535, 'unknown') for _ in range(72)]
    sectors[0] = expected_sector(0, 'contact', 0)
    sectors[1] = expected_sector(25, 'in_range', .25)
    sectors[2] = expected_sector(601, 'no_hit')
    sectors[4] = expected_sector(10, 'out_of_range')
    sectors[5] = expected_sector(602, 'out_of_range')
    sectors[6] = expected_sector(600, 'in_range', 6)
    sectors[71] = expected_sector(250, 'in_range', 2.5)
    return data.get('sectors') == sectors
