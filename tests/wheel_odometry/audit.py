"""Expected wheel encoder observation for the grouped generated-frame witness."""
import math

WHEELS = 'MAVMOOS_WHEEL_DISTANCE'


def wheel_matches(data):
    return isinstance(data, dict) and data == {
        'message_id': 9000,
        'wire_version': 2,
        'payload_length': 137,
        'time_usec_raw': '9007199254740999',
        'remote_clock': 'unix_or_boot_unmapped_uint64',
        'wheel_count': 4,
        'wheel_capacity': 16,
        'wheel_order': 'wire_index',
        'distance_m': [12.125, -3.75, None, -0.0],
        'unknown_wheel_indices': [2],
    } and math.copysign(1, data['distance_m'][3]) == -1
