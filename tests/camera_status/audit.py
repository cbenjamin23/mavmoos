"""Expected status data for the separate-component generated-wire witness."""
INFORMATION = 'MAVMOOS_CAMERA_INFORMATION'
SETTINGS = 'MAVMOOS_CAMERA_SETTINGS'
CAPTURE = 'MAVMOOS_CAMERA_CAPTURE_STATUS'
STORAGE = 'MAVMOOS_CAMERA_STORAGE'
CAMERA_COMPONENT = 100

INFORMATION_DATA = {
    'message_id': 259, 'payload_length': 236, 'time_boot_ms': 100,
    'remote_clock': 'camera_boot_ms_unmapped_uint32', 'vendor': 'Blue"Lab\n',
    'model': 'abcdefghijklmnopqrstuvwxyz012345', 'firmware_version_raw': 0x04030201,
    'firmware_version': {'major': 1, 'minor': 2, 'patch': 3, 'development': 4},
    'capability_flags_raw': 0x80000003, 'focal_length_mm': 12.5, 'sensor_width_mm': None,
    'sensor_height_mm': 4.25, 'resolution_width_px': 1280, 'resolution_height_px': None,
    'lens_id': None, 'gimbal_device_id': 154, 'definition_version': 7, 'definition_uri_present': True,
}
SETTINGS_DATA = {
    'message_id': 260, 'payload_length': 13, 'time_boot_ms': 200,
    'remote_clock': 'camera_boot_ms_unmapped_uint32', 'mode_raw': 1, 'mode': 'VIDEO',
    'zoom_pct': 0, 'focus_pct': None,
}
CAPTURE_DATA = {
    'message_id': 262, 'payload_length': 19, 'time_boot_ms': 300,
    'remote_clock': 'camera_boot_ms_unmapped_uint32', 'image_status_raw': 3,
    'image_status': 'INTERVAL_CAPTURING', 'video_status_raw': 1, 'video_status': 'RECORDING',
    'image_interval_s': 2.5, 'recording_time_ms': 1234, 'available_capacity_mib': 128,
    'image_count_raw': 42, 'image_count': 42,
}
STORAGE_DATA = {
    'message_id': 261, 'payload_length': 61, 'time_boot_ms': 401,
    'remote_clock': 'camera_boot_ms_unmapped_uint32', 'storage_id': 1, 'storage_count': 2,
    'status_raw': 2, 'status': 'READY', 'type_raw': 3, 'type': 'MICROSD',
    'storage_usage_raw': 3, 'name': 'Card\\\"A', 'total_capacity_mib': 1024, 'used_capacity_mib': 128,
    'available_capacity_mib': 896, 'read_speed_mib_s': 10.5, 'write_speed_mib_s': None,
}


def information_matches(data):
    return data == INFORMATION_DATA


def settings_matches(data):
    return data == SETTINGS_DATA


def capture_matches(data):
    return data == CAPTURE_DATA


def storage_matches(data):
    return (isinstance(data, list) and len(data) == 1 and data[0].get('data') == STORAGE_DATA
            and data[0].get('present') is True and data[0].get('fresh') is True)
