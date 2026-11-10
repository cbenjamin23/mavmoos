"""Independent expected data for the shared generated-wire observer fixture."""
REPORT = 'MAVMOOS_TERRAIN_REPORT'
REQUEST = 'MAVMOOS_TERRAIN_REQUEST_OBSERVED'

REPORT_DATA = {
    'latitude_deg': 42.3584567,
    'longitude_deg': -71.0875891,
    'horizontal_datum': 'WGS84',
    'terrain_vertical_datum': 'MSL',
    'spacing_m': 100,
    'terrain_available': True,
    'terrain_height_msl_m': -14.75,
    'terrain_height_reported_msl_m': -14.75,
    'current_height_reported_m': 5.5,
    'current_height_validity': 'unspecified',
    'pending_blocks': 12,
    'loaded_blocks': 4,
}
REQUEST_DATA = {
    'latitude_deg': 42.358,
    'longitude_deg': -71.088,
    'horizontal_datum': 'WGS84',
    'coordinate_role': 'southwest_grid_origin',
    'spacing_m': 100,
    'requested_blocks': 2,
    'grid_mask_hex': '0x80000000000001',
}


def report_matches(data):
    return data == REPORT_DATA


def request_matches(data):
    return data == REQUEST_DATA
