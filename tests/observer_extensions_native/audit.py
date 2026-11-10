"""Compare grouped stable Copter observations to independently retained MAVLink."""
from observer_matrix.native_audit import float_wire, integer, need, number

FAMILIES = {
    'OPTICAL_FLOW': ('MAVMOOS_OPTICAL_FLOW', 100),
    'TERRAIN_REQUEST': ('MAVMOOS_TERRAIN_REQUEST_OBSERVED', 133),
    'TERRAIN_REPORT': ('MAVMOOS_TERRAIN_REPORT', 136),
    'NAV_CONTROLLER_OUTPUT': ('MAVMOOS_NAV_CONTROLLER_OUTPUT', 62),
}
GENERATED = {
    'OPTICAL_FLOW_RAD': 'MAVMOOS_OPTICAL_FLOW_RAD',
    'POSITION_TARGET_LOCAL_NED': 'MAVMOOS_POSITION_TARGET_LOCAL_NED',
    'POSITION_TARGET_GLOBAL_INT': 'MAVMOOS_POSITION_TARGET_GLOBAL_INT',
}
NAV = (
    ('nav_roll_reported_deg', 'nav_roll'),
    ('nav_pitch_reported_deg', 'nav_pitch'),
    ('nav_bearing_reported_deg', 'nav_bearing'),
    ('target_bearing_reported_deg', 'target_bearing'),
    ('waypoint_distance_reported_m', 'wp_dist'),
    ('altitude_error_reported_m', 'alt_error'),
    ('airspeed_error_reported_raw', 'aspd_error'),
    ('cross_track_error_reported_m', 'xtrack_error'),
)


def framed(raw, frames):
    """Attach wire length and version after successful independent byte replay."""
    offset = 0
    for row in frames:
        need(offset + 8 <= len(raw), 'short forwarded MAVLink frame')
        magic, size = raw[offset:offset + 2]
        if magic == 0xfd:
            version = 2
            message_id = int.from_bytes(raw[offset + 7:offset + 10], 'little')
            length = 12 + size + (13 if raw[offset + 2] & 1 else 0)
        else:
            need(magic == 0xfe, 'unexpected MAVLink framing')
            version, message_id, length = 1, raw[offset + 5], 8 + size
        offset += length
        yield row, version, size, message_id
    need(offset == len(raw), 'forwarded byte framing did not consume retained stream')


def float_equal(observed, expected):
    return float_wire(observed) == float_wire(expected)


def coordinate(value):
    scaled = round(number(value) * 1e7)
    need(abs(value - scaled / 1e7) <= 1e-11, 'coordinate did not retain wire precision')
    return scaled


def optional_float(observed, expected):
    return observed is None if expected is None else observed is not None and float_equal(observed, expected)


def matches_flow(metadata, data, frame, version, size):
    distance = frame['ground_distance']
    return (
        metadata['sensor_id'] == frame['sensor_id'] == 0 and metadata['wire_version'] == version and
        metadata['payload_length'] == size and data['time_usec_raw'] == str(frame['time_usec']) and
        data['flow_x_raw'] == frame['flow_x'] and data['flow_y_raw'] == frame['flow_y'] and
        data['quality_raw'] == frame['quality'] == 51 and
        float_equal(data['flow_comp_m_x_raw'], frame['flow_comp_m_x']) and
        float_equal(data['flow_comp_m_y_raw'], frame['flow_comp_m_y']) and
        float_equal(data['ground_distance_m_raw'], distance) and
        optional_float(data['ground_distance_m'], distance if distance > 0 else None) and
        optional_float(data.get('flow_rate_x_rad_s'), frame['flow_rate_x'] if size > 26 else None) and
        optional_float(data.get('flow_rate_y_rad_s'), frame['flow_rate_y'] if size > 30 else None)
    )


def matches_payload(family, envelope, payload, frame, version, size):
    if family == 'OPTICAL_FLOW':
        return matches_flow(envelope, payload, frame, version, size)
    if family == 'NAV_CONTROLLER_OUTPUT':
        return frame['aspd_error'] == 0 and all(float_equal(payload[moos], frame[wire]) for moos, wire in NAV)

    coordinate_matches = (
        coordinate(payload['latitude_deg']) == frame['lat'] and
        coordinate(payload['longitude_deg']) == frame['lon'] and payload['horizontal_datum'] == 'WGS84'
    )
    if family == 'TERRAIN_REQUEST':
        mask = frame['mask']
        return (coordinate_matches and 0 < mask < 2**56 and
                payload['coordinate_role'] == 'southwest_grid_origin' and
                payload['grid_mask_hex'] == f'0x{mask:014x}' and payload['requested_blocks'] == mask.bit_count() and
                payload['spacing_m'] == frame['grid_spacing'] > 0)
    return (
        coordinate_matches and payload['terrain_vertical_datum'] == 'MSL' and
        payload['spacing_m'] == frame['spacing'] == 0 and payload['terrain_available'] is False and
        payload['terrain_height_msl_m'] is None and
        float_equal(payload['terrain_height_reported_msl_m'], frame['terrain_height']) and
        float_equal(payload['current_height_reported_m'], frame['current_height']) and
        payload['current_height_validity'] == 'unspecified' and
        payload['pending_blocks'] == frame['pending'] and payload['loaded_blocks'] == frame['loaded'] == 0
    )


def app_samples(family, rows):
    for row in rows:
        if family == 'OPTICAL_FLOW':
            need(row['family'] == 'OPTICAL_FLOW' and row['message_id'] == 100 and
                 row['axis_basis'] == 'sensor_wire' and row['atomic'] is False and
                 row['present'] is bool(row['samples']) and len(row['samples']) <= 1,
                 'unexpected optical flow family or pinned sensor set')
            for sample in row['samples']:
                yield sample['metadata'], sample, sample['data']
        elif row['present']:
            yield row, row, row['data']


def verify_family(family, rows, wire):
    need(rows and wire, f'missing forwarded native {family} or its MOOS observation')
    seen, fresh = {}, set()
    for stamp, envelope, payload in app_samples(family, rows):
        need(stamp['schema'] == 1 and stamp['present'] is True and stamp['read_only'] is True and
             stamp['authoritative_for_control'] is False and stamp['source_age'] == 'unknown' and
             stamp['source_quality'] == 'unassessed' and
             (stamp['system_id'], stamp['component_id'], stamp['autopilot']) == (1, 1, 3),
             f'{family} lost passive bound-source semantics')
        effective, read, age, timeout = (number(stamp[key]) for key in
                                          ('effective_at', 'read_at', 'age_s', 'sample_timeout_s'))
        need(0 < effective <= read and age >= 0 and age + 1e-6 >= read - effective and
             stamp['fresh'] is (age <= timeout), f'{family} invalid original observation age')
        basis = stamp['age_basis']
        need((basis == 'QUEUE_EMPTY_FLOOR' and stamp['age_is_upper_bound'] is True) or
             (basis == 'READ_ACQUISITION' and stamp['age_is_upper_bound'] is False and read == effective),
             f'{family} mismatched observation age basis')
        epoch = integer(stamp['session_epoch'], 1, 2**64 - 1)
        observation = integer(stamp['observation_id'], 1, 2**64 - 1)
        identity = epoch, observation
        original = effective, read, payload
        if identity not in seen:
            need(any(matches_payload(family, envelope, payload, *frame) for frame in wire),
                 f'{family} payload absent from forwarded independent native wire')
        need(seen.setdefault(identity, original) == original, f'{family} republish altered original record')
        if stamp['fresh']:
            fresh.add(identity)
    need(len(fresh) >= 2, f'{family} did not publish advancing fresh native observations')
    result = {'result': 'PASS', 'wire_frames': len(wire), 'fresh_observations': len(fresh),
              'distinct_observations': len(seen)}
    if family == 'TERRAIN_REPORT':
        need(any(row['present'] and row['data']['pending_blocks'] > 0 for row in rows),
             'missing real no-tiles terrain pending state')
        result['terrain_available'] = False
    return result


def audit_forwarded(frames, raw, samples, capture_finished):
    grouped = {name: [] for name in FAMILIES}
    extra = {name: 0 for name in GENERATED}
    heartbeats, ground = [], []
    for row, version, size, message_id in framed(raw, frames):
        family = row['mavpackettype']
        need((row['system'], row['component']) == (1, 1), 'unbound source in passive forwarded wire')
        if family == 'HEARTBEAT':
            need((row['type'], row['autopilot']) == (2, 3) and not row['base_mode'] & 128,
                 'native observer heartbeat was armed or from a different source')
            heartbeats.append(row)
        elif family == 'EXTENDED_SYS_STATE':
            need(row['landed_state'] == 1, 'native observer left the ground')
            ground.append(row)
        elif family in grouped:
            need(message_id == FAMILIES[family][1], 'MAVLink message ID does not match decoded family')
            grouped[family].append((row, version, size))
        elif family in extra:
            extra[family] += 1
    for family, witnesses in (('heartbeat', heartbeats), ('landed state', ground)):
        need(len(witnesses) >= 2 and 0 <= capture_finished - witnesses[-1]['mono'] <= 2,
             f'insufficient recent native {family} evidence')

    result = {name: verify_family(name, samples.get(key, []), grouped[name])
              for name, (key, _) in FAMILIES.items()}
    generated = {}
    for family, key in GENERATED.items():
        publications = sum(row['present'] for row in samples.get(key, []))
        need(extra[family] or publications == 0, f'{family} was published without independent native wire')
        generated[family] = {'native_wire_frames': extra[family], 'native_present_publications': publications,
                             'qualification': 'generated_fixture'}
    return {'ever_armed': False, 'ground_frames': len(ground), 'heartbeat_frames': len(heartbeats),
            'native_families': result, 'generated_fixture_families': generated}
