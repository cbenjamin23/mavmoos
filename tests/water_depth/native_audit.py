"""Independent raw WATER_DEPTH to native boat MOOS comparison."""

from math import isfinite

from observer_extensions_native.audit import framed
from water_depth.audit import MESSAGE_ID, require

SENSORS = {0, 5}
MINIMUM = 4


def numeric(value):
    return value if type(value) in (int, float) and isfinite(value) else None


def projection(frame):
    lat, lon = frame['lat'], frame['lng']
    position = -90e7 <= lat <= 90e7 and -180e7 <= lon <= 180e7 and (lat, lon) != (0, 0)
    health = frame['healthy']
    distance, temp = numeric(frame['distance']), numeric(frame['temperature'])
    available = health == 1 and distance is not None and distance >= 0
    distance_status = ('unavailable_nonfinite' if distance is None else 'invalid_negative' if distance < 0
                       else 'reported_healthy' if available else 'unavailable_health')
    return {
        'time_boot_ms_raw': frame['time_boot_ms'],
        'health_raw': health,
        'health': {0: 'unhealthy', 1: 'healthy'}.get(health, 'unknown'),
        'distance_m_raw': distance, 'bottom_distance_uncorrected_m': distance if available else None,
        'distance_status': distance_status, 'latitude_e7_raw': lat, 'longitude_e7_raw': lon,
        'position_status': 'reported_unconfirmed' if position else 'zero_ambiguous' if (lat, lon) == (0, 0)
        else 'out_of_bounds',
        'latitude_deg': lat / 1e7 if position else None, 'longitude_deg': lon / 1e7 if position else None,
        'vehicle_altitude_msl_m_reported': numeric(frame['alt']), 'vehicle_roll_rad_reported': numeric(frame['roll']),
        'vehicle_pitch_rad_reported': numeric(frame['pitch']), 'vehicle_yaw_rad_reported': numeric(frame['yaw']),
        'temperature_c_raw': temp, 'temperature_c': temp if temp else None,
        'temperature_status': 'unavailable_nonfinite' if temp is None else
        'reported_nonzero' if temp else 'zero_ambiguous',
    }


def audit_native(raw, rows, posts):
    wire = {sensor: [] for sensor in SENSORS}
    heartbeats = sum(row['mavpackettype'] == 'HEARTBEAT' for row in rows)
    require(heartbeats >= 3 and all((row['type'], row['autopilot']) == (11, 3) and not row['base_mode'] & 128
                                   for row in rows if row['mavpackettype'] == 'HEARTBEAT'),
            'native stream omitted continuous disarmed boat identity')
    for row, version, length, ident in framed(raw, rows):
        if ident != MESSAGE_ID:
            continue
        require(row['mavpackettype'] == 'WATER_DEPTH' and version == 2 and 1 <= length <= 38
                and row['id'] in SENSORS and (row['system'], row['component']) == (1, 1)
                and row['healthy'] in (0, 1), 'unexpected native WATER_DEPTH source, sensor or shape')
        wire[row['id']].append((row['mono'], length, projection(row)))
    require(all(len(items) >= MINIMUM for items in wire.values()) and
            any(item['temperature_c'] is not None for _, _, item in wire[0]) and
            all(item['temperature_status'] == 'zero_ambiguous' for _, _, item in wire[5]),
            'native NMEA temperature or I2C missing-temperature evidence absent')

    fresh, seen, epochs = {sensor: set() for sensor in SENSORS}, {}, set()
    previous = dict.fromkeys(SENSORS, -1)
    for post in posts:
        samples = post['samples']
        sensors = [item['sensor_id'] for item in samples]
        require(post['schema'] == 1 and post['message_id'] == MESSAGE_ID and post['family'] == 'WATER_DEPTH'
                and post['sensor_limit'] == 10 and post['invalid_samples'] == 0 and post['read_only'] is True
                and post['atomic'] is False and post['authoritative_for_control'] is False
                and post['present'] is bool(samples) and sensors == sorted(set(sensors))
                and set(sensors) <= SENSORS, 'unexpected native water-depth MOOS envelope')
        for item in samples:
            sensor, meta = item['sensor_id'], item['metadata']
            read, effective, age = meta['read_at'], meta['effective_at'], meta['age_s']
            require(item['wire_version'] == 2 and meta['schema'] == 1 and meta['present'] is True
                    and meta['read_only'] is True and meta['authoritative_for_control'] is False
                    and (meta['system_id'], meta['component_id'], meta['autopilot']) == (1, 1, 3)
                    and meta['source_age'] == 'unknown' and meta['source_quality'] == 'unassessed'
                    and meta['age_basis'] == 'QUEUE_EMPTY_FLOOR' and meta['age_is_upper_bound'] is True
                    and 0 < effective <= read and age + 1e-6 >= read - effective
                    and meta['fresh'] is (age <= meta['sample_timeout_s']),
                    'native water-depth MOOS sample lost acquisition provenance')
            identity = sensor, meta['session_epoch'], meta['observation_id']
            original = effective, read, item['payload_length'], item['data']
            if identity not in seen:
                candidates = [index for index, (at, length, data) in enumerate(wire[sensor])
                              if index > previous[sensor] and -.001 <= read - at <= .30
                              and length == item['payload_length'] and data == item['data']]
                require(candidates, f'native sensor {sensor} not corroborated on recent forwarded wire')
                previous[sensor] = candidates[-1]
            require(seen.setdefault(identity, original) == original,
                    'native water-depth republication changed its original observation')
            if meta['fresh']:
                fresh[sensor].add(identity)
                epochs.add(meta['session_epoch'])
    require(len(epochs) == 1 and all(len(items) >= MINIMUM for items in fresh.values()),
            'native boat did not publish independently advancing fresh NMEA and I2C samples')
    return {'result': 'PASS', 'native_disarmed_heartbeats': heartbeats, 'forward_sensor_excluded': 2,
            'sensors': {str(sensor): {'wire_frames': len(wire[sensor]), 'fresh_matches': len(fresh[sensor])}
                        for sensor in sorted(SENSORS)}}
