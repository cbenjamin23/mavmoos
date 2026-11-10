"""Compare fixed-wing airspeed MOOS records with the actual forwarded Plane bytes."""
from observer_extensions_native.audit import framed
from observer_matrix.native_audit import float_wire, integer, need, number

SENSORS = (0, 1)
KEY = 'MAVMOOS_AIRSPEED'
MINIMUM_PER_SENSOR = 8


def match_data(sample, frame, version, length):
    data = sample['data']
    pressure = frame['raw_press']
    temperature = None if frame['temperature'] == 32767 else frame['temperature']
    flags = frame['flags']
    if isinstance(pressure, dict):
        pressure_matches = (pressure == {'nonfinite': 'nan'} and data['raw_pressure_wire'] is None
                            and data['differential_pressure_hpa_if_pinned_ardupilot_pa'] is None)
    else:
        # AirData uses defaultfloat with float::max_digits10 for the JSON record,
        # including this double-precision conversion from the raw float.
        converted = float(format(number(pressure) / 100, '.9g'))
        pressure_matches = (float_wire(data['raw_pressure_wire']) == float_wire(pressure) and
                            number(data['differential_pressure_hpa_if_pinned_ardupilot_pa']) == converted)
    return (sample['sensor_id'] == frame['id'] and sample['wire_version'] == version == 2
            and sample['payload_length'] == length and pressure_matches
            and float_wire(data['airspeed_reported_cas_m_s']) == float_wire(frame['airspeed'])
            and data['temperature_cdeg'] == temperature and data['raw_pressure_mavlink_unit'] == 'hPa'
            and data['flags_raw'] == flags and data['sensor_unhealthy_flag'] is bool(flags & 1)
            and data['sensor_used_flag'] is bool(flags & 2) and data['unknown_flags_raw'] == (flags & ~3))


def audit_airspeed(raw, rows, posts):
    air = {sensor: [] for sensor in SENSORS}
    heartbeat = [row for row in rows if row['mavpackettype'] == 'HEARTBEAT']
    landed = [row for row in rows if row['mavpackettype'] == 'EXTENDED_SYS_STATE']
    need(len(heartbeat) >= 3 and all((row['type'], row['autopilot']) == (1, 3) and not row['base_mode'] & 128
                                     for row in heartbeat), 'independent fixed-wing disarmed state missing')
    need(len(landed) >= 3 and all(row['landed_state'] == 1 for row in landed),
         'independent on-ground state missing')
    for row, version, length, message_id in framed(raw, rows):
        if message_id != 295:
            continue
        need(row['mavpackettype'] == 'AIRSPEED' and (row['system'], row['component']) == (1, 1)
             and row['id'] in air and version == 2 and 1 <= length <= 12,
             'AIRSPEED lost actual ID, source, framing or sensor bounds')
        air[row['id']].append((row, version, length))
    need(all(len(items) >= MINIMUM_PER_SENSOR for items in air.values()),
         'one or more configured native airspeed senders did not advance')

    seen, epochs = {}, set()
    last_match = dict.fromkeys(SENSORS, -1)
    fresh = {sensor: set() for sensor in SENSORS}
    lag = []
    both_present = False
    for summary in posts:
        samples = summary['samples']
        sensors = {sample['sensor_id'] for sample in samples}
        need(summary['schema'] == 1 and summary['family'] == 'AIRSPEED' and summary['message_id'] == 295
             and summary['sensor_limit'] == 2 and summary['invalid_samples'] == 0
             and summary['read_only'] is True and summary['authoritative_for_control'] is False
             and summary['atomic'] is False and summary['present'] is bool(samples)
             and sensors <= set(SENSORS) and len(sensors) == len(samples), 'unexpected AIRSPEED envelope')
        both_present |= sensors == set(SENSORS)
        for sample in samples:
            sensor = sample['sensor_id']
            metadata = sample['metadata']
            need(metadata['schema'] == 1 and metadata['present'] is True and metadata['read_only'] is True
                 and metadata['authoritative_for_control'] is False and metadata['source_age'] == 'unknown'
                 and metadata['source_quality'] == 'unassessed' and
                 (metadata['system_id'], metadata['component_id'], metadata['autopilot']) == (1, 1, 3),
                 'airspeed MOOS record lost bound-source provenance')
            effective, read, age, timeout = (number(metadata[key]) for key in
                                              ('effective_at', 'read_at', 'age_s', 'sample_timeout_s'))
            basis = metadata['age_basis']
            need(0 < effective <= read and age + 1e-6 >= read - effective and age >= 0
                 and metadata['fresh'] is (age <= timeout)
                 and ((basis == 'QUEUE_EMPTY_FLOOR' and metadata['age_is_upper_bound'] is True)
                      or (basis == 'READ_ACQUISITION' and metadata['age_is_upper_bound'] is False
                          and effective == read)), 'airspeed MOOS record lost original receipt age')
            epoch = integer(metadata['session_epoch'], 1, 2**64 - 1)
            observation = integer(metadata['observation_id'], 1, 2**64 - 1)
            identity = sensor, epoch, observation
            epochs.add(epoch)
            original = effective, read, sample['wire_version'], sample['payload_length'], sample['data']
            if identity not in seen:
                candidates = [(index, read - row['mono']) for index, (row, version, length) in enumerate(air[sensor])
                              if index > last_match[sensor] and -.001 <= read - row['mono'] <= .30
                              and match_data(sample, row, version, length)]
                need(candidates, f'sensor {sensor} original record absent from recent forwarded wire')
                last_match[sensor], difference = candidates[-1]
                lag.append(difference)
            need(seen.setdefault(identity, original) == original, 'airspeed republish changed its original sample')
            if metadata['fresh']:
                fresh[sensor].add(identity)
    need(both_present and len(epochs) == 1 and all(len(items) >= MINIMUM_PER_SENSOR for items in fresh.values()),
         'app did not publish both sensors with advancing fresh native observations in one session')
    return {
        'result': 'PASS', 'wire_message_id': 295, 'native_disarmed_heartbeats': len(heartbeat),
        'native_on_ground_messages': len(landed), 'session_epoch': next(iter(epochs)),
        'max_capture_to_app_read_s': max(lag),
        'sensors': {str(sensor): {
            'wire_frames': len(air[sensor]), 'fresh_matched_observations': len(fresh[sensor]),
            'flags_observed': sorted({row['flags'] for row, _, _ in air[sensor]}),
            'wire_payload_lengths': sorted({length for _, _, length in air[sensor]}),
        } for sensor in SENSORS},
    }
