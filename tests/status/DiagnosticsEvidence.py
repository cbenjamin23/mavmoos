"""Validate status diagnostics from the grouped generated and native app witnesses."""

import math
import struct

STATUS = 'MAVMOOS_SYSTEM_STATUS'
FAMILIES = {'power': 'POWER_STATUS', 'ekf': 'EKF_STATUS_REPORT', 'estimator': 'ESTIMATOR_STATUS'}


def need(condition, reason):
    if not condition:
        raise ValueError(reason)


def mcu_samples(row):
    return row['mcu']['instances']


def absent(row):
    return not any(row[family]['present'] for family in FAMILIES) and not row['mcu']['present']


def validate(samples, timeout):
    need(bool(samples), 'missing system-status diagnostics publications')
    seen = {}
    for row in samples:
        epoch = row['session_epoch']
        need(row['schema'] == 1 and row['read_only'] is True and row['authoritative_for_control'] is False,
             'diagnostics aggregate granted authority or changed schema')
        mcu = row['mcu']
        ids = [record['data']['id'] for record in mcu_samples(row)]
        need(mcu['schema'] == 1 and mcu['session_epoch'] == epoch and mcu['present'] is bool(ids) and
             mcu['atomic'] is False and mcu['capacity'] == 8 and len(ids) <= 8 and ids == sorted(set(ids)),
             'MCU sample bounds or identities differ')
        records = [(wire, row[name]) for name, wire in FAMILIES.items()]
        records.extend(('MCU_STATUS', record) for record in mcu_samples(row))
        for wire, sample in records:
            need(sample['schema'] == 1 and sample['session_epoch'] == epoch and sample['family'] == wire and
                 sample['source_age'] == 'unknown' and sample['source_quality'] == 'unassessed',
                 'diagnostic sample provenance differs')
            if not sample['present']:
                need(sample['observation_id'] == 0 and sample['data'] is None and not sample['fresh'] and
                     sample['effective_at'] is None and sample['read_at'] is None and sample['age_s'] is None,
                     'absent diagnostic sample retained a value or time')
                continue
            effective, read, age = (sample[key] for key in ('effective_at', 'read_at', 'age_s'))
            need(row['state'] == 'BOUND' and (row['system_id'], row['component_id']) == (1, 1) and
                 sample['observation_id'] > 0 and all(map(math.isfinite, (effective, read, age))) and
                 0 < effective <= read and age >= 0 and read - effective <= age + 1e-6 and
                 sample['fresh'] is (age <= timeout), 'diagnostic identity, acquisition or age differs')
            basis = sample['age_basis']
            need((basis == 'QUEUE_EMPTY_FLOOR' and sample['age_is_upper_bound'] is True) or
                 (basis == 'READ_ACQUISITION' and sample['age_is_upper_bound'] is False and effective == read),
                 'diagnostic age basis differs')
            identity = epoch, sample['observation_id']
            original = (wire, effective, read, sample['data'])
            need(seen.setdefault(identity, original) == original, 'diagnostic republish changed source sample')
    return seen


def generated_values(wire, data):
    if wire == 'POWER_STATUS':
        return (data['board_rail_mv'] == 5025 and data['board_rail_v']['value'] == 5.025 and
                data['servo_rail_mv'] == 0 and data['servo_rail_v']['value'] is None and
                data['flags']['reported']['servo_valid'] and data['flags']['unknown_mask'] == 0x8000)
    if wire == 'MCU_STATUS':
        if data['id'] == 2:
            return (data['temperature_c']['value'] == -1.25 and data['voltage_v']['value'] == 3.3 and
                    data['reading_within_reported_range'] is True)
        return (data['id'] == 6 and data['voltage_min_v']['value'] is None and
                data['reading_within_reported_range'] is None)
    if wire == 'EKF_STATUS_REPORT':
        flags = data['flags']
        return (flags['unknown_mask'] == 0x1000 and flags['reported']['uninitialized'] and
                flags['reported']['gps_glitch'] and 'accel_error' not in flags['reported'] and
                data['terrain_altitude']['value'] is None and data['airspeed']['value'] == 1.5 and
                data['airspeed_extension_bytes'] is True)
    flags = data['flags']
    return (data['time_usec_raw'] == '9007199254740997' and flags['unknown_mask'] == 0x8000 and
            flags['reported']['gps_glitch'] and flags['reported']['accel_error'] and
            'uninitialized' not in flags['reported'] and data['height_above_ground_ratio']['value'] == 0 and
            data['horizontal_accuracy_m']['value'] == 2.5 and data['vertical_accuracy_m']['value'] == 3.75)


def all_fresh(row):
    return (all(row[key]['fresh'] for key in FAMILIES) and
            [sample['data']['id'] for sample in mcu_samples(row) if sample['fresh']] == [2, 6])


def audit_loopback(prebind, prior, samples):
    before = prebind[STATUS]
    initial = prior[STATUS]
    all_rows = samples[STATUS]
    need(bool(before) and all(absent(row) for row in before), 'diagnostics were present before heartbeat')
    seen = validate(all_rows, .8)
    need(all(generated_values(wire, data) for wire, _effective, _read, data in seen.values()),
         'wrong-source or incorrectly decoded status diagnostics')
    need(any(row['fresh'] and row['data']['load_pct']['value'] == 12.5
             for row in samples['MAVMOOS_SYS_STATUS']), 'existing system-status publication disappeared')
    for key, family in (('MAVMOOS_POWER_STATUS', 'power'), ('MAVMOOS_MCU_STATUS', 'mcu'),
                        ('MAVMOOS_EKF_STATUS_REPORT', 'ekf'), ('MAVMOOS_ESTIMATOR_STATUS', 'estimator')):
        need(bool(samples[key]) and any(sample['present'] and any(sample == row[family] for row in all_rows)
                                       for sample in samples[key]), 'standalone diagnostic publication differs: ' + key)
    start = next((i for i, row in enumerate(initial) if all_fresh(row)), None)
    need(start is not None, 'all generated diagnostic families never became fresh')
    epoch = initial[start]['session_epoch']
    split = next((i for i, row in enumerate(initial) if i > start and row['session_epoch'] == epoch and
                  row['power']['fresh'] and row['ekf']['present'] and not row['ekf']['fresh'] and
                  row['estimator']['present'] and not row['estimator']['fresh'] and len(mcu_samples(row)) == 2 and
                  all(not record['fresh'] for record in mcu_samples(row))), None)
    need(split is not None, 'independent power/MCU/estimator aging missing')
    cleared = [row for row in initial[split:] if row['session_epoch'] > epoch]
    need(bool(cleared) and all(absent(row) for row in cleared), 'diagnostics retained data or rebound without heartbeat')
    recovered = next((row for row in all_rows[len(initial):] if row['session_epoch'] > epoch and all_fresh(row)), None)
    need(recovered is not None, 'diagnostics did not recover after a new heartbeat')
    return {'result': 'PASS', 'families': sorted({entry[0] for entry in seen.values()}),
            'distinct_observations': len(seen), 'initial_epoch': epoch, 'recovered_epoch': recovered['session_epoch'],
            'independent_freshness': True, 'standalone_records_match_aggregate': True}


def f32(value):
    return struct.pack('>f', value) if type(value) in (float, int) and math.isfinite(value) and value >= 0 else None


def audit_native(samples, wire):
    rows = samples[STATUS]
    seen = validate(rows, 3)
    power = [row for row in wire if row['mavpackettype'] == 'POWER_STATUS']
    ekf = [row for row in wire if row['mavpackettype'] == 'EKF_STATUS_REPORT']
    need(len(power) >= 2 and all((row['Vcc'], row['Vservo'], row['flags']) == (5000, 0, 0) for row in power),
         'missing native synthetic board rail or unexpected servo/flag value')
    need(len(ekf) >= 2, 'missing native ArduPilot EKF producer')
    need(not any(row['mavpackettype'] in ('MCU_STATUS', 'ESTIMATOR_STATUS') for row in wire),
         'native fixture emitted an unexpected MCU/common-estimator message')
    metrics = (('velocity', 'velocity_variance'), ('horizontal_position', 'pos_horiz_variance'),
               ('vertical_position', 'pos_vert_variance'), ('compass', 'compass_variance'),
               ('terrain_altitude', 'terrain_alt_variance'), ('airspeed', 'airspeed_variance'))
    ekf_values = {(row['flags'], *(f32(row[key]) if row[key] or name not in ('terrain_altitude', 'airspeed') else None
                                  for name, key in metrics)) for row in ekf}
    delivered = {'POWER_STATUS': 0, 'EKF_STATUS_REPORT': 0}
    for name, _effective, _read, data in seen.values():
        need(name in delivered, 'native app invented an unsupported diagnostic family')
        if name == 'POWER_STATUS':
            need((data['board_rail_mv'], data['board_rail_v']['value'], data['servo_rail_mv'],
                  data['servo_rail_v']['value'], data['flags']['raw']) == (5000, 5, 0, None, 0),
                 'native board/servo observations differ from forwarded wire')
        else:
            need((data['flags']['raw'], *(f32(data[name]['value']) for name, _key in metrics)) in ekf_values,
                 'native EKF observation differs from forwarded wire')
        delivered[name] += 1
    fresh = {FAMILIES[family]: len({(row['session_epoch'], row[family]['observation_id'])
             for row in rows if row[family]['fresh']}) for family in ('power', 'ekf')}
    need(all(count >= 2 for count in fresh.values()) and
         any(row['power']['fresh'] and row['ekf']['fresh'] for row in rows), 'missing fresh diagnostic delivery')
    return {'result': 'PASS', 'power_wire_frames': len(power), 'ekf_wire_frames': len(ekf),
            'distinct_app_observations': delivered, 'fresh_app_observations': fresh,
            'board_rail_source': 'synthetic_sitl_5v',
            'servo_rail': 'unreported', 'mcu': 'no_observed_native_producer', 'common_estimator': 'no_observed_native_producer'}
