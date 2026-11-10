"""Offline acceptance for the native observer matrix's forwarded wire evidence."""
import math
import struct


KEYS = {
    'LOCAL_POSITION_NED': 'MAVMOOS_LOCAL_KINEMATICS',
    'VFR_HUD': 'MAVMOOS_VFR_HUD',
    'ESC_TELEMETRY_1_TO_4': 'MAVMOOS_ESC_TELEMETRY',
}
NED = ('x', 'y', 'z', 'vx', 'vy', 'vz')
HUD_WIRE = ('airspeed', 'groundspeed', 'heading', 'throttle', 'alt', 'climb')
HUD_MOOS = ('airspeed_reported_m_s', 'groundspeed_m_s', 'heading_deg',
            'throttle_pct', 'alt_msl_m', 'climb_m_s')
ESC_WIRE = ('voltage', 'current', 'totalcurrent', 'rpm', 'count', 'temperature')
ESC_MOOS = ('voltage_cv', 'current_ca', 'total_current_mah', 'rpm_raw',
            'packet_count', 'temperature_c')


def need(condition, reason):
    if not condition:
        raise ValueError(reason)


def number(value):
    need(type(value) in (int, float) and math.isfinite(value), 'nonfinite observation value')
    return value


def integer(value, lower, upper):
    need(type(value) is int and lower <= value <= upper, 'invalid observation integer')
    return value


def float_wire(value):
    # JSON decimal text may round on decode. Compare at MAVLink float width.
    value = number(value)
    return struct.pack('>f', 0.0 if value == 0 else value)


def sample_value(family, source):
    if family == 'LOCAL_POSITION_NED':
        return (integer(source[0], 0, 2**32 - 1), *(float_wire(value) for value in source[1:]))
    if family == 'VFR_HUD':
        air, ground, heading, throttle, alt, climb = source
        return (float_wire(air), float_wire(ground), integer(heading, -32768, 32767),
                integer(throttle, 0, 65535), float_wire(alt), float_wire(climb))
    return tuple(integer(value, 0, 255 if index % 6 == 5 else 65535)
                 for index, value in enumerate(source))


def wire_value(family, frame):
    if family == 'LOCAL_POSITION_NED':
        return sample_value(family, [frame['time_boot_ms'], *(frame[key] for key in NED)])
    if family == 'VFR_HUD':
        return sample_value(family, [frame[key] for key in HUD_WIRE])
    return sample_value(family, [frame[key][index] for index in range(4) for key in ESC_WIRE])


def app_value(family, value):
    if family == 'LOCAL_POSITION_NED':
        position, velocity = value['position_m'], value['velocity_m_s']
        need(len(position) == len(velocity) == 3, 'local position or velocity axes missing')
        return sample_value(family, [value['time_boot_ms'], *position, *velocity])
    if family == 'VFR_HUD':
        return sample_value(family, [value[key] for key in HUD_MOOS])
    entries = value['entries']
    need(value['message_id'] == 11030 and value['first_index'] == 0 and
         [entry['index'] for entry in entries] == list(range(4)), 'invalid ESC first group')
    return sample_value(family, [entry[key] for entry in entries for key in ESC_MOOS])


def moos_records(family, rows):
    for row in rows:
        if family == 'ESC_TELEMETRY_1_TO_4':
            groups = row['groups']
            need(row['present'] is bool(groups) and len(groups) <= 1 and row['atomic'] is False,
                 'ESC group presence/atomicity differs')
            for group in groups:
                yield group['metadata'], group
        elif row['present']:
            yield row, row['data']


def verify_family(family, rows, frames):
    need(bool(rows), 'missing native MOOS publication for ' + family)
    wire = {wire_value(family, frame) for frame in frames}
    if not wire:
        need(family == 'ESC_TELEMETRY_1_TO_4', 'missing forwarded native ' + family)
        need(not list(moos_records(family, rows)), 'app published ESC with no forwarded native frame')
        return {'result': 'PENDING_PRODUCER', 'forwarded_frames': 0, 'fresh_observations': 0}

    seen, fresh = {}, set()
    for metadata, payload in moos_records(family, rows):
        need(metadata.get('schema') == 1 and metadata.get('present') is True and metadata.get('read_only') is True and
             metadata.get('authoritative_for_control') is False and
             metadata.get('source_age') == 'unknown' and metadata.get('source_quality') == 'unassessed',
             'invalid observation provenance')
        need((metadata['system_id'], metadata['component_id'], metadata['autopilot']) == (1, 1, 3),
             'observation identity differs from the bound Copter')
        effective, read, age = (number(metadata[key]) for key in ('effective_at', 'read_at', 'age_s'))
        need(0 < effective <= read and age >= 0 and age + 1e-6 >= read - effective,
             'invalid original observation time')
        basis = metadata['age_basis']
        need((basis == 'QUEUE_EMPTY_FLOOR' and metadata['age_is_upper_bound'] is True) or
             (basis == 'READ_ACQUISITION' and metadata['age_is_upper_bound'] is False and
              read == effective), 'observation age basis differs')
        timeout = .8 if family == 'LOCAL_POSITION_NED' else 1.0
        need(metadata['sample_timeout_s'] == timeout and metadata['fresh'] is (age <= timeout),
             'freshness differs from the configured timeout and original age')
        identity = (integer(metadata['session_epoch'], 1, 2**64 - 1),
                    integer(metadata['observation_id'], 1, 2**64 - 1))
        value = app_value(family, payload)
        need(value in wire, 'app payload absent from forwarded native ' + family)
        original = effective, read, value
        need(seen.setdefault(identity, original) == original, 'republished observation changed')
        if metadata['fresh']:
            fresh.add(identity)
    need(len(fresh) >= 2, 'missing advancing fresh app observations for ' + family)
    return {'result': 'PASS', 'forwarded_frames': len(frames), 'fresh_observations': len(fresh),
            'distinct_app_observations': len(seen)}


def status_evidence(log, fields):
    grounded = 0
    for line in log.splitlines():
        if not line.startswith('CAPTURE\t'):
            continue
        _, key, value = line.split('\t', 2)
        if key == 'MAVMOOS_STATUS':
            need(fields(value).get('helm') == 'disabled', 'helm is active in native observer case')
        elif key == 'MAVMOOS_RUNTIME_STATUS':
            state = fields(value)
            need(state.get('armed') == '0' and state.get('in_air') == '0' and
                 state.get('control_profile') == 'none' and state.get('configuration_valid') == 'true',
                 'runtime did not remain a disarmed ground observer')
            grounded += state.get('heartbeat_valid') == '1' and state.get('landed_valid') == '1'
    need(grounded > 0, 'runtime never observed live heartbeat and landed state')
    return grounded


def audit_forwarded(rows, samples, finished_at):
    heartbeat = []
    ground = []
    grouped = {name: [] for name in KEYS}
    for row in rows:
        need((row['system'], row['component']) == (1, 1), 'foreign forwarded source')
        family = row['mavpackettype']
        if family == 'HEARTBEAT':
            need(row['type'] == 2 and row['autopilot'] == 3 and not row['base_mode'] & 128,
                 'forwarded Copter heartbeat is foreign or armed')
            heartbeat.append(row)
        elif family == 'EXTENDED_SYS_STATE':
            need(row['landed_state'] == 1, 'forwarded Copter landed state left ground')
            ground.append(row)
        elif family in grouped:
            grouped[family].append(row)
    for name, frames in (('heartbeat', heartbeat), ('ground', ground)):
        need(len(frames) >= 2 and 0 <= finished_at - frames[-1]['mono'] <= 2,
             'insufficient recent forwarded ' + name + ' evidence')
    report = {name: verify_family(name, samples.get(key, []), grouped[name])
              for name, key in KEYS.items()}
    esc_frames = grouped['ESC_TELEMETRY_1_TO_4']
    esc = report['ESC_TELEMETRY_1_TO_4']
    esc['wire_app_delivery_verified'] = esc['result'] == 'PASS'
    esc['simulated_payload_signature_seen'] = any(
        temperature == 32 and voltage in (1679, 1680) and current == 80 and total == 1
        for frame in esc_frames
        for temperature, voltage, current, total in zip(frame['temperature'], frame['voltage'],
                                                        frame['current'], frame['totalcurrent'])
    )
    if not esc['simulated_payload_signature_seen']:
        esc['result'] = 'PENDING_PRODUCER'
    return {'ever_armed': False, 'ground_observations': len(ground), 'heartbeat_observations': len(heartbeat),
            'families': report, 'scope': 'forwarded wire values and original local MOOS age, no source-clock alignment'}
