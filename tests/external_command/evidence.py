"""Offline replay of the external DISARM fixture. No process or transport APIs."""
import hashlib
import json
import math
import struct

REQUEST = 'EXAMPLE_DISARM_REQUEST'
RESULT = 'EXAMPLE_DISARM_RESULT'
SOURCE = 'mavmoos_disarm_probe'
POSITIVE = 'schema=1,id=positive,action=disarm,timeout=5'
REQUESTS = [
    ('foreign', 'mavmoos_disarm_foreign', 'schema=1,id=foreign,action=disarm,timeout=5', 0),
    ('invalid', SOURCE, 'schema=1,id=invalid,action=disarm,timeout=5,force=true', 0),
    ('expired', SOURCE, 'schema=1,id=expired,action=disarm,timeout=1', 4),
    ('missing_cancel', SOURCE, 'schema=1,id=missing,action=cancel,target=absent,timeout=1', 0),
    ('positive', SOURCE, POSITIVE, 0),
    ('duplicate', SOURCE, POSITIVE, 0),
]
NEGATIVES = [
    ('', 'wrong_request_source'), ('', 'invalid_request'),
    ('expired', 'request_time_invalid_or_expired'), ('missing', 'cancel_target_not_active'),
    ('positive', 'duplicate_or_exhausted_request_history'),
]
PHASES = ['REQUESTED', 'SUBMITTED', 'SENT', 'ACKNOWLEDGED', 'COMPLETED']
PORTS = {'bridge', 'peer', 'monitor', 'monitor_peer', 'db', 'base', 'rc'}
MODULES = {'arducopter_control', 'example_disarmed_command'}
# The pinned standard/common definitions supply the lengths and CRC extras.
MESSAGE_FIELDS = {
    0: ('HEARTBEAT', '<IBBBBB', 50, ('custom_mode', 'type', 'autopilot', 'base_mode', 'system_status', 'mavlink_version')),
    33: ('GLOBAL_POSITION_INT', '<IiiiihhhH', 104, ('time_boot_ms', 'lat', 'lon', 'alt', 'relative_alt', 'vx', 'vy', 'vz', 'hdg')),
    245: ('EXTENDED_SYS_STATE', '<BB', 130, ('vtol_state', 'landed_state')),
    77: ('COMMAND_ACK', '<HBBiBB', 143, ('command', 'result', 'progress', 'result_param2', 'target_system', 'target_component')),
}


def require(value, message):
    if not value:
        raise ValueError(message)


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def parse(raw):
    def unique(items):
        value = dict(items)
        require(len(items) == len(value), 'duplicate evidence JSON key')
        return value
    return json.loads(raw, object_pairs_hook=unique, parse_constant=lambda _: require(False, 'non-finite evidence JSON'))


def json_lines(raw, *, allow_banner=False):
    require(type(raw) is bytes and 0 < len(raw) < 12_000_000, 'invalid evidence stream size')
    result = []
    for line in raw.splitlines():
        if not line or (allow_banner and not line.startswith(b'{')):
            continue
        value = parse(line)
        require(type(value) is dict, 'evidence stream row must be an object')
        result.append(value)
    require(bool(result), 'empty evidence stream')
    return result


def crc_x25(data):
    crc = 0xffff
    for byte in data:
        low = (byte ^ crc) & 0xff
        low ^= (low << 4) & 0xff
        crc = ((crc >> 8) ^ (low << 8) ^ (low << 3) ^ (low >> 4)) & 0xffff
    return crc


def wire_projection(raw, *, ack=False):
    require(type(raw) is bytes and 0 < len(raw) < 12_000_000, 'invalid MAVLink byte evidence size')
    startup = b'0 0 0 '
    offset = len(startup) if not ack and raw.startswith(startup) else 0
    startup_bytes = offset
    rows = []
    frame_count = 0
    while offset < len(raw):
        magic = raw[offset]
        require(magic in (0xfd, 0xfe), 'invalid native MAVLink prefix')
        header = 10 if magic == 0xfd else 6
        require(offset + header <= len(raw), 'partial native MAVLink header')
        size = raw[offset + 1]
        flags = raw[offset + 2] if magic == 0xfd else 0
        require(flags & ~1 == 0, 'unsupported MAVLink incompatibility flag')
        body = offset + header
        checksum = body + size
        end = checksum + 2 + (13 if flags & 1 else 0)
        require(end <= len(raw), 'partial native MAVLink frame')
        identity = body - (5 if magic == 0xfd else 3)
        system, component = raw[identity:identity + 2]
        require((system, component) == (1, 1), 'foreign native MAVLink source')
        message_id = int.from_bytes(raw[body - (3 if magic == 0xfd else 1):body], 'little')
        frame_count += 1
        if (ack and message_id == 77) or (not ack and message_id in (0, 33, 245)):
            kind, fmt, extra, names = MESSAGE_FIELDS[message_id]
            width = struct.calcsize(fmt)
            require(0 < size <= width and (magic == 0xfd or size == (3 if ack else width)),
                    'unexpected retained MAVLink payload size')
            require(crc_x25(raw[offset + 1:checksum] + bytes([extra])) ==
                    int.from_bytes(raw[checksum:checksum + 2], 'little'), 'retained MAVLink checksum differs')
            values = struct.unpack(fmt, raw[body:checksum].ljust(width, b'\0'))
            rows.append({'kind': kind, 'system': system, 'component': component, **dict(zip(names, values))})
        offset = end
    if ack:
        require(frame_count == 1 and len(rows) == 1, 'expected exactly one COMMAND_ACK frame')
    else:
        require(frame_count > 0 and rows, 'no selected passive MAVLink frames')
    return rows, {'wire_frames': frame_count, 'startup_bytes': startup_bytes, 'bytes': len(raw)}


def catalog_proof(observations, expected_abi):
    require(bool(observations) and all(row == observations[0] for row in observations), 'missing or changing catalog')
    catalog = observations[0]
    require(type(catalog.get('schema')) is int and catalog['schema'] == 1 and
            catalog.get('execution_mode') == 'SITL_CONTROL', 'wrong command execution mode')
    composition = catalog['composition']
    require(type(composition.get('plugin_abi')) is int and composition['plugin_abi'] == expected_abi, 'wrong module ABI')
    modules = {entry['name']: entry for entry in composition['modules']}
    require(len(composition['modules']) == 2 and set(modules) == MODULES, 'wrong loaded module composition')
    backend = modules['arducopter_control']
    example = modules['example_disarmed_command']
    require(backend.get('profile') == 'arducopter' and backend.get('control_provider') is True and
            backend.get('capabilities') == [] and
            set(backend.get('request_owners', [])) == {'MAVMOOS_COMMAND', 'MAVMOOS_NAV_CONTROL',
                                                        'MAVMOOS_CONTROL_REQUEST', 'MAVMOOS_MISSION_CONTROL'} and
            example.get('control_provider') is False and example.get('request_owners') == [REQUEST],
            'disabled Copter facets or example ownership differ')
    require(example.get('capabilities') == [{'name': 'example.disarmed_command.disarm', 'request_variable': REQUEST,
            'request_value': 'schema=1,id=<id>,action=disarm,timeout=<seconds>', 'access': 'request'}],
            'wrong external command capability')
    return catalog


def audit_probe(probe, expected_abi=10):
    events = json_lines(probe, allow_banner=True)
    clocks = [row.get('elapsed') for row in events]
    require(all(finite(at) and at >= 0 for at in clocks) and clocks == sorted(clocks), 'probe clock/order invalid')
    require(events[-1].get('event') == 'done' and sum(row.get('event') == 'done' for row in events) == 1,
            'probe final result missing')
    commands = [(index, row) for index, row in enumerate(events) if row.get('event') == 'request']
    require(len(commands) == len(REQUESTS), 'unexpected request count')
    for (_, row), (phase, source, text, age) in zip(commands, REQUESTS):
        require((row.get('phase'), row.get('source'), row.get('key'), row.get('value')) ==
                (phase, source, REQUEST, text), 'unbound request payload or source')
        sample, published = row.get('moos_sample_at'), row.get('moos_published_at')
        require(finite(sample) and finite(published) and published > 0 and sample - published == age,
                'probe did not preserve original MOOS request time')
    mail = [(i, row) for i, row in enumerate(events) if row.get('event') == 'mail']
    require(all(row.get('source') == 'iMAVLink' for _, row in mail), 'unbound app publication source')
    reports = [(i, parse(row['value'])) for i, row in mail if row.get('key') == RESULT]
    require(len(reports) == 10 and all(type(row.get('schema')) is int and row['schema'] == 1 and
            finite(row.get('observed_at')) for _, row in reports), 'unexpected module result count or schema')
    negatives = [(i, row) for i, row in reports if row.get('status') == 'REJECTED']
    require(len(negatives) == len(NEGATIVES), 'missing explicit rejection')
    for number, ((index, row), (expected_id, reason)) in enumerate(zip(negatives, NEGATIVES)):
        request_number = number if number < 4 else 5
        require(row.get('id') == expected_id and row.get('reason') == reason and
                row.get('remote_effect_uncertain') is False and index > commands[request_number][0] and
                (request_number == 5 or index < commands[request_number + 1][0]), 'wrong request rejection evidence')

    positive = [(i, row) for i, row in reports if row.get('status') != 'REJECTED']
    by_state = {row.get('status'): row for _, row in positive}
    require(len(positive) == 5 and set(by_state) == set(PHASES) and
            all(row.get('id') == 'positive' and row.get('wire_id') == 'example.disarm.positive' and
                commands[4][0] < i < commands[5][0] and row.get('disarmed_ground_fresh') is True
                for i, row in positive), 'missing or wrong positive stage')
    ordered = [by_state[phase] for phase in PHASES]
    requested, _, sent, acknowledged, completed = ordered
    owner_times = [row['observed_at'] for row in ordered]
    require(owner_times == sorted(owner_times), 'module transaction stages have inconsistent owner times')
    published, expires, packet = (requested.get(key) for key in ('published_at', 'operation_expires', 'packet_expires'))
    require(all(finite(value) for value in (published, expires, packet)) and
            published <= owner_times[0] <= published + 2 and expires - published == 5 and
            all(tuple(row.get(key) for key in ('published_at', 'operation_expires', 'packet_expires')) ==
                (published, expires, packet) for row in ordered), 'original operation/packet deadline drift')
    hb, landed = requested.get('heartbeat_at'), requested.get('landed_at')
    require(all(finite(at) and 0 < at <= owner_times[0] < at + 1.5 for at in (hb, landed)) and
            packet == min(expires, hb + 1.5, landed + 1.5), 'original admission observation cutoff differs')
    handoff, ack_read, ack_floor = (row.get(key) for row, key in
        ((sent, 'handoff_at'), (acknowledged, 'ack_read_at'), (acknowledged, 'ack_effective_at')))
    require(all(finite(at) for at in (handoff, ack_read, ack_floor)) and
            owner_times[1] <= handoff <= owner_times[2] < packet <= expires and
            0 < ack_floor <= ack_read <= owner_times[3] < expires and handoff < ack_read and
            owner_times[3] < ack_floor + 1.5 and
            (acknowledged.get('ack_age_basis') == 'QUEUE_EMPTY_FLOOR' or
             acknowledged.get('ack_age_basis') == 'READ_ACQUISITION' and ack_floor == ack_read),
            'handoff/ACK timing or original deadline violated')
    require([row.get('remote_effect_uncertain') for row in ordered] == [False, False, True, True, False] and
            all(row.get('handoff_at') is None for row in ordered[:2]) and
            all(row.get('handoff_at') == handoff for row in ordered[2:]),
            'missing accepted handoff or uncertainty boundary')
    require(owner_times[4] < expires and
            all(finite(completed.get(key)) and handoff < completed[key] <= owner_times[4] < completed[key] + 1.5
                for key in ('heartbeat_at', 'landed_at')) and
            all(completed.get(key) == acknowledged.get(key) for key in
                ('ack_effective_at', 'ack_read_at', 'ack_age_basis', 'ack_frame_hex')),
            'ACK or reused pre-handoff evidence treated as observed result')
    ack_hex = acknowledged.get('ack_frame_hex')
    require(type(ack_hex) is str and 16 <= len(ack_hex) <= 80, 'primary ACK wire receipt missing')
    ack_raw = bytes.fromhex(ack_hex)
    require(ack_raw.hex() == ack_hex, 'noncanonical primary ACK bytes')
    ack = wire_projection(ack_raw, ack=True)[0][0]
    require(ack['command'] == 400 and ack['result'] == 0 and ack['target_system'] in (0, 245) and
            ack['target_component'] in (0, 191), 'primary FC ACK command/result/recipient differs')

    sends = [(i, row['value']) for i, row in mail if row.get('key') == 'MAVMOOS_TX_RESULT']
    require(len(sends) == 1 and commands[4][0] < sends[0][0] < commands[5][0] and
            sends[0][1] == 'id=example.disarm.positive,status=LOCAL_ACCEPTED', 'exactly one native command handoff required')
    status = []
    for i, row in mail:
        if row.get('key') != 'MAVMOOS_RUNTIME_STATUS':
            continue
        pairs = [value.split('=', 1) for value in row['value'].split(',')]
        fields = dict(pairs)
        require(len(pairs) == len(fields) and all(fields.get(key) == value for key, value in
            {'mode': 'SITL_CONTROL', 'control_profile': 'arducopter', 'configuration_valid': 'true',
             'permission': 'false', 'armed': '0', 'in_air': '0', 'nav_tx_accepted': '0',
             'mission_execution_held': 'false', 'mission_mutations': '0'}.items()),
             'runtime control ownership, arm or navigation state changed')
        status.append((i, fields))
    ready = lambda fields: all(fields.get(key) == value for key, value in
                               {'command_leases': '0', 'heartbeat_valid': '1', 'landed_valid': '1'}.items())
    require(any(i < commands[0][0] and ready(fields) for i, fields in status) and
            any(i > commands[5][0] and ready(fields) for i, fields in status), 'missing ready and final empty lease')
    catalog = catalog_proof([parse(row['value']) for _, row in mail if row.get('key') == 'MAVMOOS_CAPABILITIES'], expected_abi)
    return {'status': 'PASS', 'stages': PHASES, 'rejected_requests': [phase for phase, *_ in REQUESTS if phase != 'positive'],
            'local_command_handoffs': 1, 'ack': ack, 'ack_frame_sha256': hashlib.sha256(ack_raw).hexdigest(),
            'original_request_seconds': 5, 'post_handoff_ground': True,
            'runtime_status_rows': len(status), 'catalog': catalog,
            'owner_clock': {'published_at': published, 'operation_expires': expires, 'packet_expires': packet,
                            'handoff_at': handoff, 'ack_read_at': ack_read, 'completed_at': owner_times[4]}}


def at_fixture_home(row):
    # A 0/0 startup projection is not a geographic observation of this bench.
    return abs(row['lat'] - 423584560) <= 200 and abs(row['lon'] + 710875890) <= 250


def audit_fc(fc_bytes, fc_json, chronology):
    rows = json_lines(fc_json)
    projection, summary = wire_projection(fc_bytes)
    require([{key: value for key, value in row.items() if key != 'mono'} for row in rows] == projection,
            'passive FC bytes differ from retained decoded fields/order')
    clocks = [row.get('mono') for row in rows]
    require(all(finite(at) and at >= 0 for at in clocks) and clocks == sorted(clocks), 'passive actual receipt clock/order invalid')
    begin, finish, end = (chronology.get(key) for key in ('probe_started', 'probe_exit_observed', 'capture_finished'))
    require(all(finite(at) for at in (begin, finish, end)) and 0 < begin < finish < end and
            clocks[0] < begin < finish < clocks[-1] <= end, 'independent receive window did not cover the probe')
    before = [row for row in rows if begin - 1.6 <= row['mono'] < begin]
    after = [row for row in rows if finish < row['mono'] <= end]
    window = [row for row in rows if begin - 1.6 <= row['mono'] <= end]
    kinds = {'HEARTBEAT', 'EXTENDED_SYS_STATE', 'GLOBAL_POSITION_INT'}
    require({row['kind'] for row in before} == kinds and {row['kind'] for row in after} == kinds,
            'independent pre/post ground telemetry missing')
    modes = {row['custom_mode'] for row in window if row['kind'] == 'HEARTBEAT'}
    require(len(modes) == 1 and all(row['type'] == 2 and row['autopilot'] == 3 and not row['base_mode'] & 128
            for row in rows if row['kind'] == 'HEARTBEAT'), 'independent passive arm, mode or identity changed')
    require(all(row['landed_state'] == 1 for row in window if row['kind'] == 'EXTENDED_SYS_STATE'),
            'independent passive landed state changed')
    positions = [row for row in window if row['kind'] == 'GLOBAL_POSITION_INT']
    require(len(positions) >= 5 and all(at_fixture_home(row) for row in positions), 'independent passive position unavailable')
    origin = positions[0]
    meters = 111319.49 / 1e7
    distance = max(math.hypot((row['lat'] - origin['lat']) * meters,
            (row['lon'] - origin['lon']) * meters * math.cos(math.radians(origin['lat'] / 1e7)))
            for row in positions)
    speed = max(math.sqrt(row['vx'] ** 2 + row['vy'] ** 2 + row['vz'] ** 2) / 100 for row in positions)
    height = max(abs(row['relative_alt']) / 1000 for row in positions)
    require(distance <= 2 and speed <= .8 and height <= 1, 'independent passive position/speed/height indicates movement')
    return {'status': 'PASS', **summary, 'selected_rows': len(rows), 'before_rows': len(before),
            'after_rows': len(after), 'position_samples': len(positions), 'armed': False, 'on_ground': True,
            'unchanged_mode': modes.pop(), 'max_displacement_m': distance, 'max_speed_m_s': speed,
            'max_relative_height_m': height, 'read_scope': 'actual passive serial1 reads, no primary transmit capture'}


def audit(raw, read, *, expected_abi=10):
    return {'probe': audit_probe(read('probe.jsonl'), expected_abi),
            'passive_fc': audit_fc(read('fc-observer.mavlink'), read('fc-observer.jsonl'), raw['chronology'])}


def replay_archived(raw, read, *, expected_abi=9):
    """Re-run the actual assertions from indexed, closed case bytes."""
    require(raw.get('result') == 'PASS' and raw.get('scenario') == 'disarmed_command' and
            raw.get('profile') == 'arducopter' and raw.get('hardware') is False and
            type(raw.get('probe_exit')) is int and raw['probe_exit'] == 0, 'wrong closed native case scope')
    ports = raw.get('ports_clear', {})
    capture = raw.get('capture', {})
    require(set(ports) == PORTS and all(value is True for value in ports.values()) and
            raw.get('cleanup_proven') is True and raw.get('owned_processes_remaining') == [] and
            capture.get('thread_retired') is True and capture.get('socket_closed') is True and
            capture.get('read_only') is True and 'error' in capture and capture['error'] is None,
            'closed receiver/process/port cleanup unproven')
    require(raw.get('source_ref') and raw.get('source_ref_after') == raw['source_ref'] and
            raw.get('moos_ref') and raw.get('moos_ref_after') == raw['moos_ref'] and raw.get('artifact_drift') == [] and
            raw.get('firmware_identity') == raw.get('firmware_identity_after') and
            raw.get('firmware_identity') and raw.get('rc_input_sent') is False and raw.get('relay_used') is False,
            'closed native source, firmware or effect scope unproven')
    def checked(name):
        data = read(name)
        require(hashlib.sha256(data).hexdigest() == raw.get('files', {}).get(name), 'case file hash differs: ' + name)
        return data
    proof = audit(raw, checked, expected_abi=expected_abi)
    require(proof == raw.get('command_evidence'), 'replayed command evidence differs')
    return proof
