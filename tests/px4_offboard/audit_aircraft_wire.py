#!/usr/bin/env python3
"""Replay the independent GCS wire and check the aircraft qualification receipt."""
import argparse
from bisect import bisect_right
from collections import Counter
import json
import math
import struct
from pathlib import Path


WIRE = {
    0: ('HEARTBEAT', 50, '<IBBB', ('mode', 'mav_type', 'autopilot', 'armed')),
    32: ('LOCAL_POSITION_NED', 185, '<I6f', ('north', 'east', 'down', 'vx', 'vy', 'vz')),
    33: ('GLOBAL_POSITION_INT', 104, '<I4i', ('latitude_e7', 'longitude_e7', 'relative_m')),
    74: ('VFR_HUD', 20, '<f', ('airspeed',)),
    242: ('HOME_POSITION', 104, '<3i', ('latitude_e7', 'longitude_e7', 'altitude_msl_m')),
    245: ('EXTENDED_SYS_STATE', 130, '<BB', ('vtol_state', 'landed_state')),
    436: ('CURRENT_MODE', 193, '<II', ('mode', 'intended_mode')),
}
FIELDS = {name: keys for name, _, _, keys in WIRE.values()}
ULOG_LAYOUTS = {
    'offboard_control_mode': ('uint64_t timestamp;bool position;bool velocity;bool acceleration;'
                              'bool attitude;bool body_rate;bool thrust_and_torque;bool direct_actuator;'
                              'uint8_t[1] _padding0;', '<Q7?'),
    'vehicle_command': ('uint64_t timestamp;double param5;double param6;float param1;float param2;'
                        'float param3;float param4;float param7;uint32_t command;uint16_t source_component;'
                        'uint8_t target_system;uint8_t target_component;uint8_t source_system;'
                        'uint8_t confirmation;bool from_external;uint8_t[1] _padding0;', '<Qdd5fIH5B'),
    'vehicle_command_ack': ('uint64_t timestamp;uint32_t command;int32_t result_param2;uint16_t target_component;'
                            'uint8_t result;uint8_t result_param1;uint8_t target_system;bool from_external;'
                            'uint8_t[2] _padding0;', '<QIiH4B'),
    'vtol_vehicle_status': ('uint64_t timestamp;uint8_t vehicle_vtol_state;bool fixed_wing_system_failure;'
                            'uint8_t[6] _padding0;', '<QB?'),
}


def need(condition, message):
    if not condition:
        raise ValueError(message)


def crc16(data):
    checksum = 0xffff
    for byte in data:
        x = (byte ^ checksum) & 0xff
        x ^= (x << 4) & 0xff
        checksum = ((checksum >> 8) ^ (x << 8) ^ (x << 3) ^ (x >> 4)) & 0xffff
    return checksum


def wire_rows(raw, layouts=WIRE):
    at = 0
    while at < len(raw):
        need(at + 2 <= len(raw), 'short frame prefix')
        magic, length = raw[at:at + 2]
        need(magic in (0xfd, 0xfe), f'invalid frame prefix at {at}')
        body = at + (10 if magic == 0xfd else 6)
        checksum = body + length
        signed = magic == 0xfd and raw[at + 2] & 1
        end = checksum + 2 + (13 if signed else 0)
        need(end <= len(raw), f'truncated frame at {at}')
        if magic == 0xfd and tuple(raw[at + 5:at + 7]) == (1, 1):
            identifier = int.from_bytes(raw[at + 7:body], 'little')
            if identifier in layouts:
                name, crc, layout, _ = layouts[identifier]
                need(crc16(raw[at + 1:checksum] + bytes((crc,))) ==
                     struct.unpack_from('<H', raw, checksum)[0], f'bad {name} CRC at {at}')
                width = struct.calcsize(layout)
                values = struct.unpack(layout, raw[body:checksum][:width].ljust(width, b'\0'))
                if identifier == 0:
                    values = (*values[:3], bool(values[3] & 128))
                elif identifier == 32:
                    values = values[1:]
                elif identifier == 33:
                    values = (*values[1:3], values[4] / 1000)
                elif identifier == 42:
                    values = (*values[:3], values[4])
                elif identifier == 74 and not math.isfinite(values[0]):
                    values = (None,)
                elif identifier == 242:
                    values = (*values[:2], values[2] / 1000)
                if identifier != 32 or all(math.isfinite(value) for value in values):
                    yield name, values
        at = end


def replay(directory, prefix):
    with (directory / (prefix + 'moos-wire.jsonl')).open() as stream:
        rows = [json.loads(line) for line in stream if line.startswith('{')]
    actual = [(row['message'], tuple(row[key] for key in FIELDS[row['message']]))
              for row in rows if row.get('message') in FIELDS]
    expected = list(wire_rows((directory / (prefix + 'gcs.mavlink')).read_bytes()))
    need(actual and actual == expected, f'{prefix}decoded GCS rows differ from ordered CRC-checked frames')
    return rows, len(expected)


def normalized(row):
    return json.dumps(row, sort_keys=True, separators=(',', ':'))


def fields(row):
    return dict(field.split('=', 1) for field in row['value'].split(',') if '=' in field)

def stress_route(result, rows):
    """Check the revised stock route and the complete forward-phase fence."""
    transition = result['transitions'][0]
    fault = transition['observation_loss']
    route = fault['route_change']
    forward = result['legs'][1]
    quiet = fault['restored_quiet']
    expected = dict(zip(('nav_tx_accepted', 'px4_hold_tx_accepted'), map(str, transition['fenced_counts'])))
    expected['permission'] = 'false'
    states = [row for row in rows if row.get('key') == 'MAVMOOS_RUNTIME_STATUS' and
              row.get('source') == 'iMAVLink' and fault['disabled_at'] <= row['mono'] < forward['acquired_at']]
    need(len(states) >= 5 and all(all(fields(row).get(key) == value for key, value in expected.items())
                                 for row in states),
         'the forward-phase fence admitted navigation or prestream traffic')
    records = {normalized(row) for row in rows}
    need(all(normalized(row) in records for row in (
         fault['independent_stable'], fault['bridge_unknown_phase'], route['denied'], route['fenced_runtime'],
         quiet, forward['start'], forward['end'], *route['original'].values(), *route['revised'].values())),
         'revised route or independent phase evidence is absent from the capture')

    def sent(key, value, at):
        return [row for row in rows if row.get('kind') == 'request' and row['key'] == key and
                row['value'] == value and abs(row['mono'] - at) <= .2]

    old, new = (route[stage]['DESIRED_HEADING'] for stage in ('original', 'revised'))
    delta = math.remainder(new['value'] - old['value'], 360)
    need(fault['bridge_unknown_phase']['mono'] == route['phase_unknown_at'] <= route['selected_at'] <=
         old['mono'] <= route['reroute_at'] <= new['mono'] <= route['denied_at'] <= route['denied']['mono'] <=
         route['fenced_runtime']['mono'] <= fault['restore_requested_at'] <= quiet['mono'] < forward['acquired_at'] and
         all(row['source'] == 'pHelmIvP' for stage in ('original', 'revised') for row in route[stage].values()) and
         fields(route['denied'])['reason'] == 'acquisition_rejected' and delta <= -18 and
         len(sent('STRESS_ROUTE_UPDATE', 'points=350,200', route['reroute_at'])) == 1 and
         all(len(sent('MAVMOOS_NAV_CONTROL', 'acquire', at)) == 1
             for at in (route['denied_at'], forward['acquired_at'])),
         'stock reroute, denied acquisition, and post-restoration acquire order differs')
    north = forward['end']['north'] - forward['start']['north']
    east = forward['end']['east'] - forward['start']['east']
    need(north >= 7 and east >= 25, 'independent forward track missed the revised north/east corridor')
    return {'course_delta_deg': delta, 'forward_north_m': north, 'forward_east_m': east,
            'fenced_runtime_samples': len(states)}



def ulog_topics(path):
    topics, formats = {}, set()
    with path.open('rb') as stream:
        need(stream.read(16).startswith(b'ULog\x01\x125'), 'invalid onboard log header')
        while header := stream.read(3):
            length, kind = struct.unpack('<HB', header)
            payload = stream.read(length)
            need(len(payload) == length, 'truncated onboard log record')
            if kind == ord('F'):
                name, layout = payload.decode('ascii').split(':', 1)
                if name in ULOG_LAYOUTS:
                    need(layout == ULOG_LAYOUTS[name][0], f'pinned {name} ULog layout differs')
                    formats.add(name)
            elif kind == ord('A'):
                name = payload[3:].decode('ascii')
                if name in ULOG_LAYOUTS:
                    need(payload[0] == 0 and name in formats, f'unexpected {name} onboard topic instance')
                    topics[struct.unpack_from('<H', payload, 1)[0]] = name
            elif kind == ord('D'):
                name = topics.get(struct.unpack_from('<H', payload)[0])
                if name:
                    yield name, struct.unpack_from(ULOG_LAYOUTS[name][1], payload, 2)


def onboard(directory, vehicle):
    # Arming on the ground creates a short log. The owned flight log contains
    # takeoff, both guidance strategies where applicable, and final disarm.
    path = max(directory.rglob('*.ulg'), key=lambda candidate: candidate.stat().st_size)
    commands, replies, phases, modes = [], [], [], []
    for name, values in ulog_topics(path):
        if name == 'vehicle_command':
            at, _, _, p1, p2, p3, _, _, command, comp, target, target_comp, system, _, external = values
            if external and (system, comp, target, target_comp) == (245, 191, 1, 1) and command != 511:
                commands.append((at, command, p1, p2, p3))
        elif name == 'vehicle_command_ack':
            at, command, _, component, code, _, system, _ = values
            if (system, component) == (245, 191) and command != 511:
                replies.append((at, command, code))
        elif name == 'offboard_control_mode':
            modes.append(values)
        else:
            at, phase, failed = values
            need(not failed, 'VTOL onboard log reported a fixed-wing system failure')
            if not phases or phases[-1][1] != phase:
                phases.append((at, phase))

    vtol = vehicle == 'standard-vtol'
    offboard, hold = (176, 1, 6, 0), (176, 1, 4, 3)
    expected = [(22, 0, 0, 0), offboard, hold]
    if vtol:
        expected.extend(((3000, 4, 0, 0), offboard, hold, (3000, 3, 0, 0)))
    expected.extend((offboard, (20, 0, 0, 0), (21, 0, 0, 0), (400, 0, 0, 0)))
    takeoffs = [index for index, command in enumerate(commands) if command[1] == 22]
    need(len(takeoffs) == 1, 'onboard takeoff count differs')
    commands = commands[takeoffs[0]:]
    need([row[1:] for row in commands] == expected, 'onboard command order or parameters differ')
    for index, (at, command, _, _, _) in enumerate(commands):
        deadline = commands[index + 1][0] if index + 1 < len(commands) else at + 3000000
        codes = [code for reply_at, identifier, code in replies
                 if at <= reply_at < deadline and identifier == command]
        need(codes and set(codes) == {0}, f'onboard addressed command {command} ACK differs')

    need([phase for _, phase in phases] == ([3, 1, 4, 2, 3] if vtol else []),
         'onboard VTOL phase sequence differs')
    phase_times = [at for at, _ in phases]
    counts = Counter()
    for at, position, velocity, *other in modes:
        need(position != velocity and not any(other), 'onboard accepted unexpected Offboard control fields')
        index = bisect_right(phase_times, at) - 1
        phase = phases[index][1] if index >= 0 else 0
        need(not vtol or (phase == 3 or (phase == 4 and position)),
             'onboard accepted stale or transitional VTOL control')
        need(vehicle != 'fixed-wing' or position, 'onboard airplane received velocity control')
        counts[('position' if position else 'velocity') + ('_phase_' + str(phase) if vtol else '')] += 1
    expected_modes = {'position_phase_3', 'position_phase_4', 'velocity_phase_3'} if vtol else {'position'}
    if vehicle == 'quadrotor':
        expected_modes.add('velocity')
    need(set(counts) == expected_modes and min(counts.values()) >= 8, 'onboard guidance strategies differ')
    return {'log': path.relative_to(directory).as_posix(), 'commands_since_takeoff': len(commands),
            'phase_sequence': [phase for _, phase in phases], 'control_samples': dict(counts)}

def check(directory):
    result = json.loads((directory / 'result.json').read_text())
    need(result['result'] == 'PASS', 'qualification did not pass')
    preflight, preflight_frames = replay(directory, 'preflight-')
    flight, flight_frames = replay(directory, '')
    rows = preflight + flight
    records = {normalized(row) for row in rows}

    def require_rows(*required):
        need(all(normalized(row) in records for row in required), 'missing referenced trace record')

    for op in result['native_actions']:
        identity = op['id']
        requests = [(trace, row) for trace in (preflight, flight) for row in trace
                    if row.get('kind') == 'request' and row['key'] == 'MAVMOOS_COMMAND' and
                    fields(row).get('id') == identity]
        need(len(requests) == 1, f'{identity} operator request count differs')
        trace, request = requests[0]
        need(fields(request).get('action') == op['action'] and
             abs(request['mono'] - op['requested_at']) <= .2, f'{identity} operator request differs')
        # A newly connected probe can see a stored copy of the prior result.
        # The original probe is the one that issued and observed this request.
        stages = [row for row in trace if row.get('kind') == 'mail' and row['key'] == 'MAVMOOS_COMMAND_RESULT' and
                  row['source'] == 'iMAVLink' and fields(row).get('id') == identity]
        sends = [row for row in trace if row.get('kind') == 'mail' and row['key'] == 'MAVMOOS_TX_RESULT' and
                 row['source'] == 'iMAVLink' and fields(row).get('id') == identity]
        statuses = [fields(stage)['status'] for stage in stages]
        need(stages == op['stages'] and sends == op['send'] and stages[-1] == op['result'] and
             request['mono'] <= stages[0]['mono'], f'{identity} command trace differs')
        if statuses[-1] == 'REJECTED':
            need(statuses == ['REJECTED'] and not sends, f'{identity} rejection reached the wire')
        else:
            ack_status = 'ACK_OBSERVED' if 'ACK_OBSERVED' in statuses else 'ACKNOWLEDGED'
            need(len(sends) == 1 and fields(sends[0])['status'] == 'LOCAL_ACCEPTED' and
                 statuses[0] == 'ADMITTED' and statuses[-1] == 'COMPLETED' and
                 all(sends[0]['mono'] <= stages[statuses.index(status)]['mono']
                     for status in (ack_status, 'CONFIRMED', 'COMPLETED')), f'{identity} proof order differs')
        if 'ground' in op:
            require_rows(*op['ground'].values())
        if 'hold' in op:
            require_rows(*op['hold'].values())
    require_rows(*result['takeoff'].values(), *result['landed_disarmed'].values())
    for leg in result['legs']:
        require_rows(leg['offboard'], leg['start'], leg['end'], *leg['stock_inputs'].values())
        need(leg['east_m'] == leg['end']['east'] - leg['start']['east'], 'leg distance differs')
    for change in result['transitions']:
        require_rows(change['transient'], change['stable'])
    homeward = result['return_motion']
    require_rows(homeward['mode'])
    for name in ('start', 'end'):
        position, home = homeward[name]['position'], homeward[name]['home']
        require_rows(position, home)
        lat = position['latitude_e7'] * 1e-7
        north = (lat - home['latitude_e7'] * 1e-7) * 111195
        east = (position['longitude_e7'] - home['longitude_e7']) * 1e-7 * 111195 * math.cos(math.radians(lat))
        need(math.isclose(homeward[name]['distance_m'], math.hypot(north, east), abs_tol=1e-8),
             'homeward measurement differs from independent positions')
    stress = {'stress_route': stress_route(result, flight)} if result['transitions'] and (
        'route_change' in result['transitions'][0].get('observation_loss', {})) else {}
    return {'vehicle': result['vehicle'], 'preflight_crc_checked_frames': preflight_frames,
            'flight_crc_checked_frames': flight_frames, 'native_requests': len(result['native_actions']),
            'onboard': onboard(directory, result['vehicle']), **stress}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('native_directory', type=Path)
    print(json.dumps(check(parser.parse_args().native_directory), indent=2))
