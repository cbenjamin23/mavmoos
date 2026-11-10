#!/usr/bin/env python3
"""Read back a completed or failed Plane fixture directory without opening sockets."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys

from flight_fixture import json_data, need, sha
from run_native import audit_reasons, dataflash


def verify_files(directory, result):
    checked = 0
    for name, claim in result['files'].items():
        path = directory / name
        need(path.resolve().is_relative_to(directory) and path.is_file() and not path.is_symlink(),
             f'invalid archive artifact path {name}')
        need(path.stat().st_size == claim['bytes'] and sha(path) == claim['sha256'],
             f'archive artifact size or SHA mismatch {name}')
        checked += 1
    return checked


def verify_capture(directory, role):
    from pymavlink.dialects.v20 import all as codec

    raw = (directory / f'{role}-rx.mavlink').read_bytes()
    decoder = codec.MAVLink(None)
    decoder.robust_parsing = True
    decoded = Counter()
    for msg in decoder.parse_buffer(raw) or []:
        if (msg.get_srcSystem(), msg.get_srcComponent()) == (1, 1):
            decoded[json.dumps(json_data(msg.to_dict()), sort_keys=True)] += 1
    rows = Counter()
    last = -1
    with (directory / f'{role}-rx.jsonl').open() as stream:
        for line in stream:
            row = json.loads(line)
            at = row.pop('mono')
            offset = row.pop('datagram_offset')
            length = row.pop('datagram_length')
            need(at >= last and 0 <= offset < len(raw) and 0 < length <= len(raw) - offset,
                 'archive telemetry stamp or datagram position invalid')
            last = at
            rows[json.dumps(row, sort_keys=True)] += 1
    need(rows == decoded and bool(rows), f'{role} raw MAVLink does not reproduce the independent rows')
    return sum(rows.values())


def verify_lifecycle(report, frames, monitor, codec):
    workflow = report['workflow']
    modern = report['case'] == 'native-autoland'
    packets = [message for record, message in frames if record.disposition == 'T']
    commands = [message for message in packets if message.get_type() in ('COMMAND_LONG', 'COMMAND_INT')]

    def arguments(command, field):
        return [getattr(message, field) for message in commands if message.command == command]

    cursors = arguments(codec.MAV_CMD_DO_SET_MISSION_CURRENT, 'param1')
    starts = arguments(codec.MAV_CMD_MISSION_START, 'param1')
    modes = arguments(codec.MAV_CMD_DO_SET_MODE, 'param2')
    need(cursors == ([1] if modern else [1, 3]) and starts == ([0] if modern else [0, 0]) and
         modes.count(26) == int(modern) and 11 in modes and 12 in modes and
         not arguments(codec.MAV_CMD_DO_LAND_START, 'param1'), 'runway cursor, start or native mode wire proof differs')
    arm = arguments(codec.MAV_CMD_COMPONENT_ARM_DISARM, 'param1')
    need(arm.count(1) == 1 and not any(arguments(codec.MAV_CMD_COMPONENT_ARM_DISARM, 'param2')),
         'arming transaction did not replay as one non-forced arm')
    counts = Counter(message.get_type() for message in packets)
    need(counts['MISSION_COUNT'] == 1 and counts['MISSION_ITEM_INT'] == 6, 'validated six-row upload did not replay')
    airspeed_targets = arguments(codec.MAV_CMD_GUIDED_CHANGE_SPEED, 'param2')
    need(airspeed_targets == [18, 15], 'wrong adjusted airspeed targets on wire')
    for name in ('independent_armed', 'speed_sensor', 'minimum_sensor', 'independent_disarm', 'touchdown', 'landed'):
        need(workflow[name] in monitor, f'{name} is absent from independent monitor evidence')
    landing_mode = 26 if modern else 10
    mode_history = [row['ModeNum'] for row in report['dataflash']['modes']]
    touchdown = workflow['touchdown']
    ground_speed = (touchdown['vx']**2 + touchdown['vy']**2)**.5 / 100
    need(landing_mode in mode_history and mode_history[-1] == 0 and workflow['landed']['landed_state'] == 1 and
         not workflow['independent_disarm']['base_mode'] & codec.MAV_MODE_FLAG_SAFETY_ARMED and ground_speed <= 1.5,
         'native landing, stop, disarm or Manual handback did not replay')
    climb = workflow['independent_climb']['position']['relative_alt'] / 1000
    need(climb >= 55 and workflow['direct_travel_m'] >= 80 and workflow['rtl_closure_m'] >= 35,
         'measured climb, waypoint or RTL motion is below the declared bound')
    return {'mission_cursors': cursors, 'set_modes': modes, 'airspeed_targets_mps': airspeed_targets,
            'airspeed_observed_mps': [workflow['speed_sensor']['airspeed'], workflow['minimum_sensor']['airspeed']],
            'climb_m': climb, 'direct_travel_m': workflow['direct_travel_m'], 'rtl_closure_m': workflow['rtl_closure_m'],
            'touchdown_speed_mps': ground_speed, 'native_landing_mode': landing_mode, 'dataflash_modes': mode_history}


def verify_operator_nudges(directory, report, frames, monitor, codec):
    from control_workflow import angle, distance, offset, signed_turn

    rows = []
    for line in (directory / 'moos-probe.jsonl').read_text().splitlines():
        if line.startswith('{'):
            rows.append(json.loads(line))
    workflow = report['workflow']

    def request(identifier):
        return next(row['mono'] for row in rows if row['kind'] == 'request' and f'id={identifier},' in row['value'])

    def position(at):
        return next(row for row in reversed(monitor) if row['mavpackettype'] == 'GLOBAL_POSITION_INT' and row['mono'] <= at)

    points = [(record, message) for record, message in frames if record.disposition == 'T' and
              message.get_type() == 'COMMAND_INT' and message.command == codec.MAV_CMD_DO_REPOSITION]
    course_at = request('fixture-course')
    altitude_at = request('fixture-altitude')
    course = next(message for record, message in points if record.at >= course_at)
    altitude = next(message for record, message in points if record.at >= altitude_at)
    start = position(course_at)
    north, east = offset(start['lat'], start['lon'], course.x, course.y)
    turn = signed_turn(angle(start['vx'], start['vy']), angle(north, east))
    travelled = distance(start, position(workflow['course_result']['mono']))
    climbed = (workflow['independent_altitude']['relative_alt'] - position(altitude_at)['relative_alt']) / 1000
    need(20 <= turn <= 70 and travelled >= 135 and climbed >= 8 and
         abs(workflow['independent_altitude']['relative_alt'] / 1000 - altitude.z) <= 4,
         'raw course or altitude trajectory does not establish the relative adjustment')
    need(abs(travelled - workflow['course_travel_m']) <= 2 and abs(climbed - workflow['climb_m']) <= .5 and
         workflow['independent_altitude'] in monitor and workflow['home_marker'] in rows,
         'operator receipt differs from independent source rows')
    marker_at = request('fixture-home')
    acquire = next(row['mono'] for row in rows if row['kind'] == 'request' and
                   row['key'] == 'MAVMOOS_NAV_CONTROL' and row['value'] == 'acquire')
    need(not any(marker_at <= record.at < acquire and message.get_type() in ('COMMAND_LONG', 'COMMAND_INT') and
                 message.command in (codec.MAV_CMD_COMPONENT_ARM_DISARM, codec.MAV_CMD_DO_SET_MODE, codec.MAV_CMD_DO_REPOSITION)
                 for record, message in frames if record.disposition == 'T'), 'home visualization exercised vehicle control')
    return {'relative_course_deg': turn, 'course_travel_m': travelled, 'climb_m': climbed,
            'altitude_target_m': altitude.z, 'home_marker_control_free': True}


def verify_helm_tasks(directory, report, frames, monitor, codec):
    from control_workflow import distance, fields

    workflow = report['workflow']
    raw_mail = [json.loads(line) for line in (directory / 'moos-probe.jsonl').read_text().splitlines() if line.startswith('{')]
    points = {record.at: message for record, message in frames if record.disposition == 'T' and
              message.get_type() == 'COMMAND_INT' and message.command == codec.MAV_CMD_DO_REPOSITION}
    guided = [record.at for record, message in frames if record.disposition == 'T' and
              message.get_type() == 'COMMAND_LONG' and message.command == codec.MAV_CMD_DO_SET_MODE and message.param2 == 15]
    for action in ('return', 'survey', 'voronoi'):
        selected = workflow[action + '_selected']
        acquired = workflow[action + '_acquired_mono']
        active = workflow[action + '_active']
        ready, target = active['ready'], active['target']
        request = next(row for row in raw_mail if row['kind'] == 'request' and
                       f'id=fixture-task-{action},' in row['value'])
        need(selected in raw_mail and ready in raw_mail and active['active'] in raw_mail and
             active['selector'] in raw_mail and target['producer'] in raw_mail and
             ready['source'] == 'pHelmIvP' and ready['value'] == action and
             request['mono'] < ready['mono'] < acquired < target['wire_mono'] and
             not any(request['mono'] <= stamp < acquired for stamp in guided),
             f'{action} selection, stock activeflag and separate acquisition did not replay')
        sent = points.get(target['wire_mono'])
        need(sent and target['target'] in monitor and (sent.x, sent.y) ==
             (target['target']['lat_int'], target['target']['lon_int']), f'{action} lacked paired native target evidence')
    for action in ('survey', 'voronoi'):
        movement = workflow[action + '_motion']
        need(movement['start'] in monitor and movement['stop'] in monitor and
             distance(movement['start'], movement['stop']) >= 45, f'{action} physical motion did not replay')
    expiry = workflow['survey_original_expiry']
    takeover = workflow['voronoi_cancelled']['mono']
    need(workflow['return_closure_m'] >= 35 and fields(workflow['return_cancelled'])['reason'] == 'explicit_task_cancel' and
         fields(workflow['survey_expired'])['reason'] == 'original_task_deadline' and
         not any(expiry <= stamp < workflow['voronoi_acquired_mono'] for stamp in points) and
         workflow['external_takeover'] in monitor and workflow['external_takeover']['custom_mode'] == 5 and
         not any(stamp >= takeover for stamp in points), 'return cancellation, survey expiry or takeover replay differs')
    nudges = workflow['nudges']
    dwell = nudges['post_completion_altitude_dwell']
    interval = dwell[-1]['position']['mono'] - dwell[0]['position']['mono']
    max_error = max(abs(row['position']['relative_alt'] / 1000 - nudges['altitude_target_m']) for row in dwell)
    need(interval >= 1.25 and max_error <= 2 and
         all(row['position'] in monitor and row['target'] in monitor and row['heartbeat'] in monitor and
             row['heartbeat']['custom_mode'] == 12 for row in dwell), 'post-completion altitude dwell did not replay')
    return {'nudges': verify_operator_nudges(directory, dict(report, workflow=nudges), frames, monitor, codec),
            'altitude_dwell_s': interval, 'altitude_dwell_samples': len(dwell), 'maximum_altitude_error_m': max_error,
            'return_closure_m': workflow['return_closure_m'], 'survey_travel_m': workflow['survey_motion']['travel_m'],
            'voronoi_travel_m': workflow['voronoi_motion']['travel_m'], 'takeover_mode': 5}


def verify_control_boundaries(directory, report, records_by_trace):
    from control_workflow import FBWA, GUIDED, LOITER, POINT, RTL, SET_MODE, packet_frames
    from pymavlink.dialects.v20 import all as codec

    case = report['case']
    workflow = report['workflow']
    old_trace = next(iter(records_by_trace.values()))
    frames = list(packet_frames(codec, old_trace))
    monitor = [json.loads(line) for line in (directory / 'monitor-rx.jsonl').read_text().splitlines()]
    if case in ('lifecycle', 'native-autoland'):
        return verify_lifecycle(report, frames, monitor, codec)
    if case == 'operator-nudges':
        return verify_operator_nudges(directory, report, frames, monitor, codec)
    if case == 'helm-tasks':
        return verify_helm_tasks(directory, report, frames, monitor, codec)
    if case == 'helm-mode-stress':
        from helm_mode_stress import audit
        actual = audit(directory, report, old_trace)
        need(actual == report['stress_readback'], 'replayed Plane mode stress evidence differs')
        return actual
    if case == 'takeover':
        held = [(record, msg) for record, msg in frames if record.disposition == 'h']
        delivered = [(record, msg) for record, msg in frames if record.disposition == 'd']
        need(len(held) == len(delivered) == 1 and held[0][0].data == delivered[0][0].data,
             'takeover replay did not recover exactly one matching held/released packet')
        at = delivered[0][0].at
        target = delivered[0][1]
        need(at == workflow['late_target_delivered_mono'] and target.command == POINT,
             'takeover receipt did not identify the actual released point')
        previous = [row for row in monitor if row['mavpackettype'] == 'HEARTBEAT' and row['mono'] <= at]
        need(previous and previous[-1]['custom_mode'] == FBWA, 'delayed packet was not delivered under verified FBWA')
        acks = []
        for record in old_trace:
            if record.disposition != 'R' or not 0 < record.at - at <= 1:
                continue
            decoder = codec.MAVLink(None)
            decoder.robust_parsing = True
            for msg in decoder.parse_buffer(record.data) or []:
                if (msg.get_srcSystem(), msg.get_srcComponent()) == (1, 1) and msg.get_type() == 'COMMAND_ACK' and (
                        msg.command, msg.result, msg.target_system, msg.target_component) == (POINT, 4, 245, 191):
                    acks.append((record, msg))
        need(len(acks) == 1, 'exact delayed target did not yield one targeted FC failure result')
        return {'held_and_delivered_same_bytes': True, 'fc_rejected_in_mode': FBWA,
                'ack_delay_s': acks[0][0].at - at, 'ack': acks[0][1].to_dict()}
    if case.startswith('loss-'):
        rtl = workflow['rtl_mono']
        before = [row for row in monitor if row['mavpackettype'] == 'HEARTBEAT' and row['mono'] < rtl]
        after = [row for row in monitor if row['mavpackettype'] == 'HEARTBEAT' and row['mono'] == rtl]
        previous_mode = LOITER if workflow.get('intermediate_loiter') else GUIDED
        need(before and before[-1]['custom_mode'] == previous_mode and len(after) == 1 and after[0]['custom_mode'] == RTL,
             'watchdog receipt did not bind a fresh independent RTL transition')
        if previous_mode == LOITER:
            inbound_loss = min(r.at for r in old_trace if r.disposition == 'i')
            need(case == 'loss-rx' and any(r.disposition == 'T' and inbound_loss <= r.at < rtl and
                 m.get_type() == 'COMMAND_LONG' and m.command == SET_MODE and m.param2 == LOITER for r, m in frames),
                 'intermediate Loiter lacked the one-way-link handback frame')
        last = max(r.at for r, m in frames if r.disposition == 'T' and r.at < rtl and m.get_type() == 'HEARTBEAT')
        gap = rtl - last
        need(4.7 <= gap <= 6.65 and last == workflow['last_watched_heartbeat_mono'] and gap == workflow['gap_s'],
             'watchdog wire-to-RTL interval did not replay')
        return {'previous_mode': previous_mode, 'result_mode': RTL, 'watchdog_gap_s': gap}
    return None


def replay(directory):
    directory = directory.resolve()
    report = json.loads((directory / 'result.json').read_text())
    summary = {'result_in_receipt': report['result'], 'case': report['case'],
               'files_verified': verify_files(directory, report)}
    if report['result'] != 'PASS':
        summary['failure_preserved'] = report.get('error') or report.get('postflight_error')
        return summary
    need(all(report['ports_clear'].values()) and not report['owned_processes_remaining'] and
         not report.get('owned_mission_processes_remaining') and report['monitor_writes'] == 0,
         'receipt lacks exact cleanup or monitor was not read-only')
    need(report['input_sha256'] == report['input_sha256_after'], 'receipt has different before/after input digests')
    summary['independent_packets'] = {role: verify_capture(directory, role) for role in ('bridge', 'monitor', 'operator')}
    actual = dataflash(directory)
    need(actual == report['dataflash'], 'DataFlash replay differs')
    summary['mode_cause_evidence'] = audit_reasons(report.get('required_mode_reasons', []), actual)
    need(summary['mode_cause_evidence'] == report['mode_cause_evidence'], 'DataFlash mode-cause proof differs')
    if report.get('bridge_traces'):
        from control_workflow import fault_tap, packet_frames
        from pymavlink.dialects.v20 import all as codec

        trace, records_by_trace = {}, {}
        for name in report['bridge_traces']:
            path = directory / name
            records = fault_tap.read_trace(path, report['ports']['bridge'], report['ports']['bridge_native_peer'])
            records_by_trace[name] = records
            trace[name] = {'records': len(records), 'app_frames': sum(1 for _ in packet_frames(codec, records)), 'sha256': sha(path)}
        need(trace == report['wire_trace_replay'], 'application wire replay differs')
        if report['case'] in ('rx-only', 'shadow'):
            need(not any(row['app_frames'] for row in trace.values()), 'passive receipt has application egress')
        summary['app_trace'] = trace
        summary['control_boundaries'] = verify_control_boundaries(directory, report, records_by_trace)
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path, help='private directory from run_native.py')
    args = parser.parse_args()
    try:
        print(json.dumps(replay(args.directory), indent=2))
    except (OSError, RuntimeError, ValueError) as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(1)
