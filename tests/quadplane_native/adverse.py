"""Native takeover/restart cases and readback of the exact direct app route."""

import argparse
import json
from math import cos, hypot, radians
from pathlib import Path
import socket
import sys

from pymavlink.dialects.v20 import ardupilotmega as mav

TESTS = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(TESTS / name) for name in ('plane_native', 'plane_faults')]
import fault_tap
from flight_fixture import Channel, exact_peer, inventory, log, need, now, sha

ADVERSE_CASES = ('takeover', 'restart')
FBWA, AUTO, QLOITER, QRTL = 5, 10, 19, 21
RECOVERY = 'MAVMOOS_AUTO_RECOVERY_STATUS'
RECOVERY_REQUEST = 'MAVMOOS_AUTO_RECOVERY_REQUEST'
RECOVERY_RESULT = 'MAVMOOS_AUTO_RECOVERY_RESULT'
PROOF, LEASE = 'MAVMOOS_STORED_MISSION_PROOF', 'MAVMOOS_MISSION_EXECUTION_LEASE'
# The host takes the fastest interval requested by telemetry, missions, or
# QuadPlane. These are the intervals for this exact three-provider fixture.
OBSERVATIONS = {
    mav.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: (50000, 'GLOBAL_POSITION_INT'),
    mav.MAVLINK_MSG_ID_HOME_POSITION: (500000, 'HOME_POSITION'),
    mav.MAVLINK_MSG_ID_EXTENDED_SYS_STATE: (250000, 'EXTENDED_SYS_STATE'),
}


def fields(row):
    return dict(item.split('=', 1) for item in row['value'].split(',') if '=' in item)


def reset_serial_zero(flight):
    """Remove only the three recovery streams, over the former app's exact socket."""
    need(flight.app is None and flight.marker.is_file(), 'stream reset requires stopped app and retained marker')
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as endpoint:
        endpoint.bind(('127.0.0.1', flight.bridge.port))
        channel = Channel('restart-discovery', endpoint, mav, flight.stack, flight.out)
        flight.channels[channel.name] = channel
        encoder = mav.MAVLink(None, srcSystem=249, srcComponent=190)

        def transmit(command, message_id, interval=0):
            proof = inventory(flight.simulator)
            need(channel.peer == flight.bridge.peer and exact_peer(proof, channel.peer, channel.port),
                 'private discovery peer is not the original exact simulator socket')
            packet = encoder.command_long_encode(1, 1, command, 0, message_id, interval, 0, 0, 0, 0, 0)
            data = packet.pack(encoder)
            encoder.seq = (encoder.seq + 1) % 256
            at = now()
            need(endpoint.sendto(data, channel.peer) == len(data), 'serial0 interval command failed')
            log(flight.actions, {'mono': at, 'role': channel.name, 'peer': channel.peer,
                                 'bytes': data.hex(), **packet.to_dict()})
            return at

        try:
            flight.wait(3, lambda: channel.peer and channel.latest.get('HEARTBEAT'),
                        'same owned serial0 route failed rediscovery')
            readbacks = {}
            for message_id in OBSERVATIONS:
                at = transmit(mav.MAV_CMD_SET_MESSAGE_INTERVAL, message_id, -1)

                def ack():
                    row = channel.latest.get('COMMAND_ACK')
                    return row if row and row['mono'] >= at and row['command'] == mav.MAV_CMD_SET_MESSAGE_INTERVAL and \
                        row['result'] == mav.MAV_RESULT_ACCEPTED and \
                        (row['target_system'], row['target_component']) == (249, 190) else None

                accepted = flight.wait(2, ack, f'no exact disabled interval ACK for {message_id}')
                at = transmit(mav.MAV_CMD_GET_MESSAGE_INTERVAL, message_id)

                def readback():
                    row = channel.latest.get('MESSAGE_INTERVAL')
                    return row if row and row['mono'] >= at and row['message_id'] == message_id and \
                        row['interval_us'] == -1 else None

                readbacks[str(message_id)] = {'set_ack': accepted, 'get_interval': flight.wait(
                    2, readback, f'serial0 interval {message_id} did not read back disabled')}

            quiet = flight.event('serial0_streams_disabled')
            flight.wait(2.5, lambda: now() >= quiet + 1.25 and
                        channel.latest.get('HEARTBEAT', {}).get('mono', 0) > quiet,
                        'stopped stream witness lacked an active native heartbeat')
            need(not any(row['mono'] >= quiet and row['mavpackettype'] in {name for _, name in OBSERVATIONS.values()}
                         for row in channel.messages), 'disabled observations still arrived on serial0')
            flight.report['cleared_serial0_intervals'] = readbacks
            flight.report['serial0_quiet'] = {'start': quiet, 'stop': now(), 'heartbeat': channel.latest['HEARTBEAT']}
        finally:
            del flight.channels[channel.name]


def handback(flight, id_, session, status, reason):
    at = flight.probe.post(RECOVERY_REQUEST, f'schema=1,id={id_},action=handback,session={session}')
    return flight.wait(4, lambda: flight.probe.match(RECOVERY_RESULT, at, id=id_, status=status, reason=reason),
                       f'{id_} did not report {status}: {reason}')


def run_adverse(flight, case, origin, forward_at, suffix):
    report, probe = flight.report, flight.probe
    mark = flight.event('adverse_stimulus', case=case, forward_at=forward_at)
    report['adverse_at'] = mark
    original = flight.wait(3, lambda: probe.match(RECOVERY, state='OWNED', marker='present'),
                           'original current session did not own the live marker')
    report['original_recovery'] = original
    marker_sha = sha(flight.marker)
    old_session = fields(original)['session']

    def touchdown(after):
        row = flight.observation(after=after, mode=QRTL, armed=False, phase=mav.MAV_VTOL_STATE_MC,
                                 landed=mav.MAV_LANDED_STATE_ON_GROUND)
        if not row:
            return None
        p = row['position']
        distance = 111194.93 * hypot(p['lat'] - origin['lat'],
                                   (p['lon'] - origin['lon']) * cos(radians(origin['lat'] * 1e-7))) * 1e-7
        return row | {'distance_m': distance} if distance <= 40 and abs(p['relative_alt']) <= 2500 and \
            hypot(p['vx'], p['vy']) <= 100 else None

    if case == 'takeover':
        pilot_at, report['pilot_fbwa'] = flight.operator_mode(FBWA, True)
        report['pilot_takeover_fence'] = flight.wait(
            4, lambda: probe.runtime(pilot_at, fc_mode=str(FBWA), heartbeat_valid='1', armed='1',
                                     mission_execution_quarantined='true', mission_execution_held='true',
                                     permission='false'), 'fresh pilot takeover did not quarantine bridge authority')
        report['lost_continuation'] = flight.wait(
            3, lambda: probe.match('MAVMOOS_QUADPLANE_RESULT', pilot_at, id='flight',
                                   status='UNCONFIRMED', reason='execution_fenced'),
            'old bridge continuation was not explicitly fenced')
        report['old_abort_rejected'] = flight.request('old-abort', 'abort', 'REJECTED', ',target=flight', 4)[1]
        report['old_plan_rejected'] = flight.request('old-plan', 'run', 'REJECTED', suffix, 4)[1]
        need(fields(report['old_abort_rejected'])['reason'] == 'final_gate_denied' and
             fields(report['old_plan_rejected'])['reason'] == 'control_busy',
             'old operations failed for reasons unrelated to the retained takeover fence')
        report['pilot_fbwa_persists'] = flight.wait(3, lambda: flight.heartbeat(FBWA, True,
                                                                                report['old_plan_rejected']['mono']),
                                                    'old requests changed the pilot-selected mode')
    else:
        report['original_exited_at'] = flight.kill_app()
        reset_serial_zero(flight)
        report['native_continued_without_bridge'] = flight.wait(
            3, lambda: flight.observation(after=report['original_exited_at'], mode=AUTO, armed=True,
                                          phase=mav.MAV_VTOL_STATE_FW, landed=mav.MAV_LANDED_STATE_IN_AIR),
            'native AUTO did not continue in forward flight through app outage')
        report['restart_at'] = flight.event('restart_app')
        flight.boot_app('restarted')
        report['restart_fence'] = flight.wait(
            6, lambda: probe.match(RECOVERY, report['restart_at'], state='UNRESOLVED', marker='present',
                                   reason='restart_uncertainty_marker', effects_blocked='true',
                                   ready_for_handback='false'),
            'restarted app did not surface the persisted uncertainty fence')
        session = fields(report['restart_fence'])['session']
        need(session != old_session and len(set(flight.app_pids)) == 2, 'restart reused process/session identity')
        report['restart_no_proof'] = flight.wait(
            4, lambda: probe.match(PROOF, report['restart_at'], valid='false', reason='fresh_process_session'),
            'restart inherited the old mission proof')
        report['restart_observed_auto'] = flight.wait(
            4, lambda: probe.runtime(report['restart_at'], fc_mode=str(AUTO), heartbeat_valid='1', armed='1',
                                     mission_execution_held='false', permission='false', auto_recovery_blocked='true'),
            'current session did not observe native AUTO under the fence')
        report['old_abort_rejected'] = flight.request('restart-abort', 'abort', 'REJECTED', ',target=flight', 4)[1]
        report['old_plan_rejected'] = flight.request('flight', 'run', 'REJECTED', suffix, 4)[1]
        need(fields(report['old_abort_rejected'])['reason'] == 'unknown_execution' and
             fields(report['old_plan_rejected'])['reason'] == 'verified_plan_and_armed_qloiter_required',
             'restart reused retained authority or accepted the previous mission identity')
        report['airborne_handback_rejected'] = handback(
            flight, 'armed-auto', session, 'REJECTED', 'fresh_prior_non_auto_observation_required')
        report['native_auto_before_pilot_recovery'] = flight.wait(
            3, lambda: flight.heartbeat(AUTO, True, report['airborne_handback_rejected']['mono']),
            'restart or old work changed the independently observed native AUTO mode')

    need(sha(flight.marker) == marker_sha, 'uncertainty marker changed before pilot recovery')
    qrtl_at, report['pilot_qrtl'] = flight.operator_mode(QRTL, True)
    report['touchdown'] = flight.wait(155, lambda: touchdown(qrtl_at),
                                     'pilot QRTL did not end in observed disarmed touchdown within 40 m of HOME')
    need(sha(flight.marker) == marker_sha, 'marker cleared or changed before explicit ground release')
    if case == 'restart':
        report['qrtl_ground_handback_rejected'] = handback(
            flight, 'qrtl-ground', session, 'REJECTED', 'fresh_prior_non_auto_observation_required')
    report['pilot_qloiter'] = flight.qloiter()

    if case == 'takeover':
        _, report['release'] = flight.request('takeover-release', 'release', 'COMPLETED', ',target=flight', 12)
        report['released_lease'] = flight.wait(4, lambda: probe.match(LEASE, qrtl_at, held='false'),
                                                'no retained lease release was observed')
    else:
        report['ground_handback_ready'] = flight.wait(
            8, lambda: probe.match(RECOVERY, qrtl_at, state='UNRESOLVED', ready_for_handback='true',
                                   effects_blocked='true', session=session),
            'restarted process never assembled fresh disarmed QLoiter handback observations')
        report['old_session_rejected'] = handback(flight, 'old-session', old_session, 'REJECTED', 'wrong_process_session')
        need(sha(flight.marker) == marker_sha, 'old session handback cleared the marker')
        report['release'] = handback(flight, 'current-session', session, 'RELEASED', 'explicit_handback_no_grant')
        report['ready_after_handback'] = flight.wait(
            6, lambda: probe.match(RECOVERY, qrtl_at, state='READY', session=session, marker='absent',
                                   effects_blocked='false', reason='explicit_handback'),
            'current-session handback failed to clear the fence')
        report['old_proof_after_handback'] = flight.request('retired-proof', 'run', 'REJECTED', suffix, 4)[1]
        need(fields(report['old_proof_after_handback'])['reason'] == 'verified_plan_and_armed_qloiter_required',
             'released recovery restored the old proof')
        report['disarmed_no_grant'] = flight.wait(
            5, lambda: probe.runtime(report['release']['mono'], fc_mode=str(QLOITER), heartbeat_valid='1',
                                     armed='0', permission='false', mission_execution_held='false',
                                     nav_tx_accepted='0', auto_recovery_blocked='false'),
            'explicit handback conferred unexpected flight permission')
    need(not flight.marker.exists(), 'explicit handback failed to retire the durable marker')


def wire_rows(out, report, instance):
    ports = report['ports']
    records = fault_tap.read_trace(out / report['tap_traces'][instance], ports['bridge'], ports['bridge_peer'])
    for record in records:
        need(record.disposition in ('R', 'T'), 'passive direct-route witness changed a datagram')
        codec = mav.MAVLink(None)
        frames = codec.parse_buffer(record.data) or []
        need(frames and sum(len(frame.get_msgbuf()) for frame in frames) == len(record.data),
             'direct route contains incomplete MAVLink data')
        for frame in frames:
            src = (frame.get_srcSystem(), frame.get_srcComponent())
            if record.disposition == 'T':
                need(len(frames) == 1 and src == (245, 191), 'unexpected sender on the owned app route')
            elif src != (1, 1):
                continue
            yield record.at, record.disposition, frame


def only_observations(rows, ids=None):
    commands = []
    for at, way, message in rows:
        if way != 'T' or message.get_type() == 'HEARTBEAT':
            continue
        need(message.get_type() == 'COMMAND_LONG' and message.command == mav.MAV_CMD_SET_MESSAGE_INTERVAL and
             (message.target_system, message.target_component) == (1, 1) and
             message.confirmation == 0 and all(getattr(message, f'param{p}') == 0 for p in range(3, 8)) and
             (ids is None or message.param1 in ids), 'fenced app sent work outside the observation allowlist')
        commands.append((at, int(message.param1), int(message.param2)))
    return commands


def audit_wire(out, report):
    original = list(wire_rows(out, report, 'original'))
    before = [message.command for at, way, message in original if way == 'T' and
              at < report['adverse_at'] and message.get_type() == 'COMMAND_LONG' and
              message.command != mav.MAV_CMD_SET_MESSAGE_INTERVAL]
    need(before == [mav.MAV_CMD_COMPONENT_ARM_DISARM, mav.MAV_CMD_DO_SET_MISSION_CURRENT,
                    mav.MAV_CMD_MISSION_START], 'initial bridge did not send exactly one arm, cursor and start')
    after = [row for row in original if row[0] >= report['adverse_at']]
    only_observations(after)
    if report['case'] == 'takeover':
        need(any(way == 'T' for _, way, _ in after), 'original bridge trace stopped before adverse stimulus')
    want_modes = iter((QLOITER, AUTO, FBWA, QRTL, QLOITER) if report['case'] == 'takeover'
                      else (QLOITER, AUTO, QRTL, QLOITER))
    want = next(want_modes)
    for mode in report['dataflash']['modes']:
        if want == mode['ModeNum']:
            want = next(want_modes, None)
    need(want is None, 'native DataFlash missed an ordered flight or recovery mode')
    audit = {'original_datagram_frames': len(original), 'initial_commands': before,
             'post_adverse_effect_frames': 0, 'native_mode_order_verified': True}
    if report['case'] == 'restart':
        restarted = list(wire_rows(out, report, 'restarted'))
        fenced = [row for row in restarted if row[0] < report['release']['mono']]
        requests = only_observations(fenced, OBSERVATIONS)
        with (out / 'moos.jsonl').open() as stream:
            moos = [json.loads(line) for line in stream]
        results = [json.loads(row['value']) for row in moos if
                   report['restart_at'] < row['mono'] < report['release']['mono'] and
                   row['key'] == 'MAVMOOS_STREAM_REQUEST_RESULT']
        app_accepted = [(r['message_id'], r['interval_us']) for r in results if r['local_result'] == 'LOCAL_ACCEPTED']
        need(app_accepted == [(id_, interval) for _, id_, interval in requests],
             'restarted app local-acceptance report differs from actual serial0 requests')
        restored = {}
        for id_, (interval, name) in OBSERVATIONS.items():
            sent = next((at for at, msgid, rate in requests if msgid == id_ and rate == interval), None)
            need(sent and all(rate == interval for _, msgid, rate in requests if msgid == id_),
                 f'fenced process did not request only the configured rate for {name}')
            received = next((at for at, way, frame in fenced if way == 'R' and frame.get_msgId() == id_ and at > sent),
                            None)
            need(received and received - sent < 5, f'native serial0 did not restore {name} after its exact request')
            restored[name] = {'message_id': id_, 'interval_us': interval, 'request_mono': sent, 'receive_mono': received}
        only_observations([row for row in restarted if row[0] >= report['release']['mono']])
        audit.update(restarted_datagram_frames=len(restarted), restored_observations=restored,
                     fenced_observation_requests=len(requests), app_readback_matches_wire=True,
                     restarted_effect_frames=0)
    return audit


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Replay the exact owned QuadPlane adverse-case packet capture.')
    parser.add_argument('result', type=Path)
    args = parser.parse_args()
    report = json.loads(args.result.read_text())
    need(report['result'] == 'PASS' and report['cleanup_proven'] and report['input_drift'] == [],
         'case is not an immutable passing native receipt')
    for name, digest in report['files'].items():
        need(sha(args.result.parent / name) == digest, f'raw case artifact changed: {name}')
    audit = audit_wire(args.result.parent, report)
    need(audit == report['wire_audit'], 'raw replay differs from the qualification receipt')
    print(json.dumps(audit, indent=2))
