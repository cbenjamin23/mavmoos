"""Exercise a native QRTL abort under confirmation loss and restart."""

import argparse
import json
import math
from pathlib import Path

from pymavlink.dialects.v20 import ardupilotmega as mav

from adverse import AUTO, LEASE, QLOITER, QRTL, RECOVERY, fields, handback, only_observations
import fault_tap
from flight_fixture import need, now, sha


CASE = 'lost-abort-confirmation'
ACK_CASE = 'lost-abort-ack'
RESTART_CASE = 'restart-unconfirmed-abort'
CASES = (CASE, ACK_CASE, RESTART_CASE)
RESULT = 'MAVMOOS_QUADPLANE_RESULT'
ABORT = 'abort-blind'
TARGET = ',target=flight'


def rejected(flight, id_, action, reason):
    _, row = flight.request(id_, action, 'REJECTED', TARGET, 4)
    need(fields(row)['reason'] == reason, f'{id_} failed for an unexpected reason: {row}')
    return row


def run_loss(flight):
    report, probe = flight.report, flight.probe
    control = flight.out / 'original.tap-mode'
    if report['case'] == RESTART_CASE:
        owned = flight.wait(3, lambda: probe.match(RECOVERY, state='OWNED', marker='present'),
                            'original process did not own the execution marker')
        report['original_session'] = fields(owned)['session']
    fresh = flight.wait(3, lambda: flight.heartbeat(AUTO, True) and
                        now() - flight.heartbeat(AUTO, True)['mono'] < .5 and
                        probe.runtime(fc_mode=str(AUTO), armed='1', heartbeat_valid='1'),
                        'abort requires independently observed fresh armed AUTO')
    report['auto_before_loss'] = fresh
    marker = sha(flight.marker)
    ack_only = report['case'] == ACK_CASE
    report['drop_at'] = fault_tap.set_mode(control, 'drop_mode_ack' if ack_only else 'drop_rx')[1]
    at, report['abort_sent'] = flight.request(ABORT, 'abort', 'SENT', TARGET, 3)
    report['abort_at'] = at
    report['native_qrtl'] = flight.wait(5, lambda: flight.heartbeat(QRTL, True, at),
                                        'controller did not enter armed QRTL while bridge RX was lost')
    if report['case'] == RESTART_CASE:
        run_restart(flight, marker)
        return
    if ack_only:
        report['confirmed'] = flight.wait(5, lambda: probe.match(RESULT, at, id=ABORT, status='CONFIRMED',
                                                                reason='native_qrtl_observed_lease_retained'),
                                           'bridge failed to confirm QRTL from intact mode observations')
        report['initial_same_id'] = rejected(flight, ABORT, 'abort', 'invalid_stale_or_duplicate')
        report['initial_new_id'] = rejected(flight, 'abort-before-slot-expiry', 'abort', 'host_handback_denied')
    else:
        report['unconfirmed'] = flight.wait(7, lambda: probe.match(RESULT, at, id=ABORT, status='UNCONFIRMED'),
                                             'bridge did not report missing confirmation while controller ran QRTL')
        report['blind_same_id'] = rejected(flight, ABORT, 'abort', 'invalid_stale_or_duplicate')
        report['blind_new_id'] = rejected(flight, 'abort-blind-new', 'abort', 'host_handback_denied')

    need(sha(flight.marker) == marker, 'the abort or retry changed the recovery marker')
    report['restore_started'], report['restore_at'] = fault_tap.set_mode(control, 'pass')
    flight.wait(6, lambda: probe.runtime(report['restore_at'], fc_mode=str(QRTL), heartbeat_valid='1',
                                         mission_execution_quarantined='true', permission='false'),
                'fresh QRTL evidence did not return with execution quarantined')
    report['airborne_release'] = rejected(flight, 'early-release-after-abort', 'release', 'fresh_disarmed_qloiter_required')

    # Challenge the fresh-ID gate after the original abort's ten-second slot.
    flight.wait(11, lambda: now() >= at + 10.5, 'original abort slot did not close')
    retry_at = now()
    report['observed_qrtl_retry'] = rejected(flight, 'abort-seen-qrtl', 'abort', 'final_gate_denied')

    touchdown_and_qloiter(flight, marker)
    if now() < retry_at + 10.5:
        flight.wait(11, lambda: now() >= retry_at + 10.5, 'retry slot did not close before release')
    report['ground_bridge_ready'] = flight.wait(5, lambda: probe.runtime(report['pilot_qloiter']['mono'],
                                                                        fc_mode=str(QLOITER), heartbeat_valid='1',
                                                                        armed='0', in_air='0', permission='false'),
                                                 'bridge did not observe disarmed QLoiter before release')
    _, report['release'] = flight.request('observed-handback', 'release', 'COMPLETED', TARGET, 12)
    report['lease_released'] = flight.wait(4, lambda: probe.match(LEASE, held='false'), 'retained lease did not clear')
    finish_after_handback(flight, 'invalid_stale_or_duplicate')


def touchdown_and_qloiter(flight, marker):
    report = flight.report
    home = report['home']

    def touchdown():
        row = flight.observation(after=report['abort_at'], mode=QRTL, armed=False, phase=mav.MAV_VTOL_STATE_MC,
                                 landed=mav.MAV_LANDED_STATE_ON_GROUND)
        if not row:
            return None
        pos = row['position']
        distance = 111194.93e-7 * math.hypot(pos['lat'] - home['latitude'], (pos['lon'] - home['longitude']) *
                                            math.cos(math.radians(home['latitude'] * 1e-7)))
        return row if distance <= 40 and abs(pos['relative_alt']) <= 2500 and math.hypot(pos['vx'], pos['vy']) <= 100 else None

    report['touchdown'] = flight.wait(155, touchdown, 'independent disarmed QRTL touchdown not observed')
    need(sha(flight.marker) == marker, 'recovery marker cleared before observed handback')
    report['pilot_qloiter'] = flight.qloiter()


def finish_after_handback(flight, same_id_reason):
    report = flight.report
    need(not flight.marker.exists(), 'observed handback did not clear the marker')
    report['released_same_id'] = rejected(flight, ABORT, 'abort', same_id_reason)
    report['released_new_id'] = rejected(flight, 'abort-after-handback', 'abort', 'unknown_execution')
    report['quiet_end'] = flight.wait(4, lambda: flight.heartbeat(QLOITER, False, report['released_new_id']['mono'] + 2),
                                      'disarmed QLoiter did not persist through the post-handback quiet window')['mono']


def run_restart(flight, marker):
    report, probe = flight.report, flight.probe
    report['original_exited_at'] = flight.kill_app()
    need(sha(flight.marker) == marker, 'the original process did not leave its execution marker')
    report['restart_at'] = flight.event('restart_app_in_native_qrtl')
    flight.boot_app('restarted')
    report['restart_fence'] = flight.wait(6, lambda: probe.match(RECOVERY, report['restart_at'], state='UNRESOLVED',
                                                               marker='present', effects_blocked='true'),
                                         'restarted process did not retain an unresolved execution fence')
    session = fields(report['restart_fence'])['session']
    need(session != report['original_session'] and len(set(flight.app_pids)) == 2,
         'restart reused the original process or recovery session')
    report['restart_qrtl'] = flight.wait(6, lambda: probe.runtime(report['restart_at'], fc_mode=str(QRTL),
                                                                heartbeat_valid='1', armed='1',
                                                                auto_recovery_blocked='true', permission='false'),
                                        'restarted bridge did not observe armed QRTL while fenced')
    report['restart_same_id'] = rejected(flight, ABORT, 'abort', 'unknown_execution')
    report['restart_new_id'] = rejected(flight, 'abort-after-restart', 'abort', 'unknown_execution')
    report['airborne_handback'] = handback(flight, 'airborne-handback', session, 'REJECTED',
                                          'fresh_prior_non_auto_observation_required')
    touchdown_and_qloiter(flight, marker)
    report['ground_handback_ready'] = flight.wait(8, lambda: probe.match(RECOVERY, state='UNRESOLVED', session=session,
                                                                        ready_for_handback='true', effects_blocked='true'),
                                                  'restarted process never observed fresh disarmed QLoiter')
    report['stale_session_handback'] = handback(flight, 'stale-handback', report['original_session'], 'REJECTED',
                                               'wrong_process_session')
    need(sha(flight.marker) == marker, 'stale-session handback cleared the marker')
    report['release'] = handback(flight, 'current-handback', session, 'RELEASED', 'explicit_handback_no_grant')
    report['ready_no_grant'] = flight.wait(5, lambda: probe.runtime(report['release']['mono'], fc_mode=str(QLOITER),
                                                                   heartbeat_valid='1', armed='0', permission='false',
                                                                   auto_recovery_blocked='false', nav_tx_accepted='0'),
                                         'restart handback did not clear the fence with permission still closed')
    finish_after_handback(flight, 'unknown_execution')


def wire_rows(out, report, instance='original'):
    ports = report['ports']
    records = fault_tap.read_trace(out / report['tap_traces'][instance], ports['bridge'], ports['bridge_peer'])
    for record in records:
        need(record.disposition in ('T', 'R', 'i'), 'loss fixture changed outbound delivery')
        decoder = mav.MAVLink(None)
        frames = decoder.parse_buffer(record.data) or []
        need(frames and sum(len(frame.get_msgbuf()) for frame in frames) == len(record.data),
             'fault tap contains incomplete MAVLink data')
        for frame in frames:
            source = frame.get_srcSystem(), frame.get_srcComponent()
            if record.disposition == 'T':
                need(len(frames) == 1 and source == (245, 191), 'unexpected sender on owned bridge route')
            elif source != (1, 1):
                continue
            yield record.at, record.disposition, frame


def audit_loss(out, report):
    rows = list(wire_rows(out, report))
    ack_only = report['case'] == ACK_CASE
    restart = report['case'] == RESTART_CASE
    drop = report['drop_at']
    end = report['original_exited_at'] if restart else report['restore_started']
    restarted_rows = list(wire_rows(out, report, 'restarted')) if restart else []
    abort = [(at, frame) for at, way, frame in rows + restarted_rows if way == 'T' and frame.get_type() == 'COMMAND_LONG' and
             frame.command == mav.MAV_CMD_DO_SET_MODE and frame.param2 == QRTL]
    need(len(abort) == 1, 'expected exactly one transmitted QRTL command across loss, retries and handback')
    sent, command = abort[0]
    need((command.target_system, command.target_component, command.confirmation, command.param1) == (1, 1, 0, 1),
         'QRTL command did not target the owned FC with the expected mode flag')
    need(drop < report['abort_at'] <= sent < report['native_qrtl']['mono'] < end,
         'fault, wire command and independent effect are out of order')
    replies = [(at, way, frame) for at, way, frame in rows if sent <= at < end and
               frame.get_type() == 'COMMAND_ACK' and frame.command == mav.MAV_CMD_DO_SET_MODE]
    need(len(replies) == 1 and replies[0][1] == 'i' and replies[0][2].result == mav.MAV_RESULT_ACCEPTED and
         (replies[0][2].target_system, replies[0][2].target_component) == (245, 191),
         'missing discarded accepted QRTL acknowledgement addressed to the bridge')
    hearts = [(at, way) for at, way, frame in rows if sent <= at < end and
              frame.get_type() == 'HEARTBEAT' and frame.custom_mode == QRTL]
    if ack_only:
        need([at for at, way, _ in rows if way == 'i'] == [replies[0][0]] and
             any(way == 'R' and at <= report['confirmed']['mono'] for at, way in hearts) and
             sent < report['confirmed']['mono'] < end,
             'selective loss did not drop only the ACK and preserve post-send QRTL evidence')
    else:
        # The process can exit after the separate monitor gets the first QRTL
        # heartbeat and before that same heartbeat reaches the original socket.
        # An unrelated receive already in progress when the tap changes mode
        # can finish before the abort send. Only post-send observations can
        # confirm this abort.
        need((restart or hearts) and all(way == 'i' for _, way in hearts) and
             not any(way == 'R' for at, way, _ in rows if sent <= at < end),
             'QRTL mode evidence reached the original process during receive loss')
    only_observations([row for row in rows if row[0] > sent])
    only_observations(restarted_rows)
    need(any(at > report['quiet_end'] - 1 and way == 'R' for at, way, _ in (restarted_rows if restart else rows)),
         'capture did not span the post-handback observation window')
    with (out / 'monitor-rx.jsonl').open() as stream:
        monitor = [json.loads(line) for line in stream]
    need(report['native_qrtl'] in monitor and all(row in monitor for row in report['touchdown'].values()) and
         report['native_qrtl']['custom_mode'] == QRTL and report['native_qrtl']['base_mode'] & 128 and
         not report['touchdown']['heartbeat']['base_mode'] & 128 and
         report['touchdown']['state']['landed_state'] == mav.MAV_LANDED_STATE_ON_GROUND,
         'independent FC trace did not corroborate armed QRTL and disarmed touchdown')
    with (out / 'fixture-operator.jsonl').open() as stream:
        operator = [json.loads(line) for line in stream]
    need(not any(row.get('command') == mav.MAV_CMD_DO_SET_MODE and row.get('param2') == QRTL for row in operator),
         'the independent operator also requested QRTL')
    with (out / 'moos.jsonl').open() as stream:
        results = [row for line in stream if (row := json.loads(line))['key'] in
                   (RESULT, 'MAVMOOS_AUTO_RECOVERY_RESULT') and row['source'] == 'iMAVLink']
    original = [fields(row)['status'] for row in results if fields(row).get('id') == ABORT and
                report['abort_at'] <= row['mono'] < end]
    need(report['release'] in results and report['abort_sent'] in results, 'missing abort send or release receipt')
    if restart:
        need(original == ['QUEUED', 'SENT'] and end <= report['restart_at'] < report['restart_qrtl']['mono'] and
             restarted_rows and restarted_rows[0][0] >= report['restart_at'],
             'original abort was already terminal or restarted wire evidence is missing')
        rejections = ('restart_same_id', 'restart_new_id', 'airborne_handback', 'stale_session_handback')
    elif ack_only:
        need(original[:3] == ['QUEUED', 'SENT', 'CONFIRMED'] and report['confirmed'] in results and
             'UNCONFIRMED' not in original, 'MAVMOOS did not confirm solely from QRTL evidence')
        need(report['pilot_qloiter']['mono'] <= report['ground_bridge_ready']['mono'] < report['release']['mono'],
             'ground release did not follow fresh bridge QLoiter evidence')
        rejections = ('initial_same_id', 'initial_new_id', 'airborne_release', 'observed_qrtl_retry')
    else:
        need(sent < report['unconfirmed']['mono'] < end and report['unconfirmed'] in results and
             'CONFIRMED' not in original, 'bridge result trace did not preserve the uncertain abort')
        rejections = ('blind_same_id', 'blind_new_id', 'airborne_release', 'observed_qrtl_retry')
    rejections += ('released_same_id', 'released_new_id')
    need(all(report[key] in results and fields(report[key])['status'] == 'REJECTED' for key in rejections),
         'missing retry or early-release rejection in the bridge result trace')
    modes = [row['ModeNum'] for row in report['dataflash']['modes']]
    need(modes.count(QRTL) == 1 and AUTO in modes and modes.index(AUTO) < modes.index(QRTL) and modes[-1] == QLOITER,
         'onboard modes did not show one QRTL entry from AUTO and final QLoiter')
    audit = {'datagram_frames': len(rows) + len(restarted_rows), 'qrtl_transmissions': 1, 'discarded_accepted_ack': True,
             'discarded_qrtl_heartbeats': sum(way == 'i' for _, way in hearts), 'post_abort_effect_frames': 0,
             'onboard_qrtl_entries': 1, 'rejections_verified': len(rejections), 'loss_seconds': end - drop}
    if ack_only:
        audit['qrtl_heartbeats_delivered_before_confirmation'] = sum(
            way == 'R' and at <= report['confirmed']['mono'] for at, way in hearts)
    if restart:
        audit.update(original_abort_results=original, restarted_datagram_frames=len(restarted_rows),
                     restarted_effect_frames=0)
    return audit


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('result', type=Path)
    path = parser.parse_args().result
    report = json.loads(path.read_text())
    need(report['case'] in CASES and report['result'] == 'PASS' and report['cleanup_proven'] and not report['input_drift'],
         'expected a passing, unchanged loss fixture receipt')
    for name, digest in report['files'].items():
        need(sha(path.parent / name) == digest, f'raw case artifact changed: {name}')
    audit = audit_loss(path.parent, report)
    need(audit == report['wire_audit'], 'replayed wire evidence differs from the receipt')
    print(json.dumps(audit, indent=2))
