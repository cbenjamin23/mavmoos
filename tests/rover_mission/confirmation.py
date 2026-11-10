"""Applied pause/resume under ACK loss, RX loss, or restart in owned Rover SITL."""

import time

from confirmation_loss import mav, need


RESULT = 'MAVMOOS_RUNTIME_STATUS'
RECOVERY = 'MAVMOOS_AUTO_RECOVERY_STATUS'


def state_after(bench, key, after, **expected):
    row = bench.latest.get(key)
    return row and row['at'] > after and all(bench.state(key).get(k) == v for k, v in expected.items())


def retry(bench, request, action, target, status, reason):
    before = len(bench.receipts.get(request, []))
    bench.transition(request, action, target)
    row = bench.wait(lambda: next(iter(bench.receipts.get(request, [])[before:]), None),
                     f'missing {request} retry result', 5)
    need(row['status'] == status and row['reason'] == reason, f'unexpected {request} result: {row}')
    return row


def handback(bench, request, session, status, reason):
    bench.post('MAVMOOS_AUTO_RECOVERY_REQUEST', f'schema=1,id={request},action=handback,session={session}')
    row = bench.receipt(request, status, timeout=5)
    need(row['reason'] == reason, f'unexpected recovery result: {row}')
    return row


def exercise(b, route, restart, fault, action):
    events, target, request = b.events, 'loss-mission', 'loss-transition'
    proof, points = b.upload((24, 48))
    b.native_command('arm', 'arm')
    b.mission_start(target, proof)
    b.movement(points[0], 3)
    if action == 'resume':
        paused_at = b.transition('prepare-pause', 'pause', target)
        b.handback(paused_at, 'prepare-pause')
        b.wait(lambda: b.cursor(paused_at, 1, 2, 0), 'pre-resume stopped cursor absent')
    old_session = b.state(RECOVERY)['session']
    mode = 10 if action == 'resume' else b.stop_mode
    route.drop(fault, mav.MAV_CMD_DO_SET_MODE, mode)
    requested = b.transition(request, action, target)
    events['sent'] = b.receipt(request, 'SENT', timeout=4)
    origin = b.wait(lambda: b.mode(mode, requested) and b.cursor(requested, 1, 3 if action == 'resume' else 2,
                                                               1 if action == 'resume' else 0) and
                    b.last('GLOBAL_POSITION_INT', requested), 'independent native mode and cursor did not change', 5)
    events['native_effect'] = {'mode': b.mode(mode, requested), 'cursor': b.last('MISSION_CURRENT', requested), 'origin': origin}
    if fault == 'restart':
        statuses = [row['status'] for row in b.receipts[request]]
        need(statuses == ['SUBMITTED', 'SENT'], f'unexpected transition results before restart: {statuses}')
        events['original_results'] = statuses
        restarted = time.monotonic()
        restart()
        b.wait(lambda: state_after(b, RECOVERY, restarted, state='UNRESOLVED', marker='present', effects_blocked='true') and
               b.state(RECOVERY).get('session') != old_session, 'new process did not retain recovery fence')
        session = b.state(RECOVERY)['session']
        events['restart_session'] = session
    else:
        expected = 'COMPLETED' if fault == 'ack' else 'CANCELLED'
        events['terminal'] = b.receipt(request, expected, timeout=11)
        route.restore()

    observed_at = time.monotonic()
    b.wait(lambda: state_after(b, RESULT, observed_at, fc_mode=str(mode), heartbeat_valid='1', permission='false'),
           'bridge did not observe actual mode after fault')
    reason = {'restart': 'target_not_retained', 'receive': 'live_mission_and_fresh_cursor_required',
              'ack': 'requested_transition_not_ready'}[fault]
    events['retry_same'] = retry(b, request, action, target,
                                 'REJECTED' if fault == 'restart' else 'DUPLICATE',
                                 reason if fault == 'restart' else 'id_already_seen')
    events['retry_fresh'] = retry(b, 'retry-fresh', action, target, 'REJECTED', reason)
    if action == 'pause':
        events['stopped'] = b.wait(lambda: b.settled(requested) and b.cursor(requested, 1, 2, 0),
                                   'independent pause did not retain stopped cursor')
    else:
        from rover_mission.run_native import position
        events['travel'] = b.movement(position(origin), 1)
        if fault == 'restart':
            events['early_handback'] = handback(b, 'early-handback', session, 'REJECTED',
                                               'fresh_prior_non_auto_observation_required')
        # Pilot first stops the independently moving vehicle, on its separate owned RC route.
        marker = b.pilot_manual()
        at = time.monotonic()
        marker.unlink()
        b.wait(lambda: b.mode(4, at) and b.settled(at) and state_after(b, RESULT, at, fc_mode='4', heartbeat_valid='1'),
               'pilot-selected Hold did not settle')

    route.mark('handback')
    if fault == 'restart':
        b.wait(lambda: b.state(RECOVERY).get('ready_for_handback') == 'true', 'fresh recovery evidence missing')
        events['stale_handback'] = handback(b, 'old-session', old_session, 'REJECTED', 'wrong_process_session')
        events['handback'] = handback(b, 'current-session', session, 'RELEASED', 'explicit_handback_no_grant')
        b.wait(lambda: b.state(RECOVERY).get('state') == 'READY', 'recovery remained unresolved after handback')
    else:
        route.data['handback_stop_mode'] = b.stop_mode
        at = b.transition('explicit-release', 'release', target)
        events['handback'] = b.handback(at, 'explicit-release')

    route.mark('retry_after')
    events['post_handback'] = retry(b, 'post-handback', action, target, 'REJECTED', 'target_not_retained')
    until = time.monotonic() + 1.2
    b.wait(lambda: time.monotonic() >= until and b.state(RESULT).get('permission') == 'false',
           'post-handback quiet window lacked closed helm authority', 3)
    route.mark('cleanup')
    if not b.mode(4):
        b.native_command('final-hold', 'hold')
        b.wait(lambda: b.mode(4), 'native cleanup Hold was not observed')
    b.native_command('final-disarm', 'disarm')
    b.wait(lambda: b.mode(4, armed=False), 'independent final disarm missing')
