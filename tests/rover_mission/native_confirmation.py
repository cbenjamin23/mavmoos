"""Verify native Rover stop/RTL after lost confirmation, including explicit retries."""
import math
import time

from confirmation_loss import effectful, frames, mav, need
from rover_mission.confirmation import state_after
from rover_mission.run_native import HELM, NATIVE, distance, position


STATUS = 'MAVMOOS_RUNTIME_STATUS'
ORIGINAL = 'native-loss'
TERMINALS = {'COMPLETED', 'DUPLICATE', 'REJECTED', 'CANCELLED', 'TIMED_OUT', 'FAILED'}
MODES = {'hold': 4, 'loiter': 5, 'rtl': 11}
HORIZONTAL_ESTIMATOR = mav.EKF_ATTITUDE | mav.EKF_VELOCITY_HORIZ | mav.EKF_POS_HORIZ_ABS


def quiet(b, seconds):
    deadline = time.monotonic() + seconds
    b.wait(lambda: time.monotonic() >= deadline and b.state(STATUS).get('permission') == 'false',
           'native mode or pilot handback restored helm permission', seconds + 2)


def settled(b, after, mode, home):
    fix = b.last('GLOBAL_POSITION_INT', after)
    if not fix or not b.mode(mode, after):
        return None
    stop = b.settled(after)
    if not stop:
        return None
    remaining = distance(home, position(fix))
    return {'fix': fix, 'home_distance_m': remaining, **stop} if mode != 11 or remaining <= 2.5 else None


def exercise_native(b, route, restart, fault, action):
    mode = MODES[action]
    command = mav.MAV_CMD_NAV_RETURN_TO_LAUNCH if action == 'rtl' else mav.MAV_CMD_DO_SET_MODE
    result = b.result
    result['action'] = action
    result['native_retry_policy'] = 'durable_manual_fence_v2'
    def ready():
        status = b.state(STATUS)
        flags = int(status.get('estimator_flags', '0'))
        return (all(status.get(key) == '1' for key in ('heartbeat_valid', 'position_valid', 'home_valid',
                                                     'estimator_horizontal_valid')) and
                flags & HORIZONTAL_ESTIMATOR == HORIZONTAL_ESTIMATOR and not flags & mav.EKF_UNINITIALIZED and
                0 <= float(status.get('estimator_age_s', '-1')) <= 3)

    b.wait(lambda: b.mode(4, armed=False) and ready(), 'native Rover estimator or home unavailable', 80)
    home_fix = b.wait(lambda: b.last('HOME_POSITION'), 'independent FC home unavailable')
    home = home_fix['latitude'] * 1e-7, home_fix['longitude'] * 1e-7
    result['independent_home'] = home
    if b.profile == 'boat' and action == 'rtl':
        # Tide moves the neutral hull west while its initial heading faces
        # home. This isolates the RTL confirmation fault from a U-turn.
        b.native_command('setup-arm', 'arm')
        def drifted():
            fix = b.last('GLOBAL_POSITION_INT')
            if not fix or distance(home, position(fix)) < 5 or not b.mode(4):
                return None
            lat, lon = position(fix)
            north, east = home[0] - lat, (home[1] - lon) * math.cos(math.radians(lat))
            heading = math.radians(fix['hdg'] / 100)
            facing_home = (math.cos(heading) * north + math.sin(heading) * east) / math.hypot(north, east)
            return fix if facing_home >= .8 else None

        result['departure'] = b.wait(drifted, 'neutral boat did not drift out facing home', 75)
        quiet(b, .2)
    else:
        b.native_command('setup-guided', 'guided')
        b.native_command('setup-arm', 'arm')
        b.post('MOOS_MANUAL_OVERRIDE', 'false')
        b.post('DEPLOY', 'true')
        acquire = b.post(HELM, 'acquire')
        b.wait(lambda: b.state('MAVMOOS_HELM_STATUS').get('state') == 'ACTIVE' and
               all(b.helm_outputs.get(key, {}).get('at', 0) > acquire for key in ('DESIRED_HEADING', 'DESIRED_SPEED')),
               'stock helm did not acquire motion', 15)

        def departure():
            fix = b.last('GLOBAL_POSITION_INT', acquire)
            if fix and distance(home, position(fix)) >= 5 and math.hypot(fix['vx'], fix['vy']) >= 35:
                return fix
            return None

        result['departure'] = b.wait(departure, 'stock helm did not move Rover away from home', 35)
        released = b.post(HELM, 'release')
        b.wait(lambda: state_after(b, STATUS, released, permission='false'), 'helm release not observed')
        # Leave one window for the previous authority's conditional zero.
        quiet(b, .2)
    route.drop(fault, command, mode if action != 'rtl' else None)
    sent = b.post(NATIVE, f'id={ORIGINAL},action={action}')
    result['initial_mode'] = b.wait(lambda: b.mode(mode, sent), 'native command did not change FC mode', 6)
    if fault == 'restart':
        b.receipt(ORIGINAL, 'SENT', timeout=3)
        result['discarded_ack_before_restart'] = b.wait(route.discarded_ack,
                                                        'accepted ACK had not reached the fault tap before restart', 1.5)
        statuses = [row['status'] for row in b.receipts.get(ORIGINAL, [])]
        # The host publishes its local UDP acceptance alongside the native
        # provider's SENT receipt. Neither confirms the FC accepted the action.
        need([status for status in statuses if status != 'LOCAL_ACCEPTED'] == ['ADMITTED', 'SUBMITTED', 'SENT'] and
             statuses.count('LOCAL_ACCEPTED') == 1, f'original process already confirmed: {statuses}')
        result['original_at_restart'] = statuses
        restart()
    else:
        terminal = b.wait(lambda: next((row for row in b.receipts.get(ORIGINAL, []) if
                                       row['status'] in TERMINALS), None), 'original result missing', 29)
        result['original_terminal'] = terminal
        expected = {('COMPLETED', 'observed_outcome')} if fault == 'ack' else {
            ('TIMED_OUT', 'command_observation_unavailable'), ('CANCELLED', 'connection_reset')}
        need((terminal['status'], terminal['reason']) in expected, f'unexpected original result: {terminal}')
        need(not any(row['status'] == 'ACK_OBSERVED' for row in b.receipts[ORIGINAL]), 'lost ACK was reported')
        route.restore()
    result['initial_outcome'] = b.wait(lambda: settled(b, sent, mode, home),
                                      'independent stop or RTL arrival not observed', 40)
    observed = time.monotonic()
    b.wait(lambda: state_after(b, STATUS, observed, fc_mode=str(mode), heartbeat_valid='1',
                               position_valid='1', permission='false'),
           'bridge did not regain fresh mode and position')
    quiet(b, 1.2)

    def retry(marker, request, expected, reason, *, versioned=False, selected=action):
        before = len(b.receipts.get(request, []))
        route.mark(marker)
        if versioned:
            selected = 'return' if selected == 'rtl' else selected
            b.post('MAVMOOS_CONTROL_REQUEST', f'schema=1,id={request},action={selected},strategy=native')
        else:
            b.post(NATIVE, f'id={request},action={selected}')
        terminal = b.wait(lambda: next((row for row in b.receipts.get(request, [])[before:] if
                                        row['status'] in TERMINALS), None), f'missing terminal retry {marker}', 29)
        records = b.receipts[request][before:]
        outcomes = {(expected, reason)}
        if reason == 'pilot_manual':
            # A Manual heartbeat can retire the queued request before iterate
            # reaches admission. Both outcomes must retire the ID without I/O.
            outcomes.add(('CANCELLED', 'pilot_manual_takeover'))
        need((terminal['status'], terminal['reason']) in outcomes,
             f'unexpected explicit retry outcome: {terminal}')
        need(not any(row['status'] == 'SENT' for row in records), f'{marker} sent a command')
        result.setdefault('retries', {})[marker] = records
        quiet(b, 1.1)

    duplicate_reason = 'native_id_already_recorded' if fault == 'restart' else 'id_already_seen'
    retry('same_id', ORIGINAL, 'DUPLICATE', duplicate_reason, versioned=fault == 'restart')
    retry('fresh_id', 'native-fresh', 'COMPLETED', 'observed_outcome')
    need(any(row['status'] == 'OBSERVING' for row in result['retries']['fresh_id']),
         'fresh ID in the same mode failed to observe without sending')
    route.mark('handback')
    manual_at = time.monotonic()
    pilot = b.out / 'control/takeover.request'
    pilot.touch(exist_ok=False)
    result['pilot_handback'] = b.wait(lambda: b.mode(0, manual_at) and
                                    state_after(b, STATUS, manual_at, fc_mode='0', permission='false'),
                                    'independent pilot Manual handback missing', 8)
    quiet(b, 1.2)
    retry('after_same_id', ORIGINAL, 'DUPLICATE', duplicate_reason)
    retry('after_fresh_id', 'after-handback-fresh', 'REJECTED', 'pilot_manual')
    retry('after_guided', 'after-handback-guided', 'REJECTED', 'pilot_manual', selected='guided')
    result['mode_after_fresh_retry'] = b.last('HEARTBEAT')['custom_mode']
    need(result['mode_after_fresh_retry'] == 0, 'a retry displaced pilot Manual')
    route.mark('pilot_release')
    release = time.monotonic()
    pilot.unlink()
    b.wait(lambda: b.mode(4, release) and state_after(b, STATUS, release, fc_mode='4', permission='false'),
           'pilot did not explicitly select Hold')
    retry('after_pilot_release', ORIGINAL, 'DUPLICATE', duplicate_reason, versioned=True)
    retry('denied_id_after_release', 'after-handback-fresh', 'DUPLICATE', 'id_already_seen', versioned=True)
    route.mark('cleanup')
    b.post('DEPLOY', 'false')
    b.native_command('cleanup-hold', 'hold', timeout=27)
    b.native_command('cleanup-disarm', 'disarm')
    result['final'] = b.wait(lambda: b.mode(4, armed=False), 'independent final disarm missing')


def verify_modes(result):
    mode = MODES[result['action']]
    modes = [row['ModeNum'] for row in result['independent_dataflash']['MODE']]
    departure_mode = 4 if result['profile'] == 'ardurover_boat' and mode == 11 and 15 not in modes else 15
    first = modes.index(departure_mode) + 1
    manual = modes.index(0, first)
    need(modes[first:manual] == [mode], f'expected exactly one pre-handback mode transition: {modes}')
    if result.get('native_retry_policy'):
        need(modes[manual + 1:] == [4], f'pilot authority did not persist to explicit Hold: {modes}')
    elif result['mode_after_fresh_retry'] == mode and mode != 4:
        need(modes[manual + 1:] == [mode, 4], f'post-handback mode trace differs: {modes}')


def audit_native(out, result):
    fault = result['confirmation_fault']
    times = fault['times']
    rows = list(frames(out, result))
    guarded = bool(result.get('native_retry_policy'))
    phases = ('fault', 'same_id', 'fresh_id', 'handback', 'after_same_id', 'after_fresh_id')
    if guarded:
        phases += ('after_guided', 'pilot_release', 'after_pilot_release')
        if result['native_retry_policy']=='durable_manual_fence_v2':
            phases += ('denied_id_after_release',)
    phases += ('cleanup',)
    counts = {}
    sent = None
    for phase, next_phase in zip(phases, phases[1:]):
        packets = [(at, msg) for _, at, side, msg in rows if side == 'T' and
                   times[phase] <= at < times[next_phase] and effectful(msg)]
        counts[phase] = len(packets)
        if guarded or phase in ('fault', 'handback'):
            need(len(packets) == (phase == 'fault'), f'unexpected automatic/handback sends in {phase}: {len(packets)}')
        else:
            handed = sum(row['status'] == 'SENT' for row in result['retries'][phase])
            need(len(packets) == handed <= 1, f'wire and operator retry receipt differ in {phase}')
        for at, msg in packets:
            params = ((1, fault['mode'], 0, 0, 0, 0, 0) if fault['mode'] is not None else (0,) * 7)
            need(msg.get_type() == 'COMMAND_LONG' and msg.command == fault['command'] and
                 (msg.target_system, msg.target_component, msg.confirmation) == (1, 1, 0) and
                 tuple(getattr(msg, f'param{i}') for i in range(1, 8)) == params,
                 f'unexpected effectful wire packet in {phase}: {msg}')
            if phase == 'fault':
                sent = at
    end = times['stopped'] if fault['kind'] == 'restart' else times['restore']
    incoming = [(at, side, msg) for instance, at, side, msg in rows if instance == 'original' and
                sent <= at < end and side != 'T']
    ack = [(at, side, msg) for at, side, msg in incoming if msg.get_type() == 'COMMAND_ACK' and
           msg.command == fault['command'] and (msg.target_system, msg.target_component) == (245, 191)]
    need(len(ack) == 1 and ack[0][1] == 'i' and ack[0][2].result == mav.MAV_RESULT_ACCEPTED,
         'accepted addressed ACK was not discarded exactly once')
    if fault['ack_only']:
        need([(at, msg.get_type()) for _, at, side, msg in rows if side == 'i'] == [(ack[0][0], 'COMMAND_ACK')] and
             any(side == 'R' and msg.get_type() == 'HEARTBEAT' for _, side, msg in incoming),
             'selective loss did not pass independent bridge observations')
    else:
        need(not any(side == 'R' for _, side, _ in incoming), 'original process received post-send observations')
    need(any(at >= times['cleanup'] - 1 and side == 'R' for _, at, side, _ in rows), 'final retry quiet window missing')
    return {'frames': len(rows), 'effectful_sends_by_phase': counts, 'discarded_accepted_ack': True,
            'processes': list(fault['traces'])}
