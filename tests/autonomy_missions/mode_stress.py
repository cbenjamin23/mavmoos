"""Compound Copter and boat missions on the owned ArduPilot fixtures."""

import math
import time

from confirmation_loss import frames, mav
from rover_mission.confirmation import state_after as matches_state_after
from rover_mission.run_native import HELM, MISSION, NATIVE, fields, require
from stock_grade import checkpoints


COPTER_EXIT = ((-10, 19), (-12, 8), (0, 0))
BOAT_EXIT = ((0, 26), (8, 32), (8, 37))
RUNTIME = 'MAVMOOS_RUNTIME_STATUS'
HELM_STATUS = 'MAVMOOS_HELM_STATUS'
TERMINALS = {'COMPLETED', 'DUPLICATE', 'REJECTED', 'CANCELLED', 'TIMED_OUT', 'FAILED'}
NAVIGATION = {'SET_POSITION_TARGET_GLOBAL_INT', 'SET_POSITION_TARGET_LOCAL_NED'}


def state_after(bench, key, after, **expected):
    return bench.latest[key] if matches_state_after(bench, key, after, **expected) else None


def stock_fields(bench, after, copter):
    names = ('DESIRED_HEADING', 'DESIRED_SPEED', 'DESIRED_ALTITUDE') if copter else (
        'DESIRED_HEADING', 'DESIRED_SPEED')
    observed = {name: bench.helm_outputs.get(name) for name in names}
    return observed if all(row and row['at'] > after for row in observed.values()) else None


def closed_window(bench, seconds, **expected):
    begin = time.monotonic()
    require(bench.runtime(permission='false', **expected), 'authority was already open at the quiet boundary')
    nav = bench.state(RUNTIME)['nav_tx_accepted']

    def quiet():
        require(bench.runtime(permission='false', **expected) and bench.state(RUNTIME)['nav_tx_accepted'] == nav,
                'authority, native mode or navigation counter changed before explicit reacquire')
        return state_after(bench, RUNTIME, begin + seconds, permission='false', **expected)

    return bench.wait(quiet, 'no fresh runtime witness at the end of the quiet window', seconds + 3)


def deny_acquire(bench):
    at = bench.post(HELM, 'acquire')
    return bench.wait(lambda: state_after(bench, HELM_STATUS, at, reason='acquisition_rejected') and
                      state_after(bench, RUNTIME, at, permission='false'),
                      'competing helm acquire did not produce a fresh denial', 5)


def duplicate_receipt(bench, identity, request):
    seen = len(bench.receipts[identity])
    request()
    retry = bench.wait(lambda: next(iter(bench.receipts[identity][seen:]), None),
                       f'{identity} did not receive a retry receipt', 6)
    require(retry['status'] == 'DUPLICATE', f'{identity} reused the original native request ID')
    return retry


def route_update(bench, route):
    return bench.post('SURVEY_UPDATE', 'xpoints=' + ':'.join(f'{x},{y}' for x, y in route))


def reacquire(bench, tap, copter):
    # A normal producer resynchronization is separate from flight authority.
    off = bench.post('DEPLOY', 'false')
    bench.wait(lambda: state_after(bench, HELM_STATUS, off) and
               bench.state(HELM_STATUS).get('allstop') in ('NothingToDo', 'NoGoalBehavior'),
               'stock helm did not declare all-stop during resynchronization', 6)
    on = bench.post('DEPLOY', 'true')
    prepared = bench.wait(lambda: stock_fields(bench, on, copter) and
                          state_after(bench, HELM_STATUS, on, acquire_preparation='READY'),
                          'stock helm did not prepare a fresh tuple from the revised route', 8)
    closed_window(bench, 1.1)
    tap.mark('reacquire')
    at = bench.post(HELM, 'acquire')
    active = bench.wait(lambda: bench.mode(int(bench.guided_mode), at) and
                        state_after(bench, HELM_STATUS, at, state='ACTIVE') and
                        stock_fields(bench, at, copter) and
                        state_after(bench, RUNTIME, at, permission='true'),
                        'new acquire did not produce a fresh Guided grant', 12)
    return {'request_at': at, 'prepared': prepared, 'active': active,
            'stock': stock_fields(bench, at, copter), 'guided': bench.mode(int(bench.guided_mode), at)}


def finish(bench, tap, grant, route, project, *, copter):
    done = bench.wait(lambda: bench.helm_event('SITL_ROUTE_FINISHED', 'true', grant['request_at']),
                      'the revised stock route did not finish', 75)
    samples = bench.positions(grant['request_at'], done['at'] + .25)
    hits = checkpoints(samples, route, 2.5, project)
    station = bench.station(route[-1], done['at'], 2)
    tap.mark('terminal_native')
    mode = 9 if copter else 4
    started = bench.command('land' if copter else 'hold', timeout=65 if copter else 27)
    native = bench.wait(lambda: bench.mode(mode, started) or (copter and bench.mode(mode, started, armed=False)),
                        'terminal native mode was not independently observed', 6)
    require(bench.runtime(permission='false'), 'terminal native operation retained helm authority')
    bench.post('DEPLOY', 'false')
    if not bench.mode(mode, started, armed=False):
        bench.command('disarm')
    disarmed = bench.wait(lambda: bench.mode(mode, started, armed=False),
                          'terminal disarm was not independently observed', 8)
    def terminal_fix():
        fix = bench.fix(disarmed['at'])
        landed = bench.last('EXTENDED_SYS_STATE', started)
        if fix and math.dist(project(fix), route[-1]) <= 2.5 and (
                not copter or (fix['relative_alt'] <= 650 and landed and landed['landed_state'] == 1)):
            return fix
        return None

    final = bench.wait(terminal_fix, 'independent terminal effect absent at the final station', 6)
    return {'done': done, 'checkpoints': hits, 'station': station, 'native': native,
            'disarmed': disarmed, 'final_position': final}


def upload_copter_plan(bench):
    home = bench.wait(lambda: bench.state('MAVMOOS_HOME_POSITION') or None, 'missing bound HOME', 8)
    lat, lon, msl = (float(home[key]) for key in ('lat_deg', 'lon_deg', 'alt_msl_m'))
    east = 15 / (111195 * math.cos(math.radians(lat)))
    rows = ((lat, lon, msl), (lat + 55 / 111195, lon, 5), (lat + 55 / 111195, lon + east, 5))
    encoded = [f'{seq}:16:{3 if seq else 0}:0:1:0:0:0:0:{round(y * 1e7)}:{round(x * 1e7)}:{alt}'
               for seq, (y, x, alt) in enumerate(rows)]
    bench.post('MAVMOOS_MISSION_REQUEST', 'id=stress-plan,action=upload,timeout=30,items=' + '|'.join(encoded))
    receipt = bench.receipt('stress-plan', timeout=33)
    proof = bench.wait(lambda: state_after(bench, 'MAVMOOS_STORED_MISSION_PROOF', 0, valid='true',
                                          verification_id='stress-plan', count='3'),
                       'stored AUTO plan did not pass readback', 5)
    bench.result['verified_upload'] = {'result': receipt, 'proof': fields(proof['value']), 'rows': rows}


def cursor(bench, after, state):
    row = bench.last('MISSION_CURRENT', after)
    return row if row and row['seq'] == 1 and row['total'] == 2 and row['mission_state'] == state else None


def copter_stress(bench, tap):
    from run_native import xy

    takeoff = bench.command('takeoff', timeout=55, height=5)
    bench.wait(lambda: (row := bench.fix(takeoff)) and 4.4 <= row['relative_alt'] / 1000 <= 5.6,
               'independent stress takeoff height absent', 12)
    tap.mark('initial_grant')
    grant = bench.deploy()
    stock_gate = bench.wait(lambda: bench.helm_event('SURVEY_DONE', 'true', grant),
                            'stock outbound survey did not finish', 25)
    first = checkpoints(bench.positions(grant, stock_gate['at'] + .2), [(0, 8)], 2.5, xy)[0]

    proof = bench.result['verified_upload']['proof']
    tap.mark('native_start')
    started = bench.request_mission_start('stress-auto', proof)
    bench.receipt('stress-auto', 'ACTIVE')
    entry = bench.wait(lambda: bench.mode(3, started) and cursor(bench, started, mav.MISSION_STATE_ACTIVE)
                       and bench.fix(started), 'independent native AUTO entry and cursor absent', 12)
    moved = bench.wait(lambda: (fix := bench.fix(entry['at'])) and xy(fix)[1] >= xy(entry)[1] + 4 and fix,
                       'native AUTO did not move after the helm survey', 14)
    pause_at = bench.transition('stress-pause', 'pause', 'stress-auto')
    bench.receipt('stress-pause')
    bench.receipt('stress-auto', 'PAUSED')
    paused = bench.wait(lambda: bench.mode(3, pause_at) and cursor(bench, pause_at, mav.MISSION_STATE_PAUSED)
                        and bench.settled(pause_at), 'native AUTO did not brake and retain its paused cursor', 12)
    paused_cursor = cursor(bench, pause_at, mav.MISSION_STATE_PAUSED)

    update = route_update(bench, COPTER_EXIT)
    selected = bench.post('STRESS_RESUME', 'true')
    changed = bench.wait(lambda: bench.helm_event('SURVEY_RETURNING', 'true', selected),
                         'stock helm did not select the revised return while AUTO was paused', 7)
    staged = bench.wait(lambda: (inputs := stock_fields(bench, changed['at'], True)) and
                       180 < inputs['DESIRED_HEADING']['value'] < 360 and
                       inputs['DESIRED_SPEED']['value'] >= 1 and inputs,
                       'the revised stock course was not staged while AUTO owned flight', 8)
    denied = deny_acquire(bench)
    bench.post(NATIVE, 'id=stress-competing-guided,action=guided')
    competing = bench.receipt('stress-competing-guided', 'REJECTED')
    quiet = closed_window(bench, 1.2, fc_mode='3')
    require(cursor(bench, pause_at, mav.MISSION_STATE_PAUSED), 'the competing acquire changed the paused cursor')

    resume_at = bench.transition('stress-resume-auto', 'resume', 'stress-auto')
    bench.receipt('stress-resume-auto')
    resumed = bench.wait(lambda: bench.mode(3, resume_at) and cursor(bench, resume_at, mav.MISSION_STATE_ACTIVE)
                         and bench.fix(resume_at), 'native AUTO did not resume the same cursor', 10)
    resumed_travel = bench.wait(lambda: (fix := bench.fix(resumed['at'])) and xy(fix)[1] >= xy(resumed)[1] + 3 and fix,
                                 'resumed AUTO did not physically advance', 12)
    tap.drop('ack', mav.MAV_CMD_DO_SET_MODE, 4)
    abort_at = bench.transition('stress-abort', 'abort', 'stress-auto')
    abort = bench.receipt('stress-abort')
    handed_back = bench.wait(lambda: bench.mode(4, abort_at) and state_after(
        bench, 'MAVMOOS_MISSION_EXECUTION_LEASE', abort_at, held='false'), 'abort did not retire AUTO ownership', 10)
    bench.wait(tap.discarded_ack, 'addressed abort confirmation was not selectively discarded', 4)
    duplicate = duplicate_receipt(bench, 'stress-abort',
                                  lambda: bench.transition('stress-abort', 'abort', 'stress-auto'))
    tap.restore()
    closed_window(bench, 1.2, fc_mode='4')
    acquired = reacquire(bench, tap, True)
    finished = finish(bench, tap, acquired, COPTER_EXIT, xy, copter=True)
    return {'initial_gate': first, 'stock_outbound_done': stock_gate, 'auto_entry': entry, 'auto_travel': moved,
            'paused': paused, 'paused_cursor': paused_cursor, 'route_update_at': update, 'stock_return': changed,
            'staged_stock': staged, 'denied_acquire': denied, 'denied_guided': competing, 'paused_quiet': quiet,
            'resumed_travel': resumed_travel, 'resume_origin': resumed, 'abort': abort, 'handed_back': handed_back,
            'duplicate': duplicate,
            'reacquired': acquired, 'finish': finished}


def boat_stress(bench, tap):
    from run_native import contact_xy, speed, xy

    tap.mark('initial_grant')
    grant = bench.deploy()

    def established():
        fix = bench.fix(grant)
        return fix if fix and xy(fix)[1] >= 4 and abs(xy(fix)[0]) <= 1.3 and fix['vx'] >= 82 else None

    entry = bench.wait(established, 'independent boat did not establish a northbound survey', 25)
    x, y = xy(entry)
    contact = {'at': time.monotonic(), 'start_x_m': x + 14 * 1.15, 'cross_y_m': y + 14 * entry['vx'] / 100,
               'speed_m_s': 1.15}
    bench.contact = contact
    bench.result['contact_model'] = contact
    avoidance = bench.wait(lambda: bench.helm_event('CROSSING_ACTIVE', 'true', contact['at']),
                           'stock avoidance did not activate for the crossing contact', 10)
    yielded = bench.wait(lambda: (fix := bench.fix(avoidance['at'] + .5)) and
                         (speed(fix) <= .65 or abs(xy(fix)[0] - x) >= 1.5) and fix,
                         'stock collision avoidance did not physically yield or steer', 13)
    tap.drop('receive', mav.MAV_CMD_DO_SET_MODE, 5)
    at = bench.post(NATIVE, 'id=stress-loiter,action=loiter')
    loiter = bench.wait(lambda: bench.mode(5, at), 'native boat Loiter was not observed through lost RX', 6)
    terminal = bench.wait(lambda: next((row for row in bench.receipts.get('stress-loiter', ())
                                        if row['status'] in TERMINALS), None),
                          'the unconfirmed native Loiter request did not terminate', 15)
    require(terminal['status'] in ('CANCELLED', 'TIMED_OUT'), 'unconfirmed native Loiter was credited as completed')
    require(bench.runtime(permission='false', heartbeat_valid='0'), 'receive fault did not retire the old grant')
    denied = deny_acquire(bench)
    stop = bench.wait(lambda: bench.mode(5, at) and bench.settled(at), 'independent Loiter did not settle', 18)
    tap.restore()
    restored_at = time.monotonic()
    restored = bench.wait(lambda: state_after(bench, RUNTIME, restored_at, permission='false', fc_mode='5',
                                              heartbeat_valid='1'),
                          'the reconnected bridge did not observe native Loiter with a closed grant', 10)
    tap.mark('duplicate_retry')
    retry = duplicate_receipt(bench, 'stress-loiter',
                              lambda: bench.post(NATIVE, 'id=stress-loiter,action=loiter'))
    update = route_update(bench, BOAT_EXIT)
    bench.wait(lambda: (fix := bench.fix(at)) and contact_xy(contact, fix['at'])[0] <= -5 and
               bench.mode(5, at), 'native Loiter did not hold through the crossing', 30)
    closed_window(bench, 1.2, fc_mode='5')
    acquired = reacquire(bench, tap, False)
    finished = finish(bench, tap, acquired, BOAT_EXIT, xy, copter=False)
    encounter = bench.positions(contact['at'], acquired['request_at'])
    minimum = min(math.dist(xy(fix), contact_xy(contact, fix['at'])) for fix in encounter)
    require(len(encounter) >= 55 and max(b['at'] - a['at'] for a, b in zip(encounter, encounter[1:])) <= .65 and
            minimum >= 3.5, 'crossing replay had a coverage gap or inadequate closest approach')
    return {'entry': entry, 'avoidance': avoidance, 'yielded': yielded, 'loiter': loiter,
            'unconfirmed': terminal, 'denied_acquire': denied, 'native_stop': stop, 'restored': restored,
            'duplicate': retry, 'route_update_at': update, 'reacquired': acquired, 'finish': finished,
            'minimum_separation_m': minimum, 'encounter_samples': len(encounter)}


def audit_stress_wire(out, result):
    data = result['confirmation_fault']
    times = data['times']
    rows = [(at, side, packet) for _, at, side, packet in frames(out, result)]
    commands = [(at, packet) for at, side, packet in rows if side == 'T' and packet.get_type() == 'COMMAND_LONG']
    targets = [at for at, side, packet in rows if side == 'T' and packet.get_type() in NAVIGATION]
    mode_sends = [at for at, packet in commands if times['fault'] <= at < times['reacquire'] and
                  packet.command == mav.MAV_CMD_DO_SET_MODE and packet.param2 == data['mode']]
    require(len(mode_sends) == 1, f'expected one faulted native mode send, saw {len(mode_sends)}')
    affected = mode_sends[0]
    acks = [(at, side) for at, side, packet in rows if affected <= at < times['restore'] and side != 'T' and
            packet.get_type() == 'COMMAND_ACK' and packet.command == mav.MAV_CMD_DO_SET_MODE and
            (packet.target_system, packet.target_component, packet.result) == (245, 191, mav.MAV_RESULT_ACCEPTED)]
    require(len(acks) == 1 and acks[0][1] == 'i', 'the addressed accepted native mode ACK was not dropped once')
    native = affected
    if data['kind'] == 'ack':
        starts = [at for at, packet in commands if times['native_start'] <= at < times['fault'] and
                  packet.command == mav.MAV_CMD_MISSION_START]
        require(len(starts) == 1, 'expected one Copter native AUTO start')
        native = starts[0]
        require(all(packet.get_type() == 'COMMAND_ACK' and packet.command == mav.MAV_CMD_DO_SET_MODE
                    for _, side, packet in rows if side == 'i') and
                any(affected < at < times['restore'] and side == 'R' and packet.get_type() == 'HEARTBEAT'
                    for at, side, packet in rows), 'selective mode ACK fault hid other FC telemetry')
        pause_resume = [packet.param1 for at, packet in commands if native < at < affected and
                        packet.command == mav.MAV_CMD_DO_PAUSE_CONTINUE]
        require(pause_resume == [0, 1], 'pause or resume did not reach the wire exactly once and in order')
    else:
        require(not any(native <= at < times['restore'] and side == 'R' for at, side, _ in rows),
                'the boat bridge received FC data during its blackout')
        require(not any(times['duplicate_retry'] <= at < times['reacquire'] and
                        packet.command == mav.MAV_CMD_DO_SET_MODE for at, packet in commands),
                'the duplicate boat request resent a mode command')
    require(any(times['initial_grant'] < at < native for at in targets) and
            any(times['reacquire'] < at < times['terminal_native'] for at in targets) and
            not any(native <= at < times['reacquire'] for at in targets),
            'navigation did not occur on both sides of a quiet native fence')
    return {'frames': len(rows), 'targets_before_handoff': sum(at < native for at in targets),
            'native_mode_sends_before_reacquire': 1, 'discarded_accepted_ack': True,
            'targets_during_native_fence': 0,
            'targets_after_reacquire': sum(times['reacquire'] < at < times['terminal_native'] for at in targets)}
