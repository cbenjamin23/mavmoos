"""Disrupt a stock hover inspection, cross native forward flight, and resume."""
import math

from autonomy_missions.stock_grade import checkpoints, grade, stage_row
from flight_fixture import need
from hover_helm import HOVER_VALUES, acquire, adapter_loss, hold_ready, hover_wire
from hover_survey import project
from live_helm import (FW, GROUND, GUIDED, LOITER, MC, QLOITER, QRTL, TO_FW, archived_rows,
                       fields, hold_no_grant, observe, refuse, wire)
from replay import verify_capture


def stock(flight, after):
    rows = {key: flight.probe.last.get(key) for key in HOVER_VALUES}
    return rows if all(row and row['source'] == 'pHelmIvP' and row['mono'] > after for row in rows.values()) else None


def fixes(flight, start, stop):
    return [row for row in flight.monitor.messages if row['mavpackettype'] == 'GLOBAL_POSITION_INT' and
            start <= row['mono'] <= stop]


def fly(flight):
    probe = flight.probe
    initial = acquire(flight, fixed_inputs=False)
    entry_stock = flight.wait(5, lambda: stock(flight, initial['request_at']), 'initial stock hover decisions absent')

    def stage(value, after, timeout):
        return flight.wait(timeout, lambda: stage_row(probe.last.get('AUTONOMY_STAGE'), value, after),
                           f'stock QuadPlane phase {value} absent')

    climb = stage('climb', initial['request_at'], 20)
    entry, = checkpoints(fixes(flight, initial['request_at'], climb['mono'] + .25), [(8, 0)], 2.9, project)
    climb_witness = flight.wait(5, lambda: observe(flight, GUIDED, MC, climb['mono'], minimum=61),
                                'initial inspection climb did not start')
    loss = adapter_loss(flight, fixed_inputs=False)
    hold_ready(flight, loss['quiet']['through'])
    forward_at, _ = flight.operator_mode(LOITER, True)
    transition = flight.wait(9, lambda: observe(flight, LOITER, TO_FW, forward_at, minimum=35),
                             'native forward transition absent')
    during = refuse(flight, 'native transition', transition)
    forward = flight.wait(55, lambda: observe(flight, LOITER, FW, forward_at, minimum=35, velocity=18),
                          'independent forward flight absent')
    steady = refuse(flight, 'stable native forward flight', forward)
    origin = project(forward['position'])

    def progress():
        state = observe(flight, LOITER, FW, forward['position']['mono'], minimum=35, velocity=18)
        return state if state and math.dist(origin, project(state['position'])) >= 65 else None

    native_leg = flight.wait(15, progress, 'native forward Loiter did not fly the required arc')
    returning, _ = flight.operator_mode(QLOITER, True)
    settled = hold_ready(flight, returning)
    x, y = project(settled['position'])
    route = [(round(x + 8, 2), round(y, 2)), (round(x + 8, 2), round(y + 7, 2))]
    update = probe.post('AUTONOMY_ROUTE', 'points=' + ':'.join(f'{east},{north}' for east, north in route))

    def revised_course():
        rows = stock(flight, update)
        return rows if rows and abs(math.remainder(rows['DESIRED_HEADING']['value'] - 90, 360)) <= 24 else None

    route_stock = flight.wait(5, revised_course, 'revised inspection waypoint did not change the stock hover course')
    quiet = hold_no_grant(flight, loss['quiet']['nav_count'], 1.25, QLOITER)
    resumed = acquire(flight, fixed_inputs=False)
    reacquired_stock = flight.wait(5, lambda: stock(flight, resumed['request_at']),
                                   'fresh stock hover decisions absent after reacquire')
    done = stage('done', resumed['request_at'], 32)
    hit = checkpoints(fixes(flight, resumed['request_at'], done['mono'] + .25), route, 2.9, project)
    high = flight.wait(8, lambda: observe(flight, GUIDED, MC, done['mono'], minimum=63.5),
                       'independent post-inspection height absent')
    at, _ = flight.operator_mode(QRTL, True)
    retired = flight.wait(5, lambda: probe.runtime(at, permission='false', fc_mode=str(QRTL)),
                          'QRTL did not retire the second helm grant')
    touchdown = flight.wait(125, lambda: flight.observation(after=at, mode=QRTL, armed=False,
                              phase=MC, landed=GROUND), 'native QRTL did not land and disarm')
    verdict = grade(probe.post, lambda check: flight.wait(5, check, 'stock QuadPlane stress grade absent'), flight.out)
    return {'first_acquire': initial, 'entry_stock': entry_stock, 'entry_gate': entry, 'climb_stage': climb,
            'climb_witness': climb_witness, 'adapter_loss': loss, 'forward_requested_at': forward_at,
            'transition_refusal': during, 'steady_refusal': steady, 'native_forward_start': forward,
            'native_forward_end': native_leg, 'forward_displacement_m': math.dist(origin, project(native_leg['position'])),
            'qloiter_requested_at': returning, 'settled_qloiter': settled, 'route_update_at': update,
            'updated_stock': route_stock, 'qloiter_quiet': quiet, 'second_acquire': resumed,
            'reacquired_stock': reacquired_stock, 'revised_gates': hit, 'done_stage': done, 'high_hover': high,
            'qrtl_requested_at': at, 'qrtl_retired': retired, 'touchdown': touchdown, 'grade': verdict}


def audit(out, report):
    count = verify_capture(out, 'monitor')
    monitor = archived_rows(out, 'monitor-rx.jsonl')
    moos = archived_rows(out, 'moos.jsonl')
    result = report['stress']
    first, second = (result[key]['request_at'] for key in ('first_acquire', 'second_acquire'))
    need(result['grade'] == (out / 'results.txt').read_text().strip() == 'grade=pass' and
         not any(row['kind'] == 'request' and row['key'].startswith('DESIRED_') for row in moos),
         'QuadPlane stress grade or source isolation did not replay')
    for key, after in (('entry_stock', first), ('updated_stock', result['route_update_at']),
                       ('reacquired_stock', second)):
        need(all(row in moos and row['source'] == 'pHelmIvP' and row['mono'] > after
                 for row in result[key].values()), f'{key} stock decisions absent')
    need(all(row['mono'] < second for row in result['updated_stock'].values()) and
         all(result[key] in moos and result[key]['source'] == 'pHelmIvP' for key in ('climb_stage', 'done_stage')),
         'stock helm did not replan while QLoiter owned the vehicle')
    for gate in (result['entry_gate'], *result['revised_gates']):
        need(gate['fix'] in monitor and math.isclose(math.dist(project(gate['fix']), gate['goal']), gate['error_m']),
             'independent QuadPlane stress waypoint proof differs')
    for key, phase in (('transition_refusal', TO_FW), ('steady_refusal', FW)):
        denied = result[key]
        sample = denied['phase_sample']
        need(all(row in monitor for row in sample.values()) and sample['state']['vtol_state'] == phase and
             sample['heartbeat']['custom_mode'] == LOITER and denied['rejected'] in moos and denied['runtime'] in moos
             and fields(denied['rejected'])['reason'] == 'acquisition_rejected'
             and fields(denied['runtime'])['permission'] == 'false',
             'native forward acquisition denial or phase evidence differs')
    forward = [result[key]['position'] for key in ('native_forward_start', 'native_forward_end')]
    need(all(row in monitor for row in forward) and math.isclose(result['forward_displacement_m'],
         math.dist(*map(project, forward))) and result['forward_displacement_m'] >= 65,
         'native fixed-wing progress did not replay')
    loss = result['adapter_loss']
    hold = loss['hold']
    need(hold['runtime'] in moos and fields(hold['runtime'])['permission'] == 'false' and
         all(row in monitor for row in hold['native'].values()) and hold['native']['heartbeat']['custom_mode'] == QLOITER and
         all(row in monitor for row in result['climb_witness'].values()) and
         all(row in monitor for row in result['high_hover'].values()) and
         all(result['settled_qloiter'][key] in monitor for key in ('heartbeat', 'state', 'position')),
         'adapter-loss native hold or physical hover observations did not replay')
    decoded = list(wire(out, report))
    need(any(side == 'h' and at == loss['pending_at'] and msg.get_type() == 'TUNNEL' for at, side, msg in decoded) and
         any(side == 'i' and loss['lost_at'] <= at < loss['restored_at'] and msg.get_type() == 'TUNNEL'
             for at, side, msg in decoded),
         'selective adapter interruption did not hold a pending target and withhold an incoming offer')
    targets, matched, modes, _ = hover_wire(out, report, monitor)
    retired = result['qrtl_retired']
    need(retired in moos and fields(retired)['permission'] == 'false' and
         not any(loss['lost_at'] <= at < second or at >= retired['mono'] for at in targets) and
         any(first < at < loss['pending_at'] for at in matched) and
         any(second < at < result['done_stage']['mono'] for at in matched) and
         [mode for _, mode in modes].count(GUIDED) == 2 and all(mode in (GUIDED, QLOITER) for _, mode in modes),
         'old hover authority, duplicate acquire, or unconfirmed targets crossed a native boundary')
    need(all(row in monitor for row in result['touchdown'].values()) and
         result['touchdown']['heartbeat']['custom_mode'] == QRTL and
         not result['touchdown']['heartbeat']['base_mode'] & 128,
         'terminal native QRTL did not replay as landed and disarmed')
    return {'monitor_crc_frames': count, 'hover_targets': len(targets), 'target_readbacks': len(matched),
            'fenced_s': second - loss['lost_at'], 'native_forward_m': result['forward_displacement_m'],
            'denied_acquires': 2}
