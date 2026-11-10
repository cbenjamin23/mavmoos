"""Cross native pilot, Loiter and RTL ownership during a stock-helm survey."""
import json
import math

from autonomy_missions.stock_grade import checkpoints, grade, stage_row
from autonomy_survey import project
from control_workflow import (FBWA, GUIDED, LOITER, POINT, RTL, SET_MODE, angle, ensure_for,
                              fields, mode_receipt, native_command, orbit, packet_frames,
                              pilot_switch, points, positions, signed_turn)
from flight_fixture import need

SCALARS = ('DESIRED_HEADING', 'DESIRED_SPEED', 'DESIRED_ALTITUDE')


def pick_gate(position, distance, bearings):
    x, y = project(position)
    course = math.atan2(position['vy'], position['vx'])
    candidates = [(x + distance * math.sin(course + math.radians(turn)),
                   y + distance * math.cos(course + math.radians(turn))) for turn in bearings]
    bounded = [point for point in candidates if math.hypot(*point) <= 790]
    need(bounded, 'no new waypoint inside the pinned operating area')
    return tuple(round(v, 1) for v in max(bounded, key=lambda point: math.hypot(*point)))


def route_update(mission, key, point):
    return mission.notify(key, 'points=' + ','.join(map(str, point)))


def stock_scalars(mission, after, height):
    rows = {key: mission.latest(key, 'pHelmIvP', after, max_age=1.5) for key in SCALARS}
    return rows if (all(rows.values()) and rows['DESIRED_SPEED']['value'] == 22 and
                    rows['DESIRED_ALTITUDE']['value'] == height) else None


def stock_detour(mission, after, point):
    scalars = stock_scalars(mission, after, 73)
    position = mission.flight.latest('GLOBAL_POSITION_INT', after)
    if not position or not scalars:
        return None
    x, y = project(position)
    course = angle(point[1] - y, point[0] - x)
    if abs(signed_turn(course, scalars['DESIRED_HEADING']['value'])) <= 24:
        return scalars | {'position': position}


def run(mission):
    flight = mission.flight
    position = flight.latest('GLOBAL_POSITION_INT')
    need(position is not None, 'initial independent fixed-wing fix absent')
    entry = pick_gate(position, 165, (0,))
    route_update(mission, 'AUTONOMY_ROUTE', entry)
    mission.notify('DEPLOY', 'true')
    first = mission.acquire(verify=False)
    entry_stock = mission.wait(4, lambda: stock_scalars(mission, first, 65),
                               'initial survey lacked fresh stock helm decisions')

    def stage(value, after):
        return mission.wait(48, lambda: stage_row(mission.latest('AUTONOMY_STAGE', 'pHelmIvP'), value, after),
                            f'stock fixed-wing stage {value} not reached')

    detour_stage = stage('detour', first)
    entry_hit, = checkpoints(positions(flight, first, detour_stage['mono'] + .2), [entry], 48, project)
    mission.fault('hold_point')
    held, target = mission.wait(4, lambda: points(mission, kinds='h'),
                                'next owned Guided target was not held')[0]
    fbwa = pilot_switch(flight, FBWA)
    mode_receipt(flight, 'stress_pilot_fbwa', fbwa, 1)
    mark = mission.fault('release_point')
    late, _ = mission.wait(4, lambda: points(mission, mark, 'd'),
                          'held target was not delivered after FBWA takeover')[0]
    attempt = mission.notify('MAVMOOS_NAV_CONTROL', 'acquire')
    refusal = mission.wait(4, lambda: mission.helm(attempt, state='LATCHED', reason='acquisition_rejected'),
                           'FBWA did not deny a competing acquire')
    mission.wait(3, lambda: mission.runtime(attempt, permission='false', fc_mode=FBWA),
                 'FBWA did not retire the prior grant')
    ensure_for(mission, 1.25, lambda: flight.heartbeat(FBWA, True) and not points(mission, fbwa['mono']),
               'FBWA did not retain exclusive control')
    mission.fault('pass')
    native = pilot_switch(flight, LOITER)
    mode_receipt(flight, 'stress_pilot_loiter', native, 1)
    native_orbit = orbit(flight, LOITER, native['mono'])
    detour = pick_gate(flight.latest('GLOBAL_POSITION_INT'), 230, (-35, -15, 0, 15, 35))
    update_at = route_update(mission, 'AUTONOMY_DETOUR', detour)
    stock = mission.wait(6, lambda: stock_detour(mission, update_at, detour),
                         'stock helm did not publish the revised waypoint course during native Loiter')
    need(not points(mission, fbwa['mono']) and flight.heartbeat(LOITER, True),
         'a changed route silently acquired control')
    second = mission.acquire(height=73, verify=False)
    done = stage('done', second)
    detour_hit, = checkpoints(positions(flight, second, done['mono'] + .2), [detour], 48, project)
    need(abs(detour_hit['fix']['relative_alt'] / 1000 - 73) <= 11 and
         math.dist(project(detour_hit['fix']), project(stock['position'])) >= 140,
         'revised stock waypoint did not produce a distinct physical leg')
    rtl = native_command(mission, 'rtl', RTL, 'stress_native_rtl')
    final_orbit = orbit(flight, RTL, rtl)
    approach = positions(flight, rtl, final_orbit['end_mono'])
    last = approach[-1]
    homeward = math.hypot(*project(detour_hit['fix'])) - math.hypot(*project(last))
    need(homeward >= 70 and math.hypot(*project(last)) <= 165 and not points(mission, rtl),
         'native RTL did not return toward home with the bridge fenced')
    mission.notify('DEPLOY', 'false')
    verdict = grade(mission.notify, lambda check: mission.wait(5, check, 'stock Plane stress grade absent'),
                    mission.out)
    return {'entry_acquire': first, 'entry_stock': entry_stock, 'entry_gate': entry_hit, 'detour_stage': detour_stage,
            'held_at': held.at, 'delivered_at': late.at, 'held_target': target.to_dict(), 'pilot_fbwa': fbwa,
            'denied_acquire_at': attempt, 'refusal': refusal, 'native_loiter': native, 'loiter_orbit': native_orbit,
            'detour_update_at': update_at, 'detour_stock': stock, 'detour_acquire': second, 'done_stage': done,
            'detour_gate': detour_hit, 'native_rtl_at': rtl, 'rtl_orbit': final_orbit,
            'rtl_fix': last, 'homeward_m': homeward, 'grade': verdict}


def audit(directory, report, records):
    from pymavlink.dialects.v20 import all as codec

    result = report['workflow']
    moos = [json.loads(line) for line in (directory / 'moos-probe.jsonl').read_text().splitlines()
            if line.startswith('{')]
    monitor = [json.loads(line) for line in (directory / 'monitor-rx.jsonl').read_text().splitlines()]
    need(result['grade'] == (directory / 'results.txt').read_text().strip() == 'grade=pass',
         'Plane stress mission grade did not replay')
    stock = result['detour_stock']
    need(all(row in moos and row['source'] == 'pHelmIvP' and result['entry_acquire'] < row['mono'] <
             result['detour_stage']['mono'] for row in result['entry_stock'].values()) and
         all(stock[key] in moos and stock[key]['source'] == 'pHelmIvP' and
             result['detour_update_at'] < stock[key]['mono'] < result['detour_acquire'] for key in SCALARS) and
         all(result[key] in moos and result[key]['source'] == 'pHelmIvP' for key in ('detour_stage', 'done_stage')) and
         result['refusal'] in moos and fields(result['refusal'])['reason'] == 'acquisition_rejected' and
         not any(row['kind'] == 'request' and row['key'].startswith('DESIRED_') for row in moos),
         'stock helm origin, detour timing or acquisition denial differs')
    frames = list(packet_frames(codec, records))
    held = [r for r, m in frames if r.disposition == 'h' and m.command == POINT]
    delivered = [r for r, m in frames if r.disposition == 'd' and m.command == POINT]
    fence, restored, terminal = (result[key] for key in ('pilot_fbwa', 'detour_acquire', 'native_rtl_at'))
    need(len(held) == len(delivered) == 1 and held[0].data == delivered[0].data and
         held[0].at == result['held_at'] < fence['mono'] < delivered[0].at == result['delivered_at'],
         'late point did not replay across independently observed FBWA')
    acks = []
    for record in records:
        if record.disposition == 'R' and 0 < record.at - delivered[0].at <= 1:
            decoder = codec.MAVLink(None)
            decoder.robust_parsing = True
            for msg in decoder.parse_buffer(record.data) or []:
                if msg.get_type() == 'COMMAND_ACK' and (msg.get_srcSystem(), msg.get_srcComponent(),
                        msg.command, msg.result, msg.target_system, msg.target_component) == (1, 1, POINT, 4, 245, 191):
                    acks.append(msg)
    native_frames = [(r, m) for r, m in frames if r.disposition == 'T']
    live_points = [(r.at, m) for r, m in native_frames if m.get_type() == 'COMMAND_INT' and m.command == POINT]
    modes = [int(m.param2) for _, m in native_frames if m.get_type() == 'COMMAND_LONG' and m.command == SET_MODE]
    need(len(acks) == 1 and modes == [GUIDED, GUIDED, RTL] and
         not any(fence['mono'] <= at < restored or at >= terminal for at, _ in live_points),
         'late target acceptance, duplicate mode command or navigation leaked across the fence')
    point = result['held_target']
    target_echoes = [row for row in monitor if row['mavpackettype'] == 'POSITION_TARGET_GLOBAL_INT']
    need(not any(delivered[0].at <= row['mono'] < restored and
                 (row['lat_int'], row['lon_int']) == (point['x'], point['y']) for row in target_echoes),
         'delayed Guided target changed the FC target during native ownership')
    for start, end in ((result['entry_acquire'], result['entry_gate']['fix']['mono']),
                       (restored, result['detour_gate']['fix']['mono'])):
        need(any(start < at < end and any(
            0 < row['mono'] - at <= 1.5 and (row['lat_int'], row['lon_int']) == (msg.x, msg.y)
            for row in target_echoes) for at, msg in live_points), 'an active stock leg lacked an FC target readback')
    for key in ('entry_gate', 'detour_gate'):
        gate = result[key]
        need(gate['fix'] in monitor and math.isclose(math.dist(project(gate['fix']), gate['goal']), gate['error_m']),
             f'physical {key} did not replay')
    end, begin = result['rtl_fix'], result['detour_gate']['fix']
    need(fence in monitor and result['native_loiter'] in monitor and end in monitor and
         any(row['mavpackettype'] == 'HEARTBEAT' and row['mono'] == terminal and row['custom_mode'] == RTL
             for row in monitor) and
         math.isclose(result['homeward_m'], math.hypot(*project(begin)) - math.hypot(*project(end))),
         'native mode or homeward track did not replay')
    return {'held_point_rejected_in_fbwa': True, 'bridge_modes': modes, 'active_targets': len(live_points),
            'fenced_s': restored - fence['mono'], 'homeward_m': result['homeward_m']}
