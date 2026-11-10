"""Exercise task dispatch through stock helm, with separate acquisition."""
from control_workflow import POINT, distance, ensure_for, fields
from flight_fixture import FBWA, GUIDED, LOITER, need, now
from lifecycle import DIRECT, result
from operator_nudges import exercise_operator_nudges, native_handback


def select(mission, identifier, action, timeout):
    mark = mission.notify('MAVMOOS_CONTROL_REQUEST',
                          f'schema=1,id={identifier},action={action},strategy=helm,timeout={timeout}')
    selected = result(mission, DIRECT, identifier, 'SELECTED', mark)
    selector = mission.wait(3, lambda: (r if (r := mission.latest('MAVMOOS_HELM_OPERATION', after=mark)) and
                                        r['value'] == action else None), 'selected behavior variable was not observed')
    ensure_for(mission, .3, lambda: mission.flight.heartbeat(LOITER, True) and not any(
        message.get_type() == 'COMMAND_LONG' and message.command == 176 and message.param2 == GUIDED
        for _, message in mission.tx(mark)), 'task selection acquired FC control without separate permission')
    ready = mission.wait(4, lambda: (r if (r := mission.latest('MAVMOOS_HELM_TASK_READY', 'pHelmIvP', after=mark)) and
                                     r['value'] == action and mission.state('MAVMOOS_PLANE_HELM_TASK_STATUS',
                                                                            acquire_ready='true') else None),
                         'configured stock task did not post fresh activeflag readiness')
    acquired = mission.acquire(verify=False)
    active = result(mission, DIRECT, identifier, 'ACTIVE', mark)

    def paired():
        home = mission.flight.latest('HOME_POSITION')
        producer = mission.latest('DESIRED_HEADING', 'pHelmIvP', after=acquired)
        if not home or not producer:
            return None
        for record, point in mission.tx(acquired):
            if point.get_type() != 'COMMAND_INT' or point.command != POINT:
                continue
            for row in reversed(mission.flight.monitor.messages):
                if row['mono'] <= record.at:
                    break
                if row['mavpackettype'] == 'POSITION_TARGET_GLOBAL_INT' and (row['lat_int'], row['lon_int']) == (point.x, point.y) and \
                        abs(row['alt'] - home['altitude'] / 1000 - point.z) <= .3:
                    return {'wire_mono': record.at, 'producer': producer, 'target': row}
        return None

    target = mission.wait(8, paired, 'fresh stock behavior did not produce a paired native target')
    return mark, selected, acquired, {'active': active, 'selector': selector, 'ready': ready, 'target': target}


def motion(mission, after, minimum=45):
    start = mission.flight.latest('GLOBAL_POSITION_INT')

    def changed():
        position = mission.flight.latest('GLOBAL_POSITION_INT', after)
        return position if position and distance(start, position) >= minimum and mission.helm(state='ACTIVE') else None

    stop = mission.wait(20, changed, 'stock selected task did not produce independent flight motion')
    return {'start': start, 'stop': stop, 'travel_m': distance(start, stop)}


def exercise_helm_tasks(mission):
    nudges = exercise_operator_nudges(mission)
    flight = mission.flight
    mission.notify('DEPLOY', 'true')
    return_at, selected_return, return_acquire, return_active = select(mission, 'fixture-task-return', 'return', 55)
    return_start = flight.latest('GLOBAL_POSITION_INT')
    home = flight.latest('HOME_POSITION')
    update = mission.latest('MAVMOOS_OPERATION_RETURN_UPDATE', after=return_at, max_age=12)
    need(update and update['value'].startswith('points='), 'projected home was not delivered to the return behavior')
    point = {'lat': home['latitude'], 'lon': home['longitude']}
    returning = mission.wait(22, lambda: (p if (p := flight.latest('GLOBAL_POSITION_INT', return_at)) and
                                         distance(point, return_start) - distance(point, p) >= 35 else None),
                             'helm return task did not measurably approach home')
    cancel_at = mission.notify('MAVMOOS_CONTROL_REQUEST',
                               'schema=1,id=fixture-task-return-cancel,action=cancel,target=fixture-task-return')
    cancelled = result(mission, DIRECT, 'fixture-task-return', 'CANCELLED', cancel_at)
    native_handback(mission, cancel_at)

    survey_at, selected_survey, survey_acquire, survey_active = select(mission, 'fixture-task-survey', 'survey', 20)
    survey_motion = motion(mission, survey_at)
    expired = result(mission, DIRECT, 'fixture-task-survey', 'TIMED_OUT', survey_at, 24)
    expiry = float(fields(selected_survey)['expires'])
    native_handback(mission, survey_at)
    need(not any(record.at >= expiry and message.get_type() == 'COMMAND_INT' and message.command == 192
                 for record, message in mission.tx(survey_at)), 'task point crossed its original expiry')

    voronoi_at, selected_voronoi, voronoi_acquire, voronoi_active = select(mission, 'fixture-task-voronoi', 'voronoi', 50)
    voronoi_motion = motion(mission, voronoi_at)
    takeover = flight.mode(FBWA)
    taken = result(mission, DIRECT, 'fixture-task-voronoi', 'CANCELLED', voronoi_at)
    ensure_for(mission, 1, lambda: flight.heartbeat(FBWA, True) and not any(
        message.get_type() == 'COMMAND_INT' and message.command == 192
        for record, message in mission.tx(taken['mono'])), 'selected task emitted a new point after app-observed takeover')
    restored = flight.mode(LOITER)
    mission.wait(10, lambda: flight.trimmed(LOITER), 'external Loiter restoration did not settle')
    ensure_for(mission, 1, lambda: not mission.state('MAVMOOS_PLANE_HELM_TASK_STATUS', selected='true') and
               not any(message.get_type() == 'COMMAND_LONG' and message.command == 176 and message.param2 == GUIDED
                       for _, message in mission.tx(restored['mono'])), 'old selected task reacquired after restoration')
    return {'nudges': nudges, 'return_selected': selected_return, 'return_acquired_mono': return_acquire,
            'return_active': return_active, 'return_cancelled': cancelled,
            'return_update': update,
            'return_closure_m': distance(point, return_start) - distance(point, returning),
            'survey_selected': selected_survey, 'survey_acquired_mono': survey_acquire, 'survey_active': survey_active,
            'survey_motion': survey_motion, 'survey_expired': expired, 'survey_original_expiry': expiry,
            'voronoi_selected': selected_voronoi, 'voronoi_acquired_mono': voronoi_acquire, 'voronoi_active': voronoi_active,
            'voronoi_motion': voronoi_motion, 'external_takeover': takeover, 'voronoi_cancelled': taken}
