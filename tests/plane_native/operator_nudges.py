"""Witness bounded Plane course, altitude and home marker requests."""
import math

from control_workflow import COURSE, HEIGHT, POINT, angle, distance, fields, offset, signed_turn
from flight_fixture import LOITER, need
from lifecycle import DIRECT, result


def adjust(mission, identifier, action, delta):
    return mission.notify('MAVMOOS_CONTROL_REQUEST',
                          f'schema=1,id={identifier},action={action},strategy=guided,delta={delta},timeout=70')


def first_point(mission, mark):
    return next(((record, message) for record, message in mission.tx(mark)
                 if message.get_type() == 'COMMAND_INT' and message.command == POINT), None)


def native_handback(mission, mark):
    return mission.wait(10, lambda: mission.flight.heartbeat(LOITER, True, mark) and
                        mission.runtime(plane_handback='CONFIRMED'), 'observed native Loiter handback missing')


def post_altitude_dwell(mission, target, after):
    flight = mission.flight
    samples = []

    def steady():
        position = flight.latest('GLOBAL_POSITION_INT', after, max_age=.5)
        reported = flight.latest('POSITION_TARGET_GLOBAL_INT', after, max_age=.5)
        heartbeat = flight.heartbeat(LOITER, True, after)
        if not position or not reported or not heartbeat:
            samples.clear()
            return None
        if abs(position['relative_alt'] / 1000 - target) > 2:
            samples.clear()
            return None
        if samples and position['mono'] <= samples[-1]['position']['mono']:
            return None
        samples.append({'position': position, 'target': reported, 'heartbeat': heartbeat})
        return list(samples) if position['mono'] - samples[0]['position']['mono'] >= 1.25 else None

    return mission.wait(20, steady, 'post-completion native Loiter did not dwell within 2 m of the requested altitude')


def exercise_operator_nudges(mission):
    flight = mission.flight
    mission.wait(25, lambda: mission.runtime(plane_profile='VERIFIED', plane_watchdog_started=1) and
                 flight.trimmed(LOITER), 'Plane not ready for direct operator requests')
    home_at = mission.notify('MAVMOOS_CONTROL_REQUEST', 'schema=1,id=fixture-home,action=visualize_home')
    home_result = result(mission, DIRECT, 'fixture-home', 'COMPLETED', home_at)
    marker = mission.wait(4, lambda: mission.state('VIEW_MARKER', home_at, label='MAVMOOS_HOME', active='true'),
                          'projected home marker did not arrive')
    drawn = fields(marker)
    need(math.hypot(float(drawn['x']), float(drawn['y'])) < 15, 'home marker is outside the known origin')

    mission.configure_leg(COURSE, HEIGHT)
    mission.acquire(COURSE, HEIGHT)
    busy_at = adjust(mission, 'fixture-owned-by-helm', 'adjust_altitude', 12)
    blocked = result(mission, DIRECT, 'fixture-owned-by-helm', 'REJECTED', busy_at)
    need(fields(blocked)['reason'] == 'idle_native_loiter_required' and mission.helm(state='ACTIVE'),
         'relative nudge displaced the existing stock helm grant')
    released_at = mission.notify('MAVMOOS_NAV_CONTROL', 'release')
    native_handback(mission, released_at)
    mission.notify('DEPLOY', 'false')

    unsafe_at = adjust(mission, 'fixture-course-limit', 'adjust_course', 100)
    unsafe = result(mission, DIRECT, 'fixture-course-limit', 'REJECTED', unsafe_at)
    course_start = flight.latest('GLOBAL_POSITION_INT')
    course_at = adjust(mission, 'fixture-course', 'adjust_course', 45)
    _, point = mission.wait(10, lambda: first_point(mission, course_at), 'adjusted course point did not reach the FC')
    north, east = offset(course_start['lat'], course_start['lon'], point.x, point.y)
    turn = signed_turn(angle(course_start['vx'], course_start['vy']), angle(north, east))
    need(260 <= math.hypot(north, east) <= 335 and 20 <= turn <= 70, 'waypoint did not project the relative course')
    course = result(mission, DIRECT, 'fixture-course', 'COMPLETED', course_at, 55)
    course_arrival = flight.latest('GLOBAL_POSITION_INT')
    travelled = distance(course_start, course_arrival)
    need(travelled >= 135, 'independent position did not follow the adjusted course')
    native_handback(mission, course_at)

    altitude_start = flight.latest('GLOBAL_POSITION_INT')
    altitude_at = adjust(mission, 'fixture-altitude', 'adjust_altitude', 12)
    _, target = mission.wait(10, lambda: first_point(mission, altitude_at), 'adjusted altitude point did not reach the FC')
    need(abs(target.z - altitude_start['relative_alt'] / 1000 - 12) <= 2,
         'wire altitude did not bind the requested relative offset')
    altitude = result(mission, DIRECT, 'fixture-altitude', 'COMPLETED', altitude_at, 55)
    height = flight.latest('GLOBAL_POSITION_INT')
    climbed = (height['relative_alt'] - altitude_start['relative_alt']) / 1000
    need(climbed >= 8 and abs(height['relative_alt'] / 1000 - target.z) <= 4, 'aircraft did not climb to the adjusted height')
    native_handback(mission, altitude_at)
    dwell = post_altitude_dwell(mission, target.z, altitude['mono'])
    return {'home_marker': marker, 'home_result': home_result, 'existing_helm_blocked': blocked,
            'unbounded_course_rejected': unsafe, 'course_result': course, 'course_turn_deg': turn,
            'course_travel_m': travelled, 'altitude_result': altitude, 'climb_m': climbed,
            'altitude_target_m': target.z, 'independent_altitude': height, 'post_completion_altitude_dwell': dwell}
