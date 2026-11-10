"""Fly two fixed-wing survey gates, then follow the stock homeward leg."""
import math

from autonomy_missions.stock_grade import checkpoints, grade, local_xy, stage_row
from control_workflow import positions, released
from flight_fixture import HOME, need


def project(row):
    return local_xy(row, HOME[0], HOME[1])


def survey(mission):
    flight = mission.flight
    initial = flight.latest('GLOBAL_POSITION_INT')
    need(initial and math.hypot(initial['vx'], initial['vy']) > 1300, 'independent airborne trim missing')
    x, y = project(initial)
    course = math.atan2(initial['vy'], initial['vx'])
    bearing = course + math.radians(45)
    first = (x + 145 * math.sin(course), y + 145 * math.cos(course))
    second = (first[0] + 175 * math.sin(bearing), first[1] + 175 * math.cos(bearing))
    route = tuple(tuple(round(value, 1) for value in point) for point in (first, second))
    mission.notify('AUTONOMY_ROUTE', 'points=' + ':'.join(f'{a},{b}' for a, b in route))
    mission.notify('DEPLOY', 'true')
    acquired = mission.acquire(verify=False)

    def stage(value, after):
        return mission.wait(55, lambda: stage_row(mission.latest('AUTONOMY_STAGE', 'pHelmIvP'), value, after),
                            f'stock fixed-wing stage {value} not reached')

    returning = stage('return', acquired)
    hits = checkpoints(positions(flight, acquired, returning['mono'] + .25), route, 55, project)
    start_home = math.hypot(*project(hits[-1]['fix']))
    need(start_home >= 110, 'survey did not leave enough independent return distance')
    finished = stage('done', returning['mono'])
    home = mission.wait(3, lambda: flight.latest('GLOBAL_POSITION_INT', finished['mono']),
                        'independent home approach fix missing')
    home_range = math.hypot(*project(home))
    need(home_range <= 70 and start_home - home_range >= 70, 'stock return did not reach the home approach')
    release = mission.notify('MAVMOOS_NAV_CONTROL', 'release')
    loiter_at = released(mission, release, 'survey_native_loiter')
    mission.notify('DEPLOY', 'false')
    verdict = grade(mission.notify, lambda check: mission.wait(5, check, 'stock Plane mission grade absent'),
                    mission.out)
    return {'checkpoints': hits, 'return_stage': returning, 'done_stage': finished,
            'home_fix': home, 'homeward_m': start_home - home_range, 'home_range_m': home_range,
            'native_loiter_at': loiter_at, 'grade': verdict}
