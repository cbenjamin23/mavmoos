"""Conventional Plane lifecycle driven by the actual bridge API."""
import math

from control_workflow import (fields, matches, distance, ensure_for, pilot_switch, packet_frames,
                              fault_tap, COURSE, HEIGHT, FBWA)
from flight_fixture import GUIDED, LOITER, ground_speed, need, now

AUTO = 10
MANUAL = 0
EXECUTION = 'MAVMOOS_MISSION_EXECUTION_RESULT'
COMMAND = 'MAVMOOS_COMMAND_RESULT'
DIRECT = 'MAVMOOS_CONTROL_RESULT'


def result(mission, variable, identifier, status, after, timeout=8):
    def observed():
        return next((row for row in reversed(mission.moos_rows)
                     if row.get('source') == 'iMAVLink' and row.get('key') == variable
                     and row['mono'] > after and matches(fields(row), id=identifier, status=status)), None)
    return mission.wait(timeout, observed, f'{identifier} did not reach {status}')


def command(mission, identifier, action, status='CONFIRMED', extra='', timeout=8):
    mark = mission.notify('MAVMOOS_COMMAND', f'id={identifier},action={action}{extra}')
    return mark, result(mission, COMMAND, identifier, status, mark, timeout)


def proof_upload(mission, home):
    x, y = home['latitude'], home['longitude']
    runway_sign = -1 if mission.paths.get('modern') else 1

    def point(north):
        return round(x + north / 111319.5 * 1e7)

    rows = [
        (0, 16, 0, 0, 1, 0, 0, 0, 0, x, y, home['altitude'] / 1000),
        (1, 22, 3, 0, 1, 15, 0, 0, 0, point(350), y, 65),
        (2, 17, 3, 0, 1, 0, 0, 0, 0, point(360), y, 65),
        (3, 16, 3, 0, 1, 0, 0, 0, 0, point(runway_sign * 740), y, 45),
        (4, 16, 3, 0, 1, 0, 0, 0, 0, point(runway_sign * 485), y, 45),
        (5, 21, 3, 0, 1, 0, 0, 0, 1, x, y, 0),
    ]
    def request(identifier, items):
        encoded = '|'.join(':'.join(map(str, row)) for row in items)
        return mission.notify('MAVMOOS_MISSION_REQUEST', f'id={identifier},action=upload,timeout=40,items=' + encoded)

    invalid = rows.copy()
    invalid[3] = (*rows[3][:9], point(-runway_sign * 740), y, 45)
    rejected_at = request('fixture-misaligned', invalid)
    rejected = result(mission, 'MAVMOOS_MISSION_RESULT', 'fixture-misaligned', 'REJECTED', rejected_at)
    need(fields(rejected)['reason'] == 'plane_runway_geometry_unqualified' and
         not any(message.get_type() == 'MISSION_COUNT' for _, message in mission.tx(rejected_at)),
         'misaligned landing plan reached the FC')
    mark = request('fixture-plan', rows)
    completed = result(mission, 'MAVMOOS_MISSION_RESULT', 'fixture-plan', 'COMPLETED', mark, 44)
    proof = mission.wait(5, lambda: mission.state('MAVMOOS_STORED_MISSION_PROOF', mark, valid='true',
                                                verification_id='fixture-plan', count=6),
                         'verified mission proof did not bind the uploaded lifecycle')
    return proof, completed


def flight_request(mission, identifier, action, proof):
    proof = fields(proof)
    return mission.notify('MAVMOOS_MISSION_CONTROL', f'schema=1,id={identifier},action={action},timeout=115,'
                          f"verification_id={proof['verification_id']},revision={proof['revision']},"
                          f"content_id={proof['content_id']}")


def direct_request(mission, identifier, action, home, north, east, height):
    latitude = home['latitude'] / 1e7 + north / 111319.5
    longitude = home['longitude'] / 1e7 + east / (111319.5 * math.cos(math.radians(latitude)))
    return mission.notify('MAVMOOS_CONTROL_REQUEST', f'schema=1,id={identifier},action={action},strategy=guided,'
                          f'lat={latitude:.7f},lon={longitude:.7f},height={height},timeout=105')


def interrupt_approach(mission, land_at, independent_mode):
    flight = mission.flight
    recovery_key = 'MAVMOOS_AUTO_RECOVERY_STATUS'

    def final_approach():
        cursor = flight.latest('MISSION_CURRENT', land_at)
        fix = flight.latest('GLOBAL_POSITION_INT', land_at)
        if cursor and fix and cursor['seq'] == 5 and 8_000 <= fix['relative_alt'] <= 50_000 and \
                ground_speed(fix) >= 13 and flight.heartbeat(AUTO, True, land_at):
            return {'cursor': cursor, 'position': fix}
        return None

    approach = mission.wait(80, final_approach, 'independent final approach cursor and airborne state missing')
    mark = now()
    takeover = pilot_switch(flight, FBWA)
    cancelled = result(mission, EXECUTION, 'fixture-land', 'CANCELLED', land_at, 6)
    need(fields(cancelled)['reason'] == 'host_execution_fenced', 'pilot takeover did not fence landing')
    marker = mission.out / 'recovery/auto-may-continue'
    need(marker.is_file(), 'interrupted landing did not retain its recovery marker')
    forbidden_commands = {176, 192, 224, 300, 400}

    def is_late_effect(record, message):
        return record.at >= takeover['mono'] and record.disposition == 'T' and (
            message.get_type() in ('MISSION_SET_CURRENT', 'SET_MODE', 'SET_POSITION_TARGET_LOCAL_NED',
                                   'SET_POSITION_TARGET_GLOBAL_INT', 'SET_ATTITUDE_TARGET') or
            getattr(message, 'command', None) in forbidden_commands)

    def no_late_command():
        return not any(is_late_effect(record, message) for record, message in mission.tx(takeover['mono']))

    ensure_for(mission, 1.0, lambda: no_late_command() and flight.heartbeat(FBWA, True),
               'old landing mode, cursor or target escaped after pilot takeover')
    mission.stop_app(kill=True)
    original = fault_tap.read_trace(mission.traces[0], flight.bridge.port, flight.bridge.peer[1])
    need(not any(is_late_effect(record, message) for record, message in packet_frames(flight.codec, original)),
         'the complete pre-restart trace contained an effect after pilot takeover')
    restarted = now()
    mission.launch_app()
    ready = mission.wait(8, lambda: mission.state(recovery_key, restarted, state='UNRESOLVED',
                                                  ready_for_handback='true', marker='present') and
                         flight.heartbeat(FBWA, True, restarted),
                         'restarted bridge did not independently observe pilot mode and retained landing fence')
    acquire = mission.notify('MAVMOOS_NAV_CONTROL', 'acquire')
    mission.wait(4, lambda: mission.runtime(acquire, permission='false'), 'helm acquired during recovery fence')
    session = fields(mission.state(recovery_key, restarted, ready_for_handback='true'))['session']
    handback = mission.notify('MAVMOOS_AUTO_RECOVERY_REQUEST',
        f'schema=1,id=fixture-approach-handback,action=handback,session={session}')
    released = result(mission, 'MAVMOOS_AUTO_RECOVERY_RESULT', 'fixture-approach-handback', 'RELEASED', handback)
    ensure_for(mission, 1.2, lambda: no_late_command() and flight.heartbeat(FBWA, True) and
               mission.runtime(restarted, permission='false') and not marker.exists(),
               'explicit handback resumed stale landing or helm guidance')
    return {'native_landing_mode': independent_mode, 'approach': approach, 'pilot_request_at': mark,
            'pilot_mode': takeover, 'cancelled': cancelled, 'restart_mode': ready,
            'handback': released, 'forbidden_bridge_effects_after_takeover': 0}



def exercise_lifecycle(mission, *, approach_takeover=False):
    flight = mission.flight
    home = flight.latest('HOME_POSITION')
    readiness = {'plane_profile': 'VERIFIED', 'plane_watchdog_started': 1}
    if mission.paths.get('modern'):
        readiness['plane_native_autoland_verified'] = 1
    mission.wait(30, lambda: mission.runtime(**readiness) and flight.heartbeat(armed=False),
                 'cold Plane did not verify its grounded watchdog')
    initial_mode = flight.heartbeat(armed=False)['custom_mode']
    _, manual = command(mission, 'fixture-manual', 'manual',
                        status='NO_OP' if initial_mode == MANUAL else 'CONFIRMED')
    mission.wait(5, lambda: flight.heartbeat(MANUAL, False), 'native Manual ground state was not observed')
    bad = mission.notify('MAVMOOS_MISSION_CONTROL', 'schema=1,id=fixture-unverified,action=takeoff,'
                         'verification_id=old,revision=1,content_id=old')
    rejected_proof = result(mission, EXECUTION, 'fixture-unverified', 'REJECTED', bad)
    proof, upload = proof_upload(mission, home)

    mission.wait(18, lambda: mission.state('MAVMOOS_PLANE_STATUS', prearm_ready='true'),
                 'selected FC prearm evidence never became ready')
    arm_at, arm = command(mission, 'fixture-arm', 'arm')
    armed = mission.wait(8, lambda: flight.heartbeat(MANUAL, True, arm_at), 'independent bridge arming missing')
    takeoff_at = flight_request(mission, 'fixture-takeoff', 'takeoff', proof)
    takeoff_auto = result(mission, EXECUTION, 'fixture-takeoff', 'CONFIRMED', takeoff_at, 16)
    takeoff_done = result(mission, EXECUTION, 'fixture-takeoff', 'COMPLETED', takeoff_at, 65)
    climb = mission.wait(12, lambda: flight.trimmed(LOITER), 'native climb and post-flight Loiter not observed')
    need(climb['position']['relative_alt'] >= 55_000 and climb['heartbeat']['mono'] > takeoff_at,
         'takeoff did not cross the independent trim band')
    _, rejected_air_disarm = command(mission, 'fixture-air-disarm', 'disarm', status='REJECTED')

    if approach_takeover:
        land_at = flight_request(mission, 'fixture-land', 'land', proof)
        result(mission, EXECUTION, 'fixture-land', 'CONFIRMED', land_at, 16)
        landing_mode = mission.wait(8, lambda: flight.heartbeat(AUTO, True, land_at),
                                    'independent AUTO landing mode missing')
        return {'final_approach_takeover': interrupt_approach(mission, land_at, landing_mode)}

    producer = mission.configure_leg(COURSE, HEIGHT)
    helm_mark = mission.acquire(COURSE, HEIGHT)
    paired_helm = mission.wait(6, lambda: mission.target(COURSE, HEIGHT, helm_mark), 'stock helm target not read back')
    release = mission.notify('MAVMOOS_NAV_CONTROL', 'release')
    mission.wait(10, lambda: flight.heartbeat(LOITER, True, release) and mission.runtime(plane_handback='CONFIRMED'),
                 'native Loiter and bridge confirmation missing after helm release')
    mission.notify('DEPLOY', 'false')

    bound_home = {'lat': home['latitude'], 'lon': home['longitude']}
    return_start = flight.latest('GLOBAL_POSITION_INT')
    return_at, native_return = command(mission, 'fixture-return', 'rtl')
    mission.wait(8, lambda: flight.heartbeat(11, True, return_at), 'independent native RTL was not observed')
    returning = mission.wait(25, lambda: (p if (p := flight.latest('GLOBAL_POSITION_INT', return_at)) and
                                         distance(bound_home, return_start) - distance(bound_home, p) >= 35 else None),
                             'native RTL did not measurably approach home')
    _, native_loiter = command(mission, 'fixture-loiter', 'loiter')
    mission.wait(12, lambda: flight.trimmed(LOITER), 'RTL did not hand back to airborne Loiter')

    start = flight.latest('GLOBAL_POSITION_INT')
    goto_at = direct_request(mission, 'fixture-direct', 'goto', home, 270, 280, 75)
    goto_done = result(mission, DIRECT, 'fixture-direct', 'COMPLETED', goto_at, 60)
    arrived = flight.latest('GLOBAL_POSITION_INT')
    need(arrived and distance(start, arrived) >= 80, 'direct waypoint did not produce independent travel')
    mission.wait(10, lambda: flight.heartbeat(LOITER, True, goto_at) and mission.runtime(plane_handback='CONFIRMED'),
                 'direct waypoint handback was not observed by both observers')
    orbit_at = direct_request(mission, 'fixture-orbit', 'hold', home, 280, 290, 75)
    orbit = result(mission, DIRECT, 'fixture-orbit', 'CONFIRMED', orbit_at, 62)
    _, speed_rejected = command(mission, 'fixture-speed-rejected', 'change_speed', 'REJECTED', ',delta=-8')
    speed_at, speed = command(mission, 'fixture-speed', 'change_speed', 'COMPLETED', ',delta=-4', 23)
    speed_air = mission.wait(4, lambda: (row if (row := flight.latest('AIRSPEED', speed_at)) and
                                       16 <= row['airspeed'] <= 20 else None), 'independent speed adjustment not measured')
    minimum_at, minimum = command(mission, 'fixture-speed-minimum', 'reset_speed_min', 'COMPLETED', timeout=23)
    minimum_air = mission.wait(4, lambda: (row if (row := flight.latest('AIRSPEED', minimum_at)) and
                                         13 <= row['airspeed'] <= 17 else None), 'independent minimum speed not measured')
    cancel_at = mission.notify('MAVMOOS_CONTROL_REQUEST', 'schema=1,id=fixture-cancel,action=cancel,target=fixture-orbit')
    cancelled = result(mission, DIRECT, 'fixture-cancel', 'COMPLETED', cancel_at)
    mission.wait(12, lambda: flight.trimmed(LOITER) and mission.runtime(plane_handback='CONFIRMED'),
                 'cancel did not restore native airborne Loiter')

    land_at = flight_request(mission, 'fixture-land', 'land', proof)
    landing_auto = result(mission, EXECUTION, 'fixture-land', 'CONFIRMED', land_at, 16)
    expected_mode = 26 if mission.paths.get('modern') else AUTO
    independent_landing_mode = mission.wait(8, lambda: flight.heartbeat(expected_mode, True, land_at),
                                            f'independent landing mode {expected_mode} was not observed')
    landing_done = result(mission, EXECUTION, 'fixture-land', 'COMPLETED', land_at, 112)
    disarmed = mission.wait(10, lambda: flight.heartbeat(MANUAL, False, land_at), 'independent disarmed MANUAL missing')
    touchdown = flight.latest('GLOBAL_POSITION_INT')
    landed = flight.latest('EXTENDED_SYS_STATE')
    need(landed and landed['landed_state'] == 1 and touchdown and ground_speed(touchdown) <= 1.5 and
         distance(bound_home, touchdown) <= 100, 'touchdown and stop did not occur inside the independent runway envelope')
    mission.wait(5, lambda: mission.state('MAVMOOS_MISSION_EXECUTION_LEASE', land_at, held='false'),
                 'landed execution lease did not clear')
    need(not (mission.out / 'recovery/auto-may-continue').exists(), 'successful runway handback retained its marker')
    _, disarm_noop = command(mission, 'fixture-disarm', 'disarm', status='NO_OP')
    return {'ground_manual': manual, 'upload': upload, 'proof': proof, 'unverified_takeoff_rejected': rejected_proof,
            'arm': arm, 'independent_armed': armed, 'takeoff_mode': takeoff_auto, 'takeoff': takeoff_done,
            'independent_climb': climb, 'airborne_disarm_rejected': rejected_air_disarm, 'stock_producer': producer,
            'stock_target': paired_helm, 'direct_waypoint': goto_done, 'direct_travel_m': distance(start, arrived),
            'native_return': native_return, 'native_loiter': native_loiter,
            'rtl_closure_m': distance(bound_home, return_start) - distance(bound_home, returning),
            'guided_orbit': orbit, 'unsafe_speed_rejected': speed_rejected, 'speed': speed, 'speed_sensor': speed_air,
            'speed_reset': minimum, 'minimum_sensor': minimum_air, 'cancel': cancelled, 'landing_mode': landing_auto,
            'landing': landing_done, 'independent_landing_mode': independent_landing_mode, 'independent_disarm': disarmed, 'touchdown': touchdown, 'landed': landed,
            'explicit_disarm_after_landing': disarm_noop}
