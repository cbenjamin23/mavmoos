"""Exercise and replay the owned stock-helm QuadPlane workflow."""

import argparse
from bisect import bisect_left
from collections import Counter
import json
import math
from pathlib import Path
import signal
import sys

from pymavlink.dialects.v20 import all as mav

TESTS = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(TESTS / name) for name in ('plane_native', 'plane_faults', '')]
import fault_tap
from flight_fixture import need, now, sha
from owned_processes import snapshot
from replay import verify_capture
from run_native import dataflash

LIVE_CASES = ('live', 'live-auto')
LIVE_PARAMETERS = {
    'Q_GUIDED_MODE': 0, 'Q_FRAME_CLASS': 1, 'Q_FRAME_TYPE': 1, 'Q_TILT_ENABLE': 0,
    'Q_TAILSIT_ENABLE': 0, 'Q_RTL_MODE': 0, 'Q_ASSIST_SPEED': 18,
    'FS_GCS_ENABL': 1, 'FS_LONG_ACTN': 1, 'FS_LONG_TIMEOUT': 5,
    'RTL_AUTOLAND': 0, 'RTL_ALTITUDE': 65, 'RTL_RADIUS': 0, 'WP_LOITER_RAD': 80,
    'AIRSPEED_CRUISE': 22, 'AIRSPEED_MIN': 13, 'AIRSPEED_MAX': 35,
    'ARSPD_PRIMARY': 0, 'ARSPD2_TYPE': 0,
}
AUTO, RTL, LOITER, GUIDED, QLOITER, QRTL = 10, 11, 12, 15, 19, 21
MC, FW, TO_FW = mav.MAV_VTOL_STATE_MC, mav.MAV_VTOL_STATE_FW, mav.MAV_VTOL_STATE_TRANSITION_TO_FW
AIR, GROUND = mav.MAV_LANDED_STATE_IN_AIR, mav.MAV_LANDED_STATE_ON_GROUND
NAV_CONTROL, HELM = 'MAVMOOS_NAV_CONTROL', 'MAVMOOS_HELM_STATUS'
VALUES = {'DESIRED_HEADING': 90., 'DESIRED_SPEED': 22., 'DESIRED_ALTITUDE': 65.}


def fields(row):
    return dict(item.split('=', 1) for item in row['value'].split(',') if '=' in item)


def delta(start, end):
    scale = 111194.93e-7
    north = (end['lat'] - start['lat']) * scale
    east = (end['lon'] - start['lon']) * scale * math.cos(math.radians(start['lat'] * 1e-7))
    return north, east


def speed(row):
    return math.hypot(row['vx'], row['vy']) / 100


def within_course(row):
    heading = math.degrees(math.atan2(row['vy'], row['vx']))
    return abs(math.remainder(heading - VALUES['DESIRED_HEADING'], 360)) <= 28


def airspeed(flight, after):
    row = flight.latest('AIRSPEED', after, max_age=.6)
    return row if row and row['id'] == 0 and row['flags'] & 3 == 2 and 16 <= row['airspeed'] <= 30 else None


def observe(flight, mode, phase, after=0, minimum=0, velocity=0):
    row = flight.observation(after=after, mode=mode, armed=True, phase=phase, landed=AIR)
    return row if row and row['position']['relative_alt'] >= minimum * 1000 and speed(row['position']) >= velocity else None


def stock(flight, after):
    samples = {key: flight.probe.last.get(key) for key in VALUES}
    return samples if all(row and row['source'] == 'pHelmIvP' and row['mono'] > after and
                          row['value'] == VALUES[key] for key, row in samples.items()) else None


def wait_stock(flight, after):
    return flight.wait(6, lambda: stock(flight, after), 'stock helm did not issue the selected scalars')


def wait_status(flight, after, why, **expected):
    return flight.wait(7, lambda: flight.probe.runtime(after, **expected), why)


def hold_no_grant(flight, counter, dwell, mode):
    at = now()

    def check():
        current = flight.probe.runtime(at, permission='false', fc_mode=str(mode))
        hb = flight.heartbeat(mode, True)
        need(flight.probe.navigation_count() == counter, 'old stock decisions sent a new target')
        return current if now() >= at + dwell and hb else None

    status = flight.wait(dwell + 2, check, 'quiet native interval did not preserve operator handback')
    return {'from': at, 'through': status['mono'], 'mode': mode, 'nav_count': counter}


def refuse(flight, label, phase_sample):
    counter = flight.probe.navigation_count()
    at = flight.probe.post(NAV_CONTROL, 'acquire')
    row = flight.wait(4, lambda: flight.probe.match(HELM, at, state='LATCHED', reason='acquisition_rejected'),
                      f'{label} did not explicitly reject live acquisition')
    status = wait_status(flight, at, f'{label} conferred unexpected navigation permission', permission='false')
    need(flight.probe.navigation_count() == counter, f'{label} admitted a live target')
    return {'request_at': at, 'phase_sample': phase_sample, 'rejected': row, 'runtime': status}


def loiter_ready(flight, after):
    def ready():
        flight_state = observe(flight, LOITER, FW, after, minimum=35, velocity=16)
        sensor = airspeed(flight, after)
        bridge = flight.probe.runtime(after, permission='false', fc_mode=str(LOITER), plane_profile='VERIFIED',
                                      plane_watchdog='ENABLED', plane_watchdog_started='1')
        return flight_state | {'airspeed': sensor, 'runtime': bridge} if flight_state and sensor and bridge else None

    return flight.wait(65, ready, 'qualified fresh forward native Loiter never became ready')


def acquire(flight, label):
    previous = flight.probe.navigation_count()
    previous_verified = int(fields(flight.probe.last['MAVMOOS_RUNTIME_STATUS'])['plane_targets_verified'])
    at = flight.probe.post(NAV_CONTROL, 'acquire')
    guided = flight.wait(7, lambda: observe(flight, GUIDED, FW, at, minimum=35, velocity=15),
                         f'{label} did not enter native Guided with fresh stable forward flight')
    scalars = wait_stock(flight, at)

    def confirm():
        row = flight.probe.runtime(at, permission='true', fc_mode=str(GUIDED), plane_guided_confirmed='1')
        if row and flight.probe.navigation_count() > previous and int(fields(row)['plane_targets_verified']) > previous_verified:
            return row

    result = flight.wait(12, confirm, f'{label} did not send and confirm a new FC position target')
    return {'label': label, 'requested': at, 'guided': guided, 'scalars': scalars, 'runtime': result}


def fresh_target_window(flight):
    count = int(fields(flight.probe.last['MAVMOOS_RUNTIME_STATUS'])['plane_targets_verified'])

    def ready():
        row = flight.probe.runtime(permission='true', plane_target='VERIFIED')
        return row if row and int(fields(row)['plane_targets_verified']) > count else None

    return flight.wait(5, ready, 'normal handoff needed a newly verified native target')


def leg(flight, at):
    consecutive = []

    def advance():
        row = flight.latest('GLOBAL_POSITION_INT', at, max_age=.4)
        if not row or (consecutive and row['mono'] <= consecutive[-1]['mono']):
            return None
        state = observe(flight, GUIDED, FW, at, minimum=35, velocity=16)
        if (not state or not airspeed(flight, at) or not within_course(row) or speed(row) > 30 or
                abs(row['relative_alt'] / 1000 - VALUES['DESIRED_ALTITUDE']) > 12):
            consecutive.clear()
            return None
        consecutive.append(row)
        progress = delta(consecutive[0], row)[1]
        dwell = row['mono'] - consecutive[0]['mono']
        if dwell >= 3.5 and progress >= 60:
            return {'start': consecutive[0], 'end': row, 'samples': len(consecutive),
                    'duration_s': dwell, 'east_m': progress}
        return None

    return flight.wait(40, advance, 'stock eastward leg did not physically track course, speed and altitude')


def native_handback(flight, after):
    hb = flight.wait(9, lambda: flight.heartbeat(LOITER, True, after), 'native Loiter handback not observed')
    ready = loiter_ready(flight, hb['mono'])
    return {'heartbeat': hb, 'ready': ready}


def phase_prelude(flight):
    report, probe = flight.report, flight.probe
    report['arm'] = flight.request('arm', 'arm', 'COMPLETED')[1]
    _, report['pilot_guided_takeoff_mode'] = flight.operator_mode(GUIDED, True)
    command = mav.MAV_CMD_NAV_TAKEOFF
    at = flight.transmit(flight.encoder.command_long_encode(1, 1, command, 0, 0, 0, 0, 0, 0, 0, 60))
    report['takeoff_requested_at'] = at

    def takeoff_ack():
        row = flight.operator.latest.get('COMMAND_ACK')
        if (row and row['mono'] > at and row['command'] == command and row['result'] == mav.MAV_RESULT_ACCEPTED and
                (row['target_system'], row['target_component']) == (249, 190)):
            return row

    report['pilot_takeoff_ack'] = flight.wait(5, takeoff_ack, 'owned native guided takeoff was not acknowledged')
    report['native_hover_climb'] = flight.wait(45, lambda: observe(flight, GUIDED, MC, at, minimum=56),
                                               'native takeoff did not reach 56 m in multicopter phase')
    at, _ = flight.operator_mode(QLOITER, True)
    hover = flight.wait(8, lambda: observe(flight, QLOITER, MC, at, minimum=52), 'native airborne QLoiter was not observed')
    report['hover_refusal'] = refuse(flight, 'stable multicopter', hover)
    need(flight.heartbeat(QLOITER, True), 'hover refusal changed the independently observed native mode')

    # Put the operator request on serial2 and observe the transition separately.
    # Wait for the earlier phase report as well as the fresh pilot mode.
    at, _ = flight.operator_mode(LOITER, True)
    transitioning = flight.wait(9, lambda: observe(flight, LOITER, TO_FW, at, minimum=35),
                                'native forward transition not witnessed')
    report['transition_refusal'] = refuse(flight, 'forward transition', transitioning)
    report['forward_ready'] = loiter_ready(flight, at)
    report['no_cross_phase_grant'] = hold_no_grant(flight, probe.navigation_count(), 1.25, LOITER)
    need(probe.navigation_count() == 0, 'a target crossed an unowned phase boundary')


def normal_handoff(flight):
    probe = flight.probe
    verified = fresh_target_window(flight)
    at = probe.post('DEPLOY', 'false')
    paused = flight.wait(2, lambda: probe.match(HELM, at, state='PAUSED', reason='normal_handoff'),
                         'stock all-stop did not enter a bounded handoff')
    counter = probe.navigation_count()
    restore_at = probe.post('DEPLOY', 'true')
    resumed = flight.wait(1.5, lambda: probe.match(HELM, restore_at, state='ACTIVE', reason='resumed_after_normal_handoff'),
                          'short normal handoff did not resume the same navigation permission')
    need(resumed['mono'] - paused['mono'] < 1.0, 'short normal handoff exceeded its configured one-second limit')
    runtime = flight.wait(5, lambda: probe.runtime(restore_at, permission='true', fc_mode=str(GUIDED)) and
                          probe.navigation_count() > counter and probe.last['MAVMOOS_RUNTIME_STATUS'],
                          'resumed stock helm did not issue fresh Guided target')
    return {'verified': verified, 'paused': paused, 'restore_at': restore_at, 'resumed': resumed, 'runtime': runtime}


def expired_handoff(flight):
    probe = flight.probe
    verified = fresh_target_window(flight)
    at = probe.post('DEPLOY', 'false')
    expired = flight.wait(5, lambda: probe.match(HELM, at, state='LATCHED', reason='handoff_window_expired'),
                          'long stock all-stop did not expire the original handoff')
    handback = native_handback(flight, expired['mono'])
    counter = probe.navigation_count()
    restore_at = probe.post('DEPLOY', 'true')
    wait_stock(flight, restore_at)
    quiet = hold_no_grant(flight, counter, 1.25, LOITER)
    return {'verified': verified, 'expired': expired, 'handback': handback, 'quiet': quiet}


def silence(flight):
    probe = flight.probe
    pid = flight.helm.pid
    owner = snapshot([pid]).get(pid)
    need(owner and str(flight.config) in owner['command'] and str(flight.paths['helm']) in owner['command'],
         'stock helm process identity failed')
    start = flight.event('stock_helm_stopped', process=owner)
    flight.helm.send_signal(signal.SIGSTOP)
    try:
        # Withdrawal and the following handback can arrive in one probe read.
        # Read the recorded transition instead of assuming it stays the latest state.
        with (flight.out / 'moos.jsonl').open() as trace:
            withdrawn = False

            def stale():
                nonlocal withdrawn
                for line in iter(trace.readline, ''):
                    event = json.loads(line)
                    if (event.get('key') == 'MAVMOOS_RUNTIME_STATUS' and
                            event['source'] == 'iMAVLink' and event['mono'] >= start):
                        status = fields(event)
                        withdrawn |= (status.get('permission') == 'false' and
                                      status.get('reason') == 'plane_producer_or_acquisition_expired')
                row = probe.match(HELM, start, state='LATCHED')
                return row if row and (fields(row)['reason'] == 'helm_input_stale_or_incomplete' or
                                       (fields(row)['reason'] == 'quadplane_host_authority_withdrawn' and withdrawn)) else None

            stopped = flight.wait(5, stale, 'stock helm process silence did not expire steering')
        handback = native_handback(flight, stopped['mono'])
    finally:
        need(snapshot([pid]).get(pid) == owner, 'stopped stock helm process identity changed')
        flight.helm.send_signal(signal.SIGCONT)
    resumed = flight.event('stock_helm_resumed', process=owner)
    wait_stock(flight, resumed)
    quiet = hold_no_grant(flight, probe.navigation_count(), 1.25, LOITER)
    return {'stopped_at': start, 'stale': stopped, 'handback': handback, 'resumed_at': resumed, 'quiet': quiet}


def link_loss(flight):
    probe = flight.probe
    control = flight.out / 'original.tap-mode'
    drop_at = fault_tap.set_mode(control, 'drop_rx')[1]
    flight.event('drop_app_rx', after=drop_at)
    revoked = wait_status(flight, drop_at, 'FC observation loss did not revoke live authority', permission='false')
    rtl = flight.wait(12, lambda: flight.heartbeat(RTL, True, drop_at),
                      'independent FC did not enter native watchdog RTL')
    resume_at = fault_tap.set_mode(control, 'pass')[1]
    flight.event('restore_app_rx', after=resume_at)
    wait_status(flight, resume_at, 'fresh independent RTL failed to reach recovered bridge',
                permission='false', fc_mode=str(RTL), heartbeat_valid='1')
    at, pilot_loiter = flight.operator_mode(LOITER, True)
    ready = loiter_ready(flight, at)
    quiet = hold_no_grant(flight, probe.navigation_count(), 1.25, LOITER)
    return {'drop_at': drop_at, 'revoked': revoked, 'rtl': rtl, 'restore_at': resume_at,
            'pilot_loiter': pilot_loiter, 'ready': ready, 'quiet': quiet}


def pilot_takeover(flight):
    at, hb = flight.operator_mode(QLOITER, True)
    revoked = wait_status(flight, hb['mono'], 'pilot QLoiter did not revoke live authority',
                          permission='false', fc_mode=str(QLOITER))
    hover = flight.wait(18, lambda: observe(flight, QLOITER, MC, at, minimum=35),
                        'pilot takeover did not attain the independent multicopter phase')
    denied = refuse(flight, 'pilot multicopter', hover)
    quiet = hold_no_grant(flight, flight.probe.navigation_count(), .85, QLOITER)
    _, later = flight.operator_mode(LOITER, True)
    ready = loiter_ready(flight, later['mono'])
    still_quiet = hold_no_grant(flight, flight.probe.navigation_count(), 1.25, LOITER)
    return {'pilot_at': at, 'heartbeat': hb, 'revoked': revoked, 'hover': hover, 'denied': denied,
            'quiet_hover': quiet, 'later_loiter': later, 'ready': ready, 'quiet_loiter': still_quiet}


def touchdown(flight, home):
    at, hb = flight.operator_mode(QRTL, True)

    def ground():
        row = flight.observation(after=at, mode=QRTL, armed=False, phase=MC, landed=GROUND)
        if not row:
            return None
        position = row['position']
        origin = {'lat': home['latitude'], 'lon': home['longitude']}
        distance = math.hypot(*delta(origin, position))
        if distance <= 40 and speed(position) <= 1 and abs(position['relative_alt']) <= 2500:
            return row | {'distance_m': distance}

    return {'qrtl': hb, 'ground': flight.wait(150, ground, 'native QRTL did not land and disarm near home')}


def run_live(flight, home):
    flight.start_app()
    report, probe = flight.report, flight.probe
    deploy = probe.post('DEPLOY', 'true')
    report['initial_stock_inputs'] = wait_stock(flight, deploy)
    report['ground_refusal'] = refuse(flight, 'disarmed ground QLoiter', flight.heartbeat(QLOITER, False))
    phase_prelude(flight)
    report['acquisitions'] = [acquire(flight, 'initial explicit forward grant')]
    report['stock_leg'] = leg(flight, report['acquisitions'][0]['guided']['position']['mono'])
    report['short_handoff'] = normal_handoff(flight)
    report['long_handoff'] = expired_handoff(flight)
    report['acquisitions'].append(acquire(flight, 'after expired handoff'))
    report['producer_silence'] = silence(flight)
    report['acquisitions'].append(acquire(flight, 'after producer returned'))
    report['link_loss'] = link_loss(flight)
    report['acquisitions'].append(acquire(flight, 'after link recovered'))
    report['pilot_takeover'] = pilot_takeover(flight)
    report['acquisitions'].append(acquire(flight, 'after pilot handback'))
    release = probe.post(NAV_CONTROL, 'release')
    report['explicit_release'] = native_handback(flight, release)
    report['after_release'] = hold_no_grant(flight, probe.navigation_count(), 1.25, LOITER)
    report['landing'] = touchdown(flight, home)


def witness_auto_fence(flight, after):
    at = flight.probe.post('DEPLOY', 'true')
    scalars = wait_stock(flight, at)
    phase = flight.wait(4, lambda: observe(flight, AUTO, FW, after),
                        'fresh native forward AUTO did not persist until the stock attempt')
    need(flight.marker.is_file(), 'stock attempt lacked the durable native execution lease')
    report = refuse(flight, 'retained mission', phase)
    report['stock_scalars'] = scalars
    report['after_refusal_auto'] = flight.wait(4, lambda: observe(flight, AUTO, FW, report['rejected']['mono']),
                                               'refused stock request disturbed native AUTO')
    report['retained'] = wait_status(flight, at, 'native lease did not stay held after live acquire refusal',
                                     mission_execution_held='true', permission='false')
    flight.report['auto_live_fence'] = report


def wire(out, report):
    ports = report['ports']
    trace = out / report['tap_traces']['original']
    for row in fault_tap.read_trace(trace, ports['bridge'], ports['bridge_peer']):
        allowed = ('R', 'T', 'i', 'h', 'q') if report['case'] in ('hover-adapter-loss', 'hover-survey', 'hover-mode-stress') else ('R', 'T', 'i')
        need(row.disposition in allowed, 'unexpected test tap disposition')
        decoder = mav.MAVLink(None)
        frames = decoder.parse_buffer(row.data) or []
        need(frames and sum(len(frame.get_msgbuf()) for frame in frames) == len(row.data),
             'tap contained incomplete MAVLink data')
        if row.disposition == 'T':
            need(len(frames) == 1 and (frames[0].get_srcSystem(), frames[0].get_srcComponent()) == (245, 191),
                 'unexpected sender on the owned bridge route')
        for msg in frames:
            yield row.at, row.disposition, msg


def archived_rows(out, name):
    return [json.loads(line) for line in (out / name).read_text().splitlines()]


def audit_live(out, report):
    monitor_count = verify_capture(out, 'monitor')
    monitor = archived_rows(out, 'monitor-rx.jsonl')
    rows = list(wire(out, report))
    tx = [(at, msg) for at, side, msg in rows if side == 'T']
    targets = [(at, msg) for at, msg in tx if msg.get_type() == 'COMMAND_INT' and
               msg.command == mav.MAV_CMD_DO_REPOSITION]
    modes = [(at, int(msg.param2)) for at, msg in tx if msg.get_type() == 'COMMAND_LONG' and
             msg.command == mav.MAV_CMD_DO_SET_MODE]
    for _, message in targets:
        need((message.target_system, message.target_component, message.frame, message.current, message.autocontinue) ==
             (1, 1, mav.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 0, 0) and
             (message.param1, message.param2, message.param3, message.param4) == (-1, 0, 0, 0) and
             abs(message.z - VALUES['DESIRED_ALTITUDE']) < .02, 'live target wire shape differs')
    if report['case'] == 'live-auto':
        refusal = report['auto_live_fence']
        trace = archived_rows(out, 'moos.jsonl')
        request = [row for row in trace if row['kind'] == 'request' and row['key'] == NAV_CONTROL and
                   row['value'] == 'acquire' and refusal['request_at'] <= row['mono'] <= refusal['rejected']['mono']]
        need(not targets and [mode for _, mode in modes] == [QLOITER] and
             refusal['after_refusal_auto']['state']['vtol_state'] == FW and
             fields(refusal['retained'])['mission_execution_held'] == 'true' and len(request) == 1 and
             all(row in trace and row['source'] == 'pHelmIvP' for row in refusal['stock_scalars'].values()),
             'live authority or mode effects crossed retained native AUTO')
        commands = Counter(message.command for _, message in tx
                           if message.get_type() in ('COMMAND_INT', 'COMMAND_LONG'))
        ground = report['touchdown']
        home = report['home']
        touchdown_error = math.hypot(*delta({'lat': home['latitude'], 'lon': home['longitude']},
                                           ground['position']))
        selected = ('hover', 'forward', 'return_to_hover', 'touchdown')
        need(commands[mav.MAV_CMD_DO_SET_MISSION_CURRENT] == commands[mav.MAV_CMD_MISSION_START] == 1 and
             all(row in monitor for name in selected for row in report[name].values()) and
             all(row in monitor for row in refusal['after_refusal_auto'].values()) and
             ground['state']['landed_state'] == GROUND and not ground['heartbeat']['base_mode'] & 128 and
             touchdown_error <= 40 and fields(report['release'])['status'] == 'COMPLETED' and
             {AUTO, QLOITER} <= {row['ModeNum'] for row in dataflash(out)['modes']},
             'native cursor, takeoff, touchdown or handback did not replay')
        return {'monitor_crc_frames': monitor_count, 'bridge_transmissions': len(tx), 'live_targets': 0,
                'takeoff_cursor_start_closeout': True, 'native_touchdown_distance_m': touchdown_error}

    # Replay the app's own observation route in read-before-send order.
    phase_at = None
    observed_mode = None
    for at, side, message in rows:
        if side == 'R' and (message.get_srcSystem(), message.get_srcComponent()) == (1, 1):
            if message.get_type() == 'EXTENDED_SYS_STATE':
                phase_at = at if message.vtol_state == FW and message.landed_state == AIR else None
            elif message.get_type() == 'HEARTBEAT':
                observed_mode = message.custom_mode
        if side == 'T' and message.get_type() == 'COMMAND_INT' and message.command == mav.MAV_CMD_DO_REPOSITION:
            need(phase_at is not None and at - phase_at <= 1.0 and observed_mode == GUIDED,
                 'target preceded stable forward-phase evidence or followed its retirement')

    phase = [row for row in monitor if row['mavpackettype'] == 'EXTENDED_SYS_STATE']
    readbacks = [row for row in monitor if row['mavpackettype'] == 'POSITION_TARGET_GLOBAL_INT']
    readback_times = [row['mono'] for row in readbacks]
    moos = archived_rows(out, 'moos.jsonl')
    home_msl = report['home']['altitude'] / 1000
    matched = []
    for at, message in targets:
        start = bisect_left(readback_times, at)
        pair = next((row for row in readbacks[start:] if row['mono'] <= at + 1.2 and
                     (row['lat_int'], row['lon_int']) == (message.x, message.y) and
                     abs(row['alt'] - home_msl - message.z) <= .35), None)
        if pair:
            matched.append((at, pair))
    acquisitions = report['acquisitions']
    need(len(matched) >= len(acquisitions) and [mode for _, mode in modes].count(GUIDED) == len(acquisitions),
         'missing actual addressed mode commands or independent accepted position targets')
    first_grant = acquisitions[0]['requested']
    need(not any(at < first_grant for at, _ in targets + modes), 'live effect before initial explicit forward grant')
    for index, item in enumerate(acquisitions):
        at = item['requested']
        limit = acquisitions[index+1]['requested'] if index + 1 < len(acquisitions) else report['landing']['qrtl']['mono']
        need(any(at < when < limit for when, _ in matched) and
             item['guided']['state'] in phase and all(row in moos and row['mono'] > at for row in item['scalars'].values()),
             f"missing independent forward state or fresh stock input for {item['label']}")

    windows = [report['no_cross_phase_grant'], report['long_handoff']['quiet'], report['producer_silence']['quiet'],
               report['link_loss']['quiet'], report['pilot_takeover']['quiet_hover'],
               report['pilot_takeover']['quiet_loiter'], report['after_release']]
    for window in windows:
        need(not any(window['from'] <= when <= window['through'] for when, _ in targets + modes),
             'direct wire showed old effects during an unowned quiet window')
    paused = report['short_handoff']
    need(not any(paused['paused']['mono'] <= when <= paused['restore_at'] for when, _ in targets),
         'stock handoff continued to transmit a target while paused')
    for name, expected in (('hover_refusal', MC), ('transition_refusal', TO_FW)):
        witnessed = report[name]['phase_sample']['state']
        need(witnessed in phase and witnessed['vtol_state'] == expected, f'{name} lacked independent phase evidence')
    takeover = report['pilot_takeover']
    need(takeover['hover']['state'] in phase and takeover['hover']['state']['vtol_state'] == MC and
         takeover['ready']['state'] in phase and takeover['ready']['state']['vtol_state'] == FW and
         not any(takeover['revoked']['mono'] <= when < acquisitions[-1]['requested'] for when, _ in targets + modes),
         'old control crossed pilot multicopter takeover and fixed-wing return')

    loss = report['link_loss']
    rtl_at = loss['rtl']['mono']
    last_watch = max(at for at, msg in tx if at < rtl_at and msg.get_type() == 'HEARTBEAT')
    gap = rtl_at - last_watch
    need(any(side == 'i' and loss['drop_at'] <= at < loss['restore_at'] for at, side, _ in rows) and
         4.6 <= gap <= 7 and not any(loss['revoked']['mono'] <= at <= rtl_at for at, _ in targets) and
         any(row['ModeNum'] == RTL and row['Rsn'] == 5 for row in dataflash(out)['modes']),
         'native watchdog lacked directional fault, bounded heartbeat gap or DataFlash failsafe attribution')
    leg = report['stock_leg']
    need(leg['start'] in monitor and leg['end'] in monitor and delta(leg['start'], leg['end'])[1] >= 60 and
         within_course(leg['start']) and within_course(leg['end']), 'independent stock leg did not replay')
    landing = report['landing']['ground']
    need(all(landing[key] in monitor for key in ('heartbeat', 'state', 'position')) and
         landing['state']['vtol_state'] == MC and landing['state']['landed_state'] == GROUND and
         not landing['heartbeat']['base_mode'] & 128 and landing['distance_m'] <= 40,
         'independent native multicopter touchdown did not replay')
    return {'monitor_crc_frames': monitor_count, 'bridge_transmissions': len(tx), 'live_targets': len(targets),
            'paired_readbacks': len(matched), 'bridge_set_modes': dict(Counter(mode for _, mode in modes)),
            'quiet_windows': len(windows), 'watchdog_gap_s': gap, 'eastward_leg_m': leg['east_m'],
            'landing_distance_m': landing['distance_m']}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('receipt', type=Path)
    args = parser.parse_args()
    receipt = args.receipt.resolve(strict=True)
    report = json.loads(receipt.read_text())
    for name, digest in report['files'].items():
        path = receipt.parent / name
        need(path.resolve().is_relative_to(receipt.parent) and path.is_file() and sha(path) == digest,
             f'archived file digest mismatch {name}')
    print(json.dumps(audit_live(receipt.parent, report), indent=2))
