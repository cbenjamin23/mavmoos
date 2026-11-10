"""Stock-helm hover and native phase handoffs in the pinned QuadPlane."""
import argparse
from bisect import bisect_left
import json
from pathlib import Path
import signal
import socket
import struct

from live_helm import (
    AIR, FW, GROUND, GUIDED, HELM, LOITER, MC, NAV_CONTROL, QLOITER, QRTL, TO_FW,
    archived_rows, delta, fields, hold_no_grant, observe, refuse, speed, wire,
)
from flight_fixture import exact_rc, inventory, need, now
from owned_processes import snapshot
from pymavlink.dialects.v20 import all as mav
from replay import verify_capture
import fault_tap

HOVER_CASES = ('hover', 'hover-forward-takeover', 'hover-adapter-loss', 'hover-survey', 'hover-mode-stress')
HOVER_PARAMETERS = {'Q_GUIDED_MODE': 1, 'Q_RTL_MODE': 3, 'SCR_ENABLE': 1,
                    'FENCE_ENABLE': 0, 'FENCE_AUTOENABLE': 0}
HOVER_VALUES = {'DESIRED_HEADING': 90., 'DESIRED_SPEED': 2., 'DESIRED_ALTITUDE': 65.}
FBWA = 5


def stock(flight, after):
    def matched():
        samples = {key: flight.probe.last.get(key) for key in HOVER_VALUES}
        return samples if all(row and row['source'] == 'pHelmIvP' and row['mono'] > after and
                              row['value'] == HOVER_VALUES[key] for key, row in samples.items()) else None

    return flight.wait(7, matched, 'stock helm did not publish the three hover decisions')


def hold_ready(flight, after):
    settled_at = None

    def ready():
        nonlocal settled_at
        state = observe(flight, QLOITER, MC, after, minimum=35)
        bridge = flight.probe.runtime(after, fc_mode=str(QLOITER), permission='false', plane_profile='VERIFIED',
                                      plane_watchdog_started='1', quadplane_hover_adapter='READY')
        if not state or not bridge or speed(state['position']) > 3 or abs(state['position']['vz']) > 100:
            settled_at = None
            return None
        if settled_at is None:
            settled_at = state['position']['mono']
        if state['position']['mono'] - settled_at >= .65:
            return state | {'runtime': bridge, 'settled_at': settled_at}

    return flight.wait(35, ready, 'native QLoiter and verified hover adapter did not become available')


def neutral_pilot(flight, takeoff):
    port = flight.ports['rc']
    proof = inventory(flight.simulator)
    need(exact_rc(proof, port) and flight.heartbeat(GUIDED, True, takeoff['heartbeat']['mono']),
         'neutral pilot input requires the exact simulator RC endpoint and airborne Guided evidence')
    # The simulator defaults RC3 to 1000. A centered simulated pilot can hold
    # native QLoiter altitude while the independent mode and phase gates settle.
    channels = (1500, 1500, 1500, 1500, 1800, 1000, 1000, 1800)
    payload = struct.pack('<8H', *channels)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as pilot:
        pilot.bind(('127.0.0.1', 0))
        at = now()
        need(pilot.sendto(payload, ('127.0.0.1', port)) == len(payload), 'private simulator RC write failed')
    flight.event('neutral_simulated_pilot', tx_mono=at, pwm=channels, inventory=proof)

    def readback():
        row = flight.latest('RC_CHANNELS', at)
        return row if row and row['chan3_raw'] == 1500 else None

    return {'sent_at': at, 'channels': channels,
            'readback': flight.wait(3, readback, 'independent FC telemetry did not read neutral RC3')}


def acquire(flight, *, fixed_inputs=True):
    before = flight.probe.navigation_count()
    at = flight.probe.post(NAV_CONTROL, 'acquire')
    guided = flight.wait(9, lambda: observe(flight, GUIDED, MC, at, minimum=35),
                         'acquisition did not settle in stable native Guided hover')
    scalars = stock(flight, guided['heartbeat']['mono']) if fixed_inputs else None
    confirmed = flight.wait(8, lambda: flight.probe.runtime(at, permission='true', fc_mode=str(GUIDED),
                            plane_guided_confirmed='1', plane_target='VERIFIED') and
                            flight.probe.navigation_count() > before and
                            flight.probe.last['MAVMOOS_RUNTIME_STATUS'],
                            'fresh stock hover did not receive independent FC target confirmation')
    return {'request_at': at, 'guided': guided, 'scalars': scalars, 'confirmed': confirmed}


def height_reached(position, up, climb):
    if climb is not None:
        return up >= climb
    return abs(position['relative_alt'] / 1000 - HOVER_VALUES['DESIRED_ALTITUDE']) <= 1.5


def move(flight, acquisition, climb=None):
    start = acquisition['guided']['position']
    at = acquisition['confirmed']['mono']

    def check():
        state = observe(flight, GUIDED, MC, at, minimum=35)
        if not state:
            return None
        north, east = delta(start, state['position'])
        up = (state['position']['relative_alt'] - start['relative_alt']) / 1000
        if east >= 7 and abs(north) <= 4 and speed(state['position']) <= 5 and \
                height_reached(state['position'], up, climb):
            return {'start': start, 'end': state, 'east_m': east, 'north_m': north, 'up_m': up}
        return None

    return flight.wait(18, check, 'stable hover did not physically move east at its allowed height')


def silence(flight):
    pid = flight.helm.pid
    owner = snapshot([pid]).get(pid)
    need(owner and str(flight.config) in owner['command'] and str(flight.paths['helm']) in owner['command'],
         'stock helm ownership not established')
    stopped_at = flight.event('hover_helm_stopped', process=owner)
    flight.helm.send_signal(signal.SIGSTOP)
    try:
        hold = hold_ready(flight, stopped_at)
        loss = flight.wait(3, lambda: flight.probe.match(HELM, stopped_at, state='LATCHED'),
                           'producer silence did not retire old helm authority')
    finally:
        need(snapshot([pid]).get(pid) == owner, 'stopped stock helm identity changed')
        flight.helm.send_signal(signal.SIGCONT)
    resumed = flight.event('hover_helm_resumed', process=owner)
    stock(flight, resumed)
    quiet = hold_no_grant(flight, flight.probe.navigation_count(), 1.2, QLOITER)
    return {'stopped_at': stopped_at, 'hold': hold, 'latch': loss, 'resumed_at': resumed, 'quiet': quiet}


def adapter_loss(flight, *, fixed_inputs=True):
    control = flight.out / 'original.tap-mode'
    hold_at = fault_tap.set_mode(control, 'hold_hover')[1]

    def pending():
        return next((at for at, side, msg in wire(flight.out, flight.report)
                     if at >= hold_at and side == 'h' and msg.get_type() == 'TUNNEL'), None)

    pending_at = flight.wait(3, pending, 'no pending hover target captured at the socket')
    lost_at = fault_tap.set_mode(control, 'drop_hover')[1]
    hold = hold_ready_after_adapter_loss(flight, lost_at)
    count = flight.probe.navigation_count()
    restored_at = fault_tap.set_mode(control, 'pass')[1]
    if fixed_inputs:
        stock_after = stock(flight, restored_at)
    else:
        def fresh_stock():
            rows = [flight.probe.last.get(key) for key in HOVER_VALUES]
            return rows if all(row and row['source'] == 'pHelmIvP' and
                               row['mono'] > restored_at for row in rows) else None
        stock_after = flight.wait(5, fresh_stock, 'stock route scalars did not return after adapter fault')
    quiet = hold_no_grant(flight, count, 1.2, QLOITER)
    return {'pending_at': pending_at, 'lost_at': lost_at, 'hold': hold,
            'restored_at': restored_at, 'stock_after': stock_after, 'quiet': quiet}


def hold_ready_after_adapter_loss(flight, after):
    def ready():
        hold = observe(flight, QLOITER, MC, after, minimum=35)
        runtime = flight.probe.runtime(after, permission='false', fc_mode=str(QLOITER))
        return {'native': hold, 'runtime': runtime} if hold and runtime else None

    return flight.wait(6, ready, 'adapter loss did not retire helm and return to observed native QLoiter')



def run_hover(flight):
    flight.start_app()
    report, probe = flight.report, flight.probe
    forward_takeover = report['case'] == 'hover-forward-takeover'
    deploy = probe.post('DEPLOY', 'true')
    if report['case'] not in ('hover-survey', 'hover-mode-stress'):
        report['stock_ready'] = stock(flight, deploy)
    report['ground_ready'] = flight.wait(12, lambda: probe.runtime(
        fc_mode=str(QLOITER), armed='0', position_valid='1', home_valid='1', landed_valid='1',
        quadplane_phase=str(MC), plane_profile='VERIFIED', auto_recovery_state='READY'),
        'owned disarmed QuadPlane startup observations were not ready')
    report['ground_refusal'] = refuse(flight, 'disarmed hover', flight.heartbeat(QLOITER, False))
    report['arm'] = flight.request('arm', 'arm', 'COMPLETED')[1]
    _, report['pilot_guided_takeoff_mode'] = flight.operator_mode(GUIDED, True)
    at = flight.transmit(flight.encoder.command_long_encode(1, 1, mav.MAV_CMD_NAV_TAKEOFF,
                                                            0, 0, 0, 0, 0, 0, 0, 60))
    report['takeoff'] = flight.wait(45, lambda: observe(flight, GUIDED, MC, at, minimum=58),
                                    'native takeoff did not reach stable hover')
    report['neutral_pilot'] = neutral_pilot(flight, report['takeoff'])
    at, _ = flight.operator_mode(QLOITER, True)
    report['hover_entry'] = hold_ready(flight, at)
    if report['case'] == 'hover-survey':
        from hover_survey import survey
        report['autonomy'] = survey(flight)
        return
    if report['case'] == 'hover-mode-stress':
        from hover_mode_stress import fly
        report['stress'] = fly(flight)
        return
    report['initial_acquisition'] = acquire(flight)
    if report['case'] == 'hover-adapter-loss':
        report['adapter_loss'] = adapter_loss(flight)
        report['native_return'] = {'hold': hold_ready(flight, report['adapter_loss']['quiet']['through'])}
    else:
        report['initial_motion'] = move(flight, report['initial_acquisition'], climb=1.3)
        if forward_takeover:
            report['active_before_forward'] = flight.wait(3, lambda: probe.runtime(
                report['initial_motion']['end']['position']['mono'], permission='true', fc_mode=str(GUIDED),
                plane_target='VERIFIED'), 'hover permission was absent before pilot transition')
        else:
            report['producer_silence'] = silence(flight)

        count = probe.navigation_count()
        at, _ = flight.operator_mode(LOITER, True)
        transitioning = flight.wait(9, lambda: observe(flight, LOITER, TO_FW, at, minimum=35),
                                    'native hover-to-forward transition was not observed')
        report['forward_transition'] = {'request_at': at, 'phase': transitioning}
        if forward_takeover:
            report['forward_withdrawn'] = flight.wait(5, lambda: probe.runtime(
                at, permission='false', fc_mode=str(LOITER)), 'pilot forward transition retained the old hover grant')
            report['transition_stock'] = stock(flight, report['forward_withdrawn']['mono'])
            count = probe.navigation_count()
        report['forward_refusal'] = refuse(flight, 'native forward transition', transitioning)
        report['forward_flight'] = flight.wait(55, lambda: observe(flight, LOITER, FW, at, minimum=35, velocity=18),
                                               'independent native forward flight was not observed')
        if forward_takeover:
            report['steady_forward_refusal'] = refuse(flight, 'stable native forward flight', report['forward_flight'])
        report['forward_quiet'] = hold_no_grant(flight, count, 2., LOITER)
        at, _ = flight.operator_mode(QLOITER, True)
        report['native_return'] = {'request_at': at, 'hold': hold_ready(flight, at)}
        if forward_takeover:
            report['return_stock'] = stock(flight, report['native_return']['hold']['heartbeat']['mono'])
        report['return_quiet'] = hold_no_grant(flight, count, 1.25, QLOITER)
        report['native_return']['fresh_hold'] = hold_ready(flight, report['return_quiet']['through'])
    report['return_acquisition'] = acquire(flight)
    report['return_motion'] = move(flight, report['return_acquisition'])

    at, hb = flight.operator_mode(FBWA, True)
    withdrawn = flight.wait(5, lambda: probe.runtime(at, permission='false', fc_mode=str(FBWA)),
                            'native pilot takeover did not withdraw the old grant')
    quiet = hold_no_grant(flight, probe.navigation_count(), 1.25, FBWA)
    report['pilot_takeover'] = {'request_at': at, 'heartbeat': hb, 'withdrawn': withdrawn, 'quiet': quiet}
    at, _ = flight.operator_mode(QRTL, True)
    report['touchdown'] = flight.wait(125, lambda: flight.observation(after=at, mode=QRTL, armed=False,
        phase=MC, landed=GROUND), 'native QRTL did not land and disarm')


def hover_wire(out, report, monitor):
    readbacks = [row for row in monitor if row['mavpackettype'] == 'POSITION_TARGET_GLOBAL_INT']
    readback_times = [row['mono'] for row in readbacks]
    offers, used, targets, matched, modes = {}, set(), [], [], []
    phase_at, mode = None, None
    takeover_at = report.get('forward_transition', {}).get('request_at', float('inf'))
    forward_fences = []
    for at, side, message in wire(out, report):
        kind = message.get_type()
        if side == 'R' and (message.get_srcSystem(), message.get_srcComponent()) == (1, 1):
            if kind == 'EXTENDED_SYS_STATE':
                phase_at = at if message.vtol_state == MC and message.landed_state == AIR else None
                if at >= takeover_at and phase_at is None:
                    forward_fences.append(at)
            elif kind == 'HEARTBEAT':
                mode = message.custom_mode
                if at >= takeover_at and mode == LOITER:
                    forward_fences.append(at)
            elif kind == 'TUNNEL' and message.payload_length == 10:
                magic, token, offer, sentinel = struct.unpack('<4sIBB', bytes(message.payload[:10]))
                if (magic, offer, sentinel, message.target_system, message.target_component, message.payload_type) == (
                        b'MQH1', 1, 1, 245, 191, 0):
                    offers[token] = at
        if side != 'T':
            continue
        need(kind not in ('COMMAND_INT', 'SET_POSITION_TARGET_LOCAL_NED', 'SET_ATTITUDE_TARGET',
                          'RC_CHANNELS_OVERRIDE', 'MANUAL_CONTROL'), 'unexpected control message on hover route')
        if kind == 'COMMAND_LONG' and message.command == mav.MAV_CMD_DO_SET_MODE:
            modes.append((at, int(message.param2)))
        if kind != 'TUNNEL':
            continue
        need((message.target_system, message.target_component, message.payload_type, message.payload_length) ==
             (1, 1, 0, 24), 'hover target lacked exact addressing or envelope')
        magic, token, action, ttl, lat, lon, alt, sentinel = struct.unpack('<4sIBHiiiB', bytes(message.payload[:24]))
        need((magic, action, sentinel) == (b'MQH1', 2, 1) and token in offers and token not in used and
             0 <= at - offers[token] < min(.35, ttl / 1000) and 0 < ttl <= 750 and
             phase_at is not None and at - phase_at <= 1.5 and mode == GUIDED,
             'hover target had stale or reused authority, or crossed a phase boundary')
        used.add(token)
        targets.append(at)
        for row in readbacks[bisect_left(readback_times, at):]:
            if row['mono'] > at + ttl / 1000:
                break
            if (row['lat_int'], row['lon_int']) == (lat, lon) and abs(row['alt'] - alt / 100) <= .3:
                matched.append(at)
                break
    return targets, matched, modes, forward_fences


def audit_hover(out, report):
    monitor_count = verify_capture(out, 'monitor')
    monitor = archived_rows(out, 'monitor-rx.jsonl')
    moos = archived_rows(out, 'moos.jsonl')
    targets, matched, modes, forward_fences = hover_wire(out, report, monitor)
    takeover_at = report.get('forward_transition', {}).get('request_at', float('inf'))
    first, second = report['initial_acquisition'], report['return_acquisition']
    pilot = report['neutral_pilot']
    need(pilot['readback'] in monitor and pilot['readback']['mono'] > pilot['sent_at'] and
         pilot['readback']['chan3_raw'] == 1500, 'neutral simulation pilot readback did not replay')
    motions = [(second, report['return_motion'], None)]
    if report['case'] != 'hover-adapter-loss':
        motions.append((first, report['initial_motion'], 1.3))
    for acquisition, motion, climb in motions:
        end = motion['end']['position']
        north, east = delta(motion['start'], end)
        up = (end['relative_alt'] - motion['start']['relative_alt']) / 1000
        need(all(row in moos and row['source'] == 'pHelmIvP' and row['mono'] > acquisition['request_at']
                 for row in acquisition['scalars'].values()) and motion['start'] in monitor and
             all(row in monitor for row in motion['end'].values()) and
             (north, east, up) == (motion['north_m'], motion['east_m'], motion['up_m']) and
             east >= 7 and abs(north) <= 4 and speed(end) <= 5 and height_reached(end, up, climb) and
             any(acquisition['request_at'] < at <= motion['end']['position']['mono'] for at in matched),
             'actual motion, stock scalars or matched target could not be replayed')
    result = {'monitor_crc_frames': monitor_count, 'live_targets': len(targets),
              'target_readbacks': len(matched), 'stable_phase_and_unique_offer': True}
    if report['case'] == 'hover-forward-takeover':
        active, withdrawn = report['active_before_forward'], report['forward_withdrawn']
        need(active in moos and fields(active)['permission'] == 'true' and active['mono'] < takeover_at and
             withdrawn in moos and fields(withdrawn)['permission'] == 'false' and
             forward_fences and takeover_at <= forward_fences[0] <= withdrawn['mono'] and
             any(active['mono'] - .75 <= at < takeover_at for at in matched),
             'pilot request did not cross from verified active hover to revoked navigation')
        for name, lower, upper in (
            ('transition_stock', withdrawn['mono'], report['forward_refusal']['request_at']),
            ('return_stock', report['native_return']['hold']['heartbeat']['mono'], second['request_at']),
        ):
            need(all(row in moos and row['source'] == 'pHelmIvP' and lower < row['mono'] < upper
                     for row in report[name].values()), 'stock outputs did not continue across handback')
        for name, phase in (('forward_refusal', TO_FW), ('steady_forward_refusal', FW)):
            denial = report[name]
            phase_sample = denial['phase_sample']
            need(all(row in monitor for row in phase_sample.values()) and
                 phase_sample['state']['vtol_state'] == phase and
                 phase_sample['heartbeat']['custom_mode'] == LOITER and
                 denial['rejected'] in moos and fields(denial['rejected'])['reason'] == 'acquisition_rejected' and
                 denial['runtime'] in moos and fields(denial['runtime'])['permission'] == 'false',
                 'forward-phase denial lacked independent mode, phase or rejection evidence')
        forward_quiet_from = forward_fences[0]
        result['active_hover_forward_takeover'] = True
        result['quiet_until_explicit_reacquire_s'] = round(second['request_at'] - forward_quiet_from, 3)
    elif report['case'] == 'hover-adapter-loss':
        loss = report['adapter_loss']
        observed = list(wire(out, report))
        need(any(at == loss['pending_at'] and side == 'h' and msg.get_type() == 'TUNNEL'
                 for at, side, msg in observed) and
             any(loss['lost_at'] <= at < loss['restored_at'] and side == 'i' and msg.get_type() == 'TUNNEL'
                 for at, side, msg in observed), 'pending target or selective adapter loss absent on wire')
        hold = loss['hold']
        need(all(row in monitor for row in hold['native'].values()) and
             hold['native']['heartbeat']['custom_mode'] == QLOITER and hold['runtime'] in moos and
             fields(hold['runtime'])['permission'] == 'false' and
             any(first['request_at'] < at < loss['pending_at'] for at in matched),
             'initial verified target or native adapter-loss hold did not replay')
        need(all(row in moos and row['source'] == 'pHelmIvP' for row in loss['stock_after'].values()),
             'fresh stock outputs after adapter restoration did not replay')
        forward_quiet_from = loss['lost_at']
        result['pending_target_adapter_loss_fenced'] = True
    else:
        forward_quiet_from = report['producer_silence']['hold']['heartbeat']['mono']
    quiet = [(forward_quiet_from, second['request_at']),
             (report['pilot_takeover']['withdrawn']['mono'], report['touchdown']['heartbeat']['mono'])]
    need(all(not any(start <= at < end for at in targets) for start, end in quiet) and
         [destination for _, destination in modes].count(GUIDED) == 2 and
         all(destination in (GUIDED, QLOITER) for _, destination in modes) and
         len(matched) >= 6, 'old grants or unverified targets crossed native ownership')
    return result


def audit_case(out, report):
    if report['case'] == 'hover-survey':
        from hover_survey import audit
        return audit(out, report)
    if report['case'] == 'hover-mode-stress':
        from hover_mode_stress import audit
        return audit(out, report)
    return audit_hover(out, report)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Replay a pinned QuadPlane hover receipt.')
    parser.add_argument('receipt', type=Path)
    args = parser.parse_args()
    report = json.loads(args.receipt.read_text())
    need(report['case'] in HOVER_CASES and report['result'] == 'PASS', 'expected a passing hover receipt')
    evidence = audit_case(args.receipt.parent, report)
    need(evidence == report['wire_audit'], 'replayed hover audit differs from the stored result')
    print(json.dumps(evidence, indent=2))
