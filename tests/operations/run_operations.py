#!/usr/bin/env python3
"""Owned native Copter operation workflow; no hardware or non-loopback route."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tests'))
from copter_frame_fixture import STANDARD_COPTER_FRAMES, bind_copter_frame, copter_default_paths, one_copter_frame_log

PRECISION_BEACON = (42.358456, -71.087515)


def fields(text):
    return dict(part.split('=', 1) for part in text.split(',') if '=' in part)


def finite_number(value, label):
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        raise RuntimeError('invalid numeric evidence: '+label) from None
    if isinstance(value, bool) or not math.isfinite(result):
        raise RuntimeError('nonfinite or invalid numeric evidence: '+label)
    return result


def finite_time(value, label):
    result = finite_number(value, label)
    if result < 0:
        raise RuntimeError('negative evidence timestamp: '+label)
    return result


def events(path):
    result = []
    for line in path.read_text().splitlines():
        try:
            item = json.loads(line)
            if isinstance(item, dict):
                result.append(item)
        except json.JSONDecodeError:
            continue  # Native MOOS client startup text is not an event.
    return result


def catalog_evidence(trace, parity=False):
    catalogs = [json.loads(item['value']) for item in trace
                if item.get('key') == 'MAVMOOS_CAPABILITIES']
    if not catalogs or len({json.dumps(item, sort_keys=True) for item in catalogs}) != 1:
        raise RuntimeError('missing or changing observed capability composition')
    catalog = catalogs[0]
    composition = catalog.get('composition', {})
    modules = composition.get('modules', [])
    if catalog.get('schema') != 1 or catalog.get('execution_mode') != 'SITL_CONTROL' or composition.get('plugin_abi') != 10:
        raise RuntimeError('wrong observed execution mode/schema/ABI')
    if len(modules) != 3 or {m['name'] for m in modules} != {'telemetry', 'parameters', 'arducopter_control'}:
        raise RuntimeError('operation mission composition differs from requested modules')
    controls = [m for m in modules if m['control_provider']]
    if len(controls) != 1 or controls[0]['name'] != 'arducopter_control':
        raise RuntimeError('expected exactly one Copter control provider')
    capabilities = [c for m in modules for c in m['capabilities']]
    ids = [c['name'] for c in capabilities]
    owners = [owner for m in modules for owner in m['request_owners']]
    if len(ids) != len(set(ids)) or len(owners) != len(set(owners)):
        raise RuntimeError('ambiguous capability/request ownership')
    expected = {'control.arm', 'control.disarm', 'control.takeoff', 'control.hold.native_loiter',
                'control.land', 'control.return.native_rtl', 'control.guided',
                'control.navigation.helm', 'control.hold.guided_recovery',
                'control.operations.native', 'control.operations.guided',
                'control.operations.helm', 'control.operations.cancel'}
    if parity:
        expected.update({'control.guided.speed_adjustment', 'control.guided.speed_reset',
                         'control.guided.course_adjustment', 'control.precision_loiter.enable',
                         'control.precision_loiter.disable', 'control.return.dead_man_rtl',
                         'control.operations.helm.adjustment', 'control.operations.helm.survey',
                         'control.operations.helm.voronoi'})
    if {name for name in ids if name.startswith('control.')} != expected:
        raise RuntimeError('configured operation strategy catalog is incomplete or unexpected')
    if 'MAVMOOS_CONTROL_REQUEST' not in controls[0]['request_owners']:
        raise RuntimeError('new operation channel is not owned by the Copter provider')
    if parity and 'DEAD_MAN_POST_INTERRUPT' not in controls[0]['request_owners']:
        raise RuntimeError('parity dead-man channel is not owned by the Copter provider')
    return catalog


def operation_evidence(trace, parity=False):
    results = {}
    for item in trace:
        if item.get('type') in ('phase', 'stock_helm'):
            finite_time(item.get('mono'), 'probe phase/helm mono')
        if item.get('type') == 'observation' and item.get('key') == 'MAVMOOS_CONTROL_RESULT':
            value = fields(item['value'])
            if value.get('schema') != '1' or 'id' not in value or 'status' not in value:
                raise RuntimeError('malformed public operation result')
            if finite_number(value.get('expires'), 'operation expires') <= 0:
                raise RuntimeError('nonpositive operation expiry')
            finite_time(value.get('at'), 'operation at')
            results.setdefault(value['id'], []).append(value)
    expected = {
        'ops-reject-ground': 'REJECTED', 'ops-guided': 'COMPLETED', 'ops-arm': 'COMPLETED',
        'ops-takeoff': 'COMPLETED', 'ops-guided-goto': 'COMPLETED',
        'ops-guided-hold': 'CANCELLED', 'ops-guided-cancel': 'COMPLETED',
        'ops-helm-goto': 'COMPLETED', 'ops-helm-hold': 'CANCELLED',
        'ops-helm-cancel': 'COMPLETED', 'ops-helm-return': 'COMPLETED',
        'ops-native-hold': 'COMPLETED', 'ops-rtl-stage': 'COMPLETED', 'ops-native-rtl': 'COMPLETED'}
    if parity:
        del expected['ops-native-rtl']
        expected.update({'ops-adjust-altitude': 'COMPLETED', 'ops-visualize-home': 'COMPLETED',
                         'ops-helm-adjust-speed': 'ACCEPTED', 'ops-helm-adjust-altitude': 'ACCEPTED',
                         'ops-survey': 'CANCELLED', 'ops-survey-cancel': 'COMPLETED',
                         'ops-voronoi': 'CANCELLED', 'ops-voronoi-cancel': 'COMPLETED',
                         'ops-precision-retreat': 'COMPLETED', 'ops-precision-disabled-hold': 'COMPLETED'})
    proof = {}
    for ident, outcome in expected.items():
        items = results.get(ident, [])
        if not items or items[-1]['status'] != outcome:
            raise RuntimeError('missing/unexpected terminal operation result: ' + ident)
        bounds = {item.get('expires') for item in items}
        if len(bounds) != 1 or float(next(iter(bounds))) <= 0:
            raise RuntimeError('original public expiry changed or missing: ' + ident)
        if any(float(item['at']) > float(item['expires']) + .15
               for item in items if item['status'] not in ('TIMED_OUT', 'REJECTED')):
            raise RuntimeError('successful result after original operation deadline: ' + ident)
        proof[ident] = {'statuses': [item['status'] for item in items],
                        'original_expires': float(next(iter(bounds))),
                        'terminal': outcome}
    for ident, items in results.items():
        if ident not in expected and (ident != 'ops-disarm' or items[-1]['status'] != 'COMPLETED'):
            raise RuntimeError('unexpected operation result: ' + ident)
    if parity:
        for task in ('ops-survey', 'ops-voronoi'):
            statuses = proof[task]['statuses']
            if 'CONFIRMED' not in statuses or 'COMPLETED' in statuses:
                raise RuntimeError('expected a confirmed and explicitly cancelled helm task: '+task)
    for phase in ('helm_goto', 'helm_hold', 'helm_return'):
        begin = next((item['mono'] for item in trace if item.get('type') == 'phase' and item.get('key') == phase), None)
        end = next((item['mono'] for item in trace if item.get('type') == 'phase' and begin is not None and item['mono'] > begin), None)
        keys = {item['key'] for item in trace if item.get('type') == 'stock_helm' and begin is not None and item['mono'] >= begin and (end is None or item['mono'] < end)}
        if keys != {'DESIRED_HEADING', 'DESIRED_SPEED', 'DESIRED_ALTITUDE'}:
            raise RuntimeError('all three real stock helm outputs not observed in ' + phase)
    return proof


def parity_evidence(trace, *, legacy_speed_ack=False, legacy_precision_ack=False):
    speed_terminal, speed_reason = (('ACCEPTED', 'guided_speed_limit_ack_only') if legacy_speed_ack
                                    else ('UNCONFIRMED', 'guided_speed_cap_not_observable'))
    precision_terminal, precision_reason = (('ACCEPTED', 'precision_aux_ack_only') if legacy_precision_ack
                                            else ('UNCONFIRMED', 'precision_aux_state_not_observable'))
    expected = {
        'parity-reset-speed': ('reset_speed_min', speed_terminal, {'reason': speed_reason}),
        'parity-change-speed': ('change_speed', speed_terminal, {'reason': speed_reason, 'delta': '.3'}),
        'parity-change-course': ('change_course', 'COMPLETED', {'delta': '20'}),
        'parity-precision-on': ('precision_loiter_on', precision_terminal, {'reason': precision_reason}),
        'parity-precision-off': ('precision_loiter_off', precision_terminal, {'reason': precision_reason}),
        'deadman.1': ('dead_man_rtl', 'COMPLETED', {}),
    }
    requests, outcomes, handoffs, parameter_handoffs = {}, {}, {}, {}
    markers = []
    for item in trace:
        kind, key = item.get('type'), item.get('key')
        if kind == 'request' and key == 'DEAD_MAN_POST_INTERRUPT':
            if item['value'] != 'true' or 'deadman.1' in requests:
                raise RuntimeError('unexpected dead-man request evidence')
            requests['deadman.1'] = (finite_time(item.get('mono'), 'dead-man request'), {})
        if kind == 'request' and key == 'MAVMOOS_COMMAND':
            value = fields(item['value'])
            ident = value.get('id')
            if ident not in expected or ident in requests:
                raise RuntimeError('unexpected parity command request: '+str(ident))
            requests[ident] = (finite_time(item.get('mono'), 'parity command request'), value)
        if kind != 'observation':
            continue
        value = fields(item['value'])
        ident = value.get('id', '')
        if key == 'VIEW_MARKER' and value.get('label') == 'MAVMOOS_HOME':
            markers.append({'x': finite_number(value.get('x'), 'home marker x'),
                            'y': finite_number(value.get('y'), 'home marker y')})
        if key == 'MAVMOOS_PARAM_TX_RESULT' and ident.startswith('parity-precision-on/'):
            parameter_handoffs.setdefault(ident, []).append((finite_time(item.get('mono'), 'parameter handoff'), value))
        if not ident.startswith(('parity-', 'deadman.')) or key not in ('MAVMOOS_COMMAND_RESULT', 'MAVMOOS_TX_RESULT'):
            continue
        if ident not in expected:
            raise RuntimeError('unexpected parity command outcome: '+ident)
        collection = outcomes if key == 'MAVMOOS_COMMAND_RESULT' else handoffs
        collection.setdefault(ident, []).append((finite_time(item.get('mono'), 'parity command observation'), value))
    if len(markers) != 1:
        raise RuntimeError('expected one projected home marker')
    proof = {}
    for ident, (action, terminal, details) in expected.items():
        if ident not in requests or ident not in outcomes:
            raise RuntimeError('missing parity request or result: '+ident)
        request_at, request = requests[ident]
        if ident != 'deadman.1' and (request.get('action') != action or
                                   request.get('delta') != details.get('delta')):
            raise RuntimeError('parity command request does not match fixture: '+ident)
        sequence = outcomes[ident]
        if any(value.get('schema') != '1' or value.get('action') != action for _, value in sequence):
            raise RuntimeError('malformed parity command result: '+ident)
        timestamps = [finite_time(value.get('at'), 'parity command result at') for _, value in sequence]
        if timestamps != sorted(timestamps):
            raise RuntimeError('parity command result timestamps went backwards: '+ident)
        terminal_at, result = sequence[-1]
        if result.get('status') != terminal or ('reason' in details and result.get('reason') != details['reason']):
            raise RuntimeError('unexpected parity command terminal result: '+ident)
        local = handoffs.get(ident, [])
        ack_status, ack_reason = (('ACK_OBSERVED', 'uncorrelated_ack_result_0') if not legacy_precision_ack or terminal == 'UNCONFIRMED'
                                  else ('ACKNOWLEDGED', 'accepted'))
        accepted_ack = [when for when, value in sequence if value.get('status') == ack_status and
                        value.get('reason') == ack_reason]
        if len(local) != 1 or local[0][1].get('status') != 'LOCAL_ACCEPTED' or len(accepted_ack) != 1 or \
                not request_at <= local[0][0] <= accepted_ack[0] <= terminal_at:
            raise RuntimeError('missing ordered local handoff and accepted FC ACK: '+ident)
        proof[ident] = {'action': action, 'terminal': terminal, 'reason': result.get('reason'),
                        'statuses': [value['status'] for _, value in sequence],
                        'request_mono': request_at, 'local_handoff_mono': local[0][0],
                        'fc_ack_mono': accepted_ack[0], 'terminal_mono': terminal_at}
    precision = proof['parity-precision-on']
    parameter_ids = ['parity-precision-on/PLND_ENABLED', 'parity-precision-on/PLND_TYPE']
    if set(parameter_handoffs) != set(parameter_ids) or any(len(parameter_handoffs[key]) != 1 for key in parameter_ids):
        raise RuntimeError('missing scoped precision parameter read handoff')
    param_receipts = [parameter_handoffs[key][0] for key in parameter_ids]
    if any(value.get('status') != 'LOCAL_ACCEPTED' or value.get('late') != 'false' or
           value.get('clock_valid') != 'true' for _, value in param_receipts) or \
            not precision['request_mono'] <= param_receipts[0][0] <= param_receipts[1][0] <= precision['local_handoff_mono']:
        raise RuntimeError('precision parameter reads were not handed off before auxiliary enable')
    return {'commands': proof, 'home_marker': markers[0], 'precision_parameter_read_ids': parameter_ids,
            'precision_scope': 'parameter reads and auxiliary command acknowledgements'}


def helm_parity_evidence(trace):
    def phase_rows(phase):
        begin = [index for index, item in enumerate(trace) if item.get('type') == 'phase' and item.get('key') == phase]
        if len(begin) != 1:
            raise RuntimeError('missing or repeated parity helm phase: '+phase)
        end = next((index for index in range(begin[0]+1, len(trace)) if trace[index].get('type') == 'phase'), len(trace))
        return trace[begin[0]+1:end]

    def observation(rows, key, value=None):
        matches = [item for item in rows if item.get('type') == 'observation' and item.get('key') == key and
                   (value is None or item['value'] == value)]
        if len(matches) != 1:
            raise RuntimeError('missing or repeated parity helm publication: '+key)
        return matches[0]

    proof = {'tasks': {}, 'adjustments': {}, 'scope': 'stock behavior selection, updates and fresh scalar output'}
    for task in ('survey', 'voronoi'):
        rows = phase_rows('parity_helm_'+task)
        selected = observation(rows, 'MAVMOOS_HELM_OPERATION', task)
        selected_at = finite_time(selected.get('moos_at'), 'task selector publication')
        samples = {}
        for item in rows:
            if item.get('type') != 'stock_helm':
                continue
            when = finite_time(item.get('moos_at'), 'stock scalar publication')
            if when > selected_at:
                samples[item['key']] = when
        if set(samples) != {'DESIRED_HEADING', 'DESIRED_SPEED', 'DESIRED_ALTITUDE'}:
            raise RuntimeError('all three post-selector stock outputs required: '+task)
        cleared = observation(phase_rows('parity_helm_'+task+'_cancel'), 'MAVMOOS_HELM_OPERATION', 'none')
        cleared_at = finite_time(cleared.get('moos_at'), 'task selector clear')
        if cleared_at <= max(samples.values()):
            raise RuntimeError('old selector clear cannot prove parity task teardown: '+task)
        proof['tasks'][task] = {'selector_moos_at': selected_at, 'scalar_moos_at': samples, 'clear_moos_at': cleared_at}
    for kind, delta, update_key, before_phase in (
            ('speed', .5, 'MAVMOOS_HELM_SPEED_UPDATE', 'parity_helm_seed'),
            ('altitude', 1, 'MAVMOOS_OPERATION_ALTITUDE_UPDATE', 'parity_helm_adjust_speed')):
        scalar_key = 'DESIRED_'+kind.upper()
        before = [item for item in phase_rows(before_phase) if item.get('type') == 'stock_helm' and item.get('key') == scalar_key]
        if not before:
            raise RuntimeError('missing pre-adjustment stock scalar: '+kind)
        previous = finite_number(before[-1]['value'], 'initial stock scalar')
        rows = phase_rows('parity_helm_adjust_'+kind)
        update = observation(rows, update_key)
        target = finite_number(fields(update['value']).get(kind), 'requested behavior update')
        update_at = finite_time(update.get('moos_at'), 'behavior update publication')
        after = [finite_number(item['value'], 'updated stock scalar') for item in rows if
                 item.get('type') == 'stock_helm' and item.get('key') == scalar_key and
                 finite_time(item.get('moos_at'), 'stock scalar publication') > update_at]
        if not math.isclose(target, previous+delta, abs_tol=.011) or not any(abs(value-target) < .11 for value in after):
            raise RuntimeError('behavior update failed to change the stock scalar: '+kind)
        proof['adjustments'][kind] = {'before': previous, 'target': target, 'post_update_samples': after}
    return proof


def parity_behaviors(ordinary):
    old = 'condition = MAVMOOS_HELM_OPERATION != none'
    if ordinary.count(old) != 2:
        raise RuntimeError('parity requires the two existing point/height bindings')
    exact = 'condition = ((MAVMOOS_HELM_OPERATION == route) or (MAVMOOS_HELM_OPERATION == hold))'
    value = 'initialize PARITY_HELM_SEED = false\n'+ordinary.replace(old, exact)
    for name, selector, points in (('seed', 'none', '12,0:12,5'),
                                   ('survey', 'survey', '8,7:11,7'),
                                   ('voronoi', 'voronoi', '-4,9:-6,11')):
        value += f'''
Behavior = BHV_Waypoint
{{
  name = parity_{name}_route
  pwt = 100
  condition = PARITY_HELM_SEED == true
  condition = MAVMOOS_HELM_OPERATION == {selector}
  updates = MAVMOOS_HELM_SPEED_UPDATE
  points = {points}
  speed = 1.5
  capture_radius = 1
  slip_radius = 1
}}
'''
    return value + '''
Behavior = BHV_ConstantAltitude
{
  name = parity_height
  pwt = 100
  condition = PARITY_HELM_SEED == true
  altitude = 6
  duration = -1
  updates = MAVMOOS_OPERATION_ALTITUDE_UPDATE
}
'''


def distance(lat, lon, home_lat, home_lon):
    a, b = math.radians(lat), math.radians(home_lat)
    hav = math.sin((b-a)/2)**2 + math.cos(a)*math.cos(b)*math.sin(math.radians(home_lon-lon)/2)**2
    return 12742000 * math.asin(math.sqrt(min(1, max(0, hav))))


def precision_motion_evidence(path):
    from pymavlink import mavutil

    reader = mavutil.mavlink_connection(str(path))
    modes, sensor, truth = [], [], []
    samples = {'MODE': modes, 'PL': sensor, 'SIM': truth}
    while (message := reader.recv_match(type=list(samples))) is not None:
        samples[message.get_type()].append(message.to_dict())

    holds = [(mode['TimeUS'], following['TimeUS']) for mode, following in zip(modes, modes[1:])
             if mode['ModeNum'] == 5 and following['ModeNum'] != 5]
    if len(holds) != 2:
        raise RuntimeError('expected two independently logged native Loiter intervals')
    (start, end), (disabled_start, disabled_end) = holds
    if end - start < 5_000_000 or disabled_end - disabled_start < 2_500_000:
        raise RuntimeError('native Loiter intervals too short for precision-motion proof')

    def dwell(begin, finish, minimum, maximum):
        positions = [p for p in truth if begin <= p['TimeUS'] <= finish]
        detected = [p for p in sensor if begin <= p['TimeUS'] <= finish and p['Heal'] and p['TAcq']]
        if len(positions) < 8 or positions[-1]['TimeUS'] - positions[0]['TimeUS'] < 1_250_000:
            raise RuntimeError('insufficient independent SITL truth for precision dwell')
        if len({p['LastMeasMS'] for p in detected}) < 2:
            raise RuntimeError('precision sensor did not independently report fresh acquired target')
        distances = [distance(p['Lat'], p['Lng'], *PRECISION_BEACON) for p in positions]
        excursion = max(distance(p['Lat'], p['Lng'], positions[0]['Lat'], positions[0]['Lng']) for p in positions)
        if min(distances) < minimum or max(distances) > maximum or excursion > .8:
            raise RuntimeError(f'precision dwell outside bounds, target={min(distances):.2f}..{max(distances):.2f}m '
                               f'excursion={excursion:.2f}m')
        return {'start_boot_us': positions[0]['TimeUS'], 'finish_boot_us': positions[-1]['TimeUS'],
                'truth_samples': len(positions), 'target_distance_m': [min(distances), max(distances)],
                'max_excursion_m': excursion, 'fresh_target_records': len(detected),
                'first_truth': positions[0], 'last_truth': positions[-1]}

    offset = dwell(start + 300_000, start + 2_100_000, 3.5, 9.)
    captured = dwell(end - 1_950_000, end - 100_000, 0., 1.55)
    disabled = dwell(disabled_start + 350_000, disabled_end - 100_000, 3.5, 9.)
    origin, finish = offset['first_truth'], captured['last_truth']
    travelled = distance(origin['Lat'], origin['Lng'], finish['Lat'], finish['Lng'])
    if travelled < 3.5:
        raise RuntimeError('independent SITL truth did not establish travel to the beacon')
    return {'beacon_lat_lon': PRECISION_BEACON, 'mode': 5, 'travelled_m': travelled,
            'ordinary_offset': offset, 'precision_captured': captured, 'disabled_offset': disabled}


def flight_evidence(packets, trace, monitor_finished_mono=None, landing_target=None):
    if not packets:
        raise RuntimeError('no independent FC packets')
    receipts = [finite_time(item.get('received_mono'), 'FC receipt') for item in packets]
    if receipts != sorted(receipts):
        raise RuntimeError('FC receipt clock/order changed')
    latest_receipt = max(receipts)
    if monitor_finished_mono is None:
        # Offline replay can assess the last recorded packet, including status
        # traffic, but cannot invent the unrecorded time when monitoring ended.
        freshness_reference, freshness_scope = latest_receipt, 'last_received_packet_only'
    else:
        freshness_reference = finite_time(monitor_finished_mono, 'monitor_finished_mono')
        if freshness_reference < latest_receipt:
            raise RuntimeError('monitor end precedes an FC receipt')
        freshness_scope = 'monitor_finished_mono'
    homes = [fields(item['value']) for item in trace if item.get('key') == 'MAVMOOS_HOME_POSITION'
             and item.get('type') == 'observation']
    homes = [item for item in homes if item.get('encoding_valid') == 'true']
    if not homes:
        raise RuntimeError('no observed bound FC home reference')
    home_lat = finite_number(homes[-1].get('lat_deg'), 'home latitude')
    home_lon = finite_number(homes[-1].get('lon_deg'), 'home longitude')
    if abs(home_lat) > 90 or abs(home_lon) > 180:
        raise RuntimeError('home coordinate outside bounds')
    positions = [m for m in packets if m['mavpackettype'] == 'GLOBAL_POSITION_INT']
    hearts = [m for m in packets if m['mavpackettype'] == 'HEARTBEAT']
    landed = [m for m in packets if m['mavpackettype'] == 'EXTENDED_SYS_STATE']
    if not positions or not hearts or not landed:
        raise RuntimeError('missing independent FC flight observations')
    for position in positions:
        if abs(finite_number(position.get('lat'), 'FC latitude')) > 9e8 or \
           abs(finite_number(position.get('lon'), 'FC longitude')) > 18e8:
            raise RuntimeError('FC position outside bounds')
        finite_number(position.get('relative_alt'), 'FC relative height')
    hold_positions, rtl_positions = [], []
    mode = None
    # FC receipt evidence stays entirely on the observer's own monotonic clock.
    # Python and C++ steady-clock epochs differ on some macOS versions; probe
    # phase timestamps must never be compared with these receipt timestamps.
    rtl_begin = next((item['received_mono'] for item in hearts if item['custom_mode'] == 6), None)
    if rtl_begin is None:
        raise RuntimeError('native RTL mode not independently observed')
    before_rtl = [m for m in positions if rtl_begin-2 <= m['received_mono'] <= rtl_begin]
    for item in packets:
        if item['mavpackettype'] == 'HEARTBEAT':
            mode = item['custom_mode']
        if item['mavpackettype'] == 'GLOBAL_POSITION_INT':
            if mode == 5:
                hold_positions.append(item)
            if mode == 6 and item['received_mono'] >= rtl_begin:
                rtl_positions.append(item)
    hold_heights = [m['relative_alt']/1000 for m in hold_positions]
    hold_span = hold_positions[-1]['received_mono']-hold_positions[0]['received_mono'] if hold_positions else 0
    spread = max(hold_heights, default=0)-min(hold_heights, default=0)
    if len(hold_heights) < 5 or hold_span < 1 or spread > .5:
        raise RuntimeError('independent FC Loiter height stability not corroborated')
    metres = lambda m: distance(m['lat']/1e7, m['lon']/1e7, home_lat, home_lon)
    start_distance = max((metres(item) for item in before_rtl), default=0)
    rtl_distances = [metres(m) for m in rtl_positions]
    if start_distance < 6 or len(rtl_distances) < 5 or min(rtl_distances) > 2:
        raise RuntimeError('native RTL did not independently traverse from away-home to home region')
    modes = {m['custom_mode'] for m in hearts}
    final_disarmed = not bool(hearts[-1]['base_mode'] & 128)
    final_ground = landed[-1]['landed_state'] == 1
    max_height = max(m['relative_alt']/1000 for m in positions)
    final_distance = metres(positions[-1])
    target_distance = final_distance if landing_target is None else distance(
        positions[-1]['lat']/1e7, positions[-1]['lon']/1e7, *landing_target)
    final_ages = {'position_s': freshness_reference-positions[-1]['received_mono'],
                  'heartbeat_s': freshness_reference-hearts[-1]['received_mono'],
                  'landed_s': freshness_reference-landed[-1]['received_mono']}
    final_fresh = final_ages['position_s'] <= .5 and final_ages['heartbeat_s'] <= 2 and final_ages['landed_s'] <= 2
    final_after_rtl = all(items[-1]['received_mono'] >= rtl_begin for items in (positions, hearts, landed))
    if not {4, 5, 6}.issubset(modes) or not final_disarmed or not final_ground or max_height < 4 or \
       not final_fresh or not final_after_rtl or target_distance > 2 or abs(positions[-1]['relative_alt']) > 500:
        raise RuntimeError('independent observer did not corroborate required flight and final state')
    return {'position_samples': len(positions), 'heartbeat_samples': len(hearts),
            'home': {'latitude': home_lat, 'longitude': home_lon}, 'observed_modes': sorted(modes),
            'maximum_relative_height_m': max_height, 'final_disarmed': final_disarmed,
            'final_on_ground': final_ground,
            'native_loiter': {'samples': len(hold_heights), 'span_s': hold_span, 'height_spread_m': spread},
            'native_rtl': {'clock_basis': 'independent FC observer receipt monotonic only',
                           'first_mode6_heartbeat_received_mono': rtl_begin,
                           'before_mode6_window_s': 2,
                           'before_mode6_max_home_distance_m': start_distance,
                           'mode6_position_samples': len(rtl_distances),
                           'closest_home_distance_m': min(rtl_distances),
                           'final_home_distance_m': final_distance,
                           'final_landing_target_distance_m': target_distance,
                           'final_relative_height_m': positions[-1]['relative_alt']/1000,
                           'freshness_scope': freshness_scope,
                           'freshness_reference_mono': freshness_reference,
                           'final_observation_ages': final_ages,
                           'final_fresh': final_fresh}}


def main():
    # Keep evidence auditors importable with ordinary Python for hostile-fixture
    # tests. Only the explicitly invoked native workflow needs pymavlink.
    import run_sitl as fixture
    from control.catalog_evidence import loaded_module_hashes
    from pymavlink import mavutil
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('ardupilot-root', 'moos-root', 'build', 'output'):
        parser.add_argument('--'+name, required=True)
    parser.add_argument('--behavior-dir', default=str(Path.home()/'moos-ivp-uav-base/lib'))
    parser.add_argument('--parity', action='store_true', help='exercise Copter parity additions on the pinned 4.6.3 profile')
    parser.add_argument('--copter-frame', choices=tuple(STANDARD_COPTER_FRAMES), default='quad')
    parser.add_argument('--firmware-attestation', type=Path)
    args = parser.parse_args()
    frame = args.copter_frame
    expected_type, expected_class = STANDARD_COPTER_FRAMES[frame]
    if frame != 'quad' and (not args.parity or not args.firmware_attestation):
        parser.error('standard hexa and octa require --parity and an explicit firmware attestation')
    ap, moos, build, out = (Path(getattr(args, key)).resolve() for key in ('ardupilot_root', 'moos_root', 'build', 'output'))
    manifest = ROOT/'config/firmware/arducopter-4.6.3-quad.json'
    firmware = (fixture.native_firmware_profile(ROOT, ap, manifest, args.firmware_attestation)
                if args.firmware_attestation else fixture.native_firmware_profile(ROOT, ap))
    if args.parity and (not firmware or firmware.get('profile_id') != 'arducopter-4.6.3-quad'):
        raise RuntimeError('--parity requires the selected and attested Copter 4.6.3 binary')
    revision = lambda path: subprocess.check_output(['git', '-C', str(path), 'rev-parse', 'HEAD'], text=True).strip()
    native_defaults = copter_default_paths(ap, frame)
    if frame != 'quad':
        if (subprocess.check_output(['git', '-C', str(ROOT), 'status', '--porcelain'], text=True).strip() or
            revision(moos) != '023fb54e00692611ecee2cd6f1138d10eec2127e' or
            fixture.sha(moos/'bin/pHelmIvP') != '5b3d2e705609c37b5b5000384655b4491010d6d797b94c43830d9972e684a876'):
            raise RuntimeError('frame qualification requires a clean source and the selected stock helm')
    out.mkdir(parents=True, exist_ok=False)  # A rerun cannot inherit prior PASS/logs.
    (out/'logs').mkdir()
    copter, probe_path, app = ap/'build/sitl/bin/arducopter', build/'bin/copter_operation_probe', build/'bin/iMAVLink'
    for path in (copter, probe_path, app, moos/'bin/pAntler', moos/'bin/nsplug'):
        if not path.is_file():
            raise RuntimeError('required runnable artifact missing: '+str(path))
    bridge = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    bridge.bind(('127.0.0.1', 0)); bridge.settimeout(.2)
    monitor = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    monitor.bind(('127.0.0.1', 0)); monitor.setblocking(False)
    bridge_port, monitor_port = bridge.getsockname()[1], monitor.getsockname()[1]
    db, base, rc = fixture.port(socket.SOCK_STREAM), fixture.port(socket.SOCK_STREAM), fixture.port(socket.SOCK_DGRAM)
    defaults = out/'fixture.parm'
    fixture_defaults = 'SERIAL1_PROTOCOL 2\nMAV2_POSITION 10\nMAV2_EXT_STAT 2\nMAV2_EXTRA3 2\nLOG_DISARMED 1\nRTL_ALT 500\nRTL_LOIT_TIME 500\nRTL_ALT_FINAL 0\n'
    if args.parity:
        fixture_defaults += ('PLND_ENABLED 1\nPLND_TYPE 4\nSIM_PLD_ENABLE 1\nSIM_PLD_OPTIONS 1\n'
                             f'SIM_PLD_LAT {PRECISION_BEACON[0]}\nSIM_PLD_LON {PRECISION_BEACON[1]}\n')
    defaults.write_text(fixture_defaults)
    (out/'message-intervals-chan1.txt').write_text('33 100\n245 500\n')
    command = [str(copter), '--model', frame, '--home', '42.358456,-71.087589,10,0', '--speedup', '1', '--wipe',
               '--defaults', ','.join(map(str, [*native_defaults, defaults])),
               '--serial0', f'udpclient:127.0.0.1:{bridge_port}', '--serial1', f'udpclient:127.0.0.1:{monitor_port}',
               '--serial2', 'none', '--serial5', 'none', '--serial6', 'none', '--serial7', 'none', '--serial8', 'none',
               '--base-port', str(base), '--rc-in-port', str(rc)]
    source_paths = [Path(__file__), Path(__file__).with_name('CopterOperationProbe.cpp'),
                    ROOT/'src/libmavmoos_control/CopterOperations.cpp', ROOT/'src/libmavmoos_control/ArduCopterBackend.cpp',
                    ROOT/'src/plugins/ArduCopterControlPlugin.cpp', ROOT/'tests/run_sitl.py',ROOT/'tests/copter_frame_fixture.py',
                    ROOT/'tests/owned_processes.py', ROOT/'missions/copter_operations/meta_vehicle.moos',
                    ROOT/'missions/copter_operations/meta_vehicle.bhv']
    if args.parity:
        source_paths.extend((ROOT/'src/libmavmoos_control/CopterCommands.cpp', ROOT/'src/iMAVLink/MAVLink.cpp',
                             ROOT/'src/libmavmoos_runtime/Runtime.cpp'))
    result = {'result': 'FAIL', 'layer': 'owned native ArduCopter + stock pHelmIvP + new public Copter operations',
              'hardware': False, 'firmware_identity': firmware, 'timewarp': 1, 'command': command,
              'revision': revision(ROOT), 'ardupilot_revision': revision(ap), 'moos_revision': revision(moos),
              'copter_sha256': fixture.sha(copter), 'app_sha256': fixture.sha(app), 'probe_sha256': fixture.sha(probe_path),
              'defaults_sha256': fixture.sha(ap/'Tools/autotest/default_params/copter.parm'),
              'fixture_sha256': fixture.sha(defaults),
              'source_sha256': {str(path.relative_to(ROOT)): fixture.sha(path) for path in source_paths},
              'ports': {'bridge': bridge_port, 'monitor': monitor_port, 'db': db, 'base': base, 'rc': rc}}
    if args.parity:
        result['parity'] = True
    protected = {}
    if frame != 'quad':
        paths = [*source_paths, *native_defaults, defaults, copter, app, probe_path, moos/'bin/pHelmIvP',
                 moos/'bin/MOOSDB', moos/'bin/pAntler', moos/'bin/nsplug', manifest, args.firmware_attestation,
                 *(build/'plugins'/f'mavmoos_{name}.so' for name in ('telemetry', 'parameters', 'arducopter_control'))]
        protected = {str(path.resolve()): fixture.sha(path) for path in paths}
        result.update(frame=frame, frame_identity={'expected_mav_type': expected_type,
                                                  'expected_frame_class': expected_class},
                      protected_inputs_sha256=protected)
        (out/'firmware-manifest.json').write_bytes(manifest.read_bytes())
        (out/'firmware-attestation.json').write_bytes(args.firmware_attestation.read_bytes())
    sitl = launch = probe = pilot = None
    handles, packets, observed = [], [], {}
    pilot_ready = out/'fixture-pilot-ready.txt'
    try:
        log = (out/'sitl.log').open('w'); handles.append(log)
        sitl = subprocess.Popen(command, cwd=out, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        decoder = mavutil.mavlink.MAVLink(None); decoder.robust_parsing = True
        deadline, peer = time.monotonic()+20, None
        while time.monotonic() < deadline:
            if sitl.poll() is not None:
                raise RuntimeError('owned SITL exited during discovery')
            try:
                data, address = bridge.recvfrom(4096)
            except socket.timeout:
                continue
            for message in decoder.parse_buffer(data) or []:
                if (message.get_type() == 'HEARTBEAT' and message.get_srcSystem() == 1 and message.get_srcComponent() == 1
                    and (message.type, message.autopilot) == (expected_type, 3)):
                    peer = address
                    break
            if peer:
                break
        if peer is None or peer[0] != '127.0.0.1':
            raise RuntimeError('exact owned native loopback peer not discovered')
        sockets = subprocess.run(['lsof', '-nP', '-a', '-p', str(sitl.pid), '-iUDP'], capture_output=True, text=True, timeout=3)
        if sockets.returncode or not re.search(r'127\.0\.0\.1:'+str(peer[1])+r'->127\.0\.0\.1:'+str(bridge_port)+r'\b', sockets.stdout):
            raise RuntimeError('discovered peer socket is not proven to belong to the owned native process')
        result.update(sitl_pid=sitl.pid, sitl_udp_sockets=sockets.stdout)
        result['ports']['peer'] = peer[1]
        bridge.close()
        macros = {'MOOS_PORT': str(db), 'APP_PATH': str(app), 'BIND_PORT': str(bridge_port), 'PEER_PORT': str(peer[1]),
                  'LAT_ORIGIN': '42.358456', 'LONG_ORIGIN': '-71.087589', 'SIMULATION_ONLY': 'true',
                  'SHADOW_MODE': 'false', 'SITL_PID': str(sitl.pid), 'PLUGIN_DIRECTORY': str(build/'plugins'),
                  'BEHAVIOR_DIRECTORY': str(Path(args.behavior_dir).resolve()), 'HEIGHT': '5', 'SPEED': '1',
                  'WAYPOINTS': '0,0', 'FACING_POLICY': 'course', 'FACING_HEADING': '0'}
        env = dict(os.environ)
        env['PATH'] = os.pathsep.join([str(build/'bin'), str(moos/'bin'), env.get('PATH', '')])
        env['IVP_BEHAVIOR_DIRS'] = macros['BEHAVIOR_DIRECTORY']
        env['MAVMOOS_SITL_FIXTURE_PILOT_READY'] = str(pilot_ready)
        for suffix in ('moos', 'bhv'):
            generated = out/f'targ_copter.{suffix}'
            subprocess.run([str(moos/'bin/nsplug'), str(ROOT/f'missions/copter_operations/meta_vehicle.{suffix}'),
                            str(generated), '--strict', '--force', '-x']+[f'{k}={v}' for k, v in macros.items()],
                           check=True, env=env, capture_output=True, timeout=5)
        if frame != 'quad':
            generated = out/'targ_copter.moos'
            generated.write_text(bind_copter_frame(generated.read_text(), frame))
        if args.parity:
            generated = out/'targ_copter.moos'
            settings = ('\n  control_copter_adjustments = true\n  command_speed_min = 1.5\n'
                        '  control_precision_loiter = true\n  dead_man_source = mavmoos_copter_operations\n'
                        '  operation_helm_tasks = survey,voronoi\n  operation_helm_adjustments = true')
            updated, count = re.subn(r'(?m)^[ \t]*operation_helm_enabled[ \t]*=[ \t]*true[ \t]*$',
                                     lambda match: match.group() + settings, generated.read_text())
            if count != 1:
                raise RuntimeError('parity config requires one existing operation helm opt-in anchor')
            generated.write_text(updated)
            behaviors = out/'targ_copter.bhv'
            behaviors.write_text(parity_behaviors(behaviors.read_text()))
        log = (out/'pantler.log').open('w'); handles.append(log)
        launch = subprocess.Popen([str(moos/'bin/pAntler'), str(out/'targ_copter.moos')], cwd=out, env=env,
                                  stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        log = (out/'workflow.log').open('w'); handles.append(log)
        probe_command = [str(probe_path), str(db)]
        if args.parity:
            probe_command.extend(['--parity', *(str(coord) for coord in PRECISION_BEACON)])
        probe = subprocess.Popen(probe_command, cwd=out, env=env, stdout=log,
                                 stderr=subprocess.STDOUT, start_new_session=True)
        pilot = fixture.FixturePilot(sitl, rc, pilot_ready, out/'workflow.log', out/'fixture-pilot.jsonl')
        result['independent_monitor_writes'] = 'none'
        result['fixture_pilot_writes'] = 'exact owned native RC socket; airborne neutral throttle for native Loiter only; explicit restore'
        decoder = mavutil.mavlink.MAVLink(None); decoder.robust_parsing = True
        with (out/'fc-monitor.jsonl').open('w') as trace, (out/'fc-monitor.mavlink').open('wb') as raw:
            deadline, drain = time.monotonic()+310, None
            while time.monotonic() < deadline:
                if sitl.poll() is not None or launch.poll() is not None:
                    raise RuntimeError('owned simulator/MOOS launcher exited during workflow')
                if probe.poll() is not None:
                    if drain is None:
                        pilot.restore('operation probe exited; receive-only drain')
                        drain = time.monotonic()+1.5
                    if time.monotonic() >= drain:
                        break
                else:
                    pilot.update(observed)
                try:
                    data, sender = monitor.recvfrom(65535)
                except BlockingIOError:
                    time.sleep(.02)
                    continue
                if sender[0] != '127.0.0.1':
                    raise RuntimeError('non-loopback sender on owned observer')
                if frame != 'quad':
                    if 'monitor_peer' not in result['ports']:
                        sockets = subprocess.run(['lsof', '-nP', '-a', '-p', str(sitl.pid), '-iUDP'],
                                                 capture_output=True, text=True, timeout=3, check=True)
                        if not re.search(r'127\.0\.0\.1:'+str(sender[1])+r'->127\.0\.0\.1:'+str(monitor_port)+r'\b', sockets.stdout):
                            raise RuntimeError('independent observer source is not owned')
                        result['ports']['monitor_peer'] = sender[1]
                        result['native_observer_socket_inventory'] = sockets.stdout
                    if sender[1] != result['ports']['monitor_peer']:
                        raise RuntimeError('independent observer source changed')
                raw.write(data)
                for message in decoder.parse_buffer(data) or []:
                    if message.get_srcSystem() == 1 and message.get_srcComponent() == 1 and message.get_type() in ('HEARTBEAT', 'GLOBAL_POSITION_INT', 'EXTENDED_SYS_STATE', 'STATUSTEXT'):
                        if message.get_type() == 'HEARTBEAT' and (message.type, message.autopilot) != (expected_type, 3):
                            raise RuntimeError('independent heartbeat differs from selected Copter frame')
                        item = {'received_mono': time.monotonic(), **message.to_dict()}
                        trace.write(json.dumps(item)+'\n'); trace.flush(); packets.append(item)
                        observed[message.get_type()] = item
            result['monitor_finished_mono'] = time.monotonic()
            if probe.poll() is None:
                raise RuntimeError('operation workflow exceeded310s')
        result['probe_exit'] = probe.returncode
        if probe.returncode != 0:
            raise RuntimeError('public operation probe failed; see workflow.log')
        trace = events(out/'workflow.log')
        result['capabilities_catalog'] = catalog_evidence(trace, args.parity)
        result['loaded_modules_sha256'] = loaded_module_hashes(build/'plugins', result['capabilities_catalog'])
        result['operations'] = operation_evidence(trace, args.parity)
        if args.parity:
            result['parity_evidence'] = parity_evidence(trace)
            result['parity_helm'] = helm_parity_evidence(trace)
        result['independent_fc'] = flight_evidence(packets, trace, result['monitor_finished_mono'],
                                                   PRECISION_BEACON if args.parity else None)
        result['result'] = 'PASS'
    except Exception as error:
        result['error'] = str(error)
    finally:
        if pilot is not None:
            try:
                pilot.close()
            except Exception as error:
                result.update(result='FAIL', pilot_cleanup_error=str(error))
        remaining = []
        for process in (probe, launch, sitl):
            remaining.extend(fixture.stop(process))
        bridge.close(); monitor.close()
        for handle in handles:
            handle.close()
        clear = {}
        for key in ('bridge', 'monitor', 'rc', 'peer', 'monitor_peer'):
            if key not in result['ports']:
                continue
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                try:
                    sock.bind(('127.0.0.1', result['ports'][key])); clear[key] = True
                except OSError:
                    clear[key] = False
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(.2); clear['moos_tcp'] = sock.connect_ex(('127.0.0.1', db)) != 0
        if frame != 'quad':
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.settimeout(.2); clear['base_tcp'] = sock.connect_ex(('127.0.0.1', base)) != 0
        groups = {process.pid for process in (probe, launch, sitl) if process is not None}
        for line in subprocess.check_output(['ps', '-axo', 'pid,pgid,comm'], text=True, timeout=3).splitlines()[1:]:
            values = line.strip().split(None, 2)
            if len(values) == 3 and int(values[1]) in groups:
                remaining.append(line.strip())
        result['ports_clear'], result['owned_processes_remaining'] = clear, remaining
        if not all(clear.values()) or remaining:
            result.update(result='FAIL', cleanup_error='owned process or selected port remains')
        if result['result'] == 'PASS':
            try:
                selected = fixture.select_run_dataflash_log(out/'logs', set())
                original = fixture.sha(selected)
                proof = fixture.pilot_rcin_evidence(selected)
                if args.parity:
                    result['independent_precision_loiter'] = precision_motion_evidence(selected)
                if frame != 'quad':
                    result['frame_identity']['dataflash_parameters'] = one_copter_frame_log(out/'logs', frame)
                if fixture.sha(selected) != original:
                    raise RuntimeError('new owned DataFlash input changed during analysis')
                result['independent_pilot_rcin'] = {'path': str(selected), 'sha256': original,
                                                    'prelaunch_BIN_names': [], **proof}
            except Exception as error:
                result.update(result='FAIL', error=str(error))
        if protected:
            result['cleanup_proven'] = (not remaining and all(clear.values()) and
                set(clear) == {'bridge', 'monitor', 'rc', 'peer', 'monitor_peer', 'moos_tcp', 'base_tcp'})
            result['artifact_drift'] = [path for path, digest in protected.items() if fixture.sha(path) != digest]
            if not result['cleanup_proven'] or result['artifact_drift'] or revision(ROOT) != result['revision']:
                result.update(result='FAIL', postflight_error='protected source, artifacts or cleanup changed')
        result['artifact_sha256'] = {str(path.relative_to(out)): fixture.sha(path)
                                     for path in sorted(out.rglob('*')) if path.is_file() and path.name != 'result.json'}
        (out/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    return 0 if result['result'] == 'PASS' else 1


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(143)))
    with (Path(tempfile.gettempdir())/'mavmoos-native-sitl.lock').open('w') as lease:
        fcntl.flock(lease, fcntl.LOCK_EX)
        raise SystemExit(main())
