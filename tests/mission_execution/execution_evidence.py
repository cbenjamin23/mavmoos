"""Offline gates for explicit AUTO execution, independent handback and landing.

Probe elapsed, actual Python receive and FC boot clocks stay separate. Passive
MAVLink-1 progress has no extension state; missing extensions are never decoded
as evidence of NOT_STARTED/COMPLETE. Primary execution checks and independent
FC navigation effects have deliberately distinct evidence roles.
"""
import hashlib
import json
import math
import re

SCENARIOS = {'auto_complete_handback': {'supervisor_s': 280},
             'auto_pause_abort': {'supervisor_s': 240},
             'auto_typed_lifecycle': {'supervisor_s': 185}}
TYPED_MARKERS = ('upload.started', 'upload.finished', 'takeoff.started', 'auto.started',
                 'takeoff.finished', 'auto.paused', 'resume.requested', 'auto.resumed', 'land.finished',
                 'abort.started', 'abort.finished', 'finished')
COMMON_MARKERS = ('upload.started', 'upload.finished', 'takeoff.started', 'takeoff.finished',
                  'auto.started', 'auto.finished', 'abort.started', 'abort.finished',
                  'return.started', 'return.arrived', 'return.finished', 'land.started', 'land.finished', 'finished')


def require(value, reason):
    if not value:
        raise ValueError(reason)


def number(value):
    require(type(value) in (int, float) and math.isfinite(value), 'finite number required')
    return value


def parse(data):
    def pairs(rows):
        result = {}
        for key, value in rows:
            require(key not in result, 'duplicate JSON key')
            result[key] = value
        return result
    def bad(value):
        raise ValueError('nonfinite JSON ' + value)
    return json.loads(data, object_pairs_hook=pairs, parse_constant=bad)


def fields(text):
    result = {}
    for part in text.split(','):
        key, separator, value = part.partition('=')
        require(separator and key and key not in result, 'malformed/duplicate KV')
        result[key] = value
    return result


def yes(value):
    require(value in ('true', 'false', '1', '0'), 'invalid boolean field')
    return value in ('true', '1')


def markers(scenario):
    require(scenario in SCENARIOS, 'unknown AUTO scenario')
    if scenario == 'auto_typed_lifecycle':
        return TYPED_MARKERS
    return COMMON_MARKERS[:5] + (('auto.completed',) if scenario == 'auto_complete_handback'
                                 else ('auto.paused',)) + COMMON_MARKERS[5:]


def phase(events, key):
    matches = [e for e in events if e['type'] == 'phase' and e['key'] == key]
    require(len(matches) == 1, 'missing/duplicate phase ' + key)
    return number(matches[0]['elapsed'])


def phase_ack_observation(original,ack,request,owner):
    published=number(request['local_publication_attempt_elapsed']);expires=number(request['local_original_expiry_elapsed'])
    timeout=number(float(fields(request['value'])['timeout']))
    require(request.get('local_clock_scope')=='probe_publication_attempt_not_host_expiry' and
            0<=published<=number(request['elapsed']) and abs(expires-published-timeout)<=1e-6,'original probe-local publication deadline invalid')
    started=number(ack['wait_started_elapsed']);deadline=number(ack['deadline_elapsed']);observed=number(ack['ack_observed_elapsed'])
    require(ack.get('original_request_id')==owner and
            abs(number(ack['local_publication_attempt_elapsed'])-published)<=1e-6 and
            abs(number(ack['local_original_expiry_elapsed'])-expires)<=1e-6 and
            published<=started<=number(original['elapsed'])<=observed<=number(ack['elapsed']) and
            abs(deadline-min(started+3,expires))<=1e-6 and observed<deadline,'phase acknowledgement escaped original local deadline')
    return observed


def audit_probe(events, scenario):
    require(events and all(type(e) is dict and number(e['elapsed']) >= 0 for e in events), 'invalid probe trace')
    require(all(b['elapsed'] >= a['elapsed'] for a, b in zip(events, events[1:])), 'probe trace reordered')
    require(not any(e['type'] == 'fail' for e in events), 'probe reported failure')
    passed = [e for e in events if e['type'] == 'pass']
    require(len(passed) == 1 and passed[0]['key'] == scenario and passed[0]['source'] == 'probe', 'positive scenario receipt missing')
    times = {key: phase(events, key) for key in markers(scenario)}
    require(all(times[b] > times[a] for a, b in zip(markers(scenario), markers(scenario)[1:])), 'phase order invalid')
    require(times['upload.finished'] < times['takeoff.started'] < times['auto.started'], 'upload granted implicit execution')
    results = {};requests = [];proofs = [];snapshots = [];progress = []
    for e in events:
        if e['type'] in ('control_result', 'mission_result', 'execution_result'):
            require(e['source'] == 'iMAVLink', 'foreign result publisher')
            f = fields(e['value']);number(float(f['expires']));require(float(f['expires']) > 0, 'invalid original expiry')
            values = results.setdefault(f['id'], []);require(not values or f['expires'] == values[0][1]['expires'], 'result original expiry renewed')
            values.append((e, f))
        elif e['type'] == 'request':
            require(e['source'] == 'probe', 'foreign request trace')
            if e['key'] in ('MAVMOOS_CONTROL_REQUEST', 'MAVMOOS_MISSION_REQUEST', 'MAVMOOS_MISSION_CONTROL'):
                requests.append((e, fields(e['value'])))
        elif e['type'] == 'proof':
            require(e['source'] == 'iMAVLink' and e['key'] == 'MAVMOOS_STORED_MISSION_PROOF', 'foreign proof publication')
            f = fields(e['value'])
            if f.get('valid') == 'true':proofs.append((e, f))
        elif e['type'] == 'snapshot':
            require(e['source'] == 'iMAVLink', 'foreign mission snapshot')
            snapshot = parse(e['value'])
            if snapshot.get('id') == 'upload' and snapshot.get('status') == 'complete':snapshots.append((e, snapshot))
        elif e['type'] == 'execution_progress':
            require(e['source'] == 'iMAVLink', 'foreign mission progress')
            f = fields(e['value']);require(f['id'] == 'run' and f['total'] == '2' and f['wire_operation_id'] == 'false' and f['source_age'] == 'unknown', 'unbound progress')
            require(number(float(f['effective_at'])) <= number(float(f['read_at'])), 'progress was restamped')
            progress.append((e, f))
    def states(key):return {f['status'] for _, f in results.get(key, [])}
    for key in ('guided', 'arm', 'takeoff', 'return', 'land'):
        require('COMPLETED' in states(key), 'lifecycle operation incomplete ' + key)
        require(not states(key) & {'FAILED', 'REJECTED', 'LOST', 'TIMED_OUT'}, 'lifecycle operation failed ' + key)
    require('COMPLETED' in states('upload'), 'upload did not finish')
    require(len(snapshots) == 1 and snapshots[0][1]['count'] == 3 and
            snapshots[0][1]['verification'] == 'semantic_profile_match' and
            len(snapshots[0][1]['items']) == 3, 'verified three-row stored plan missing')
    plan = snapshots[0][1]['items']
    require([i['seq'] for i in plan] == [0, 1, 2] and all(i['command'] == 16 and i['autocontinue'] == 1 for i in plan), 'stored plan shape differs')
    require(plan[0]['frame'] == 0 and all(i['frame'] == 3 and i['z'] == 5 for i in plan[1:]), 'stored plan reference differs')
    require(all(i['params'] == [0, 0, 0, 0] for i in plan), 'stored plan parameters differ')
    uploads = [f for e, f in requests if e['key'] == 'MAVMOOS_MISSION_REQUEST' and f['action'] == 'upload']
    require(len(uploads) == 1 and uploads[0]['id'] == 'upload', 'missing/repeated upload request')
    requested = [row.split(':') for row in uploads[0]['items'].split('|')]
    require(len(requested) == 3 and all(len(row) == 12 for row in requested), 'invalid declared upload')
    require(all(int(row[0]) == i['seq'] and int(row[1]) == i['command'] and int(row[2]) == i['frame'] and
                int(row[9]) == i['x'] and int(row[10]) == i['y'] and math.isclose(float(row[11]), i['z'], abs_tol=.010001)
                for row, i in zip(requested[1:], plan[1:])), 'executable stored rows differ from declared upload')
    starts = [(e, f) for e, f in requests if e['key'] == 'MAVMOOS_MISSION_CONTROL' and f['action'] == 'start']
    require(len(starts) == 1 and starts[0][1]['id'] == 'run', 'missing/repeated explicit start')
    start, selection = starts[0]
    selected = [(e, p) for e, p in proofs if e['elapsed'] < start['elapsed'] and p.get('verification_id') == selection['verification_id'] and p.get('content_id') == selection['content_id'] and p.get('revision') == selection['proof_revision']]
    require(selected, 'explicit start selectors did not match host publication')
    proof = selected[-1][1]
    require(proof['schema'] == '1' and proof['count'] == '3' and proof['atomic'] == 'false' and proof['external_writer_excluded_by_profile'] == 'true' and int(proof['session_epoch']) > 0, 'proof scope differs')
    require({'ADMITTED', 'SUBMITTED', 'SENT', 'ACTIVE'} <= states('run') and
            states('run') & {'ACK_OBSERVED', 'ACKNOWLEDGED'}, 'execution stages missing')
    require(sum(f['status'] == 'SENT' for _, f in results['run']) >= 2, 'selection and start handoffs missing')
    for _, f in results['run']:
        require(f['content_id'] == proof['content_id'] and f['verification_id'] == proof['verification_id'] and f['proof_revision'] == proof['revision'] and f['session_epoch'] == proof['session_epoch'], 'execution result lost proof binding')
        require(f['wire_operation_id'] == 'false', 'wire operation identity invented')
    require(not states('run') & {'FAILED', 'REJECTED', 'LOST', 'TIMED_OUT'}, 'execution failed')
    require(progress and any(f['seq'] == '1' and f['mission_state'] == '3' for _, f in progress), 'post-start first ACTIVE progress missing')
    if scenario == 'auto_complete_handback':
        require('COMPLETED' in states('run') and 'CANCELLED' not in states('run'), 'completed parent was cancelled/reclassified')
        require(any(f['seq'] == '2' and f['mission_state'] == '5' for _, f in progress), 'COMPLETE final progress missing')
    else:
        require({'PAUSED', 'CANCELLED'} <= states('run') and 'COMPLETED' not in states('run'), 'pause/abort incorrectly qualified mission completion')
        require('COMPLETED' in states('pause') and any(f['mission_state'] == '4' for _, f in progress), 'observed pause missing')
    require('COMPLETED' in states('abort'), 'observed abort handback missing')
    acknowledgements = [e for e in events if e['type'] == 'assertion' and e['key'] == 'auto_finish_ack']
    require(len(acknowledgements) == 1 and times['auto.finished'] <= acknowledgements[0]['elapsed'] < times['abort.started'], 'abort preceded causal AUTO finish acknowledgement')
    ack = fields(acknowledgements[0]['value']);require(0 <= number(float(ack['wait_s'])) < 3 and ack['timeout_s'] == '3', 'late finish acknowledgement')
    window = [e for e in events if times['auto.started'] <= e['elapsed'] <= times['auto.finished']]
    runtime = [fields(e['value']) for e in window if e['type'] == 'runtime']
    require(len(runtime) >= 5 and all(f['fc_mode'] == '3' and yes(f['armed']) and yes(f['in_air']) and not yes(f['permission']) for f in runtime), 'AUTO mode/arming/local permission continuity failed')
    require(len({f['nav_tx_accepted'] for f in runtime}) == 1, 'Guided targets continued during AUTO')
    leases = [fields(e['value']) for e in window if e['type'] == 'execution_status']
    require(leases and all(f['lease_retained'] == 'true' for f in leases), 'AUTO execution lease released early')
    requests_by_id = {f['id']: e for e, f in requests}
    expected_boundaries={'auto.completed' if scenario=='auto_complete_handback' else 'auto.paused','auto.finished','return.arrived','return.finished'}
    boundary_tokens={};acknowledged={};ack_events={}
    for key in expected_boundaries:
        original=next(e for e in events if e['type']=='phase' and e['key']==key)
        rows=[e for e in events if e['type']=='phase_ack' and e['key']==key]
        require(len(rows)==1 and rows[0]['source']=='probe','missing/duplicate/foreign phase acknowledgement')
        ack=rows[0];token=ack['value']
        require(type(token) is str and len(token)<=128 and re.fullmatch(r'phase:'+re.escape(key)+r':[1-9][0-9]{0,3}:[0-9]+(?:\.[0-9]+)?\n',token) is not None,'phase acknowledgement token invalid')
        owner='run' if key.startswith('auto.') else 'return'
        acknowledged[key]=phase_ack_observation(original,ack,requests_by_id[owner],owner);boundary_tokens[key]=token;ack_events[key]=ack['elapsed']
    require({e['key'] for e in events if e['type']=='phase_ack'}==expected_boundaries,'unrequested phase acknowledgement')
    held='auto.completed' if scenario=='auto_complete_handback' else 'auto.paused'
    require(times['auto.finished']-acknowledged[held]>=3,'AUTO hold began before matching start acknowledgement')
    require(times['abort.started']>ack_events['auto.finished'],'abort preceded acknowledged AUTO boundary')
    require(times['return.finished']-acknowledged['return.arrived']>=3,'return observation dwell shorter than declared3s')
    require(times['land.started']>ack_events['return.finished'],'LAND preceded acknowledged return boundary')
    return_request=fields(requests_by_id['return']['value'])
    require(selection.get('timeout')=='90' and return_request.get('action')=='return' and return_request.get('strategy')=='helm' and return_request.get('timeout')=='45' and float(return_request.get('radius','nan'))==1.5 and float(return_request.get('height','nan'))==5 and float(return_request.get('speed','nan'))==1.5,'declared AUTO/return request profile changed')
    completed_returns=[e['elapsed'] for e,f in results['return'] if f['status']=='COMPLETED']
    require(completed_returns and max(completed_returns)<=times['return.arrived'],'return observation preceded operation completion')
    observed=[e for e in events if e['type']=='runtime' and acknowledged['return.arrived']<=e['elapsed']<=times['return.finished']]
    require(len(observed)>=5 and max(b['elapsed']-a['elapsed'] for a,b in zip(observed,observed[1:]))<=.75,'return observation Runtime evidence missing/gapped')
    require(all(fields(e['value'])['fc_mode']=='4' and yes(fields(e['value'])['armed']) and yes(fields(e['value'])['in_air']) and not yes(fields(e['value'])['permission']) and all(yes(fields(e['value'])[k]) for k in ('heartbeat_valid','position_valid','home_valid','landed_valid','estimator_valid')) for e in observed),'return observation lost fresh Guided state')
    released=[fields(e['value']) for e in events if e['type']=='execution_status' and acknowledged['return.arrived']<=e['elapsed']<=times['return.finished']]
    require(released and all(f['lease_retained']=='false' for f in released),'AUTO ownership retained during return observation')
    require(requests_by_id['return']['elapsed'] > times['abort.finished'], 'helm return started before observed handback')
    require(any(e['type'] == 'helm_scalar' and times['return.started'] < e['elapsed'] < times['return.finished'] for e in events), 'unchanged stock helm did not supply return objectives')
    final = [fields(e['value']) for e in events if e['type'] == 'runtime' and e['elapsed'] >= times['land.finished']]
    require(len(final) >= 2 and all(not yes(f['armed']) and not yes(f['in_air']) and not yes(f['permission']) for f in final), 'final ground/disarmed state missing')
    return {'result': 'PASS', 'scenario': scenario, 'proof_selectors': proof,
            'stored_plan': plan, 'operation_statuses': {k: sorted(states(k)) for k in results},
            'phases_probe_elapsed': times, 'ack_probe_elapsed': acknowledgements[0]['elapsed'],'boundary_tokens':boundary_tokens,
            'return_observation_probe_elapsed':[acknowledged['return.arrived'],times['return.finished']],
            'mission_completed': scenario == 'auto_complete_handback', 'paused_then_cancelled': scenario == 'auto_pause_abort',
            'automatic_helm_reacquisition': False, 'wire_operation_id': False}


def audit_boundary(events, marker, acknowledgement):
    require(type(marker) is bytes and 0 < len(marker) <= 128 and marker == acknowledgement, 'AUTO finish marker/ack mismatch')
    finish = [e for e in events if e['kind'] == 'auto.finished'];acks = [e for e in events if e['kind'] == 'auto.finished.ack']
    require(len(finish) == len(acks) == 1 and events.index(acks[0]) > events.index(finish[0]) and
            number(acks[0]['python_mono']) >= number(finish[0]['python_mono']) and
            acks[0]['token_sha256'] == hashlib.sha256(marker).hexdigest(), 'coordinator causal boundary missing')
    return {'result': 'PASS', 'python_finish_boundary': finish[0]['python_mono'], 'python_ack_written': acks[0]['python_mono'], 'token_sha256': acks[0]['token_sha256'], 'clock_conversion': False}


def distance(lat, lon, x, y):
    return 6371000 * math.hypot(math.radians(x-lat), math.radians(y-lon)*math.cos(math.radians(lat)))


def audit_fc(packets, times, finished, scenario, probe):
    require(set(times) == set(markers(scenario)), 'independent phase bracket missing')
    require(all(p['system'] == p['component'] == 1 and number(p['mono']) >= 0 for p in packets), 'foreign/invalid passive sample')
    begin, end = number(times['auto.started']), number(times['auto.finished'])
    require(end > begin and number(finished) > end, 'AUTO passive window invalid')
    def rows(kind, a=begin, b=end):return [p for p in packets if p['mavpackettype'] == kind and a <= p['mono'] <= b]
    heart = rows('HEARTBEAT');position = rows('GLOBAL_POSITION_INT');current = rows('MISSION_CURRENT')
    require(len(heart) >= 2 and all(p['custom_mode'] == 3 and p['base_mode'] & 128 for p in heart), 'independent AUTO/arming continuity failed')
    require(position and max([position[0]['mono']-begin, end-position[-1]['mono']] + [b['mono']-a['mono'] for a, b in zip(position, position[1:])]) <= .5, 'independent position reception gap')
    require(all(3.5 <= p['relative_alt']/1000 <= 6.5 for p in position), 'AUTO height outside declared flight envelope')
    require(current and any(p['seq'] == 1 for p in current), 'independent first mission cursor missing')
    extended = [p for p in current if p.get('wire_magic') == 253 and p.get('wire_payload_length', 0) >= 5]
    require(all(p['total'] == 2 for p in extended), 'independent extended mission count differs')
    if scenario == 'auto_complete_handback':
        require(any(p['seq'] == 2 for p in current) and any(p['seq'] == 2 for p in rows('MISSION_ITEM_REACHED')), 'independent final cursor/reached missing')
        stop = times['auto.completed'];last = probe['stored_plan'][-1]
    else:stop = times['auto.paused'];last = None
    late = rows('GLOBAL_POSITION_INT', max(stop, end-2), end)
    require(len(late) >= 8 and late[-1]['mono']-late[0]['mono'] >= 1.2 and
            max(math.hypot(p['vx'],p['vy'])/100 for p in late) <= .5, 'independent final/paused hold not settled')
    if last is not None:
        require(all(distance(p['lat']/1e7,p['lon']/1e7,last['x']/1e7,last['y']/1e7) <= 2.5 and abs(p['relative_alt']/1000-last['z']) <= .6 for p in late), 'independent final arrival differs')
        require(distance(position[0]['lat']/1e7,position[0]['lon']/1e7,late[-1]['lat']/1e7,late[-1]['lon']/1e7) >= 6, 'independent AUTO route displacement missing')
    else:
        require(max(distance(late[0]['lat']/1e7,late[0]['lon']/1e7,p['lat']/1e7,p['lon']/1e7) for p in late) <= 2, 'paused AUTO drifted')
    after = rows('HEARTBEAT', times['abort.finished'], times['return.started'])
    # The explicit post-abort phase can be shorter than a heartbeat period;
    # require fresh independent Guided before the end of the subsequent return.
    after += rows('HEARTBEAT', times['return.started'], times['return.finished'])
    require(any(p['custom_mode'] == 4 and p['base_mode'] & 128 for p in after), 'independent Guided handback missing')
    home = probe['stored_plan'][0]
    returned = rows('GLOBAL_POSITION_INT', max(times['return.started'],times['return.finished']-2),times['return.finished'])
    require(len(returned) >= 5 and all(distance(p['lat']/1e7,p['lon']/1e7,home['x']/1e7,home['y']/1e7) <= 2.5 and abs(p['relative_alt']/1000-5) <= .6 for p in returned), 'independent helm return arrival missing')
    return_heart=rows('HEARTBEAT',times['return.finished']-2,times['return.finished'])
    require(return_heart and all(p['custom_mode']==4 and p['base_mode']&128 for p in return_heart),'independent return observation left armed Guided before acknowledgement')
    final_heart = rows('HEARTBEAT', times['land.finished'], finished)
    final_landed = rows('EXTENDED_SYS_STATE', times['land.finished'], finished)
    final_position = rows('GLOBAL_POSITION_INT', times['land.finished'], finished)
    require(len(final_heart) >= 2 and all(not p['base_mode'] & 128 for p in final_heart) and
            final_landed and all(p['landed_state'] == 1 for p in final_landed) and final_position and
            all(abs(p['relative_alt']/1000) <= .5 for p in final_position), 'independent final disarmed/on-ground missing')
    return {'result': 'PASS', 'auto_window': [begin, end], 'late_hold_samples': len(late),
            'passive_extended_progress_samples': len(extended), 'passive_extension_scope': 'only actual MAVLink2 payloads qualify extension fields',
            'mission_completed': scenario == 'auto_complete_handback', 'final_disarmed': True, 'final_on_ground': True,
            'source_age': 'unknown', 'receiver_clock': 'actual recvfrom completion; no retiming'}


def audit_dataflash(arms, modes, commands, params, fixture):
    require(all(k in params and math.isclose(params[k],v,rel_tol=1e-5,abs_tol=1e-4) for k,v in fixture.items()), 'closed fixture parameter confirmation missing')
    require(any(p['ArmState'] == 1 for p in arms) and any(p['ArmState'] == 0 for p in arms), 'closed arm/disarm records missing')
    values = [int(p['ModeNum']) for p in modes]
    require(3 in values and 4 in values and 9 in values and values.index(3) < max(i for i,x in enumerate(values) if x == 4) < values.index(9), 'closed AUTO/Guided/LAND order missing')
    observed = {int(p['CNum']) for p in commands if int(p['CTot']) == 3 and int(p['CId']) == 16}
    require({1, 2} <= observed, 'closed stored waypoint records missing')
    return {'result': 'PASS', 'closed_record_projection': True, 'whole_bin_framing_qualified': False,
            'uploaded_waypoint_sequences': sorted(observed), 'mode_order': values,
            'scope': 'closed PARM/ARM/MODE/CMD semantic projection; raw BIN retained; no whole-BIN framing claim'}


def audit_typed_probe(events):
    require(events and not any(e['type'] == 'fail' for e in events), 'typed MOOS probe reported failure')
    passed = [e for e in events if e['type'] == 'pass' and e['key'] == 'auto_typed_lifecycle' and e['source'] == 'probe']
    require(len(passed) == 1, 'typed MOOS pass receipt missing')
    times = {key: phase(events, key) for key in TYPED_MARKERS}
    require(all(times[b] > times[a] for a, b in zip(TYPED_MARKERS, TYPED_MARKERS[1:])), 'typed probe phases reordered')
    snapshots = [parse(e['value']) for e in events if e['type'] == 'snapshot' and e['source'] == 'iMAVLink']
    plans = [p for p in snapshots if p.get('id') == 'upload' and p.get('verification') == 'semantic_profile_match']
    require(len(plans) == 1 and [i['command'] for i in plans[0]['items']] == [16, 22, 16, 16, 21] and
            plans[0]['items'][2]['params'][0] == 3, 'typed mission readback differs')
    route = [fields(e['value']) for e in events if e['type'] == 'runtime' and e['source'] == 'iMAVLink'
             and times['takeoff.started'] <= e['elapsed'] <= times['finished']]
    require(route and len({p['nav_tx_accepted'] for p in route}) == 1 and
            all(p['permission'] == 'false' for p in route), 'native mission allowed competing helm output')
    outcomes = [fields(e['value']) for e in events if e['type'] == 'execution_result' and e['source'] == 'iMAVLink']
    for id_, reason in (('pause', 'observed_auto_pause_not_mode_exit'), ('resume', 'observed_active_same_cursor'),
                        ('run', 'bound_landing_and_disarmed_dwell_observed_lease_retained'),
                        ('abort', 'observed_guided_no_automatic_helm_acquire')):
        require(sum(p['id'] == id_ and p['status'] == 'COMPLETED' and p['reason'] == reason for p in outcomes) == 1,
                f'typed {id_} terminal receipt missing')
    return {'result': 'PASS', 'phases_probe_elapsed': times, 'stored_plan': plans[0]['items'],
            'helm_tx_counter': route[0]['nav_tx_accepted']}


def audit_typed_fc(packets, times, finished, plan):
    def rows(kind, begin, end):
        return [p for p in packets if p['mavpackettype'] == kind and begin <= p['mono'] <= end]

    def error(position, item):
        return distance(position['lat']*1e-7, position['lon']*1e-7, item['x']*1e-7, item['y']*1e-7)

    def ground_speed(position):
        return math.hypot(position['vx'], position['vy'])/100

    lift = rows('GLOBAL_POSITION_INT', times['takeoff.started'], times['takeoff.finished'])
    lift_progress = rows('MISSION_CURRENT', times['takeoff.started'], times['takeoff.finished'])
    require(lift and min(p['relative_alt'] for p in lift) <= 500 and max(p['relative_alt'] for p in lift) >= 4400 and
            any(p['seq'] == 1 and p['mission_state'] == 3 for p in lift_progress),
            'independent ground-to-air climb under native takeoff cursor missing')
    quiet = rows('GLOBAL_POSITION_INT', times['auto.paused'], times['resume.requested'])
    paused = rows('MISSION_CURRENT', times['auto.paused'], times['resume.requested'])
    resume = [p for p in rows('MISSION_CURRENT', times['resume.requested'], times['auto.resumed'] + 1)
              if p['mission_state'] == 3]
    require(resume and any(p['mission_state'] == 4 and p['seq'] == resume[0]['seq'] for p in paused),
            'independent paused then active same cursor missing')
    require(len(quiet) >= 8 and quiet[-1]['mono']-quiet[0]['mono'] >= 1 and max(map(ground_speed, quiet)) <= .5,
            'independent paused motion did not settle')
    route = rows('GLOBAL_POSITION_INT', times['auto.resumed'], times['land.finished'])
    progress = rows('MISSION_CURRENT', times['auto.resumed'], times['land.finished'])
    forward = next((p for p in progress if p['seq'] == 3 and p['mission_state'] == 3), None)
    dwell = [p for p in route if forward and p['mono'] < forward['mono'] and error(p, plan[2]) <= 1.5 and
             ground_speed(p) <= .55 and 4400 <= p['relative_alt'] <= 5600]
    require(dwell and dwell[-1]['mono']-dwell[0]['mono'] >= 2.4 and
            any(p['seq'] == 4 and p['mission_state'] == 3 for p in progress),
            'independent waypoint dwell or terminal LAND cursor missing')
    before = quiet[-1]
    travelled = max(distance(before['lat']*1e-7, before['lon']*1e-7, p['lat']*1e-7, p['lon']*1e-7) for p in route)
    require(travelled >= 10, 'same-cursor resume did not physically continue mission')
    landed = rows('EXTENDED_SYS_STATE', times['land.finished'], finished)
    heart = rows('HEARTBEAT', times['land.finished'], finished)
    final = rows('GLOBAL_POSITION_INT', times['land.finished'], finished)
    require(len(landed) >= 2 and all(p['landed_state'] == 1 for p in landed) and
            len(heart) >= 2 and all(not p['base_mode'] & 128 for p in heart) and
            any(p['custom_mode'] == 4 and p['mono'] >= times['abort.started'] for p in heart) and
            final and all(error(p, plan[-1]) <= 3 and abs(p['relative_alt']) <= 700 for p in final),
            'independent disarm, ground handback or bound touchdown missing')
    return {'result': 'PASS', 'takeoff_height_m': max(p['relative_alt'] for p in lift)/1000,
            'paused_cursor': resume[0]['seq'], 'paused_samples': len(quiet), 'resume_travel_m': travelled,
            'observed_dwell_s': dwell[-1]['mono']-dwell[0]['mono'], 'final_error_m': error(final[-1], plan[-1]),
            'observed_landed_disarmed_guided': True}


def audit_typed_dataflash(arms, modes, commands, params, fixture):
    require(all(k in params and math.isclose(params[k], v, rel_tol=1e-5, abs_tol=1e-4) for k, v in fixture.items()),
            'typed native fixture parameter readback missing')
    require(any(p['ArmState'] == 1 for p in arms) and any(p['ArmState'] == 0 for p in arms),
            'typed mission arm and disarm records missing')
    order = [int(p['ModeNum']) for p in modes]
    require(3 in order and 4 in order[:order.index(3)] and 4 in order[order.index(3)+1:],
            'typed native Guided/AUTO/Guided mode order missing')
    observed = {(int(p['CNum']), int(p['CId'])) for p in commands if int(p['CTot']) == 5}
    require({(1, 22), (2, 16), (3, 16), (4, 21)} <= observed, 'typed native mission item records missing')
    return {'result': 'PASS', 'mode_order': order, 'mission_items': sorted(observed), 'closed_record_projection': True}
