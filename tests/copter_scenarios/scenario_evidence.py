"""Offline semantic gates for the two owned stock-helm Copter scenarios.

Probe elapsed time, Python receive time and FC boot time are separate domains.
Window comparisons use their own phase brackets; no cross-clock age is invented.
"""
import hashlib
import json
import math
import re
import statistics

SCENARIOS = {
    'route_hold_return': {'cycles': 1, 'wind_speed_mps': 0.0, 'supervisor_s': 300},
    'wind_repeated_cycle': {'cycles': 2, 'wind_speed_mps': 2.0, 'supervisor_s': 520},
}
STEPS = ('ground.started', 'ground.finished', 'takeoff.started', 'takeoff.finished',
         'route1.started', 'route1.finished', 'hold_helm.started', 'hold_helm.finished',
         'cancel_helm.started', 'cancel_helm.finished', 'route2.started', 'route2.finished',
         'hold_guided.started', 'hold_guided.finished', 'cancel_guided.started', 'cancel_guided.finished',
         'return.started', 'return.finished', 'land.started', 'land.finished', 'finished')


def require(value, why):
    if not value:
        raise ValueError(why)


def number(value):
    require(type(value) in (int, float) and math.isfinite(value), 'finite numeric value required')
    return value


def hold_ack_observation(original,ack):
    """One probe clock; the witness follows the matching file read/check."""
    waited,deadline,observed=number(ack['wait_started_elapsed']),number(ack['deadline_elapsed']),number(ack['ack_observed_elapsed'])
    require(0<=waited<=number(original['elapsed'])<=observed<=number(ack['elapsed']) and
            abs(deadline-waited-3)<=1e-6 and observed<deadline,'hold acknowledgement missed original deadline')
    return observed


def parse(data):
    def pairs(rows):
        result = {}
        for key, value in rows:
            require(key not in result, 'duplicate JSON key')
            result[key] = value
        return result
    def bad(value):
        raise ValueError('nonfinite JSON: ' + value)
    return json.loads(data, object_pairs_hook=pairs, parse_constant=bad)


def fields(text):
    result = {}
    for item in text.split(','):
        key, separator, value = item.partition('=')
        require(separator and key and key not in result, 'malformed/duplicate KV field')
        result[key] = value
    return result


def yes(value):
    return value in ('true', '1')


def markers(scenario):
    require(scenario in SCENARIOS, 'unknown scenario')
    return tuple(f'cycle{cycle}.{step}' for cycle in range(1, SCENARIOS[scenario]['cycles'] + 1) for step in STEPS)


def phase(events, key):
    rows = [row for row in events if row['type'] == 'phase' and row['key'] == key]
    require(len(rows) == 1, 'missing/duplicate phase: ' + key)
    return rows[0]


FIRMWARE_FILES = {'manifest': 'firmware-source.json', 'attestation': 'firmware-attestation.json'}


def firmware_inputs(manifest_bytes, attestation_bytes, identity):
    """Bind an expected advertisement to the exact preflight verified inputs.

    This is offline consistency readback, not another native build verification.
    The runner calls verify_native before I/O and retains these same bytes; the
    additive reader obtains them from the whole-result/whole-archive SHA chain.
    """
    manifest, attestation = parse(manifest_bytes), parse(attestation_bytes)
    manifest_sha = hashlib.sha256(manifest_bytes).hexdigest()
    attestation_sha = hashlib.sha256(attestation_bytes).hexdigest()
    require(identity['manifest_sha256'] == manifest_sha and identity['qualification'] == 'not_inferred',
            'copied source manifest differs from verified native identity')
    require(manifest['schema'] == 1 and manifest['kind'] == 'mavmoos.firmware_source' and
            manifest['vehicle_profile'] == 'arducopter' and manifest['qualification'] == 'not_inferred' and
            manifest['profile_id'] == identity['profile_id'] and manifest['source']['revision'] == identity['source_revision'] and
            re.fullmatch('[0-9a-f]{40}', manifest['source']['revision']) and manifest['fixture']['model'] == identity['frame'] == 'quad' and
            manifest['target'] == {'system_id': 1, 'component_id': 1, 'mav_type': 2, 'autopilot': 3},
            'copied source manifest/profile differs')
    require(attestation['schema'] == 1 and attestation['kind'] == 'mavmoos.native_build_attestation' and
            attestation['qualification'] == 'not_inferred' and attestation['profile_id'] == identity['profile_id'] and
            attestation['manifest_sha256'] == manifest_sha and attestation['source_identity']['revision'] == identity['source_revision'] and
            attestation['artifact']['path'] == manifest['artifact_path'] and
            attestation['artifact']['sha256'] == identity['native_sha256'] and attestation['platform'] == identity['platform'],
            'copied build attestation differs from verified native identity')
    version = manifest['source']['version']
    require(set(version) == {'major', 'minor', 'patch', 'release_type'} and
            all(type(v) is int and 0 <= v <= 255 for v in version.values()), 'manifest version is unknown/malformed')
    expected = {key: version[key] for key in ('major', 'minor', 'patch')}
    expected['release_type_raw'] = version['release_type']
    expected['flight_sw_version_raw'] = (version['major'] << 24) | (version['minor'] << 16) | (version['patch'] << 8) | version['release_type']
    require(expected['flight_sw_version_raw'] != 0, 'unknown source version cannot qualify discovery')
    return {'manifest_sha256': manifest_sha, 'attestation_sha256': attestation_sha,
            'source_revision': manifest['source']['revision'], 'expected_version': expected}


def audit_firmware(events, expected, early_end):
    """Require an early original version sample; later stale identity is useful.

    Host observation time and probe elapsed time are separate clock domains.
    Only within-record/within-observation host-time checks are made here.
    Republishing one sample cannot advance its original effective/read time.
    """
    rows = [row for row in events if row['type'] == 'firmware' or row['key'] == 'MAVMOOS_FIRMWARE_STATUS']
    require(rows, 'firmware discovery observation missing')
    previous_epoch, previous_id = 0, 0
    observations, emitted_at = {}, {}
    early, stale = [], 0
    for row in rows:
        require(row['type'] == 'firmware' and row['key'] == 'MAVMOOS_FIRMWARE_STATUS' and row['source'] == 'iMAVLink',
                'firmware observation publisher/interface differs')
        status = parse(row['value'])
        require(status['schema'] == 1 and status['read_only'] is True and status['authoritative_for_control'] is False and
                status['qualification'] == 'not_evaluated' and status['source_age'] == 'unknown' and
                status['system_id'] == status['component_id'] == 1 and status['retained_until_session_reset'] is True,
                'firmware status changed observation/control semantics or target')
        epoch, ident = status['epoch'], status['observation_id']
        require(type(epoch) is int and epoch >= 1 and epoch >= previous_epoch and type(ident) is int and ident >= previous_id and
                type(status['identity_revision']) is int and status['identity_revision'] >= 0, 'firmware epoch/observation counters regressed')
        previous_epoch, previous_id = epoch, ident
        require(status['state'] in ('OBSERVED', 'STALE', 'WAITING_HEARTBEAT', 'WAITING_VERSION', 'CLOCK_FAULT'), 'unknown firmware state')
        if status['present'] is False:
            require(status['fresh'] is False and status['reported'] is None and status['effective_at'] is None and
                    status['read_at'] is None and status['age_s'] is None and status['state'] in ('WAITING_HEARTBEAT', 'WAITING_VERSION', 'CLOCK_FAULT'),
                    'cleared firmware epoch retained or promoted an old observation')
            continue
        require(status['present'] is True and ident > 0 and status['state'] in ('OBSERVED', 'STALE'), 'present firmware state missing')
        effective, read, age = (number(status[key]) for key in ('effective_at', 'read_at', 'age_s'))
        require(0 <= effective <= read <= effective + age and age >= 0, 'invalid original firmware observation times/age')
        require(status['age_basis'] in ('READ_ACQUISITION', 'QUEUE_EMPTY_FLOOR') and
                (status['age_basis'] != 'READ_ACQUISITION' or read == effective) and
                status['age_is_upper_bound'] is (status['age_basis'] == 'QUEUE_EMPTY_FLOOR'), 'firmware age basis differs')
        require(status['fresh'] is (age <= 30) and status['state'] == ('OBSERVED' if age <= 30 else 'STALE'),
                'firmware republication renewed freshness or mislabelled stale identity')
        reported = status['reported']
        require(reported['version_known'] is True and
                all(type(reported[key]) is int and reported[key] == value for key, value in expected['expected_version'].items()),
                'reported firmware version is unknown or differs from copied source manifest')
        prefix = expected['source_revision'][:8]
        require(reported['flight_custom_version_ascii'] == prefix and reported['flight_custom_version_raw_hex'] == prefix.encode('ascii').hex() and
                reported['custom_version_semantics'] == 'ardupilot_ascii_prefix_or_opaque_bytes_not_full_git_revision' and
                reported['capability_assessment'] == 'reported_bits_not_behavior_qualification', 'reported short source prefix/semantics differs')
        key = (epoch, ident)
        original = {'effective_at': effective, 'read_at': read, 'age_basis': status['age_basis'], 'reported': reported,
                    'identity_revision': status['identity_revision']}
        if key in observations:
            require(observations[key] == original, 'firmware republication changed original observation/identity')
            require(effective + age >= emitted_at[key], 'firmware observation age moved backwards')
        else:
            earlier = [value for (old_epoch, old_id), value in observations.items() if old_epoch == epoch and old_id < ident]
            require(not earlier or effective >= earlier[-1]['effective_at'], 'new firmware sample moved original time backwards')
            require(not any(old_id == ident for old_epoch, old_id in observations if old_epoch != epoch), 'reset reused an old firmware sample')
            observations[key] = original
        emitted_at[key] = effective + age
        stale += status['state'] == 'STALE'
        if status['fresh'] is True and row['elapsed'] <= early_end:
            early.append({'probe_elapsed': row['elapsed'], 'epoch': epoch, 'observation_id': ident,
                          'effective_at': effective, 'read_at': read, 'age_s': age, 'age_basis': status['age_basis']})
    require(early, 'no early bound fresh known firmware observation before takeoff')
    return {'result': 'PASS', **expected, 'early_observation': early[0], 'publication_rows': len(rows),
            'unique_observations': len(observations), 'stale_publications': stale,
            'scope': 'observed FC advertisement plus short source prefix; no full revision, capability, control authority or field qualification inferred'}


def nav_points(events, begin, end):
    current = {}
    points = []
    for row in events:
        if row['type'] != 'navigation':
            continue
        current[row['key']] = number(float(row['value']))
        if begin <= row['elapsed'] <= end and all(k in current for k in ('NAV_X', 'NAV_Y', 'NAV_ALTITUDE', 'NAV_SAMPLE_RECEIVED_AT')):
            points.append((row['elapsed'], dict(current)))
    return points


def audit_probe(events, scenario, firmware):
    require(scenario in SCENARIOS and isinstance(events, list) and events, 'missing scenario events')
    require(not [row for row in events if row['type'] == 'fail'], 'probe reported failure')
    require(len([r for r in events if r['type'] == 'pass' and r['key'] == scenario]) == 1, 'missing/repeated scenario pass')
    require(events[-1]['type'] == 'pass' and events[-1]['key'] == scenario, 'scenario pass is not final')
    previous = -1
    for row in events:
        elapsed = number(row['elapsed'])
        require(elapsed >= previous >= -1, 'probe elapsed time moved backwards')
        previous = elapsed
        if row['type']=='boundary_ack':require(row.get('source')=='probe','foreign hold acknowledgement publisher')
        elif row['type'] not in ('phase', 'request', 'pass', 'fail'):
            require(row['source'] == ('pHelmIvP' if row['type'] == 'helm_scalar' else 'iMAVLink'), 'unbound MOOS publisher')
    actual = [r['key'] for r in events if r['type'] == 'phase']
    require(actual == list(markers(scenario)), 'phase sequence differs from complete scenario')
    require({r['key'] for r in events if r['type'] == 'helm_scalar'} == {'DESIRED_HEADING', 'DESIRED_SPEED', 'DESIRED_ALTITUDE'},
            'ordinary stock-helm scalar evidence incomplete')
    operations = {}
    for row in events:
        if row['type'] != 'operation_result':
            continue
        result = fields(row['value'])
        require(result['schema'] == '1', 'operation schema changed')
        ident = result['id']
        expiry, at = number(float(result['expires'])), number(float(result['at']))
        require(expiry > 0 and at <= expiry, 'operation expired or has invalid original deadline')
        entry = operations.setdefault(ident, {'expires': expiry, 'statuses': [], 'terminal': None})
        require(expiry == entry['expires'], 'operation original deadline renewed')
        require(result['status'] not in ('FAILED', 'TIMED_OUT', 'REJECTED', 'DUPLICATE'), 'unexpected operation failure')
        entry['statuses'].append(result['status'])
        entry['terminal'] = result['status']
    requests = [fields(r['value']) for r in events if r['type'] == 'request' and r['key'] == 'MAVMOOS_CONTROL_REQUEST']
    require(requests and len({r['id'] for r in requests}) == len(requests), 'missing/duplicate operation requests')
    require(all(r['schema'] == '1' for r in requests), 'request schema changed')
    require(not [r for r in events if r['type'] == 'request' and r['key'] not in ('MAVMOOS_CONTROL_REQUEST', 'MOOS_MANUAL_OVERRIDE')],
            'scenario injected an undeclared effect/request')
    proof = {'result': 'PASS', 'scenario': scenario, 'cycles': SCENARIOS[scenario]['cycles'], 'windows': {}, 'operations': operations,'boundary_tokens':{},
             'firmware_discovery': audit_firmware(events, firmware, phase(events, 'cycle1.takeoff.started')['elapsed'])}
    for cycle in range(1, SCENARIOS[scenario]['cycles'] + 1):
        prefix, ident = f'cycle{cycle}', f'c{cycle}'
        expected = {ident + '-' + name: 'COMPLETED' for name in ('guided', 'arm', 'takeoff', 'route1', 'route2', 'return', 'land')}
        for strategy in ('helm', 'guided'):
            expected[prefix + '-hold-' + strategy] = 'CANCELLED'
            expected[prefix + '-cancel-' + strategy] = 'COMPLETED'
        if ident + '-disarm' in operations:
            expected[ident + '-disarm'] = 'COMPLETED'
        for opid, terminal in expected.items():
            require(opid in operations and operations[opid]['terminal'] == terminal, 'operation terminal missing: ' + opid)
            if '-hold-' in opid:
                require('CONFIRMED' in operations[opid]['statuses'], 'held target was not confirmed')
        require({r['id'] for r in requests if r['id'].startswith((ident + '-', prefix + '-'))} == set(expected),
                'unexpected/missing cycle request ID')
        for request in [r for r in requests if r['id'] in expected]:
            suffix = request['id'].split('-', 1)[1]
            action = 'goto' if suffix in ('route1', 'route2') else suffix.split('-', 1)[0]
            strategy = suffix.split('-', 1)[1] if suffix.startswith('hold-') else 'helm' if suffix in ('route1', 'route2', 'return') else 'native'
            require(request['action'] == action and request.get('strategy', 'native') == strategy, 'wrong request action/strategy')
            if action == 'cancel':
                require(request['target'] == prefix + '-hold-' + suffix.split('-', 1)[1], 'cancel target changed')
        for strategy in ('helm', 'guided'):
            label = prefix + '.hold_' + strategy
            first, last = phase(events, label + '.started'), phase(events, label + '.finished')
            end = last['elapsed']
            boundary_acks={}
            for suffix in ('.started','.finished'):
                key=label+suffix
                acks=[row for row in events if row['type']=='boundary_ack' and row['key']==key]
                require(len(acks)==1 and acks[0].get('source')=='probe','missing/duplicate/foreign hold acknowledgement')
                ack=acks[0];token=ack['value']
                require(type(token) is str and len(token)<=128 and re.fullmatch(r'hold:'+re.escape(key)+r':[1-9][0-9]{0,3}:[0-9]+(?:\.[0-9]+)?\n',token) is not None,'hold acknowledgement token invalid')
                original=first if suffix=='.started' else last
                observed=hold_ack_observation(original,ack)
                proof['boundary_tokens'][key]=token;boundary_acks[suffix]=observed
            begin=boundary_acks['.started']
            require(3.8 <= end - begin <= 5.5, 'hold window duration invalid')
            require(end-begin>=4,'four-second hold began before start acknowledgement')
            cancellation=[row for row in events if row['type']=='request' and row['key']=='MAVMOOS_CONTROL_REQUEST' and fields(row['value']).get('id')==prefix+'-cancel-'+strategy]
            require(len(cancellation)==1 and cancellation[0]['elapsed']>boundary_acks['.finished'],'hold cancelled before finish acknowledgement')
            anchor = fields(first['value'])
            ax, ay = number(float(anchor['anchor_x'])), number(float(anchor['anchor_y']))
            require(anchor['strategy'] == strategy and float(anchor['height']) == 5, 'wrong held-target strategy/height')
            rows = [r for r in events if r['type'] == 'runtime' and begin <= r['elapsed'] <= end]
            require(len(rows) >= 15 and rows[0]['elapsed'] <= begin + .5 and rows[-1]['elapsed'] >= end - .5, 'late/current Runtime evidence missing')
            require(max(b['elapsed'] - a['elapsed'] for a, b in zip(rows, rows[1:])) <= .5, 'Runtime window coverage gap')
            states = [fields(r['value']) for r in rows]
            for state in states:
                require(all(yes(state[k]) for k in ('heartbeat_valid', 'position_valid', 'home_valid', 'landed_valid', 'estimator_valid', 'armed', 'in_air', 'permission')) and state['fc_mode'] == '4',
                        'held target lost state/authority')
            counters = [int(s['nav_tx_accepted']) for s in states]
            require(counters == sorted(counters) and counters[-1] - counters[0] >= 15, 'held target lacks continuing NAV')
            before, after = int(anchor['nav_tx_accepted']), int(fields(last['value'])['nav_tx_accepted'])
            require(before <= counters[0] <= counters[-1] <= after, 'phase NAV counters contradict Runtime')
            last_progress, prior = begin, before
            gaps = []
            for row, count in zip(rows, counters):
                if count > prior:
                    gaps.append(row['elapsed'] - last_progress)
                    last_progress, prior = row['elapsed'], count
            require(max(gaps + [end - last_progress]) <= .75, 'late NAV progression gap')
            points = nav_points(events, end - 2.5, end)
            require(len({p['NAV_SAMPLE_RECEIVED_AT'] for _, p in points}) >= 8, 'late original position samples did not advance')
            require(all(abs(p['NAV_ALTITUDE'] - 5) <= .65 and math.hypot(p['NAV_X'] - ax, p['NAV_Y'] - ay) <= 2 for _, p in points), 'held target did not settle')
            proof['windows'][label] = {'probe_elapsed': [begin, end], 'anchor_xy': [ax, ay], 'nav_counter_delta': counters[-1] - counters[0],
                                        'late_position_samples': len({p['NAV_SAMPLE_RECEIVED_AT'] for _, p in points})}
            cancel_begin = phase(events, prefix + '.cancel_' + strategy + '.started')['elapsed']
            cancel_end = phase(events, prefix + '.cancel_' + strategy + '.finished')['elapsed']
            require(1 <= cancel_end - cancel_begin <= 2.5, 'post-cancel observation window missing')
            quiet = [fields(r['value']) for r in events if r['type'] == 'runtime' and cancel_end - .6 <= r['elapsed'] <= cancel_end]
            require(len(quiet) >= 3 and all(not yes(s['permission']) for s in quiet) and len({s['nav_tx_accepted'] for s in quiet}) == 1,
                    'cancel resumed or kept locally streaming NAV')
        targets = []
        for route in ('route1', 'route2'):
            request = next(r for r in requests if r['id'] == ident + '-' + route)
            targets.append((route, fixture_xy(number(float(request['lat'])), number(float(request['lon'])))))
        home = fields(phase(events, prefix + '.return.started')['value'])
        targets.append(('return', fixture_xy(number(float(home['home_lat'])), number(float(home['home_lon'])))))
        for route, target in targets:
            end = phase(events, prefix + '.' + route + '.finished')['elapsed']
            points = nav_points(events, end - .6, end)
            require(points and any(math.hypot(p['NAV_X'] - target[0], p['NAV_Y'] - target[1]) <= 2.3 and abs(p['NAV_ALTITUDE'] - 5) <= .65 for _, p in points),
                    'route/home arrival not observed')
    require({row['key'] for row in events if row['type']=='boundary_ack'}==set(proof['boundary_tokens']),'unrequested hold acknowledgement')
    required_ops = {r['id'] for r in requests}
    require(set(operations) == required_ops, 'unrequested operation results or missing results')
    final_elapsed = phase(events, f"cycle{SCENARIOS[scenario]['cycles']}.finished")['elapsed']
    final = [fields(r['value']) for r in events if r['type'] == 'runtime' and final_elapsed - 1 <= r['elapsed'] <= final_elapsed]
    require(len(final) >= 5 and all(not yes(s['armed']) and not yes(s['in_air']) and not yes(s['permission']) and
            yes(s['heartbeat_valid']) and yes(s['landed_valid']) for s in final), 'final disarmed/on-ground state missing')
    return proof


def fixture_xy(latitude, longitude):
    # Explicit short-distance comparison to this fixture's supplied origin.
    return ((longitude + 71.087589) * 111195 * math.cos(math.radians(42.358456)),
            (latitude - 42.358456) * 111195)


def local_xy(packet):
    return fixture_xy(packet['lat'] / 1e7, packet['lon'] / 1e7)


def audit_fc(packets, phase_times, finished, proof):
    require(set(phase_times) == set(markers(proof['scenario'])), 'independent phase markers missing')
    require(packets and all(p['system'] == p['component'] == 1 and number(p['mono']) >= 0 for p in packets), 'foreign/passive FC source missing')
    require(all(a['mono'] <= b['mono'] for a, b in zip(packets, packets[1:])), 'passive receive time regressed')
    report = {'result': 'PASS', 'windows': {}, 'scope': 'separate passive native endpoint; own receive-time brackets, no cross-clock/source-age mapping'}
    for label, window in proof['windows'].items():
        begin, end = phase_times[label + '.started'], phase_times[label + '.finished']
        require(3.5 <= end - begin <= 5.8, 'independent hold bracket duration invalid')
        rows = [p for p in packets if end - 2.5 <= p['mono'] <= end]
        positions = [p for p in rows if p['mavpackettype'] == 'GLOBAL_POSITION_INT']
        heartbeats = [p for p in rows if p['mavpackettype'] == 'HEARTBEAT']
        landed = [p for p in rows if p['mavpackettype'] == 'EXTENDED_SYS_STATE']
        targets = [p for p in rows if p['mavpackettype'] == 'POSITION_TARGET_LOCAL_NED']
        require(len(positions) >= 8 and len({p['time_boot_ms'] for p in positions}) >= 8, 'late independent positions missing/frozen')
        require(len(heartbeats) >= 2 and all(p['base_mode'] & 128 and p['custom_mode'] == 4 for p in heartbeats), 'independent armed Guided state missing')
        require(len(landed) >= 2 and all(p['landed_state'] == 2 for p in landed), 'independent in-air state missing')
        require(len(targets) >= 5, 'late independent Guided target reports missing')
        ax, ay = window['anchor_xy']
        require(all(math.hypot(local_xy(p)[0] - ax, local_xy(p)[1] - ay) <= 2.5 and abs(p['relative_alt'] / 1000 - 5) <= .8 for p in positions), 'independent held position/height disagrees')
        speeds = [math.hypot(p['vx'], p['vy']) / 100 for p in positions]
        require(statistics.median(speeds) <= .8, 'independent hold speed not settled')
        report['windows'][label] = {'late_unique_positions': len({p['time_boot_ms'] for p in positions}), 'median_horizontal_speed_mps': statistics.median(speeds), 'target_reports': len(targets)}
    late = [p for p in packets if finished - 2.5 <= p['mono'] <= finished]
    hb = [p for p in late if p['mavpackettype'] == 'HEARTBEAT']
    ls = [p for p in late if p['mavpackettype'] == 'EXTENDED_SYS_STATE']
    require(len(hb) >= 2 and all(not p['base_mode'] & 128 for p in hb) and len(ls) >= 2 and all(p['landed_state'] == 1 for p in ls),
            'late independent disarmed/on-ground state missing')
    return report


def audit_dataflash(arms, modes, parameters, scenario):
    fixture = SCENARIOS[scenario]
    require(parameters.get('SIM_WIND_SPD') == fixture['wind_speed_mps'] and parameters.get('SIM_WIND_DIR') == 270 and parameters.get('SIM_WIND_TURB') == 0,
            'closed DataFlash wind fixture mismatch')
    require(parameters.get('LOG_DISARMED') == 1 and parameters.get('SERIAL0_PROTOCOL') == 2 and parameters.get('SERIAL1_PROTOCOL') == 2,
            'closed fixture serial/log confirmation missing')
    require(arms and all(int(r.get('Forced', 0)) == 0 for r in arms), 'forced/missing DataFlash arming evidence')
    transitions = []
    for row in arms:
        state = int(row['ArmState'])
        require(state in (0, 1), 'unknown DataFlash arm state')
        if not transitions or transitions[-1] != state:
            transitions.append(state)
    if transitions and transitions[0] == 0:
        transitions.pop(0)
    require(transitions == [state for _ in range(fixture['cycles']) for state in (1, 0)], 'wrong number/order of complete ordinary arm/disarm cycles')
    require(modes and {4, 9} <= {int(r['ModeNum']) for r in modes} and all(int(r['ModeNum']) in (0, 4, 9) for r in modes), 'unexpected/missing DataFlash Guided/Land modes')
    return {'result': 'PASS', 'complete_arm_disarm_cycles': fixture['cycles'], 'wind_speed_mps': fixture['wind_speed_mps'],
            'scope': 'closed native DataFlash configuration/mode/arming corroboration, not physical wind/sensor accuracy'}
