"""Pure read-only mission fixture gates; expectations come from pinned declarations."""
import json
import math


def mission_read_evidence(events, selected_fixture):
    if (not isinstance(selected_fixture, dict) or selected_fixture.get('scenario') != 'armed_wiped_mission_read'
            or selected_fixture.get('qualification') != 'not_inferred'):
        raise RuntimeError('source-bound mission fixture selection missing')
    expected = selected_fixture['expectation'];count = expected['expected_count']
    declared = [e['value'] for e in events if e.get('type') == 'fixture' and e.get('key') == 'mission_shape']
    if declared != [expected['shape']]:
        raise RuntimeError('probe mission fixture enum differs from source declaration')
    def fields(value):
        return dict(part.split('=', 1) for part in value.split(','))
    requests = [fields(e['value']) for e in events if e.get('type') == 'request' and e['key'] == 'MAVMOOS_MISSION_REQUEST']
    if len(requests) != 1 or requests[0] != {'id': 'concurrent-mission-read', 'action': 'download', 'timeout': '15'}:
        raise RuntimeError('mission-read fixture requested mutation/execution or wrong read')
    parameter_requests = [fields(e['value']) for e in events if e.get('type') == 'request' and e['key'] == 'MAVMOOS_PARAM_REQUEST']
    if not parameter_requests or any(r.get('action') != 'read' or r.get('name') != 'SIM_WIND_SPD' for r in parameter_requests):
        raise RuntimeError('mission-read fixture exposed a non-read parameter request')
    snapshots = [json.loads(e['value']) for e in events if e.get('type') == 'mission_snapshot']
    if len(snapshots) != 1:
        raise RuntimeError('missing or repeated mission read snapshot')
    snapshot = snapshots[0]
    if (snapshot.get('schema') != 1 or snapshot.get('id') != 'concurrent-mission-read'
            or snapshot.get('status') != 'complete' or type(snapshot.get('count')) is not int
            or snapshot['count'] != count or not isinstance(snapshot.get('items'), list) or len(snapshot['items']) != count
            or snapshot.get('mission_type') != 0 or snapshot.get('verification') != 'observation'
            or snapshot.get('atomic') is not False):
        raise RuntimeError('armed wiped simulator mission read differs from the declared shape')
    homes = [fields(e['value']) for e in events if e.get('type') == 'home_observation' and e.get('key') == 'MAVMOOS_HOME_POSITION']
    homes = [h for h in homes if h.get('present') == 'true' and h.get('encoding_valid') == 'true'
             and h.get('identity') == 'EXPECTED' and h.get('freshness') == 'REFERENCE_KNOWN']
    if not homes:
        raise RuntimeError('mission HOME lacks observed bound FC home reference')
    home = homes[-1]; latitude, longitude, altitude = [float(home[k]) for k in ('lat_deg', 'lon_deg', 'alt_msl_m')]
    if not all(math.isfinite(v) for v in (latitude, longitude, altitude)) or abs(latitude) > 90 or abs(longitude) > 180:
        raise RuntimeError('bound home reference is not finite/bounded')
    if count:
        item = snapshot['items'][0]
        if (item.get('seq') != 0 or item.get('command') != 16 or item.get('frame') != 0
                or item.get('current') not in (0, 1) or item.get('autocontinue') != 1
                or item.get('params') != [0, 0, 0, 0] or item.get('x') != round(latitude * 1e7)
                or item.get('y') != round(longitude * 1e7) or not math.isfinite(item.get('z', math.nan))
                or abs(item['z'] - altitude) > .010001):
            raise RuntimeError('mission HOME wire fields differ from bound FC home')
    mission_results = [fields(e['value']) for e in events if e.get('type') == 'mission_result']
    if (not mission_results or any(r['id'] != 'concurrent-mission-read' or r['action'] != 'download'
                                   or r['mutation_may_have_applied'] != 'false'
                                   or r['status'] in ('FAILED', 'CANCELLED', 'REJECTED', 'TIMED_OUT', 'INCOMPLETE') for r in mission_results)
            or not any(r['status'] == 'COMPLETED' for r in mission_results)):
        raise RuntimeError('mission read failed or reported mutation uncertainty')
    completed = [r for r in mission_results if r['status'] == 'COMPLETED']
    deadlines = [float(r['expires']) for r in mission_results]
    if (len(completed) != 1 or any(not math.isfinite(d) or d <= 0 or d != deadlines[0] for d in deadlines)
            or not 0 < deadlines[0] - float(mission_results[0]['at']) <= 15.000001
            or not math.isfinite(float(completed[0]['at'])) or not 0 < float(completed[0]['at']) < deadlines[0]
            or completed[0].get('received') != str(count) or completed[0].get('expected') != str(count)):
        raise RuntimeError('mission terminal/snapshot counts or original deadline differ')
    tx = [fields(e['value']) for e in events if e.get('type') == 'mission_tx']
    if (not tx or {int(r['msgid']) for r in tx} != set(expected['required_host_message_ids'])
            or any(r['operation_id'] != 'concurrent-mission-read' or r['status'] != 'LOCAL_ACCEPTED'
                   or r['late'] != 'false' or r['clock_valid'] != 'true' for r in tx)):
        raise RuntimeError('mission read emitted a mutation/unexpected packet or lacked accepted read/ACK sends')
    parameter_results = [fields(e['value']) for e in events if e.get('type') == 'parameter_result']
    if not any(r['id'] == 'concurrent-parameter-read' and r['status'] == 'COMPLETED' for r in parameter_results):
        raise RuntimeError('ordinary parameter read did not complete beside mission read')
    assertions = {e['key']: fields(e['value']) for e in events if e.get('type') == 'assertion'
                  and e['key'] in ('read_services_complete', 'mission_read_continuity', 'transfer_outcome')}
    if set(assertions) != {'read_services_complete', 'mission_read_continuity', 'transfer_outcome'}:
        raise RuntimeError('read service/continuity assertions missing')
    service, continuity, transfer = [assertions[k] for k in ('read_services_complete', 'mission_read_continuity', 'transfer_outcome')]
    duration = float(service['service_window_s']); post = float(continuity['post_read_observation_s'])
    gap = float(transfer['max_observed_nav_counter_update_gap_s'])
    if (service['mission_status'] != 'COMPLETED' or service['parameter_status'] != 'COMPLETED'
            or service['mission_mutations'] != '0' or not math.isfinite(duration) or duration < 0 or duration >= 18
            or not math.isfinite(post) or post < 2 or continuity['no_mutation'] != 'true'
            or transfer['status'] != 'COMPLETED' or transfer['loss_seen'] != 'false'
            or int(transfer['nav_after']) <= int(transfer['nav_before'])
            or not math.isfinite(gap) or gap < 0 or gap > .75):
        raise RuntimeError('read-only transfer did not preserve bounded active navigation')
    return {'mission_status': 'COMPLETED', 'parameter_read_status': 'COMPLETED', 'snapshot_count': count,
            'mission_shape': expected['shape'], 'item_records_exercised': count,
            'original_expires': deadlines[0], 'completed_at': float(completed[0]['at']),
            'home_reference': {'latitude': latitude, 'longitude': longitude, 'altitude_msl_m': altitude},
            'mission_message_ids': sorted({int(r['msgid']) for r in tx}), 'mutation_requested': False,
            'service_window_s_cpp': duration, 'post_read_observation_s_cpp': post,
            'nav_before': int(transfer['nav_before']), 'nav_after': int(transfer['nav_after']),
            'note': 'C++ service window ends when both read outcomes and snapshot are received; the separate post-read continuation interval is not protocol transfer time.'}
