"""Pure acceptance for the actual app/demo logs; never opens sockets/processes."""
import json
import math
import statistics

INVALID_SCENARIOS = {'invalid_config','wrong_abi','unknown_key','duplicate_key','missing_owner','reserved_policy','noncanonical'}


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def fields(text):
    return dict(part.split('=',1) for part in text.split(',') if '=' in part)


def observations(text):
    result = []
    for line in text.splitlines():
        if not line.startswith('{'):
            continue  # MOOS library diagnostics remain in the unmodified log.
        value = json.loads(line)
        require(value.get('source')=='iMAVLink' and math.isfinite(value['mono']), 'invalid MOOS source/read stamp')
        require(not result or value['mono']>=result[-1]['mono'], 'MOOS read clock regressed')
        result.append(value)
    require(0<len(result)<=10000,'missing/excessive actual MOOS observations')
    return result


def audit(events, supervisor, *, scenario='normal', expected_abi=9):
    require(supervisor.get('result')=='PASS' and supervisor.get('cleanup_proven') is True and
            supervisor.get('owned_processes_remaining')==[] and supervisor.get('cleanup_errors')==[] and
            supervisor.get('input_drift')==[] and supervisor.get('inputs_before')==supervisor.get('inputs_after'),
            'supervisor/cleanup/input proof incomplete')
    require(supervisor.get('ports_clear_after')=={'moos':True,'app':True,'peer':True},'selected ports not all clear')
    require(set(supervisor.get('owned_app_roster',{}))=={'MOOSDB','iMAVLink'},'actual live app roster missing')
    peer = supervisor.get('peer_capture',{})
    require(peer.get('thread_retired') is True and peer.get('socket_closed') is True and peer.get('error') is None,
            'independent peer capture not retired cleanly')
    require(peer.get('peer')==['127.0.0.1',supervisor.get('ports',{}).get('peer')], 'primary capture bound endpoint differs')
    require(peer.get('sent_heartbeats',0)>0 and peer.get('received_datagrams')==0 and peer.get('received_bytes')==0,
            'primary independent peer saw egress or did not send fixture data')
    status = [fields(e['value']) for e in events if e['key']=='MAVMOOS_STATUS']
    require(status, 'actual app status missing')
    invalid = scenario in INVALID_SCENARIOS
    for value in status:
        require(value.get('state')==('CONFIG_ERROR' if invalid else 'RX_ONLY') and
                value.get('tx_enabled')=='false' and value.get('mavlink_tx_attempts')=='0' and
                value.get('mavlink_tx_accepted')=='0' and value.get('helm')=='disabled',
                'app mode/output gate differs')
    samples = [json.loads(e['value']) for e in events if e['key']=='EXAMPLE_HEARTBEAT_OBSERVATION']
    catalogs = [json.loads(e['value']) for e in events if e['key']=='MAVMOOS_CAPABILITIES']
    if invalid:
        require(not samples and not catalogs, 'invalid startup published a live module/catalog')
        require(all(v.get('loaded_modules')=='0' and v.get('rx_datagrams')=='0' for v in status),
                'invalid startup retains operational reads/modules')
        expected={'invalid_config':'sample_timeout','wrong_abi':'ABI','unknown_key':'typo','duplicate_key':'duplicate',
                  'missing_owner':'unloaded_observer','reserved_policy':'simulation_only','noncanonical':'canonical'}[scenario]
        diagnostics=[e['value'] for e in events if e['key']=='APPCAST']
        require(any(expected in value for value in diagnostics),'useful actual startup diagnostic missing')
        return {'result':'PASS','scenario':scenario,'observed_app_states':['CONFIG_ERROR'],
                'module_publications':0,'diagnostic_witness':expected,'independent_primary_egress_datagrams':0}
    require(catalogs and all(c==catalogs[0] for c in catalogs),'catalog absent/changed')
    catalog = catalogs[0]
    require(catalog.get('schema')==1 and catalog.get('execution_mode')=='RX_ONLY','catalog mode/schema differs')
    composition = catalog.get('composition',{})
    modules = composition.get('modules',[])
    require(composition.get('plugin_abi')==expected_abi and len(modules)==1,'loaded ABI/module inventory differs')
    module = modules[0]
    require(module.get('name')=='example_heartbeat_observer' and module.get('control_provider') is False and
            module.get('request_owners')==[] and module.get('capabilities')==[
                {'name':'example.heartbeat.observation','request_variable':'','request_value':'','access':'observation'}],
            'external module capability/control ownership differs')
    timeout,interval = (2.,.25) if scenario=='defaults' else (.5,.1)
    require(len(samples)>=8,'too few actual observer publications')
    sample_events = [e for e in events if e['key']=='EXAMPLE_HEARTBEAT_OBSERVATION']
    publication_gaps = [b['moos_time']-a['moos_time'] for a,b in zip(sample_events,sample_events[1:])
                        if b['moos_time']>a['moos_time']]
    require(publication_gaps and interval*.7<=statistics.median(publication_gaps)<=interval*1.8,
            'actual ordinary publication cadence differs from configured timing')
    by_identity, fresh_first, stale, reset, recovered = {},[],[],[],[]
    first_epoch = None
    for value in samples:
        require(value.get('schema')==1 and value.get('read_only') is True and
                value.get('authoritative_for_control') is False and value.get('example') is True and
                value.get('clock_fault') is False and value.get('source_age')=='unknown' and
                value.get('source_quality')=='unassessed','observer diagnostic semantics differ')
        require(value.get('sample_timeout_s')==timeout and value.get('publish_interval_s')==interval,
                'effective checked module configuration differs')
        if not value['present']:
            require(value.get('fresh') is False and all(value.get(k) is None for k in
                    ('effective_at','read_at','age_s','system_id','component_id','custom_mode_raw')),
                    'absent/reset sample retains apparent freshness/source')
            if first_epoch is not None and value['session_epoch']>first_epoch:
                reset.append(value)
            continue
        require(value.get('system_id')==1 and value.get('component_id')==1 and value.get('custom_mode_raw') in (42,84),
                'foreign source/peer changed the sample')
        effective,read,age = (value[k] for k in ('effective_at','read_at','age_s'))
        require(all(type(v) in (int,float) and math.isfinite(v) for v in (effective,read,age)) and
                0<effective<=read and age>=0 and value.get('age_basis')=='QUEUE_EMPTY_FLOOR',
                'observation original metadata invalid')
        require(value['fresh']==(age<=timeout),'freshness does not use original sample age')
        key = (value['session_epoch'],value['observation_id'])
        original = (effective,read,value['custom_mode_raw'],value['system_id'],value['component_id'])
        if key in by_identity:
            old,old_age = by_identity[key]
            require(original==old and age>=old_age,'cached publication renewed/changed original evidence')
        by_identity[key] = (original,age)
        if value['custom_mode_raw']==42 and value['fresh']:
            first_epoch = value['session_epoch'] if first_epoch is None else first_epoch
            fresh_first.append(value)
        if value['custom_mode_raw']==42 and not value['fresh']:
            stale.append(value)
        if value['custom_mode_raw']==84 and value['fresh']:
            require(first_epoch is not None and value['session_epoch']>first_epoch,'recovery did not follow a reset epoch')
            recovered.append(value)
    require(fresh_first and reset and recovered,'fresh→reset→post-boundary recovery sequence missing')
    if scenario!='defaults':
        require(stale,'no stale-preserved sample before the host reset')
        original = stale[0]
        require(any(v['session_epoch']==original['session_epoch'] and v['observation_id']==original['observation_id'] and
                    v['effective_at']==original['effective_at'] and v['read_at']==original['read_at'] for v in fresh_first),
                'stale sample lacks its originally observed fresh identity')
    require(recovered[0]['effective_at']>fresh_first[-1]['effective_at'] and
            recovered[0]['observation_id']>fresh_first[-1]['observation_id'], 'recovery reused pre-reset observation')
    require(any(int(v.get('wrong_target','0'))>=1 and int(v.get('wrong_peer','0'))>=1 for v in status),
            'actual host filter rejection counters missing')
    runtime = [fields(e['value']) for e in events if e['key']=='MAVMOOS_RUNTIME_STATUS']
    require(runtime,'actual Runtime status missing')
    for value in runtime:
        require(value.get('mode')=='RX_ONLY' and value.get('permission')=='false' and
                value.get('control_profile')=='none' and value.get('tx_attempts')=='0' and
                value.get('tx_accepted')=='0' and value.get('nav_tx_accepted')=='0' and
                value.get('mission_execution_held')=='false','Runtime effects/authority found')
    return {'result':'PASS','scenario':scenario,'sample_timeout_s':timeout,'publish_interval_s':interval,
            'fresh_publications':len(fresh_first),'stale_preserved_publications':len(stale),
            'reset_publications':len(reset),'recovered_publications':len(recovered),
            'first_epoch':first_epoch,'recovered_epoch':recovered[0]['session_epoch'],
            'median_publication_gap_s':statistics.median(publication_gaps),
            'independent_primary_egress_datagrams':0,'catalog':catalog,
            'scope':'actual local fake-peer/MOOS app; no simulator, physical vehicle, remote sample-age or authority claim'}
