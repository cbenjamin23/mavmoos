"""Offline gates for owned stock-helm range observation; never operate a vehicle.

Probe elapsed windows and Python observer windows stay separate. Same-FC boot
stamps compare source values in this short no-wrap run, not remote source ages.
"""
import json
import math
import statistics

LEVELS={'height.low':5.,'height.high':8.,'height.return':5.}
MARKERS=tuple(x+'.'+end for x in ('ground',*LEVELS) for end in ('started','finished'))


def require(value,why):
    if not value:raise ValueError(why)


def number(value):
    require(type(value) in (int,float) and math.isfinite(value),'finite numeric value required')
    return value


def parse(text):
    def bad(value):raise ValueError('nonfinite JSON '+value)
    def pairs(rows):
        result={}
        for key,value in rows:
            require(key not in result,'duplicate JSON key');result[key]=value
        return result
    return json.loads(text,parse_constant=bad,parse_float=lambda x:number(float(x)),object_pairs_hook=pairs)


def fields(text):
    result={}
    for item in text.split(','):
        key,separator,value=item.partition('=');require(separator and key not in result,'bad KV record');result[key]=value
    return result


def yes(value):return value in ('true','1')


def phase(events,key):
    selected=[row for row in events if row['type']=='phase' and row['key']==key]
    require(len(selected)==1,'missing/repeated phase '+key)
    return number(selected[0]['elapsed'])


def sample_rows(events):
    seen={};samples=[];status_count=0
    for event in events:
        require(number(event['elapsed'])>=0,'negative probe elapsed time')
        if event['type'] not in ('request','phase','pass','fail'):
            require(event.get('source')=='iMAVLink','unbound MOOS source')
        if event['key']=='MAVMOOS_DISTANCE_STATUS':
            state=parse(event['value']);require(state['read_only'] is True and state['authoritative_for_control'] is False,'distance acquired control policy')
            if state['state']=='BOUND':require(state['system_id']==state['component_id']==1,'distance source binding changed')
            status_count+=1
        if event['key']!='MAVMOOS_DISTANCE_SENSORS':continue
        value=parse(event['value']);rows=value['sensors']
        require(value['schema']==1 and value['atomic'] is False and value['cache_capacity']==8,'distance aggregate contract changed')
        require(type(value['present']) is bool and value['present']==bool(rows) and len(rows)<=8,'distance aggregate presence/capacity invalid')
        ids=[row['data']['id'] for row in rows];require(ids==sorted(set(ids)),'unsorted/duplicated sensor IDs')
        for row in rows:
            require(row['schema']==1 and row['family']=='DISTANCE_SENSOR' and row['present'] is True and row['encoding_valid'] is True,'invalid distance sample')
            effective,read,age=[number(row[k]) for k in ('effective_at','read_at','age_s')]
            require(0<=effective<=read and age>=0 and age+1e-6>=read-effective,'contradictory acquisition metadata')
            require(row['source_age']=='unknown' and row['source_quality']=='unassessed','invented source quality/age')
            if row['age_basis']=='QUEUE_EMPTY_FLOOR':require(row['age_is_upper_bound'] is True,'lost queue upper bound')
            else:require(row['age_basis']=='READ_ACQUISITION' and effective==read and row['age_is_upper_bound'] is False,'bad read-acquisition metadata')
            require(type(row['fresh']) is bool and (not row['fresh'] or age<=1),'false freshness')
            oid=row['observation_id'];require(type(oid) is int and oid>0,'invalid observation ID')
            key=(row['session_epoch'],oid);signature=json.dumps([effective,read,row['age_basis'],row['data']],sort_keys=True)
            require(key not in seen or seen[key]==signature,'cached sample renewed/changed original data')
            seen[key]=signature;data=row['data']
            require(type(data['id']) is int and data['id']==0 and data['orientation_raw']==25 and data['orientation_name']=='PITCH_270','unexpected fixture ID/orientation')
            require(data['orientation_frame']=='BODY_FRD' and data['distance_interpretation']=='sensor_report_no_vertical_or_terrain_projection' and data['physical_accuracy']=='unassessed','invented frame/terrain/accuracy claim')
            require(data['timestamp_semantics']=='ardupilot_message_send_time_not_certified_measurement_time','false measurement timestamp claim')
            for name in ('min_distance','max_distance','current_distance'):
                raw=data[name+'_cm'];quantity=data[name+'_m']
                require(type(raw) is int and 0<=raw<=65535,'invalid raw distance')
                require(quantity['known'] is True and quantity['unit']=='m' and math.isclose(number(quantity['value']),raw*.01,abs_tol=1e-10),'distance conversion/unknown mismatch')
            for value,unit,reason in ((data['covariance']['quantity'],'m^2','ardupilot_zero_unknown_or_invalid'),(data['signal_quality']['quantity'],'percent','unknown_unset_or_omitted')):
                require(value['known'] is False and value['value'] is None and value['unit']==unit and value['reason']==reason,'contradictory unknown covariance/quality quantity')
            require(type(data['covariance']['raw_cm2']) is int and data['covariance']['raw_cm2']==0,'unexpected fixture covariance')
            require(type(data['signal_quality']['raw']) is int and data['signal_quality']['raw']==0,'unexpected fixture quality')
            for key in ('horizontal_fov','vertical_fov'):
                value=data[key];quantity=value['quantity']
                require(value['raw_rad']==0 and quantity['known'] is False and quantity['value'] is None and quantity['unit']=='rad' and quantity['reason']=='unknown_unset_or_omitted','invented FOV extension')
            q=data['quaternion'];require(q['raw_wxyz']==[0,0,0,0] and q['unit_norm'] is False and q['usable_for_custom_orientation'] is False and q['required_for_custom_orientation'] is False,'invented custom-orientation extension')
            samples.append((event,row))
    require(status_count>=10,'distance status missing/early-only')
    return samples


def selected(events,proof,label):
    low,high=proof['windows'][label]['probe_elapsed']
    result={}
    for event,row in sample_rows(events):
        if high-2.5<=event['elapsed']<=high and row['fresh']:
            result[(row['session_epoch'],row['observation_id'])]=row
    require(len(result)>=8,'no advancing late range data '+label)
    require(len({r['data']['time_boot_ms'] for r in result.values()})>=8,'constant remote time in late range window '+label)
    return list(result.values())


def audit_probe(events,profile='distance'):
    require(profile=='distance' and bool(events),'invalid distance evidence profile')
    require(not [e for e in events if e['type']=='fail'] and len([e for e in events if e['type']=='pass' and e['key']=='distance_flight'])==1,'probe did not pass')
    sample_rows(events)
    proof={'result':'PASS','windows':{},'scope':'original local acquisition metadata and observed stock-helm height changes; no sensor accuracy guarantee'}
    for label,target in LEVELS.items():
        begin,end=phase(events,label+'.started'),phase(events,label+'.finished')
        require(3.8<=end-begin<=5.5,'range window duration invalid '+label)
        proof['windows'][label]={'probe_elapsed':[begin,end],'target_height_m':target}
        rows=selected(events,proof,label)
        distances=[r['data']['current_distance_m']['value'] for r in rows]
        require(abs(statistics.median(distances)-target)<=.6,'range level did not follow fixture height '+label)
        heights=[number(float(e['value'])) for e in events if e['key']=='NAV_ALTITUDE' and end-2.5<=e['elapsed']<=end]
        require(len(heights)>=8 and max(abs(x-target) for x in heights)<=.45,'MOOS height was not stable '+label)
        runtime_events=[e for e in events if e['type']=='runtime' and begin<=e['elapsed']<=end]
        require(len(runtime_events)>=15 and runtime_events[-1]['elapsed']>=end-.5,'late Runtime evidence missing')
        require(max([runtime_events[0]['elapsed']-begin,end-runtime_events[-1]['elapsed']]+[b['elapsed']-a['elapsed'] for a,b in zip(runtime_events,runtime_events[1:])])<=.5,'Runtime evidence gap')
        states=[fields(e['value']) for e in runtime_events]
        for value in states:
            require(all(yes(value[k]) for k in ('permission','armed','in_air','heartbeat_valid','position_valid','home_valid','landed_valid','estimator_valid')) and value['fc_mode']=='4','stock-helm ownership/prerequisites lost')
        counters=[int(value['nav_tx_accepted']) for value in states]
        require(counters==sorted(counters) and counters[-1]-counters[0]>=15,'range stream interrupted NAV progression')
        beginning=fields(next(e['value'] for e in events if e['type']=='phase' and e['key']==label+'.started'))
        ending=fields(next(e['value'] for e in events if e['type']=='phase' and e['key']==label+'.finished'))
        base,final=int(beginning['nav_tx_accepted']),int(ending['nav_tx_accepted'])
        require(base<=counters[0]<=counters[-1]<=final and final-base>=15,'phase NAV counters contradict Runtime progression')
        previous=base;last=begin;gaps=[]
        for event,value in zip(runtime_events,counters):
            if value>previous:gaps.append(event['elapsed']-last);previous=value;last=event['elapsed']
        gaps.append(end-last);require(max(gaps)<=.75,'NAV counter final-tail gap')
        proof['windows'][label].update({'late_unique_samples':len(rows),'median_range_m':statistics.median(distances),'nav_counter_delta':counters[-1]-counters[0]})
    require(2.5<=proof['windows']['height.high']['median_range_m']-proof['windows']['height.low']['median_range_m']<=3.5,'range did not change by three metres')
    require(abs(proof['windows']['height.return']['median_range_m']-proof['windows']['height.low']['median_range_m'])<=.3,'range did not return to initial level')
    updates=[e['value'] for e in events if e['type']=='request' and e['key']=='ALTITUDE_UPDATE']
    require(updates==['altitude=8','altitude=5'],'nonstandard or missing behavior updates')
    return proof


def audit_fc(packets,markers,finished):
    require(set(markers)==set(MARKERS),'missing independent phase markers')
    require(all(p['system']==p['component']==1 for p in packets),'unbound FC source')
    proof={'result':'PASS','windows':{}}
    for label,target in LEVELS.items():
        begin,end=markers[label+'.started'],markers[label+'.finished']
        require(3.5<=end-begin<=5.7,'independent window duration invalid')
        window=[p for p in packets if end-2.5<=p['mono']<=end]
        ranges=[p for p in window if p['mavpackettype']=='DISTANCE_SENSOR' and p['id']==0 and p['orientation']==25]
        positions=[p for p in window if p['mavpackettype']=='GLOBAL_POSITION_INT']
        targets=[p for p in window if p['mavpackettype']=='POSITION_TARGET_LOCAL_NED']
        heartbeats=[p for p in window if p['mavpackettype']=='HEARTBEAT']
        landed=[p for p in window if p['mavpackettype']=='EXTENDED_SYS_STATE']
        require(len({r['time_boot_ms'] for r in ranges})>=8 and len(positions)>=8 and len(targets)>=5,'late independent range/position/target reports missing')
        require(abs(statistics.median(p['current_distance']*.01 for p in ranges)-target)<=.6,'independent range level differs')
        require(max(abs(p['relative_alt']*.001-target) for p in positions)<=.6,'independent flight height unstable')
        require(len(heartbeats)>=2 and all(p['base_mode']&128 and p['custom_mode']==4 for p in heartbeats) and end-heartbeats[-1]['mono']<=1.5,'fresh armed Guided FC evidence missing')
        require(len(landed)>=2 and all(p['landed_state']==2 for p in landed) and end-landed[-1]['mono']<=1,'fresh in-air FC evidence missing')
        proof['windows'][label]={'range_samples':len(ranges),'position_samples':len(positions),'target_samples':len(targets)}
    for family,limit in [('HEARTBEAT',2),('GLOBAL_POSITION_INT',.75),('EXTENDED_SYS_STATE',2)]:
        values=[p for p in packets if p['mavpackettype']==family];require(values and 0<=finished-values[-1]['mono']<=limit,'final FC evidence stale '+family)
        if family=='HEARTBEAT':require(values[-1]['base_mode']&128==0,'final FC remained armed')
        if family=='EXTENDED_SYS_STATE':require(values[-1]['landed_state']==1,'final FC not on ground')
    return proof


def audit_distance_consistency(events,packets,proof):
    reports=[p for p in packets if p['mavpackettype']=='DISTANCE_SENSOR' and p['id']==0]
    result={'result':'PASS','comparison':'same-FC boot timestamps, <=150ms separation, <=0.05m distance difference; different source samples','windows':{}}
    for label in LEVELS:
        rows=selected(events,proof,label);matched=[];unmatched=[]
        for row in rows:
            data=row['data'];candidates=[p for p in reports if abs(p['time_boot_ms']-data['time_boot_ms'])<=150 and p['orientation']==data['orientation_raw'] and p['min_distance']==data['min_distance_cm'] and p['max_distance']==data['max_distance_cm'] and abs(p['current_distance']-data['current_distance_cm'])<=5]
            if candidates:matched.append(row['observation_id'])
            else:unmatched.append(row['observation_id'])
        require(len(matched)>=8 and len(matched)/len(rows)>=.8,'insufficient independently paired source values '+label)
        result['windows'][label]={'matched':len(matched),'selected':len(rows),'unmatched_ids':unmatched}
    return result


def audit_dataflash(events,records,proof):
    result={'result':'PASS','comparison':'RFND source values, same-FC boot interval; <=150ms and <=0.05m; no independent physical sensor reference','windows':{}}
    require(records,'no RFND DataFlash records')
    for label in LEVELS:
        rows=selected(events,proof,label);matched=[]
        for row in rows:
            data=row['data'];candidates=[r for r in records if r['Instance']==0 and r['Orient']==25 and abs(r['TimeUS']*.001-data['time_boot_ms'])<=150 and abs(r['Dist']-data['current_distance_m']['value'])<=.05]
            if candidates:matched.append(row['observation_id'])
        require(len(matched)>=8 and len(matched)/len(rows)>=.8,'insufficient RFND source comparisons '+label)
        result['windows'][label]={'matched':len(matched),'selected':len(rows)}
    return result
