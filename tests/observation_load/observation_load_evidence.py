"""Offline evidence gates for one owned observation-load flight.

MOOS elapsed windows and Python FC reception windows are evaluated separately;
their clock values are never subtracted from one another.
"""
import json
import hashlib
import math

BASE_MODULES={'telemetry','system_status','parameters','missions','arducopter_control'}
FAMILIES={'MAVMOOS_SYS_STATUS':3.0,'MAVMOOS_GPS_STATUS':3.0,'MAVMOOS_BATTERY_STATUS':3.0,
          'MAVMOOS_ATTITUDE_EULER':1.0,'MAVMOOS_ATTITUDE_QUATERNION':1.0}
TERMINAL={'COMPLETED','FAILED','CANCELLED','TIMED_OUT','INCOMPLETE','REJECTED'}

def number(value):
    if type(value) not in (int,float):raise ValueError('finite number required')
    try:valid=math.isfinite(value)
    except OverflowError:valid=False
    if not valid:raise ValueError('nonfinite number')
    return value

def parse(text):
    def reject(value):raise ValueError('nonfinite JSON '+value)
    def floating(value):return number(float(value))
    return json.loads(text,parse_constant=reject,parse_float=floating)

def fields(value):
    result={}
    for part in value.split(','):
        if '=' not in part:raise ValueError('malformed fields')
        key,text=part.split('=',1)
        if not key or key in result:raise ValueError('duplicate/empty field')
        result[key]=text
    return result

def yes(value):return value in ('1','true')

def phase(events,key):
    rows=[e for e in events if e['type']=='phase' and e['key']==key]
    if len(rows)!=1:raise ValueError('missing/repeated phase '+key)
    return number(rows[0]['elapsed'])

def counter(text):
    if not isinstance(text,str) or not text.isascii() or not text.isdigit() or not 0<=int(text)<2**64:raise ValueError('invalid accepted NAV counter')
    return int(text)

def advancing(rows,begin,end,timeout,key):
    result=[]
    for event in rows:
        value=parse(event['value'])
        if value.get('schema')!=1 or type(value.get('present')) is not bool or type(value.get('fresh')) is not bool:raise ValueError('invalid observer schema/presence '+key)
        if not value['present']:
            if value['fresh']:raise ValueError('fresh absent observation '+key)
            continue
        effective,read,age=[number(value[n]) for n in ('effective_at','read_at','age_s')]
        if effective<0 or read<effective or age<0 or age+1e-6<read-effective:raise ValueError('contradictory sample timing '+key)
        basis=value.get('age_basis')
        if basis=='QUEUE_EMPTY_FLOOR':
            if value.get('age_is_upper_bound') is not True:raise ValueError('missing conservative acquisition flag '+key)
        elif basis=='READ_ACQUISITION':
            if effective!=read or value.get('age_is_upper_bound') is not False:raise ValueError('contradictory read acquisition '+key)
        else:raise ValueError('unknown acquisition basis '+key)
        if value.get('source_age')!='unknown' or value.get('source_quality')!='unassessed':raise ValueError('invented remote age/quality '+key)
        if type(value.get('observation_id')) is not int or not 0<value['observation_id']<2**64:raise ValueError('invalid observation identity '+key)
        if value['fresh'] and age>timeout:raise ValueError('fresh observation exceeds timeout '+key)
        if not begin<=event['elapsed']<=end or not value['fresh'] or value.get('encoding_valid') is not True:continue
        if key=='MAVMOOS_BATTERY_STATUS':
            data=value['data'];voltage=data.get('reported_voltage_sum_v',{})
            if data.get('origin')!='BATTERY_STATUS' or data.get('battery_id')!=0 or type(data.get('battery_id')) is not int:continue
            if voltage.get('known') is not True or voltage.get('unit')!='V' or data.get('voltage_layout_valid') is not True or not 12.3<=number(voltage.get('value'))<=12.9:raise ValueError('battery fixture mismatch')
        result.append(value)
    if len({v['observation_id'] for v in result})<2 or len({v['read_at'] for v in result})<2:raise ValueError('no advancing fresh observations '+key)
    return result

def fixed_results(events,kind,ident,action,limit=8):
    rows=[fields(e['value']) for e in events if e['type']==kind and fields(e['value']).get('id')==ident]
    if not rows or any(r.get('action')!=action for r in rows):raise ValueError('missing/wrong result action '+ident)
    deadlines=[number(float(r['expires'])) for r in rows]
    if any(d<=0 or abs(d-deadlines[0])>1e-8 for d in deadlines):raise ValueError('original deadline changed '+ident)
    if not 0<deadlines[0]-number(float(rows[0]['at']))<=limit+1e-6:raise ValueError('original operation exceeded configured bound '+ident)
    terminals=[r for r in rows if r.get('status') in TERMINAL]
    if len(terminals)!=1 or terminals[0]['status']!='COMPLETED':raise ValueError('service terminal failed '+ident)
    at=number(float(terminals[0]['at']))
    if not 0<at<deadlines[0]:raise ValueError('late completion '+ident)
    return terminals[0]

def audit_probe(events,profile='attitude',selected_fixture=None):
    if not isinstance(selected_fixture,dict) or selected_fixture.get('scenario')!='armed_wiped_mission_read' or selected_fixture.get('qualification')!='not_inferred':raise ValueError('source-bound mission fixture missing')
    expected_mission=selected_fixture['expectation'];mission_count=expected_mission['expected_count']
    shapes=[e['value'] for e in events if e.get('type')=='assertion' and e.get('key')=='mission_fixture_shape']
    if shapes!=[expected_mission['shape']]:raise ValueError('probe mission fixture enum differs from source declaration')
    if profile not in ('status','attitude') or not events:raise ValueError('unsupported/empty evidence profile')
    for event in events:
        if not isinstance(event,dict) or number(event.get('elapsed'))<0:raise ValueError('invalid event')
        if event.get('type') not in ('request','phase','pass','fail','assertion') and event.get('source')!='iMAVLink':raise ValueError('unbound publication source')
    if any(e['type']=='fail' for e in events) or len([e for e in events if e['type']=='pass'])!=1:raise ValueError('probe did not uniquely pass')
    a,done,b=[phase(events,k) for k in ('load.started','reads.completed','load.finished')]
    if not 0<a<done<b or not 4<=done-a<=65 or not 4.9<=b-done<=6.5:raise ValueError('invalid read/continuation windows')
    requests=[e for e in events if e['type']=='request']
    expected_params={'observation-before':{'id':'observation-before','action':'read','name':'SIM_WIND_SPD'},
                     'observation-list':{'id':'observation-list','action':'list'},
                     'observation-after':{'id':'observation-after','action':'read','name':'SIM_WIND_SPD'}}
    params=[fields(e['value']) for e in requests if e['key']=='MAVMOOS_PARAM_REQUEST']
    if len(params)!=3 or {p['id']:p for p in params}!=expected_params:raise ValueError('parameter mutation or wrong requests')
    missions=[fields(e['value']) for e in requests if e['key']=='MAVMOOS_MISSION_REQUEST']
    if missions!=[{'id':'observation-mission','action':'download','timeout':'15'}]:raise ValueError('mission mutation or wrong read')
    controls=[e['value'] for e in requests if e['key']=='MAVMOOS_NAV_CONTROL']
    if controls!=['acquire']:raise ValueError('implicit/repeated authority acquisition')
    commands=[fields(e['value']) for e in requests if e['key']=='MAVMOOS_COMMAND']
    expected=[{'id':'observation-guided','action':'guided'},{'id':'observation-arm','action':'arm'},
              {'id':'observation-takeoff','action':'takeoff','height':'5'},{'id':'observation-land','action':'land'}]
    if commands not in (expected,expected+[{'id':'observation-disarm','action':'disarm'}]):raise ValueError('unexpected native workflow command')
    for command in commands:
        outcomes=[fields(e['value']) for e in events if e['type']=='command_result' and fields(e['value']).get('id')==command['id']]
        terminals=[r for r in outcomes if r.get('status') in TERMINAL]
        if len(terminals)!=1 or terminals[0]['status']!='COMPLETED':raise ValueError('native command result missing/failed '+command['id'])
    if any(e['key'] not in {'MAVMOOS_PARAM_REQUEST','MAVMOOS_MISSION_REQUEST','MAVMOOS_NAV_CONTROL','MAVMOOS_COMMAND','DEPLOY','MOOS_MANUAL_OVERRIDE'} for e in requests):raise ValueError('unexpected effect request')
    catalogs=[parse(e['value']) for e in events if e['type']=='catalog']
    if not catalogs or len({json.dumps(c,sort_keys=True) for c in catalogs})!=1:raise ValueError('missing/changing composition')
    catalog=catalogs[0];expected_modules=BASE_MODULES|({'attitude'} if profile=='attitude' else set())
    modules=catalog.get('composition',{}).get('modules',[])
    if catalog.get('schema')!=1 or catalog.get('execution_mode')!='SITL_CONTROL' or catalog.get('composition',{}).get('plugin_abi')!=10 or {m['name'] for m in modules}!=expected_modules or len(modules)!=len(expected_modules):raise ValueError('wrong composition/ABI')
    if {m['name'] for m in modules if m.get('control_provider')}!={'arducopter_control'}:raise ValueError('wrong control owner')
    for module in modules:
        if module['name'] in {'system_status','attitude'} and (module.get('control_provider') is not False or module.get('request_owners')!=[] or not module.get('capabilities') or any(c.get('access')!='observation' for c in module['capabilities'])):raise ValueError('observer owns effects')
    runtimes=[e for e in events if e['type']=='runtime' and a<=e['elapsed']<=b]
    if len(runtimes)<10 or runtimes[-1]['elapsed']<b-.75:raise ValueError('insufficient late runtime evidence')
    for event in runtimes:
        r=fields(event['value'])
        if not all(yes(r.get(k)) for k in ('permission','heartbeat_valid','position_valid','landed_valid','estimator_valid')) or r.get('fc_mode')!='4' or r.get('mission_mutations')!='0' or r.get('control_profile')!='arducopter':raise ValueError('navigation continuity lost during load')
    if max([runtimes[0]['elapsed']-a,b-runtimes[-1]['elapsed']]+[y['elapsed']-x['elapsed'] for x,y in zip(runtimes,runtimes[1:])])>.75:raise ValueError('runtime observation gap')
    helm=[e for e in events if e['type']=='helm' and a<=e['elapsed']<=b]
    if not helm or any('state=ACTIVE,' not in e['value'] for e in helm):raise ValueError('helm left active state')
    summaries=[fields(e['value']) for e in events if e['type']=='assertion' and e['key']=='load_continuity']
    if len(summaries)!=1:raise ValueError('missing continuity assertion')
    summary=summaries[0];claimed_gap=number(float(summary['max_nav_counter_update_gap_s']))
    start_event=next(e for e in events if e['type']=='phase' and e['key']=='load.started')
    end_event=next(e for e in events if e['type']=='phase' and e['key']=='load.finished')
    baseline=counter(fields(start_event['value'])['nav_tx_accepted']);final=counter(fields(end_event['value'])['nav_tx_accepted'])
    nav_rows=[e for e in events if e['type']=='nav_counter' and a<=e['elapsed']<=b]
    if len(nav_rows)<2:raise ValueError('missing logged accepted-NAV progression')
    increments=[(number(e['elapsed']),counter(fields(e['value'])['nav_tx_accepted'])) for e in nav_rows]
    if any(t<a or t>b or n<=baseline for t,n in increments) or any(t2<t1 or n2<=n1 for (t1,n1),(t2,n2) in zip(increments,increments[1:])):raise ValueError('nonadvancing/reordered accepted-NAV counter')
    if final!=increments[-1][1] or final<=baseline:raise ValueError('phase end counter differs from accepted-NAV log')
    gap=max([increments[0][0]-a,b-increments[-1][0]]+[y[0]-x[0] for x,y in zip(increments,increments[1:])])
    if not 0<=gap<=.75 or not 0<=claimed_gap<=.75 or abs(claimed_gap-gap)>.02 or counter(summary['nav_before'])!=baseline or counter(summary['nav_after'])!=final or summary['mission_mutations']!='0':raise ValueError('local accepted-target progress failed')
    for event in runtimes:
        accepted=counter(fields(event['value'])['nav_tx_accepted'])
        prior=[n for t,n in increments if t<=event['elapsed']]
        if accepted!=(prior[-1] if prior else baseline):raise ValueError('runtime and accepted-NAV logs contradict')
    parameter=fixed_results(events,'parameter_result','observation-list','list',60)
    if parameter.get('list_mode')!='paced':raise ValueError('list was not paced')
    for ident in ('observation-before','observation-after'):fixed_results(events,'parameter_result',ident,'read')
    wind=[]
    for ident in ('observation-before','observation-after'):
        values=[fields(e['value']) for e in events if e['type']=='parameter_value' and fields(e['value']).get('id')==ident]
        if not values or any(v.get('name')!='SIM_WIND_SPD' for v in values):raise ValueError('read value missing/wrong wind')
        current=[number(float(v['value'])) for v in values]
        if len(set(current))!=1:raise ValueError('conflicting wind read values')
        wind.append(current[0])
    if wind[0]!=wind[1]:raise ValueError('wind changed during read-only load')
    mission=fixed_results(events,'mission_result','observation-mission','download',15)
    if counter(mission.get('received'))!=mission_count or counter(mission.get('expected'))!=mission_count:raise ValueError('mission terminal count differs from source declaration')
    if any(fields(e['value']).get('mutation_may_have_applied')!='false' for e in events if e['type']=='mission_result'):raise ValueError('mission mutation uncertainty')
    snapshots=[parse(e['value']) for e in events if e['type']=='parameter_snapshot']
    if len(snapshots)!=1:raise ValueError('missing/repeated parameter snapshot')
    table=snapshots[0];count=table.get('count');entries=table.get('values',[])
    if table.get('schema')!=1 or table.get('id')!='observation-list' or table.get('status')!='complete' or table.get('list_mode')!='paced' or type(count) is not int or not 100<=count<=4096 or len(entries)!=count or sorted(e['index'] for e in entries)!=list(range(count)) or len({e['name'] for e in entries})!=count:raise ValueError('incomplete/non-distinct parameter table')
    for entry in entries:number(entry['value'])
    if int(parameter['received'])!=count or int(parameter['expected'])!=count or not number(table['completed_at'])<float(parameter['expires']):raise ValueError('parameter snapshot/terminal mismatch')
    mission_snapshots=[parse(e['value']) for e in events if e['type']=='mission_snapshot']
    if len(mission_snapshots)!=1:raise ValueError('missing/repeated mission snapshot')
    ms=mission_snapshots[0]
    if ms.get('id')!='observation-mission' or ms.get('schema')!=1 or ms.get('status')!='complete' or ms.get('mission_type')!=0 or type(ms.get('count')) is not int or ms['count']!=mission_count or ms.get('verification')!='observation' or ms.get('atomic') is not False or not isinstance(ms.get('items'),list) or len(ms['items'])!=mission_count:raise ValueError('mission observation differs from source-declared shape')
    homes=[fields(e['value']) for e in events if e['type']=='home'];homes=[h for h in homes if h.get('present')=='true' and h.get('encoding_valid')=='true' and h.get('freshness')=='REFERENCE_KNOWN' and h.get('identity')=='EXPECTED']
    if not homes:raise ValueError('HOME not independently bound')
    home=homes[-1];lat,lon,alt=[number(float(home[k])) for k in ('lat_deg','lon_deg','alt_msl_m')]
    if abs(lat)>90 or abs(lon)>180:raise ValueError('bound HOME coordinates invalid')
    if mission_count:
        item=ms['items'][0]
        if item.get('seq')!=0 or item.get('command')!=16 or item.get('frame')!=0 or item.get('current') not in (0,1) or item.get('autocontinue')!=1 or item.get('params')!=[0,0,0,0] or item.get('x')!=round(lat*1e7) or item.get('y')!=round(lon*1e7) or abs(number(item.get('z'))-alt)>.010001:raise ValueError('HOME raw fields mismatch')
    tx=[fields(e['value']) for e in events if e['type']=='mission_tx']
    if {int(r['msgid']) for r in tx}!=set(expected_mission['required_host_message_ids']) or any(r.get('operation_id')!='observation-mission' or r.get('status')!='LOCAL_ACCEPTED' or r.get('late')!='false' or r.get('clock_valid')!='true' for r in tx):raise ValueError('mission transport not accepted source-declared bounded read/ACK')
    families=FAMILIES if profile=='attitude' else {k:v for k,v in FAMILIES.items() if 'ATTITUDE' not in k}
    observations={}
    for key,timeout in families.items():
        rows=[e for e in events if e['type']=='observation' and e['key']==key]
        service=advancing(rows,max(a,done-5),done,timeout,key)
        late=advancing(rows,b-5,b,timeout,key)
        observations[key]={'during_read_unique':len({v['observation_id'] for v in service}),'continuation_unique':len({v['observation_id'] for v in late}),'max_during_read_age_s':max(v['age_s'] for v in service),'max_continuation_age_s':max(v['age_s'] for v in late)}
    for key in ('MAVMOOS_SYSTEM_STATUS',)+(('MAVMOOS_ATTITUDE_STATUS',) if profile=='attitude' else ()):
        aggregates=[(e,parse(e['value'])) for e in events if e['type']=='observation' and e['key']==key]
        if not any(b-5<=e['elapsed']<=b and v.get('state')=='BOUND' for e,v in aggregates):raise ValueError('missing bound late aggregate '+key)
        if any(v.get('read_only') is not True or v.get('authoritative_for_control') is not False or 'safe' in v or 'healthy' in v for _,v in aggregates):raise ValueError('observer asserted authority/safety')
        if key=='MAVMOOS_ATTITUDE_STATUS' and any(v.get('atomic') is not False for _,v in aggregates):raise ValueError('attitude families falsely atomic')
    return {'result':'PASS','profile':profile,'scope':'MOOS-visible coexistence and local send counters; not independent FC/wire proof',
        'read_service_window_s':done-a,'post_read_continuation_s':b-done,'table_count':count,'mission_count':mission_count,
        'mission_shape':expected_mission['shape'],'mission_item_records_exercised':mission_count,
        'scenario_fixture_sha256':selected_fixture['declaration_sha256'],'observer_families':observations,
        'nav_counter_update_gap_s':gap,'nav_counter_summary_gap_s':claimed_gap,'nav_counter_before':baseline,'nav_counter_after':final,
        'source_quality':'unassessed','source_age':'unknown','load_window_probe_elapsed':[a,b],
        'read_end_probe_elapsed':done,'catalog':catalog}

def distance(a,b):
    return math.hypot((a['lat']-b['lat'])/1e7*111195,(a['lon']-b['lon'])/1e7*111195*math.cos(math.radians(a['lat']/1e7)))

def audit_finish_boundary(events,coordinator,marker,acknowledgement):
    if not isinstance(marker,bytes) or not 0<len(marker)<=128 or acknowledgement!=marker:raise ValueError('finish marker/acknowledgement bytes mismatch')
    matches=[(i,e) for i,e in enumerate(events) if e['type']=='assertion' and e['key']=='finish_boundary_ack']
    if len(matches)!=1:raise ValueError('missing/repeated finish acknowledgement trace')
    index,event=matches[0];f=fields(event['value']);end=phase(events,'load.finished');at=number(event['elapsed'])
    if event.get('source')!='probe' or set(f)!={'token_hex','wait_s','timeout_s','nav_tx_accepted','max_nav_counter_update_gap_s'}:raise ValueError('invalid finish acknowledgement source/fields')
    wait=number(float(f['wait_s']));gap=number(float(f['max_nav_counter_update_gap_s']))
    if f['token_hex']!=marker.hex() or not 0<=wait<3 or f['timeout_s']!='3' or at<end or not 0<=gap<=.75:raise ValueError('unbound/late/noncontinuous finish acknowledgement')
    finish_index=next(i for i,e in enumerate(events) if e['type']=='phase' and e['key']=='load.finished')
    if index<=finish_index:raise ValueError('acknowledgement precedes finish publication')
    final=counter(fields(events[finish_index]['value'])['nav_tx_accepted']);accepted=counter(f['nav_tx_accepted'])
    updates=[e for e in events if e['type']=='nav_counter' and end<e['elapsed']<=at]
    counts=[counter(fields(e['value'])['nav_tx_accepted']) for e in updates]
    if accepted!=(counts[-1] if counts else final) or any(b<=a for a,b in zip([final]+counts,counts)):raise ValueError('finish acknowledgement NAV counter inconsistent')
    times=[end]+[number(e['elapsed']) for e in updates]+[at]
    if any(b<a for a,b in zip(times,times[1:])):raise ValueError('finish acknowledgement NAV trace reordered')
    actual_gap=max(b-a for a,b in zip(times,times[1:]))
    if actual_gap>.75 or actual_gap>gap+.02:raise ValueError('finish acknowledgement NAV tail gap')
    after=[]
    for i,e in enumerate(events):
        if e['type']!='request':continue
        post=(e['key']=='DEPLOY' and e['value']=='false')
        if e['key']=='MAVMOOS_PARAM_REQUEST':post=fields(e['value']).get('id')=='observation-after'
        if e['key']=='MAVMOOS_COMMAND':post=fields(e['value']).get('action') in ('land','disarm')
        if post:
            if i<=index or number(e['elapsed'])<at:raise ValueError('post-load action precedes finish acknowledgement')
            after.append(e['key'])
    if not {'MAVMOOS_PARAM_REQUEST','DEPLOY','MAVMOOS_COMMAND'}.issubset(after):raise ValueError('post-ack read/landing workflow missing')
    finishes=[e for e in coordinator if e['kind']=='load.finished'];acks=[e for e in coordinator if e['kind']=='load.finished.ack']
    digest=hashlib.sha256(marker).hexdigest()
    if len(finishes)!=1 or len(acks)!=1 or coordinator.index(acks[0])<=coordinator.index(finishes[0]) or number(acks[0]['python_mono'])<number(finishes[0]['python_mono']) or acks[0].get('token_sha256')!=digest:raise ValueError('coordinator finish/ack ordering or token mismatch')
    return {'result':'PASS','scope':'content-bound causal finish acknowledgement; separate probe/Python clock domains',
            'token_sha256':digest,'probe_ack_elapsed':at,'probe_wait_s':wait,'probe_nav_tail_gap_s':actual_gap,
            'python_finish_boundary':finishes[0]['python_mono'],'python_ack_recorded':acks[0]['python_mono']}

def audit_fc(packets,begin,done,end,finished,profile='attitude'):
    a,b,c,finish=[number(n) for n in (begin,done,end,finished)]
    if not 0<a<b<c<=finish:raise ValueError('invalid independent observer windows')
    if any(p.get('system')!=1 or p.get('component')!=1 or number(p.get('mono'))<0 for p in packets):raise ValueError('unbound/nonfinite independent sample')
    def rows(kind,start=a,stop=c):return [p for p in packets if p['mavpackettype']==kind and start<=p['mono']<=stop]
    def fresh_stream(kind,limit):
        stream=rows(kind)
        if len(stream)<2 or any(y['mono']<x['mono'] for x,y in zip(stream,stream[1:])):raise ValueError('insufficient independent '+kind)
        gap=max([stream[0]['mono']-a,c-stream[-1]['mono']]+[y['mono']-x['mono'] for x,y in zip(stream,stream[1:])])
        if gap>limit:raise ValueError('independent reception gap '+kind)
        return stream,gap
    heart,hgap=fresh_stream('HEARTBEAT',2)
    if any(p['custom_mode']!=4 or not p['base_mode']&128 for p in heart):raise ValueError('independent mode/arming continuity failed')
    poses,pgap=fresh_stream('GLOBAL_POSITION_INT',.5)
    for p in poses:
        for key in ('lat','lon','relative_alt','vx','vy'):number(p[key])
        if abs(p['lat'])>9e8 or abs(p['lon'])>18e8 or p['relative_alt']<4000:raise ValueError('independent airborne geometry invalid')
    displacement=max(distance(poses[0],p) for p in poses)
    if displacement<.5:raise ValueError('no independent motion during reads')
    targets,tgap=fresh_stream('POSITION_TARGET_LOCAL_NED',.75)
    if len({p['time_boot_ms'] for p in targets})<2:raise ValueError('target state was a repeated sample')
    moving=[]
    for p in targets:
        if p['coordinate_frame']!=1 or type(p['type_mask']) is not int or not 0<=p['type_mask']<=65535 or p['type_mask']&0x38 or p['type_mask']&7!=7:raise ValueError('not local-NED velocity/acceleration target state')
        velocity=[number(p[k]) for k in ('vx','vy','vz')]
        if math.hypot(*velocity[:2])>.1:moving.append(p)
    if len(moving)<2:raise ValueError('independent FC target state lacks moving velocity')
    late={}
    kinds=['SYS_STATUS','BATTERY_STATUS','GPS_RAW_INT']+(['ATTITUDE','ATTITUDE_QUATERNION'] if profile=='attitude' else [])
    for kind in kinds:
        for start,stop,label in ((max(a,b-5),b,'read'),(c-5,c,'post')):
            samples=rows(kind,start,stop)
            if len({p['mono'] for p in samples})<2:raise ValueError('no advancing late independent '+kind)
            if kind=='BATTERY_STATUS':
                voltage=[]
                for p in samples:
                    if p.get('id')!=0:continue
                    cells=p.get('voltages',[])+p.get('voltages_ext',[]);total=0;gap=False;valid=True;known=0
                    for index,value in enumerate(cells):
                        if type(value) is not int or not 0<=value<=65535:raise ValueError('invalid wire voltage')
                        present=value!=65535 and (index<10 or value!=0)
                        if present:
                            if gap:valid=False
                            total+=value/1000.;known+=1
                        else:gap=True
                    if valid and known and 12.3<=total<=12.9:voltage.append(p)
                if len({p['mono'] for p in voltage})<2:raise ValueError('wire battery0 fixture mismatch')
            late[kind+'_'+label]=len(samples)
    attitude_rows=rows('ATTITUDE')
    yaw_values=[number(p['yaw']) for p in attitude_rows]
    yaw_range=[min(yaw_values),max(yaw_values)] if yaw_values else None
    if profile=='attitude' and (not yaw_values or len([v for v in yaw_values if abs(v)>.4])<2):raise ValueError('East-first fixture did not exercise nonzero source yaw')
    allheart=[p for p in packets if p['mavpackettype']=='HEARTBEAT'];landed=[p for p in packets if p['mavpackettype']=='EXTENDED_SYS_STATE']
    if not allheart or not landed or allheart[-1]['base_mode']&128 or landed[-1]['landed_state']!=1 or not 0<=finish-allheart[-1]['mono']<=2 or not 0<=finish-landed[-1]['mono']<=2:raise ValueError('independent final disarmed/ground evidence missing/stale')
    return {'result':'PASS','scope':'passive second FC telemetry link; no observer writes',
        'load_window_python_monotonic':[a,c],'read_end_python_monotonic':b,'independent_displacement_m':displacement,
        'position_samples':len(poses),'target_state_samples':len(targets),'moving_target_state_samples':len(moving),
        'max_position_reception_gap_s':pgap,'max_target_state_reception_gap_s':tgap,'max_heartbeat_reception_gap_s':hgap,
        'late_counts':late,'final_disarmed':True,'final_on_ground':True,
        'source_yaw_range_rad':yaw_range,'source_yaw_scope':'FC-reported fused attitude, not physical yaw-tracking qualification',
        'target_semantics':'pinned AP Guided VelAccel report; received FC target-state gap, not an uplink-send or motor-update interval'}

def audit_attitude_consistency(events,packets,probe_windows,profile='attitude'):
    if profile=='status':return {'result':'NOT_REQUESTED','scope':'status-only comparison'}
    a,done,b=probe_windows
    result={}
    for key,kind in (('MAVMOOS_ATTITUDE_EULER','ATTITUDE'),('MAVMOOS_ATTITUDE_QUATERNION','ATTITUDE_QUATERNION')):
        observations={}
        for event in events:
            if event.get('type')!='observation' or event.get('key')!=key or not (max(a,done-5)<=event['elapsed']<=done or b-5<=event['elapsed']<=b):continue
            value=parse(event['value'])
            if value.get('present') is True and value.get('fresh') is True and value.get('encoding_valid') is True:observations[value['observation_id']]=value
        wire=[p for p in packets if p.get('mavpackettype')==kind]
        if len(observations)<2 or not wire:raise ValueError('missing attitude value pairing '+key)
        maximum={'source_timestamp_delta_ms':0,'orientation_error':0,'angular_rate_error_rad_s':0}
        for sample in observations.values():
            data=sample['data'];boot=data.get('time_boot_ms')
            if type(boot) is not int or not 0<=boot<2**32 or data.get('remote_clock')!='boot_ms_unmapped_uint32' or data.get('reference_frame')!='LOCAL_NED' or data.get('body_frame')!='BODY_FRD' or data.get('estimate')!='fused_fc_attitude' or data.get('orientation_valid') is not True or data.get('angular_velocity_valid') is not True:raise ValueError('attitude convention/validity differs')
            for packet in wire:
                if type(packet.get('time_boot_ms')) is not int or not 0<=packet['time_boot_ms']<2**32:raise ValueError('invalid independent boot timestamp')
                if kind=='ATTITUDE' and any(abs(number(packet[k]))>math.pi+1e-6 for k in ('roll','pitch','yaw')):raise ValueError('independent Euler declared range violated')
            paired=min(wire,key=lambda p:abs(p['time_boot_ms']-boot));delta=abs(paired['time_boot_ms']-boot)
            if delta>300:raise ValueError('attitude source timestamps too far apart')
            maximum['source_timestamp_delta_ms']=max(maximum['source_timestamp_delta_ms'],delta)
            if kind=='ATTITUDE':
                values=data.get('raw_angles_rad',[])
                if len(values)!=3:raise ValueError('missing raw Euler angles')
                if any(abs(number(x))>math.pi+1e-6 for x in values) or data.get('wire_declared_angle_range_conformant') is not True or data.get('profile_angle_range')!='plusminus_pi_with_float_tolerance':raise ValueError('Euler raw declared range violated')
                validate_rpy(data.get('reported_rpy',{}),values,True)
                errors=[abs(math.atan2(math.sin(number(x)-number(paired[k])),math.cos(x-paired[k]))) for x,k in zip(values,('roll','pitch','yaw'))]
            else:
                values=data.get('quaternion_wxyz',[])
                if len(values)!=4:raise ValueError('missing raw quaternion')
                for x in values:number(x)
                other=[number(paired[k]) for k in ('q1','q2','q3','q4')]
                length=math.sqrt(sum(x*x for x in values))
                if abs(length-1)>.001 or abs(math.sqrt(sum(x*x for x in other))-1)>.001:raise ValueError('nonunit quaternion pairing')
                if abs(number(data.get('quaternion_norm'))-length)>1e-9 or abs(number(data.get('unit_norm_tolerance'))-.001)>1e-12 or data.get('quaternion_components_finite') is not True or data.get('derived_normalized_within_tolerance') is not True or data.get('derived_convention')!='hamilton_wxyz_zyx_in_source_ned_frd':raise ValueError('quaternion metadata/derivation contract differs')
                w,x,y,z=[v/length for v in values]
                rpy=[math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),math.asin(max(-1,min(1,2*(w*y-z*x)))),math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))]
                validate_rpy(data.get('derived_rpy',{}),rpy,False)
                errors=[min(math.sqrt(sum((x-y)**2 for x,y in zip(values,other))),math.sqrt(sum((x+y)**2 for x,y in zip(values,other))))]
                offset=data.get('representation_offset',{});raw=offset.get('raw_wxyz',[])
                if len(raw)!=4 or any(type(v) not in (int,float) or not math.isfinite(v) for v in raw):raise ValueError('invalid offset components')
                zero=all(v==0 for v in raw);unit=abs(math.sqrt(sum(v*v for v in raw))-1)<=.001
                if not(zero or unit) or offset.get('status')!=('unsupported_or_omitted' if zero else 'unit_offset_unapplied') or offset.get('encoding_valid') is not True or type(offset.get('extension_bytes_observed')) is not bool or (not zero and not offset['extension_bytes_observed']) or offset.get('display_adjustment_applied') is not False:raise ValueError('unsupported display-offset adjustment/encoding')
            orientation=max(errors)
            if orientation>.1:raise ValueError('MOOS/native attitude convention/value mismatch')
            rates=data.get('angular_velocity_body_frd_rad_s',{})
            errors=[]
            for axis,field in zip(('x','y','z'),('rollspeed','pitchspeed','yawspeed')):
                quantity=rates.get(axis,{})
                if quantity.get('known') is not True or quantity.get('unit')!='rad/s':raise ValueError('body angular rate unknown/wrong unit')
                errors.append(abs(number(quantity.get('value'))-number(paired[field])))
            if max(errors)>1:raise ValueError('MOOS/native angular rate mismatch')
            maximum['orientation_error']=max(maximum['orientation_error'],orientation)
            maximum['angular_rate_error_rad_s']=max(maximum['angular_rate_error_rad_s'],max(errors))
        result[kind]={'paired_distinct_observations':len(observations),**maximum}
    return {'result':'PASS','scope':'nearest same-family FC boot timestamp value check; no cross-family fusion, timestamp synchronization or source-age claim',
        'maximum_pair_window_ms':300,'Euler_error_unit':'rad wrapped independently','quaternion_error':'sign-equivalent Euclidean component distance',
        'families':result}

def validate_rpy(reported,expected,source):
    unique=abs(math.cos(expected[1]))>1e-6
    if reported.get('rpy_unique') is not unique:raise ValueError('incorrect RPY uniqueness')
    for index,key in enumerate(('roll_rad','pitch_rad','yaw_rad')):
        quantity=reported.get(key,{})
        known=source or unique or index==1
        if quantity.get('known') is not known or quantity.get('unit')!='rad':raise ValueError('incorrect reported RPY units/knownness')
        if known:
            value=number(quantity.get('value'))
            if abs(value-expected[index])>1e-8:raise ValueError('incorrect reported/derived RPY')
        elif quantity.get('value') is not None:raise ValueError('singular RPY reported a value')
    heading=reported.get('source_yaw_heading_deg',{})
    if heading.get('known') is not unique or heading.get('unit')!='deg':raise ValueError('incorrect heading units/knownness')
    if unique:
        target=math.degrees(expected[2])%360
        if target>=360:target=0
        value=number(heading.get('value'))
        if not 0<=value<360 or abs(value-target)>1e-7:raise ValueError('incorrect source yaw heading')
    elif heading.get('value') is not None:raise ValueError('singular yaw produced a heading')
