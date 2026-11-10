"""Offline read-only IMU acceptance. Consumer, host and FC clocks stay distinct."""
import json
import math

FAMILIES={27:('RAW_IMU',None),26:('SCALED_IMU',0),116:('SCALED_IMU2',1),129:('SCALED_IMU3',2)}


def parse(text):
    def unique(pairs):
        result={}
        for key,value in pairs:
            if key in result:raise ValueError('duplicate JSON key '+key)
            result[key]=value
        return result
    def invalid(value):raise ValueError('nonfinite JSON '+value)
    def floating(value):
        result=float(value)
        if not math.isfinite(result):raise ValueError('nonfinite JSON float')
        return result
    return json.loads(text,parse_constant=invalid,parse_float=floating,object_pairs_hook=unique)


def number(value):
    if type(value) not in (float,int):raise ValueError('expected finite number')
    try:valid=math.isfinite(value)
    except OverflowError:valid=False
    if not valid:raise ValueError('expected finite number')
    return value


def integer(value,low,high):
    if type(value) is not int or not low<=value<=high:raise ValueError('integer outside schema bounds')
    return value


def fields(text):
    result={}
    for part in text.split(','):
        if '=' not in part:raise ValueError('malformed status field')
        key,value=part.split('=',1)
        if not key or key in result:raise ValueError('duplicate/empty status field')
        result[key]=value
    return result


def observation(record,seen):
    if type(record.get('schema')) is not int or record['schema']!=1 or record.get('present') is not True or record.get('encoding_valid') is not True:raise ValueError('invalid present IMU record')
    epoch=integer(record.get('session_epoch'),1,2**64-1);oid=integer(record.get('observation_id'),1,2**64-1)
    effective,read,age=(number(record.get(key)) for key in ('effective_at','read_at','age_s'))
    if effective<0 or read<effective or age<0 or age+1e-6<read-effective:raise ValueError('contradictory original observation timestamps')
    if type(record.get('fresh')) is not bool or record['fresh'] is not (age<=1):raise ValueError('freshness does not match configured1s original age')
    if record.get('age_basis')=='QUEUE_EMPTY_FLOOR':
        if record.get('age_is_upper_bound') is not True:raise ValueError('queue age bound lost')
    elif record.get('age_basis')=='READ_ACQUISITION':
        if read!=effective or record.get('age_is_upper_bound') is not False:raise ValueError('read acquisition clock contradiction')
    else:raise ValueError('unknown acquisition basis')
    if record.get('source_age')!='unknown' or record.get('source_quality')!='unassessed':raise ValueError('invented source freshness/quality')
    data=record.get('data',{});mid=integer(data.get('message_id'),0,2**24-1)
    if mid not in FAMILIES or data.get('family')!=FAMILIES[mid][0]:raise ValueError('unknown/mismatched sensor family')
    instance=integer(data.get('sensor_instance'),0,255)
    if mid!=27 and instance!=FAMILIES[mid][1]:raise ValueError('scaled logical instance changed')
    version=integer(data.get('wire_version'),1,2);length=integer(data.get('payload_length'),1,29 if mid==27 else 24)
    if version==1 and length!=(26 if mid==27 else 22):raise ValueError('invalid MAVLink1 wire length')
    if type(data.get('remote_timestamp_regressed')) is not bool:raise ValueError('missing remote-clock uncertainty')
    integer(data.get('remote_timestamp'),0,2**64-1 if mid==27 else 2**32-1)
    expected_clock='ardupilot_boot_us_unmapped_uint64' if mid==27 else 'boot_ms_unmapped_uint32'
    if data.get('remote_clock')!=expected_clock or data.get('timestamp_semantics')!='ardupilot_message_send_time_not_certified_measurement_time':raise ValueError('invented measurement clock')
    identity=data.get('identity_basis')
    expected_identity='message_family_logical_instance' if mid!=27 else 'mavlink1_no_id_extension' if version==1 else 'reported_extension_id' if instance else 'zero_default_or_omitted_extension_id'
    if identity!=expected_identity or (mid==27 and version==1 and instance!=0):raise ValueError('sensor identity extension contradiction')
    if data.get('body_frame')!='BODY_FRD' or data.get('estimate')!='sensor_report_not_fused_attitude':raise ValueError('invented frame/estimate')
    if data.get('physical_sensor_identity')!='unverified' or data.get('sensor_presence')!='unknown' or data.get('sensor_health')!='unassessed' or data.get('physical_accuracy')!='unassessed':raise ValueError('invented physical sensor inference')
    if data.get('covariance','absent') is not None or data.get('gravity_removed') is not False or 'orientation' in data:raise ValueError('invented fused/gravity/covariance data')
    expected_units={'acceleration':'milli-g','angular_velocity':'mrad/s','magnetic_field':'milligauss'}
    if data.get('raw_units')!=expected_units:raise ValueError('raw units changed')
    if data.get('unit_basis')!=('ardupilot_raw_imu_scaled_sender_convention' if mid==27 else 'mavlink_scaled_imu_units'):raise ValueError('raw sender-unit convention missing')
    for key,unit,scale in (('acceleration','m/s^2',9.80665/1000),('angular_velocity','rad/s',.001),('magnetic_field','T',1e-7)):
        raw=data.get('raw_'+key);converted=data.get(key,{})
        if not isinstance(raw,list) or len(raw)!=3 or any(type(n) is not int or not -32768<=n<=32767 for n in raw):raise ValueError('raw signed axes invalid')
        values=converted.get('value')
        if converted.get('conversion_known') is not True or converted.get('unit')!=unit or not isinstance(values,list) or len(values)!=3:raise ValueError('SI conversion metadata invalid')
        if any(not math.isclose(number(got),want*scale,rel_tol=1e-12,abs_tol=1e-14) for got,want in zip(values,raw)):raise ValueError('SI conversion differs from raw wire value')
    temp=data.get('temperature',{});raw=temp.get('raw_cdegC')
    if temp.get('unit')!='degC':raise ValueError('temperature unit changed')
    if version==1:
        if raw is not None or temp.get('known') is not False or temp.get('value') is not None or temp.get('reason')!='mavlink1_extension_absent':raise ValueError('invented MAVLink1 temperature')
    else:
        integer(raw,-32768,32767);known=raw!=0
        reason='zero_unknown_or_omitted' if raw==0 else 'reported_0_01C_protocol_zero_degree_encoding_ambiguous' if raw==1 else 'reported_temperature'
        if temp.get('known') is not known or temp.get('reason')!=reason:raise ValueError('temperature sentinel/ambiguity changed')
        if (known and not math.isclose(number(temp.get('value')),raw*.01,rel_tol=1e-12,abs_tol=1e-12)) or (not known and temp.get('value') is not None):raise ValueError('temperature conversion invented/changed')
    signature=json.dumps([effective,read,record['age_basis'],data],sort_keys=True)
    key=(epoch,oid)
    if key in seen and seen[key]['signature']!=signature:raise ValueError('cached observation identifier renewed/changed')
    if key in seen and age+1e-6<seen[key]['age']:raise ValueError('cached observation age regressed')
    seen[key]={'signature':signature,'age':age}
    return (mid,instance),epoch,oid


def audit_probe(rows, *, expect_vibration=False):
    expected_capabilities = {'imu.acceleration', 'imu.angular_velocity', 'imu.magnetic_field', 'imu.temperature'}
    if expect_vibration:
        expected_capabilities.add('imu.vibration')
    if not isinstance(rows,list) or not rows:raise ValueError('empty MOOS observations')
    seen={};samples=[];statuses=[];runtimes=[];app=[];catalogs=[];last_elapsed=-1;last_epoch=0
    for row in rows:
        if row.get('source')!='iMAVLink':raise ValueError('foreign MOOS source')
        elapsed=number(row.get('elapsed'))
        if not 0<=elapsed<=26 or elapsed<last_elapsed:raise ValueError('consumer elapsed clock invalid/regressed')
        last_elapsed=elapsed;key=row.get('key');text=row.get('value')
        if not isinstance(text,str):raise ValueError('MOOS observation must be original string')
        if key=='MAVMOOS_IMU_SENSORS':
            value=parse(text);epoch=integer(value.get('session_epoch'),1,2**64-1)
            if type(value.get('schema')) is not int or value['schema']!=1 or value.get('atomic') is not False or type(value.get('cache_capacity')) is not int or value['cache_capacity']!=8 or epoch<last_epoch:raise ValueError('cache/session contract changed')
            last_epoch=epoch;values=value.get('sensors')
            if not isinstance(values,list) or len(values)>8 or value.get('present') is not bool(values):raise ValueError('cache bound/presence contradiction')
            keys=[]
            for sample in values:
                sensor_key,session,oid=observation(sample,seen)
                if session!=epoch or sensor_key in keys:raise ValueError('cross-session/duplicate sensor sample')
                keys.append(sensor_key);samples.append((elapsed,epoch,oid,sensor_key,sample))
        elif key=='MAVMOOS_IMU_STATUS':
            value=parse(text)
            if type(value.get('schema')) is not int or value['schema']!=1 or value.get('read_only') is not True or value.get('authoritative_for_control') is not False or value.get('atomic') is not False or value.get('family_selection')!='independent_no_precedence_or_fusion':raise ValueError('status infers control/fusion')
            state=value.get('state');floor=number(value.get('session_boundary'));epoch=integer(value.get('session_epoch'),1,2**64-1)
            if state not in ('BOUND','WAITING_HEARTBEAT') or floor<0:raise ValueError('native clock fault/unknown binding state')
            if state=='BOUND' and (value.get('system_id')!=1 or value.get('component_id')!=1):raise ValueError('bound source identity changed')
            if state=='WAITING_HEARTBEAT' and (value.get('system_id') is not None or value.get('component_id') is not None):raise ValueError('unbound source carries identity')
            for name in ('metadata_rejected','source_rejected','wire_shape_rejected','out_of_order_rejected','sensor_capacity_rejected'):integer(value.get(name),0,2**64-1)
            nested=value.get('sensors',{})
            if nested.get('session_epoch')!=epoch or not isinstance(nested.get('sensors'),list) or len(nested['sensors'])>8:raise ValueError('status/cache session/bound differs')
            for sample in nested.get('sensors',[]):
                # Separate replay map: status and data publications may interleave.
                observation(sample,{})
                if sample['effective_at']<=floor or sample['session_epoch']!=epoch:raise ValueError('pre-floor/cross-session data published')
            statuses.append((elapsed,value))
        elif key=='MAVMOOS_RUNTIME_STATUS':
            value=fields(text)
            if value.get('mode')!='RX_ONLY' or value.get('permission')!='false' or value.get('control_profile')!='none':raise ValueError('runtime control composition active')
            if any(value.get(name)!='0' for name in ('tx_attempts','tx_accepted','nav_tx_accepted')):raise ValueError('runtime reports send effect')
            if value.get('armed') not in ('0','false') or value.get('in_air') not in ('0','false'):raise ValueError('native observation not disarmed/ground')
            runtimes.append((elapsed,value))
        elif key=='MAVMOOS_STATUS':
            value=fields(text)
            if value.get('state')!='RX_ONLY' or value.get('tx_enabled')!='false' or value.get('helm')!='disabled' or value.get('mavlink_tx_attempts')!='0' or value.get('mavlink_tx_accepted')!='0':raise ValueError('app not receive-only')
            app.append((elapsed,value))
        elif key=='MAVMOOS_CAPABILITIES':
            value=parse(text)
            if type(value.get('schema')) is not int or value['schema']!=1 or value.get('execution_mode')!='RX_ONLY' or type(value.get('composition',{}).get('plugin_abi')) is not int or value['composition']['plugin_abi']!=10:raise ValueError('catalog execution/ABI mismatch')
            modules=value['composition'].get('modules',[]);names=[m['name'] for m in modules]
            if len(names)!=len(set(names)) or set(names)!={'telemetry','imu'}:raise ValueError('unexpected loaded modules')
            if any(m.get('control_provider') is not False or m.get('request_owners')!=[] for m in modules):raise ValueError('observer module owns control requests')
            declarations=[c['name'] for m in modules if m['name']=='imu' for c in m['capabilities']];capabilities=set(declarations)
            if len(declarations)!=len(expected_capabilities) or capabilities!=expected_capabilities:raise ValueError('wrong/duplicate loaded IMU capability family')
            catalogs.append(value)
        elif key!='MAVMOOS_FIRMWARE_STATUS':raise ValueError('unexpected probe variable')
    if not statuses or not runtimes or not app or not catalogs:raise ValueError('missing app/runtime/status/catalog evidence')
    if len({json.dumps(c,sort_keys=True) for c in catalogs})!=1:raise ValueError('catalog composition changed')
    fresh=[s for s in samples if s[4]['fresh']]
    late=[s for s in fresh if s[0]>=20]
    if len({(s[1],s[2]) for s in fresh})<4 or len({(s[1],s[2]) for s in late})<2:raise ValueError('missing advancing/late fresh bound sensor data')
    stale=[s for s in samples if not s[4]['fresh']]
    if not stale:raise ValueError('stale original-age sample not observed during pause')
    # The three outcomes are ordered on consumer elapsed time, not mapped to the
    # coordinator's Python or FC boot clock. No sample is filtered or backdated.
    first=min(s[0] for s in fresh);lost=[(at,s) for at,s in statuses if at>first and s['state']=='WAITING_HEARTBEAT' and s['sensors'].get('sensors')==[]]
    if not lost:raise ValueError('heartbeat-loss clearing not observed')
    loss_at,loss=lost[0]
    recovered=[s for s in late if s[0]>loss_at and s[1]>=loss['session_epoch']]
    if not recovered or recovered[0][1]<=fresh[0][1]:raise ValueError('new post-loss sensor epoch not observed')
    if not any(s[0]>first and s[0]<loss_at for s in stale):raise ValueError('stale age did not precede cache clearing')
    if not any(s[4]['age_basis']=='QUEUE_EMPTY_FLOOR' for s in fresh):raise ValueError('conservative original-age basis absent')
    return {'schema':1,'result':'PASS','scope':'actual MOOS sensor JSON/raw-to-SI self-consistency, bound epochs and self-reported zero-send counters; separate passive channel is not exact-sample or primary-egress proof',
            'records':len(rows),'sensor_observations':len({(s[1],s[2]) for s in samples}),'fresh_samples':len(fresh),'late_fresh_samples':len(late),'families_observed':sorted({s[3][0] for s in fresh}),
            'logical_keys':sorted({s[3] for s in fresh}),'physical_sensor_count':'unknown','heartbeat_loss_elapsed':loss_at,'epochs':sorted({s[1] for s in samples}),'raw_scaling_checked':True,'stale_then_cleared_then_new_epoch':True,
            'io_status_records':len(app),'runtime_status_records':len(runtimes),'zero_tx_counters_observed':True,'authority_granted':False,'loaded_modules':['imu','telemetry']}


def audit_fc(rows,finish):
    if not rows:raise ValueError('empty passive FC observations')
    number(finish);heartbeats=[];ground=[];sensors=[];last=-1
    for row in rows:
        at=number(row.get('mono'))
        if at<last:raise ValueError('passive local read clock regressed')
        last=at
        if row.get('system')!=1 or row.get('component')!=1:raise ValueError('foreign passive FC source')
        kind=row.get('mavpackettype')
        if kind=='HEARTBEAT':
            if row.get('type')!=2 or row.get('autopilot')!=3 or integer(row.get('base_mode'),0,255)&128:raise ValueError('passive FC identity/arming contradiction')
            heartbeats.append(row)
        elif kind=='EXTENDED_SYS_STATE':
            if row.get('landed_state')!=1:raise ValueError('passive FC not ground')
            ground.append(row)
        elif kind in {f[0] for f in FAMILIES.values()}:sensors.append(row)
    if len(heartbeats)<3 or not ground or not sensors:raise ValueError('missing passive source/ground/sensor reports')
    if not 0<=finish-heartbeats[-1]['mono']<=2 or not 0<=finish-ground[-1]['mono']<=2:raise ValueError('final passive state stale')
    late=[s for s in sensors if 0<=finish-s['mono']<=5]
    if len(late)<2:raise ValueError('missing late passive sensor reports')
    return {'schema':1,'result':'PASS','scope':'separately observed exact native endpoint; no exact primary/app-sample equivalence or physical accuracy inferred','ever_armed':False,'final_ground':True,'heartbeat_samples':len(heartbeats),'sensor_samples':len(sensors),'late_sensor_samples':len(late),'families':sorted({s['mavpackettype'] for s in sensors}),'last_heartbeat_age_s':finish-heartbeats[-1]['mono'],'last_ground_age_s':finish-ground[-1]['mono']}


def corroborate(provider,fc):
    names={FAMILIES[mid][0] for mid in provider['families_observed']}
    common=sorted(names&set(fc['families']))
    if not common:raise ValueError('passive endpoint did not corroborate any app sensor family')
    return {'common_message_families':common,'same_samples_compared':False,'clock_mapping_inferred':False,'physical_sensor_health_inferred':False,'scope':'family/source corroboration across independent channels; SI scaling is checked against each MOOS record own raw fields'}
