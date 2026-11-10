#!/usr/bin/env python3
"""Audit a read-only native MOOS observation trace; never launch/connect/send."""
import argparse
import hashlib
import json
import math
from pathlib import Path

def parse(text):
    def reject(value):raise ValueError('nonfinite JSON '+value)
    def finite_float(value):
        result=float(value)
        if not math.isfinite(result):raise ValueError('overflowed JSON float '+value)
        return result
    return json.loads(text,parse_constant=reject,parse_float=finite_float)

def number(value):
    if type(value) not in (int,float):raise ValueError('numeric metadata must not be bool/string/null')
    try:valid=math.isfinite(value)
    except OverflowError:valid=False
    if not valid:raise ValueError('nonfinite numeric metadata')
    return value

def fixture_battery(record):
    data=record.get('data',{})
    if data.get('origin')!='BATTERY_STATUS' or type(data.get('battery_id')) is not int or data['battery_id']!=0:return False
    voltage=data.get('reported_voltage_sum_v',{})
    if voltage.get('known') is not True or voltage.get('unit')!='V' or data.get('voltage_layout_valid') is not True:return False
    return 12.3<=number(voltage.get('value'))<=12.9

def audit(rows,status_timeout=3.0,duration=25.0):
    if not rows:raise ValueError('empty trace')
    if not .25<=number(status_timeout)<=30 or not 5<=number(duration)<=60:raise ValueError('invalid audit timing configuration')
    late=duration-5
    known={'MAVMOOS_SYS_STATUS','MAVMOOS_BATTERY_STATUS','MAVMOOS_GPS_STATUS','MAVMOOS_FC_TEXT','MAVMOOS_SYSTEM_STATUS'}
    seen={key:[] for key in known};runtimes=[];catalogs=[];floor_seen=False
    for row in rows:
        if row.get('source')!='iMAVLink':raise ValueError('unexpected source')
        elapsed=number(row.get('elapsed'))
        if not 0<=elapsed<=duration+1:raise ValueError('invalid probe elapsed time')
        key=row['key'];text=row['value']
        if key in known:
            record=parse(text)
            if record.get('schema')!=1:raise ValueError('unsupported status schema')
            record['_probe_elapsed']=elapsed
            seen[key].append(record)
            if key in ('MAVMOOS_SYS_STATUS','MAVMOOS_BATTERY_STATUS','MAVMOOS_GPS_STATUS'):
                if type(record.get('present')) is not bool or type(record.get('fresh')) is not bool:raise ValueError('sample presence/freshness must be boolean')
                if not record['present'] and record['fresh']:raise ValueError('absent sample cannot be fresh')
            if key in ('MAVMOOS_SYS_STATUS','MAVMOOS_BATTERY_STATUS','MAVMOOS_GPS_STATUS') and record.get('present') is True:
                effective=number(record['effective_at']);read=number(record['read_at']);age=number(record['age_s'])
                if effective<0 or read<effective or age<0 or age+1e-6<read-effective:raise ValueError('invalid observation stamps/age contradiction')
                if record.get('fresh') is not True and record.get('fresh') is not False:raise ValueError('fresh must be boolean')
                if record['fresh'] and age>status_timeout:raise ValueError('fresh observation exceeds configured timeout')
                if type(record.get('observation_id')) is not int or not 1<=record['observation_id']<2**64:raise ValueError('invalid observation identifier')
                if record.get('source_age')!='unknown' or record.get('source_quality')!='unassessed':raise ValueError('invented source quality/clock')
                if record['age_basis']=='QUEUE_EMPTY_FLOOR':
                    if record.get('age_is_upper_bound') is not True:raise ValueError('lost conservative age flag')
                    floor_seen=True
                elif record['age_basis']=='READ_ACQUISITION':
                    if effective!=read or record.get('age_is_upper_bound') is not False:raise ValueError('contradictory read-acquisition metadata')
                else:raise ValueError('unknown acquisition basis')
        elif key=='MAVMOOS_RUNTIME_STATUS':
            fields=dict(part.split('=',1) for part in text.split(',') if '=' in part)
            if fields.get('mode')!='RX_ONLY' or fields.get('permission')!='false' or fields.get('control_profile')!='none':raise ValueError('not read-only/no-control composition')
            for name in ('tx_attempts','tx_accepted','nav_tx_accepted'):
                if fields.get(name)!='0':raise ValueError('nonzero or missing runtime send counter '+name)
            if fields.get('armed') not in ('0','false') or fields.get('in_air') not in ('0','false'):raise ValueError('native observation fixture not disarmed/on-ground')
            fields['_probe_elapsed']=elapsed;runtimes.append(fields)
        elif key=='MAVMOOS_CAPABILITIES':catalogs.append(parse(text))
    if not runtimes or not catalogs:raise ValueError('missing runtime/capability observations')
    if len({r['_probe_elapsed'] for r in runtimes if r['_probe_elapsed']>=late and r.get('heartbeat_valid') in ('true','1')})<2:raise ValueError('missing advancing late fresh-heartbeat runtime observations')
    for catalog in catalogs:
        if catalog.get('execution_mode')!='RX_ONLY' or catalog.get('composition',{}).get('plugin_abi')!=10:raise ValueError('wrong execution/ABI')
        modules={m['name']:m for m in catalog['composition']['modules']}
        if set(modules)!={'telemetry','system_status'}:raise ValueError('unexpected composition')
        if modules['system_status'].get('control_provider') is not False or modules['system_status'].get('request_owners')!=[]:raise ValueError('status owns effects or missing descriptor')
    for key in ('MAVMOOS_SYS_STATUS','MAVMOOS_BATTERY_STATUS','MAVMOOS_GPS_STATUS'):
        fresh=[r for r in seen[key] if r.get('present') is True and r.get('fresh') is True and r['_probe_elapsed']>=late]
        if key=='MAVMOOS_BATTERY_STATUS':fresh=[r for r in fresh if fixture_battery(r)]
        if len({r['observation_id'] for r in fresh})<2 or len({r['read_at'] for r in fresh})<2:raise ValueError('no advancing fresh source samples for '+key)
    if not floor_seen:raise ValueError('conservative acquisition basis not observed')
    aggregates=seen['MAVMOOS_SYSTEM_STATUS']
    bound=[r for r in aggregates if r.get('state')=='BOUND' and r['_probe_elapsed']>=late]
    if not bound:raise ValueError('source never bound')
    for state in aggregates:
        if state.get('read_only') is not True or state.get('authoritative_for_control') is not False:raise ValueError('control authority assertion')
        if type(state['battery_ids']) is not int or not 0<=state['battery_ids']<=8 or type(state['text_assemblies']) is not int or not 0<=state['text_assemblies']<=4:raise ValueError('cache bound/type exceeded')
        if 'safe' in state or 'healthy' in state:raise ValueError('aggregate safety inference')
    text=seen['MAVMOOS_FC_TEXT']
    return {'schema':1,'result':'pass','scope':'MOOS-published native observations and self-reported zero-send counters; not independent wire/hardware evidence',
            'record_counts':{key:len(value) for key,value in seen.items()},'runtime_records':len(runtimes),
            'identified_battery_ids':sorted({r['data']['battery_id'] for r in seen['MAVMOOS_BATTERY_STATUS'] if r.get('present') and r['data'].get('origin')=='BATTERY_STATUS'}),
            'text_observed':bool(text),'conservative_acquisition_observed':floor_seen,
            'status_timeout_s':status_timeout,'probe_duration_s':duration,'required_late_window_s':[late,duration],
            'fixture_battery_0_voltage_v':[12.3,12.9]}

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--trace',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--status-timeout',type=float,default=3);p.add_argument('--duration',type=float,default=25);args=p.parse_args()
    try:
        result=audit([parse(line) for line in args.trace.read_text().splitlines() if line],args.status_timeout,args.duration)
    except Exception as error:result={'schema':1,'result':'fail','reason':str(error)}
    result['trace_sha256']=hashlib.sha256(args.trace.read_bytes()).hexdigest()
    args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result));return int(result['result']!='pass')

if __name__=='__main__':raise SystemExit(main())
