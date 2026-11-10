#!/usr/bin/env python3
"""Offline component readback; never load a MODULE, start a process or open a socket."""
import argparse
import copy
import gzip
import hashlib
import json
import math
from pathlib import Path
import zipfile


def sha(data):return hashlib.sha256(data).hexdigest()
def require(value,why):
    if not value:raise ValueError(why)
def parse(text):
    def bad(value):raise ValueError('nonfinite JSON '+value)
    def finite(value):
        number=float(value)
        if not math.isfinite(number):raise ValueError('overflow JSON float '+value)
        return number
    def unique(pairs):
        result={}
        for key,value in pairs:
            if key in result:raise ValueError('duplicate JSON key '+key)
            result[key]=value
        return result
    return json.loads(text,parse_constant=bad,parse_float=finite,object_pairs_hook=unique)


def trace(rows):
    require(rows and rows[-1]=={'case':'driver_complete','key':'summary','value':{'effects':0,'abi':7,'random_frames':2048}},'missing driver/zero-effect summary')
    ids={};cases=set()
    for row in rows[:-1]:
        cases.add(row['case']);key,value=row['key'],row['value']
        require(key in ('MAVMOOS_DISTANCE_SENSORS','MAVMOOS_DISTANCE_STATUS'),'unexpected publication')
        if key=='MAVMOOS_DISTANCE_STATUS':
            require(value['read_only'] is True and value['authoritative_for_control'] is False,'observer effect/authority claim')
            a=value['sensors']
        else:a=value
        require(a['schema']==1 and a['atomic'] is False and a['cache_capacity']==8,'aggregate contract')
        sensors=a['sensors'];require(len(sensors)<=8 and a['present'] is bool(sensors),'aggregate bound/presence')
        numbers=[r['data']['id'] for r in sensors]
        require(numbers==sorted(set(numbers)),'duplicate/unsorted sensor IDs')
        for record in sensors:
            require(record['session_epoch']==a['session_epoch'] and record['family']=='DISTANCE_SENSOR','session/family')
            require(record['source_age']=='unknown' and record['source_quality']=='unassessed','invented source freshness')
            effective,read,age=record['effective_at'],record['read_at'],record['age_s']
            require(effective>=0 and read>=effective and age>=0 and age+1e-8>=read-effective,'impossible acquisition age')
            d=record['data'];require(type(d['id']) is int and 0<=d['id']<=255,'ID shape')
            require(d['timestamp_semantics']=='ardupilot_message_send_time_not_certified_measurement_time','measurement timestamp claim')
            require(d['distance_interpretation']=='sensor_report_no_vertical_or_terrain_projection' and d['physical_accuracy']=='unassessed','invented projection/accuracy')
            for name in ('min_distance','max_distance','current_distance'):
                raw=d[name+'_cm'];q=d[name+'_m']
                require(type(raw) is int and 0<=raw<=65535 and q['known'] is True and q['unit']=='m' and abs(q['value']-raw/100)<1e-10,'wrong raw/metre quantity')
            consistent=d['min_distance_cm']<=d['max_distance_cm']
            require(d['bounds_consistent'] is consistent and d['current_within_reported_limits'] is (consistent and d['min_distance_cm']<=d['current_distance_cm']<=d['max_distance_cm']),'reported limits contradiction')
            covariance=d['covariance'];raw=covariance['raw_cm2'];q=covariance['quantity'];known=raw not in (0,255)
            require(q['known'] is known and q['unit']=='m^2' and (abs(q['value']-raw/10000)<1e-10 if known else q['value'] is None),'wrong covariance unknown/units')
            quality=d['signal_quality'];raw=quality['raw'];q=quality['quantity'];known=d['wire_version']==2 and raw is not None and 2<=raw<=100
            require(q['known'] is known and q['unit']=='percent' and q['value']==(raw if known else None),'wrong signal quality unknown/units')
            if d['wire_version']==1:
                require(raw is None and d['quaternion']['raw_wxyz']==[None]*4 and not d['extension_bytes_observed'],'fabricated MAVLink1 extension')
            signature=json.dumps([d,effective,read,record['age_basis']],sort_keys=True)
            identity=(row['case'],record['session_epoch'],record['observation_id'])
            require(identity not in ids or ids[identity]==signature,'same cached ID changed data/stamps')
            ids[identity]=signature
    require({'normal','cached','coalesced','premature_reset','clock_recovered','capacity','random_frames','invalid_start_recovered'}<=cases,'required boundary cases absent')
    return {'records':len(rows),'cases':len(cases),'zero_effect_summary':True}


def controls(rows):
    rejected=[]
    def reject(name,mutate):
        changed=copy.deepcopy(rows);mutate(changed)
        try:trace(changed)
        except (ValueError,KeyError,TypeError):rejected.append(name)
        else:raise ValueError('negative control accepted: '+name)
    def normal(rows):return next(r['value']['sensors'][0] for r in rows if r['case']=='normal' and r['key']=='MAVMOOS_DISTANCE_SENSORS')
    def normal_a(rows):return next(r['value'] for r in rows if r['case']=='normal' and r['key']=='MAVMOOS_DISTANCE_SENSORS')
    reject('missing_summary',lambda r:r.pop())
    reject('effect_claim',lambda r:r[-1]['value'].__setitem__('effects',1))
    reject('metre_scale',lambda r:normal(r)['data']['current_distance_m'].__setitem__('value',432))
    reject('false_age',lambda r:normal(r).__setitem__('age_s',0))
    reject('invented_source_age',lambda r:normal(r).__setitem__('source_age','synchronized'))
    reject('sensor_capacity',lambda r:normal_a(r)['sensors'].extend([normal(r)]*8))
    reject('duplicate_sensor',lambda r:normal_a(r)['sensors'].append(copy.deepcopy(normal(r))))
    reject('mixed_session',lambda r:normal(r).__setitem__('session_epoch',999))
    reject('terrain_projection',lambda r:normal(r)['data'].__setitem__('distance_interpretation','height_above_home'))
    reject('covariance_units',lambda r:normal(r)['data']['covariance']['quantity'].__setitem__('value',25))
    reject('quality_percentage',lambda r:normal(r)['data']['signal_quality']['quantity'].__setitem__('value',100))
    reject('wrong_message_time',lambda r:normal(r)['data'].__setitem__('timestamp_semantics','certified_measurement_time'))
    return rejected


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive',type=Path)
    parser.add_argument('--current-source-root',type=Path)
    parser.add_argument('--negative-controls',action='store_true')
    args=parser.parse_args();root=args.archive;q=parse((root/'qualification.json').read_text())
    require(q['result']=='PASS' and q['abi']==7,'qualification failed/ABI')
    for name,expected in q['archive_files'].items():
        p=root/name;require(p.stat().st_size==expected['bytes'] and sha(p.read_bytes())==expected['sha256'],'archive member '+name)
    for filename,expected in [('tested-sources.zip',q['source_files']),('primary-sources.zip',{v['snapshot_member']:v['sha256'] for v in parse((root/'primary-sources.json').read_text())['files']})]:
        with zipfile.ZipFile(root/filename) as source:
            require(set(source.namelist())==set(expected) and len(source.namelist())==len(expected),'source ZIP members')
            for name,digest in expected.items():require(sha(source.read(name))==digest,'source ZIP bytes '+name)
    if args.current_source_root:
        for name,expected in q['source_files'].items():require(sha((args.current_source_root/name).read_bytes())==expected,'current source '+name)
    traces={}
    for mode in ('normal','sanitized'):
        raw=gzip.decompress((root/mode/'trace.jsonl.gz').read_bytes())
        require(sha(raw)==q[mode]['trace_sha256'],'trace hash')
        rows=[parse(line) for line in raw.decode().splitlines()];traces[mode]=rows
        require(q[mode]['semantic_tests']==33 and q[mode]['passed']==33,'semantic totals')
        log=(root/mode/'test.log').read_text();require('Ran 33 tests' in log and log.strip().endswith('OK') and 'FAILED' not in log,'test result')
        require('100% tests passed out of 1' in (root/mode/'ctest.log').read_text(),'CTest result')
        trace(rows)
    require(traces['normal']==traces['sanitized'],'logical traces differ across sanitizer modes')
    mutations=parse((root/'mutations/results.json').read_text());require(mutations['result']=='PASS' and len(mutations['variants'])==3,'mutation evidence')
    for variant in mutations['variants']:
        name=variant['name'];require(variant['compile_exit']==0 and variant['test_exit']!=0,'bad alternative not rejected')
        require('FAILED' in (root/'mutations'/name/'test.log').read_text(),'missing mutation failure')
    result={'result':'PASS','modes':['normal','sanitized'],'semantic_groups_each':33,'source_files':len(q['source_files']),'trace':trace(traces['normal']),'defective_modules_rejected':3,'current_source_match':bool(args.current_source_root),'scope':'Offline archive/component software evidence only; no native, network or hardware action'}
    if args.negative_controls:result['negative_controls_rejected']=controls(traces['normal'])
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
