#!/usr/bin/env python3
"""Offline integrity and reset-boundary replay of the private observer archive."""
import argparse
import collections
import copy
import gzip
import hashlib
import json
import math
from pathlib import Path
import re
import zipfile


def require(value,reason):
    if not value:raise ValueError(reason)


def digest(data):return hashlib.sha256(data).hexdigest()


def parse(data):
    def pairs(rows):
        result={}
        for key,value in rows:
            require(key not in result,'duplicate JSON key');result[key]=value
        return result
    def bad(value):raise ValueError('non-JSON numeric constant '+value)
    result=json.loads(data,object_pairs_hook=pairs,parse_constant=bad)
    def finite(value):
        if isinstance(value,float):require(math.isfinite(value),'nonfinite float')
        elif isinstance(value,dict):
            for child in value.values():finite(child)
        elif isinstance(value,list):
            for child in value:finite(child)
    finite(result);return result


def semantic(rows,name):
    by_case=collections.defaultdict(list)
    for row in rows:by_case[row['case']].append(row)
    def one(case,key):
        values=[v['value'] for v in by_case[case] if v['key']==key]
        require(len(values)==1,'missing/duplicate case publication: '+case+'/'+key);return values[0]
    state_key='MAVMOOS_ATTITUDE_STATUS' if name=='attitude' else 'MAVMOOS_SYSTEM_STATUS'
    sample_key='MAVMOOS_ATTITUDE_EULER' if name=='attitude' else 'MAVMOOS_SYS_STATUS'
    state=lambda case:one(case,state_key)
    sample=lambda case:one(case,sample_key)
    reliable=sample('hardening_reliable')
    require(reliable['present'] is True and reliable['fresh'] is True and reliable['effective_at']==101,'reliable seed absent')
    cases=['hardening_early_reset','hardening_input_before_highwater','hardening_catchup_without_reset']+['hardening_invalid_reset_'+str(i) for i in range(4)]
    for case in cases:
        require(state(case)['state']=='CLOCK_FAULT','bad reset/catchup unlatched: '+case)
        require(sample(case)['present'] is False and sample(case)['fresh'] is False and sample(case)['data'] is None,'bad reset retained data: '+case)
        require(sample(case)['effective_at'] is None and sample(case)['read_at'] is None,'bad reset retained stamps')
    require(state('hardening_early_reset')['reset_reason']=='behind_highwater','reset reason lost')
    for case in ['hardening_valid_reset','hardening_old_floor']:
        require(state(case)['state']=='WAITING_HEARTBEAT' and sample(case)['present'] is False,'reset floor rebound old source')
        require(state(case)['session_boundary']==102.1,'original valid reset floor changed')
    current=sample('hardening_recovered')
    require(state('hardening_recovered')['state']=='BOUND' and current['fresh'] is True and current['present'] is True,'valid explicit reset did not recover')
    require(current['effective_at']==current['read_at']==102.45,'new sample original stamps changed')
    require(current['observation_id']>reliable['observation_id'] and current['session_epoch']>reliable['session_epoch'],'new sample identity/epoch failed')
    require(state('hardening_equal_reset')['state']=='BOUND' and state('hardening_equal_reset')['session_boundary']==101 and sample('hardening_equal_reset')['fresh'] is True,'equal high-water reset rejected')
    require(one('driver_complete','summary')=={'effects':0,'abi':7},'observer effect/ABI claim changed')
    return {'hardening_groups':5,'invalid_reset_values':4,'effect_calls':0}


def host_semantic(text):
    names=['rejected checkpoint cannot rewind owner time or retain observations',
        'rejected sampled clock clears caches at retained good time',
        'final handoff predicate cannot pass a regressing or nonfinite clock',
        'explicit lower disconnect boundary retains owner high-water',
        'host fault reason does not prevent valid new heartbeat recovery',
        'retained reset floor rejects old acquisitions despite later read']
    require(text.rstrip().endswith('PASS 6 actual Runtime observer-clock groups; fake I/O only'),'host terminal missing')
    for name in names:require(text.splitlines().count('PASS '+name)==1,'host group missing/duplicate')
    rows=[parse(line) for line in text.splitlines() if line.startswith('{')]
    counts=collections.Counter(row['case'] for row in rows)
    require(counts=={'checkpoint_rejected':3,'sample_rejected':3,'final_predicate_rejected':3,'disconnect_retained':1,'new_session_observed':1,'old_floor_rejected':1},'host checkpoint matrix incomplete')
    for row in rows:
        require(row['writes']==0 and row['runtime_now']>=101,'host wrote/rewound')
        require('permission=false' in row['runtime_status'],'observer restored permission')
        require(set(row['observers'])=={'MAVMOOS_ATTITUDE_STATUS','MAVMOOS_SYSTEM_STATUS','MAVMOOS_RC_IO_STATUS'},'host observer set incomplete')
        for key,state in row['observers'].items():
            require(state['session_epoch']>1 and state['read_only'] is True and state['authoritative_for_control'] is False,'host reset epoch/effect boundary failed')
            require(state['state']==('BOUND' if row['case']=='new_session_observed' else 'WAITING_HEARTBEAT'),'numeric state versus host reason contract changed')
            expected='reset_floor' if row['case']=='old_floor_rejected' else 'explicit_test_reset' if row['case']=='disconnect_retained' else 'clock_fault'
            require(state['reset_reason']==expected,'host reason lost')
    return {'runtime_groups':6,'snapshots':len(rows),'writes':0}


def controls(rows,name):
    rejected=[]
    def attempt(label,change):
        altered=copy.deepcopy(rows);change(altered)
        try:semantic(altered,name)
        except (ValueError,KeyError,TypeError,IndexError):rejected.append(label)
        else:raise ValueError('negative control accepted: '+label)
    status_key='MAVMOOS_ATTITUDE_STATUS' if name=='attitude' else 'MAVMOOS_SYSTEM_STATUS'
    sample_key='MAVMOOS_ATTITUDE_EULER' if name=='attitude' else 'MAVMOOS_SYS_STATUS'
    def change(case,key,field,value):
        def apply(items):next(row['value'] for row in items if row['case']==case and row['key']==key)[field]=value
        return apply
    attempt('bad_reset_unlatched',change('hardening_early_reset',status_key,'state','BOUND'))
    attempt('catchup_implicitly_recovers',change('hardening_catchup_without_reset',status_key,'state','BOUND'))
    attempt('cleared_sample_retained',change('hardening_invalid_reset_0',sample_key,'present',True))
    attempt('reset_floor_rewound',change('hardening_valid_reset',status_key,'session_boundary',100))
    attempt('new_stamp_renewed',change('hardening_recovered',sample_key,'effective_at',102.5))
    attempt('lost_source_epoch',change('hardening_recovered',sample_key,'session_epoch',1))
    attempt('effect_claim',change('driver_complete','summary','effects',1))
    attempt('missing_recovery',lambda items:items.__setitem__(slice(None),[r for r in items if r['case']!='hardening_recovered']))
    attempt('duplicate_state',lambda items:items.append(copy.deepcopy(next(r for r in items if r['case']=='hardening_early_reset' and r['key']==status_key))))
    return rejected


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('archive',type=Path);parser.add_argument('--current-component-root',type=Path);parser.add_argument('--current-runtime-root',type=Path);parser.add_argument('--negative-controls',action='store_true');args=parser.parse_args()
    base=args.archive;manifest=parse((base/'qualification.json').read_bytes())
    require(manifest['result']=='PASS' and manifest['old_modules_reverified_unchanged'] is True,'qualification not passed')
    for name,item in manifest['files'].items():
        p=base/name;require(p.stat().st_size==item['bytes'] and digest(p.read_bytes())==item['sha256'],'archive member changed: '+name)
    require(digest((base/'tested-sources.zip').read_bytes())==manifest['tested_sources_sha256'],'source ZIP changed')
    with zipfile.ZipFile(base/'tested-sources.zip') as source:
        require(source.testzip() is None,'source ZIP corrupt')
        for name,expected in manifest['inputs'].items():
            require(digest(source.read(name))==expected,'source snapshot changed: '+name)
            root=args.current_component_root if name.startswith('component/') else args.current_runtime_root
            if root:require(digest((root/name.split('/',1)[1]).read_bytes())==expected,'current source differs: '+name)
    require(digest((base/'generated-headers.json').read_bytes())==manifest['generated_headers']['manifest_sha256'],'header manifest changed')
    require(len(parse((base/'generated-headers.json').read_bytes()))==manifest['generated_headers']['files'],'header count changed')
    outcomes={}
    for mode in ['normal','sanitized']:
        info=manifest['modes'][mode];mode_results={}
        for name,count in [('attitude',45),('status',43)]:
            trace_info=info['traces'][name];packed=(base/trace_info['file']).read_bytes();raw=gzip.decompress(packed)
            require(digest(packed)==trace_info['sha256'] and digest(raw)==trace_info['raw_sha256'],'trace hash changed')
            rows=[parse(line) for line in raw.decode().splitlines()]
            mode_results[name]=semantic(rows,name)
            log=(base/mode/(name+'-tests.log')).read_text();require('Ran '+str(count)+' tests' in log and log.rstrip().endswith('OK'),'actual component test log failed')
            mode_results[name]['semantic_tests']=count
            if args.negative_controls:mode_results[name]['negative_controls_rejected']=controls(rows,name)
            if mode=='normal':
                old=gzip.decompress((base/info['traces']['old_'+name]['file']).read_bytes())
                try:semantic([parse(line) for line in old.decode().splitlines()],name)
                except ValueError:pass
                else:raise ValueError('old MODULE unexpectedly satisfies hardening contract')
                require('FAILED (failures=3)' in (base/'negative'/(name+'-old-module-tests.log')).read_text(),'old negative failure count changed')
        mode_results['host']=host_semantic((base/mode/'host-trace.log').read_text());outcomes[mode]=mode_results
    for command in manifest['commands']:
        require(command['exit']==command['expected_exit'] and digest((base/command['log']).read_bytes())==command['log_sha256'],'command record mismatch')
    print(json.dumps({'result':'PASS','modes':outcomes,'source_files':len(manifest['inputs']),'generated_headers':manifest['generated_headers']['files'],'scope':'Offline archive integrity and reset-boundary replay, not a new MODULE/native execution'},indent=2))


if __name__=='__main__':main()
