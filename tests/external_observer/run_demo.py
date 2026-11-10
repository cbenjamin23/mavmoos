#!/usr/bin/env python3
"""Actual external DSO + ordinary mission + independent fake-peer acceptance.

This runner never launches ArduPilot, a vehicle or a control provider. The
documented human launcher is the path under test. It does not borrow a native
test window. Positive and expected-invalid app launches are each bounded.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import socket
import subprocess
import sys
import time

SOURCE = Path(__file__).resolve().parents[2]
EXTRA_SETTINGS = {
    'unknown_key':'plugin.example_heartbeat_observer.typo=1',
    'duplicate_key':'plugin.example_heartbeat_observer.sample_timeout=1',
    'missing_owner':'plugin.unloaded_observer.sample_timeout=1',
    'reserved_policy':'plugin.example_heartbeat_observer.simulation_only=true',
    'noncanonical':'plugin.Example_heartbeat_observer.sample_timeout=1'}


def load(path,name):
    spec=importlib.util.spec_from_file_location(name,path)
    if spec is None or spec.loader is None:raise ValueError('selected-source helper unavailable')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def dump(path,value):Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def revision(path):
    """Return only this checkout's own HEAD, never an enclosing repository's."""
    path=Path(path).resolve()
    if not (path/'.git').exists():return None
    env={k:v for k,v in os.environ.items() if k not in {'GIT_DIR','GIT_WORK_TREE','GIT_COMMON_DIR','GIT_INDEX_FILE'}}
    command=['git','-C',str(path),'rev-parse']
    root=subprocess.check_output(command+['--show-toplevel'],text=True,env=env,timeout=3).strip()
    if Path(root).resolve()!=path:raise ValueError('Git checkout does not own selected source: '+str(path))
    value=subprocess.check_output(command+['--verify','HEAD'],text=True,env=env,timeout=3).strip()
    if not re.fullmatch(r'[0-9a-f]{40}|[0-9a-f]{64}',value):raise ValueError('invalid selected Git HEAD: '+str(path))
    return value


def revisions(source,host_source,moos):
    return {key:revision(path) for key,path in (('source_ref',source),('host_source_ref',host_source),('moos_ref',moos))}


def cache(path,key):
    rows=[line.split('=',1)[1] for line in Path(path).read_text().splitlines()
          if re.match(re.escape(key)+r':[A-Z_]+=',line)]
    if len(rows)!=1:raise ValueError('CMake identity missing/ambiguous: '+key)
    return rows[0]


def free_port(kind):
    with socket.socket(socket.AF_INET,kind) as stream:
        stream.bind(('127.0.0.1',0));return stream.getsockname()[1]


def copy_sdk_inputs(original,destination,expected):
    raw=original.read_bytes()
    if hashlib.sha256(raw).hexdigest()!=expected:
        raise ValueError('configured SDK input bytes changed before copy')
    with destination.open('xb') as stream:stream.write(raw)
    if sha(destination)!=expected:raise ValueError('copied SDK input bytes differ')
    return {'file':destination.name,'sha256':expected,'original_path':str(original)}


def preflight(args):
    source=Path(args.source_root).resolve() if args.source_root else SOURCE
    if source/'tests/external_observer/run_demo.py'!=Path(__file__).resolve():
        raise ValueError('run the fixture from its explicitly selected source')
    build,module,moos,probe=[Path(getattr(args,k)).expanduser().resolve() for k in ('build','module','moos_root','probe')]
    host_source=Path(cache(build/'CMakeCache.txt','CMAKE_HOME_DIRECTORY')).resolve()
    inputs=module.parent.parent/'example-inputs.json'
    if not inputs.is_file():raise ValueError('independently configured example input record missing')
    input_bytes=inputs.read_bytes()
    declared=json.loads(input_bytes)
    module_cache=module.parent.parent/'CMakeCache.txt'
    module_source=Path(cache(module_cache,'CMAKE_HOME_DIRECTORY')).resolve()
    module_kind='root_integrated' if module_source==host_source else 'standalone_consumer'
    if module_kind=='standalone_consumer' and cache(module_cache,'CMAKE_PROJECT_NAME')!='MAVMOOSExternalObserverExample':
        raise ValueError('module configuration is not the matching standalone example build')
    for key,path in (('api_sha256',host_source/'src/libmavmoos_runtime/Plugin.h'),
                     ('loader_sha256',host_source/'src/libmavmoos_runtime/PluginHost.cpp')):
        if declared.get(key)!=sha(path):raise ValueError('independent example/host SDK input hashes differ')
    match=re.search(r'constexpr unsigned plugin_abi\s*=\s*([0-9]+)',(host_source/'src/libmavmoos_runtime/Plugin.h').read_text())
    if not match or int(match[1])<=0:raise ValueError('configured SDK has no positive development ABI')
    artifacts=[build/'bin/iMAVLink',module,probe,moos/'bin/MOOSDB',moos/'bin/pAntler',moos/'bin/nsplug',inputs,
               build/'CMakeCache.txt',module_cache]
    if args.scenario=='wrong_abi':
        if not args.wrong_abi:raise ValueError('wrong_abi requires independently built --wrong-abi module')
        artifacts.append(Path(args.wrong_abi).resolve())
    for path in artifacts:
        if not path.is_file() or path.is_symlink():raise ValueError('required regular artifact unavailable: '+str(path))
    if Path(cache(build/'CMakeCache.txt','MOOS_IVP_ROOT')).resolve()!=moos:
        raise ValueError('MOOS checkout differs from configured host dependency')
    for expected,path in ((args.expected_app_sha256,artifacts[0]),(args.expected_module_sha256,module)):
        if expected and (not re.fullmatch('[0-9a-f]{64}',expected) or sha(path)!=expected):
            raise ValueError('expected artifact hash differs')
    return source,host_source,build,module,moos,probe,artifacts,{
        'module_build_kind':module_kind,'module_cmake_source':str(module_source),'plugin_abi':int(match[1]),
        'module_cmake_cache':str(module_cache),'module_configured_inputs':declared,
        'module_inputs_path':str(inputs),'module_inputs_sha256':hashlib.sha256(input_bytes).hexdigest(),
        **revisions(source,host_source,moos),
        'build_identity_scope':'observed configured build inputs; actual standalone compilation/CTest proof is retained separately'}


def run(args):
    source,host_source,build,module,moos,probe,artifacts,module_metadata=preflight(args)
    ownership=load(source/'tests/owned_processes.py','_external_demo_owned')
    audit=load(source/'tests/external_observer/evidence.py','_external_demo_audit')
    out=Path(args.output).resolve();out.mkdir(parents=True,exist_ok=False)
    sdk_copy=out/'sdk-inputs.json'
    copy_receipt=copy_sdk_inputs(Path(module_metadata['module_inputs_path']),sdk_copy,module_metadata['module_inputs_sha256'])
    selected=out/'plugins';selected.mkdir()
    chosen=Path(args.wrong_abi).resolve() if args.scenario=='wrong_abi' else module
    selected_module=selected/('mavmoos_example_heartbeat_observer'+module.suffix)
    shutil.copyfile(chosen,selected_module)
    ports={'moos':free_port(socket.SOCK_STREAM),'app':free_port(socket.SOCK_DGRAM),'peer':free_port(socket.SOCK_DGRAM)}
    while len(set(ports.values()))!=3:
        ports['peer']=free_port(socket.SOCK_DGRAM)
    protected=artifacts+[source/'tests/owned_processes.py',host_source/'src/libmavmoos_runtime/Plugin.h',host_source/'src/libmavmoos_runtime/PluginHost.cpp',selected_module,sdk_copy]
    protected += sorted(p for folder in ('missions/external_observer','tests/external_observer')
                        for p in (source/folder).glob('*') if p.is_file())
    before={str(p):sha(p) for p in protected}
    if before[copy_receipt['original_path']]!=copy_receipt['sha256'] or before[str(sdk_copy)]!=copy_receipt['sha256']:
        raise ValueError('configured SDK input copy not bound to prelaunch inputs')
    result={'schema':1,'result':'FAIL','scenario':args.scenario,'source_root':str(source),
            'host_source_root':str(host_source),'host_build':str(build),'moos_root':str(moos),
            'scope':'real MOOS/externally loaded example DSO/local fake-peer only; module_build_kind distinguishes standalone consumer from routine root integration',
            'hardware':False,'simulator_used':False,'runner_pid':os.getpid(),'ports':ports,
            'app_sha256':sha(build/'bin/iMAVLink'),'module_sha256':sha(chosen),'normal_module_sha256':sha(module),
            'probe_sha256':sha(probe),'inputs_before':before,'cleanup_proven':False,'owned_processes_remaining':None}
    result.update(module_metadata)
    result['module_inputs_copy']=copy_receipt
    refs=('source_ref','host_source_ref','moos_ref')
    result['revision_identity_scope']={key:('own_git_head' if result[key] is not None else 'snapshot_hash_only') for key in refs}
    processes,logs=[],[]
    try:
        invalid=args.scenario in audit.INVALID_SCENARIOS
        duration=8 if invalid else 12
        command=[str(source/'missions/external_observer/launch.sh'),'--build='+str(build),'--module='+str(selected_module),
                 '--moos-root='+str(moos),'--work-dir='+str(out/'mission'),'--mport='+str(ports['moos']),
                 '--bind_port='+str(ports['app']),'--peer_port='+str(ports['peer']),'--duration='+str(duration),'--auto','--nogui']
        if args.scenario=='defaults':command.append('--defaults')
        if args.scenario=='invalid_config':command.append('--sample_timeout=nan')
        if args.scenario in EXTRA_SETTINGS:command.append('--setting='+EXTRA_SETTINGS[args.scenario])
        result['launcher_command']=command
        env=dict(os.environ);env['PATH']=os.pathsep.join([str(moos/'bin'),str(build/'bin'),env.get('PATH','')])
        probe_log=(out/'probe.jsonl').open('x');logs.append(probe_log)
        subscriber=subprocess.Popen([str(probe),str(ports['moos']),str(duration+4)],stdout=probe_log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
        processes.append(subscriber)
        launch_log=(out/'launcher.log').open('x');logs.append(launch_log)
        launcher=subprocess.Popen(command,stdout=launch_log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
        processes.append(launcher)
        result['processes']=[{'pid':p.pid,'command':p.args} for p in processes]
        began=time.monotonic();checked_unbound=False
        while time.monotonic()-began<27:
            if invalid and not checked_unbound and time.monotonic()-began>3:
                with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as absent:
                    absent.bind(('127.0.0.1',ports['app']))
                checked_unbound=True;result['invalid_app_udp_port_unbound_mono']=time.monotonic()
            if launcher.poll() is not None and subscriber.poll() is not None:break
            time.sleep(.02)
        if launcher.poll() is None or subscriber.poll() is None:raise RuntimeError('original27s demonstration deadline exceeded')
        result['launcher_exit']=launcher.returncode;result['probe_exit']=subscriber.returncode
        if launcher.returncode or subscriber.returncode:raise RuntimeError('documented launcher or subscriber failed')
        for stream in logs:stream.flush()
        receipts=list((out/'mission/logs').glob('run-*/result.json'))
        if len(receipts)!=1:raise RuntimeError('unique mission supervisor receipt missing')
        supervisor=json.loads(receipts[0].read_text())
        result['supervisor_file']=str(receipts[0].relative_to(out));result['supervisor_sha256']=sha(receipts[0])
        events=audit.observations((out/'probe.jsonl').read_text())
        result['acceptance']=audit.audit(events,supervisor,scenario=args.scenario,expected_abi=module_metadata['plugin_abi'])
        if invalid and not checked_unbound:
            raise RuntimeError('invalid startup UDP absence not directly witnessed')
        peer=receipts[0].parent
        if (peer/'primary-egress.bin').stat().st_size!=0:raise RuntimeError('independent primary egress byte capture is not empty')
        peer_events=[json.loads(line) for line in (peer/'peer-events.jsonl').read_text().splitlines()]
        if not {'initial','foreign_target','wrong_peer','recovered'} <= {v.get('phase') for v in peer_events}:
            raise RuntimeError('actual source/foreign-peer/recovery injection trace incomplete')
        result['independent_primary_egress_bytes']=0
        result['result']='PASS'
    except BaseException as error:
        result['error']=f'{type(error).__name__}: {error}'
    finally:
        errors,remaining=[],[]
        for process in reversed(processes):
            try:
                answer=ownership.stop_process_tree(process)
                if not isinstance(answer,list):raise RuntimeError('cleanup inventory unknown')
                remaining.extend(answer)
            except BaseException as error:errors.append(f'{type(error).__name__}: {error}')
        for stream in logs:stream.close()
        result['cleanup_errors']=errors;result['owned_processes_remaining']=None if errors else remaining
        result['cleanup_proven']=not errors and not remaining
        try:
            result['inputs_after']={p:sha(p) for p in before}
            result['input_drift']=[p for p,v in before.items() if result['inputs_after'][p]!=v]
            after_refs=revisions(source,host_source,moos)
            for key,value in after_refs.items():
                result[key+'_after']=value
                if value!=result[key]:result['input_drift'].append('Git revision changed: '+key)
        except (ValueError,OSError,subprocess.SubprocessError) as error:
            result.setdefault('input_drift',[]).append('postflight identity unavailable: '+str(error))
        if not result['cleanup_proven'] or result['input_drift']:result['result']='FAIL'
        result['files']={str(p.relative_to(out)):sha(p) for p in out.rglob('*') if p.is_file() and p!=out/'result.json'}
        dump(out/'result.json',result)
    print(json.dumps({'result':result['result'],'scenario':args.scenario,'receipt':str(out/'result.json'),'error':result.get('error')}))
    return 0 if result['result']=='PASS' else 1


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('build','module','moos-root','probe','output'):parser.add_argument('--'+name,required=True)
    parser.add_argument('--source-root');parser.add_argument('--wrong-abi')
    parser.add_argument('--expected-app-sha256');parser.add_argument('--expected-module-sha256')
    parser.add_argument('--scenario',choices=['normal','defaults','invalid_config','wrong_abi',*EXTRA_SETTINGS],default='normal')
    return run(parser.parse_args(argv))


if __name__=='__main__':
    try:raise SystemExit(main())
    except (ValueError,OSError,subprocess.SubprocessError) as error:
        print('External observer demo: '+str(error),file=sys.stderr);raise SystemExit(1)
