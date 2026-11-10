#!/usr/bin/env python3
"""Private normal/sanitized observer qualification; fake Services/I/O only."""
import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import zipfile


def sha(data):
    return hashlib.sha256(data).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime-source-root',type=Path,required=True)
    parser.add_argument('--build',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    repo=Path(__file__).resolve().parents[2];runtime=args.runtime_source_root.resolve()
    build,out=args.build.resolve(),args.output.resolve()
    out.mkdir(parents=True,exist_ok=False);build.mkdir(parents=True,exist_ok=False)
    headers=runtime/'build-rc-stream-host/_deps/mavlink-src'
    compiler='/Library/Developer/CommandLineTools/usr/bin/clang++'
    sdk='/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk'
    commands=[]
    component_names=['attitude','status']
    source_names=['src/plugins/AttitudePlugin.cpp','src/plugins/SystemStatusPlugin.cpp',
        'tests/attitude/AttitudeProviderTests.py','tests/attitude/AttitudeScenarioDriver.cpp','tests/attitude/CMakeLists.txt',
        'tests/status/StatusProviderTests.py','tests/status/StatusScenarioDriver.cpp','tests/status/CMakeLists.txt',
        'tests/observer_hardening/ObserverRuntimeClockTests.cpp','tests/observer_hardening/CMakeLists.txt',
        'tests/observer_hardening/run_validation.py','tests/observer_hardening/verify.py']
    runtime_paths=list((runtime/'src/libmavmoos_runtime').glob('*.cpp'))+list((runtime/'src/libmavmoos_runtime').glob('*.h'))
    inputs={str(Path('component')/name):(repo/name).read_bytes() for name in source_names}
    inputs.update({str(Path('runtime')/p.relative_to(runtime)):p.read_bytes() for p in runtime_paths})
    source_hashes={name:sha(data) for name,data in inputs.items()}
    header_hashes={str(p.relative_to(headers)):sha(p.read_bytes()) for p in sorted(headers.rglob('*.h'))}
    mavlink_archive=runtime/'build-ingress/mavlink-a2d0ecf.tar.gz'
    if sha(mavlink_archive.read_bytes())!='3fc41a3cd46717b1aa226b958aed0829fbe7b7c9793bc8ffdcca76345a9dfde3':raise RuntimeError('MAVLink archive pin mismatch')
    with tarfile.open(mavlink_archive) as archive:
        expected_headers={p.name.split('/',1)[1]:sha(archive.extractfile(p).read()) for p in archive if p.isfile() and p.name.endswith('.h')}
    if expected_headers!=header_hashes:raise RuntimeError('compiled generated headers differ from pinned archive')
    (out/'generated-headers.json').write_text(json.dumps(header_hashes,indent=2)+'\n')
    old_paths={name:runtime/('build-rc-stream-host/plugins/mavmoos_'+('attitude' if name=='attitude' else 'system_status')+'.so') for name in component_names}
    old_hashes={name:sha(path.read_bytes()) for name,path in old_paths.items()}
    frozen=json.loads((runtime/'docs/implementation/2026-11-04-rc-and-stream-requests/integration/freeze-and-upstream.json').read_text())
    for name,path in old_paths.items():
        if old_hashes[name]!=frozen['normal_artifacts'][str(path.relative_to(runtime))]:raise RuntimeError('old frozen MODULE drift')
    def run(argv,log,env=None,expected=0):
        destination=out/log;destination.parent.mkdir(parents=True,exist_ok=True)
        settings=os.environ.copy();settings.update(env or {})
        result=subprocess.run([str(x) for x in argv],cwd=repo,env=settings,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        destination.write_bytes(result.stdout)
        commands.append({'argv':[str(x) for x in argv],'cwd':str(repo),'environment_overrides':env or {},'expected_exit':expected,'exit':result.returncode,'log':log,'log_sha256':sha(result.stdout)})
        if result.returncode!=expected:raise RuntimeError('command outcome differs: '+log)
        return result.stdout.decode()
    report={'schema':1,'result':'RUNNING','component_ref':subprocess.check_output(['git','-C',str(repo),'rev-parse','HEAD'],text=True).strip(),
        'runtime_ref':subprocess.check_output(['git','-C',str(runtime),'rev-parse','HEAD'],text=True).strip(),
        'scope':'Actual MODULE and Runtime/fake Services/I/O; no sockets, native simulator, field or upstream changes',
        'inputs':source_hashes,'old_module_sha256':old_hashes,'commands':commands,'modes':{},
        'generated_headers':{'files':len(header_hashes),'manifest_sha256':sha((out/'generated-headers.json').read_bytes()),'pin':'a2d0ecf011a4aaa7840480e43f4e38d8c9e79d29','all_header_bytes_match_archive':True}}
    try:
        with zipfile.ZipFile(out/'tested-sources.zip','x',zipfile.ZIP_DEFLATED,compresslevel=9) as archive:
            for name,data in sorted(inputs.items()):archive.writestr(name,data)
            for name in component_names:
                filename='AttitudePlugin.cpp' if name=='attitude' else 'SystemStatusPlugin.cpp'
                old=subprocess.check_output(['git','-C',str(repo),'show','5cd81b7:src/plugins/'+filename])
                archive.writestr('old-component/src/plugins/'+filename,old)
        report['tested_sources_sha256']=sha((out/'tested-sources.zip').read_bytes())
        for mode in ['normal','sanitized']:
            env={} if mode=='normal' else {'ASAN_OPTIONS':'detect_leaks=0:halt_on_error=1','UBSAN_OPTIONS':'halt_on_error=1:print_stacktrace=1'}
            flags=[] if mode=='normal' else ['-DCMAKE_CXX_FLAGS=-fsanitize=address,undefined -fno-omit-frame-pointer','-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined','-DCMAKE_MODULE_LINKER_FLAGS=-fsanitize=address,undefined']
            built={};mode_info={'semantic_tests':{},'traces':{},'artifacts':{}}
            for name in component_names:
                folder=build/(mode+'-'+name);folder.mkdir()
                run(['cmake','-G','Ninja','-S',repo/'tests'/name,'-B',folder,'-DCMAKE_BUILD_TYPE=Debug','-DMAVLINK_DIR='+str(headers),'-DCMAKE_CXX_COMPILER='+compiler,'-DCMAKE_OSX_SYSROOT='+sdk,*flags],mode+'/'+name+'-configure.log',env)
                run(['cmake','--build',folder,'--parallel','4'],mode+'/'+name+'-build.log',env)
                targets=json.loads((folder/(name+'-targets.json')).read_text());built[name]=targets
                trace=folder/'trace.jsonl';title='Attitude' if name=='attitude' else 'Status'
                output=run(['python3','-B',repo/'tests'/name/(title+'ProviderTests.py'),'--driver',targets['driver'],'--module',targets['module'],'--trace',trace],mode+'/'+name+'-tests.log',env)
                count=45 if name=='attitude' else 43
                if ('Ran '+str(count)+' tests') not in output or not output.rstrip().endswith('OK'):raise RuntimeError('component test count/terminal mismatch')
                run(['ctest','--test-dir',folder,'--output-on-failure'],mode+'/'+name+'-ctest.log',env)
                raw=trace.read_bytes();compressed=gzip.compress(raw,mtime=0);rel=mode+'/'+name+'-trace.jsonl.gz';(out/rel).write_bytes(compressed)
                mode_info['traces'][name]={'file':rel,'sha256':sha(compressed),'raw_sha256':sha(raw)}
                mode_info['semantic_tests'][name]=count
                for role,path in targets.items():mode_info['artifacts'][name+'/'+role]={'path':path,'sha256':sha(Path(path).read_bytes())}
                if mode=='normal':
                    old_trace=folder/'old-module-trace.jsonl'
                    failed=run(['python3','-B',repo/'tests'/name/(title+'ProviderTests.py'),'--driver',targets['driver'],'--module',old_paths[name],'--trace',old_trace],'negative/'+name+'-old-module-tests.log',env,1)
                    if 'FAILED (failures=3)' not in failed:raise RuntimeError('expected old-module regression count differs')
                    raw=old_trace.read_bytes();compressed=gzip.compress(raw,mtime=0);rel='negative/'+name+'-old-module-trace.jsonl.gz';(out/rel).write_bytes(compressed)
                    mode_info['traces']['old_'+name]={'file':rel,'sha256':sha(compressed),'raw_sha256':sha(raw)}
            folder=build/(mode+'-host');folder.mkdir()
            rc=runtime/('build-rc-stream-host' if mode=='normal' else 'build-rc-stream-asan')/'plugins/mavmoos_rc_io.so'
            run(['cmake','-G','Ninja','-S',repo/'tests/observer_hardening','-B',folder,'-DCMAKE_BUILD_TYPE=Debug','-DMAVLINK_DIR='+str(headers),'-DRUNTIME_SOURCE_ROOT='+str(runtime),'-DCMAKE_CXX_COMPILER='+compiler,'-DCMAKE_OSX_SYSROOT='+sdk,
                '-DATTITUDE_MODULE='+built['attitude']['module'],'-DSTATUS_MODULE='+built['status']['module'],'-DRC_MODULE='+str(rc),*flags],mode+'/host-configure.log',env)
            run(['cmake','--build',folder,'--parallel','4'],mode+'/host-build.log',env)
            driver=folder/'observer_runtime_clock_tests'
            host=run([driver,built['attitude']['module'],built['status']['module'],rc],mode+'/host-trace.log',env)
            if not host.rstrip().endswith('PASS 6 actual Runtime observer-clock groups; fake I/O only'):raise RuntimeError('host groups incomplete')
            run(['ctest','--test-dir',folder,'--output-on-failure'],mode+'/host-ctest.log',env)
            mode_info['runtime_groups']=6;mode_info['artifacts']['host/driver']={'path':str(driver),'sha256':sha(driver.read_bytes())};mode_info['artifacts']['host/rc_module']={'path':str(rc),'sha256':sha(rc.read_bytes())}
            report['modes'][mode]=mode_info
        for name,data in inputs.items():
            path=(repo if name.startswith('component/') else runtime)/name.split('/',1)[1]
            if path.read_bytes()!=data:raise RuntimeError('source changed during qualification: '+name)
        if any(sha(path.read_bytes())!=old_hashes[name] for name,path in old_paths.items()):raise RuntimeError('old frozen MODULE changed')
        report['old_modules_reverified_unchanged']=True;report['result']='PASS'
    except BaseException as error:
        report['result']='FAIL';report['error']=str(error);raise
    finally:
        report['files']={str(p.relative_to(out)):{'bytes':p.stat().st_size,'sha256':sha(p.read_bytes())} for p in sorted(out.rglob('*')) if p.is_file() and p.name!='qualification.json'}
        (out/'qualification.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'result':report['result'],'modes':{name:{'semantic_tests':value['semantic_tests'],'runtime_groups':value['runtime_groups']} for name,value in report['modes'].items()},'old_module_failures_each':3},indent=2))


if __name__=='__main__':main()
