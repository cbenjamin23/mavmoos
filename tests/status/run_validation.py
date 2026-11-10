#!/usr/bin/env python3
"""Build/load the status MODULE under fake Services; no network or FC process."""
import argparse
import datetime
import gzip
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
import zipfile

REPO=Path(__file__).resolve().parents[2]
INPUTS=['src/plugins/SystemStatusPlugin.cpp','src/libmavmoos_runtime/Plugin.h',
        'tests/status/CMakeLists.txt','tests/status/StatusScenarioDriver.cpp',
        'tests/status/StatusProviderTests.py','tests/status/run_validation.py',
        'tests/status/StatusMoosProbe.cpp','tests/status/StatusObservationAudit.py',
        'tests/status/StatusObservationAuditTests.py']
HEADERS=['common/mavlink_msg_sys_status.h','common/mavlink_msg_battery_status.h',
         'common/mavlink_msg_gps_raw_int.h','common/mavlink_msg_statustext.h','common/common.h']

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def git(*args):return subprocess.check_output(['git','-C',str(REPO),*args],text=True).strip()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mavlink-dir',type=Path,required=True)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--build-dir',type=Path,default=REPO/'build-status-private'/'validation')
    p.add_argument('--compiler',default=shutil.which('clang++'))
    p.add_argument('--sdk')
    p.add_argument('--moos-root',type=Path,help='Optional built stock checkout: compile subscriber, do not run it')
    args=p.parse_args()
    output=args.output_dir.resolve();build=args.build_dir.resolve();headers=args.mavlink_dir.resolve()
    if not args.compiler:p.error('clang++ not found; provide --compiler')
    if output.exists() and any(output.iterdir()):p.error('output directory must be empty to preserve prior evidence')
    output.mkdir(parents=True,exist_ok=True)
    manifest={'schema':1,'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'repo_commit':git('rev-parse','HEAD'),'worktree_status':git('status','--short'),
              'platform':platform.platform(),'scope':'real MODULE + fake Services only; no network/simulation/hardware',
              'python':sys.version,'compiler':str(args.compiler),'mavlink_dir':str(headers),
              'inputs':{name:sha(REPO/name) for name in INPUTS},
              'mavlink_headers':{name:sha(headers/name) for name in HEADERS},'runs':[]}
    # Fixed metadata makes this source snapshot independently reproducible.
    with zipfile.ZipFile(output/'tested-sources.zip','w',zipfile.ZIP_DEFLATED) as z:
        for name in INPUTS:
            info=zipfile.ZipInfo(name,(1980,1,1,0,0,0));info.external_attr=0o100644<<16
            z.writestr(info,(REPO/name).read_bytes(),compress_type=zipfile.ZIP_DEFLATED)
    manifest['source_snapshot_sha256']=sha(output/'tested-sources.zip')
    failure=None
    def run(command,log,env):
        r=subprocess.run([str(v) for v in command],cwd=REPO,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=180)
        log.write_text(r.stdout)
        manifest['runs'].append({'command':[str(v) for v in command],'log':str(log.relative_to(output)),
                                 'returncode':r.returncode,'log_sha256':sha(log)})
        if r.returncode:raise RuntimeError(f'{log.name}: exit {r.returncode}')
        return r.stdout
    try:
        checker=run([sys.executable,'-B',REPO/'tests/status/StatusObservationAuditTests.py'],output/'audit-checker-tests.log',os.environ.copy())
        count=re.search(r'Ran (\d+) tests',checker)
        manifest['audit_checker_test_groups']=int(count[1]) if count else None
        for mode in ('normal','sanitized'):
            dest=output/mode;dest.mkdir();target=build/mode
            env=os.environ.copy()
            command=['cmake','-G','Ninja','-S',REPO/'tests/status','-B',target,
                     '-DCMAKE_BUILD_TYPE=RelWithDebInfo',f'-DCMAKE_CXX_COMPILER={args.compiler}',f'-DMAVLINK_DIR={headers}']
            if args.sdk:command.append(f'-DCMAKE_OSX_SYSROOT={args.sdk}')
            if args.moos_root:command.append(f'-DMOOS_IVP_ROOT={args.moos_root.resolve()}')
            if mode=='sanitized':
                command+=['-DCMAKE_CXX_FLAGS=-fsanitize=address,undefined -fno-omit-frame-pointer',
                          '-DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=address,undefined',
                          '-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined']
                env.update(ASAN_OPTIONS='detect_leaks=0',UBSAN_OPTIONS='halt_on_error=1')
            run(command,dest/'configure.log',env)
            run(['cmake','--build',target,'--parallel','4'],dest/'build.log',env)
            targets=json.loads((target/'status-targets.json').read_text())
            trace=dest/'publications.jsonl'
            log=run([sys.executable,'-B',REPO/'tests/status/StatusProviderTests.py',
                     '--driver',targets['driver'],'--module',targets['module'],'--trace',trace],dest/'tests.log',env)
            count=re.search(r'Ran (\d+) tests',log)
            manifest[mode]={'test_groups':int(count[1]) if count else None,
                            'module_sha256':sha(Path(targets['module'])),'driver_sha256':sha(Path(targets['driver'])),
                            'trace_sha256':sha(trace),'result':'pass'}
            if args.moos_root:
                manifest[mode]['probe_status']='compiled only; no MOOS connection/native process launched'
                manifest[mode]['probe_sha256']=sha(target/'status_moos_probe')
            with gzip.GzipFile(filename=str(trace)+'.gz',mode='wb',mtime=0) as f:f.write(trace.read_bytes())
            manifest[mode]['compressed_trace_sha256']=sha(Path(str(trace)+'.gz'));trace.unlink()
    except Exception as exc:
        failure=str(exc)
    manifest['result']='fail' if failure else 'pass';manifest['failure']=failure
    (output/'validation.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps({'result':manifest['result'],'manifest':str(output/'validation.json'),'failure':failure}))
    return 1 if failure else 0

if __name__=='__main__':raise SystemExit(main())
