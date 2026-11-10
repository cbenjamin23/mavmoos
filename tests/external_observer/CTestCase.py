#!/usr/bin/env python3
"""Fresh retained output per actual loopback-app CTest; never a native case."""
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('scenario','build','module','moos-root','probe','output-parent'):
        parser.add_argument('--'+name,required=True)
    parser.add_argument('--source-root',default=str(Path(__file__).resolve().parents[2]))
    parser.add_argument('--wrong-abi')
    args=parser.parse_args(argv)
    source=Path(args.source_root).resolve()
    if Path(__file__).resolve()!=source/'tests/external_observer/CTestCase.py':
        raise ValueError('CTest adapter must come from selected source')
    parent=Path(args.output_parent).resolve();parent.mkdir(parents=True,exist_ok=True)
    case=Path(tempfile.mkdtemp(prefix=args.scenario+'-',dir=parent))
    output=case/'evidence'
    command=[sys.executable,'-B',str(source/'tests/external_observer/run_demo.py'),
             '--source-root',str(source),'--scenario',args.scenario,'--build',args.build,
             '--module',args.module,'--moos-root',args.moos_root,'--probe',args.probe,'--output',str(output)]
    if args.wrong_abi:command+=['--wrong-abi',args.wrong_abi]
    spec=importlib.util.spec_from_file_location('_external_ctest_owned',source/'tests/owned_processes.py')
    if spec is None or spec.loader is None:raise ValueError('selected-source process helper unavailable')
    ownership=importlib.util.module_from_spec(spec);spec.loader.exec_module(ownership)
    result={'schema':1,'result':'FAIL','scenario':args.scenario,'command':command,
            'evidence':str(output),'scope':'software/loopback MOOS app; no native simulator/vehicle',
            'cleanup_proven':False,'owned_processes_remaining':None}
    process=None
    with (case/'driver.log').open('x') as log:
        try:
            process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            result['driver_pid']=process.pid
            result['exit_code']=process.wait(timeout=42)
            report=json.loads((output/'result.json').read_text())
            if process.returncode==0 and report.get('result')=='PASS' and report.get('scenario')==args.scenario and report.get('cleanup_proven') is True:
                result['result']='PASS'
        except BaseException as error:
            result['error']=f'{type(error).__name__}: {error}'
        finally:
            if process:
                try:
                    remaining=ownership.stop_process_tree(process)
                    if not isinstance(remaining,list):raise RuntimeError('unknown cleanup inventory')
                    result['owned_processes_remaining']=remaining
                    result['cleanup_proven']=not remaining
                except BaseException as error:
                    result['cleanup_error']=f'{type(error).__name__}: {error}'
            if not result['cleanup_proven']:result['result']='FAIL'
    (case/'ctest-result.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'result':result['result'],'scenario':args.scenario,'receipt':str(case/'ctest-result.json')}))
    return 0 if result['result']=='PASS' else 1


if __name__=='__main__':
    try:raise SystemExit(main())
    except (ValueError,OSError,subprocess.SubprocessError) as error:
        print('External observer CTest adapter: '+str(error),file=sys.stderr);raise SystemExit(1)
