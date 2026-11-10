#!/usr/bin/env python3
"""Reproduce pure/owned-loopback handoff checks, never an ArduPilot run."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mavlink-dir',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[2]
    headers=args.mavlink_dir.resolve()
    if not (headers/'ardupilotmega/mavlink.h').is_file():
        parser.error('--mavlink-dir must contain the pinned generated dialect')
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    commands=[]
    report={'result':'FAIL','scope':'pure policy and owned localhost/inert process only; no native simulator, real vehicle, MOOS, or hardware',
            'platform':platform.platform(),'machine':platform.machine(),'mavlink_dir':str(headers),'profiles':{}}

    def run(argv,log,env=None):
        with log.open('w') as handle:
            completed=subprocess.run(argv,stdout=handle,stderr=subprocess.STDOUT,env=env,timeout=90)
        commands.append({'argv':argv,'exit':completed.returncode,'log':str(log.relative_to(output))})
        if completed.returncode:
            raise RuntimeError(str(log.relative_to(output))+' exited '+str(completed.returncode))

    try:
        for profile in ('normal','asan-ubsan'):
            directory=output/profile;directory.mkdir()
            build=directory/'build'
            command=['cmake','-G','Ninja','-S',str(root/'tests/send_boundary'),'-B',str(build),
                     '-DMAVLINK_DIR='+str(headers),'-DCMAKE_BUILD_TYPE=Debug']
            if platform.system()=='Darwin':
                command.extend(['-DCMAKE_CXX_COMPILER=/Library/Developer/CommandLineTools/usr/bin/clang++',
                                '-DCMAKE_OSX_SYSROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk'])
            if profile=='asan-ubsan':
                command.extend(['-DCMAKE_CXX_FLAGS=-fsanitize=address,undefined -fno-omit-frame-pointer',
                                '-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined'])
            run(command,directory/'configure.log')
            run(['cmake','--build',str(build),'--parallel','4'],directory/'build.log')
            environment=dict(os.environ)
            if profile=='asan-ubsan':
                environment.update(ASAN_OPTIONS='detect_leaks=0',UBSAN_OPTIONS='halt_on_error=1')
            run(['ctest','--test-dir',str(build),'--output-on-failure','-V'],directory/'tests.log',environment)
            artifacts=[build/'send_boundary_tests',build/'test-fixtures/copter/arducopter']
            report['profiles'][profile]={'result':'PASS','ctest':'2/2','artifacts':{
                str(path.relative_to(output)):hashlib.sha256(path.read_bytes()).hexdigest() for path in artifacts}}
        report['result']='PASS'
    except Exception as error:
        report['error']=str(error)
    finally:
        source=[root/'src/libmavmoos_rx'/name for name in ['SimulationSendGate.h','SimulationSendGate.cpp','UdpReceiver.h','UdpReceiver.cpp','Config.h','Config.cpp','Telemetry.h','Telemetry.cpp']]
        source.extend(root/'tests/send_boundary'/name for name in ['CMakeLists.txt','SendBoundaryTests.cpp','InertSocketOwner.cpp','validate_private.py'])
        report['sources']={str(path.relative_to(root)):hashlib.sha256(path.read_bytes()).hexdigest() for path in source}
        (output/'commands.json').write_text(json.dumps(commands,indent=2)+'\n')
        (output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    return 0 if report['result']=='PASS' else 1


if __name__=='__main__':
    raise SystemExit(main())
