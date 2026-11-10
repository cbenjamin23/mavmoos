#!/usr/bin/env python3
"""Owned native disarmed protocol service bench. Never a relay or physical route."""
import argparse
import fcntl
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import tempfile
import time

PIN='04cf6486e8d42ce138de816e8466244346ed77cb'
NATIVE='fb60bf63c7cd49a37ccf80d7669c6377351affa666741f7067275100ba16f02a'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def safe(value):
    if isinstance(value,float) and not math.isfinite(value):return None
    if isinstance(value,bytes):return value.decode('ascii',errors='replace')
    if isinstance(value,dict):return {str(k):safe(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [safe(v) for v in value]
    return value
def port(kind):
    with socket.socket(socket.AF_INET,kind) as s:s.bind(('127.0.0.1',0));return s.getsockname()[1]
def ref(root):return subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD'],text=True).strip()
def inventory(process):
    if process.poll() is not None:raise RuntimeError('owned native process exited')
    p=subprocess.run(['lsof','-nP','-a','-p',str(process.pid),'-iUDP'],capture_output=True,text=True,timeout=3)
    if p.returncode:raise RuntimeError('owned native UDP inventory unavailable')
    return p.stdout
def peer_matches(text,source,destination):return bool(re.search(r'UDP\s+127\.0\.0\.1:'+str(source)+r'->127\.0\.0\.1:'+str(destination)+r'\b',text))


def observer_message(message,received_at,valid_count,startup_bytes):
    """Keep GCS_Common.cpp's three SiK sync pairs out of observation evidence."""
    if message.get_type()=='BAD_DATA':
        data=bytes(message.data)
        expected=b'0 '*3
        if valid_count or not data or startup_bytes+len(data)>len(expected) or data!=expected[startup_bytes:startup_bytes+len(data)] or message.reason!='Bad prefix':
            raise RuntimeError('bad independent native frame')
        return 'startup_diagnostic',{'received_mono':received_at,'raw_hex':data.hex(),'reason':message.reason,
                                     'scope':'pinned native SiK sync prefix before first MAVLink frame; not observation evidence'}
    if message.get_srcSystem()!=1 or message.get_srcComponent()!=1:raise RuntimeError('foreign independent native source')
    return 'packet',safe({'received_mono':received_at,'system_id':message.get_srcSystem(),
                         'component_id':message.get_srcComponent(),**message.to_dict()})


def run(args):
    os.environ['MAVLINK20']='1'
    from pymavlink import mavutil,DFReader
    from audit_native import audit,parse
    source,ap,build,out=[Path(getattr(args,k)).resolve() for k in ('source_root','ardupilot_root','build','output')]
    native=ap/'build/sitl/bin/arducopter';driver=Path(args.driver).resolve() if args.driver else build/'bin/protocol_native_driver'
    params,missions=build/'plugins/mavmoos_parameters.so',build/'plugins/mavmoos_missions.so'
    helper=source/'tests/owned_processes.py';spec=importlib.util.spec_from_file_location('_mavmoos_protocol_owned',helper);owned=importlib.util.module_from_spec(spec);spec.loader.exec_module(owned)
    firmware=owned.native_firmware_profile(source,ap,getattr(args,'firmware_manifest',None),getattr(args,'firmware_attestation',None))
    native_pin=firmware['source_revision'] if firmware else PIN
    native_sha=firmware['native_sha256'] if firmware else NATIVE
    expected={driver:args.expected_driver_sha256,params:args.expected_parameters_sha256,missions:args.expected_missions_sha256,native:native_sha}
    if ref(ap)!=native_pin or any(sha(p)!=h for p,h in expected.items()):raise RuntimeError('frozen native/driver/module identity mismatch')
    startup_source=ap/'libraries/GCS_MAVLink/GCS_Common.cpp'
    if hashlib.sha256(subprocess.check_output(['git','-C',str(ap),'show',native_pin+':libraries/GCS_MAVLink/GCS_Common.cpp'])).hexdigest()!=sha(startup_source):raise RuntimeError('pinned native startup source mismatch')
    protected=[native,driver,params,missions,helper,startup_source,Path(__file__).resolve(),Path(__file__).with_name('audit_native.py'),Path(__file__).with_name('verify_fake.py')]
    protected += [source/p for p in ('src/libmavmoos_runtime/Runtime.cpp','src/libmavmoos_runtime/Runtime.h','src/libmavmoos_runtime/RuntimeMission.cpp','src/libmavmoos_runtime/RuntimeStream.cpp','src/libmavmoos_runtime/Plugin.h','src/libmavmoos_rx/UdpReceiver.cpp','src/libmavmoos_rx/Telemetry.cpp','src/plugins/ParameterPlugin.cpp','src/plugins/MissionPlugin.cpp','tests/protocol_faults/ProtocolNativeDriver.cpp','tests/protocol_faults/ProtocolSupport.h')]
    protected += owned.firmware_source_inputs(source,firmware)
    before={str(p):sha(p) for p in protected};out.mkdir(parents=True,exist_ok=False)
    discovery=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);discovery.bind(('127.0.0.1',0));discovery.settimeout(.2)
    monitor=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);monitor.bind(('127.0.0.1',0));monitor.setblocking(False)
    bind,mon=discovery.getsockname()[1],monitor.getsockname()[1];base,rc=port(socket.SOCK_STREAM),port(socket.SOCK_DGRAM)
    fixture=out/'fixture.parm';fixture.write_text('LOG_DISARMED 1\nSERIAL0_PROTOCOL 2\nSERIAL1_PROTOCOL 2\nMAV2_POSITION 2\nMAV2_EXT_STAT 2\nMAV2_EXTRA3 2\nSIM_WIND_SPD 0\n')
    (out/'message-intervals-chan1.txt').write_text('33 100\n245 500\n')
    command=[str(native),'--model','quad','--home','42.358456,-71.087589,10,0','--speedup','1','--wipe','--defaults',str(ap/'Tools/autotest/default_params/copter.parm')+','+str(fixture),'--serial0',f'udpclient:127.0.0.1:{bind}','--serial1',f'udpclient:127.0.0.1:{mon}','--serial2','none','--serial5','none','--serial6','none','--serial7','none','--serial8','none','--base-port',str(base),'--rc-in-port',str(rc)]
    result={'result':'FAIL','scenario':args.scenario,'scope':'owned disarmed native Runtime/provider service bench; direct Mail fixture, no MOOSDB/helm','hardware':False,'mission_execution_requested':False,'general_parameter_write_requested':False,'observer_writes':0,'source_root':str(source),'source_ref':ref(source),'tooling_source_root':str(Path(__file__).resolve().parents[2]),'tooling_source_ref':ref(Path(__file__).resolve().parents[2]),'observer_startup_policy':'Only the finite 302030203020 SiK sync prefix before first validated MAVLink frame; separately retained, never observation evidence','ardupilot_ref':ref(ap),'native_sha256':native_sha,'firmware_identity':firmware,'driver_sha256':sha(driver),'loaded_modules_sha256':{p.name:sha(p) for p in (params,missions)},'command':command,'ports':{'bridge':bind,'monitor':mon,'base':base,'rc':rc},'artifacts_before':before}
    processes=[];handles=[];packets=[];startup_diagnostics=[];native_process=driver_process=None;peer=None
    def launch(argv,log):
        handle=(out/log).open('w');handles.append(handle);process=subprocess.Popen(argv,cwd=out,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True);processes.append(process);return process
    try:
        native_process=launch(command,'sitl.log');result['native_pid']=native_process.pid
        decoder=mavutil.mavlink.MAVLink(None);decoder.robust_parsing=True;deadline=time.monotonic()+20
        raw=(out/'discovery.mavlink').open('wb');handles.append(raw)
        while peer is None and time.monotonic()<deadline:
            if native_process.poll() is not None:raise RuntimeError('native died during discovery')
            try:data,sender=discovery.recvfrom(4096)
            except socket.timeout:continue
            raw.write(data)
            if any(m.get_type()=='HEARTBEAT' and m.get_srcSystem()==1 and m.get_srcComponent()==1 and m.type==2 and m.autopilot==3 for m in decoder.parse_buffer(data) or []):peer=sender
        inv=inventory(native_process);result['initial_native_socket_inventory']=inv
        if peer is None or peer[0]!='127.0.0.1' or not peer_matches(inv,peer[1],bind):raise RuntimeError('exact owned native peer not proven')
        result['ports']['peer']=peer[1];discovery.close()
        argv=[str(driver),str(bind),str(peer[1]),str(native_process.pid),str(params),str(missions),args.scenario];result['driver_command']=argv
        driver_process=launch(argv,'driver.jsonl');result['driver_pid']=driver_process.pid
        decoder=mavutil.mavlink.MAVLink(None);decoder.robust_parsing=True;deadline=time.monotonic()+100;monitor_peer=None
        raw=(out/'observer.mavlink').open('wb');handles.append(raw);trace=(out/'observer.jsonl').open('w');handles.append(trace)
        diagnostics=(out/'observer-startup.jsonl').open('w');handles.append(diagnostics)
        ended=None
        while time.monotonic()<deadline:
            if native_process.poll() is not None:raise RuntimeError('owned native exited during driver')
            while True:
                try:data,sender=monitor.recvfrom(4096)
                except BlockingIOError:break
                if sender[0]!='127.0.0.1' or monitor_peer is not None and sender!=monitor_peer:raise RuntimeError('passive observer source changed')
                if monitor_peer is None:
                    if not peer_matches(inventory(native_process),sender[1],mon):raise RuntimeError('passive native endpoint not owned')
                    monitor_peer=sender;result['ports']['monitor_peer']=sender[1]
                raw.write(data)
                for m in decoder.parse_buffer(data) or []:
                    startup_bytes=sum(len(bytes.fromhex(r['raw_hex'])) for r in startup_diagnostics)
                    kind,row=observer_message(m,time.monotonic(),len(packets),startup_bytes)
                    if kind=='startup_diagnostic':startup_diagnostics.append(row);diagnostics.write(json.dumps(row,allow_nan=False)+'\n')
                    else:packets.append(row);trace.write(json.dumps(row,allow_nan=False)+'\n')
                if raw.tell()>32*1024*1024 or len(packets)>100000:raise RuntimeError('passive capture bound exceeded')
            if driver_process.poll() is not None:
                if ended is None:ended=time.monotonic()
                if time.monotonic()-ended>=1:break
            time.sleep(.01)
        result['driver_exit']=driver_process.poll()
        result['observer_startup_diagnostics']=startup_diagnostics
        result['monitor_finished_python_mono']=time.monotonic()
        if driver_process.poll() is None or driver_process.returncode!=0:raise RuntimeError('native service driver did not pass within bound')
        inv=inventory(native_process);result['final_native_socket_inventory']=inv
        if monitor_peer is None or not peer_matches(inv,peer[1],bind) or not peer_matches(inv,monitor_peer[1],mon):raise RuntimeError('native socket identity lost')
        result['result']='PASS'
    except BaseException as error:result['error']=str(error)
    finally:
        result['observer_startup_diagnostics']=startup_diagnostics
        remaining=[]
        for process in reversed(processes):remaining.extend(owned.stop_process_tree(process))
        discovery.close();monitor.close()
        for handle in handles:handle.close()
        result['owned_processes_remaining']=remaining;cleared={}
        for name,p in result['ports'].items():
            kind=socket.SOCK_STREAM if name=='base' else socket.SOCK_DGRAM
            with socket.socket(socket.AF_INET,kind) as s:
                try:s.bind(('127.0.0.1',p));cleared[name]=True
                except OSError:cleared[name]=False
        result['ports_clear']=cleared
        if remaining or not all(cleared.values()):result['result']='FAIL';result['cleanup_error']='owned resources remain'
        try:
            dataflash={'arms':[],'parameters':[]};logs=[]
            for path in sorted((out/'logs').glob('*.BIN')):
                logs.append({'path':str(path.relative_to(out)),'sha256':sha(path)});reader=DFReader.DFReader_binary(str(path))
                while True:
                    m=reader.recv_match(type=['ARM','PARM'])
                    if m is None:break
                    row=safe(m.to_dict())
                    if m.get_type()=='ARM':dataflash['arms'].append(row)
                    elif row['Name']=='SIM_WIND_SPD':dataflash['parameters'].append(row)
            if not logs:raise RuntimeError('no closed DataFlash evidence')
            (out/'dataflash.json').write_text(json.dumps(dataflash,indent=2,allow_nan=False)+'\n');result['dataflash_logs']=logs
            rows=[parse(line) for line in (out/'driver.jsonl').read_text().splitlines() if line.startswith('{')]
            proof=audit(rows,packets,dataflash,result.get('monitor_finished_python_mono'));result['protocol_evidence']=proof
            (out/'audit.json').write_text(json.dumps(proof,indent=2)+'\n')
        except BaseException as error:result['result']='FAIL';result['audit_error']=str(error)
        result['artifacts_after']={str(p):sha(p) for p in protected};result['artifact_drift']=[p for p in before if before[p]!=result['artifacts_after'][p]]
        if result['artifact_drift']:result['result']='FAIL';result['artifact_error']='source/artifact drift during native run'
        result['files']={str(p.relative_to(out)):sha(p) for p in sorted(out.rglob('*')) if p.is_file() and p.name!='result.json'}
        (out/'result.json').write_text(json.dumps(safe(result),indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:result.get(k) for k in ('result','scenario','error','audit_error','driver_exit','protocol_evidence','ports_clear','owned_processes_remaining','artifact_drift')},indent=2))
    return int(result['result']!='PASS')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('source-root','ardupilot-root','build','output','scenario','expected-driver-sha256','expected-parameters-sha256','expected-missions-sha256'):p.add_argument('--'+name,required=True)
    p.add_argument('--driver');p.add_argument('--firmware-manifest');p.add_argument('--firmware-attestation')
    a=p.parse_args();signal.signal(signal.SIGTERM,lambda *_:(_ for _ in ()).throw(SystemExit(143)))
    with (Path(tempfile.gettempdir())/'mavmoos-native-sitl.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);raise SystemExit(run(a))
