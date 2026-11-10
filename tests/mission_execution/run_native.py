#!/usr/bin/env python3
"""Exact owned native Copter AUTO scenarios, explicit lifecycle and handback.

Only the selected owned loopback simulator can receive effects. Upload and
execution are separately explicit; no physical transport, RC input, general
parameter write, automatic ARM/takeoff or automatic helm reacquisition.
"""
import argparse
import fcntl
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import shlex
import signal
import socket
import subprocess
import sys
import tempfile
import time
import traceback
from execution_evidence import SCENARIOS,markers,parse,audit_probe,audit_fc,audit_boundary,audit_dataflash
from execution_evidence import audit_typed_probe,audit_typed_fc,audit_typed_dataflash

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tests'))
from copter_frame_fixture import STANDARD_COPTER_FRAMES,bind_copter_frame,copter_default_paths,one_copter_frame_log

FIXTURE_EXTRA={'LOG_DISARMED':1,'SERIAL0_PROTOCOL':2,'SERIAL1_PROTOCOL':2,'SIM_WIND_SPD':0,'SIM_WIND_TURB':0}
INTERVALS='1 500\n33 100\n42 500\n85 200\n245 500\n'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def revision(path):return subprocess.check_output(['git','-C',str(path),'rev-parse','HEAD'],text=True).strip()
def safe(value):
    if isinstance(value,float) and not math.isfinite(value):return {'nonfinite':str(value)}
    if isinstance(value,bytes):return {'raw_hex':value.hex()}
    if isinstance(value,dict):return {key:safe(item) for key,item in value.items()}
    if isinstance(value,(list,tuple)):return [safe(item) for item in value]
    return value
def dump(path,value):Path(path).write_text(json.dumps(safe(value),indent=2,allow_nan=False)+'\n')
def source_helpers(source):
    result={}
    for role,relative in (('ownership','tests/owned_processes.py'),('catalog','tests/control/catalog_evidence.py'),
                          ('firmware','scripts/firmware/manifest.py'),('firmware_evidence','tests/copter_scenarios/scenario_evidence.py'),
                          ('receiver','tests/observation_load/passive_receiver.py'),('boundary','tests/mission_execution/phase_boundary.py')):
        path=(source/relative).resolve();spec=importlib.util.spec_from_file_location('_mission_execution_'+role,path)
        if spec is None or spec.loader is None:raise RuntimeError('source helper unavailable '+str(path))
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);result[role]=module
    return result
def free_port(kind,address='127.0.0.1'):
    with socket.socket(socket.AF_INET,kind) as stream:stream.bind((address,0));return stream.getsockname()[1]
def inventory(process):
    if process.poll() is not None:raise RuntimeError('owned native exited')
    result=subprocess.run(['lsof','-nP','-a','-p',str(process.pid),'-iUDP'],capture_output=True,text=True,timeout=3)
    if result.returncode:raise RuntimeError('native socket inventory unavailable')
    return result.stdout
def exact_peer(text,source,destination):return bool(re.search(r'UDP\s+127\.0\.0\.1:'+str(source)+r'->127\.0\.0\.1:'+str(destination)+r'\b',text))
def exact_rc(text,port):return bool(re.search(r'UDP\s+(?:\*|0\.0\.0\.0):'+str(port)+r'(?:\s|$)',text))
def wait_rc(process,port,deadline):
    while time.monotonic()<deadline:
        text=inventory(process);observed=time.monotonic()
        if observed>=deadline:break
        if exact_rc(text,port):return text,observed
        time.sleep(min(.02,max(0,deadline-time.monotonic())))
    raise RuntimeError('owned native RC socket not bound within original startup deadline')
def live_apps(ownership,antler,mission,bridge,expected=None):
    apps={}
    for record in ownership.owned_records(antler):
        try:tokens=shlex.split(record['command'])
        except ValueError:continue
        if not tokens or str(mission) not in tokens:continue
        name=Path(tokens[0]).name
        if name in ('iMAVLink','pHelmIvP'):
            if name in apps:raise RuntimeError('duplicate owned app '+name)
            apps[name]=record
    if set(apps)!={'iMAVLink','pHelmIvP'}:raise RuntimeError('required owned apps not live')
    if expected and any(apps[k]['pid']!=expected[k]['pid'] or apps[k]['command']!=expected[k]['command'] for k in apps):raise RuntimeError('owned app identity changed')
    result=subprocess.run(['lsof','-nP','-a','-p',str(apps['iMAVLink']['pid']),'-iUDP'],capture_output=True,text=True,timeout=3)
    if result.returncode or not re.search(r'UDP\s+127\.0\.0\.1:'+str(bridge)+r'\b',result.stdout):raise RuntimeError('owned bridge socket not live')
    return apps,result.stdout
def record_marker(control,key,times,events,receiver):
    path=Path(__file__).with_name('phase_boundary.py');spec=importlib.util.spec_from_file_location('_local_phase_boundary',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module.record_marker(control,key,times,events,receiver)

def stop_owned_processes(ownership,processes):
    remaining=[];proven=True;errors=[]
    for process in reversed(processes):
        try:
            observed=ownership.stop_process_tree(process)
            if type(observed) is not list:raise RuntimeError('owned cleanup inventory must return a list')
            remaining.extend(observed)
        except BaseException as error:
            proven=False;errors.append(str(error) or type(error).__name__)
    return remaining,proven,errors

def finalize_cleanup(result,remaining,process_inventories_proven,errors):
    expected={key for key in ('bridge','monitor','peer','monitor_peer','rc','db','base') if key in result['ports']}
    ports=result.get('ports_clear');ports_proven={'bridge','monitor','rc','db','base'}<=expected and type(ports) is dict and set(ports)==expected and all(value is True for value in ports.values())
    receiver=result.get('passive_receiver')
    receiver_clean=type(receiver) is dict and receiver.get('thread_retired') is True and receiver.get('socket_closed') is True and receiver.get('error') is None
    unknown=not process_inventories_proven or bool(errors) or not ports_proven or not receiver_clean
    result['known_owned_processes_remaining']=remaining
    result['owned_processes_remaining']=None if unknown else remaining
    result['cleanup_proven']=not unknown and not remaining and all(ports.values())
    if errors:result['cleanup_exceptions']=errors
    if not result['cleanup_proven']:
        result['result']='FAIL';result['cleanup_error']='owned cleanup unknown or resources remain'


def run(args):
    os.environ['MAVLINK20']='1'
    from pymavlink import mavutil,DFReader
    ap,moos,build,out=[Path(getattr(args,key)).resolve() for key in ('ardupilot_root','moos_root','build','output')]
    source=Path(args.source_root).resolve();behavior=Path(args.behavior_dir).resolve()
    typed=args.scenario=='auto_typed_lifecycle'
    frame=args.copter_frame
    expected_type,expected_class=STANDARD_COPTER_FRAMES[frame]
    if typed and subprocess.check_output(['git','-C',str(source),'status','--porcelain'],text=True).strip():
        raise RuntimeError('typed lifecycle requires a clean source checkout')
    probe_path=Path(args.probe).resolve() if args.probe else build/'bin/mission_execution_probe'
    helpers=source_helpers(source);ownership,catalog,fe=helpers['ownership'],helpers['catalog'],helpers['firmware_evidence']
    verified=helpers['firmware'].verify_native(args.firmware_manifest,args.firmware_attestation,ap,expected_profile='arducopter')
    manifest=helpers['firmware'].validate_manifest(args.firmware_manifest)
    manifest_bytes=Path(args.firmware_manifest).read_bytes();attestation_bytes=Path(args.firmware_attestation).read_bytes()
    expected=fe.firmware_inputs(manifest_bytes,attestation_bytes,verified)
    if parse(manifest_bytes)!=manifest:raise RuntimeError('firmware source manifest changed during preflight')
    if manifest['fixture']['home']!=[42.358456,-71.087589,10,0] or verified['frame']!='quad':raise RuntimeError('attested standard Copter binary and fixture home required')
    if frame!='quad' and (source!=ROOT or verified['profile_id']!='arducopter-4.6.3-quad' or
       revision(moos)!=manifest['dependencies']['moos_revision']):
        raise RuntimeError('standard hexa/octa qualification requires the selected runner, 4.6.3 binary and stock MOOS')
    native_defaults=copter_default_paths(ap,frame)
    fixture={**manifest['fixture']['parameters'],**FIXTURE_EXTRA}
    native,app=Path(verified['native_path']),build/'bin/iMAVLink'
    modules=['telemetry','parameters','missions','arducopter_control']
    protected=[native,app,probe_path,moos/'bin/pAntler',moos/'bin/pHelmIvP',Path(__file__).resolve(),
               Path(__file__).with_name('MissionExecutionProbe.cpp'),Path(__file__).with_name('execution_evidence.py'),
               source/'tests/observation_load/FinishBoundary.h',source/'tests/copter_frame_fixture.py']
    protected += [build/'plugins'/('mavmoos_'+name+'.so') for name in modules]
    protected += [Path(module.__file__) for module in helpers.values()]
    protected += ownership.firmware_source_inputs(source)
    protected += [p for p in (source/'src').rglob('*') if p.is_file() and p.suffix in ('.h','.cpp')]
    protected += [source/'missions/copter_operations'/('meta_vehicle.'+suffix) for suffix in ('moos','bhv')]
    protected += [ownership.arducopter_gcs_source(ap)]
    protected += [ap/name for name in ('ArduCopter/mode_auto.cpp','libraries/AP_Mission/AP_Mission.cpp',
                 'libraries/GCS_MAVLink/GCS_Common.cpp','libraries/AP_RCProtocol/AP_RCProtocol_UDP.cpp',
                 'libraries/SITL/SITL.cpp','libraries/AP_Logger/LogStructure.h')]
    protected += native_defaults
    protected += [Path(args.firmware_manifest).resolve(),Path(args.firmware_attestation).resolve()]
    protected=list(dict.fromkeys(protected))
    for path in protected:
        if not path.is_file():raise RuntimeError('required frozen input missing '+str(path))
    if sha(app)!=args.expected_app_sha256 or sha(probe_path)!=args.expected_probe_sha256:raise RuntimeError('frozen app/probe hash differs')
    before={str(p):sha(p) for p in protected}
    out.mkdir(parents=True,exist_ok=False);control=out/'control';control.mkdir();(out/'logs').mkdir()
    copied={}
    for role,data in (('manifest',manifest_bytes),('attestation',attestation_bytes)):
        original=Path(getattr(args,'firmware_'+role)).resolve()
        if hashlib.sha256(data).hexdigest()!=before[str(original)]:raise RuntimeError('firmware input drift before archive')
        path=out/fe.FIRMWARE_FILES[role];path.write_bytes(data);protected.append(path);before[str(path)]=sha(path)
        copied[role]={'path':fe.FIRMWARE_FILES[role],'sha256':before[str(path)]}
    if helpers['firmware'].verify_native(out/fe.FIRMWARE_FILES['manifest'],out/fe.FIRMWARE_FILES['attestation'],ap,expected_profile='arducopter')!=verified:raise RuntimeError('archived firmware inputs differ before I/O')
    bridge=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);bridge.bind(('127.0.0.1',0));bridge.setblocking(False)
    monitor=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);monitor.bind(('127.0.0.1',0));monitor.setblocking(False)
    bridge_port,monitor_port=bridge.getsockname()[1],monitor.getsockname()[1]
    db,base,rc=free_port(socket.SOCK_STREAM),free_port(socket.SOCK_STREAM),free_port(socket.SOCK_DGRAM,'0.0.0.0')
    while db==base:base=free_port(socket.SOCK_STREAM)
    defaults=out/'fixture.parm';defaults.write_text(''.join(f'{k} {v}\n' for k,v in fixture.items()))
    (out/'message-intervals-chan1.txt').write_text(INTERVALS)
    command=[str(native),'--model',frame,'--home','42.358456,-71.087589,10,0','--speedup','1','--wipe',
             '--defaults',','.join(map(str,[*native_defaults,defaults])),
             '--serial0',f'udpclient:127.0.0.1:{bridge_port}','--serial1',f'udpclient:127.0.0.1:{monitor_port}',
             '--serial2','none','--serial5','none','--serial6','none','--serial7','none','--serial8','none',
             '--base-port',str(base),'--rc-in-port',str(rc)]
    result={'result':'FAIL','scenario':args.scenario,'profile':'arducopter','scope':'explicit AUTO execution and handback in exact owned native SITL; no hardware',
            'source_ref':revision(source),'ardupilot_ref':revision(ap),'moos_ref':revision(moos),'source_root':str(source),
            'firmware_identity':verified,'firmware_inputs':copied,'command':command,'hardware':False,'timewarp':1,
            'observer_writes':0,'rc_input_sent':False,'mission_execution_requested':True,'parameter_write_requested':False,
            'frame':frame,'frame_identity':{'expected_mav_type':expected_type,'expected_frame_class':expected_class},
            'fixture_parameters':fixture,'modules_requested':modules,'app_sha256':sha(app),'native_sha256':sha(native),'probe_sha256':sha(probe_path),
            'ports':{'bridge':bridge_port,'monitor':monitor_port,'db':db,'base':base,'rc':rc},'events':[],'artifacts_before':before,
            'loaded_helpers':{role:{'path':m.__file__,'sha256':sha(m.__file__)} for role,m in helpers.items()}}
    processes=[];streams=[];capture_streams=[];packets=[];sitl=antler=probe=receiver=None;owned=None
    mission=out/'targ_copter.moos';times={};phase_names=markers(args.scenario)
    def launch(argv,log,env=None):
        handle=(out/log).open('w');streams.append(handle)
        process=subprocess.Popen(argv,cwd=out,env=env,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True);processes.append(process);return process
    try:
        sitl=launch(command,'sitl.log');result['sitl_pid']=sitl.pid
        decoder=mavutil.mavlink.MAVLink(None);decoder.robust_parsing=True;peer=None;deadline=time.monotonic()+20
        with (out/'discovery.mavlink').open('wb') as raw:
            while time.monotonic()<deadline:
                if sitl.poll() is not None:raise RuntimeError('native startup exited')
                try:data,sender=bridge.recvfrom(65535)
                except BlockingIOError:time.sleep(.005);continue
                raw.write(data)
                if any(m.get_type()=='HEARTBEAT' and m.get_srcSystem()==1 and m.get_srcComponent()==1 and m.type==expected_type and m.autopilot==3 for m in decoder.parse_buffer(data) or []):peer=sender;break
        snapshot=inventory(sitl)
        if peer is None or peer[0]!='127.0.0.1' or not exact_peer(snapshot,peer[1],bridge_port):raise RuntimeError('exact owned primary endpoint unproven')
        result['ports']['peer']=peer[1];result['native_primary_socket_inventory']=snapshot
        rc_snapshot,rc_at=wait_rc(sitl,rc,deadline);result['native_rc_socket_readiness']={'inventory':rc_snapshot,'observed_python_mono':rc_at,'original_deadline':deadline,'rc_input_sent':False}
        bridge.close()
        env=dict(os.environ);env['PATH']=os.pathsep.join([str(build/'bin'),str(moos/'bin'),env.get('PATH','')]);env['IVP_BEHAVIOR_DIRS']=str(behavior)
        macros={'MOOS_PORT':db,'APP_PATH':app,'BIND_PORT':bridge_port,'PEER_PORT':peer[1],'LAT_ORIGIN':'42.358456','LONG_ORIGIN':'-71.087589',
                'SIMULATION_ONLY':'true','SHADOW_MODE':'false','SITL_PID':sitl.pid,'PLUGIN_DIRECTORY':build/'plugins','BEHAVIOR_DIRECTORY':behavior,
                'HEIGHT':5,'SPEED':1.5,'FACING_POLICY':'course','FACING_HEADING':0}
        for suffix in ('moos','bhv'):
            argv=[str(moos/'bin/nsplug'),str(source/f'missions/copter_operations/meta_vehicle.{suffix}'),str(out/f'targ_copter.{suffix}'),'--strict','--force','-x']+[f'{k}={v}' for k,v in macros.items()]
            completed=subprocess.run(argv,capture_output=True,text=True,env=env,timeout=10);(out/('nsplug-'+suffix+'.log')).write_text(completed.stdout+completed.stderr)
            if completed.returncode:raise RuntimeError('ordinary stock mission generation failed '+suffix)
        text=bind_copter_frame(mission.read_text(),frame)
        recovery_directory=out/'auto-recovery'
        recovery_directory.mkdir(mode=0o700)
        replacements=[
            ('  plugins = telemetry,parameters,arducopter_control',
             '  plugins = telemetry,parameters,missions,arducopter_control\n'
             '  control_mission_execution = true\n'
             f'  auto_recovery_directory = {recovery_directory}'),
            ('  AppTick = 30','  AppTick = 50'),('  CommsTick = 30','  CommsTick = 50')]
        for old,new in replacements:
            if text.count(old)!=1:raise RuntimeError('ordinary template anchor drift '+old)
            text=text.replace(old,new,1)
        mission.write_text(text);result['mission_sha256']=sha(mission);result['behavior_sha256']=sha(out/'targ_copter.bhv')
        antler=launch([str(moos/'bin/pAntler'),str(mission)],'pantler.log',env)
        probe=launch([str(probe_path),str(db),str(control),args.scenario],'probe.jsonl',env)
        decoder=mavutil.mavlink.MAVLink(None);decoder.robust_parsing=True;valid_frames=0;startup_bytes=0
        raw=(out/'fc-observer.mavlink').open('wb');trace=(out/'fc-observer.jsonl').open('w');startup=(out/'observer-startup.jsonl').open('w');capture_streams=[raw,trace,startup]
        def verify_peer(sender):
            text=inventory(sitl)
            if sender[0]!='127.0.0.1' or not exact_peer(text,sender[1],monitor_port):raise RuntimeError('exact owned passive endpoint unproven')
            result['ports']['monitor_peer']=sender[1];result['native_observer_socket_inventory']=text;return text
        def consume(data,sender,at):
            nonlocal valid_frames,startup_bytes
            raw.write(data);raw.flush()
            for message in decoder.parse_buffer(data) or []:
                if message.get_type()=='BAD_DATA':
                    bad=bytes(message.data);prefix=b'0 '*3
                    if valid_frames or not bad or startup_bytes+len(bad)>6 or bad!=prefix[startup_bytes:startup_bytes+len(bad)] or message.reason!='Bad prefix':raise RuntimeError('invalid passive startup/frame bytes')
                    startup_bytes+=len(bad);startup.write(json.dumps({'mono':at,'raw_hex':bad.hex(),'reason':message.reason})+'\n');startup.flush();continue
                if startup_bytes not in (0,6):raise RuntimeError('incomplete passive startup prefix')
                if message.get_srcSystem()!=1 or message.get_srcComponent()!=1:raise RuntimeError('foreign passive FC source')
                if message.get_type()=='HEARTBEAT' and (message.type,message.autopilot)!=(expected_type,3):
                    raise RuntimeError('independent passive heartbeat differs from selected Copter frame')
                valid_frames+=1
                if message.get_type() not in ('HEARTBEAT','GLOBAL_POSITION_INT','EXTENDED_SYS_STATE','MISSION_CURRENT','MISSION_ITEM_REACHED','COMMAND_ACK','HOME_POSITION','AUTOPILOT_VERSION','STATUSTEXT'):continue
                wire=bytes(message.get_msgbuf());row={'mono':at,'system':1,'component':1,'wire_magic':wire[0],'wire_payload_length':wire[1],**safe(message.to_dict())};packets.append(row);trace.write(json.dumps(row,allow_nan=False)+'\n');trace.flush()
        receiver=helpers['receiver'].PassiveReceiver(monitor,verify_peer,consume,max_datagrams=150000);receiver.start()
        last_check=-1;drain=None;deadline=time.monotonic()+SCENARIOS[args.scenario]['supervisor_s']
        with (out/'ownership.jsonl').open('w') as owner_trace:
            while time.monotonic()<deadline:
                receiver.check()
                if sitl.poll() is not None or antler.poll() is not None:raise RuntimeError('owned native/launcher exited')
                now=time.monotonic()
                if now-last_check>=.5:
                    if owned is None:
                        try:owned,app_sockets=live_apps(ownership,antler,mission,bridge_port)
                        except RuntimeError:
                            if now>deadline-SCENARIOS[args.scenario]['supervisor_s']+10:raise
                        else:result['owned_apps']=owned
                    else:owned,app_sockets=live_apps(ownership,antler,mission,bridge_port,owned)
                    if owned:owner_trace.write(json.dumps({'python_mono':time.monotonic(),'owned_apps':owned,'socket_inventory':app_sockets})+'\n');owner_trace.flush()
                    last_check=now
                for name in phase_names:helpers['boundary'].record_marker(control,name,times,result['events'],receiver)
                if probe.poll() is not None:
                    if drain is None:drain=time.monotonic()+1.5
                    if time.monotonic()>=drain:break
                time.sleep(.005)
        receiver.close();result['passive_receiver']=receiver.status();result['monitor_finished_python_mono']=time.monotonic()
        for handle in capture_streams:handle.close()
        if probe.poll() is None:raise RuntimeError('bounded whole AUTO scenario timeout')
        result['probe_exit']=probe.returncode
        if probe.returncode:raise RuntimeError('AUTO probe failed')
        if owned is None:raise RuntimeError('owned app identity never proven')
        result['final_owned_apps'],result['final_app_socket_inventory']=live_apps(ownership,antler,mission,bridge_port,owned)
        events=[parse(line) for line in (out/'probe.jsonl').read_text().splitlines() if line.startswith('{')]
        result['probe_evidence']=audit_typed_probe(events) if typed else audit_probe(events,args.scenario)
        result['firmware_discovery']=fe.audit_firmware(events,expected,result['probe_evidence']['phases_probe_elapsed']['takeoff.started'])
        observed=catalog.observed_catalog(out/'probe.jsonl',set(modules),native=True,helm=True,mode='SITL_CONTROL',operations_helm=True,mission_execution=True,mission_resume=True)
        result['catalog']=observed;result['loaded_modules_sha256']=catalog.loaded_module_hashes(build/'plugins',observed)
        if typed:
            result['independent_fc']=audit_typed_fc(packets,times,result['monitor_finished_python_mono'],
                                                  result['probe_evidence']['stored_plan'])
        else:
            result['finish_boundary']=audit_boundary(result['events'],(control/'auto.finished').read_bytes(),(control/'auto.finished.ack').read_bytes())
            result['phase_boundaries']=helpers['boundary'].audit_boundaries(result['events'],result['probe_evidence'],control)
            result['independent_fc']=audit_fc(packets,times,result['monitor_finished_python_mono'],args.scenario,result['probe_evidence'])
        result['result']='PASS'
    except BaseException as error:result['error']=str(error) or type(error).__name__;result['traceback']=traceback.format_exc()
    finally:
        cleanup_errors=[]
        if receiver is not None:
            try:receiver.close();result['passive_receiver']=receiver.status()
            except BaseException as error:
                result['result']='FAIL';result['receiver_cleanup_error']=str(error);cleanup_errors.append(str(error) or type(error).__name__)
                try:result['passive_receiver']=receiver.status()
                except BaseException as status_error:result['passive_receiver']=None;cleanup_errors.append(str(status_error) or type(status_error).__name__)
        else:result['passive_receiver']={'not_started':True,'thread_retired':True,'socket_closed':False,'error':None}
        retired=result.get('passive_receiver')
        if receiver is None or (type(retired) is dict and retired.get('thread_retired') is True):
            for handle in capture_streams:
                try:handle.close()
                except BaseException as error:cleanup_errors.append(str(error) or type(error).__name__)
        remaining,processes_proven,process_errors=stop_owned_processes(ownership,processes);cleanup_errors.extend(process_errors)
        for handle in [bridge,monitor]+streams:
            try:handle.close()
            except BaseException as error:cleanup_errors.append(str(error) or type(error).__name__)
        if receiver is None:result['passive_receiver']['socket_closed']=monitor.fileno()==-1
        result['ports_clear']={}
        for key in ('bridge','monitor','peer','monitor_peer','rc'):
            if key not in result['ports']:continue
            try:
                with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as stream:
                    try:stream.bind(('0.0.0.0' if key=='rc' else '127.0.0.1',result['ports'][key]));result['ports_clear'][key]=True
                    except OSError:result['ports_clear'][key]=False
            except BaseException as error:result['ports_clear'][key]=None;cleanup_errors.append(str(error) or type(error).__name__)
        for key in ('db','base'):
            try:
                with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as stream:stream.settimeout(.2);result['ports_clear'][key]=stream.connect_ex(('127.0.0.1',result['ports'][key]))!=0
            except BaseException as error:result['ports_clear'][key]=None;cleanup_errors.append(str(error) or type(error).__name__)
        finalize_cleanup(result,remaining,processes_proven,cleanup_errors)
        try:
            logs=[];params={};arms=[];modes=[];commands=[]
            for path in sorted((out/'logs').glob('*.BIN')):
                digest=sha(path);reader=DFReader.DFReader_binary(str(path))
                while True:
                    record=reader.recv_match(type=['PARM','ARM','MODE','CMD'])
                    if record is None:break
                    row={'log':path.name,**safe(record.to_dict())}
                    if record.get_type()=='PARM':
                        name=record.Name.decode(errors='replace') if isinstance(record.Name,bytes) else str(record.Name)
                        if name in fixture:params[name]=float(record.Value)
                    elif record.get_type()=='ARM':arms.append(row)
                    elif record.get_type()=='MODE':modes.append(row)
                    else:commands.append(row)
                if sha(path)!=digest:raise RuntimeError('closed FC log drifted during extraction')
                logs.append({'path':str(path.relative_to(out)),'sha256':digest})
            if not logs:raise RuntimeError('closed owned FC log missing')
            result['dataflash']={'logs':logs,'fixture_parameter_readback':params,'arm_records':arms,'mode_records':modes,'mission_command_records':commands}
            dump(out/'dataflash-observations.json',result['dataflash'])
            result['dataflash_evidence']=(audit_typed_dataflash if typed else audit_dataflash)(arms,modes,commands,params,fixture)
            if frame!='quad':
                result['frame_identity']['dataflash_parameters']=one_copter_frame_log(out/'logs',frame)
        except BaseException as error:result['result']='FAIL';result['dataflash_error']=str(error)
        try:result['firmware_identity_after']=helpers['firmware'].verify_native(args.firmware_manifest,args.firmware_attestation,ap,expected_profile='arducopter')
        except BaseException as error:result['result']='FAIL';result['firmware_error']=str(error)
        try:
            result['artifacts_after']={str(p):sha(p) for p in protected};result['artifact_drift']=[p for p in before if before[p]!=result['artifacts_after'][p]]
            if result['artifact_drift']:raise RuntimeError('source/artifact drift during scenario')
        except BaseException as error:result['result']='FAIL';result['artifact_error']=str(error)
        result['files']={str(p.relative_to(out)):sha(p) for p in sorted(out.rglob('*')) if p.is_file() and p.name!='result.json'}
        dump(out/'result.json',result)
    print(json.dumps(safe({k:result.get(k) for k in ('result','scenario','error','dataflash_error','firmware_error','probe_exit','probe_evidence','independent_fc','dataflash_evidence','ports_clear','cleanup_proven','owned_processes_remaining','artifact_drift')}),indent=2))
    return int(result['result']!='PASS')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('ardupilot-root','moos-root','build','behavior-dir','output','source-root','expected-app-sha256','expected-probe-sha256','firmware-manifest','firmware-attestation'):parser.add_argument('--'+key,required=True)
    parser.add_argument('--probe');parser.add_argument('--scenario',choices=tuple(SCENARIOS),required=True)
    parser.add_argument('--copter-frame',choices=tuple(STANDARD_COPTER_FRAMES),default='quad');args=parser.parse_args()
    signal.signal(signal.SIGTERM,lambda *_:(_ for _ in ()).throw(SystemExit(143)))
    with (Path(tempfile.gettempdir())/'mavmoos-native-sitl.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        raise SystemExit(run(args))
