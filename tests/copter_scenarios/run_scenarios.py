#!/usr/bin/env python3
"""Owned native Copter lifecycle/hold scenarios through unchanged stock helm.

No physical route, relay, RC/servo write, mission execution or general parameter
write. Longer bounds cover GPS/EKF convergence and complete flight cycles.
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
import tempfile
import time
import traceback
from scenario_evidence import audit_probe,audit_fc,audit_dataflash,firmware_inputs,FIRMWARE_FILES,parse,markers,SCENARIOS

ROOT=Path(__file__).resolve().parents[2]
FIXTURE_BASE={'LOG_DISARMED':1,'SERIAL0_PROTOCOL':2,'SERIAL1_PROTOCOL':2,'SIM_WIND_DIR':270,'SIM_WIND_TURB':0}
INTERVALS='1 500\n33 100\n85 200\n245 500\n'

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
    for role,relative in (('ownership','tests/owned_processes.py'),('catalog','tests/control/catalog_evidence.py'),('firmware','scripts/firmware/manifest.py'),
                          ('receiver','tests/observation_load/passive_receiver.py'),('boundary','tests/copter_scenarios/hold_boundary.py')):
        path=(source/relative).resolve();spec=importlib.util.spec_from_file_location('_observation_load_'+role,path)
        if spec is None or spec.loader is None:raise RuntimeError('source helper unavailable: '+str(path))
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);result[role]=module
    return result
def free_port(kind):
    with socket.socket(socket.AF_INET,kind) as stream:stream.bind(('127.0.0.1',0));return stream.getsockname()[1]
def inventory(process):
    if process.poll() is not None:raise RuntimeError('owned native exited')
    result=subprocess.run(['lsof','-nP','-a','-p',str(process.pid),'-iUDP'],capture_output=True,text=True,timeout=3)
    if result.returncode:raise RuntimeError('native socket inventory unavailable')
    return result.stdout
def exact_peer(text,source,destination):return bool(re.search(r'UDP\s+127\.0\.0\.1:'+str(source)+r'->127\.0\.0\.1:'+str(destination)+r'\b',text))
def live_apps(ownership,antler,mission,bridge,expected=None):
    apps={}
    for record in ownership.owned_records(antler):
        try:tokens=shlex.split(record['command'])
        except ValueError:continue
        if not tokens or str(mission) not in tokens:continue
        name=Path(tokens[0]).name
        if name in ('iMAVLink','pHelmIvP'):
            if name in apps:raise RuntimeError('duplicate owned mission app '+name)
            apps[name]=record
    if set(apps)!={'iMAVLink','pHelmIvP'}:raise RuntimeError('required owned mission apps not live')
    if expected and any(apps[k]['pid']!=expected[k]['pid'] or apps[k]['command']!=expected[k]['command'] for k in apps):raise RuntimeError('owned mission app identity changed')
    result=subprocess.run(['lsof','-nP','-a','-p',str(apps['iMAVLink']['pid']),'-iUDP'],capture_output=True,text=True,timeout=3)
    if result.returncode or not re.search(r'UDP\s+127\.0\.0\.1:'+str(bridge)+r'\b',result.stdout):raise RuntimeError('owned bridge socket not live')
    return apps,result.stdout

def stop_owned_processes(ownership,processes):
    remaining=[];proven=True;errors=[]
    for process in reversed(processes):
        try:
            observed=ownership.stop_process_tree(process)
            if type(observed) is not list:raise RuntimeError('owned cleanup inventory must return a list')
            remaining.extend(observed)
        except BaseException as error:proven=False;errors.append(str(error) or type(error).__name__)
    return remaining,proven,errors

def finalize_cleanup(result,remaining,processes_proven,errors):
    expected={key for key in ('bridge','monitor','peer','monitor_peer','rc','db','base') if key in result['ports']}
    ports=result.get('ports_clear');receiver=result.get('passive_receiver')
    ports_proven={'bridge','monitor','rc','db','base'}<=expected and type(ports) is dict and set(ports)==expected and all(x is True for x in ports.values())
    receiver_proven=type(receiver) is dict and receiver.get('thread_retired') is True and receiver.get('socket_closed') is True and receiver.get('error') is None
    unknown=not processes_proven or bool(errors) or not ports_proven or not receiver_proven
    result['known_owned_processes_remaining']=remaining
    result['owned_processes_remaining']=None if unknown else remaining
    result['cleanup_proven']=not unknown and not remaining
    if errors:result['cleanup_exceptions']=errors
    if not result['cleanup_proven']:result['result']='FAIL';result['cleanup_error']='owned cleanup unknown or resources remain'

def run(args):
    os.environ["MAVLINK20"]="1"
    from pymavlink import mavutil,DFReader
    ap,moos,build,out=[Path(getattr(args,key)).resolve() for key in ('ardupilot_root','moos_root','build','output')]
    source=Path(args.source_root).resolve() if args.source_root else ROOT
    behavior=Path(args.behavior_dir).resolve()
    probe_path=Path(args.probe).resolve() if args.probe else build/'bin/copter_scenario_probe'
    helpers=source_helpers(source);ownership,catalog=helpers['ownership'],helpers['catalog']
    module_names=['telemetry','parameters','arducopter_control']
    verified=helpers['firmware'].verify_native(args.firmware_manifest,args.firmware_attestation,ap,expected_profile='arducopter')
    manifest=helpers['firmware'].validate_manifest(args.firmware_manifest)
    manifest_bytes=Path(args.firmware_manifest).read_bytes();attestation_bytes=Path(args.firmware_attestation).read_bytes()
    firmware_proof=firmware_inputs(manifest_bytes,attestation_bytes,verified)
    if parse(manifest_bytes)!=manifest:raise RuntimeError('source manifest changed while verifying fixture')
    if manifest['fixture']['home']!=[42.358456,-71.087589,10,0] or verified['frame']!='quad':raise RuntimeError('scenario requires explicit standard quad/home fixture')
    scenario=SCENARIOS[args.scenario];FIXTURE={**manifest['fixture']['parameters'],**FIXTURE_BASE,'SIM_WIND_SPD':scenario['wind_speed_mps']}
    MARKERS=markers(args.scenario)
    native,app=Path(verified['native_path']),build/'bin/iMAVLink'
    protected=[native,app,probe_path,moos/'bin/pAntler',moos/'bin/pHelmIvP',Path(__file__).resolve(),Path(__file__).with_name('CopterScenarioProbe.cpp'),Path(__file__).with_name('scenario_evidence.py')]
    protected += [build/'plugins'/('mavmoos_'+name+'.so') for name in module_names]
    protected += [Path(m.__file__) for m in helpers.values()]
    protected += [source/'tests/observation_load/FinishBoundary.h']
    protected += ownership.firmware_source_inputs(source)
    protected += [source/name for name in ('src/libmavmoos_runtime/Plugin.h','src/libmavmoos_runtime/Runtime.cpp','src/libmavmoos_runtime/RuntimeMission.cpp','src/libmavmoos_runtime/FirmwareDiscovery.h','src/libmavmoos_runtime/FirmwareDiscovery.cpp','src/libmavmoos_rx/UdpReceiver.cpp','src/iMAVLink/MAVLink.cpp')]
    protected += [source/'src/plugins'/name for name in ['TelemetryPlugin.cpp','ArduCopterControlPlugin.cpp']]
    protected += [source/'missions/copter_operations'/('meta_vehicle.'+suffix) for suffix in ('moos','bhv')]
    protected += [ownership.arducopter_gcs_source(ap)]
    protected += [ap/name for name in ('Tools/autotest/default_params/copter.parm','libraries/SITL/SIM_Frame.cpp','libraries/SITL/SIM_Battery.cpp')]
    for path in protected:
        if not path.is_file():raise RuntimeError('required protected input missing '+str(path))
    if sha(app)!=args.expected_app_sha256 or sha(probe_path)!=args.expected_probe_sha256:raise RuntimeError('frozen app/probe identity differs')
    protected += [ap/name for name in ('libraries/GCS_MAVLink/GCS_Common.cpp','libraries/SITL/SITL.cpp','libraries/AP_Logger/LogStructure.h')]
    protected += [Path(args.firmware_manifest).resolve(),Path(args.firmware_attestation).resolve()]
    protected += [source/name for name in ('src/libmavmoos_control/CopterOperations.cpp','src/libmavmoos_control/CopterCommands.cpp','src/libmavmoos_control/ArduCopterBackend.cpp')]
    before={str(path):sha(path) for path in protected}
    out.mkdir(parents=True,exist_ok=False);control=out/'control';control.mkdir();(out/'logs').mkdir()
    # Exact identity inputs are retained before opening any socket or process.
    # The offline reader derives its expected version from these bound bytes.
    copied_inputs={}
    for role,data in (('manifest',manifest_bytes),('attestation',attestation_bytes)):
        original=Path(getattr(args,'firmware_'+role)).resolve()
        if hashlib.sha256(data).hexdigest()!=before[str(original)]:raise RuntimeError('firmware input changed before archival')
        path=out/FIRMWARE_FILES[role];path.write_bytes(data);protected.append(path);before[str(path)]=sha(path)
        copied_inputs[role]={'path':FIRMWARE_FILES[role],'sha256':before[str(path)]}
    if helpers['firmware'].verify_native(out/FIRMWARE_FILES['manifest'],out/FIRMWARE_FILES['attestation'],ap,expected_profile='arducopter')!=verified:
        raise RuntimeError('archived firmware identity differs before I/O')
    bridge=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);bridge.bind(('127.0.0.1',0));bridge.setblocking(False)
    monitor=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);monitor.bind(('127.0.0.1',0));monitor.setblocking(False)
    bridge_port,monitor_port=bridge.getsockname()[1],monitor.getsockname()[1]
    db,base,rc=free_port(socket.SOCK_STREAM),free_port(socket.SOCK_STREAM),free_port(socket.SOCK_DGRAM)
    while base==db:base=free_port(socket.SOCK_STREAM)
    defaults=out/'fixture.parm';defaults.write_text(''.join(f'{key} {value}\n' for key,value in FIXTURE.items()))
    (out/'message-intervals-chan1.txt').write_text(INTERVALS)
    command=[str(native),'--model','quad','--home','42.358456,-71.087589,10,0','--speedup','1','--wipe',
        '--defaults',str(ap/'Tools/autotest/default_params/copter.parm')+','+str(defaults),
        '--serial0',f'udpclient:127.0.0.1:{bridge_port}','--serial1',f'udpclient:127.0.0.1:{monitor_port}',
        '--serial2','none','--serial5','none','--serial6','none','--serial7','none','--serial8','none',
        '--base-port',str(base),'--rc-in-port',str(rc)]
    result={'result':'FAIL','scope':'owned native Copter complete lifecycle/held-target/return scenarios with unchanged stock helm','scenario':args.scenario,'profile':'arducopter','firmware_identity':verified,'firmware_inputs':copied_inputs,'hardware':False,'timewarp':1,
        'command':command,'modules_requested':module_names,'source_root':str(source),'source_ref':revision(source),'ardupilot_ref':revision(ap),'moos_ref':revision(moos),
        'ports':{'bridge':bridge_port,'monitor':monitor_port,'db':db,'base':base,'rc':rc},'observer_writes':0,'rc_input_sent':False,'fixture_overrides':{'SIM_WIND_SPD':scenario['wind_speed_mps'],'SIM_WIND_DIR':270,'SIM_WIND_TURB':0},
        'mission_execution_requested':False,'parameter_write_requested':False,'fixture_parameters':FIXTURE,'interval_file_units':'milliseconds',
        'host_interval_request_evidence':'descriptor intent + unit-layer encoder tests separately; primary uplink packets/ACKs not captured here',
        'loaded_helpers':{role:{'path':m.__file__,'sha256':sha(m.__file__)} for role,m in helpers.items()},'artifacts_before':before,
        'app_sha256':before[str(app)],'native_sha256':before[str(native)],'probe_sha256':before[str(probe_path)],'events':[]}
    processes=[];streams=[];packets=[];capture_streams=[];receiver=None;antler=sitl=probe=None;owned=None;mission=out/'targ_copter.moos'
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
                if any(m.get_type()=='HEARTBEAT' and m.get_srcSystem()==1 and m.get_srcComponent()==1 and m.type==2 and m.autopilot==3 for m in decoder.parse_buffer(data) or []):peer=sender;break
        sockets=inventory(sitl)
        if peer is None or peer[0]!='127.0.0.1' or not exact_peer(sockets,peer[1],bridge_port):raise RuntimeError('exact owned primary FC endpoint unproven')
        result['ports']['peer']=peer[1];result['native_primary_socket_inventory']=sockets;bridge.close()
        env=dict(os.environ);env['PATH']=os.pathsep.join([str(build/'bin'),str(moos/'bin'),env.get('PATH','')]);env['IVP_BEHAVIOR_DIRS']=str(behavior)
        macros={'MOOS_PORT':db,'APP_PATH':app,'BIND_PORT':bridge_port,'PEER_PORT':peer[1],'LAT_ORIGIN':'42.358456','LONG_ORIGIN':'-71.087589',
                'SIMULATION_ONLY':'true','SHADOW_MODE':'false','SITL_PID':sitl.pid,'PLUGIN_DIRECTORY':build/'plugins','BEHAVIOR_DIRECTORY':behavior,
                'HEIGHT':5,'SPEED':1.5,'FACING_POLICY':'course','FACING_HEADING':0}
        for suffix in ('moos','bhv'):
            argv=[str(moos/'bin/nsplug'),str(source/f'missions/copter_operations/meta_vehicle.{suffix}'),str(out/f'targ_copter.{suffix}'),'--strict','--force','-x']+[f'{k}={v}' for k,v in macros.items()]
            completed=subprocess.run(argv,capture_output=True,text=True,env=env);(out/('nsplug-'+suffix+'.log')).write_text(completed.stdout+completed.stderr)
            if completed.returncode:raise RuntimeError('stock mission generation failed '+suffix)
        text=mission.read_text()
        for before_text,after_text in [('  AppTick = 30','  AppTick = 50'),('  CommsTick = 30','  CommsTick = 50')]:
            if text.count(before_text)!=1:raise RuntimeError('stock template anchor drift '+before_text)
            text=text.replace(before_text,after_text,1)
        mission.write_text(text);result['mission_sha256']=sha(mission);result['behavior_sha256']=sha(out/'targ_copter.bhv')
        antler=launch([str(moos/'bin/pAntler'),str(mission)],'pantler.log',env)
        probe=launch([str(probe_path),str(db),str(control),args.scenario],'probe.jsonl',env)
        decoder=mavutil.mavlink.MAVLink(None);decoder.robust_parsing=True;phase_times={};drain=None;last_check=-1;startup_bytes=0;valid_frames=0
        raw=(out/'fc-observer.mavlink').open('wb');trace=(out/'fc-observer.jsonl').open('w');startup_trace=(out/'observer-startup.jsonl').open('w');capture_streams=[raw,trace,startup_trace]
        def verify_peer(sender):
            snapshot=inventory(sitl)
            if sender[0]!='127.0.0.1' or not exact_peer(snapshot,sender[1],monitor_port):raise RuntimeError('exact owned passive observer source unproven')
            return snapshot
        def consume(data,sender,at):
            nonlocal startup_bytes,valid_frames
            raw.write(data);raw.flush()
            for message in decoder.parse_buffer(data) or []:
                if message.get_type()=='BAD_DATA':
                    bad=bytes(message.data);prefix=b'0 '*3
                    if valid_frames or not bad or startup_bytes+len(bad)>6 or bad!=prefix[startup_bytes:startup_bytes+len(bad)] or message.reason!='Bad prefix':raise RuntimeError('invalid passive native startup/frame bytes')
                    startup_bytes+=len(bad);startup_trace.write(json.dumps({'mono':at,'raw_hex':bad.hex(),'reason':message.reason,'scope':'finite native SiK startup prefix, not observation evidence'})+'\n');startup_trace.flush();continue
                if startup_bytes not in (0,6):raise RuntimeError('incomplete native startup prefix')
                if message.get_srcSystem()!=1 or message.get_srcComponent()!=1:raise RuntimeError('foreign passive native source')
                valid_frames+=1
                if message.get_type() not in ('HEARTBEAT','GLOBAL_POSITION_INT','EXTENDED_SYS_STATE','SYS_STATUS','BATTERY_STATUS','GPS_RAW_INT','ATTITUDE','ATTITUDE_QUATERNION','POSITION_TARGET_LOCAL_NED','DISTANCE_SENSOR','AUTOPILOT_VERSION','STATUSTEXT'):continue
                row={'mono':at,'system':1,'component':1,**safe(message.to_dict())};packets.append(row);trace.write(json.dumps(row,allow_nan=False)+'\n');trace.flush()
        receiver=helpers['receiver'].PassiveReceiver(monitor,verify_peer,consume,max_datagrams=150000);receiver.start()
        with (out/'ownership.jsonl').open('w') as owner_trace:
            deadline=time.monotonic()+scenario['supervisor_s']
            while time.monotonic()<deadline:
                receiver.check();now=time.monotonic()
                if sitl.poll() is not None or antler.poll() is not None:raise RuntimeError('owned native or MOOS launcher exited')
                if now-last_check>=.5:
                    if owned is None:
                        try:owned,app_sockets=live_apps(ownership,antler,mission,bridge_port)
                        except RuntimeError:
                            if now>deadline-scenario['supervisor_s']+10:raise
                        else:result['owned_apps']=owned
                    else:owned,app_sockets=live_apps(ownership,antler,mission,bridge_port,owned)
                    if owned:owner_trace.write(json.dumps({'python_mono':now,'owned_apps':owned,'socket_inventory':app_sockets})+'\n');owner_trace.flush()
                    last_check=now
                for marker in MARKERS:helpers['boundary'].record_marker(control,marker,phase_times,result['events'],receiver)
                if probe.poll() is not None:
                    if drain is None:drain=time.monotonic()+1.5
                    if time.monotonic()>=drain:break
                time.sleep(.005)
        receiver.close();result['passive_receiver']=receiver.status();result['monitor_finished_python_mono']=time.monotonic()
        if receiver.peer is not None:
            result['ports']['monitor_peer']=receiver.peer[1];result['native_observer_socket_inventory']=receiver.peer_proof
        for handle in capture_streams:handle.close()
        if probe.poll() is None:raise RuntimeError('Copter scenario exceeded bounded lifecycle supervisor')
        result['probe_exit']=probe.returncode
        if probe.returncode:raise RuntimeError('Copter scenario probe failed')
        if owned is None:raise RuntimeError('owned app identity never proven')
        result['final_owned_apps'],result['final_app_socket_inventory']=live_apps(ownership,antler,mission,bridge_port,owned)
        events=[parse(line) for line in (out/'probe.jsonl').read_text().splitlines() if line.startswith('{')]
        result['probe_evidence']=audit_probe(events,args.scenario,firmware_proof)
        result['firmware_discovery']=result['probe_evidence']['firmware_discovery']
        observed=catalog.observed_catalog(out/'probe.jsonl',set(module_names),native=True,helm=True,mode='SITL_CONTROL',operations_helm=True)
        result['catalog']=observed;result['loaded_modules_sha256']=catalog.loaded_module_hashes(build/'plugins',observed)
        if set(phase_times)!=set(MARKERS):raise RuntimeError('independent phase brackets missing')
        result['hold_boundaries']=helpers['boundary'].audit_boundaries(result['events'],result['probe_evidence'],control)
        result['independent_fc']=audit_fc(packets,phase_times,result['monitor_finished_python_mono'],result['probe_evidence'])
        result['result']='PASS'
    except BaseException as error:result['error']=str(error) or type(error).__name__;result['traceback']=traceback.format_exc()
    finally:
        cleanup_errors=[]
        if receiver is not None:
            try:receiver.close();result['passive_receiver']=receiver.status()
            except BaseException as error:
                result['result']='FAIL';cleanup_errors.append(str(error) or type(error).__name__)
                try:result['passive_receiver']=receiver.status()
                except BaseException as status_error:result['passive_receiver']=None;cleanup_errors.append(str(status_error) or type(status_error).__name__)
        else:result['passive_receiver']={'not_started':True,'thread_retired':True,'socket_closed':False,'error':None}
        retired=result.get('passive_receiver')
        if receiver is not None and type(retired) is dict and retired.get('thread_retired') is True and receiver.peer is not None:
            result['ports']['monitor_peer']=receiver.peer[1];result['native_observer_socket_inventory']=receiver.peer_proof
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
                    try:stream.bind(('127.0.0.1',result['ports'][key]));result['ports_clear'][key]=True
                    except OSError:result['ports_clear'][key]=False
            except BaseException as error:result['ports_clear'][key]=None;cleanup_errors.append(str(error) or type(error).__name__)
        for key in ('db','base'):
            try:
                with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as stream:stream.settimeout(.2);result['ports_clear'][key]=stream.connect_ex(('127.0.0.1',result['ports'][key]))!=0
            except BaseException as error:result['ports_clear'][key]=None;cleanup_errors.append(str(error) or type(error).__name__)
        finalize_cleanup(result,remaining,processes_proven,cleanup_errors)
        try:
            logs=[];params={};arms=[];modes=[];wind=[]
            for path in sorted((out/'logs').glob('*.BIN')):
                digest=sha(path);reader=DFReader.DFReader_binary(str(path))
                while True:
                    record=reader.recv_match(type=['PARM','ARM','MODE'])
                    if record is None:break
                    row={'log':path.name,**safe(record.to_dict())}
                    if record.get_type()=='PARM':
                        name=record.Name.decode(errors='replace') if isinstance(record.Name,bytes) else str(record.Name)
                        if name in FIXTURE:params[name]=float(record.Value)
                        if name=='SIM_WIND_SPD':wind.append(float(record.Value))
                    elif record.get_type()=='ARM':arms.append(row)
                    else:modes.append(row)
                if sha(path)!=digest:raise RuntimeError('DataFlash changed during extraction')
                logs.append({'path':str(path.relative_to(out)),'sha256':digest})
            if not logs or not wind or set(wind)!={scenario['wind_speed_mps']} or not all(name in params and math.isclose(params[name],value,rel_tol=1e-5,abs_tol=1e-4) for name,value in FIXTURE.items()):raise RuntimeError('closed fixture/wind confirmation missing')
            result['dataflash']={'logs':logs,'fixture_parameter_readback':params,'wind_observed':sorted(set(wind)),'arm_records':arms,'mode_records':modes}
            dump(out/'dataflash-observations.json',{'arms':arms,'modes':modes,'fixture_parameters':params,'wind':wind})
            result['dataflash_evidence']=audit_dataflash(arms,modes,params,args.scenario)
        except BaseException as error:result['result']='FAIL';result['dataflash_error']=str(error)
        try:result['firmware_identity_after']=helpers['firmware'].verify_native(args.firmware_manifest,args.firmware_attestation,ap,expected_profile='arducopter')
        except BaseException as error:result['result']='FAIL';result['firmware_error']=str(error)
        result['artifacts_after']={str(path):sha(path) for path in protected};result['artifact_drift']=[path for path in before if before[path]!=result['artifacts_after'][path]]
        if result['artifact_drift']:result['result']='FAIL';result['artifact_error']='source/artifact changed during run'
        unfinalized=set() if receiver is None or (type(retired) is dict and retired.get('thread_retired') is True) else {'fc-observer.mavlink','fc-observer.jsonl','observer-startup.jsonl'}
        if unfinalized:result['unfinalized_capture_files']=sorted(unfinalized)
        result['files']={str(path.relative_to(out)):sha(path) for path in sorted(out.rglob('*')) if path.is_file() and path.name!='result.json' and path.name not in unfinalized}
        dump(out/'result.json',result)
    print(json.dumps(safe({key:result.get(key) for key in ('result','scenario','profile','error','dataflash_error','firmware_error','probe_exit','probe_evidence','independent_fc','dataflash_evidence','ports_clear','owned_processes_remaining','artifact_drift')}),indent=2))
    return int(result['result']!='PASS')

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('ardupilot-root','moos-root','build','behavior-dir','output'):parser.add_argument('--'+key,required=True)
    parser.add_argument('--source-root',required=True);parser.add_argument('--probe');parser.add_argument('--expected-app-sha256',required=True);parser.add_argument('--expected-probe-sha256',required=True)
    parser.add_argument('--firmware-manifest',required=True);parser.add_argument('--firmware-attestation',required=True)
    parser.add_argument('--scenario',choices=tuple(SCENARIOS),required=True);args=parser.parse_args()
    signal.signal(signal.SIGTERM,lambda *_:(_ for _ in ()).throw(SystemExit(143)))
    with (Path(tempfile.gettempdir())/'mavmoos-native-sitl.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        raise SystemExit(run(args))
