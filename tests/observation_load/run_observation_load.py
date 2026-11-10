#!/usr/bin/env python3
"""Owned native Copter: observers beside stock helm and bounded read-only services.

No relay, hardware route, mission execution, parameter mutation or observer writes.
The >30s bound covers GPS/EKF convergence, flight, one60s paced inventory and landing.
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
from observation_load_evidence import audit_probe,audit_fc,audit_attitude_consistency,audit_finish_boundary,parse

ROOT=Path(__file__).resolve().parents[2]
PIN='04cf6486e8d42ce138de816e8466244346ed77cb'
FIXTURE={'LOG_DISARMED':1,'SERIAL0_PROTOCOL':2,'SERIAL1_PROTOCOL':2,'BATT_MONITOR':4,'BATT_VOLT_PIN':13,'BATT_CURR_PIN':12,
         'BATT_VOLT_MULT':10.1,'BATT_AMP_PERVLT':17,'BATT_CAPACITY':3300,'SIM_BATT_VOLTAGE':12.6,'SIM_BATT_CAP_AH':0}
INTERVALS='1 500\n24 500\n30 200\n31 200\n33 100\n85 200\n147 1000\n245 500\n'

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
                          ('passive_receiver','tests/observation_load/passive_receiver.py'),
                          ('scenario_fixtures','scripts/firmware/scenario_fixtures.py')):
        path=(source/relative).resolve();spec=importlib.util.spec_from_file_location('_observation_load_'+role,path)
        if spec is None or spec.loader is None:raise RuntimeError('source helper unavailable: '+str(path))
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);result[role]=module
    return result
def free_port(kind,address='127.0.0.1'):
    with socket.socket(socket.AF_INET,kind) as stream:stream.bind((address,0));return stream.getsockname()[1]
def free_rc_port():
    # AP's pinned UDP RC backend binds INADDR_ANY, not the MAVLink loopback
    # address. Availability has the same scope; release-to-native-bind still
    # has a residual race, checked separately by the owned native inventory.
    return free_port(socket.SOCK_DGRAM,'0.0.0.0')
def inventory(process):
    if process.poll() is not None:raise RuntimeError('owned native exited')
    result=subprocess.run(['lsof','-nP','-a','-p',str(process.pid),'-iUDP'],capture_output=True,text=True,timeout=3)
    if result.returncode:raise RuntimeError('native socket inventory unavailable')
    return result.stdout
def exact_peer(text,source,destination):return bool(re.search(r'UDP\s+127\.0\.0\.1:'+str(source)+r'->127\.0\.0\.1:'+str(destination)+r'\b',text))
def exact_rc_input(text,port):
    return bool(re.search(r'UDP\s+(?:\*|0\.0\.0\.0):'+str(port)+r'(?:\s|$)',text))
def wait_rc_input(process,port,deadline):
    # This is readiness of the exact native PID's selected input socket, not
    # proof of RC traffic. It neither supplies pilot input nor renews startup.
    while time.monotonic()<deadline:
        snapshot=inventory(process);observed=time.monotonic()
        if observed>=deadline:break
        if exact_rc_input(snapshot,port):return snapshot,observed
        time.sleep(min(.02,max(0,deadline-time.monotonic())))
    raise RuntimeError('owned native RC input not bound within original startup deadline')
def record_marker(control,key,markers,events):
    if key in markers or not (control/key).exists():return
    if key=='load.finished' and (control/'load.finished.ack').exists():raise RuntimeError('preexisting finish acknowledgement')
    observed=time.monotonic();markers[key]=observed;events.append({'kind':key,'python_mono':observed})
    if key=='load.finished':
        token=(control/key).read_bytes()
        if not 0<len(token)<=128:raise RuntimeError('finish marker token invalid')
        temporary=control/'load.finished.ack.tmp'
        with temporary.open('xb') as output:output.write(token)
        # Link a complete file into an absent destination. This is atomic and
        # refuses to replace a stale/foreign acknowledgement that races us.
        os.link(temporary,control/'load.finished.ack');temporary.unlink()
        events.append({'kind':'load.finished.ack','python_mono':time.monotonic(),'token_sha256':hashlib.sha256(token).hexdigest()})
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

def run(args):
    from pymavlink import mavutil,DFReader
    ap,moos,build,out=[Path(getattr(args,key)).resolve() for key in ('ardupilot_root','moos_root','build','output')]
    source=Path(args.source_root).resolve() if args.source_root else ROOT
    behavior=Path(args.behavior_dir).resolve()
    probe_path=Path(args.probe).resolve() if args.probe else build/'bin/observation_load_probe'
    helpers=source_helpers(source);ownership,catalog=helpers['ownership'],helpers['catalog']
    firmware=ownership.native_firmware_profile(source,ap)
    selector=helpers['scenario_fixtures'];selected_fixture=selector.select(source,firmware,'armed_wiped_mission_read')
    module_names=['telemetry','system_status','parameters','missions','arducopter_control']+(['attitude'] if args.profile=='attitude' else [])
    native,app=ap/'build/sitl/bin/arducopter',build/'bin/iMAVLink'
    protected=[native,app,probe_path,moos/'bin/pAntler',moos/'bin/pHelmIvP',Path(__file__).resolve(),Path(__file__).with_name('ObservationLoadProbe.cpp'),Path(__file__).with_name('FinishBoundary.h'),Path(__file__).with_name('observation_load_evidence.py')]
    protected += [build/'plugins'/('mavmoos_'+name+'.so') for name in module_names]
    protected += [Path(m.__file__) for m in helpers.values()]
    protected += [source/name for name in ('src/libmavmoos_runtime/Plugin.h','src/libmavmoos_runtime/Runtime.cpp','src/libmavmoos_runtime/RuntimeMission.cpp','src/libmavmoos_rx/UdpReceiver.cpp','src/iMAVLink/MAVLink.cpp')]
    protected += [source/'src/plugins'/name for name in ['SystemStatusPlugin.cpp','TelemetryPlugin.cpp']+(['AttitudePlugin.cpp'] if args.profile=='attitude' else [])]
    protected += [source/'missions/copter_sitl'/('meta_vehicle.'+suffix) for suffix in ('moos','bhv')]
    protected += [ownership.arducopter_gcs_source(ap)]
    protected += selector.protected_paths(selected_fixture)+[source/'tests/firmware/MissionFixtureShape.h']
    protected += [ap/name for name in ('Tools/autotest/default_params/copter.parm','libraries/SITL/SIM_Frame.cpp','libraries/SITL/SIM_Battery.cpp')]
    for path in protected:
        if not path.is_file():raise RuntimeError('required protected input missing '+str(path))
    if revision(ap)!=(firmware['source_revision'] if firmware else PIN):raise RuntimeError('native source is not pinned')
    if args.expected_app_sha256 and sha(app)!=args.expected_app_sha256:raise RuntimeError('frozen app hash differs')
    protected += ownership.firmware_source_inputs(source,firmware)
    before={str(path):sha(path) for path in protected}
    out.mkdir(parents=True,exist_ok=False);control=out/'control';control.mkdir();(out/'logs').mkdir()
    fixture_file=selector.copy_to_evidence(selected_fixture,out)
    bridge=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);bridge.bind(('127.0.0.1',0));bridge.setblocking(False)
    monitor=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);monitor.bind(('127.0.0.1',0));monitor.setblocking(False)
    bridge_port,monitor_port=bridge.getsockname()[1],monitor.getsockname()[1]
    db,base,rc=free_port(socket.SOCK_STREAM),free_port(socket.SOCK_STREAM),free_rc_port()
    while base==db:base=free_port(socket.SOCK_STREAM)
    defaults=out/'fixture.parm';defaults.write_text(''.join(f'{key} {value}\n' for key,value in FIXTURE.items()))
    (out/'message-intervals-chan1.txt').write_text(INTERVALS)
    command=[str(native),'--model','quad','--home','42.358456,-71.087589,10,0','--speedup','1','--wipe',
        '--defaults',str(ap/'Tools/autotest/default_params/copter.parm')+','+str(defaults),
        '--serial0',f'udpclient:127.0.0.1:{bridge_port}','--serial1',f'udpclient:127.0.0.1:{monitor_port}',
        '--serial2','none','--serial5','none','--serial6','none','--serial7','none','--serial8','none',
        '--base-port',str(base),'--rc-in-port',str(rc)]
    result={'result':'FAIL','scope':'owned native Copter observers + unchanged stock helm + independent read services','profile':args.profile,'hardware':False,'firmware_identity':firmware,'timewarp':1,
        'command':command,'modules_requested':module_names,'source_root':str(source),'source_ref':revision(source),'ardupilot_ref':revision(ap),'moos_ref':revision(moos),
        'ports':{'bridge':bridge_port,'monitor':monitor_port,'db':db,'base':base,'rc':rc},'observer_writes':0,'rc_input_sent':False,
        'mission_execution_requested':False,'parameter_write_requested':False,'fixture_parameters':FIXTURE,'interval_file_units':'milliseconds',
        'scenario_fixture':selected_fixture,'scenario_fixture_file':fixture_file,
        'host_interval_request_evidence':'descriptor intent + unit-layer encoder tests separately; primary uplink packets/ACKs not captured here',
        'loaded_helpers':{role:{'path':m.__file__,'sha256':sha(m.__file__)} for role,m in helpers.items()},'artifacts_before':before,
        'app_sha256':before[str(app)],'native_sha256':before[str(native)],'probe_sha256':before[str(probe_path)],'events':[]}
    processes=[];streams=[];packets=[];antler=sitl=probe=None;owned=None;mission=out/'targ_copter.moos'
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
        rc_sockets,rc_observed=wait_rc_input(sitl,rc,deadline)
        result['native_rc_socket_inventory']=rc_sockets;result['rc_input_bound_at_startup']=True
        result['rc_input_readiness_python_mono']=rc_observed
        result['rc_port_selection_scope']='temporary IPv4 INADDR_ANY availability bind; released before native startup; exact owned native bind checked before MOOS/probe'
        env=dict(os.environ);env['PATH']=os.pathsep.join([str(build/'bin'),str(moos/'bin'),env.get('PATH','')]);env['IVP_BEHAVIOR_DIRS']=str(behavior)
        macros={'MOOS_PORT':db,'APP_PATH':app,'BIND_PORT':bridge_port,'PEER_PORT':peer[1],'LAT_ORIGIN':'42.358456','LONG_ORIGIN':'-71.087589',
                'SIMULATION_ONLY':'true','SHADOW_MODE':'false','SITL_PID':sitl.pid,'PLUGIN_DIRECTORY':build/'plugins','BEHAVIOR_DIRECTORY':behavior,
                'HEIGHT':5,'SPEED':1,'WAYPOINTS':'100,0:100,100:0,100:0,0'}
        for suffix in ('moos','bhv'):
            argv=[str(moos/'bin/nsplug'),str(source/f'missions/copter_sitl/meta_vehicle.{suffix}'),str(out/f'targ_copter.{suffix}'),'--strict','--force','-x']+[f'{k}={v}' for k,v in macros.items()]
            completed=subprocess.run(argv,capture_output=True,text=True,env=env);(out/('nsplug-'+suffix+'.log')).write_text(completed.stdout+completed.stderr)
            if completed.returncode:raise RuntimeError('stock mission generation failed '+suffix)
        text=mission.read_text()
        for before_text,after_text in [('  AppTick = 30','  AppTick = 50'),('  CommsTick = 30','  CommsTick = 50'),
            ('  nav_position_timeout = 0.5','  nav_position_timeout = 0.5\n  plugins = '+','.join(module_names)+'\n  max_datagrams_per_iter = 128\n  parameter_list_mode = paced\n  parameter_list_timeout = 60\n  mission_timeout = 15')]:
            if text.count(before_text)!=1:raise RuntimeError('stock template anchor drift '+before_text)
            text=text.replace(before_text,after_text,1)
        mission.write_text(text);result['mission_sha256']=sha(mission);result['behavior_sha256']=sha(out/'targ_copter.bhv')
        antler=launch([str(moos/'bin/pAntler'),str(mission)],'pantler.log',env)
        probe=launch([str(probe_path),str(db),str(control),selected_fixture['expectation']['shape']],'probe.jsonl',env)
        decoder=mavutil.mavlink.MAVLink(None);decoder.robust_parsing=True;markers={};drain=None;last_check=-1
        with (out/'fc-observer.mavlink').open('wb') as raw,(out/'fc-observer.jsonl').open('w') as trace,(out/'ownership.jsonl').open('w') as owner_trace:
            def prove_monitor_peer(sender):
                snapshot=inventory(sitl)
                if sender[0]!='127.0.0.1' or not exact_peer(snapshot,sender[1],monitor_port):raise RuntimeError('exact owned passive observer source unproven')
                return snapshot
            def record_datagram(data,sender,at):
                raw.write(data)
                for message in decoder.parse_buffer(data) or []:
                    if message.get_srcSystem()!=1 or message.get_srcComponent()!=1:continue
                    if message.get_type() not in ('HEARTBEAT','GLOBAL_POSITION_INT','EXTENDED_SYS_STATE','SYS_STATUS','BATTERY_STATUS','GPS_RAW_INT','ATTITUDE','ATTITUDE_QUATERNION','POSITION_TARGET_LOCAL_NED','STATUSTEXT'):continue
                    row={'mono':at,'system':1,'component':1,**safe(message.to_dict())};packets.append(row);trace.write(json.dumps(row,allow_nan=False)+'\n')
                trace.flush()
            receiver=helpers['passive_receiver'].PassiveReceiver(monitor,prove_monitor_peer,record_datagram,max_datagrams=50000)
            try:
                deadline=time.monotonic()+250
                receiver.start()
                while time.monotonic()<deadline:
                    receiver.check()
                    now=time.monotonic()
                    if sitl.poll() is not None or antler.poll() is not None:raise RuntimeError('owned native or MOOS launcher exited')
                    if now-last_check>=.5:
                        if owned is None:
                            try:owned,app_sockets=live_apps(ownership,antler,mission,bridge_port)
                            except RuntimeError:
                                if now>deadline-240:raise
                            else:result['owned_apps']=owned
                        else:owned,app_sockets=live_apps(ownership,antler,mission,bridge_port,owned)
                        if owned:owner_trace.write(json.dumps({'python_mono':now,'owned_apps':owned,'socket_inventory':app_sockets})+'\n');owner_trace.flush()
                        last_check=now
                    receiver.check()
                    for marker in ('load.started','reads.completed','load.finished'):
                        record_marker(control,marker,markers,result['events'])
                    if probe.poll() is not None:
                        if drain is None:drain=time.monotonic()+1.5
                        if time.monotonic()>=drain:break
                    time.sleep(.005)
                receiver.check()
            finally:
                active_error=sys.exc_info()[0] is not None
                try:receiver.close(timeout=4)
                except BaseException as error:
                    result['passive_receiver_retirement_error']=str(error)
                    if not active_error:raise
                finally:
                    result['passive_receiver']=receiver.status()
                    if receiver.peer is not None:
                        result['ports']['monitor_peer']=receiver.peer[1]
                        result['native_observer_socket_inventory']=receiver.peer_proof
            result['monitor_finished_python_mono']=time.monotonic()
        if probe.poll() is None:raise RuntimeError('observation load exceeded250s supervisor')
        result['probe_exit']=probe.returncode
        if probe.returncode:raise RuntimeError('observation load probe failed')
        if owned is None:raise RuntimeError('owned app identity never proven')
        result['final_owned_apps'],result['final_app_socket_inventory']=live_apps(ownership,antler,mission,bridge_port,owned)
        events=[parse(line) for line in (out/'probe.jsonl').read_text().splitlines() if line.startswith('{')]
        expected_fixture=selector.reselect((out/fixture_file['path']).read_bytes(),firmware,'armed_wiped_mission_read',selected_fixture)
        result['probe_evidence']=audit_probe(events,args.profile,expected_fixture)
        result['finish_boundary']=audit_finish_boundary(events,result['events'],(control/'load.finished').read_bytes(),(control/'load.finished.ack').read_bytes())
        observed=catalog.observed_catalog(out/'probe.jsonl',set(module_names),native=True,helm=True,mode='SITL_CONTROL')
        result['catalog']=observed;result['loaded_modules_sha256']=catalog.loaded_module_hashes(build/'plugins',observed)
        if set(markers)!=set(('load.started','reads.completed','load.finished')):raise RuntimeError('independent phase brackets missing')
        result['independent_fc']=audit_fc(packets,markers['load.started'],markers['reads.completed'],markers['load.finished'],result['monitor_finished_python_mono'],args.profile)
        pw=result['probe_evidence'];result['attitude_consistency']=audit_attitude_consistency(events,packets,(pw['load_window_probe_elapsed'][0],pw['read_end_probe_elapsed'],pw['load_window_probe_elapsed'][1]),args.profile)
        result['result']='PASS'
    except BaseException as error:result['error']=str(error) or type(error).__name__;result['traceback']=traceback.format_exc()
    finally:
        remaining=[]
        for process in reversed(processes):
            try:remaining.extend(ownership.stop_process_tree(process))
            except BaseException as error:result['result']='FAIL';result.setdefault('cleanup_exceptions',[]).append(str(error))
        bridge.close();monitor.close()
        for handle in streams:handle.close()
        result['ports_clear']={};result['owned_processes_remaining']=remaining
        for key in ('bridge','monitor','peer','monitor_peer','rc'):
            if key not in result['ports']:continue
            with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as stream:
                try:stream.bind(('127.0.0.1',result['ports'][key]));result['ports_clear'][key]=True
                except OSError:result['ports_clear'][key]=False
        for key in ('db','base'):
            with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as stream:stream.settimeout(.2);result['ports_clear'][key]=stream.connect_ex(('127.0.0.1',result['ports'][key]))!=0
        if remaining or not all(result['ports_clear'].values()):result['result']='FAIL';result['cleanup_error']='owned resources remain'
        try:
            logs=[];wind=[];params={};arms=[];modes=[];battery=[]
            for path in sorted((out/'logs').glob('*.BIN')):
                digest=sha(path);reader=DFReader.DFReader_binary(str(path))
                while True:
                    record=reader.recv_match(type=['PARM','ARM','MODE','BAT'])
                    if record is None:break
                    row={'log':path.name,**safe(record.to_dict())}
                    if record.get_type()=='PARM':
                        name=record.Name.decode(errors='replace') if isinstance(record.Name,bytes) else str(record.Name)
                        if name in FIXTURE:params[name]=float(record.Value)
                        if name=='SIM_WIND_SPD':wind.append(float(record.Value))
                    elif record.get_type()=='ARM':arms.append(row)
                    elif record.get_type()=='MODE':modes.append(row)
                    else:battery.append(row)
                if sha(path)!=digest:raise RuntimeError('DataFlash changed during extraction')
                logs.append({'path':str(path.relative_to(out)),'sha256':digest})
            if not logs or not wind or len(set(wind))!=1 or not all(name in params and math.isclose(params[name],value,rel_tol=1e-5,abs_tol=1e-4) for name,value in FIXTURE.items()):raise RuntimeError('DataFlash fixture/wind confirmation missing')
            if not any(int(r['ArmState'])==1 for r in arms) or not arms or int(arms[-1]['ArmState'])!=0 or any(int(r.get('Forced',0))!=0 for r in arms):raise RuntimeError('DataFlash ordinary arm/disarm not confirmed')
            if not {4,9}.issubset({int(r['ModeNum']) for r in modes}) or not battery or not any(r['Inst']==0 and 12.3<=r['Volt']<=12.9 for r in battery):raise RuntimeError('DataFlash Guided/Land/battery confirmation missing')
            result['dataflash']={'logs':logs,'fixture_parameter_readback':params,'wind_observed':sorted(set(wind)),'arm_records':arms,'mode_records':modes,'battery_samples':len(battery)}
            dump(out/'dataflash-observations.json',{'arms':arms,'modes':modes,'battery':battery,'fixture_parameters':params,'wind':wind})
        except BaseException as error:result['result']='FAIL';result['dataflash_error']=str(error)
        result['artifacts_after']={str(path):sha(path) for path in protected};result['artifact_drift']=[path for path in before if before[path]!=result['artifacts_after'][path]]
        if result['artifact_drift']:result['result']='FAIL';result['artifact_error']='source/artifact changed during run'
        if sha(out/fixture_file['path'])!=fixture_file['sha256']:result['result']='FAIL';result['scenario_fixture_error']='copied declaration changed'
        result['files']={str(path.relative_to(out)):sha(path) for path in sorted(out.rglob('*')) if path.is_file() and path.name!='result.json'}
        dump(out/'result.json',result)
    print(json.dumps(safe({key:result.get(key) for key in ('result','profile','error','dataflash_error','probe_exit','probe_evidence','independent_fc','attitude_consistency','ports_clear','owned_processes_remaining','artifact_drift')}),indent=2))
    return int(result['result']!='PASS')

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('ardupilot-root','moos-root','build','behavior-dir','output'):parser.add_argument('--'+key,required=True)
    parser.add_argument('--source-root');parser.add_argument('--probe');parser.add_argument('--expected-app-sha256')
    parser.add_argument('--profile',choices=('status','attitude'),default='attitude');args=parser.parse_args()
    signal.signal(signal.SIGTERM,lambda *_:(_ for _ in ()).throw(SystemExit(143)))
    with (Path(tempfile.gettempdir())/'mavmoos-native-sitl.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        raise SystemExit(run(args))
