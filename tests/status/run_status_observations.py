#!/usr/bin/env python3
"""Owned RX_ONLY native-Copter observation bench, with independently observed app egress.

Native serial0 -> owned RX-only ingress relay -> iMAVLink. Any app-origin UDP is
logged, never forwarded to native, and fails the run. Serial1 has an independent
passive observer. There are no MAVLink/RC requests, arming or flight operations.
Discovery is bounded20s, the probe25s and its supervisor40s; common native lock.
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

ROOT=Path(__file__).resolve().parents[2]
from StatusObservationAudit import audit,parse
owned_records=stop_process_tree=None

PIN='04cf6486e8d42ce138de816e8466244346ed77cb'
FIXTURE={'LOG_DISARMED':1,'SERIAL0_PROTOCOL':2,'SERIAL1_PROTOCOL':2,
         'BATT_MONITOR':4,'BATT_VOLT_PIN':13,'BATT_CURR_PIN':12,
         'BATT_VOLT_MULT':10.1,'BATT_AMP_PERVLT':17,'BATT_CAPACITY':3300,
         'SIM_BATT_VOLTAGE':12.6,'SIM_BATT_CAP_AH':0}
INTERVALS='1 500\n24 500\n147 1000\n33 500\n245 500\n'

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def ref(path):return subprocess.check_output(['git','-C',str(path),'rev-parse','HEAD'],text=True).strip()
def free_port(kind):
    with socket.socket(socket.AF_INET,kind) as stream:
        stream.bind(('127.0.0.1',0));return stream.getsockname()[1]
def safe(value):
    if isinstance(value,float) and not math.isfinite(value):return {'nonfinite':str(value)}
    if isinstance(value,bytes):return {'raw_hex':value.hex()}
    if isinstance(value,dict):return {key:safe(item) for key,item in value.items()}
    if isinstance(value,(list,tuple)):return [safe(item) for item in value]
    return value
def write(stream,row):stream.write(json.dumps(safe(row),allow_nan=False)+'\n');stream.flush()
def sockets(process):
    if process.poll() is not None:raise RuntimeError('owned process exited before socket inventory')
    result=subprocess.run(['lsof','-nP','-a','-p',str(process.pid),'-iUDP'],capture_output=True,text=True,timeout=3)
    if result.returncode:raise RuntimeError('owned process UDP inventory unavailable')
    return result.stdout
def exact_peer(inventory,source,destination):
    return bool(re.search(r'UDP\s+127\.0\.0\.1:'+str(source)+r'->127\.0\.0\.1:'+str(destination)+r'\b',inventory))
def source_helpers(source):
    # Resolve against the explicitly described build source, not this runner's
    # worktree or a cached `control.catalog_evidence` from another ABI revision.
    result={}
    for role,relative in (('ownership','tests/owned_processes.py'),('catalog','tests/control/catalog_evidence.py')):
        path=(source/relative).resolve();spec=importlib.util.spec_from_file_location('_mavmoos_status_'+role,path)
        if spec is None or spec.loader is None:raise RuntimeError('cannot load source helper '+str(path))
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);result[role]=module
    return result
def exact_app(antler,mission):
    found=[]
    for record in owned_records(antler):
        try:tokens=shlex.split(record['command'])
        except ValueError:continue
        if tokens and Path(tokens[0]).name=='iMAVLink' and str(mission) in tokens:found.append(record)
    if len(found)!=1:raise RuntimeError('exact owned mission app not unique')
    return found[0]

def live_app(antler,mission,bridge,expected=None):
    record=exact_app(antler,mission)
    if expected and (record['pid']!=expected['pid'] or record['command']!=expected['command']):raise RuntimeError('owned app identity changed')
    result=subprocess.run(['lsof','-nP','-a','-p',str(record['pid']),'-iUDP'],capture_output=True,text=True,timeout=3)
    if result.returncode or not re.search(r'UDP\s+127\.0\.0\.1:'+str(bridge)+r'\b',result.stdout):raise RuntimeError('owned app bound socket no longer live')
    return record,result.stdout

def observed_battery_zero(packets,end):
    samples=[]
    for packet in packets:
        if packet['mavpackettype']!='BATTERY_STATUS' or packet.get('id')!=0 or packet['received_mono']<end-5:continue
        base=list(packet.get('voltages',[]));extended=list(packet.get('voltages_ext',[]));total=0;count=0;gap=False;layout=True
        for index,raw in enumerate(base+extended):
            if type(raw) is not int or not 0<=raw<=65535:raise RuntimeError('invalid independent voltage slot')
            known=raw!=65535 and (index<len(base) or raw!=0)
            if known:
                if gap:layout=False
                total+=raw/1000.;count+=1
            else:gap=True
        if layout and count and 12.3<=total<=12.9:samples.append({'received_mono':packet['received_mono'],'battery_id':0,'reported_voltage_sum_v':total})
    if len({row['received_mono'] for row in samples})<2:raise RuntimeError('independent late battery0 reports do not confirm configured12.6V fixture')
    return samples

def run(args):
    global owned_records,stop_process_tree
    from pymavlink import mavutil,DFReader
    ap,moos,build,out=[Path(getattr(args,key)).resolve() for key in ('ardupilot_root','moos_root','build','output')]
    source=Path(args.source_root).resolve() if args.source_root else ROOT
    helpers=source_helpers(source)
    firmware=helpers['ownership'].native_firmware_profile(source,ap)
    owned_records,stop_process_tree=helpers['ownership'].owned_records,helpers['ownership'].stop_process_tree
    observed_catalog,loaded_module_hashes=helpers['catalog'].observed_catalog,helpers['catalog'].loaded_module_hashes
    native,app,plugins=ap/'build/sitl/bin/arducopter',build/'bin/iMAVLink',build/'plugins'
    probe_path=Path(args.probe).resolve() if args.probe else build/'bin/status_moos_probe'
    artifact_paths=[native,app,probe_path,plugins/'mavmoos_telemetry.so',plugins/'mavmoos_system_status.so',moos/'bin/pAntler']
    for path in artifact_paths:
        if not path.is_file():raise RuntimeError('required artifact missing: '+str(path))
    if ref(ap)!=(firmware['source_revision'] if firmware else PIN):raise RuntimeError('native source checkout not pinned')
    if args.expected_app_sha256 and sha(app)!=args.expected_app_sha256:raise RuntimeError('frozen app hash mismatch')
    if args.expected_status_sha256 and sha(plugins/'mavmoos_system_status.so')!=args.expected_status_sha256:raise RuntimeError('frozen status module hash mismatch')
    out.mkdir(parents=True,exist_ok=False)
    relay=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);relay.bind(('127.0.0.1',0));relay.setblocking(False)
    monitor=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);monitor.bind(('127.0.0.1',0));monitor.setblocking(False)
    # Keep this port reserved until app launch, so native UDP source allocation
    # cannot accidentally choose the address classified as app egress.
    bridge_reservation=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);bridge_reservation.bind(('127.0.0.1',0))
    bridge,relay_port,monitor_port=bridge_reservation.getsockname()[1],relay.getsockname()[1],monitor.getsockname()[1]
    db,base,rc=free_port(socket.SOCK_STREAM),free_port(socket.SOCK_STREAM),free_port(socket.SOCK_DGRAM)
    while base==db:base=free_port(socket.SOCK_STREAM)
    defaults=out/'fixture.parm';defaults.write_text(''.join(f'{key} {value}\n' for key,value in FIXTURE.items()))
    for channel in (0,1):(out/f'message-intervals-chan{channel}.txt').write_text(INTERVALS)
    command=[str(native),'--model','quad','--home','42.358456,-71.087589,10,0','--speedup','1','--wipe',
             '--defaults',str(ap/'Tools/autotest/default_params/copter.parm')+','+str(defaults),
             '--serial0',f'udpclient:127.0.0.1:{relay_port}','--serial1',f'udpclient:127.0.0.1:{monitor_port}',
             '--serial2','none','--serial5','none','--serial6','none','--serial7','none','--serial8','none',
             '--base-port',str(base),'--rc-in-port',str(rc)]
    protected=artifact_paths+[Path(__file__).resolve(),Path(__file__).with_name('StatusObservationAudit.py'),
        source/'tests/owned_processes.py',source/'tests/control/catalog_evidence.py']
    protected += [source/name for name in ('src/plugins/SystemStatusPlugin.cpp','src/plugins/TelemetryPlugin.cpp',
        'src/libmavmoos_runtime/Plugin.h','src/libmavmoos_runtime/PluginHost.cpp','src/libmavmoos_runtime/Runtime.cpp',
        'src/iMAVLink/MAVLink.cpp','src/libmavmoos_rx/UdpReceiver.cpp')]
    protected += [ap/name for name in ('Tools/autotest/default_params/copter.parm','libraries/SITL/SITL.cpp',
        'libraries/AP_HAL_SITL/AnalogIn.cpp','libraries/AP_HAL_SITL/SITL_State_common.cpp',
        'libraries/AP_BattMonitor/AP_BattMonitor_Analog.cpp','libraries/GCS_MAVLink/GCS_Common.cpp')]
    protected += helpers['ownership'].firmware_source_inputs(source,firmware)
    before={str(path):sha(path) for path in protected}
    result={'result':'FAIL','scope':'owned disarmed RX_ONLY native Copter; observation provider only','hardware':False,'firmware_identity':firmware,
        'timewarp':1,'command':command,'runner_pid':os.getpid(),'ports':{'bridge':bridge,'relay':relay_port,'monitor':monitor_port,'db':db,'base':base,'rc':rc},
        'topology':'native serial0 -> owned receive-only ingress relay -> app; native serial1 -> separate passive observer',
        'observer_writes':0,'relay_forwards_to_native':0,'relay_forwards_to_app':0,'app_origin_datagrams':0,
        'arming_requested':False,'rc_input_sent':False,'source_root':str(source),'source_ref':ref(source),
        'ardupilot_ref':ref(ap),'moos_ref':ref(moos),'fixture_parameters':FIXTURE,'interval_file_units':'milliseconds',
        'loaded_helpers':{role:{'path':module.__file__,'sha256':sha(module.__file__)} for role,module in helpers.items()},
        'artifacts_before':before,'app_sha256':before[str(app)],'status_module_sha256':before[str(plugins/'mavmoos_system_status.so')],
        'native_sha256':before[str(native)],'probe_sha256':before[str(probe_path)]}
    processes=[];streams=[];packets=[];native_peer=None;monitor_peer=None;forward=False;total_bytes=0;datagrams=0
    sitl=antler=probe=None
    def launch(argv,log,env=None):
        stream=(out/log).open('w');streams.append(stream)
        process=subprocess.Popen(argv,cwd=out,env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(process);return process
    try:
        wire=(out/'wire-datagrams.jsonl').open('w');streams.append(wire)
        trace=(out/'observer.jsonl').open('w');streams.append(trace)
        ownership=(out/'app-ownership.jsonl').open('w');streams.append(ownership)
        primary_raw=(out/'primary.mavlink').open('wb');streams.append(primary_raw)
        observer_raw=(out/'observer.mavlink').open('wb');streams.append(observer_raw)
        decoders={name:mavutil.mavlink.MAVLink(None) for name in ('primary','monitor')}
        for decoder in decoders.values():decoder.robust_parsing=True
        sitl=launch(command,'sitl.log');result['sitl_pid']=sitl.pid
        def pump():
            nonlocal native_peer,monitor_peer,total_bytes,datagrams
            for name,stream in (('primary',relay),('monitor',monitor)):
                for _ in range(128):
                    try:data,sender=stream.recvfrom(65535)
                    except BlockingIOError:break
                    at=time.monotonic();datagrams+=1;total_bytes+=len(data)
                    write(wire,{'mono':at,'role':name,'source':list(sender),'destination':['127.0.0.1',stream.getsockname()[1]],'raw_hex':data.hex()})
                    if datagrams>20000 or total_bytes>16*1024*1024:raise RuntimeError('native wire evidence bound exceeded')
                    if name=='primary' and sender==('127.0.0.1',bridge):
                        result['app_origin_datagrams']+=1
                        # This is deliberately never forwarded to the FC.
                        raise RuntimeError('independent relay received app-origin UDP in RX_ONLY')
                    if sender[0]!='127.0.0.1':raise RuntimeError('non-loopback observation source')
                    if name=='primary':
                        if native_peer is None:
                            inventory=sockets(sitl)
                            if not exact_peer(inventory,sender[1],relay_port):raise RuntimeError('primary source socket not owned by exact native process')
                            native_peer=sender;result['native_primary_peer']=list(sender);result['ports']['peer']=sender[1];result['native_primary_socket_inventory']=inventory
                        elif sender!=native_peer:raise RuntimeError('primary source endpoint changed')
                        primary_raw.write(data)
                        if forward:
                            if relay.sendto(data,('127.0.0.1',bridge))!=len(data):raise RuntimeError('relay forward to app not locally accepted')
                            result['relay_forwards_to_app']+=1
                            write(wire,{'mono':time.monotonic(),'role':'relay_to_app','source':['127.0.0.1',relay_port],'destination':['127.0.0.1',bridge],'raw_hex':data.hex()})
                    else:
                        if monitor_peer is None:
                            inventory=sockets(sitl)
                            if not exact_peer(inventory,sender[1],monitor_port):raise RuntimeError('observer source socket not owned by exact native process')
                            monitor_peer=sender;result['native_monitor_peer']=list(sender);result['ports']['monitor_peer']=sender[1];result['native_monitor_socket_inventory']=inventory
                        elif sender!=monitor_peer:raise RuntimeError('observer source endpoint changed')
                        observer_raw.write(data)
                    for message in decoders[name].parse_buffer(data) or []:
                        if message.get_type()=='HEARTBEAT' and message.get_srcSystem()==1 and message.get_srcComponent()==1:
                            if message.base_mode&128:raise RuntimeError('independent wire observation saw arming')
                            if message.type!=2 or message.autopilot!=3:raise RuntimeError('native heartbeat identity changed')
                        if name=='monitor' and message.get_srcSystem()==1 and message.get_srcComponent()==1:
                            row={'received_mono':at,**message.to_dict()};packets.append(row);write(trace,row)
        deadline=time.monotonic()+20
        while time.monotonic()<deadline:
            if sitl.poll() is not None:raise RuntimeError('native process exited during discovery')
            pump()
            if native_peer and any(p['mavpackettype']=='HEARTBEAT' for p in packets):break
            time.sleep(.005)
        if native_peer is None or not any(p['mavpackettype']=='HEARTBEAT' for p in packets):raise RuntimeError('owned native primary/observer discovery deadline')
        mission=out/'status_observations.moos'
        mission.write_text(f'''ServerHost = localhost
ServerPort = {db}
Community = status_observations
MOOSTimeWarp = 1
TERM_REPORTING = false
ProcessConfig = ANTLER
{{
  MSBetweenLaunches = 50
  Run = MOOSDB @ NewConsole = false
  Run = iMAVLink @ NewConsole = false
}}
ProcessConfig = iMAVLink
{{
  AppTick = 50
  CommsTick = 50
  bind_address = 127.0.0.1
  bind_port = {bridge}
  peer_address = 127.0.0.1
  peer_port = {relay_port}
  target_system = 1
  target_component = 1
  expected_type = 2
  expected_autopilot = 3
  simulation_only = false
  shadow_mode = false
  plugins = telemetry,system_status
  plugin_directory = {plugins}
  max_datagrams_per_iter = 128
  status_timeout = 3
  status_publish_interval = 0.5
  status_text_timeout = 2
}}
''')
        env=dict(os.environ);env['PATH']=os.pathsep.join([str(build/'bin'),str(moos/'bin'),env.get('PATH','')])
        bridge_reservation.close()
        antler=launch([str(moos/'bin/pAntler'),str(mission)],'pantler.log',env)
        probe=launch([str(probe_path),str(db),'25'],'probe.jsonl',env);forward=True
        deadline=time.monotonic()+40;drain=None;next_app_check=0
        while time.monotonic()<deadline:
            if sitl.poll() is not None or antler.poll() is not None:raise RuntimeError('owned native or MOOS launcher exited')
            pump()
            if time.monotonic()>=next_app_check:
                try:record,inventory=live_app(antler,mission,bridge,result.get('owned_app'))
                except RuntimeError:
                    if 'owned_app' in result:raise
                else:
                    result['owned_app']=record;result['app_socket_inventory']=inventory
                    write(ownership,{'verified_mono':time.monotonic(),'owned_process':record,'udp_inventory':inventory})
                next_app_check=time.monotonic()+.5
            if probe.poll() is not None:
                if drain is None:drain=time.monotonic()+1.2
                if time.monotonic()>=drain:break
            time.sleep(.005)
        result['monitor_finished_mono']=time.monotonic();result['wire_datagrams']=datagrams;result['wire_bytes']=total_bytes
        if probe.poll() is None:raise RuntimeError('status probe exceeded40s supervisor')
        result['probe_exit']=probe.returncode
        if probe.returncode:raise RuntimeError('status subscriber failed')
        if 'owned_app' not in result:raise RuntimeError('app bound socket ownership unproven')
        record,inventory=live_app(antler,mission,bridge,result['owned_app'])
        result['final_owned_app']=record;result['final_app_socket_inventory']=inventory
        write(ownership,{'verified_mono':time.monotonic(),'phase':'final_before_pass','owned_process':record,'udp_inventory':inventory})
        events=[parse(line) for line in (out/'probe.jsonl').read_text().splitlines() if line.startswith('{')]
        result['provider_evidence']=audit(events)
        (out/'status-audit.json').write_text(json.dumps(result['provider_evidence'],indent=2)+'\n')
        catalog=observed_catalog(out/'probe.jsonl',{'telemetry','system_status'},mode='RX_ONLY')
        result['catalog']=catalog;result['loaded_modules_sha256']=loaded_module_hashes(plugins,catalog)
        heartbeat=[p for p in packets if p['mavpackettype']=='HEARTBEAT'];landed=[p for p in packets if p['mavpackettype']=='EXTENDED_SYS_STATE']
        end=result['monitor_finished_mono']
        if not heartbeat or not landed or landed[-1]['landed_state']!=1 or any(p['base_mode']&128 for p in heartbeat):raise RuntimeError('independent ground/disarmed state not observed')
        if not 0<=end-heartbeat[-1]['received_mono']<=2 or not 0<=end-landed[-1]['received_mono']<=2:raise RuntimeError('independent final state stale')
        counts={kind:len([p for p in packets if p['mavpackettype']==kind]) for kind in ('SYS_STATUS','BATTERY_STATUS','GPS_RAW_INT')}
        late_counts={kind:len([p for p in packets if p['mavpackettype']==kind and p['received_mono']>=end-5]) for kind in counts}
        if any(n<2 for n in late_counts.values()):raise RuntimeError('independent observer lacks late advancing required message families')
        result['independent_battery_0_fixture']=observed_battery_zero(packets,end)
        if result['app_origin_datagrams'] or not result['relay_forwards_to_app']:raise RuntimeError('wire egress/forwarding assertion failed')
        result['independent_wire']={'app_origin_datagrams':0,'relay_forwards_to_native':0,'passive_observer_writes':0,
            'relay_forwards_to_app':result['relay_forwards_to_app'],'scope':'observed configured app-peer route, not a general host-wide packet capture'}
        result['observer']={'ever_armed':False,'final_on_ground':True,'heartbeat_samples':len(heartbeat),'landed_samples':len(landed),
            'final_heartbeat_age_s':end-heartbeat[-1]['received_mono'],'final_landed_age_s':end-landed[-1]['received_mono'],
            'message_counts':counts,'late_message_counts':late_counts,'modes':sorted({p['custom_mode'] for p in heartbeat})}
        result['result']='PASS'
    except BaseException as error:
        result['error']=str(error) or type(error).__name__;result['traceback']=traceback.format_exc()
    finally:
        remaining=[]
        for process in reversed(processes):
            try:remaining.extend(stop_process_tree(process))
            except BaseException as error:result['result']='FAIL';result.setdefault('cleanup_exceptions',[]).append(str(error))
        # Include anything queued through owned app shutdown in the independent
        # egress check. Never forward these final datagrams in either direction.
        if 'wire' in locals():
            for _ in range(512):
                try:data,sender=relay.recvfrom(65535)
                except BlockingIOError:break
                write(wire,{'mono':time.monotonic(),'role':'shutdown_drain','source':list(sender),
                    'destination':['127.0.0.1',relay_port],'raw_hex':data.hex()})
                if sender==('127.0.0.1',bridge):
                    result['app_origin_datagrams']+=1;result['result']='FAIL';result['egress_error']='app-origin datagram queued at shutdown'
            else:result['result']='FAIL';result['egress_error']='shutdown datagram drain bound exceeded'
        if 'independent_wire' in result:result['independent_wire']['app_origin_datagrams']=result['app_origin_datagrams']
        bridge_reservation.close();relay.close();monitor.close()
        for stream in streams:stream.close()
        cleared={}
        for key in ('bridge','relay','monitor','peer','monitor_peer','rc'):
            if key not in result['ports']:continue
            with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as stream:
                try:stream.bind(('127.0.0.1',result['ports'][key]));cleared[key]=True
                except OSError:cleared[key]=False
        for key in ('db','base'):
            with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as stream:
                stream.settimeout(.2);cleared[key]=stream.connect_ex(('127.0.0.1',result['ports'][key]))!=0
        result['ports_clear']=cleared;result['owned_processes_remaining']=remaining
        if not all(cleared.values()) or remaining:result['result']='FAIL';result['cleanup_error']='owned resources remain'
        try:
            logs=[];arms=[];battery=[];params={}
            for path in sorted((out/'logs').glob('*.BIN')):
                digest=sha(path);reader=DFReader.DFReader_binary(str(path))
                while True:
                    message=reader.recv_match(type=['ARM','BAT','PARM'])
                    if message is None:break
                    row={'log':path.name,**message.to_dict()}
                    if message.get_type()=='ARM':arms.append(row)
                    elif message.get_type()=='BAT':battery.append(row)
                    else:
                        name=message.Name.decode(errors='replace') if isinstance(message.Name,bytes) else message.Name
                        if name in FIXTURE:params[name]=float(message.Value)
                if sha(path)!=digest:raise RuntimeError('owned DataFlash changed during extraction')
                logs.append({'path':str(path.relative_to(out)),'sha256':digest})
            if not logs:raise RuntimeError('owned DataFlash unavailable')
            if any(int(row['ArmState'])!=0 for row in arms):raise RuntimeError('DataFlash observed arming')
            if any(name not in params or not math.isclose(params[name],value,rel_tol=1e-5,abs_tol=1e-4) for name,value in FIXTURE.items()):raise RuntimeError('DataFlash did not confirm all fixture parameters')
            if not battery or not any(row['Inst']==0 and 12.3<=row['Volt']<=12.9 for row in battery):raise RuntimeError('DataFlash lacks configured simulated battery voltage')
            result['dataflash']={'logs':logs,'arming_records':arms,'battery_samples':len(battery),'fixture_parameter_readback':params}
            (out/'dataflash-observations.json').write_text(json.dumps(safe({'arms':arms,'battery':battery,'fixture_parameters':params}),indent=2,allow_nan=False)+'\n')
        except BaseException as error:result['result']='FAIL';result['dataflash_error']=str(error)
        result['artifacts_after']={str(path):sha(path) for path in protected}
        result['artifact_drift']=[path for path in before if before[path]!=result['artifacts_after'][path]]
        if result['artifact_drift']:result['result']='FAIL';result['artifact_error']='source/artifact changed during run'
        result['files']={str(path.relative_to(out)):sha(path) for path in sorted(out.rglob('*')) if path.is_file() and path.name!='result.json'}
        (out/'result.json').write_text(json.dumps(safe(result),indent=2,allow_nan=False)+'\n')
    print(json.dumps(safe({key:result.get(key) for key in ('result','error','dataflash_error','probe_exit','provider_evidence','independent_wire','observer','ports_clear','owned_processes_remaining','artifact_drift')}),indent=2))
    return int(result['result']!='PASS')

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('ardupilot-root','moos-root','build','output'):parser.add_argument('--'+key,required=True)
    parser.add_argument('--source-root');parser.add_argument('--probe')
    parser.add_argument('--expected-app-sha256');parser.add_argument('--expected-status-sha256')
    args=parser.parse_args()
    signal.signal(signal.SIGTERM,lambda *_:(_ for _ in ()).throw(SystemExit(143)))
    with (Path(tempfile.gettempdir())/'mavmoos-native-sitl.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        raise SystemExit(run(args))
