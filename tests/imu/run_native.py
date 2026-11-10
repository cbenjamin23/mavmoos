#!/usr/bin/env python3
"""Owned disarmed Copter RX_ONLY IMU bench. No MAVLink/RC writes or relay.

Strict source/native/attestation preflight precedes any socket/process. Native
serial0 reaches the app directly; serial1 is a separate passive endpoint. One
exact-owned SIGSTOP3s/SIGCONT fault tests original age, clearing and recovery.
Zero TX is app/runtime-counter plus frozen-source/config evidence, not an
independent capture of the primary egress route. No DataFlash completeness claim.
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

ROOT=Path(__file__).resolve().parents[2]
FIXTURE={'LOG_DISARMED':1,'SERIAL0_PROTOCOL':2,'SERIAL1_PROTOCOL':2}
INTERVALS='27 200\n26 200\n116 200\n129 200\n245 500\n'
PROTECTED_SOURCE=('tests/imu/run_native.py','tests/imu/ImuMoosProbe.cpp','tests/imu/audit_native.py','tests/imu/wire_evidence.py',
 'src/plugins/ImuPlugin.cpp','src/plugins/ImuObservation.cpp','src/plugins/ImuObservation.h',
 'src/plugins/TelemetryPlugin.cpp','src/libmavmoos_runtime/Plugin.h','src/libmavmoos_runtime/PluginHost.cpp',
 'src/libmavmoos_runtime/Runtime.cpp','src/libmavmoos_runtime/Runtime.h','src/libmavmoos_runtime/RuntimeStream.cpp',
 'src/libmavmoos_runtime/ObservationRequests.cpp','src/libmavmoos_runtime/ObservationRequests.h',
 'src/iMAVLink/MAVLink.cpp','src/libmavmoos_rx/UdpReceiver.cpp','src/libmavmoos_rx/Telemetry.cpp')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def revision(path):return subprocess.check_output(['git','-C',str(path),'rev-parse','HEAD'],text=True).strip()
def dump(path,value):Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
def load_helpers(source):
    result={}
    for role,relative in (('ownership','tests/owned_processes.py'),('catalog','tests/control/catalog_evidence.py'),
                          ('passive','tests/observation_load/passive_receiver.py'),('manifest','scripts/firmware/manifest.py'),
                          ('audit','tests/imu/audit_native.py'),('wire','tests/imu/wire_evidence.py')):
        path=(source/relative).resolve()
        if not path.is_file() or source not in path.parents:raise ValueError('selected-source helper unavailable/outside source')
        spec=importlib.util.spec_from_file_location('_mavmoos_imu_'+role,path)
        if spec is None or spec.loader is None:raise ValueError('source helper cannot load')
        module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module);result[role]=module
    return result


def free_port(kind,address='127.0.0.1'):
    with socket.socket(socket.AF_INET,kind) as stream:stream.bind((address,0));return stream.getsockname()[1]


def inventory(process):
    if process.poll() is not None:raise RuntimeError('exact owned native exited')
    answer=subprocess.run(['lsof','-nP','-a','-p',str(process.pid),'-iUDP'],capture_output=True,text=True,timeout=3)
    if answer.returncode:raise RuntimeError('exact owned UDP inventory unavailable')
    return answer.stdout


def exact_peer(text,source,destination):
    return bool(re.search(r'UDP\s+127\.0\.0\.1:'+str(source)+r'->127\.0\.0\.1:'+str(destination)+r'(?:\s|$)',text))


def live_app(ownership,antler,mission,bridge,expected=None):
    matches=[]
    for row in ownership.owned_records(antler):
        try:tokens=shlex.split(row['command'])
        except ValueError:continue
        if tokens and Path(tokens[0]).name=='iMAVLink' and str(mission) in tokens:matches.append(row)
    if len(matches)!=1:raise RuntimeError('exact owned app identity is not unique/live')
    row=matches[0]
    if expected and (row['pid']!=expected['pid'] or row['command']!=expected['command']):raise RuntimeError('owned app identity changed')
    answer=subprocess.run(['lsof','-nP','-a','-p',str(row['pid']),'-iUDP'],capture_output=True,text=True,timeout=3)
    if answer.returncode or not re.search(r'UDP\s+127\.0\.0\.1:'+str(bridge)+r'(?:\s|$)',answer.stdout):raise RuntimeError('owned app bound UDP socket unproven')
    return row,answer.stdout


def preflight(args):
    if args.scenario!='rx_only':raise ValueError('unsupported IMU native scenario')
    source=Path(args.source_root).resolve() if args.source_root else ROOT
    if Path(__file__).resolve()!=source/'tests/imu/run_native.py':raise ValueError('runner must be from the explicitly selected source')
    helpers=load_helpers(source);ap,moos,build=[Path(getattr(args,k)).resolve() for k in ('ardupilot_root','moos_root','build')]
    firmware=helpers['ownership'].native_firmware_profile(source,ap,args.firmware_manifest,args.firmware_attestation)
    if not firmware or firmware.get('qualification')!='not_inferred' or firmware.get('frame')!='quad':raise ValueError('explicit verified native quad manifest/attestation required')
    native,app=Path(firmware['native_path']),build/'bin/iMAVLink';probe=Path(args.probe).resolve() if args.probe else build/'bin/imu_moos_probe'
    plugins=build/'plugins';artifacts=[native,app,probe,plugins/'mavmoos_telemetry.so',plugins/'mavmoos_imu.so',moos/'bin/pAntler',moos/'bin/MOOSDB']
    for path in artifacts:
        if not path.is_file() or path.is_symlink():raise ValueError('required regular native/app/module/dependency artifact missing')
    for expected,path in ((args.expected_app_sha256,app),(args.expected_probe_sha256,probe)):
        if expected and (not re.fullmatch('[0-9a-f]{64}',expected) or sha(path)!=expected):raise ValueError('expected frozen artifact SHA differs')
    manifest=helpers['manifest'].validate_manifest(Path(firmware['manifest_path']))
    if revision(moos)!=manifest['dependencies']['moos_revision']:raise ValueError('stock MOOS source revision differs from selected firmware dependency')
    return source,ap,moos,build,helpers,firmware,artifacts


def run(args):
    source,ap,moos,build,helpers,firmware,artifacts=preflight(args)
    # Imports do not launch the simulator or open a transport.
    os.environ['MAVLINK20']='1'
    import pymavlink
    from pymavlink import mavutil
    native,app,probe,telemetry_module,imu_module,antler_path,db_path=artifacts
    out=Path(args.output).resolve();out.mkdir(parents=True,exist_ok=False)
    copied={}
    for key,original,expected in (('manifest',Path(firmware['manifest_path']),firmware['manifest_sha256']),('attestation',Path(firmware['attestation_path']),firmware['attestation_sha256'])):
        path=out/('firmware-source.json' if key=='manifest' else 'firmware-attestation.json');raw=original.read_bytes()
        if hashlib.sha256(raw).hexdigest()!=expected:raise ValueError('verified firmware input changed before exclusive copy')
        with path.open('xb') as stream:stream.write(raw)
        copied[key]={'file':path.name,'sha256':sha(path)}
    copied_identity=helpers['manifest'].verify_native(out/copied['manifest']['file'],out/copied['attestation']['file'],ap)
    for key,value in copied_identity.items():
        if firmware.get(key)!=value:raise ValueError('copied firmware identity differs from preflight')
    protected=artifacts+[source/name for name in PROTECTED_SOURCE]+[Path(m.__file__) for m in helpers.values()]
    protected+=helpers['ownership'].firmware_source_inputs(source,firmware)
    protected+=[ap/'Tools/autotest/default_params/copter.parm',ap/'libraries/GCS_MAVLink/GCS_Common.cpp',ap/'libraries/AP_InertialSensor/AP_InertialSensor.h',ap/'libraries/AP_Compass/AP_Compass.h']
    protected+=[out/copied['manifest']['file'],out/copied['attestation']['file']]
    protected+=[Path(sys.executable).resolve(),Path(mavutil.mavlink.__file__).resolve()]
    before={str(path):sha(path) for path in protected}
    result={'schema':1,'result':'FAIL','scenario':'rx_only','profile':'arducopter','scope':'owned disarmed direct RX_ONLY native Copter IMU observations; no physical sensor/vehicle claim',
      'hardware':False,'source_root':str(source),'source_ref':revision(source),'ardupilot_ref':firmware['source_revision'],'moos_ref':revision(moos),
      'firmware_identity':firmware,'copied_firmware':copied,'runner_pid':os.getpid(),'app_sha256':sha(app),'probe_sha256':sha(probe),'native_sha256':sha(native),'imu_module_sha256':sha(imu_module),
      'artifacts_before':before,'loaded_helpers':{role:{'path':m.__file__,'sha256':sha(m.__file__)} for role,m in helpers.items()},
      'fixture_parameters':FIXTURE,'interval_file_units':'milliseconds','observer_writes':0,'arming_requested':False,'rc_input_sent':False,'relay_used':False,
      'transport_effects_claim':'frozen source/config and actual app/runtime counters; no independent primary-egress capture','timewarp':1,'observation_duration_s':25,
      'decoder':{'interpreter':str(Path(sys.executable).resolve()),'pymavlink_version':getattr(pymavlink,'__version__','unknown'),'dialect_file':str(Path(mavutil.mavlink.__file__).resolve())},
      'topology':'native serial0 directly to app; native serial1 separately passive','pause_requested':True,'pause_duration_requested_s':3}
    processes=[];logs=[];sockets=[];sitl=antler=probe_process=capture=None;paused=False;packets=[];decoder=None;capture_closed=False
    def launch(argv,log,env=None):
        stream=(out/log).open('w');logs.append(stream)
        process=subprocess.Popen(argv,cwd=out,stdout=stream,stderr=subprocess.STDOUT,env=env,start_new_session=True)
        processes.append(process);return process
    try:
        primary=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);primary.bind(('127.0.0.1',0));primary.setblocking(False);sockets.append(primary)
        monitor=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);monitor.bind(('127.0.0.1',0));monitor.setblocking(False);sockets.append(monitor)
        bridge,monitor_port=primary.getsockname()[1],monitor.getsockname()[1]
        db,base,rc=free_port(socket.SOCK_STREAM),free_port(socket.SOCK_STREAM),free_port(socket.SOCK_DGRAM,'0.0.0.0')
        while base==db:base=free_port(socket.SOCK_STREAM)
        result['ports']={'bridge':bridge,'monitor':monitor_port,'db':db,'base':base,'rc':rc}
        defaults=out/'fixture.parm';defaults.write_text(''.join(f'{key} {value}\n' for key,value in FIXTURE.items()))
        for channel in (0,1):(out/f'message-intervals-chan{channel}.txt').write_text(INTERVALS)
        command=[str(native),'--model','quad','--home','42.358456,-71.087589,10,0','--speedup','1','--wipe','--defaults',str(ap/'Tools/autotest/default_params/copter.parm')+','+str(defaults),
          '--serial0',f'udpclient:127.0.0.1:{bridge}','--serial1',f'udpclient:127.0.0.1:{monitor_port}',
          '--serial2','none','--serial5','none','--serial6','none','--serial7','none','--serial8','none','--base-port',str(base),'--rc-in-port',str(rc)]
        result['command']=command;sitl=launch(command,'sitl.log');result['sitl_pid']=sitl.pid
        raw=(out/'fc-observer.mavlink').open('wb');trace=(out/'fc-observer.jsonl').open('w');startup=(out/'observer-startup.jsonl').open('w');logs.extend([raw,trace,startup])
        decoder=helpers['wire'].FrameDecoder(mavutil.mavlink.MAVLink(None));byte_count=0
        def verify(sender):
            proof=inventory(sitl)
            if sender[0]!='127.0.0.1' or not exact_peer(proof,sender[1],monitor_port):raise RuntimeError('passive source not exact owned native peer')
            return proof
        def consume(data,sender,at):
            nonlocal byte_count
            byte_count+=len(data)
            if byte_count>8*1024*1024:raise RuntimeError('passive byte evidence bound exceeded')
            raw.write(data);rows,prefix=decoder.feed(data,at)
            for row in rows:packets.append(row);trace.write(json.dumps(row,allow_nan=False)+'\n')
            for row in prefix:startup.write(json.dumps(row,allow_nan=False)+'\n')
            raw.flush();trace.flush();startup.flush()
        capture=helpers['passive'].PassiveReceiver(monitor,verify,consume,max_datagrams=10000);capture.start()
        deadline=time.monotonic()+20;native_peer=None
        with (out/'primary-startup.mavlink').open('wb') as initial:
            while time.monotonic()<deadline:
                if sitl.poll() is not None:raise RuntimeError('owned native startup exited')
                capture.check()
                if native_peer is None:
                    try:data,sender=primary.recvfrom(65535)
                    except BlockingIOError:pass
                    else:
                        proof=inventory(sitl)
                        if sender[0]!='127.0.0.1' or not exact_peer(proof,sender[1],bridge):raise RuntimeError('primary source not exact owned native peer')
                        native_peer=sender;initial.write(data);result['native_primary_peer']=list(sender);result['native_primary_socket_inventory']=proof;result['ports']['peer']=sender[1]
                if native_peer and capture.peer and any(row['mavpackettype']=='HEARTBEAT' for row in packets):break
                time.sleep(.005)
        if not native_peer or not capture.peer or time.monotonic()>=deadline:raise RuntimeError('owned native primary/passive discovery deadline')
        result['native_observer_peer']=list(capture.peer);result['native_observer_socket_inventory']=capture.peer_proof;result['ports']['monitor_peer']=capture.peer[1]
        mission=out/'imu_observations.moos'
        mission.write_text(f'''ServerHost = localhost
ServerPort = {db}
Community = imu_observations
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
  peer_port = {native_peer[1]}
  target_system = 1
  target_component = 1
  expected_type = 2
  expected_autopilot = 3
  simulation_only = false
  shadow_mode = false
  plugins = telemetry,imu
  plugin_directory = {build/'plugins'}
  max_datagrams_per_iter = 128
  imu_timeout = 1
  imu_publish_interval = 0.1
}}
''')
        env=dict(os.environ);env['PATH']=os.pathsep.join([str(build/'bin'),str(moos/'bin'),env.get('PATH','')])
        primary.close();antler=launch([str(antler_path),str(mission)],'pantler.log',env)
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            capture.check()
            if sitl.poll() is not None or antler.poll() is not None:raise RuntimeError('owned native/app startup exited')
            try:owned,proof=live_app(helpers['ownership'],antler,mission,bridge)
            except RuntimeError:time.sleep(.05)
            else:break
        else:raise RuntimeError('owned app startup deadline')
        result['owned_app']=owned;result['app_socket_inventory']=proof
        probe_process=launch([str(probe),str(db),'25'],'probe.jsonl',env);started=time.monotonic();next_check=started;faults=[]
        with (out/'ownership.jsonl').open('w') as owner_log:
            while time.monotonic()-started<35:
                capture.check()
                if sitl.poll() is not None or antler.poll() is not None:raise RuntimeError('owned native/app exited during observations')
                at=time.monotonic()
                if not faults and at-started>=8:
                    # Popen is unreaped/live and exact native identity was fixed
                    # before any fixture signal. No process-name-wide signal.
                    proof=inventory(sitl)
                    if not exact_peer(proof,native_peer[1],bridge):raise RuntimeError('native peer changed before owned pause')
                    os.kill(sitl.pid,signal.SIGSTOP);paused=True;faults.append({'kind':'native_stopped','mono':time.monotonic(),'pid':sitl.pid,'inventory':proof})
                elif paused and at-faults[0]['mono']>=3:
                    os.kill(sitl.pid,signal.SIGCONT);paused=False;faults.append({'kind':'native_continued','mono':time.monotonic(),'pid':sitl.pid})
                if at>=next_check:
                    row,proof=live_app(helpers['ownership'],antler,mission,bridge,owned)
                    owner_log.write(json.dumps({'mono':time.monotonic(),'process':row,'udp_inventory':proof})+'\n');owner_log.flush();next_check=time.monotonic()+.5
                if probe_process.poll() is not None:break
                time.sleep(.005)
        if probe_process.poll() is None:raise RuntimeError('IMU subscriber exceeded original35s supervisor')
        result['probe_exit']=probe_process.returncode
        if probe_process.returncode:raise RuntimeError('IMU MOOS subscriber failed')
        if paused or len(faults)!=2 or not 3<=faults[1]['mono']-faults[0]['mono']<=5:raise RuntimeError('owned pause/continue sequence missing/outside bound')
        result['fault_events']=faults;result['pause_observed_s']=faults[1]['mono']-faults[0]['mono']
        row,proof=live_app(helpers['ownership'],antler,mission,bridge,owned);result['final_owned_app']=row;result['final_app_socket_inventory']=proof
        capture.close();capture_closed=True;result['capture']=capture.status();result['capture_finished_mono']=time.monotonic();result['wire_decode']=decoder.finish()
        raw.close();trace.close();startup.close()
        events=[helpers['audit'].parse(line) for line in (out/'probe.jsonl').read_text().splitlines() if line]
        # The catalog reflects the current build. The witness below still
        # qualifies only the original raw/scaled IMU wire families.
        result['provider_evidence']=helpers['audit'].audit_probe(events,expect_vibration=True)
        result['passive_fc_evidence']=helpers['audit'].audit_fc(packets,result['capture_finished_mono'])
        result['corroboration']=helpers['audit'].corroborate(result['provider_evidence'],result['passive_fc_evidence'])
        result['wire_replay']=helpers['wire'].replay((out/'fc-observer.mavlink').read_bytes(),packets,lambda:mavutil.mavlink.MAVLink(None))
        result['catalog']=helpers['catalog'].observed_catalog(out/'probe.jsonl',{'telemetry','imu'},mode='RX_ONLY')
        result['loaded_modules_sha256']=helpers['catalog'].loaded_module_hashes(build/'plugins',result['catalog'])
        dump(out/'imu-audit.json',result['provider_evidence']);dump(out/'passive-fc-audit.json',result['passive_fc_evidence']);dump(out/'wire-replay.json',result['wire_replay'])
        result['result']='PASS'
    except BaseException as error:
        result['error']=str(error) or type(error).__name__;result['traceback']=traceback.format_exc()
    finally:
        if paused and sitl is not None and sitl.poll() is None:
            try:os.kill(sitl.pid,signal.SIGCONT);paused=False
            except BaseException as error:result['result']='FAIL';result['resume_cleanup_error']=str(error)
        if capture is not None and not capture_closed:
            try:capture.close();capture_closed=True
            except BaseException as error:result['result']='FAIL';result['capture_cleanup_error']=str(error)
            result['capture']=capture.status()
        remaining=[];cleanup_proven=True
        for process in reversed(processes):
            try:
                left=helpers['ownership'].stop_process_tree(process)
                if not isinstance(left,list):raise RuntimeError('owned cleanup returned unknown inventory')
                remaining.extend(left)
            except BaseException as error:
                cleanup_proven=False;result['result']='FAIL';result.setdefault('cleanup_errors',[]).append(str(error) or type(error).__name__)
        for stream in sockets:
            try:stream.close()
            except BaseException as error:cleanup_proven=False;result['result']='FAIL';result.setdefault('cleanup_errors',[]).append(str(error))
        for stream in logs:
            try:stream.close()
            except BaseException as error:cleanup_proven=False;result['result']='FAIL';result.setdefault('cleanup_errors',[]).append(str(error) or type(error).__name__)
        result['owned_processes_remaining']=remaining if cleanup_proven else None
        cleared={}
        for key,port in result.get('ports',{}).items():
            if key in ('db','base'):
                with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as stream:stream.settimeout(.2);cleared[key]=stream.connect_ex(('127.0.0.1',port))!=0
            else:
                with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as stream:
                    try:stream.bind(('0.0.0.0' if key=='rc' else '127.0.0.1',port));cleared[key]=True
                    except OSError:cleared[key]=False
        result['ports_clear']=cleared;result['cleanup_proven']=cleanup_proven and not remaining and bool(cleared) and all(cleared.values()) and (capture is None or (capture_closed and result.get('capture',{}).get('thread_retired') is True and result.get('capture',{}).get('socket_closed') is True and result.get('capture',{}).get('error') is None))
        if not result['cleanup_proven']:result['result']='FAIL'
        try:
            result['firmware_identity_after']=helpers['ownership'].native_firmware_profile(source,ap,args.firmware_manifest,args.firmware_attestation)
            result['source_ref_after']=revision(source);result['moos_ref_after']=revision(moos)
            after_copied=helpers['manifest'].verify_native(out/copied['manifest']['file'],out/copied['attestation']['file'],ap)
            if result['firmware_identity_after']!=firmware or after_copied!=copied_identity or result['source_ref_after']!=result['source_ref'] or result['moos_ref_after']!=result['moos_ref']:raise ValueError('firmware/native/copied/source/dependency identity drift')
            result['artifacts_after']={str(path):sha(path) for path in protected};result['artifact_drift']=[path for path,digest in before.items() if result['artifacts_after'][path]!=digest]
            if result['artifact_drift']:raise ValueError('protected input/artifact drift')
        except BaseException as error:result['result']='FAIL';result['postflight_error']=str(error) or type(error).__name__
        result['files']={str(p.relative_to(out)):sha(p) for p in sorted(out.rglob('*')) if p.is_file() and p.name!='result.json'}
        dump(out/'result.json',result)
    print(json.dumps({key:result.get(key) for key in ('result','error','postflight_error','probe_exit','provider_evidence','passive_fc_evidence','cleanup_proven','artifact_drift')},indent=2,allow_nan=False))
    return int(result['result']!='PASS')


def parser():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--scenario',choices=['rx_only'],required=True)
    for key in ('ardupilot-root','moos-root','build','output'):p.add_argument('--'+key,required=True)
    for key in ('source-root','probe','firmware-manifest','firmware-attestation','expected-app-sha256','expected-probe-sha256'):p.add_argument('--'+key)
    return p


if __name__=='__main__':
    args=parser().parse_args();signal.signal(signal.SIGTERM,lambda *_:(_ for _ in ()).throw(SystemExit(143)))
    with (Path(tempfile.gettempdir())/'mavmoos-native-sitl.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);raise SystemExit(run(args))
