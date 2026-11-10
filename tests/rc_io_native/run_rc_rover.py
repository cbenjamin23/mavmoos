#!/usr/bin/env python3
"""Existing owned Rover workflow plus optional read-only RC observer.

Reuses the built unchanged rover_workflow_probe and stock2D helm mission.
Native200s wrapper permits explicit GPS/EKF readiness, Guided/hold/manual
pilot takeover/reacquisition/RTL/disarm. Only the existing owned native RC
fixture controls neutral RC1/RC3 and the RC8 switch. No new motion strategy,
RC override/servo action, parameter write or physical route. Common lock.
"""
import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time

from native_support import source_helpers,NativePilot,live_app,exact_peer,ref
from RcNativeAudit import audit,parse,summary,corroborate
os.environ['MAVLINK20']='1'  # Extension-aware passive decoder, not an FC writer.
from pymavlink import mavutil
PIN='04cf6486e8d42ce138de816e8466244346ed77cb'
NATIVE_SHA='a859ab703395bbb04da523dae5414cc5fbc23b736a1c71639f2ef3e255df5fc9'
stop_process_tree=loaded_module_hashes=None


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def port(kind):
    with socket.socket(socket.AF_INET, kind) as stream:
        stream.bind(('127.0.0.1', 0))
        return stream.getsockname()[1]


def sockets(process):
    if process.poll() is not None:
        raise RuntimeError('owned native Rover exited')
    answer = subprocess.run(['lsof', '-nP', '-a', '-p', str(process.pid), '-iUDP'],
                            capture_output=True, text=True, timeout=3)
    if answer.returncode:
        raise RuntimeError('owned native Rover UDP inventory unavailable')
    return answer.stdout


def owns_udp(inventory, number, allow_wildcard=False):
    addresses = r'(?:\*|0\.0\.0\.0|127\.0\.0\.1)' if allow_wildcard else r'127\.0\.0\.1'
    return bool(re.search(r'UDP\s+' + addresses + ':' + str(number) + r'\b', inventory))



def dataflash(path):
    reader = mavutil.mavlink_connection(str(path))
    records = {'MODE': [], 'ARM': [], 'RCIN': [], 'RCOU': []}
    while True:
        message = reader.recv_match(type=list(records))
        if message is None:
            break
        records[message.get_type()].append(message.to_dict())
    armed = next(item for item in records['ARM'] if item['ArmState'] == 1)
    manual = next(item for item in records['MODE'] if item['ModeNum'] == 0 and item['TimeUS'] > armed['TimeUS'])
    recovered = next(item for item in records['MODE'] if item['ModeNum'] == 15 and item['TimeUS'] > manual['TimeUS'])
    samples = [item for item in records['RCIN'] if manual['TimeUS'] <= item['TimeUS'] < recovered['TimeUS']]
    if manual.get('Rsn') != 1 or not samples or any(item['C1'] != 1500 or item['C3'] != 1500 or item['C8'] != 1000 for item in samples):
        raise RuntimeError('DataFlash does not corroborate RC-command Manual takeover with neutral drive inputs')
    if armed.get('Forced', 0) != 0 or not any(item['ArmState'] == 0 and item['TimeUS'] > armed['TimeUS'] for item in records['ARM']):
        raise RuntimeError('normal arm/final disarm not corroborated in DataFlash')
    if not {4, 11, 15}.issubset({item['ModeNum'] for item in records['MODE']}):
        raise RuntimeError('Hold/RTL/Guided not corroborated in DataFlash')
    return {'armed': armed, 'pilot_manual': manual, 'explicit_guided_recovery': recovered,
            'manual_neutral_samples': len(samples), 'first_manual_rcin': samples[0], 'last_manual_rcin': samples[-1]}, records


def main(args):
    global stop_process_tree,loaded_module_hashes
    root = Path(args.source_root).resolve()
    helpers=source_helpers(root)
    stop_process_tree=helpers['ownership'].stop_process_tree
    loaded_module_hashes=helpers['catalog'].loaded_module_hashes
    ap, moos, build, out = (Path(getattr(args, key)).resolve() for key in ('ardupilot_root', 'moos_root', 'build', 'output'))
    binary, probe_path = ap / 'build/sitl/bin/ardurover', build / 'bin/rover_workflow_probe'
    rc_probe=Path(args.rc_probe).resolve() if args.rc_probe else build/'bin/rc_io_moos_probe'
    artifact_paths=[binary,probe_path,rc_probe,build/'bin/iMAVLink',moos/'bin/pAntler',moos/'bin/pHelmIvP']
    artifact_paths += [build/'plugins'/('mavmoos_'+name+'.so') for name in ('telemetry','parameters','ardurover_control','rc_io')]
    for path in artifact_paths:
        if not path.is_file():raise RuntimeError('required artifact missing: '+str(path))
    if ref(ap)!=PIN or sha(binary)!=NATIVE_SHA:raise RuntimeError('native Rover source/binary not pinned')
    if sha(build/'bin/iMAVLink')!=args.expected_app_sha256 or sha(build/'plugins/mavmoos_rc_io.so')!=args.expected_rc_sha256:
        raise RuntimeError('frozen app/RC observer hash mismatch')
    protected=artifact_paths+[Path(__file__).resolve(),Path(__file__).with_name('RcNativeAudit.py'),Path(__file__).with_name('native_support.py'),Path(__file__).with_name('RcIoMoosProbe.cpp')]
    protected += [root/name for name in ('tests/owned_processes.py','tests/control/catalog_evidence.py','tests/rover/RoverWorkflowProbe.cpp',
        'missions/rover_sitl/meta_vehicle.moos','missions/rover_sitl/meta_vehicle.bhv','src/plugins/RcIoPlugin.cpp','src/plugins/TelemetryPlugin.cpp',
        'src/plugins/ArduRoverControlPlugin.cpp','src/libmavmoos_runtime/Runtime.cpp','src/libmavmoos_runtime/Runtime.h',
        'src/libmavmoos_runtime/RuntimeStream.cpp','src/libmavmoos_runtime/ObservationRequests.cpp','src/iMAVLink/MAVLink.cpp')]
    protected += [ap/name for name in ('Tools/autotest/default_params/rover.parm','libraries/AP_RCProtocol/AP_RCProtocol_UDP.cpp',
        'libraries/GCS_MAVLink/GCS_Common.cpp','libraries/AP_Logger/LogStructure.h')]
    before={str(path):sha(path) for path in protected}
    # A fresh directory gives this run sole ownership of logs, markers and BIN files.
    out.mkdir(parents=True, exist_ok=False)
    (out / 'logs').mkdir()
    control = out / 'control'
    control.mkdir()
    bridge, monitor = (socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(2))
    bridge.bind(('127.0.0.1', 0)); bridge.settimeout(.2)
    monitor.bind(('127.0.0.1', 0)); monitor.setblocking(False)
    bridge_port, monitor_port = bridge.getsockname()[1], monitor.getsockname()[1]
    db, base, rc = port(socket.SOCK_STREAM), port(socket.SOCK_STREAM), port(socket.SOCK_DGRAM)
    fixture = out / 'fixture.parm'
    fixture.write_text('SERIAL1_PROTOCOL 2\nMAV2_POSITION 10\nMAV2_EXT_STAT 2\nMAV2_EXTRA3 2\nLOG_DISARMED 1\nWP_RADIUS 1\nWP_SPEED 3\nMODE_CH 8\nMODE1 0\nMODE4 4\n')
    (out / 'message-intervals-chan1.txt').write_text('33 100\n193 500\n65 100\n36 200\n')
    defaults = ap / 'Tools/autotest/default_params/rover.parm'
    command = [str(binary), '--model', 'rover', '--home', '42.358456,-71.087589,10,0', '--speedup', '1', '--wipe',
               '--defaults', str(defaults) + ',' + str(fixture),
               '--serial0', f'udpclient:127.0.0.1:{bridge_port}', '--serial1', f'udpclient:127.0.0.1:{monitor_port}',
               '--serial2', 'none', '--serial5', 'none', '--serial6', 'none', '--serial7', 'none', '--serial8', 'none',
               '--base-port', str(base), '--rc-in-port', str(rc)]
    result = {'result': 'FAIL', 'layer': 'owned native ground ArduRover + stock pHelmIvP + dynamic iMAVLink',
              'hardware': False, 'timewarp': 1, 'command': command, 'native_sha256': sha(binary),
              'app_sha256': sha(build / 'bin/iMAVLink'), 'probe_sha256': sha(probe_path),
              'harness_sha256': sha(__file__), 'defaults_sha256': sha(defaults), 'fixture_sha256': sha(fixture),
              'ardupilot_revision': subprocess.check_output(['git', '-C', str(ap), 'rev-parse', 'HEAD'], text=True).strip(),
              'moos_revision': subprocess.check_output(['git', '-C', str(moos), 'rev-parse', 'HEAD'], text=True).strip(),
              'mavmoos_revision': subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip(),
              'mavmoos_revision_scope': 'actual source-root semantics and recorded matching app/probe/module artifacts; before/after hashes detect drift',
              'source_root':str(root),'artifacts_before':before,
              'loaded_helpers':{role:{'path':module.__file__,'sha256':sha(module.__file__)} for role,module in helpers.items()},
              'rc_module_sha256':sha(build/'plugins/mavmoos_rc_io.so'),'rc_probe_sha256':sha(rc_probe),
              'ports': {'bridge': bridge_port, 'monitor': monitor_port, 'db': db, 'base': base, 'rc': rc},
              'independent_monitor_writes': 'none; pinned fixture and message intervals set observation streams',
              'pilot_writes': 'native owned RC UDP only; neutral RC1/RC3, RC8 Hold-to-Manual switch'}
    sitl = launch = probe = pilot = rc_observer = None
    provider_proof=None
    handles, packets = [], []
    try:
        native_log = (out / 'sitl.log').open('w'); handles.append(native_log)
        sitl = subprocess.Popen(command, cwd=out, stdout=native_log, stderr=subprocess.STDOUT, start_new_session=True)
        result['sitl_pid'] = sitl.pid
        decoder = mavutil.mavlink.MAVLink(None); decoder.robust_parsing = True
        deadline, peer = time.monotonic() + 20, None
        while time.monotonic() < deadline:
            if sitl.poll() is not None:
                raise RuntimeError('owned Rover exited during startup')
            try:
                data, address = bridge.recvfrom(4096)
            except socket.timeout:
                continue
            for message in decoder.parse_buffer(data) or []:
                if message.get_type() == 'HEARTBEAT' and message.get_srcSystem() == 1 and message.get_srcComponent() == 1:
                    if message.type != 10 or message.autopilot != 3:
                        raise RuntimeError('unexpected native heartbeat identity')
                    peer = address
            if peer:
                break
        # The first heartbeat can precede the lazily initialized native RC
        # backend. Preserve every ownership snapshot and wait boundedly for it.
        # MAVLink still needs exact loopback binding; only the native RC socket
        # may use AP_RCProtocol_UDP's source-defined 0.0.0.0 bind.
        result['discovered_peer'] = peer
        if peer is not None:
            result['ports']['peer'] = peer[1]
        result['sitl_udp_socket_attempts'] = []
        ready_by = time.monotonic() + 5
        while True:
            inventory = sockets(sitl)
            result['sitl_udp_sockets'] = inventory
            result['sitl_udp_socket_attempts'].append({'observed_mono': time.monotonic(), 'inventory': inventory})
            if peer is not None and peer[0] == '127.0.0.1' and exact_peer(inventory, peer[1], bridge_port) and owns_udp(inventory, rc, allow_wildcard=True):
                break
            if time.monotonic() >= ready_by:
                break
            time.sleep(.1)
        if peer is None or peer[0] != '127.0.0.1' or not exact_peer(inventory, peer[1], bridge_port) or not owns_udp(inventory, rc, allow_wildcard=True):
            raise RuntimeError('exact owned Rover loopback/RC endpoints not verified')
        bridge.close()
        pilot = NativePilot(sitl, rc, control, out / 'fixture-pilot.jsonl','rover')
        pilot.update()
        macros = {'MOOS_PORT': str(db), 'APP_PATH': str(build / 'bin/iMAVLink'), 'BIND_PORT': str(bridge_port),
                  'PEER_PORT': str(peer[1]), 'LAT_ORIGIN': '42.358456', 'LONG_ORIGIN': '-71.087589',
                  'SIMULATION_ONLY': 'true', 'SHADOW_MODE': 'false', 'SITL_PID': str(sitl.pid),
                  'PLUGIN_DIRECTORY': str(build / 'plugins'), 'SPEED': '1.2', 'WAYPOINTS': '0,12:12,12:12,0:0,0'}
        env = dict(os.environ)
        env['PATH'] = os.pathsep.join([str(build / 'bin'), str(moos / 'bin'), env.get('PATH', '')])
        # This mission uses stock built-in 2D behaviors only.
        env.pop('IVP_BEHAVIOR_DIRS', None)
        for suffix in ('moos', 'bhv'):
            subprocess.run([str(moos / 'bin/nsplug'), str(root / f'missions/rover_sitl/meta_vehicle.{suffix}'),
                            str(out / f'targ_rover.{suffix}'), '--strict', '--force', '-x'] +
                           [f'{key}={value}' for key, value in macros.items()], env=env, check=True, capture_output=True)
            if suffix=='moos':
                target=out/'targ_rover.moos';text=target.read_text()
                old='plugins = telemetry,parameters,ardurover_control'
                if text.count(old)!=1:raise RuntimeError('stock Rover mission plugin line changed unexpectedly')
                target.write_text(text.replace(old,old+',rc_io\n  rc_io_timeout = 1\n  rc_io_publish_interval = 0.1'))
            result[f'generated_{suffix}_sha256'] = sha(out / f'targ_rover.{suffix}')
        log = (out / 'pantler.log').open('w'); handles.append(log)
        launch = subprocess.Popen([str(moos / 'bin/pAntler'), str(out / 'targ_rover.moos')], cwd=out,
                                  env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        log = (out / 'probe-stdout.log').open('w'); handles.append(log)
        probe = subprocess.Popen([str(probe_path), str(db), str(out / 'workflow.jsonl'), str(control)],
                                 cwd=out, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rc_log=(out/'rc-probe.jsonl').open('w');handles.append(rc_log)
        rc_observer=subprocess.Popen([str(rc_probe),str(db),'210',str(control)],cwd=out,env=env,stdout=rc_log,stderr=subprocess.STDOUT,start_new_session=True)
        ownership_log=(out/'app-ownership.jsonl').open('w');handles.append(ownership_log)
        next_app_check=0
        decoder = mavutil.mavlink.MAVLink(None); decoder.robust_parsing = True
        deadline, drain, monitor_peer = time.monotonic() + 200, None, None
        with (out / 'fc-monitor.jsonl').open('w') as trace, (out / 'fc-monitor.mavlink').open('wb') as raw:
            while time.monotonic() < deadline:
                if sitl.poll() is not None or launch.poll() is not None:raise RuntimeError('owned native/MOOS launcher exited')
                if rc_observer.poll() is not None:raise RuntimeError('RC subscriber exited before workflow finished')
                if time.monotonic()>=next_app_check:
                    try:owned,inventory=live_app(helpers['ownership'],launch,out/'targ_rover.moos',bridge_port,result.get('owned_app'),result.get('owned_helm'))
                    except RuntimeError:
                        if 'owned_app' in result:raise
                    else:
                        if 'pHelmIvP' in owned:
                            result['owned_app'],result['owned_helm']=owned['iMAVLink'],owned['pHelmIvP']
                            ownership_log.write(json.dumps({'verified_mono':time.monotonic(),'owned':owned,'udp_inventory':inventory})+'\n');ownership_log.flush()
                    next_app_check=time.monotonic()+.5
                if probe.poll() is not None:
                    if drain is None:
                        drain = time.monotonic() + 1.5
                    if time.monotonic() >= drain:
                        break
                else:
                    pilot.update()
                try:
                    data, address = monitor.recvfrom(65535)
                except BlockingIOError:
                    time.sleep(.02); continue
                if monitor_peer is None:
                    if address[0] != '127.0.0.1' or not exact_peer(sockets(sitl), address[1], monitor_port):
                        raise RuntimeError('independent telemetry endpoint is not owned by native Rover')
                    monitor_peer = address
                    result['monitor_peer'] = address
                    result['ports']['monitor_peer'] = address[1]
                if address != monitor_peer:
                    raise RuntimeError('independent monitor received another endpoint')
                raw.write(data)
                for message in decoder.parse_buffer(data) or []:
                    if message.get_srcSystem() == 1 and message.get_srcComponent() == 1 and message.get_type() in {'HEARTBEAT', 'GLOBAL_POSITION_INT', 'STATUSTEXT','RC_CHANNELS','SERVO_OUTPUT_RAW'}:
                        row = {'received_mono':time.monotonic(),'wire_version':2 if message.get_msgbuf()[0]==253 else 1,'system_id':message.get_srcSystem(),'component_id':message.get_srcComponent(),**message.to_dict()}
                        trace.write(json.dumps(row) + '\n');trace.flush();packets.append(row)
                    if len(packets)>100000 or raw.tell()>16*1024*1024:raise RuntimeError('passive evidence bound exceeded')
            if probe.poll() is None:
                raise RuntimeError('bounded 200 second wrapper deadline exceeded')
        result['monitor_finished_mono']=time.monotonic()
        (control/'observer.stop').write_text('workflow_finished\n')
        rc_observer.wait(timeout=3)
        if rc_observer.returncode:raise RuntimeError('read-only RC subscriber failed')
        if 'owned_app' not in result or 'owned_helm' not in result:raise RuntimeError('owned app/helm identity was not proved')
        owned,inventory=live_app(helpers['ownership'],launch,out/'targ_rover.moos',bridge_port,result['owned_app'],result['owned_helm'])
        ownership_log.write(json.dumps({'verified_mono':time.monotonic(),'phase':'final_before_pass','owned':owned,'udp_inventory':inventory})+'\n');ownership_log.flush()
        provider_proof=audit([parse(line) for line in (out/'rc-probe.jsonl').read_text().splitlines() if line.startswith('{')],mode='SITL_CONTROL',duration=210)
        result['rc_provider_evidence']=summary(provider_proof)
        (out/'rc-audit.json').write_text(json.dumps(result['rc_provider_evidence'],indent=2)+'\n')
        result['probe_exit'] = probe.returncode
        hearts = [row for row in packets if row['mavpackettype'] == 'HEARTBEAT']
        positions = [row for row in packets if row['mavpackettype'] == 'GLOBAL_POSITION_INT' and abs(row['lat']/1e7-42.358456) < .001 and abs(row['lon']/1e7+71.087589) < .001]
        def distance(row):
            return math.hypot((row['lat']/1e7-42.358456)*111195., (row['lon']/1e7+71.087589)*111195.*math.cos(math.radians(42.358456)))
        independent = {'observed_modes': sorted({row['custom_mode'] for row in hearts}),
                       'armed_observed': any(row['base_mode'] & 128 for row in hearts),
                       'final_disarmed': bool(hearts) and not bool(hearts[-1]['base_mode'] & 128),
                       'position_samples': len(positions), 'heartbeat_samples': len(hearts),
                       'maximum_home_distance_m': max(map(distance, positions), default=0),
                       'final_home_distance_m': distance(positions[-1]) if positions else -1,
                       'final_speed_m_s': math.hypot(positions[-1]['vx'], positions[-1]['vy'])/100 if positions else -1}
        result['independent_monitor'] = independent
        if not hearts or not positions or any(not 0<=result['monitor_finished_mono']-rows[-1]['received_mono']<=2 for rows in (hearts,positions)):
            raise RuntimeError('final independent vehicle state stale')
        if probe.returncode != 0:
            raise RuntimeError('Rover MOOS workflow failed; inspect workflow.jsonl')
        result['capabilities_catalog'] = helpers['catalog'].observed_catalog(
            out / 'workflow.jsonl', {'telemetry', 'parameters', 'ardurover_control', 'rc_io'},
            native=True, helm=True, mode='SITL_CONTROL')
        result['loaded_modules_sha256'] = loaded_module_hashes(build / 'plugins', result['capabilities_catalog'])
        if not {0, 4, 11, 15}.issubset(independent['observed_modes']) or not independent['armed_observed'] or not independent['final_disarmed'] or independent['maximum_home_distance_m'] < 4 or not 0 <= independent['final_home_distance_m'] <= 2.5 or not 0 <= independent['final_speed_m_s'] <= .4:
            raise RuntimeError('independent FC monitor did not corroborate motion, takeover, return and disarmed stop')
        result['result'] = 'PASS'
    except BaseException as error:
        result['error'] = str(error) or type(error).__name__
    finally:
        if pilot is not None:
            result['native_rc_local_handoffs']=pilot.sent;pilot.close()
        remaining = []
        for process in (rc_observer,probe, launch, sitl):
            remaining.extend(stop_process_tree(process))
        bridge.close(); monitor.close()
        for handle in handles:
            handle.close()
        clear = {}
        for name in ('bridge', 'monitor', 'rc', 'peer', 'monitor_peer'):
            if name in result['ports']:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as stream:
                    try:
                        stream.bind(('127.0.0.1', result['ports'][name])); clear[name] = True
                    except OSError:
                        clear[name] = False
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as stream:
            stream.settimeout(.2); clear['moos_tcp'] = stream.connect_ex(('127.0.0.1', db)) != 0
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as stream:
            stream.settimeout(.2); clear['base_tcp'] = stream.connect_ex(('127.0.0.1', base)) != 0
        result['ports_clear'], result['owned_processes_remaining'] = clear, remaining
        if not all(clear.values()) or remaining:
            result['result'], result['cleanup_error'] = 'FAIL', 'owned process/endpoint cleanup incomplete'
        if result['result'] == 'PASS':
            try:
                bins = list((out / 'logs').glob('*.BIN'))
                if len(bins) != 1:
                    raise RuntimeError('expected exactly one new BIN from fresh owned output')
                digest = sha(bins[0]); proof,records = dataflash(bins[0])
                if sha(bins[0]) != digest:
                    raise RuntimeError('selected DataFlash file changed during read')
                result['independent_dataflash'] = {'path':str(bins[0]),'sha256':digest,**proof}
                if provider_proof is None:raise RuntimeError('RC source-value audit missing')
                result['independent_rc_values']=corroborate(provider_proof,packets,records)
                (out/'dataflash-observations.json').write_text(json.dumps(records,indent=2)+'\n')
                (out/'value-corroboration.json').write_text(json.dumps(result['independent_rc_values'],indent=2)+'\n')
            except Exception as error:
                result['result'], result['error'] = 'FAIL', str(error)
        result['artifacts_after']={str(path):sha(path) for path in protected}
        result['artifact_drift']=[path for path in before if before[path]!=result['artifacts_after'][path]]
        if result['artifact_drift']:result['result']='FAIL';result['artifact_error']='source/artifact changed during run'
        result['files']={str(path.relative_to(out)):sha(path) for path in sorted(out.rglob('*')) if path.is_file() and path.name!='result.json'}
        (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'result': result['result'], 'output': str(out), 'error': result.get('error'),
                      'independent_monitor': result.get('independent_monitor'), 'ports_clear': result['ports_clear']}, indent=2))
    return 0 if result['result'] == 'PASS' else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('ardupilot-root', 'moos-root', 'build', 'output'):
        parser.add_argument('--' + name, required=True)
    parser.add_argument('--source-root',required=True);parser.add_argument('--rc-probe')
    parser.add_argument('--expected-app-sha256',required=True);parser.add_argument('--expected-rc-sha256',required=True)
    arguments = parser.parse_args()
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(143)))
    with (Path(tempfile.gettempdir()) / 'mavmoos-native-sitl.lock').open('w') as lease:
        fcntl.flock(lease, fcntl.LOCK_EX|fcntl.LOCK_NB)
        raise SystemExit(main(arguments))
