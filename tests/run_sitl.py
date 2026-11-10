#!/usr/bin/env python3
"""Owned native Copter + stock MOOS mission. No serial devices or external routes."""
import argparse
import hashlib
import json
import os
import re
import struct
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import time
from control.catalog_evidence import observed_catalog, loaded_module_hashes

from pymavlink import mavutil
from owned_processes import native_firmware_profile, owned_records, stop_process_tree

COPTER_FRAMES = {'hexa': (13, 2), 'octa': (14, 3)}


def port(kind):
    with socket.socket(socket.AF_INET, kind) as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stop(process):
    return stop_process_tree(process)


class FixturePilot:
    """Only the exact owned native SITL RC port; no MAVLink observer writes."""
    def __init__(self, sitl, rc_port, ready_path, workflow_path, trace_path):
        self.sitl, self.rc_port, self.ready_path = sitl, rc_port, ready_path
        self.workflow_path = workflow_path
        self.workflow_offset = 0
        self.workflow_tail = ''
        self.phase = ''
        self.prepare_phase = None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 0)); self.sock.setblocking(False)
        self.trace = trace_path.open('w')
        self.active = self.prepared = self.seen_loiter = False
        self.last_send = self.started = 0
        self.sent_neutral = self.sent_restore = 0
        self.ownership = None

    def send(self, pwm, reason):
        if self.sitl.poll() is not None:
            raise RuntimeError('owned SITL exited before fixture pilot send')
        packet = struct.pack('<8H', 0, 0, pwm, 0, 0, 0, 0, 0)
        if self.sock.sendto(packet, ('127.0.0.1', self.rc_port)) != len(packet):
            raise RuntimeError('fixture pilot RC packet not locally accepted')
        self.last_send = time.monotonic()
        if pwm == 1500: self.sent_neutral += 1
        else: self.sent_restore += 1
        self.trace.write(json.dumps({'sent_mono': self.last_send, 'target': ['127.0.0.1', self.rc_port],
            'channel': 3, 'pwm': pwm, 'other_channels': 'zero=retain', 'reason': reason})+'\n')
        self.trace.flush()

    def restore(self, reason):
        if self.active and self.sitl.poll() is None:
            self.send(1000, reason)  # Last PWM persists; silence is not release.
        self.active = False
        self.ready_path.unlink(missing_ok=True)

    def update(self, observed):
        now = time.monotonic()
        if self.workflow_path.exists():
            with self.workflow_path.open() as stream:
                stream.seek(self.workflow_offset)
                self.workflow_tail += stream.read(); self.workflow_offset = stream.tell()
            lines = self.workflow_tail.split('\n'); self.workflow_tail = lines.pop()
            for line in lines:
                try: item = json.loads(line)
                except json.JSONDecodeError: continue
                if item.get('type') == 'phase': self.phase = item['key']
        hb, pos, landed = (observed.get(t) for t in ['HEARTBEAT','GLOBAL_POSITION_INT','EXTENDED_SYS_STATE'])
        fresh = hb and pos and landed and now-hb['received_mono']<=2 and now-pos['received_mono']<=.5 and now-landed['received_mono']<=2
        airborne = fresh and (hb['base_mode'] & 128) and landed['landed_state']==2
        if self.phase in ('prepare_hold', 'parity_precision_prepare_disabled') and self.phase != self.prepare_phase:
            if self.active:
                raise RuntimeError('previous fixture pilot window still active')
            self.prepare_phase = self.phase
            self.prepared = self.seen_loiter = False
        if self.phase==self.prepare_phase and not self.prepared:
            if not airborne or hb['custom_mode']!=4 or pos['relative_alt']<4000: return
            if self.sitl.poll() is not None: raise RuntimeError('owned SITL exited before pilot preparation')
            self.ownership = subprocess.run(['lsof','-nP','-a','-p',str(self.sitl.pid),'-iUDP'],capture_output=True,text=True)
            if self.ownership.returncode or not re.search(r'UDP\s+(?:\*|0\.0\.0\.0|127\.0\.0\.1):'+str(self.rc_port)+r'\b',self.ownership.stdout):
                raise RuntimeError('exact owned SITL RC socket not verified')
            self.trace.write(json.dumps({'event':'independent_airborne_ready','received_mono':now,
                'heartbeat':hb,'position':pos,'landed':landed,'owned_pid':self.sitl.pid,
                'owned_socket_snapshot':self.ownership.stdout})+'\n'); self.trace.flush()
            self.send(1500, 'prepare airborne pilot-neutral Loiter'); self.started = self.last_send
            self.active = self.prepared = True
        if not self.active: return
        if self.sitl.poll() is not None: raise RuntimeError('owned SITL exited during fixture pilot input')
        if not fresh: raise RuntimeError('independent pilot-fixture observation continuity lost')
        if hb['custom_mode']==5: self.seen_loiter = True
        if not airborne or self.phase in ['land','disarm','final'] or (self.seen_loiter and hb['custom_mode']!=5):
            self.restore('left Loiter or observed ground/disarmed'); return
        if now-self.last_send>=.05: self.send(1500, 'retain airborne neutral pilot throttle')
        # Preparation gives the native50Hz receiver several polls; later DataFlash
        # independently verifies actual RCIN before/during Loiter. Marker is not ACK.
        if now-self.started>=.15 and self.sent_neutral>=3:
            self.ready_path.write_text('neutral-ready\n')

    def close(self):
        try: self.restore('fixture exit/failure')
        finally: self.sock.close(); self.trace.close()


def select_run_dataflash_log(directory, before_names):
    candidates = sorted(path for path in directory.glob('*.BIN')
                        if path.is_file() and path.name not in before_names)
    if len(candidates) != 1:
        raise RuntimeError('expected exactly one newly created owned DataFlash BIN; found '+str(len(candidates)))
    return candidates[0]


def pilot_rcin_evidence(path):
    reader = mavutil.mavlink_connection(str(path))
    rc, modes, arms = [], [], []
    while True:
        message = reader.recv_match(type=['RCIN','MODE','ARM'])
        if message is None: break
        item = message.to_dict()
        {'RCIN':rc,'MODE':modes,'ARM':arms}[message.get_type()].append(item)
    begin = next(m['TimeUS'] for m in modes if m['ModeNum']==5)
    end = next(m['TimeUS'] for m in modes if m['TimeUS']>begin and m['ModeNum']!=5)
    armed = next(m['TimeUS'] for m in arms if m['ArmState']==1)
    neutral = [m for m in rc if m['C3']==1500]
    during = [m for m in rc if begin<=m['TimeUS']<end]
    after = [m for m in rc if m['TimeUS']>end and m['C3']==1000]
    if not neutral or neutral[0]['TimeUS']<=armed or neutral[0]['TimeUS']>=begin or not during or any(m['C3']!=1500 for m in during) or not after:
        raise RuntimeError('independent DataFlash did not corroborate scoped neutral RC3 and explicit restore')
    if any(m['C3']!=1000 for m in rc if m['TimeUS']<=armed):
        raise RuntimeError('pilot neutral throttle was present during ARM')
    return {'loiter_begin_TimeUS':begin,'loiter_end_TimeUS':end,'armed_TimeUS':armed,
            'first_neutral':neutral[0],'first_restored_after_loiter':after[0],
            'loiter_rcin_samples':len(during),'loiter_rc3_values':sorted({m['C3'] for m in during})}


def frame_flight_proof(hearts, positions, landed):
    returned = next(row for row in hearts if row['custom_mode'] == 6)
    touchdown = next(row for row in landed
                     if row['received_mono'] > returned['received_mono'] and row['landed_state'] == 1)
    guided = next(row for row in hearts
                  if row['received_mono'] > touchdown['received_mono'] and row['custom_mode'] == 4)
    final_land = next(row for row in hearts
                      if row['received_mono'] > guided['received_mono'] and row['custom_mode'] == 9)
    return_positions = [row for row in positions
                        if returned['received_mono'] <= row['received_mono'] <= touchdown['received_mono']]
    if len(return_positions) < 3:
        raise RuntimeError('independent Copter Return position interval is incomplete')

    def distance(row):
        north = (row['lat'] / 1e7 - 42.358456) * 111319.5
        east = (row['lon'] / 1e7 + 71.087589) * 82280.
        return (north*north + east*east)**.5

    travelled = distance(return_positions[0]) - min(map(distance, return_positions))
    if travelled < .75:
        raise RuntimeError('independent Copter Return did not reduce the distance home')
    return {'return_heartbeat': returned, 'return_touchdown': touchdown, 'guided_handback': guided,
            'final_land_heartbeat': final_land, 'return_homeward_m': travelled,
            'return_position_samples': len(return_positions)}


def main(*, installed=None):
    p = argparse.ArgumentParser()
    for name in ['ardupilot-root', 'moos-root', 'build', 'output']:
        p.add_argument('--' + name, required=True)
    p.add_argument('--behavior-dir', default=str(Path.home() / 'moos-ivp-uav-base/lib'))
    p.add_argument('--release-demo', action='store_true', help='exercise the Copter release operator scenario')
    p.add_argument('--pilot-guided-handback', action='store_true',
                   help='extend the operator scenario with pilot Guided before an explicit helm acquire')
    p.add_argument('--copter-frame', choices=tuple(COPTER_FRAMES), help='qualify an exact standard Copter frame')
    p.add_argument('--firmware-attestation', type=Path, help='attestation for the release demo Copter 4.6.3 binary')
    p.add_argument('--expected-app-revision', help='exact release demo source revision')
    p.add_argument('--public-stock', action='store_true',
                   help='run the quad starter with the pinned public stock MOOS checkout')
    args = p.parse_args()
    ap, moos, build, out = (Path(getattr(args, x)).resolve() for x in ['ardupilot_root', 'moos_root', 'build', 'output'])
    frame_case = args.copter_frame is not None
    sealed = args.release_demo or frame_case
    if args.pilot_guided_handback and (not args.release_demo or installed):
        p.error('--pilot-guided-handback requires the source-built release demo')
    if args.release_demo and frame_case:
        p.error('select either the release demo or a Copter frame')
    if args.public_stock and not args.release_demo:
        p.error('--public-stock requires the quad release demo')
    if installed and not (args.public_stock and args.expected_app_revision):
        p.error('installed bundle verification requires the public stock starter and an exact app revision')
    if sealed and not args.firmware_attestation:
        p.error('sealed Copter qualification requires --firmware-attestation')
    if frame_case and not args.expected_app_revision:
        p.error('frame qualification requires --expected-app-revision')
    if not sealed and (args.firmware_attestation or args.expected_app_revision):
        p.error('explicit attestation and source revision options require a sealed Copter case')
    out.mkdir(parents=True, exist_ok=not sealed)
    (out / 'logs').mkdir(exist_ok=True)
    before_dataflash = {path.name for path in (out/'logs').glob('*.BIN') if path.is_file()}
    root = Path(__file__).resolve().parents[1]
    behavior_dir = Path(args.behavior_dir).resolve()
    demo = None
    frame = args.copter_frame or 'quad'
    if sealed:
        from copter_release import (PilotTakeover, PROFILE, audit, exact_peer, frozen_inputs, inventory,
                                    logged_wind, original_files, profile_receipt)
        from pymavlink.dialects.v20 import ardupilotmega
        manifest = root/'config/firmware'/f'{PROFILE}.json'
        try:
            firmware = native_firmware_profile(root, ap, manifest, args.firmware_attestation)
            source, inputs, hashes = frozen_inputs(root, moos, build, behavior_dir, firmware,
                                                  args.expected_app_revision, args.copter_frame,
                                                  public_stock=args.public_stock, installed=installed)
        except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
            (out/'result.json').write_text(json.dumps({'result':'FAIL', 'preflight_error':str(error),
                                                      'hardware':False, 'resources_started':False},indent=2)+'\n')
            print(f'Copter release preflight failed: {error}', file=sys.stderr)
            return 1
        shutil.copyfile(inputs['firmware_manifest'], out/'firmware-manifest.json')
        shutil.copyfile(inputs['firmware_attestation'], out/'firmware-attestation.json')
    else:
        firmware = native_firmware_profile(root, ap)
    copter = ap / 'build/sitl/bin/arducopter'
    if not copter.is_file():
        raise RuntimeError('native ArduCopter SITL binary required')
    bridge = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    bridge.bind(('127.0.0.1', 0)); bridge.settimeout(.2)
    bridge_port = bridge.getsockname()[1]
    monitor = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    monitor.bind(('127.0.0.1', 0)); monitor.setblocking(False)
    monitor_port = monitor.getsockname()[1]
    db, base, rc = port(socket.SOCK_STREAM), port(socket.SOCK_STREAM), port(socket.SOCK_DGRAM)
    defaults = out / 'fixture.parm'
    fixture_defaults = 'SERIAL1_PROTOCOL 2\nMAV2_POSITION 10\nMAV2_EXT_STAT 2\nMAV2_EXTRA3 2\nLOG_DISARMED 1\n'
    if frame_case:
        fixture_defaults += 'RTL_ALT 650\n'
    if args.release_demo:
        demo = PilotTakeover(out, ardupilotmega, guided_handback=args.pilot_guided_handback)
        irlock = port(socket.SOCK_DGRAM)
        fixture_defaults += 'SERIAL2_PROTOCOL 2\nSIM_WIND_SPD 0\n'
        (out/'message-intervals-chan2.txt').write_text('0 1000\n')
    defaults.write_text(fixture_defaults)
    (out / 'message-intervals-chan1.txt').write_text('33 100\n245 500\n')
    defaults_files = [ap / 'Tools/autotest/default_params/copter.parm']
    if frame_case:
        defaults_files.append(inputs['frame_defaults'])
    defaults_files.append(defaults)
    command = [str(copter), '--model', frame, '--home', '42.358456,-71.087589,10,0', '--speedup', '1', '--wipe',
               '--defaults', ','.join(map(str, defaults_files)),
               '--serial0', f'udpclient:127.0.0.1:{bridge_port}', '--serial1', f'udpclient:127.0.0.1:{monitor_port}',
               '--serial2', f'udpclient:127.0.0.1:{demo.port}' if demo else 'none',
               '--serial5', 'none', '--serial6', 'none', '--serial7', 'none', '--serial8', 'none',
               '--base-port', str(base), '--rc-in-port', str(rc)]
    result = {'layer': 'native ArduCopter SITL + stock pHelmIvP/MOOSDB + dynamic iMAVLink modules',
              'timewarp': 1, 'hardware': False, 'firmware_identity': firmware, 'command': command, 'result': 'FAIL',
              'copter_sha256': sha(copter), 'app_sha256': sha(build / 'bin/iMAVLink'),
              'ardupilot_revision': firmware['source_revision'] if firmware else
              subprocess.check_output(['git', '-C', str(ap), 'rev-parse', 'HEAD'], text=True).strip(),
              'moos_revision': installed.manifest['moos_revision'] if installed else
              subprocess.check_output(['git', '-C', str(moos), 'rev-parse', 'HEAD'], text=True).strip(),
              'defaults_sha256': sha(ap / 'Tools/autotest/default_params/copter.parm'),
              'ports': {'bridge': bridge_port, 'monitor': monitor_port, 'db': db, 'base': base, 'rc': rc}}
    if sealed:
        result.update(source_revision=source, helm_sha256=hashes['pHelmIvP'], inputs_sha256=hashes)
    if frame_case:
        result['frame'] = frame
        result['frame_control_surface'] = 'standard_parity'
        result['frame_identity'] = {'expected_mav_type': COPTER_FRAMES[frame][0],
                                    'expected_frame_class': COPTER_FRAMES[frame][1]}
    if demo:
        command.extend(['--irlock-port', str(irlock)])
        result['ports'].update(operator=demo.port, irlock=irlock)
        if installed:
            result.update(scope='installed Linux ARM64 v1 bundle with stock helm',
                          fixture_revision=installed.fixture_revision)
        elif args.public_stock:
            result['scope'] = 'public-source Copter starter demonstration'
        else:
            result['release_case'] = 'copter_operator'
        if args.pilot_guided_handback:
            result['release_case'] = 'copter_pilot_guided_handback'
    sitl = launch = probe = pilot = None
    pilot_ready = out / "fixture-pilot-ready.txt"
    pilot_ready.unlink(missing_ok=True)
    handles = []
    packets = []
    try:
        log = (out / 'sitl.log').open('w'); handles.append(log)
        sitl = subprocess.Popen(command, cwd=out, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        decoder = mavutil.mavlink.MAVLink(None)
        decoder.robust_parsing = True
        deadline = time.monotonic() + 20
        peer = None
        while time.monotonic() < deadline:
            if sitl.poll() is not None:
                raise RuntimeError('SITL exited during startup')
            try:
                data, address = bridge.recvfrom(4096)
            except socket.timeout:
                continue
            for msg in decoder.parse_buffer(data) or []:
                if msg.get_type() == 'HEARTBEAT' and msg.get_srcSystem() == 1 and msg.get_srcComponent() == 1 and (
                        not frame_case or (msg.type, msg.autopilot) == (COPTER_FRAMES[frame][0], 3)):
                    peer = address
                    break
            if peer:
                break
        if peer is None or peer[0] != '127.0.0.1':
            raise RuntimeError('owned native SITL loopback peer not established')
        result['sitl_pid'] = sitl.pid
        result['ports']['peer'] = peer[1]
        # Preserve provenance of the actual UDP owner before enabling simulation effects.
        result['sitl_udp_sockets'] = subprocess.run(['lsof', '-nP', '-a', '-p', str(sitl.pid), '-iUDP'],
            capture_output=True, text=True).stdout
        if sealed and not exact_peer(result['sitl_udp_sockets'], peer, bridge_port):
            raise RuntimeError('bridge peer is not the exact owned simulator socket')
        bridge.close()
        macros = {'MOOS_PORT': str(db), 'APP_PATH': str(build / 'bin/iMAVLink'), 'BIND_PORT': str(bridge_port),
                  'PEER_PORT': str(peer[1]), 'LAT_ORIGIN': '42.358456', 'LONG_ORIGIN': '-71.087589',
                  'SIMULATION_ONLY': 'true', 'SHADOW_MODE': 'false', 'SITL_PID': str(sitl.pid),
                  'PLUGIN_DIRECTORY': str(build / 'plugins'), 'BEHAVIOR_DIRECTORY': str(Path(args.behavior_dir).resolve()),
                  'HEIGHT': '5', 'SPEED': '1', 'WAYPOINTS': '0,8:8,8:8,0:0,0'}
        env = dict(os.environ)
        if sealed:
            env = {key:value for key,value in env.items()
                   if not key.startswith(('DYLD_', 'LD_PRELOAD', 'LD_LIBRARY_PATH', 'MOOS'))}
            # pAntler looks up the hashed app first on PATH, including builds with spaces in their directory.
            macros['APP_PATH'] = 'iMAVLink'
        env['PATH'] = os.pathsep.join([str(build / 'bin'), str(moos / 'bin'), env.get('PATH', '')])
        env['IVP_BEHAVIOR_DIRS'] = macros['BEHAVIOR_DIRECTORY']
        env['MAVMOOS_SITL_FIXTURE_PILOT_READY'] = str(pilot_ready)
        for suffix in ['moos', 'bhv']:
            mission_dir = 'copter_release' if demo and suffix=='bhv' else 'copter_sitl'
            subprocess.run([str(moos / 'bin/nsplug'), str(root / f'missions/{mission_dir}/meta_vehicle.{suffix}'),
                            str(out / f'targ_copter.{suffix}'), '--strict', '--force', '-x'] +
                           [f'{k}={v}' for k, v in macros.items()], check=True, env=env, capture_output=True, timeout=10)
        if frame_case:
            mission = out / 'targ_copter.moos'
            text = mission.read_text()
            identity = '  expected_type = 2\n'
            if text.count(identity) != 1:
                raise RuntimeError('stock Copter mission identity anchor differs')
            text = text.replace(identity, f'  expected_type = {COPTER_FRAMES[frame][0]}\n'
                                f'  copter_frame = {frame}\n'
                                '  plugins = telemetry,arducopter_control\n'
                                '  firmware_requests = false\n', 1)
            mission.write_text(text)
        log = (out / 'pantler.log').open('w'); handles.append(log)
        launch = subprocess.Popen([str(moos / 'bin/pAntler'), str(out / 'targ_copter.moos')], cwd=out,
                                  env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        log = (out / 'workflow.log').open('w'); handles.append(log)
        probe_command = [str(inputs['probe'] if sealed else build / 'bin/sitl_workflow_probe'), str(db)]
        if demo:
            probe_command.append('--pilot-guided-handback' if args.pilot_guided_handback else '--release-demo')
        if frame_case:
            probe_command.append('--frame-lifecycle')
        probe = subprocess.Popen(probe_command, cwd=out,
                                 env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        decoder = mavutil.mavlink.MAVLink(None)
        decoder.robust_parsing = True
        result['independent_monitor_writes'] = 'none; owned SITL defaults and message-intervals-chan1.txt configure telemetry'
        pilot = FixturePilot(sitl, rc, pilot_ready, out/'workflow.log', out/'fixture-pilot.jsonl')
        observed = {}
        result['fixture_pilot_writes'] = 'native owned loopback RC input only; neutral RC3 after independently airborne readiness and before Loiter; explicit1000 restore'
        if demo:
            result['fixture_pilot_writes'] += '; Loiter mode request on the exact owned serial2 endpoint'
            if args.pilot_guided_handback:
                result['fixture_pilot_writes'] += ', followed by one Guided mode request'
        pinned_monitor_peer = None
        with (out / 'fc-monitor.jsonl').open('w') as trace, (out / 'fc-monitor.mavlink').open('wb') as raw:
            deadline = time.monotonic() + 200
            drain_until = None
            while time.monotonic() < deadline:
                if sealed and (sitl.poll() is not None or launch.poll() is not None):
                    raise RuntimeError('owned simulator or MOOS launcher exited during sealed Copter flight')
                if probe.poll() is not None:
                    if drain_until is None:
                        pilot.restore('workflow probe exited; observer drain only')
                        drain_until = time.monotonic()+1.5
                    if time.monotonic() >= drain_until:
                        break
                else:
                    pilot.update(observed)
                if demo:
                    demo.tick(pilot, observed)
                try:
                    data, monitor_peer = monitor.recvfrom(65535)
                except BlockingIOError:
                    time.sleep(.02); continue
                if sealed:
                    if pinned_monitor_peer is None:
                        proof = inventory(sitl)
                        if not exact_peer(proof, monitor_peer, monitor_port):
                            raise RuntimeError('independent monitor endpoint is not owned')
                        pinned_monitor_peer = monitor_peer
                        result['monitor_socket_proof'] = proof
                        result['ports']['monitor_peer'] = monitor_peer[1]
                    if monitor_peer!=pinned_monitor_peer or raw.tell()+len(data)>8*1024*1024:
                        raise RuntimeError('independent monitor endpoint changed or exceeded capture bound')
                raw.write(data)
                for msg in decoder.parse_buffer(data) or []:
                    if msg.get_srcSystem()==1 and msg.get_srcComponent()==1 and msg.get_type() in ['HEARTBEAT', 'GLOBAL_POSITION_INT', 'EXTENDED_SYS_STATE', 'STATUSTEXT']:
                        if frame_case and msg.get_type()=='HEARTBEAT' and (msg.type,msg.autopilot)!=(COPTER_FRAMES[frame][0],3):
                            raise RuntimeError('independent frame heartbeat identity changed')
                        item = {'received_mono': time.monotonic(), **msg.to_dict()}
                        trace.write(json.dumps(item) + '\n'); packets.append(item)
                        observed[msg.get_type()] = item
            result['monitor_finished_mono'] = time.monotonic()
            if probe.poll() is None:
                raise RuntimeError('workflow deadline exceeded')
        result['probe_exit'] = probe.returncode
        # The FC observer remains receive-only; assess height using its own time axis.
        latest_mode = None
        hold_positions = []
        for item in packets:
            if item['mavpackettype']=='HEARTBEAT': latest_mode = item['custom_mode']
            if item['mavpackettype']=='GLOBAL_POSITION_INT' and latest_mode==5:
                hold_positions.append(item)
        hold_heights = [item['relative_alt']/1000 for item in hold_positions]
        hold_span = hold_positions[-1]['received_mono']-hold_positions[0]['received_mono'] if hold_positions else 0
        result['independent_loiter_hold'] = {'samples':len(hold_heights),'observation_span_s':hold_span,
            'minimum_m':min(hold_heights,default=0),'maximum_m':max(hold_heights,default=0),
            'spread_m':max(hold_heights,default=0)-min(hold_heights,default=0)}
        positions = [m for m in packets if m['mavpackettype'] == 'GLOBAL_POSITION_INT']
        hearts = [m for m in packets if m['mavpackettype'] == 'HEARTBEAT']
        landed = [m for m in packets if m['mavpackettype'] == 'EXTENDED_SYS_STATE']
        valid_positions = [m for m in positions if abs(m['lat']/1e7-42.358456)<.001 and abs(m['lon']/1e7+71.087589)<.001]
        displacement=max([((m['lat']/1e7-42.358456)*111319.5)**2 +
                          ((m['lon']/1e7+71.087589)*82280.)**2 for m in valid_positions],default=0)**.5
        result['independent_monitor'] = {'position_samples': len(positions), 'heartbeat_samples': len(hearts),
            'maximum_relative_height_m': max([m['relative_alt'] / 1000 for m in positions], default=0),
            'observed_modes': sorted(set(m['custom_mode'] for m in hearts)),
            'observed_armed': any(m['base_mode'] & 128 for m in hearts),
            'maximum_home_displacement_m': displacement,
            'final_disarmed': bool(hearts) and not bool(hearts[-1]['base_mode'] & 128),
            'final_on_ground': bool(landed) and landed[-1]['landed_state']==1}
        if probe.returncode != 0:
            raise RuntimeError('MOOS workflow probe failed; see workflow.log')
        expected_modules = {"telemetry", "arducopter_control"}
        if not frame_case:
            expected_modules.add("parameters")
        result["capabilities_catalog"] = observed_catalog(out / "workflow.log", expected_modules, native=True,
            helm=True, mode="SITL_CONTROL")
        result["loaded_modules_sha256"] = loaded_module_hashes(build / "plugins", result["capabilities_catalog"])
        if len(hold_heights)<5 or hold_span<1 or result['independent_loiter_hold']['spread_m']>.5:
            raise RuntimeError('independent FC Loiter hold-height stability not corroborated')
        corroboration=result['independent_monitor']
        if not positions or corroboration['maximum_relative_height_m'] < 4 or not corroboration['observed_armed'] or \
           not {4,5,9}.issubset(corroboration['observed_modes']) or displacement<4 or \
           not corroboration['final_disarmed'] or not corroboration['final_on_ground']:
            raise RuntimeError('independent FC telemetry observer did not corroborate flight')
        if frame_case:
            result['frame_identity']['observed_mav_types'] = sorted({m['type'] for m in hearts})
            result['independent_frame_flight'] = frame_flight_proof(hearts, positions, landed)
        if sealed:
            if not all(result['monitor_finished_mono']-rows[-1]['received_mono']<=limit
                       for rows,limit in ((positions,.5), (hearts,2), (landed,2))):
                raise RuntimeError('final independent FC evidence is stale')
        if demo:
            if demo.channel.peer is not None:
                result['ports']['operator_peer'] = demo.channel.peer[1]
            result['proofs'] = audit(out, demo, packets, result)
        if installed:
            running = {Path(f"/proc/{row['pid']}/exe").resolve(strict=True)
                       for row in owned_records(launch) if Path(f"/proc/{row['pid']}/exe").is_symlink()}
            installed_processes = {name: moos / 'bin' / name for name in ('iMAVLink', 'pHelmIvP', 'MOOSDB')}
            if not set(installed_processes.values()) <= running:
                raise RuntimeError('running bridge, helm or MOOSDB differs from the installed runtime')
            result['installed_processes'] = {name: str(path) for name, path in installed_processes.items()}
        result['result'] = 'PASS'
    except Exception as error:
        result['error'] = str(error)
    finally:
        cleanup_remaining=[]
        if pilot is not None:
            try: pilot.close()
            except Exception as error:
                result['result']='FAIL'; result['pilot_cleanup_error']=str(error)
        if demo:
            if demo.channel.peer is not None:
                result['ports']['operator_peer'] = demo.channel.peer[1]
            demo.close()
        for process in [probe, launch, sitl]:
            cleanup_remaining.extend(stop(process))
        bridge.close(); monitor.close()
        for handle in handles:
            handle.close()
        clear = {}
        udp = ['bridge', 'monitor', 'rc', 'peer']
        if sealed:
            udp.append('monitor_peer')
        if demo:
            udp.extend(('operator', 'operator_peer', 'irlock'))
        for name in udp:
            if name not in result['ports']:
                continue
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                try:
                    s.bind(('127.0.0.1', result['ports'][name])); clear[name] = True
                except OSError:
                    clear[name] = False
        result['ports_clear'] = clear
        with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as tcp:
            tcp.settimeout(.2)
            result['ports_clear']['moos_tcp']=tcp.connect_ex(('127.0.0.1',db))!=0
        if sealed:
            with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as tcp:
                tcp.settimeout(.2)
                result['ports_clear']['base_tcp']=tcp.connect_ex(('127.0.0.1',base))!=0
        groups={p.pid for p in [probe,launch,sitl] if p is not None}
        result['owned_processes_remaining']=cleanup_remaining
        for line in subprocess.check_output(['ps','-axo','pid,pgid,comm'],text=True).splitlines()[1:]:
            fields=line.strip().split(None,2)
            if len(fields)==3 and int(fields[1]) in groups:
                result['owned_processes_remaining'].append(line.strip())
        if not all(clear.values()) or result['owned_processes_remaining']:
            result['result'] = 'FAIL'
        if demo:
            result['cleanup_proven'] = (all(clear.values()) and not result['owned_processes_remaining'] and
                all(name in clear for name in ('peer', 'monitor_peer', 'operator_peer', 'irlock', 'base_tcp', 'moos_tcp')))
            if not result['cleanup_proven']:
                result['result'] = 'FAIL'
        if frame_case:
            result['cleanup_proven'] = (all(clear.values()) and not result['owned_processes_remaining'] and
                set(clear) == {'bridge', 'monitor', 'monitor_peer', 'rc', 'peer', 'moos_tcp', 'base_tcp'})
            if not result['cleanup_proven']:
                result['result'] = 'FAIL'
        if result['result']=='PASS':
            try:
                selected = select_run_dataflash_log(out/'logs', before_dataflash)
                selected_sha = sha(selected)
                proof = pilot_rcin_evidence(selected)
                if frame_case:
                    from pymavlink import DFReader
                    reader = DFReader.DFReader_binary(str(selected))
                    parameters = {}
                    while (record := reader.recv_match(type='PARM')) is not None:
                        if record.Name in ('FRAME_CLASS', 'FRAME_TYPE'):
                            parameters[record.Name] = record.Value
                    if parameters != {'FRAME_CLASS': COPTER_FRAMES[frame][1], 'FRAME_TYPE': 0}:
                        raise RuntimeError('pinned Copter DataFlash frame parameters differ')
                    result['frame_identity']['dataflash_parameters'] = parameters
                if demo:
                    result['proofs']['in_flight_parameter_read']['dataflash_values'] = logged_wind(selected)
                if sha(selected) != selected_sha:
                    raise RuntimeError('selected owned DataFlash input changed during analysis')
                result['independent_pilot_rcin'] = {
                    'input_path':str(selected), 'input_sha256':selected_sha,
                    'prelaunch_BIN_names':sorted(before_dataflash), **proof}
            except Exception as error:
                result['result']='FAIL'; result['error']=str(error)
        native = out/'result.json'
        if sealed:
            try:
                reverified = native_firmware_profile(root, ap, manifest, args.firmware_attestation)
                _,_,after = frozen_inputs(root, moos, build, behavior_dir, reverified, source, args.copter_frame,
                                         public_stock=args.public_stock, installed=installed)
                if after!=hashes or firmware!=reverified or sha(out/'firmware-manifest.json')!=hashes['firmware_manifest'] or \
                   sha(out/'firmware-attestation.json')!=hashes['firmware_attestation']:
                    raise RuntimeError('source, runtime or firmware artifact changed during release demo')
                result['inputs_sha256_after'] = after
                result['files'] = original_files(out)
            except Exception as error:
                result.update(result='FAIL', postflight_error=str(error))
        native.write_text(json.dumps(result, indent=2) + '\n')
        if demo and not args.public_stock:
            (out/'profile-result.json').write_text(json.dumps(profile_receipt(result, native), indent=2)+'\n')
    print(json.dumps({'result':result['result'], 'evidence':str(out)} if sealed else result, indent=2))
    if result['result'] != 'PASS':
        for name in ['workflow.log', 'sitl.log', 'pantler.log']:
            if (out / name).exists():
                print(name, (out / name).read_text()[-4500:])
    return 0 if result['result'] == 'PASS' else 1


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(143)))
    import fcntl
    import tempfile
    with (Path(tempfile.gettempdir()) / 'mavmoos-native-sitl.lock').open('w') as lease:
        try:
            sealed = '--release-demo' in sys.argv or any(arg.startswith('--copter-frame') for arg in sys.argv)
            fcntl.flock(lease, fcntl.LOCK_EX | (fcntl.LOCK_NB if sealed else 0))
        except BlockingIOError:
            raise SystemExit('Another native MAVMOOS test owns the local simulator lease')
        raise SystemExit(main())
