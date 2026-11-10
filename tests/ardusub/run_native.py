#!/usr/bin/env python3
"""Pinned vectored ArduSub, a stock depth helm, and independently observed handbacks."""
import argparse
from contextlib import ExitStack
import errno
import fcntl
import json
import math
import os
from pathlib import Path
import select
import shlex
import signal
import socket
import subprocess
import sys
import tempfile
import traceback

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tests'))
from owned_processes import owned_records, snapshot, stop_process_tree
from plane_native.flight_fixture import Channel, exact_peer, inventory, json_data, log, need, now, reserve, revision, sha, start
from pymavlink import DFReader
from pymavlink.dialects.v20 import ardupilotmega as mav

FIRMWARES = {
    'baseline': {
        'source': '92b0cd788ec29406f26c6f9c31d5ceedbd1cc538',
        'label': 'ArduSub V4.6.0-dev',
        'simulator': '994970cc4d69145d537e4bae48c5ef8a5a254626c30c924151a1ab2b8cc3d5f8',
        'defaults': '4e859049a16e4577bdd3b09be0f7b7b2c7c086f1e15a03e93835c4f1b4aca728',
        'system_id': 'SYSID_THISMAV', 'gcs_id': 'SYSID_MYGCS', 'arming': ('ARMING_CHECK', 1),
        'guip_position_metres': 0.01,
    },
    'stable-4.7.2': {
        'source': 'fb773567896014da007bca58358704d4a0d8516f',
        'label': 'ArduSub V4.7.2',
        'simulator': '2f169160e96e284e9db770efd3b1b8a8bf89836c84013995bc2c6e623be0126d',
        'defaults': '4999326376ba407fada6933ca29f2a7b89ef655f076bbe9e6aac8267a194a786',
        'defaults_6dof': '1e82c14242c7c386bf2dd6bfede0be887ee82f5277bd2d0d5c8055d019cf61d8',
        'system_id': 'MAV_SYSID', 'gcs_id': 'MAV_GCS_SYSID', 'arming': ('ARMING_SKIPCHK', 0),
        'guip_position_metres': 1.,
    },
}
MOOS_PIN = '023fb54e00692611ecee2cd6f1138d10eec2127e'
HELM_SHA = '5b3d2e705609c37b5b5000384655b4491010d6d797b94c43830d9972e684a876'
RELEASE_APP_PIN = '4eb4e295acdbfd7f119040da49e4847d6e8dc337'
RELEASE_SHA = {
    'db': '31c247e5711c2d0f4037d2ff987eec23793870f6e90effb6d2ef765f88bd9861',
    'antler': '1e2692e4d440163c603442669af4516cd92f3f75efa4dec72da305f29eca71c7',
    'nsplug': 'da230cd7dbbdbe72835c43769cf60e86099026b387848159189e4835a472179c',
    'app': 'ca86f8c7a48493f5380b67e18813be958bec620cf43c207a1a6366e88af192cc',
    'probe': 'd3684e1b7e23517f4ee4168c39dafbb19c55cbdb57fa9ebdf8dd92b103a73c5f',
    'sub_plugin': '9f51832d68387f68991f8653fbe4b2c4bb175096d7cccb1a37617b714374fa39',
    'telemetry_plugin': '47dd214f1c045eaafea43eab65718b4acc603c75055bba252c1c8009de246682',
    'moos_template': '193ebe0388ebfb68998433e839940614e6aa4cce87722d9fa495d4a09b6bced8',
    'bhv_template': '2e3f0b74ea3994e83dc32c58839c6b7d283d564eee598b3cde790467ddfe9177',
}
HOME = (42.358456, -71.087589, 0, 0)
DEPTH_HOLD, GUIDED, SURFACE, POSITION_HOLD, MANUAL = 2, 4, 9, 16, 19
FRAMES = {'vectored': (1, 'sub.parm', 'defaults'),
          'vectored_6dof': (2, 'sub-6dof.parm', 'defaults_6dof')}
FIXTURE = {'FRAME_CONFIG': 1, 'SERIAL0_PROTOCOL': 2, 'SERIAL1_PROTOCOL': 2, 'SERIAL2_PROTOCOL': 2,
           'FS_PILOT_INPUT': 0, 'FS_GCS_ENABLE': 3, 'FS_GCS_TIMEOUT': 2.5}
INTERVALS = '0 1000\n1 250\n32 100\n33 100\n49 1000\n193 400\n242 500\n'


def fields(text):
    return dict(token.split('=', 1) for token in text.split(',') if '=' in token)


def local_difference(first, second):
    north = (second['lat']-first['lat'])*1e-7*111195.
    east = (second['lon']-first['lon'])*1e-7*111195.*math.cos(math.radians(first['lat']*1e-7))
    return north, east


def identity(args, profile):
    moos, firmware = args.moos_root.resolve(), args.ardupilot_root.resolve()
    _, defaults, defaults_pin = FRAMES[args.frame]
    files = {'simulator': firmware/'build/sitl/bin/ardusub', 'defaults': firmware/'Tools/autotest/default_params'/defaults,
             'helm': moos/'bin/pHelmIvP', 'db': moos/'bin/MOOSDB', 'antler': moos/'bin/pAntler',
             'nsplug': moos/'bin/nsplug', 'runner': Path(__file__).resolve()}
    need(revision(firmware, clean=False)==profile['source'] and revision(moos, clean=False)==MOOS_PIN,
         'selected stock firmware or MOOS checkout differs')
    for path in (firmware, moos):
        need(subprocess.run(['git', '-C', str(path), 'diff', 'HEAD', '--quiet'], timeout=15).returncode==0,
             f'tracked dependency drift: {path}')
    if not args.preflight:
        build = args.build.resolve()
        files.update({'app': build/'bin/iMAVLink', 'probe': build/'bin/ardusub_moos_probe',
                      'sub_plugin': build/'plugins/mavmoos_ardusub_control.so',
                      'telemetry_plugin': build/'plugins/mavmoos_telemetry.so',
                      'moos_template': ROOT/'tests/ardusub/meta_vehicle.moos',
                      'bhv_template': ROOT/'tests/ardusub/meta_vehicle.bhv'})
    if args.autonomy or args.stress:
        route = 'sub_stress.bhv' if args.stress else 'sub_transect.bhv'
        runner = 'stress_transect.py' if args.stress else 'autonomy_transect.py'
        files.update(probe=build/'bin/depth_autonomy_probe', evaluator=moos/'bin/pMissionEval',
                     bhv_template=ROOT/'missions/autonomy_sitl'/route,
                     mission_runner=ROOT/'tests/ardusub'/runner,
                     grader=ROOT/'tests/autonomy_missions/stock_grade.py')
    if args.stress:
        extension = '.dylib' if sys.platform == 'darwin' else '.so'
        files.update(tap=build/f'bin/libmavmoos_plane_fault_tap{extension}',
                     tap_helper=ROOT/'tests/plane_faults/fault_tap.py',
                     tap_source=ROOT/'tests/plane_faults/OwnedPlaneFaultTap.cpp',
                     monitor_source=ROOT/'tests/plane_native/flight_fixture.py')
    expected = {'simulator':profile['simulator'], 'defaults':profile[defaults_pin], 'helm':HELM_SHA}
    if args.surface:
        need(revision(ROOT, clean=True)==args.app_revision,'Surface candidate must name the current clean app revision')
    elif args.firmware_release != 'baseline':
        if not args.preflight:
            need(args.app_revision==RELEASE_APP_PIN,'stable comparison requires the exact baseline app revision')
        expected.update({name:digest for name,digest in RELEASE_SHA.items() if name in files})
    for name,digest in expected.items():
        need(sha(files[name])==digest,f'{name} differs from the exact qualification input')
    return files


class Probe:
    def __init__(self, paths, port, out, stack, arguments=()):
        error = stack.enter_context((out/'probe.stderr').open('x'))
        self.process = subprocess.Popen([str(paths['probe']), str(port), *arguments], cwd=out, stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, stderr=error, start_new_session=True)
        self.raw = stack.enter_context((out/'probe.stdout').open('xb'))
        self.trace = stack.enter_context((out/'moos.jsonl').open('x'))
        self.last, self.partial = {}, b''
        os.set_blocking(self.process.stdout.fileno(), False)

    def read(self):
        data = os.read(self.process.stdout.fileno(), 65536)
        need(bool(data) and self.raw.tell()+len(data)<=16*1024*1024, 'MOOS probe ended or exceeded byte limit')
        self.raw.write(data)
        lines = (self.partial + data).split(b'\n')
        self.partial = lines.pop()
        need(len(self.partial)<16384, 'MOOS probe line exceeds the bound')
        for line in lines:
            parts = line.lstrip(b'\r').decode('utf-8', 'replace').split('\t', 5)
            if len(parts)!=6 or parts[0] not in ('string', 'number', 'request'):
                continue
            kind, key, seen, published, source, value = parts
            row = {'kind':kind, 'key':key, 'mono':float(seen), 'published':float(published),
                   'source':source, 'value':float(value) if kind=='number' else value}
            log(self.trace, row)
            self.last[key] = row

    def post(self, key, value):
        mark = now()
        self.process.stdin.write(f'{key}\t{value}\n'.encode('ascii'))
        self.process.stdin.flush()
        return mark

    def match(self, key, after=0, **expected):
        row = self.last.get(key)
        return row if row and row['mono']>=after and all(fields(row['value']).get(k)==v for k,v in expected.items()) else None

    def runtime(self, after=0, **expected):
        return self.match('MAVMOOS_RUNTIME_STATUS', after, **expected)

    def navigation_count(self):
        return int(fields(self.last['MAVMOOS_RUNTIME_STATUS']['value'])['nav_tx_accepted'])


class Bench:
    def __init__(self, files, out, stack, report, profile, frame='vectored'):
        self.files, self.out, self.stack, self.report = files, out, stack, report
        self.frame = frame
        arming_name, arming_value = profile['arming']
        self.fixture = {profile['system_id']:1, profile['gcs_id']:245, **FIXTURE, arming_name:arming_value}
        self.fixture['FRAME_CONFIG'] = FRAMES[frame][0]
        self.fixture_readback = ('FRAME_CONFIG','FS_PILOT_INPUT','FS_GCS_ENABLE','FS_GCS_TIMEOUT',
                                 arming_name,profile['gcs_id'])
        self.channels = {name:Channel(name, stack.enter_context(reserve(socket.SOCK_DGRAM)), mav, stack, out)
                         for name in ('bridge', 'monitor', 'operator')}
        self.bridge, self.monitor, self.operator = (self.channels[name] for name in ('bridge', 'monitor', 'operator'))
        self.probe, self.app, self.antler, self.simulator = None, None, None, None
        self.paused = None
        self.proofs = stack.enter_context((out/'ownership.jsonl').open('x'))
        self.actions = stack.enter_context((out/'operator-tx.jsonl').open('x'))
        self.events = stack.enter_context((out/'events.jsonl').open('x'))
        self.encoder = mav.MAVLink(None, srcSystem=249, srcComponent=190)
        self.ports = {key:value.port for key,value in self.channels.items()}
        self.aux_endpoints = {name: stack.enter_context(reserve(kind,host)) for name,kind,host in (
            ('db',socket.SOCK_STREAM,'127.0.0.1'), ('base',socket.SOCK_STREAM,'127.0.0.1'),
            ('rc',socket.SOCK_DGRAM,'0.0.0.0'), ('irlock',socket.SOCK_DGRAM,'127.0.0.1'))}
        self.ports.update({name:endpoint.getsockname()[1] for name,endpoint in self.aux_endpoints.items()})
        report['ports'] = self.ports

    def mark(self, event, **values):
        log(self.events, {'event':event, 'mono':now(), **values})

    def start(self):
        fixture = self.out/'fixture.parm'
        fixture.write_text(''.join(f'{k} {v}\n' for k,v in self.fixture.items()))
        for channel in range(3):
            (self.out/f'message-intervals-chan{channel}.txt').write_text(INTERVALS)
        args = [self.files['simulator'], '--model',self.frame,'--home',','.join(map(str,HOME)),
                '--speedup','1','--wipe','--defaults',f'{self.files["defaults"]},{fixture}',
                '--base-port',self.ports['base'],'--rc-in-port',self.ports['rc'],
                '--irlock-port',self.ports['irlock']]
        for channel, name in enumerate(('bridge','monitor','operator')):
            args.extend([f'--serial{channel}',f'udpclient:127.0.0.1:{self.ports[name]}'])
        args.extend(['--serial3','GPS1','--serial4','GPS2'])
        for channel in range(5, 9):
            args.extend([f'--serial{channel}','none'])
        self.report['command'] = list(map(str,args))
        for name in ('base','rc','irlock'):
            self.aux_endpoints[name].close()
        self.simulator = start(args,self.out,'sitl.log')
        self.wait(22,lambda: all(ch.peer for ch in self.channels.values()) and self.heartbeat(armed=False),
                  'exact owned Sub endpoint or unarmed heartbeat absent')
        self.ports.update({f'{name}_peer':channel.peer[1] for name,channel in self.channels.items()})
        self.wait(85,self.sensor_ready,'native Sub pressure, estimator or surface reference never became ready')
        self.report['boot_observations'] = {kind:self.sample(kind) for kind in
                                            ('HEARTBEAT','SYS_STATUS','EKF_STATUS_REPORT','HOME_POSITION','GLOBAL_POSITION_INT')}

    def pump(self, delay):
        need(self.simulator.poll() is None, 'owned Sub process exited')
        channels = {c.endpoint:c for c in self.channels.values() if c.endpoint.fileno()>=0}
        readers = list(channels)
        if self.probe:
            readers.append(self.probe.process.stdout)
        selected, _, _ = select.select(readers, [], [], delay)
        for endpoint in selected:
            if endpoint in channels:
                channels[endpoint].read(self.simulator,self.proofs)
            else:
                self.probe.read()

    def wait(self, seconds, check, reason):
        deadline = now()+seconds
        while now()<deadline:
            self.pump(min(.035,max(0.,deadline-now())))
            observation = check()
            if observation:
                return observation
        raise RuntimeError(reason)

    def dwell(self, seconds):
        deadline = now()+seconds
        while now()<deadline:
            self.pump(min(.035,max(0.,deadline-now())))

    def sample(self, message, after=0, age=2.):
        value = self.monitor.latest.get(message)
        return value if value and after<=value['mono'] and now()-value['mono']<=age else None

    def heartbeat(self, mode=None, armed=None, after=0):
        row = self.sample('HEARTBEAT',after)
        return row if row and row['type']==mav.MAV_TYPE_SUBMARINE and row['autopilot']==mav.MAV_AUTOPILOT_ARDUPILOTMEGA and \
            (mode is None or row['custom_mode']==mode) and (armed is None or bool(row['base_mode']&128)==armed) else None

    def sensor_ready(self):
        pressure, estimator = self.sample('SYS_STATUS'), self.sample('EKF_STATUS_REPORT')
        bit = mav.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
        return pressure and estimator and self.sample('HOME_POSITION') and self.sample('GLOBAL_POSITION_INT') and \
            all(pressure[f'onboard_control_sensors_{field}'] & bit for field in ('present','enabled','health')) and \
            (estimator['flags']&51)==51 and not estimator['flags'] & (128|1024|32768)

    def send(self, message):
        proof = inventory(self.simulator)
        need(exact_peer(proof,self.operator.peer,self.operator.port),'operator route no longer belongs to the exact simulator')
        packet = message.pack(self.encoder)
        self.encoder.seq = (self.encoder.seq+1)%256
        at = now()
        need(self.operator.endpoint.sendto(packet,self.operator.peer)==len(packet),'fixture command short send')
        log(self.actions, {'mono':at, 'peer':self.operator.peer, 'bytes':packet.hex(), **message.to_dict()})
        return at

    def mode(self, mode):
        need(mode in (POSITION_HOLD,MANUAL),'fixture mode outside its policy')
        at = self.send(self.encoder.command_long_encode(1,1,mav.MAV_CMD_DO_SET_MODE,0,1,mode,0,0,0,0,0))
        return self.wait(5,lambda:self.heartbeat(mode,after=at),f'independent fixture mode {mode} not observed')

    def arm(self, requested):
        if requested:
            self.wait(12,lambda:self.sample('SYS_STATUS') and self.sample('SYS_STATUS')['onboard_control_sensors_health']&
                      mav.MAV_SYS_STATUS_PREARM_CHECK,'ordinary Sub prearm check not healthy')
        at = self.send(self.encoder.command_long_encode(1,1,mav.MAV_CMD_COMPONENT_ARM_DISARM,0,
                                                        int(requested),0,0,0,0,0,0))
        return self.wait(6,lambda:self.heartbeat(armed=requested,after=at),'ordinary Sub arm/disarm not observed')

    def read_parameters(self):
        values = {}
        for name in self.fixture_readback:
            at = self.send(self.encoder.param_request_read_encode(1,1,name.encode(),-1))
            def fresh():
                return next((row for row in reversed(self.operator.messages) if row['mono']>=at and
                             row['mavpackettype']=='PARAM_VALUE' and row['param_id']==name), None)
            row = self.wait(2,fresh,f'pinned fixture parameter {name} unavailable')
            need(abs(row['param_value']-self.fixture[name])<=.001,f'pinned fixture parameter {name} differs')
            values[name] = row
        return values

    def start_moos(self):
        self.bridge.endpoint.close()
        options = {'MOOS_PORT': self.ports['db'], 'DB_PATH': self.files['db'], 'APP_PATH':self.files['app'],
                   'HELM_PATH':self.files['helm'], 'PLUGIN_DIRECTORY':self.files['sub_plugin'].parent,
                   'LAT_ORIGIN':HOME[0], 'LONG_ORIGIN':HOME[1], 'BIND_PORT':self.bridge.port,
                   'PEER_PORT':self.bridge.peer[1], 'SITL_PID':self.simulator.pid}
        env = dict(os.environ)
        env.pop('IVP_BEHAVIOR_DIRS',None)
        for suffix in ('moos','bhv'):
            subprocess.run([str(self.files['nsplug']),str(self.files[suffix+'_template']),
                            str(self.out/f'targ_sub.{suffix}'),'--strict','--force','-x']+
                           [f'{key}={value}' for key,value in options.items()],env=env,check=True,timeout=8,
                           capture_output=True,text=True)
        target = self.out/'targ_sub.moos'
        if 'evaluator' in self.files:
            from autonomy_missions.stock_grade import with_evaluator
            target.write_text(with_evaluator(target.read_text(), self.files['evaluator']))
        if 'tap' in self.files:
            from plane_faults import fault_tap
            roster = f"  Run = {self.files['app']} @ NewConsole = false ~ iMAVLink\n"
            config = target.read_text()
            need(config.count(roster) == 1, 'could not isolate the owned bridge for wire capture')
            target.write_text(config.replace(roster, '', 1))
            env = fault_tap.clean_environment()
            env.pop('IVP_BEHAVIOR_DIRS', None)
            trace, control = self.out/'app-wire.plf', self.out/'app-tap-mode.txt'
            fault_tap.set_mode(control, 'pass')
            app_env = env | fault_tap.environment(self.files['tap'], self.bridge.port, self.bridge.peer[1],
                                                   trace, control)
        self.aux_endpoints['db'].close()
        self.antler = start([self.files['antler'], target],self.out,'pantler.log',env)
        if 'tap' in self.files:
            self.app = start([self.files['app'], target, 'iMAVLink'], self.out, 'imavlink.log', app_env)
            self.report['bridge_pid'] = self.app.pid
        self.probe = Probe(self.files,self.ports['db'],self.out,self.stack)
        self.wait(9,lambda:self.probe.match('MAVMOOS_SUB_STATUS',ready='true') and
                  self.probe.runtime(control_profile='ardusub',permission='false',fc_mode=str(POSITION_HOLD)),
                  'owned ArduSub bridge did not become ready in native Position Hold')
        self.probe.post('DEPLOY','true')
        self.wait(5,lambda:self.probe.last.get('IVPHELM_STATE',{}).get('value')=='DRIVE' and
                  self.probe.last.get('DESIRED_DEPTH') and
                  self.probe.last.get('IVPHELM_ALLSTOP',{}).get('value')=='clear','stock depth helm did not drive')

    def acquire(self, *, route=False, depth=2.):
        count = self.probe.navigation_count()
        started = now()
        native_hold = self.wait(3,lambda:self.heartbeat(POSITION_HOLD,True,started),
                                'fresh independent armed Position Hold absent before acquire')

        def bridge_hold():
            return self.probe.runtime(native_hold['mono'],permission='false',fc_mode=str(POSITION_HOLD),
                                      armed='1',heartbeat_valid='1')

        bridge = self.wait(3,bridge_hold,'bridge did not observe armed Position Hold before acquire')

        def sub_ready():
            if bridge_hold() and self.heartbeat(POSITION_HOLD,True,native_hold['mono']):
                return self.probe.match('MAVMOOS_SUB_STATUS',bridge['mono'],ready='true',pressure_healthy='true')

        readiness = self.wait(3,sub_ready,'bridge lacked fresh Sub readiness after Position Hold')
        need(self.probe.navigation_count()==count,'navigation sent while waiting for explicit acquire')
        at = self.probe.post('MAVMOOS_NAV_CONTROL','acquire')
        heartbeat = self.wait(5,lambda:self.heartbeat(GUIDED,True,at),'independent Guided acquisition absent')
        def motion():
            row = self.probe.runtime(at,permission='true',fc_mode=str(GUIDED))
            return row if row and self.probe.navigation_count()>count else None
        status = self.wait(5,motion,'fresh stock depth input did not reach the confirmed Guided route')
        scalars = {key:self.probe.last[key] for key in ('DESIRED_HEADING','DESIRED_SPEED','DESIRED_DEPTH')}
        need(all(row['source']=='pHelmIvP' for row in scalars.values()) and
             ((scalars['DESIRED_DEPTH']['value'] == depth and 0 < scalars['DESIRED_SPEED']['value'] <= .8)
              if route else [row['value'] for row in scalars.values()]==[90.,.7,2.]),
             'unexpected stock depth helm values')
        return {'requested':at,'heartbeat':heartbeat,'runtime':status,'stock_scalars':scalars,
                'position_hold_before_request':{'native':native_hold,'bridge':bridge,'sub':readiness}}

    def pause(self, name):
        owner = self.app if self.app and name == 'iMAVLink' else self.antler
        rows = [row for row in owned_records(owner)
                if Path(shlex.split(row['command'])[0]).name==name and str(self.out/'targ_sub.moos') in shlex.split(row['command'])]
        need(len(rows)==1,'owned pause target ambiguous')
        row = rows[0]
        current = snapshot([row['pid']])
        need(current.get(row['pid'])==row,'owned pause target changed')
        self.paused = row
        at = now()
        os.kill(row['pid'],signal.SIGSTOP)
        self.mark('paused_owned',process=row)
        return at

    def resume(self):
        if self.paused:
            row = self.paused
            need(snapshot([row['pid']]).get(row['pid'])==row,'owned resume target changed')
            os.kill(row['pid'],signal.SIGCONT)
            self.paused = None
            self.mark('resumed_owned',process=row)

    def close(self):
        self.resume()
        remaining = []
        for child in (self.probe.process if self.probe else None,self.app,self.antler,self.simulator):
            remaining.extend(stop_process_tree(child))
        self.report['owned_processes_remaining'] = remaining
        for channel in self.channels.values():
            channel.endpoint.close()


def exercise(bench):
    report, probe = bench.report, bench.probe
    probe_count = probe.navigation_count()
    need(probe_count==0,'setpoint accepted before explicit acquisition')
    report['acquisition'] = bench.acquire()
    home = bench.sample('HOME_POSITION')
    start_at = bench.sample('GLOBAL_POSITION_INT')
    def transit():
        position = bench.sample('GLOBAL_POSITION_INT',start_at['mono'],.5)
        if not position:
            return None
        north,east = local_difference(start_at,position)
        depth = (home['altitude']-position['alt'])/1000.
        if east>=3 and abs(north)<=2 and 1.65<=depth<=2.35:
            return {'from':start_at,'to':position,'north_m':north,'east_m':east,'depth_m':depth,'home':home}
        return None
    report['transit'] = bench.wait(22,transit,'independent Sub travel or surface-relative depth target not reached')

    lost = bench.pause('pHelmIvP')
    latched = bench.wait(3,lambda:probe.runtime(lost,permission='false'),'silent helm grant did not expire')
    count = probe.navigation_count()
    bench.dwell(2.3)
    hold_start = bench.sample('GLOBAL_POSITION_INT',lost,.5)
    need(hold_start and bench.heartbeat(GUIDED,True),'independent Guided hold state missing')
    bench.resume()
    bench.wait(3,lambda:probe.last.get('DESIRED_DEPTH',{}).get('mono',0)>lost,'resumed stock depth mail absent')
    bench.dwell(1.5)
    hold_end = bench.sample('GLOBAL_POSITION_INT',hold_start['mono'],.5)
    need(hold_end and probe.navigation_count()==count,'stale grant resumed on recovered helm messages')
    drift = math.hypot(*local_difference(hold_start,hold_end))
    need(drift<.65 and abs(hold_end['alt']-hold_start['alt'])<300,'captured hold drifted after helm silence')
    report['helm_silence'] = {'started':lost,'revoked':latched,'from':hold_start,'to':hold_end,'drift_m':drift}

    report['fixture_native_hold'] = bench.mode(POSITION_HOLD)
    report['second_acquisition'] = bench.acquire()
    hold_request = 'schema=1,id=operator-hold,action=hold'
    at = probe.post('MAVMOOS_SUB_HANDBACK',hold_request)
    report['operator_handback'] = bench.wait(4,lambda:probe.match('MAVMOOS_SUB_HANDBACK_RESULT',at,status='CONFIRMED'),
                                              'bridge native handback confirmation unavailable')
    need(bench.heartbeat(POSITION_HOLD,True,at) and probe.runtime(at,permission='false'),
         'handback lacked independent mode or grant retirement')
    replay_at = probe.post('MAVMOOS_SUB_HANDBACK',hold_request)
    report['operator_handback_replay'] = bench.wait(4,lambda:probe.match(
        'MAVMOOS_SUB_HANDBACK_RESULT',replay_at,id='operator-hold',status='DUPLICATE',reason='native_id_already_recorded'),
        'same operator hold ID was not fenced')

    report['third_acquisition'] = bench.acquire()
    lost = bench.pause('iMAVLink')
    report['native_link_loss'] = bench.wait(7,lambda:bench.heartbeat(DEPTH_HOLD,True,lost),
                                           'configured native Depth Hold after bridge silence not observed')
    bench.resume()
    retired = bench.wait(4,lambda:probe.runtime(lost,permission='false',fc_mode=str(DEPTH_HOLD)),
                         'bridge resumed old grant after route silence')
    report['bridge_after_link_loss'] = retired

    bench.mode(POSITION_HOLD)
    report['fourth_acquisition'] = bench.acquire()
    takeover = bench.mode(MANUAL)
    retired = bench.wait(3,lambda:probe.runtime(takeover['mono'],permission='false',fc_mode=str(MANUAL)),
                         'external Manual takeover did not retire control')
    count = probe.navigation_count()
    bench.dwell(.9)
    need(probe.navigation_count()==count and bench.heartbeat(MANUAL,True),'helmed targets resumed after takeover')
    report['pilot_takeover'] = {'native':takeover,'retired':retired,'nav_counter_frozen':count}


def submerged_position(bench, home):
    def in_band():
        position = bench.sample('GLOBAL_POSITION_INT', age=.5)
        return position if position and 1.4 <= (home['altitude'] - position['alt']) / 1000 <= 2.6 else None

    return bench.wait(18, in_band, 'Sub was not submerged under stock depth control')


def exercise_release(bench):
    probe = bench.probe
    bench.mode(POSITION_HOLD)
    acquired = bench.acquire()
    home = bench.sample('HOME_POSITION')
    submerged = submerged_position(bench, home)
    at = probe.post('MAVMOOS_NAV_CONTROL', 'release')
    retired = bench.wait(3, lambda: probe.runtime(at, permission='false'),
                         'explicit release did not retire Sub helm control')
    first = bench.wait(3, lambda: bench.sample('GLOBAL_POSITION_INT', at, .5),
                       'no independent position after explicit release')
    bench.dwell(1.2)
    count = probe.navigation_count()
    last = bench.wait(3, lambda: bench.sample('GLOBAL_POSITION_INT', first['mono'], .5),
                      'no independent Sub hold observation')
    depth_mail = probe.last.get('DESIRED_DEPTH', {})
    need(depth_mail.get('source') == 'pHelmIvP' and depth_mail.get('mono', 0) > at and
         probe.runtime(at, permission='false') and bench.heartbeat(GUIDED, True),
         'release did not leave continued stock depth output fenced in Guided hold')
    bench.dwell(1.2)
    final = bench.sample('GLOBAL_POSITION_INT', last['mono'], .5)
    need(final and probe.navigation_count() == count and
         math.hypot(*local_difference(last, final)) < .65 and abs(final['alt'] - last['alt']) < 300,
         'fresh helm output moved the Sub after explicit release')
    bench.report['explicit_release'] = {'acquired': acquired, 'started': at, 'submerged': submerged, 'retired': retired,
                                        'stock_depth_after_release': depth_mail,
                                        'hold_from': last, 'hold_to': final, 'nav_count': count}


def exercise_surface_takeover(bench):
    probe = bench.probe
    bench.mode(POSITION_HOLD)
    bench.acquire()
    home = bench.sample('HOME_POSITION')

    submerged_position(bench, home)
    request = 'schema=1,id=interrupted-surface,action=surface'
    at = probe.post('MAVMOOS_SUB_HANDBACK', request)
    entered = bench.wait(3, lambda: bench.heartbeat(SURFACE, True, at),
                         'Surface did not start before takeover')
    bench.wait(2, lambda: probe.match('MAVMOOS_SUB_HANDBACK_RESULT', at, id='interrupted-surface',
                                    status='CONFIRMED'), 'Surface entry was not confirmed')
    before = bench.sample('GLOBAL_POSITION_INT', age=.5)
    need(before and (home['altitude'] - before['alt']) / 1000 > .35,
         'Surface interruption must precede the shallow completion band')
    pilot = bench.mode(MANUAL)
    depth_sample = bench.wait(1.5, lambda: bench.sample('GLOBAL_POSITION_INT', pilot['mono'], .5),
                              'no independent depth at Manual takeover')
    depth_at_takeover = (home['altitude'] - depth_sample['alt']) / 1000
    need(depth_at_takeover > .35, 'pilot Manual did not precede the shallow completion band')
    preempted = bench.wait(3, lambda: probe.match('MAVMOOS_SUB_HANDBACK_RESULT', pilot['mono'] - 1,
                                                id='interrupted-surface', status='PREEMPTED'),
                           'Surface was not preempted by Manual')
    count = probe.navigation_count()
    replay_at = probe.post('MAVMOOS_SUB_HANDBACK', request)
    duplicate = bench.wait(3, lambda: probe.match('MAVMOOS_SUB_HANDBACK_RESULT', replay_at,
                                                id='interrupted-surface', status='DUPLICATE'),
                           'interrupted Surface ID could be submitted again')
    bench.dwell(.8)
    need(bench.heartbeat(MANUAL, True) and probe.navigation_count() == count,
         'old Surface or helm output disturbed pilot control')
    hold = bench.mode(POSITION_HOLD)
    fresh = bench.acquire()
    bench.report['surface_takeover'] = {'entered': entered, 'depth_at_takeover_m': depth_at_takeover,
        'pilot_manual': pilot, 'takeover_depth_sample': depth_sample, 'preempted': preempted,
        'duplicate': duplicate, 'recovery_hold': hold, 'fresh_acquisition': fresh}



def exercise_surface(bench):
    probe, report = bench.probe, bench.report
    at = probe.post('MAVMOOS_SUB_HANDBACK','surface')
    rejected = bench.wait(3,lambda:probe.match('MAVMOOS_SUB_HANDBACK_RESULT',at,status='REJECTED'),
                          'Surface request was not rejected under Manual takeover')
    bench.mode(POSITION_HOLD)
    acquired = bench.acquire()
    home = bench.sample('HOME_POSITION')

    def submerged():
        position = bench.sample('GLOBAL_POSITION_INT',age=.5)
        return position if position and 1.4 <= (home['altitude']-position['alt'])/1000 <= 2.6 else None

    start = bench.wait(15,submerged,'could not establish fresh submerged starting depth')
    at = probe.post('MAVMOOS_SUB_HANDBACK','surface')
    retired = bench.wait(3,lambda:probe.runtime(at,permission='false'),'Surface did not retire helm')
    stopped_count = probe.navigation_count()
    entered = bench.wait(5,lambda:bench.heartbeat(SURFACE,True,at),'independent native Surface entry absent')
    confirmed = bench.wait(3,lambda:probe.match('MAVMOOS_SUB_HANDBACK_RESULT',at,status='CONFIRMED'),
                           'app did not confirm fresh Surface mode')
    ended = bench.wait(40,lambda:bench.heartbeat(DEPTH_HOLD,True,entered['mono']),
                       'Surface did not hand off into native Depth Hold')
    completed = bench.wait(6,lambda:probe.match('MAVMOOS_SUB_HANDBACK_RESULT',at,status='COMPLETED'),
                           'app did not confirm ascent and shallow Depth Hold')
    position = bench.sample('GLOBAL_POSITION_INT',ended['mono'],.5)
    need(position is not None and -.6 <= (home['altitude']-position['alt'])/1000 <= .35 and
         position['alt']-start['alt'] >= 1000 and probe.navigation_count()==stopped_count,
         'independent ascent, surface depth or post-retirement setpoint fence differs')
    report['surface'] = {'manual_rejection':rejected,'acquired':acquired,'retired':retired,'entered':entered,
                         'confirmed':confirmed,'native_depth_hold':ended,'completed':completed,
                         'initial_depth_m':(home['altitude']-start['alt'])/1000,
                         'final_depth_m':(home['altitude']-position['alt'])/1000,
                         'ascended_m':(position['alt']-start['alt'])/1000,'nav_count':stopped_count}


def verify_captured_hold(bench, position_metres, explicit_release=False):
    # GUIP is the FC's record of a received and accepted Guided position target.
    files = sorted((bench.out/'logs').glob('*.BIN'))
    need(len(files)==1,'unique onboard FC log unavailable')
    reader = DFReader.DFReader_binary(str(files[0]))
    boundary = bench.report['explicit_release' if explicit_release else 'helm_silence']
    end = boundary['hold_from' if explicit_release else 'from']['mono']
    observations = [row for row in bench.monitor.messages if row['mavpackettype']=='LOCAL_POSITION_NED'
                    and boundary['started'] - (.25 if explicit_release else 0) <= row['mono'] <= end]
    need(bool(observations),'native position missing around the captured hold')
    first = observations[0]['time_boot_ms']*1000
    last = observations[-1]['time_boot_ms']*1000
    targets = []
    while message := reader.recv_match(type='GUIP'):
        target = message.to_dict()
        if first<=target['TimeUS']<=last and (not explicit_release or target['Type']==0):
            targets.append(target)
    need(len(targets)==1,'FC log lacks exactly one accepted position capture')
    target = targets[0]
    position = min(observations,key=lambda row:abs(row['time_boot_ms']*1000-target['TimeUS']))
    capture_error = math.dist((target['pX']*position_metres,target['pY']*position_metres,
                               -target['pZ']*position_metres),
                              (position['x'],position['y'],position['z']))
    need(target['Type']==0 and all(target[key]==0 for key in ('vX','vY','vZ')) and
         abs(position['time_boot_ms']*1000-target['TimeUS'])<=200000 and capture_error<.25,
         'FC did not capture the current local horizontal position and depth')
    return {'file':str(files[0].relative_to(bench.out)),'target':target,'position':position,'error_m':capture_error}


def ports_clear(ports):
    clear = {}
    for name, port in ports.items():
        tcp = name in ('db','base')
        with socket.socket(socket.AF_INET,socket.SOCK_STREAM if tcp else socket.SOCK_DGRAM) as endpoint:
            try:
                if tcp:
                    endpoint.settimeout(.25)
                    clear[name] = endpoint.connect_ex(('127.0.0.1',port))==errno.ECONNREFUSED
                else:
                    endpoint.bind(('0.0.0.0' if name=='rc' else '127.0.0.1',port))
                    clear[name] = True
            except OSError:
                clear[name] = False
    return clear


def main(args):
    out = args.output.resolve()
    out.mkdir(mode=0o700,parents=True,exist_ok=False)
    profile = FIRMWARES[args.firmware_release]
    scope = ('stock depth transect with native hold, Manual, watchdog recovery and Surface' if args.stress else
             'fixture preflight' if args.preflight else
             'stock depth helm, explicit Guided and handback, stale helm, link loss, and native takeover')
    report = {'result':'FAIL','scope':'pinned native ArduSub SITL; '+scope,
              'frame':args.frame,'surface_requested':args.surface,'stress_requested':args.stress,
              'hardware':False,'firmware_source':profile['source'],'firmware_label':profile['label'],
              'moos_source':MOOS_PIN,'fixture_source':revision(ROOT,clean=False),'app_source':args.app_revision}
    files, bench = {}, None
    try:
        files = identity(args,profile)
        report['inputs'] = {name:{'path':str(path),'sha256':sha(path)} for name,path in files.items()}
        with ExitStack() as stack:
            lease = stack.enter_context((Path(tempfile.gettempdir())/'mavmoos-native-sitl.lock').open('a+'))
            fcntl.flock(lease.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
            bench = Bench(files,out,stack,report,profile,args.frame)
            try:
                bench.start()
                report['parameters'] = bench.read_parameters()
                report['initial_hold'] = bench.mode(POSITION_HOLD)
                report['arm'] = bench.arm(True)
                if not args.preflight:
                    bench.start_moos()
                    if args.stress:
                        from stress_transect import stress
                        stress(bench)
                    elif args.autonomy:
                        from autonomy_transect import transect
                        report['autonomy'] = transect(bench)
                    elif args.release_and_takeover:
                        exercise_release(bench)
                        exercise_surface_takeover(bench)
                        bench.mode(MANUAL)
                    else:
                        exercise(bench)
                    if args.surface and not (args.autonomy or args.stress):
                        exercise_surface(bench)
                report['disarm'] = bench.arm(False)
                if args.autonomy or args.stress:
                    from autonomy_missions.stock_grade import grade
                    mission = 'stress' if args.stress else 'autonomy'
                    report[mission]['grade'] = grade(bench.probe.post, lambda check: bench.wait(5, check,
                        'stock Sub mission grade absent'), out)
            finally:
                bench.close()
            if not (args.stress or args.preflight or args.autonomy):
                report['captured_hold_readback'] = verify_captured_hold(bench,profile['guip_position_metres'],args.release_and_takeover)
        if args.stress:
            from stress_transect import audit
            report['stress_wire_audit'] = audit(out, report)
        report['result'] = 'PASS'
    except BaseException as error:
        report['error'] = str(error) or type(error).__name__
        (out/'failure.log').write_text(traceback.format_exc())
    try:
        report['ports_clear'] = ports_clear(report.get('ports',{}))
        report['cleanup_proven'] = bool(report['ports_clear']) and all(report['ports_clear'].values()) and \
            report.get('owned_processes_remaining')==[]
        report['input_drift'] = [key for key,path in files.items() if sha(path)!=report['inputs'][key]['sha256']]
        need(report['cleanup_proven'] and not report['input_drift'],'scoped cleanup or input immutability was not proven')
    except BaseException as error:
        report['result'] = 'FAIL'
        report['postflight_error'] = str(error)
    report['files'] = {str(path.relative_to(out)):sha(path) for path in sorted(out.rglob('*')) if path.is_file()}
    (out/'result.json').write_text(json.dumps(json_data(report),indent=2,allow_nan=False)+'\n')
    print(report['result'],out/'result.json')
    return 0 if report['result']=='PASS' else 1


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for arg in ('ardupilot-root','moos-root','output'):
        parser.add_argument('--'+arg,type=Path,required=True)
    parser.add_argument('--build',type=Path)
    parser.add_argument('--app-revision')
    parser.add_argument('--firmware-release',choices=FIRMWARES,default='baseline')
    parser.add_argument('--frame',choices=FRAMES,default='vectored')
    parser.add_argument('--surface',action='store_true',help='qualify native ascent on the exact current stable-release build')
    parser.add_argument('--release-and-takeover', action='store_true',
                        help='qualify explicit release and pilot interruption during native Surface')
    parser.add_argument('--autonomy',action='store_true',help='run the stock depth-changing transect')
    parser.add_argument('--stress',action='store_true',
                        help='run the stock transect with native handback, pilot takeover and link loss')
    parser.add_argument('--preflight',action='store_true')
    args = parser.parse_args()
    if not args.preflight and (not args.build or not args.app_revision or len(args.app_revision)!=40):
        parser.error('normal native run requires the build and exact app revision')
    if args.autonomy and (not args.surface or args.release_and_takeover):
        parser.error('--autonomy requires stable --surface and a separate mission run')
    if args.stress and (not args.surface or args.autonomy or args.release_and_takeover or args.frame != 'vectored'):
        parser.error('--stress requires a separate --surface run on the pinned vectored frame')
    if args.release_and_takeover and not args.surface:
        parser.error('--release-and-takeover requires the exact stable Surface fixture')
    if args.surface and (args.firmware_release!='stable-4.7.2' or args.preflight):
        parser.error('Surface candidate requires stable-4.7.2 and a current application build')
    if args.frame!='vectored' and not args.surface:
        parser.error('alternate frames require the explicit stable Surface candidate workflow')
    raise SystemExit(main(args))
