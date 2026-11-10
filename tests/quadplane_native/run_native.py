#!/usr/bin/env python3
"""Owned ArduPlane QuadPlane SITL qualification with a separate FC monitor."""
import argparse
from contextlib import ExitStack
import fcntl
import importlib.util
import json
import math
from pathlib import Path
import platform
import re
import select
import shutil
import signal
import sys
import tempfile
import traceback

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'tests' / folder) for folder in ('plane_native', 'plane_faults', 'ardusub', '')]
from flight_fixture import (ATTESTATION_SHA, BINARY_SHA, HELM_SHA, HOME, INTERVALS,
                            MOOS_PIN, PIN, Flight, json_data, need, now, revision, sha, start)
from owned_processes import stop_process_tree
from run_native import dataflash, ports_clear
from pymavlink.dialects.v20 import ardupilotmega as mav
import fault_tap
from adverse import ADVERSE_CASES, audit_wire, run_adverse
from live_helm import LIVE_CASES, LIVE_PARAMETERS, audit_live, run_live, witness_auto_fence
from hover_helm import HOVER_CASES, HOVER_PARAMETERS, audit_case, run_hover
from lost_confirmation import CASES as CONFIRMATION_CASES, audit_loss, run_loss


def sub_probe():
    spec = importlib.util.spec_from_file_location('sub_native', ROOT / 'tests/ardusub/run_native.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.Probe, module.fields


Probe, fields = sub_probe()
DEFAULTS_SHA = '3b736735829637583fcac4349d1dabc925dddb29dacf3cd023ddbf30587c24e9'
QLOITER, AUTO, QRTL = 19, 10, 21
HELM_CASES = (*LIVE_CASES, *HOVER_CASES)
FAULT_CASES = (*ADVERSE_CASES, *CONFIRMATION_CASES)
# Older pymavlink dialects retain the reserved name for this wire value.
VTOL_FIXEDROTOR_TYPE = 22
REQUEST, RESULT = 'MAVMOOS_QUADPLANE_REQUEST', 'MAVMOOS_QUADPLANE_RESULT'
PROOF, LEASE = 'MAVMOOS_STORED_MISSION_PROOF', 'MAVMOOS_MISSION_EXECUTION_LEASE'
FIXTURE = {'SYSID_THISMAV': 1, 'SYSID_MYGCS': 245, 'SERIAL0_PROTOCOL': 2, 'SERIAL1_PROTOCOL': 2,
           'SERIAL2_PROTOCOL': 2, 'Q_ENABLE': 1, 'Q_MAV_TYPE': VTOL_FIXEDROTOR_TYPE,
           'FS_GCS_ENABL': 0, 'LOG_DISARMED': 1, 'SIM_WIND_SPD': 0, 'SIM_WIND_TURB': 0,
           'ARSPD_TYPE': 100, 'ARSPD_USE': 1, 'Q_RTL_ALT': 25,
           # Darwin clang speculates a guarded motor-mixer division at disarmed boot.
           'SIM_FLOAT_EXCEPT': 0}


def inputs(args):
    source = args.plane_build_root.resolve() / 'source'
    build, moos = args.build.resolve(), args.moos_root.resolve()
    paths = {
        'native': source / 'build/sitl/bin/arduplane',
        'defaults': source / 'Tools/autotest/default_params/quadplane.parm',
        'attestation': source.parent / 'build-attestation.json',
        'helm': moos / 'bin/pHelmIvP', 'db': moos / 'bin/MOOSDB',
        'app': build / 'bin/iMAVLink', 'probe': build / 'bin/ardusub_moos_probe',
        'telemetry': build / 'plugins/mavmoos_telemetry.so',
        'control': build / 'plugins/mavmoos_arduquadplane_control.so',
        'missions': build / 'plugins/mavmoos_missions.so',
        'cache': build / 'CMakeCache.txt', 'runner': Path(__file__).resolve(),
    }
    if args.case in (*FAULT_CASES, *HELM_CASES):
        paths.update({
            'tap': build / 'bin/libmavmoos_plane_fault_tap.dylib',
            'tap_helper': Path(fault_tap.__file__).resolve(),
        })
    if args.case in ADVERSE_CASES:
        paths['case_logic'] = Path(__file__).with_name('adverse.py')
    if args.case in CONFIRMATION_CASES:
        paths.update(case_logic=Path(__file__).with_name('lost_confirmation.py'),
                     wire_helpers=Path(__file__).with_name('adverse.py'),
                     monitor=ROOT / 'tests/plane_native/flight_fixture.py',
                     probe_source=ROOT / 'tests/ardusub/ArduSubMoosProbe.cpp',
                     tap_source=ROOT / 'tests/plane_faults/OwnedPlaneFaultTap.cpp')
    if args.case in HELM_CASES:
        need(args.behavior_dir is not None, 'live helm requires the existing altitude behavior directory')
        case_name = 'hover_helm' if args.case in HOVER_CASES else 'live_helm'
        paths.update({
            'case_logic': Path(__file__).with_name(case_name + '.py'),
            'behavior': Path(__file__).with_name(case_name + '.bhv'),
            'altitude_behavior': args.behavior_dir.resolve() / 'libBHV_ConstantAltitude.dylib',
            'monitor': ROOT / 'tests/plane_native/flight_fixture.py',
            'monitor_replay': ROOT / 'tests/plane_native/replay.py',
            'probe_source': ROOT / 'tests/ardusub/ArduSubMoosProbe.cpp',
        })
        if args.case in ('hover-survey', 'hover-mode-stress'):
            behavior_name = 'quadplane_stress.bhv' if args.case == 'hover-mode-stress' else 'quadplane_survey.bhv'
            paths.update(probe=build / 'bin/depth_autonomy_probe',
                         behavior=ROOT / 'missions/autonomy_sitl' / behavior_name,
                         evaluator=moos / 'bin/pMissionEval',
                         autonomy_logic=Path(__file__).with_name('hover_survey.py'),
                         grader=ROOT / 'tests/autonomy_missions/stock_grade.py')
        if args.case == 'hover-mode-stress':
            paths['stress_logic'] = Path(__file__).with_name('hover_mode_stress.py')
        if args.case in HOVER_CASES:
            paths.update({'adapter': Path(__file__).with_name('hover_adapter.lua'),
                          'shared_helm_fixture': Path(__file__).with_name('live_helm.py')})
    hashes = {'native': BINARY_SHA, 'defaults': DEFAULTS_SHA, 'attestation': ATTESTATION_SHA, 'helm': HELM_SHA}
    need(platform.system() == 'Darwin' and platform.machine() == 'arm64', 'pinned binary requires Darwin ARM64')
    need(revision(ROOT) == args.app_revision and revision(moos) == MOOS_PIN and
         revision(source, clean=False) == PIN, 'exact MAVMOOS, stock MOOS or ArduPlane revision differs')
    for key, path in paths.items():
        need(path.is_file() and not path.is_symlink(), f'missing regular input {key}')
        if key in hashes:
            need(sha(path) == hashes[key], f'{key} digest differs from the exact pin')
    for key, directory in (('CMAKE_HOME_DIRECTORY', ROOT), ('MOOS_IVP_ROOT', moos)):
        line = re.search(rf'^{key}:[^=]+=([^\n]+)$', paths['cache'].read_text(), re.M)
        need(line and Path(line[1]).resolve() == directory, f'build provenance differs for {key}')
    attestation = json.loads(paths['attestation'].read_text())
    need(attestation['status'] == 'OFFLINE_BUILD_PASS' and attestation['source_revision'] == PIN and
         attestation['artifact']['sha256'] == BINARY_SHA, 'native build attestation differs')
    return paths


def distance(a, b):
    return 111194.93 * math.hypot((a['lat'] - b['lat']) * 1e-7,
                                 (a['lon'] - b['lon']) * 1e-7 * math.cos(math.radians(a['lat'] * 1e-7)))


def speed(position):
    return math.hypot(position['vx'], position['vy']) / 100


class QuadFlight(Flight):
    def __init__(self, paths, out, stack, report):
        if report['case'] in HELM_CASES:
            from pymavlink.dialects.v20 import all as codec
        else:
            codec = mav
        super().__init__(paths, out, stack, codec, report)
        self.paths, self.stack = paths, stack
        self.probe = self.app = self.moosdb = self.helm = self.evaluator = None
        self.live = report['case'] in HELM_CASES
        self.fixture = FIXTURE | LIVE_PARAMETERS if self.live else FIXTURE
        self.hover = report['case'] in HOVER_CASES
        if self.hover:
            self.fixture |= HOVER_PARAMETERS
            script_dir = out / 'scripts'
            script_dir.mkdir(mode=0o700)
            script = script_dir / 'mavmoos-hover.lua'
            shutil.copyfile(paths['adapter'], script)
            need(sha(script) == sha(paths['adapter']), 'deployed hover adapter differs from source')
        self.app_pids = []
        self.tap_traces = {}
        self.command[self.command.index('--model') + 1] = 'quadplane'
        (out / 'fixture.parm').write_text(''.join(f'{key} {value}\n' for key, value in self.fixture.items()))
        for channel in range(3):
            intervals = INTERVALS
            if channel == 0 and report['case'] not in ('live', *HOVER_CASES):
                # ArduPilot slows telemetry on the active upload channel. Give
                # the ground-evidence gate the host's normal rates from boot.
                intervals = intervals.replace('33 100\n', '33 50\n').replace('245 500\n', '245 250\n')
            rc_interval = '65 200\n' if self.hover and channel == 1 else ''
            (out / f'message-intervals-chan{channel}.txt').write_text(intervals + '1 500\n' + rc_interval)
        report['native_command'] = [str(arg) for arg in self.command]

    def pump(self, duration=.04):
        need(self.simulator.poll() is None, f'owned QuadPlane exited {self.simulator.returncode}')
        need(self.app is None or self.app.poll() is None, 'owned QuadPlane app exited')
        need(self.helm is None or self.helm.poll() is None, 'owned stock helm exited')
        active = {c.endpoint: c for c in self.channels.values() if c.endpoint.fileno() != -1}
        if self.probe:
            active[self.probe.process.stdout] = None
        for ready in select.select(active, (), (), duration)[0]:
            if active[ready] is None:
                self.probe.read()
            else:
                active[ready].read(self.simulator, self.proofs)

    def heartbeat(self, mode=None, armed=None, after=0):
        row = self.latest('HEARTBEAT', after)
        return row if (row and row['type'] == VTOL_FIXEDROTOR_TYPE and
                       row['autopilot'] == mav.MAV_AUTOPILOT_ARDUPILOTMEGA and
                       (mode is None or row['custom_mode'] == mode) and
                       (armed is None or bool(row['base_mode'] & 128) == armed)) else None

    def qloiter(self):
        need(self.heartbeat(armed=False), 'fixture QLoiter selection requires disarmed aircraft')
        return self.operator_mode(QLOITER, False)[1]

    def operator_mode(self, mode, armed):
        modes = ((QLOITER, False), (5, True), (QRTL, True))
        if self.live:
            modes += ((15, True), (QLOITER, True), (12, True))
        need((mode, armed) in modes and self.heartbeat(armed=armed),
             'fixture operator mode requires its exact observed arming state')
        at = self.transmit(self.encoder.command_long_encode(1, 1, mav.MAV_CMD_DO_SET_MODE,
                                                            0, 1, mode, 0, 0, 0, 0, 0))
        hb = self.wait(10, lambda: self.heartbeat(mode, armed, at), f'independent FC mode {mode} missing')
        return at, hb

    def observation(self, *, after=0, mode=None, armed=None, phase=None, landed=None):
        hb = self.heartbeat(mode, armed, after)
        state = self.latest('EXTENDED_SYS_STATE', after)
        position = self.latest('GLOBAL_POSITION_INT', after, max_age=.5)
        if (hb and state and position and (phase is None or state['vtol_state'] == phase) and
                (landed is None or state['landed_state'] == landed)):
            return {'heartbeat': hb, 'state': state, 'position': position}

    def setup(self):
        def owned_ground():
            return (all(channel.peer for channel in self.channels.values()) and self.heartbeat(armed=False) and
                    self.latest('HOME_POSITION'))

        if self.live:
            self.wait(55, owned_ground, 'QuadPlane lacked disarmed HOME or exact owned routes')
            self.report['boot_disarmed'] = self.heartbeat(armed=False)
            # The live watchdog fixture can boot in disarmed RTL, which is
            # intentionally unarmable. Choose native QLoiter first.
            self.report['qloiter'] = self.qloiter()

        def ready():
            status, home = self.latest('SYS_STATUS'), self.latest('HOME_POSITION')
            return (owned_ground() and home and status and
                    (not self.live or self.heartbeat(QLOITER, False)) and
                    status['onboard_control_sensors_health'] & mav.MAV_SYS_STATUS_PREARM_CHECK)

        self.wait(65, ready, 'QuadPlane lacked prearm health, home or exact owned routes')
        self.report['prearm'] = self.latest('SYS_STATUS')
        values = {}
        checked = ('Q_ENABLE', 'Q_MAV_TYPE', 'Q_RTL_ALT', 'FS_GCS_ENABL', 'SIM_FLOAT_EXCEPT')
        if self.live:
            checked += ('SYSID_MYGCS', 'ARSPD_TYPE', 'ARSPD_USE')
            checked += tuple(key for key in LIVE_PARAMETERS if key not in checked)
        if self.hover:
            checked += tuple(key for key in HOVER_PARAMETERS if key not in checked)
        for name in checked:
            at = self.transmit(self.encoder.param_request_read_encode(1, 1, name.encode(), -1))
            def match():
                sample = self.operator.latest.get('PARAM_VALUE')
                return sample if sample and sample['mono'] >= at and sample['param_id'] == name else None
            row = self.wait(3, match, f'missing active {name} readback')
            need(abs(row['param_value'] - self.fixture[name]) < .001, f'unexpected active {name}')
            values[name] = row
        self.report['parameters'] = values
        if not self.live:
            self.report['qloiter'] = self.qloiter()
        home = self.latest('HOME_POSITION')
        need(home, 'fresh home disappeared before mission construction')
        self.report['home'] = home
        return home

    def start_app(self):
        self.bridge.endpoint.close()
        self.db.close()
        slot = self.out / 'recovery'
        slot.mkdir(mode=0o700)
        self.marker = slot / 'auto-may-continue'
        self.config = self.out / 'quadplane.moos'
        helm_timing = ('  helm_source = pHelmIvP\n  freshness_timeout = 1.5\n'
                       '  handoff_window = 1.0\n  field_spread = 0.35\n') if self.live else ''
        hover_setting = '  plugin.arduquadplane_control.hover_adapter = true\n' if self.hover else ''
        self.config.write_text(f"""ServerHost = localhost
ServerPort = {self.ports['db']}
Community = ardupilot_quadplane_sitl
MOOSTimeWarp = 1
LatOrigin = {HOME[0]}
LongOrigin = {HOME[1]}
ProcessConfig = iMAVLink
{{
  AppTick = 40
  CommsTick = 40
  bind_address = 127.0.0.1
  bind_port = {self.bridge.port}
  peer_address = 127.0.0.1
  peer_port = {self.bridge.peer[1]}
  target_system = 1
  target_component = 1
  expected_type = {VTOL_FIXEDROTOR_TYPE}
  expected_autopilot = {mav.MAV_AUTOPILOT_ARDUPILOTMEGA}
  simulation_only = true
  sitl_pid = {self.simulator.pid}
  shadow_mode = false
  helm_enabled = {'true' if self.live else 'false'}
  control_helm = {'true' if self.live else 'false'}
  control_native_actions = true
  control_mission_execution = true
  vehicle_profile = arduplane_quadplane
  plugins = telemetry,arduquadplane_control,missions
  plugin_directory = {self.paths['control'].parent}
  firmware_requests = false
  auto_recovery_directory = {slot}
{(helm_timing + hover_setting).rstrip()}
}}
""")
        if self.live:
            with self.config.open('a') as stream:
                stream.write(f"""
ProcessConfig = pHelmIvP
{{
  AppTick = 10
  CommsTick = 20
  behaviors = {self.paths['behavior']}
  domain = course:0:359:360
  domain = speed:0:35:36
  domain = altitude:0:130:131
  start_in_drive = true
  verbose = quiet
  ivp_behavior_dir = {self.paths['altitude_behavior'].parent}
}}
""")
        if 'evaluator' in self.paths:
            from autonomy_missions.stock_grade import EVALUATOR
            with self.config.open('a') as stream:
                stream.write(EVALUATOR)
        self.moosdb = start([self.paths['db'], self.config, 'MOOSDB'], self.out, 'moosdb.log')
        self.boot_app('original')
        if self.live:
            env = fault_tap.clean_environment()
            env.pop('IVP_BEHAVIOR_DIRS', None)
            self.helm = start([self.paths['helm'], self.config, 'pHelmIvP'], self.out, 'helm.log', env)
            self.report['helm_pid'] = self.helm.pid
        if 'evaluator' in self.paths:
            self.evaluator = start([self.paths['evaluator'], self.config, 'pMissionEval'],
                                   self.out, 'evaluator.log')
        self.probe = Probe(self.paths, self.ports['db'], self.out, self.stack, ('quadplane',))
        self.wait(9, lambda: self.probe.runtime(control_profile='arduplane_quadplane', fc_mode=str(QLOITER),
                                                heartbeat_valid='1', armed='0', permission='false'),
                  'QuadPlane owned app did not report fresh disarmed QLoiter')

    def boot_app(self, instance):
        need(self.app is None and instance in ('original', 'restarted') and instance not in self.tap_traces,
             'app process already owned or reused')
        env = fault_tap.clean_environment()
        if 'tap' in self.paths:
            trace, control = self.out / f'{instance}.plf', self.out / f'{instance}.tap-mode'
            fault_tap.set_mode(control, 'pass')
            env.update(fault_tap.environment(self.paths['tap'], self.bridge.port, self.bridge.peer[1], trace, control))
            self.tap_traces[instance] = trace.name
        name = 'imavlink.log' if instance == 'original' else 'imavlink-restarted.log'
        self.app = start([self.paths['app'], self.config, 'iMAVLink'], self.out, name, env)
        self.app_pids.append(self.app.pid)
        self.report['app_pids'] = self.app_pids
        self.report['tap_traces'] = self.tap_traces

    def kill_app(self):
        process = self.app
        need(process and process.poll() is None, 'restart requires the original app process alive')
        process.kill()
        need(process.wait(timeout=4) == -signal.SIGKILL and not stop_process_tree(process),
             'original app failed to stop cleanly for isolated restart')
        self.app = None
        return self.event('original_app_exited', pid=process.pid, status=process.returncode)

    def request(self, id_, action, status, extra='', timeout=10):
        at = self.probe.post(REQUEST, f'schema=1,id={id_},action={action}{extra}')
        row = self.wait(timeout, lambda: self.probe.match(RESULT, at, id=id_, status=status),
                        f'{id_} never reached {status}')
        return at, row

    def close(self):
        remaining = []
        for process in (self.probe.process if self.probe else None, self.evaluator, self.helm, self.app, self.moosdb, self.simulator):
            remaining.extend(stop_process_tree(process))
        self.report['owned_processes_remaining'] = remaining


def plan(home, adverse=False):
    x, y, z = home['latitude'], home['longitude'], home['altitude'] / 1000
    return [(0, 16, 0, 1, 1, 0, 0, 0, 0, x, y, z),
            (1, 84, 3, 0, 1, 0, 0, 0, 0, x, y, 30),
            (2, 16, 3, 0, 1, 0, 0, 0, 0, x + (44000 if adverse else 34000), y, 35),
            (3, 85, 3, 0, 1, 0, 0, 0, 0, x, y, 0)]


def flight_case(flight, home, case):
    report = flight.report
    flight.start_app()
    report['no_proof_rejected'] = flight.request('no-proof', 'run', 'REJECTED')[1]
    data = '|'.join(':'.join(map(str, row)) for row in plan(home, case in FAULT_CASES))
    at = flight.probe.post('MAVMOOS_MISSION_REQUEST', f'id=quad-plan,action=upload,timeout=20,items={data}')
    report['upload'] = flight.wait(22, lambda: flight.probe.match('MAVMOOS_MISSION_RESULT', at, id='quad-plan',
                                                                  status='COMPLETED', readback_confirmed='true'),
                                   'typed QuadPlane upload/readback failed')
    row = flight.wait(4, lambda: flight.probe.match(PROOF, at, valid='true', verification_id='quad-plan', count='4'),
                      'valid host-owned QuadPlane proof unavailable')
    proof = fields(row['value'])
    report['proof'] = row
    at, report['arming'] = flight.request('arm', 'arm', 'COMPLETED')
    report['armed'] = flight.wait(4, lambda: flight.heartbeat(QLOITER, True, at), 'independent QLoiter arm absent')
    suffix = f",verification_id=quad-plan,revision={proof['revision']},content_id={proof['content_id']}"
    report['stale_proof_rejected'] = flight.request('wrong-proof', 'run', 'REJECTED', suffix + '-wrong')[1]
    at, report['start'] = flight.request('flight', 'run', 'CONFIRMED', suffix, 14)
    need(flight.marker.is_file(), 'native AUTO started without a durable lease')
    report['early_release_rejected'] = flight.request('early-release', 'release', 'REJECTED', ',target=flight')[1]

    def capture(after, phase, height, velocity, mode=AUTO):
        row = flight.observation(after=after, mode=mode, armed=True, phase=phase, landed=mav.MAV_LANDED_STATE_IN_AIR)
        return row if row and row['position']['relative_alt'] >= height * 1000 and speed(row['position']) >= velocity else None

    report['hover'] = flight.wait(30, lambda: capture(at, mav.MAV_VTOL_STATE_MC, 8, 0),
                                  'independent VTOL hover climb never reached 8 m')
    hover_at = report['hover']['position']['mono']
    report['forward'] = flight.wait(50, lambda: capture(hover_at, mav.MAV_VTOL_STATE_FW, 12, 10),
                                     'independent armed forward flight not observed')
    forward_at = report['forward']['position']['mono']
    if case == 'live-auto':
        witness_auto_fence(flight, forward_at)
    origin = {'lat': home['latitude'], 'lon': home['longitude']}
    if case in CONFIRMATION_CASES:
        run_loss(flight)
        return
    if case in ADVERSE_CASES:
        run_adverse(flight, case, origin, forward_at, suffix)
        return

    if case == 'abort':
        at, report['abort'] = flight.request('stop', 'abort', 'CONFIRMED', ',target=flight')
        report['native_abort'] = flight.wait(8, lambda: flight.heartbeat(QRTL, after=at), 'independent QRTL absent')
    else:
        def descent():
            row = capture(forward_at, mav.MAV_VTOL_STATE_MC, 3, 0)
            cursor = flight.latest('MISSION_CURRENT', forward_at)
            return row if row and cursor and cursor['seq'] == 3 else None

        report['return_to_hover'] = flight.wait(55, descent, 'independent hover on landing approach absent')
        need(any(distance(origin, position) >= 200 for position in flight.monitor.messages
                 if position['mavpackettype'] == 'GLOBAL_POSITION_INT' and position['mono'] >= forward_at),
             'forward mission did not physically transit 200 m')

    def ground():
        row = flight.observation(after=forward_at, armed=False, phase=mav.MAV_VTOL_STATE_MC,
                                 landed=mav.MAV_LANDED_STATE_ON_GROUND)
        return row if row and abs(row['position']['relative_alt']) <= 2500 and speed(row['position']) <= 1 and \
            distance(origin, row['position']) <= 40 else None

    report['touchdown'] = flight.wait(125 if case == 'abort' else 70, ground,
                                     'disarmed physical touchdown within 40 m of home absent')
    if case == 'abort':
        report['post_abort_qloiter'] = flight.qloiter()
    else:
        report['completion'] = flight.wait(12, lambda: flight.probe.match(RESULT, at, id='flight', status='COMPLETED'),
                                           'native mission did not finish the observed ground handback')
    need(flight.marker.is_file(), 'durable marker cleared before explicit release')
    if case == 'abort':
        # The original abort admission occupies its bounded slot for ten seconds.
        flight.wait(12, lambda: now() >= report['abort']['mono'] + 10, 'abort slot did not expire')
    _, report['release'] = flight.request('release', 'release', 'COMPLETED', ',target=flight', 12)
    need(not flight.marker.exists() and flight.probe.match(LEASE, held='false'),
         'explicit observed handback did not clear the lease')


def main(args):
    out = args.output.resolve()
    out.mkdir(mode=0o700, parents=True, exist_ok=False)
    report = {'result': 'FAIL', 'case': args.case, 'app_revision': args.app_revision,
              'firmware_revision': PIN, 'moos_revision': MOOS_PIN, 'hardware': False}
    paths = {}
    try:
        paths = inputs(args)
        report['inputs'] = {key: {'path': str(path), 'sha256': sha(path)} for key, path in paths.items()}
        with ExitStack() as stack:
            lock = stack.enter_context((Path(tempfile.gettempdir()) / 'mavmoos-native-sitl.lock').open('a+'))
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            flight = QuadFlight(paths, out, stack, report)
            try:
                flight.launch()
                home = flight.setup()
                if args.case in (*FAULT_CASES, *HELM_CASES):
                    report['ports'].update({f'{name}_peer': channel.peer[1] for name, channel in flight.channels.items()})
                if args.case == 'live':
                    run_live(flight, home)
                elif args.case in HOVER_CASES:
                    run_hover(flight)
                else:
                    flight_case(flight, home, args.case)
            finally:
                flight.close()
        report['dataflash'] = dataflash(out)
        if args.case in ADVERSE_CASES:
            report['wire_audit'] = audit_wire(out, report)
        if args.case in CONFIRMATION_CASES:
            report['wire_audit'] = audit_loss(out, report)
        if args.case in LIVE_CASES:
            report['wire_audit'] = audit_live(out, report)
        if args.case in HOVER_CASES:
            report['wire_audit'] = audit_case(out, report)
        report['result'] = 'PASS'
    except BaseException as error:
        report['error'] = str(error) or type(error).__name__
        (out / 'failure.log').write_text(traceback.format_exc())
    try:
        report['ports_clear'] = ports_clear(report.get('ports', {}))
        report['cleanup_proven'] = bool(report['ports_clear']) and all(report['ports_clear'].values()) and \
            report.get('owned_processes_remaining') == []
        report['input_drift'] = [key for key, path in paths.items() if sha(path) != report['inputs'][key]['sha256']]
        need(report['cleanup_proven'] and not report['input_drift'], 'scoped cleanup or input immutability failed')
    except BaseException as error:
        report['result'] = 'FAIL'
        report['postflight_error'] = str(error)
    report['files'] = {str(path.relative_to(out)): sha(path) for path in sorted(out.rglob('*')) if path.is_file()}
    (out / 'result.json').write_text(json.dumps(json_data(report), indent=2, allow_nan=False) + '\n')
    print(report['result'], out / 'result.json', report.get('error', ''))
    return 0 if report['result'] == 'PASS' else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('plane-build-root', 'moos-root', 'build', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--app-revision', required=True)
    parser.add_argument('--behavior-dir', type=Path)
    parser.add_argument('--case', choices=('normal', 'abort', *FAULT_CASES, *HELM_CASES), required=True)
    raise SystemExit(main(parser.parse_args()))
