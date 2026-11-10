#!/usr/bin/env python3
"""Exercise the MAVMOOS Ackermann provider with stock helm and an independent GCS."""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
from types import SimpleNamespace

from run_ackermann import EXTRA_PINS, HOLD, MISSION, OFFBOARD, PX4_REVISION, parse_ground_truth, ports_free, waypoint

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'px4_offboard'))
from run_px4_offboard import STABLE_PIN, Workflow, fields, free_ports, main, need, sha, udp_sockets

PROFILE = 'px4_rover_ackermann'
MODEL = 'sihsim_rover_ackermann'
FIXTURE_PARAMS = {'COM_OF_LOSS_T': .7, 'COM_OBL_RC_ACT': 5, 'MAV_FWDEXTSP': 1}
STOCK_FILES = tuple(f'moos-ivp/bin/{name}' for name in ('pHelmIvP', 'MOOSDB', 'pAntler', 'pLogger', 'nsplug'))
ARTIFACT_FILES = (
    'bin/iMAVLink', 'bin/px4_offboard_moos_probe', 'plugins/mavmoos_telemetry.so',
    'plugins/mavmoos_px4_rover_control.so', 'plugins/mavmoos_missions.so',
)
BUILD_TARGETS = ('iMAVLink', 'px4_offboard_moos_probe', 'mavmoos_telemetry',
                 'mavmoos_px4_rover_control', 'mavmoos_missions')


class AckermannWorkflow(Workflow):
    model = MODEL
    profile = PROFILE
    mav_type = 10
    probe_args = ('--rover',)
    fixture_params = FIXTURE_PARAMS
    scope = 'owned PX4 Ackermann SITL native mission and stock helm with verified stopped handback'

    def __init__(self, args, out, report):
        super().__init__(args, out, report)
        self.deadline = time.monotonic() + 215
        self.recovery = out / 'recovery'
        self.bridge = None
        self.autonomy = args.case == 'autonomy'
        self.stress = args.case == 'stress'
        report.update(profile=PROFILE, model=MODEL, case=args.case, build_source_revision=args.build_source_revision)

    def protect(self):
        for path in (self.px4_root, self.moos, self.build, Path('/source')):
            need(os.statvfs(path).f_flag & os.ST_RDONLY, f'protected mount must be read-only: {path}')
        stable = self.args.stable_pin
        pins = {self.px4_root / name: digest for name, digest in stable['files'].items()}
        pins.update({Path('/work') / name: stable['retained_work_files'][name] for name in STOCK_FILES})
        build = json.loads(self.args.build_manifest.read_text())
        artifacts = build['artifacts']
        need(build['source_revision'] == self.args.build_source_revision and set(artifacts) == set(ARTIFACT_FILES)
             and all(re.fullmatch(r'[a-f0-9]{64}', digest) for digest in artifacts.values()),
             'sealed build identity or artifacts differ')
        pins.update({self.build / name: digest for name, digest in artifacts.items()})
        sources = (Path(__file__), self.args.build_manifest, HERE / 'run_ackermann.py',
                   HERE / 'meta_ackermann.moos', HERE / 'meta_ackermann.bhv',
                   HERE.parent / 'px4_offboard/run_px4_offboard.py',
                   HERE.parent / 'px4_offboard/Px4MoosProbe.cpp')
        if self.autonomy:
            sources += (HERE / 'meta_ackermann_autonomy.bhv',)
        self.protected = {str(path): sha(path) for path in (*pins, *sources)}
        for path, digest in pins.items():
            need(self.protected[str(path)] == digest, f'protected input differs: {path}')

    def generate(self, origin):
        self.bearing = math.radians(origin['heading_deg'])
        self.route_origin = origin
        self.recovery.mkdir(mode=0o700)
        settings = dict(MOOS_PORT=self.db, DB_PATH=self.bindir / 'MOOSDB', APP_PATH=self.app,
                        HELM_PATH=self.helm, LOGGER_PATH=self.bindir / 'pLogger', SITL_PID=self.px4.pid,
                        PLUGIN_DIRECTORY=self.build / 'plugins', RECOVERY_DIRECTORY=self.recovery,
                        LAT_ORIGIN=origin['latitude_e7'] / 1e7, LONG_ORIGIN=origin['longitude_e7'] / 1e7)
        n, e = math.cos(self.bearing), math.sin(self.bearing)
        # Use the actual initial vehicle heading, with a 30 degree steering leg.
        bearing = self.bearing + math.radians(30)
        self.goal_straight = (47 * e, 47 * n)
        self.goal_turn = (31 * e + 21 * math.sin(bearing), 31 * n + 21 * math.cos(bearing))
        route = {'STRAIGHT_POINT': f'{self.goal_straight[0]:.3f},{self.goal_straight[1]:.3f}',
                 'TURN_POINT': f'{self.goal_turn[0]:.3f},{self.goal_turn[1]:.3f}'}
        if self.autonomy:
            # Native mission hands back at the 28 m waypoint. Preserve its
            # forward heading for the next 9 m before the helm selects a bend.
            route.update(CORNER=f'{37 * e:.3f},{37 * n:.3f}',
                         BEND=f'{37 * e + 9 * math.sin(bearing):.3f},{37 * n + 9 * math.cos(bearing):.3f}',
                         EXIT=f'{46 * e + 9 * math.sin(bearing):.3f},{46 * n + 9 * math.cos(bearing):.3f}',
                         OVERRUN=f'{54 * e + 9 * math.sin(bearing):.3f},{54 * n + 9 * math.cos(bearing):.3f}')
        for suffix, variables in (('moos', settings), ('bhv', route)):
            source = HERE / ('meta_ackermann_autonomy.bhv' if self.autonomy and suffix == 'bhv' else 'meta_ackermann.' + suffix)
            target = self.out / ('targ_px4.' + suffix)
            subprocess.run([str(self.nsplug), str(source), str(target), '--strict', '--force', '-x',
                            *(f'{key}={value}' for key, value in variables.items())], cwd=self.out,
                           env=self.env, capture_output=True, text=True, timeout=5, check=True)
            need(target.is_file() and '$(' not in target.read_text(), 'unresolved stock mission substitution')
        self.report['stock_route'] = route
        mission = self.out / 'targ_px4.moos'
        if self.args.case in ('restart', 'stress'):
            # Restart only the directly supervised bridge. The stock MOOS
            # community, helm, probe and PX4 stay live throughout the case.
            line = f'  Run = {self.app} @ NewConsole = false ~ iMAVLink\n'
            text = mission.read_text()
            need(text.count(line) == 1, 'generated restart mission app launch is ambiguous')
            mission.write_text(text.replace(line, ''))
        return mission

    def heartbeat(self, mode=None, armed=None, after=0):
        row = self.latest('wire', 'HEARTBEAT', after, age=2)
        return row if row and row['mav_type'] == self.mav_type and row['autopilot'] == 12 and (
            mode is None or row['mode'] == mode) and (armed is None or row['armed'] is armed) else None

    def rover(self, after=0, **wanted):
        row = self.latest('mail', 'MAVMOOS_PX4_ROVER_STATUS', after, age=.7)
        state = fields(row)
        return row if row and row['source'] == 'iMAVLink' and all(
            state.get(key) == str(value) for key, value in wanted.items()) else None

    def event_result(self, key, operation, status, after):
        for row in reversed(self.rows):
            if row['mono'] < after:
                break
            if row.get('key') == key and row.get('source') == 'iMAVLink':
                found = fields(row)
                if found.get('id') == operation and found.get('status') == status:
                    return row
        return None

    def mission_result(self, operation, status, after):
        return self.event_result('MAVMOOS_MISSION_EXECUTION_RESULT', operation, status, after)

    def boot(self):
        need(all(free_ports(self.db).values()) and all(ports_free().values()), 'private fixture ports are in use')
        self.start_probe()
        self.px4 = self.launch([self.native, '-d', self.px4_root / 'etc', '-w', self.out, '-s',
                                self.px4_root / 'etc/init.d-posix/rcS'], 'startup')
        self.wait(20, lambda: self.heartbeat(armed=False), 'owned disarmed PX4 rover heartbeat absent')
        version = self.cli('ver', 'all')
        need(re.search(r'PX4 git-hash:\s*' + PX4_REVISION + r'\b', version) and
             re.search(r'PX4 version:\s*Release 1\.17\.0\b', version),
             'owned PX4 revision or release differs')
        model = self.cli('simulator_sih', 'status')
        need('ackermann' in model.lower() and 'running' in self.cli('rover_ackermann', 'status').lower(),
             'owned simulator/controller identity differs')
        sockets = udp_sockets(self.px4.pid)
        need({14580, 18570}.issubset(row['local_port'] for row in sockets), 'owned PX4 UDP route differs')
        self.report.update(px4_pid=self.px4.pid, px4_udp_sockets=sockets, px4_cli_version=version,
                           px4_sih_status=model)
        for name, expected in self.fixture_params.items():
            self.cli('param', 'set', name, str(expected))
            value = re.search(r'\b' + name + r'\b[^\r\n]*?:\s*([-+0-9.]+)', self.cli('param', 'show', name))
            need(value and math.isclose(float(value[1]), expected, abs_tol=.001), f'fixture parameter differs: {name}')
        self.report['verified_fixture_parameters'] = self.fixture_params
        for message, rate in (('GPS_RAW_INT', 10), ('CURRENT_MODE', 5), ('HOME_POSITION', 2)):
            self.cli('mavlink', 'stream', '-u', '18570', '-s', message, '-r', str(rate))
        self.wait(20, lambda: (self.latest('wire', 'SYS_STATUS') or {}).get('prearm_healthy'),
                  'owned independent PX4 prearm checks not healthy')
        origin = self.wait(8, lambda: self.global_position(0), 'independent origin and heading absent')
        self.home = self.wait(8, lambda: self.latest('wire', 'HOME_POSITION'), 'independent HOME_POSITION absent')
        mission = self.generate(origin)
        self.antler = self.launch([self.bindir / 'pAntler', mission], 'pantler')
        if self.args.case in ('restart', 'stress'):
            self.wait(8, lambda: self.latest('connected'), 'restart fixture private MOOSDB did not start')
            self.bridge = self.launch([self.app, mission, 'iMAVLink'], 'bridge-one')
        self.helm_record = self.wait(10, lambda: self.stock_helm_process(mission), 'exact stock helm did not launch')
        self.report['stock_helm_process'] = self.helm_record
        self.wait(15, lambda: self.runtime(configuration_valid='true', heartbeat_valid=1,
                                          control_profile=PROFILE, nav_tx_accepted=0, armed=0) and
                  self.rover(config_verified='true'), 'owned Rover app startup or verified configuration missing')

    def global_position(self, after):
        row = self.latest('wire', 'GLOBAL_POSITION_INT', after, age=.65)
        return row if row and row['latitude_e7'] and row['longitude_e7'] and 0 <= row['heading_deg'] < 360 else None

    def local_position(self, after=0):
        return self.latest('wire', 'LOCAL_POSITION_NED', after, age=.65)

    def prepare_mission(self):
        self.wait(15, lambda: self.heartbeat(HOLD, False) and self.current_mode(HOLD, HOLD) and
                  self.rover(config_verified='true', hold_ready='true'),
                  'verified stopped disarmed Hold required before the mission upload')
        home = SimpleNamespace(lat=self.home['latitude_e7'], lon=self.home['longitude_e7'])
        points = [waypoint(home, self.bearing, distance) for distance in (13, 28)]
        items = '|'.join(f'{i}:16:6:{int(i == 0)}:1:0:0:0:0:{lat}:{lon}:0'
                         for i, (lat, lon) in enumerate(points))
        after = self.poke('MAVMOOS_MISSION_REQUEST', 'id=rover-upload,action=upload,timeout=30,items=' + items)

        def uploaded():
            row = self.event_result('MAVMOOS_MISSION_RESULT', 'rover-upload', 'COMPLETED', after)
            proof_row = self.latest('mail', 'MAVMOOS_STORED_MISSION_PROOF', after, age=5)
            snapshot = self.latest('mail', 'MAVMOOS_MISSION_SNAPSHOT', after, age=5)
            if not all((row, proof_row, snapshot, self.heartbeat(armed=False))):
                return None
            proof, stored = fields(proof_row), json.loads(snapshot['value'])
            need(fields(row).get('remote_accepted') == 'true' and fields(row).get('readback_confirmed') == 'true'
                 and proof.get('valid') == 'true' and proof.get('verification_id') == 'rover-upload'
                 and proof.get('count') == '2' and int(proof.get('remote_id', '0')) != 0
                 and stored['id'] == 'rover-upload' and stored['count'] == 2,
                 'verified disarmed mission proof differs')
            for i, (item, coordinates) in enumerate(zip(stored['items'], points)):
                need(item['seq'] == i and item['command'] == 16 and item['frame'] in (3, 6)
                     and item['autocontinue'] == 1 and item['params'] == [0] * 4
                     and (item['x'], item['y']) == coordinates and item['z'] == 0, 'returned mission item differs')
            need(stored['opaque_id'] == int(proof['remote_id']), 'mission readback plan ID differs')
            return {'proof': proof, 'storage': stored, 'result': row, 'disarmed': self.heartbeat(armed=False)}

        result = self.wait(33, uploaded, 'MAVMOOS disarmed mission upload and readback proof absent')
        self.report['verified_upload'] = result
        return result['proof']

    def stopped(self, after, intended=HOLD, timeout=12, *, confirmed=True):
        samples = []
        next_query = 0.

        def dwell():
            nonlocal next_query
            now = time.monotonic()
            if now < next_query:
                return None
            next_query = now + .2
            status = self.rover(after, stop_confirmed=str(confirmed).lower())
            mode = self.current_mode(HOLD, intended, after)
            gps = self.latest('wire', 'GPS_RAW_INT', after, age=.5)
            if not all((status, mode, gps, self.heartbeat(HOLD, True, after))):
                samples.clear()
                return None
            truth = parse_ground_truth(self.cli('listener', 'vehicle_local_position_groundtruth', '-n', '1'))
            speed = math.hypot(truth['vx'], truth['vy'])
            if speed > .05 or (samples and truth['timestamp'] <= samples[-1]['truth']['timestamp']):
                samples.clear()
                return None
            samples.append({'truth': truth, 'truth_speed_m_s': speed, 'independent_gps': gps, 'status': status})
            if len(samples) >= 5 and truth['timestamp'] - samples[0]['truth']['timestamp'] >= 1_500_000:
                return {'mode': mode, 'samples': samples,
                        'firmware_dwell_s': (truth['timestamp'] - samples[0]['truth']['timestamp']) / 1e6,
                        'max_truth_speed_m_s': max(row['truth_speed_m_s'] for row in samples)}
            return None

        return self.wait(timeout, dwell, f'no advancing independent SIH dwell with stop_confirmed={confirmed}')

    def arm(self):
        select = self.event('operator_select_hold')
        self.cli('commander', 'mode', 'auto:loiter')
        self.wait(6, lambda: self.heartbeat(HOLD, False, select), 'disarmed native Hold not independently observed')
        arm = self.event('operator_arm')
        self.cli('commander', 'arm')
        self.wait(10, lambda: self.heartbeat(HOLD, True, arm) and
                  self.rover(arm, config_verified='true', hold_ready='true'),
                  'external arm and positive Hold readiness missing')
        self.report['armed_hold'] = self.stopped(arm)

    def command(self, operation, action, proof=None):
        text = f'schema=1,id={operation},action={action}'
        if proof is None:
            text += ',target=rover-run,timeout=15'
        else:
            text += (f",verification_id={proof['verification_id']},content_id={proof['content_id']},"
                     f"proof_revision={proof['revision']},timeout=120")
        return self.poke('MAVMOOS_MISSION_CONTROL', text)

    @staticmethod
    def moved(start, finish):
        return math.hypot(finish['north'] - start['north'], finish['east'] - start['east'])

    def start_native_mission(self, proof):
        start = self.wait(3, lambda: self.local_position(), 'initial local position absent')

        def ready():
            status = self.latest('mail', 'MAVMOOS_MISSION_EXECUTION_STATUS', age=.55)
            cursor = self.latest('wire', 'MISSION_CURRENT')
            return status and status['source'] == 'iMAVLink' and fields(status).get('start_ready') == 'true' and (
                cursor and cursor['mission_id'] == int(proof['remote_id']) and cursor['total'] == int(proof['count']))

        self.wait(8, ready, 'start readiness and independent verified PX4 mission cursor absent')
        after = self.command('rover-run', 'start', proof)
        active = self.wait(13, lambda: self.mission_result('rover-run', 'ACTIVE', after) and
                           self.heartbeat(MISSION, True, after) and self.current_mode(MISSION, MISSION, after),
                           'native Mission not independently activated by MAVMOOS')
        first = self.wait(16, lambda: next((row for row in [self.local_position(after)] if row and
                                          self.moved(start, row) >= 3), None),
                          'native Mission did not move before pause')
        return start, active, first

    def run_native_mission(self, proof):
        start, active, first = self.start_native_mission(proof)
        paused = self.command('rover-pause', 'pause')
        pause_result = self.wait(16, lambda: self.mission_result('rover-pause', 'COMPLETED', paused),
                                 'MAVMOOS Mission pause failed')
        hold = self.stopped(paused)
        nav_before = fields(self.runtime()).get('nav_tx_accepted')
        self.poke('DEPLOY', 'true')
        denied_at = self.poke('MAVMOOS_NAV_CONTROL', 'acquire')
        denied = self.wait(4, lambda: self.helm_state(denied_at, reason='acquisition_rejected') and
                           self.runtime(denied_at, permission='false', mission_execution_held='true',
                                        nav_tx_accepted=nav_before), 'paused Mission did not fence helm acquisition')
        self.poke('DEPLOY', 'false')
        resume = self.command('rover-resume', 'resume')
        resumed = self.wait(16, lambda: self.mission_result('rover-resume', 'COMPLETED', resume) and
                            self.heartbeat(MISSION, True, resume), 'native Mission resume not independently observed')

        def completed():
            terminal = self.mission_result('rover-run', 'COMPLETED', resume)
            reached = next((row for row in reversed(self.rows) if row['mono'] > resume and
                            row.get('message') == 'MISSION_ITEM_REACHED' and row.get('sequence') == 1), None)
            position = self.local_position(resume)
            if terminal and reached and position and self.moved(first, position) >= 8:
                return {'result': terminal, 'reached': reached, 'position': position}
            return None

        done = self.wait(38, completed, 'final waypoint, motion or Mission completion absent')
        release = self.command('rover-release', 'release')
        released = self.wait(16, lambda: self.mission_result('rover-release', 'COMPLETED', release) and
                             self.runtime(release, mission_execution_held='false', permission='false'),
                             'explicit Mission release did not retire authority')
        self.report['mission'] = {'active': active, 'motion_before_pause_m': self.moved(start, first),
                                  'pause_result': pause_result, 'hold_dwell': hold, 'paused_helm_denial': denied,
                                  'resumed': resumed, 'completion': done, 'motion_after_resume_m':
                                  self.moved(first, done['position']), 'released': released}

    def run_native_abort(self, proof):
        start, active, first = self.start_native_mission(proof)
        at = self.command('rover-abort', 'abort')
        result = self.wait(16, lambda: self.mission_result('rover-abort', 'COMPLETED', at),
                           'stop-only native Mission abort did not finish')
        cancelled = self.wait(3, lambda: self.mission_result('rover-run', 'CANCELLED', at),
                              'aborted Mission did not report cancellation')
        retired = self.wait(3, lambda: self.runtime(at, mission_execution_held='false', permission='false',
                                                    nav_tx_accepted=0, px4_hold_tx_accepted=0),
                              'native Mission abort did not retire execution authority')
        hold = self.stopped(at)
        need(not (self.recovery / 'auto-may-continue').exists(), 'native Mission abort retained uncertainty')
        resume = self.command('rover-aborted-resume', 'resume')
        denied = self.wait(5, lambda: self.mission_result('rover-aborted-resume', 'REJECTED', resume),
                           'resume of aborted native Mission was not denied')
        need(fields(denied).get('reason') == 'target_not_retained', 'aborted execution remained selectable')
        self.wait(3, lambda: self.runtime(resume, mission_execution_held='false', permission='false',
                                         nav_tx_accepted=0, px4_hold_tx_accepted=0),
                  'aborted execution regained authority')
        self.report['abort'] = {'active': active, 'motion_before_abort_m': self.moved(start, first),
                                'result': result, 'mission_cancelled': cancelled, 'lease_retired': retired, 'stopped_hold': hold,
                                'resume_denied': denied}

    def helm_state(self, after=0, **wanted):
        row = self.latest('mail', 'MAVMOOS_HELM_STATUS', after, age=.55)
        state = fields(row)
        return row if row and row['source'] == 'iMAVLink' and all(
            state.get(key) == str(value) for key, value in wanted.items()) else None

    def stock_inputs(self, after=0):
        heading = self.latest('mail', 'DESIRED_HEADING', after, age=.55)
        speed = self.latest('mail', 'DESIRED_SPEED', after, age=.55)
        drive = self.latest('mail', 'IVPHELM_STATE', after, age=.55)
        return {'heading': heading, 'speed': speed, 'drive': drive} if all(
            row and row['source'] == 'pHelmIvP' for row in (heading, speed, drive)) and (
            isinstance(heading['value'], (int, float)) and isinstance(speed['value'], (int, float))
            and .9 <= speed['value'] <= 1.1 and drive['value'] == 'DRIVE') else None

    def enter_offboard(self):
        status = fields(self.runtime())
        start = self.wait(3, lambda: self.local_position(), 'initial helm local position absent')
        nav, hold = int(status['nav_tx_accepted']), int(status['px4_hold_tx_accepted'])
        after = self.poke('MAVMOOS_NAV_CONTROL', 'acquire')
        independent = self.wait(9, lambda: self.heartbeat(OFFBOARD, True, after),
                                'independent PX4 Offboard after helm acquire absent')
        acquired = self.wait(7, lambda: self.runtime(after, permission='true', px4_offboard_confirmed=1) and
                             int(fields(self.runtime())['nav_tx_accepted']) > nav and
                             self.stock_inputs(independent['mono']),
                             'no fresh stock helm motion after independent Offboard')
        state = fields(self.runtime())
        need(int(state['px4_hold_tx_accepted']) - hold >= 8, 'Offboard prestream proof missing')
        return {'start': start, 'offboard': independent, 'stock_inputs': acquired,
                'prestream_tx': int(state['px4_hold_tx_accepted']) - hold, 'acquired_at': after}

    def handoff(self):
        before = fields(self.runtime())
        pause = self.poke('DEPLOY', 'false')

        def held():
            stop = self.latest('mail', 'IVPHELM_ALLSTOP', pause, age=.55)
            row = self.runtime(pause, permission='false')
            if stop and stop['source'] == 'pHelmIvP' and stop['value'] in ('NothingToDo', 'NoGoalBehavior') and row and (
                    int(fields(row)['px4_hold_tx_accepted']) > int(before['px4_hold_tx_accepted'])):
                return {'stock_allstop': stop, 'runtime': row}
            return None

        evidence = self.wait(.85, held, 'stock all-stop did not produce bounded Ackermann zero-speed handoff')
        resume = self.poke('DEPLOY', 'true')
        evidence['resumed'] = self.wait(3, lambda: self.runtime(resume, permission='true',
                                                               px4_offboard_confirmed=1) and
                                       int(fields(self.runtime())['nav_tx_accepted']) > int(before['nav_tx_accepted']) and
                                       self.stock_inputs(resume), 'fresh stock helm did not resume within handoff')
        need(not any(row.get('kind') == 'request' and row.get('key') == 'MAVMOOS_NAV_CONTROL'
                     and pause <= row['mono'] <= evidence['resumed']['heading']['mono'] for row in self.rows),
             'normal handoff required a separate acquire')
        return evidence

    def withdraw_by_silence(self):
        need(self.stock_helm_process(self.out / 'targ_px4.moos') == self.helm_record,
             'helm silence would affect an unexpected process')
        at = self.event('stock_helm_suspended')
        os.kill(self.helm_record['pid'], signal.SIGSTOP)
        self.paused = True
        retired = self.wait(4, lambda: self.runtime(at, permission='false'), 'helm silence did not retire the grant')
        independent = self.wait(6, lambda: self.heartbeat(HOLD, True, at) and
                                self.current_mode(HOLD, OFFBOARD, at),
                                'PX4 fallback Hold with retained Offboard intent absent')
        stopped = self.stopped(at, intended=OFFBOARD)
        resumed = self.event('stock_helm_resumed')
        self.resume_helm()
        counts = fields(retired)
        fresh = self.wait(4, lambda: self.stock_inputs(resumed) and self.runtime(
            resumed, permission='false', nav_tx_accepted=counts['nav_tx_accepted'],
            px4_hold_tx_accepted=counts['px4_hold_tx_accepted']),
                          'fresh stock output restored authority without an acquire')
        denied_at = self.poke('MAVMOOS_NAV_CONTROL', 'acquire')
        denied = self.wait(4, lambda: self.helm_state(denied_at, reason='acquisition_rejected') and
                           self.runtime(denied_at, permission='false', nav_tx_accepted=counts['nav_tx_accepted']),
                           'acquire from retained Offboard intent was not denied')
        native = self.event('operator_explicit_hold_handback')
        self.cli('commander', 'mode', 'auto:loiter')
        handback = self.wait(7, lambda: self.current_mode(HOLD, HOLD, native) and
                             self.rover(native, hold_ready='true'),
                             'explicit native Hold did not supply reacquisition evidence')
        return {'retired': retired, 'independent_hold': independent, 'stopped': stopped,
                'fresh_helm_fenced': fresh, 'retained_intent_denied': denied, 'explicit_handback': handback}

    def withdraw_by_gps_loss(self):
        self.cli('mavlink', 'stream', '-u', '14580', '-s', 'GPS_RAW_INT', '-r', '0')
        at = self.event('bridge_gps_stream_disabled')
        retired = self.wait(4, lambda: self.runtime(at, permission='false') and self.stock_inputs(at),
                            'missing bridge GPS did not retire the grant with fresh stock helm')
        independent = self.wait(6, lambda: self.heartbeat(HOLD, True, at) and
                                self.current_mode(HOLD, OFFBOARD, at), 'independent fallback Hold absent')
        degraded = self.stopped(at, intended=OFFBOARD, confirmed=False)
        counts = {key: fields(self.runtime())[key] for key in ('nav_tx_accepted', 'px4_hold_tx_accepted')}
        denied_at = self.poke('MAVMOOS_NAV_CONTROL', 'acquire')
        def denied_without_navigation():
            helm = self.helm_state(denied_at, reason='acquisition_rejected')
            runtime = self.runtime(denied_at, permission='false', **counts)
            rover = self.rover(denied_at, stop_confirmed='false', hold_ready='false')
            return {'helm': helm, 'runtime': runtime, 'rover': rover} if all((helm, runtime, rover)) else None

        denied = self.wait(4, denied_without_navigation, 'degraded stopped rover admitted navigation')
        self.cli('mavlink', 'stream', '-u', '14580', '-s', 'GPS_RAW_INT', '-r', '10')
        restored = self.event('bridge_gps_stream_restored')
        stopped = self.stopped(restored, intended=OFFBOARD)
        fresh = self.wait(4, lambda: self.stock_inputs(restored) and
                          self.runtime(restored, permission='false', **counts),
                          'restored GPS and fresh stock helm restored authority implicitly')
        native = self.event('operator_explicit_hold_handback')
        self.cli('commander', 'mode', 'auto:loiter')
        handback = self.wait(7, lambda: self.current_mode(HOLD, HOLD, native) and
                             self.rover(native, hold_ready='true'),
                             'explicit native Hold did not supply reacquisition evidence')
        return {'retired': retired, 'independent_hold': independent, 'stopped': stopped,
                'degraded_unconfirmed_stop': degraded, 'denied_while_degraded': denied,
                'fresh_helm_fenced': fresh, 'explicit_handback': handback}

    def run_helm(self):
        deploy = self.poke('DEPLOY', 'true')
        self.wait(6, lambda: self.stock_inputs(deploy), 'stock planar helm course/speed absent')
        first = self.enter_offboard()
        straight = self.wait(12, lambda: next((row for row in [self.local_position(first['acquired_at'])] if row
                                             and self.moved(first['start'], row) >= 2), None),
                             'stock helm did not move the rover on the straight leg')
        handoff = self.handoff()
        before_turn = self.wait(3, lambda: self.global_position(0), 'pre-turn vehicle heading missing')
        if self.autonomy:
            def selected():
                row = self.latest('mail', 'ROVER_LEG', first['acquired_at'], age=1.5)
                return row if row and row['source'] == 'pHelmIvP' and row['value'] == 'turn' else None
            corner = self.wait(18, selected, 'stock transect did not select the bend')
            corner_fix = self.wait(2, lambda: self.local_position(corner['mono']), 'independent corner fix absent')
            need(self.moved(first['start'], corner_fix) >= 6, 'stock turn selected before physical corner')
            turn = corner['mono']
        else:
            turn = self.poke('ROVER_LEG', 'turn')

        def turned():
            position = self.global_position(turn)
            local = self.local_position(turn)
            if position and local and self.stock_inputs(turn) and self.heartbeat(OFFBOARD, True, turn):
                angle = math.remainder(position['heading_deg'] - before_turn['heading_deg'], 360)
                if 14 <= angle <= 65 and self.moved(straight, local) >= 2:
                    return {'heading_change_deg': angle, 'position': local, 'heading': position}
            return None

        turned_result = self.wait(15, turned, 'independent rover steering and travel evidence absent')
        fallback = self.withdraw_by_gps_loss() if self.args.case == 'gps-loss' else self.withdraw_by_silence()
        second = self.enter_offboard()
        self.wait(10, lambda: (row := self.local_position(second['acquired_at'])) and
                  self.moved(second['start'], row) >= 1, 'fresh reacquire did not produce new physical travel')
        if self.autonomy:
            def finished():
                row = self.latest('mail', 'AUTONOMY_INDEX', turn, age=4)
                return row if row and row['source'] == 'pHelmIvP' and row['value'] == 2 else None
            exit_gate = self.wait(26, finished, 'stock Ackermann route did not reach the exit waypoint')
            exit_fix = self.wait(2, lambda: self.local_position(exit_gate['mono']), 'independent exit fix absent')
            need(self.moved(corner_fix, exit_fix) >= 13, 'Ackermann route did not physically traverse the bend')
            self.report['autonomy_route'] = {'corner': corner, 'corner_fix': corner_fix,
                                              'exit_gate': exit_gate, 'exit_fix': exit_fix,
                                              'corner_to_exit_m': self.moved(corner_fix, exit_fix)}
        release = self.poke('MAVMOOS_NAV_CONTROL', 'release')
        released = self.wait(7, lambda: self.runtime(release, permission='false') and
                             self.heartbeat(HOLD, True, release), 'explicit helm release/fallback absent')
        stopped = self.stopped(release, intended=OFFBOARD)
        self.report['helm'] = {'first_acquire': first, 'straight_motion_m': self.moved(first['start'], straight),
                               'normal_handoff': handoff, 'turn': turned_result, 'fallback': fallback,
                               'fresh_acquire': second, 'release': released, 'release_stopped': stopped}

    def stock_stop(self):
        release = self.poke('MAVMOOS_NAV_CONTROL', 'release')
        retired = self.wait(7, lambda: self.runtime(release, permission='false') and
                            self.heartbeat(HOLD, True, release), 'stock release did not enter fallback Hold')
        fallback = self.stopped(release, intended=OFFBOARD)
        selected = self.event('operator_select_native_hold')
        self.cli('commander', 'mode', 'auto:loiter')
        return {'released_at': release, 'retired': retired, 'fallback': fallback,
                'native_hold': self.stopped(selected)}

    @staticmethod
    def navigation_counts(row):
        state = fields(row)
        return {key: state[key] for key in ('nav_tx_accepted', 'px4_hold_tx_accepted')}

    def route_point(self, position):
        lat, lon = (self.route_origin[key] * 1e-7 for key in ('latitude_e7', 'longitude_e7'))
        return ((position['longitude_e7'] * 1e-7 - lon) * 111195 * math.cos(math.radians(lat)),
                (position['latitude_e7'] * 1e-7 - lat) * 111195)

    def resumed_stock_route(self):
        # Handback resets the stock-helm adapter. A normal deploy cycle
        # supplies the fresh all-stop transition before new authority.
        idle = self.poke('DEPLOY', 'false')

        def stopped_helm():
            row = self.latest('mail', 'IVPHELM_ALLSTOP', idle, age=.6)
            return row if row and row['source'] == 'pHelmIvP' and row['value'] in (
                'NothingToDo', 'NoGoalBehavior') else None

        allstop = self.wait(5, stopped_helm, 'stock all-stop did not resynchronize the handback')
        fix = self.wait(3, lambda: self.global_position(0), 'stationary recovery origin missing')
        x, y = self.route_point(fix)
        bearing = math.radians(fix['heading_deg'] + 32)
        points = [(x + distance * math.sin(bearing), y + distance * math.cos(bearing))
                  for distance in (10, 18, 25)]
        update = 'points=' + ':'.join(f'{x:.3f},{y:.3f}' for x, y in points)
        update_at = self.poke('STRESS_ROUTE_UPDATE', update)
        selected = self.poke('ROVER_LEG', 'turn')
        deploy = self.poke('DEPLOY', 'true')

        def fenced_route():
            stock = self.stock_inputs(deploy)
            state = self.runtime(deploy + .5, permission='false', nav_tx_accepted=0,
                                 px4_hold_tx_accepted=0)
            ready = self.helm_state(deploy, allstop='clear', acquire_preparation='READY')
            if stock and state and ready and self.rover(deploy, hold_ready='true') and (
                    18 <= math.remainder(stock['heading']['value'] - fix['heading_deg'], 360) <= 48):
                return {'stock': stock, 'runtime': state, 'ready': ready}
            return None

        prepared = self.wait(5, fenced_route, 'new stock bend lacked fresh output or sent before explicit acquire')
        acquired = self.enter_offboard()

        def completed():
            index = self.latest('mail', 'AUTONOMY_INDEX', acquired['acquired_at'], age=3)
            position = self.local_position(acquired['acquired_at'])
            if (index and index['source'] == 'pHelmIvP' and index['value'] == 2 and
                    self.heartbeat(OFFBOARD, True, acquired['acquired_at']) and
                    position and self.moved(acquired['start'], position) >= 14):
                return index, position
            return None

        index, end = self.wait(33, completed, 'reacquired stock detour missed its two waypoint gates')
        track = iter(row for row in self.rows if row.get('message') == 'GLOBAL_POSITION_INT' and
                     acquired['acquired_at'] <= row['mono'] <= index['mono'] + .25)
        hits = []
        for point in points[:2]:
            hit = next((row for row in track if math.dist(self.route_point(row), point) <= 3), None)
            need(hit, 'independent Ackermann track missed a stock detour gate')
            hits.append(hit)
        return {'origin': self.route_origin, 'start_fix': fix, 'points': points, 'update': update,
                'allstop': allstop, 'updated_at': update_at, 'selected_at': selected, 'prepared': prepared,
                'acquired': acquired, 'index': index, 'hits': hits, 'end': end}

    def run_stress(self, proof):
        deploy = self.poke('DEPLOY', 'true')
        self.wait(6, lambda: self.stock_inputs(deploy), 'first stock route did not produce fresh course and speed')
        initial = self.enter_offboard()
        progress = self.wait(13, lambda: (position := self.local_position(initial['acquired_at'])) and
                             self.heartbeat(OFFBOARD, True, initial['acquired_at']) and
                             self.moved(initial['start'], position) >= 3 and position,
                             'independent travel on initial stock route missing')
        before_mission = self.stock_stop()
        self.run_restart(proof)
        detour = self.resumed_stock_route()
        self.report['compound'] = {'first': initial, 'first_end': progress, 'before_mission': before_mission,
                                   'detour': detour, 'terminal_stop': self.stock_stop()}

    def run(self):
        self.boot()
        proof = self.prepare_mission()
        self.arm()
        if self.args.case == 'restart':
            self.run_restart(proof)
        elif self.stress:
            self.run_stress(proof)
        elif self.args.case == 'abort':
            self.run_native_abort(proof)
        else:
            self.run_native_mission(proof)
            self.run_helm()
            # Disarm clears the FC's Offboard-loss failsafe. Select actual
            # native Hold first so a retained Offboard intent cannot return.
            select = self.event('operator_select_native_hold_for_disarm')
            self.cli('commander', 'mode', 'auto:loiter')
            self.report['pre_disarm_handback'] = self.stopped(select)
        disarm = self.event('operator_disarm')
        self.cli('commander', 'disarm')
        self.report['disarmed'] = self.wait(6, lambda: self.heartbeat(HOLD, False, disarm),
                                            'independent disarmed Hold not observed')
        need(not (self.recovery / 'auto-may-continue').exists(), 'released control left an uncertainty marker')

    def recovery_state(self, after=0, **wanted):
        row = self.latest('mail', 'MAVMOOS_AUTO_RECOVERY_STATUS', after, age=1.6)
        state = fields(row)
        return state if row and row['source'] == 'iMAVLink' and all(
            state.get(key) == str(value) for key, value in wanted.items()) else None

    def run_restart(self, proof):
        _, active, first = self.start_native_mission(proof)
        if self.stress:
            pause = self.command('rover-pause', 'pause')
            paused = self.wait(16, lambda: self.mission_result('rover-pause', 'COMPLETED', pause),
                               'compound Mission pause did not complete')
            stopped = self.stopped(pause)
            before = self.wait(3, lambda: self.local_position(pause), 'paused Mission position missing')
            cursor = self.wait(3, lambda: self.latest('wire', 'MISSION_CURRENT', pause), 'paused cursor missing')
            counts = self.navigation_counts(self.runtime())
            attempted = self.poke('MAVMOOS_NAV_CONTROL', 'acquire')
            denial = self.wait(4, lambda: self.stock_inputs(attempted) and
                               self.runtime(attempted, permission='false', mission_execution_held='true', **counts) and
                               self.helm_state(attempted, reason='acquisition_rejected'),
                               'fresh stock helm bypassed a paused native Mission lease')
            resume = self.command('rover-resume', 'resume')
            resumed = self.wait(16, lambda: self.mission_result('rover-resume', 'COMPLETED', resume),
                                'paused Mission did not resume')
            same_cursor = self.wait(4, lambda: self.current_mode(MISSION, MISSION, resume) and
                                    (row := self.latest('wire', 'MISSION_CURRENT', resume)) and
                                    row['sequence'] == cursor['sequence'] and row['mission_id'] == cursor['mission_id']
                                    and row, 'resumed Mission changed its independent cursor')
            first = self.wait(6, lambda: (row := self.local_position(resume)) and
                              self.moved(before, row) >= 1 and row, 'resumed Mission did not move again')
            self.report['pause_resume'] = {'paused': paused, 'stopped': stopped, 'before': before, 'cursor': cursor,
                                           'attempted_at': attempted, 'denied': denial, 'resumed': resumed,
                                           'same_cursor': same_cursor, 'after': first, 'fenced_counts': counts}
        old = self.wait(3, lambda: self.recovery_state(state='OWNED', marker='present'),
                        'durable mission ownership marker absent before interruption')
        marker = self.recovery / 'auto-may-continue'
        need(marker.is_file() and not marker.is_symlink(), 'unexpected native mission marker')
        bridge = self.bridge
        need(bridge and bridge.poll() is None and Path(f'/proc/{bridge.pid}/exe').resolve() == self.app,
             'restart signal would target an unexpected process')
        crash = self.event('owned_bridge_interrupted', pid=bridge.pid, session=old['session'])
        os.kill(bridge.pid, signal.SIGKILL)
        need(bridge.wait(timeout=3) == -signal.SIGKILL and marker.is_file(), 'interruption or marker persistence differs')
        self.processes.remove(bridge)
        continued = self.wait(6, lambda: self.heartbeat(MISSION, True, crash) and
                              (row := self.local_position(crash)) and self.moved(first, row) >= 1 and row,
                              'remote Mission did not independently show continued motion across bridge loss')
        still_mission = self.heartbeat(MISSION, True, crash)
        mission = self.out / 'targ_px4.moos'
        launch = self.event('new_owned_bridge', original_px4_pid=self.px4.pid)
        self.bridge = self.launch([self.app, mission, 'iMAVLink'], 'bridge-two')
        new = self.wait(12, lambda: self.recovery_state(launch, state='UNRESOLVED', marker='present',
                                                       effects_blocked='true', ready_for_handback='false'),
                        'new bridge did not recover retained Mission uncertainty')
        need(new['session'] != old['session'] and self.stock_helm_process(mission) == self.helm_record,
             'restart changed stock helm or did not create a new bridge session')
        blocked = self.poke('MAVMOOS_NAV_CONTROL', 'acquire')
        blocked_result = self.wait(4, lambda: self.runtime(blocked, permission='false', nav_tx_accepted=0,
                                                          px4_hold_tx_accepted=0) and
                                   self.helm_state(blocked, reason='acquisition_rejected'),
                                   'new bridge admitted helm while remote Mission remained uncertain')
        hold = self.event('operator_select_hold_after_restart')
        self.cli('commander', 'mode', 'auto:loiter')
        stopped = self.stopped(hold)
        ready = self.wait(5, lambda: self.recovery_state(hold, state='UNRESOLVED', ready_for_handback='true'),
                          'explicit Hold and advancing stopped evidence did not permit handback')
        wrong = self.poke('MAVMOOS_AUTO_RECOVERY_REQUEST', 'schema=1,id=wrong-session,action=handback,session='
                          + old['session'])
        rejected = self.wait(5, lambda: self.event_result('MAVMOOS_AUTO_RECOVERY_RESULT', 'wrong-session',
                                                          'REJECTED', wrong), 'stale recovery session was not rejected')
        need(marker.is_file(), 'rejected handback cleared marker')
        release = self.poke('MAVMOOS_AUTO_RECOVERY_REQUEST', 'schema=1,id=owned-handback,action=handback,session='
                            + ready['session'])
        released = self.wait(6, lambda: self.event_result('MAVMOOS_AUTO_RECOVERY_RESULT', 'owned-handback',
                                                          'RELEASED', release) and
                             self.runtime(release, permission='false', nav_tx_accepted=0),
                             'explicit handback did not clear uncertainty without granting control')
        self.report['restart'] = {'mission': active, 'crash': crash, 'new_session': new, 'old_session': old,
                                  'stopped_hold': stopped, 'wrong_session_denied': rejected, 'released': released}
        if self.stress:
            self.report['restart'].update(continued=continued, continued_mode=still_mission, launched_at=launch,
                                          blocked_at=blocked, blocked_result=blocked_result)

    def cleanup(self):
        report = super().cleanup()
        report['ports_clear'].update(ports_free())
        return report


def run_candidate(revision, build_revision, output, build_manifest, case):
    pin = json.loads(STABLE_PIN.read_text())
    need(pin['revision'] == PX4_REVISION, 'pinned stable PX4 revision differs')
    pin['files'].update(EXTRA_PINS)
    args = argparse.Namespace(output=output, source_revision=revision, build_source_revision=build_revision,
                              db_port=15300, px4_root=Path('/opt/px4'),
                              moos_root=Path('/work/moos-ivp'), behavior_dir=Path('/work/moos-ivp/lib'),
                              build=Path('/candidate'), probe=None, nsplug=None, stable_pin=pin,
                              build_manifest=build_manifest, case=case)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(143)))
    return main(args, AckermannWorkflow)
