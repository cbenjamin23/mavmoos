#!/usr/bin/env python3
"""One stock-helm flight on the pinned PX4 SIH package in a private Linux namespace."""
import argparse
import errno
import hashlib
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
import time

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from owned_processes import owned_records, stop_process_tree

LOOPBACK = '127.0.0.1'
PORTS = (14540, 14550, 14580, 18570)
PX4_REVISION = 'fca3df865af36124a28c9d607e850f111dbaaea9'
STABLE_PX4_REVISION = 'd6f12ad1c4f70ad3230afd7d86e971421e02fef4'
STABLE_PIN = HERE / 'firmware-v1.17.0.json'
PX4_PINS = {
    'bin/px4': '23f0b5eb07430868b5525a5c59e28185fe5b0da35ade9a7e04bdea5894144ea8',
    'etc/init.d-posix/rcS': '124da15a45de3d682e51810774992c07b9bdcd5d154766f42ed8f21431251854',
    'etc/init.d-posix/px4-rc.mavlink': '81532497c837d022b9322033c136dd6d3aa3b99522948fccc62e49e8e6195d1d',
}
HOLD, OFFBOARD = (4 << 16) | (3 << 24), 6 << 16
FIXTURE_PARAMS = {'COM_OF_LOSS_T': 0.7, 'COM_OBL_RC_ACT': 5, 'MIS_TAKEOFF_ALT': 8}


def need(ok, why):
    if not ok:
        raise RuntimeError(why)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def namespace():
    flags = {path.parent.name: int(path.read_text().strip(), 16)
             for path in Path('/sys/class/net').glob('*/flags')}
    status = dict(line.split(':', 1) for line in Path('/proc/self/status').read_text().splitlines())
    routes = Path('/proc/net/route').read_text().splitlines()[1:]
    roots = [line for line in Path('/proc/self/mountinfo').read_text().splitlines() if line.split()[4] == '/']
    need([name for name, value in flags.items() if value & 1] == ['lo'] and not routes,
         'active test requires an isolated loopback-only namespace')
    need(status['CapEff'].strip() == '0000000000000000' and status['NoNewPrivs'].strip() == '1' and
         len(roots) == 1 and 'ro' in roots[0].split()[5].split(','),
         'active test requires capability drop, no-new-privileges, and read-only root')
    return {'interface_flags': flags, 'ipv4_routes': routes, 'cap_eff': status['CapEff'].strip(),
            'no_new_privileges': True, 'root_read_only': True}


def udp_sockets(pid):
    sockets = set()
    for path in Path(f'/proc/{pid}/fd').iterdir():
        try:
            target = os.readlink(path)
        except FileNotFoundError:
            continue
        if target.startswith('socket:['):
            sockets.add(target[8:-1])
    records = []
    for line in Path('/proc/net/udp').read_text().splitlines()[1:]:
        parts = line.split()
        if parts[9] not in sockets:
            continue
        ip, port = parts[1].split(':')
        peer, peer_port = parts[2].split(':')
        need(ip in ('00000000', '0100007F') and peer in ('00000000', '0100007F'),
             'owned simulator UDP socket left the isolated loopback route')
        records.append({'local': ip, 'local_port': int(port, 16), 'peer': peer,
                        'peer_port': int(peer_port, 16), 'inode': parts[9]})
    return records


def free_ports(db):
    verdict = {}
    for port in PORTS:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as endpoint:
            try:
                endpoint.bind(('0.0.0.0', port))
                verdict[str(port)] = True
            except OSError:
                verdict[str(port)] = False
    with socket.socket() as endpoint:
        endpoint.settimeout(.2)
        verdict['db'] = endpoint.connect_ex((LOOPBACK, db)) == errno.ECONNREFUSED
    return verdict


def fields(row):
    value = row.get('value') if row else None
    return dict(part.split('=', 1) for part in value.split(',') if '=' in part) if isinstance(value, str) else {}


class Workflow:
    model = 'sihsim_quadx'
    mav_type = 2
    profile = 'px4_multicopter'
    stable_model_label = r'Quadcopter'
    template = 'meta_vehicle'
    fixture_params = FIXTURE_PARAMS
    probe_args = ()
    scope = 'owned PX4 SIH quad stock helm Offboard, failsafe Hold, and explicit handback'

    def __init__(self, args, out, report):
        self.args, self.out, self.report = args, out, report
        self.deadline = time.monotonic() + 105
        self.processes = []
        self.probe = self.px4 = self.antler = None
        self.reader = None
        self.helm_record = None
        self.paused = False
        self.rows = []
        self.latest_rows = {}
        self.db = args.db_port
        self.px4_root, self.moos, self.build = args.px4_root, args.moos_root, args.build
        self.bindir = self.moos / 'bin'
        self.app = self.build / 'bin/iMAVLink'
        self.probe_path = args.probe or self.build / 'bin/px4_offboard_moos_probe'
        self.nsplug = args.nsplug or self.bindir / 'nsplug'
        self.helm = self.bindir / 'pHelmIvP'
        self.native = self.px4_root / 'bin/px4'
        self.env = {
            'PATH': os.pathsep.join(map(str, (self.native.parent, self.app.parent, self.bindir,
                                           '/usr/local/bin', '/usr/bin', '/bin'))),
            'LC_ALL': 'C', 'IVP_BEHAVIOR_DIRS': str(args.behavior_dir),
            'PX4_SIM_MODEL': self.model, 'PX4_SIMULATOR': 'sihsim',
        }
        self.protected = {}

    def stable_work_pins(self, stable):
        return {Path('/work') / name: digest for name, digest in stable['retained_work_files'].items()}

    def fixture_sources(self):
        return [Path(__file__), HERE / 'Px4MoosProbe.cpp',
                HERE / (self.template + '.moos'), HERE / (self.template + '.bhv')]

    def protect(self):
        need(os.statvfs(self.px4_root).f_flag & os.ST_RDONLY, 'PX4 package must be mounted read-only')
        stable = self.args.stable_pin
        pins = {self.px4_root / name: digest for name, digest in (stable['files'] if stable else PX4_PINS).items()}
        work_inputs = [
            self.app, self.probe_path, self.helm, self.bindir / 'MOOSDB', self.bindir / 'pAntler',
            self.bindir / 'pLogger', self.nsplug, self.args.behavior_dir / 'libBHV_ConstantAltitude.so',
            self.build / 'plugins/mavmoos_telemetry.so', self.build / 'plugins/mavmoos_px4_control.so']
        if stable:
            work_pins = self.stable_work_pins(stable)
            need(work_pins.keys() == set(work_inputs) and os.statvfs('/work').f_flag & os.ST_RDONLY,
                 'stable candidate must use its exact pinned build inputs and read-only stock dependency mount')
            pins.update(work_pins)
        needed = [*pins, *work_inputs, *self.fixture_sources()]
        self.protected = {str(path): sha(path) for path in dict.fromkeys(needed)}
        for path, expected in pins.items():
            need(self.protected[str(path)] == expected, f'exact pinned PX4 or retained release input differs: {path}')

    def launch(self, argv, label, operator_trace=None):
        log = self.out / (label + '.log')
        with log.open('xb') as output:
            if operator_trace is not None:
                with operator_trace.open('xb') as trace:
                    process = subprocess.Popen(list(map(str, argv)), cwd=self.out, env=self.env, text=True,
                                               stdout=trace, stderr=output, stdin=subprocess.PIPE,
                                               start_new_session=True)
            else:
                process = subprocess.Popen(list(map(str, argv)), cwd=self.out, env=self.env,
                                           stdout=output, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                           start_new_session=True)
        self.processes.append(process)
        return process

    def collect(self):
        if self.reader is None:
            return
        while True:
            at = self.reader.tell()
            line = self.reader.readline()
            if not line.endswith('\n'):
                self.reader.seek(at)
                return
            if not line.startswith('{'):
                continue
            row = json.loads(line)
            self.rows.append(row)
            key = row.get('message') or row.get('key')
            self.latest_rows[row['kind'], key] = row

    def wait(self, duration, check, why):
        until = min(time.monotonic() + duration, self.deadline)
        while time.monotonic() < until:
            for child in self.processes:
                need(child.poll() is None, f'owned process {Path(child.args[0]).name} exited')
            self.collect()
            result = check()
            if result:
                return result
            time.sleep(.025)
        raise RuntimeError(why)

    def latest(self, kind, key=None, after=0, age=2):
        row = self.latest_rows.get((kind, key))
        return row if row and row['mono'] >= after and time.monotonic() - row['mono'] <= age else None

    def runtime(self, after=0, **values):
        row = self.latest('mail', 'MAVMOOS_RUNTIME_STATUS', after, age=.5)
        state = fields(row)
        return row if row and row['source'] == 'iMAVLink' and all(
            state.get(key) == str(value) for key, value in values.items()) else None

    def air_mode(self, mode, after=0):
        row = self.latest('wire', 'HEARTBEAT', after)
        landed = self.latest('wire', 'EXTENDED_SYS_STATE', after)
        return row if row and landed and row['mav_type'] == self.mav_type and row['autopilot'] == 12 and row['armed'] and \
            row['mode'] == mode and landed['landed_state'] == 2 else None

    def current_mode(self, active, intended, after=0):
        row = self.latest('wire', 'CURRENT_MODE', after, age=4)
        return row if row and row['mode'] == active and row['intended_mode'] == intended else None

    def event(self, name, **details):
        self.report.setdefault('events', []).append({'event': name, 'mono': time.monotonic(), **details})
        return self.report['events'][-1]['mono']

    def cli(self, component, *args):
        need(self.px4 is not None and self.px4.poll() is None and
             Path(f'/proc/{self.px4.pid}/exe').resolve() == self.native,
             'PX4 CLI lost the exact owned simulator process')
        argv = [str(self.native.parent / ('px4-' + component)), *args]
        result = subprocess.run(argv, cwd=self.out, env=self.env, capture_output=True, text=True, timeout=5)
        with (self.out / 'fixture-cli.log').open('a') as log:
            log.write(json.dumps({'mono': time.monotonic(), 'argv': argv, 'exit': result.returncode,
                                  'stdout': result.stdout, 'stderr': result.stderr}) + '\n')
        need(result.returncode == 0, f'owned PX4 CLI failed: {component} {args}')
        return result.stdout + result.stderr

    def poke(self, key, value):
        self.probe.stdin.write(f'{key}\t{value}\n')
        self.probe.stdin.flush()
        return self.event('operator_request', key=key, value=value)

    def generate(self, origin):
        settings = dict(MOOS_PORT=self.db, DB_PATH=self.bindir / 'MOOSDB', APP_PATH=self.app,
                        HELM_PATH=self.helm, LOGGER_PATH=self.bindir / 'pLogger', SITL_PID=self.px4.pid,
                        PLUGIN_DIRECTORY=self.build / 'plugins', BEHAVIOR_DIRECTORY=self.args.behavior_dir,
                        LAT_ORIGIN=origin['latitude_e7'] / 1e7, LONG_ORIGIN=origin['longitude_e7'] / 1e7)
        for suffix, variables in (('moos', settings), ('bhv', {})):
            source, target = HERE / (self.template + '.' + suffix), self.out / ('targ_px4.' + suffix)
            subprocess.run([str(self.nsplug), str(source), str(target), '--strict', '--force', '-x',
                            *(f'{key}={value}' for key, value in variables.items())], cwd=self.out, env=self.env,
                           capture_output=True, text=True, timeout=5, check=True)
            need(target.is_file() and '$(' not in target.read_text(), 'unresolved stock mission substitution')
        return self.out / 'targ_px4.moos'

    def stock_helm_process(self, mission):
        records = []
        for record in owned_records(self.antler):
            argv = shlex.split(record['command'])
            if argv and argv[0] == str(self.helm) and str(mission) in argv:
                records.append(record)
        need(len(records) <= 1, 'duplicated stock helm in the private mission')
        return records[0] if records else None

    def stock_inputs(self, after=0):
        expected = {'DESIRED_HEADING': (80, 100), 'DESIRED_SPEED': (1.9, 2.1), 'DESIRED_ALTITUDE': (7.9, 8.1)}
        inputs = {}
        for key, (minimum, maximum) in expected.items():
            row = self.latest('mail', key, after, age=.5)
            if not row or row['source'] != 'pHelmIvP' or not isinstance(row['value'], (float, int)) or \
                    not minimum <= row['value'] <= maximum:
                return None
            inputs[key] = row
        drive = self.latest('mail', 'IVPHELM_STATE', after, age=.5)
        return inputs if drive and drive['source'] == 'pHelmIvP' and drive['value'] == 'DRIVE' else None

    def start_probe(self, prefix=''):
        started = time.monotonic()
        trace = self.out / (prefix + 'moos-wire.jsonl')
        self.probe = self.launch([self.probe_path, self.db, self.out / (prefix + 'gcs.mavlink'), *self.probe_args],
                                 prefix + 'operator', operator_trace=trace)
        self.reader = trace.open()
        self.wait(4, lambda: self.latest('listening', after=started), 'independent GCS observer did not bind')
        return started

    def boot(self):
        need(all(free_ports(self.db).values()), 'private fixture ports are in use')
        self.start_probe('preflight-' if self.args.stable_pin else '')
        self.px4 = self.launch([self.native, '-d', self.px4_root / 'etc', '-w', self.out, '-s',
                                self.px4_root / 'etc/init.d-posix/rcS'], 'startup')
        self.wait(20, lambda: 'Ready for takeoff' in (self.out / 'startup.log').read_text(errors='replace'),
                  'exact SIH startup never became ready for takeoff')
        version = self.cli('ver', 'all')
        expected_variant = 'default' if self.args.stable_pin else 'sih'
        need(re.search(r'PX4 git-hash:\s*' + self.report['firmware_revision'] + r'\b', version) and
             re.search(r'Build variant:\s*' + expected_variant + r'\s*$', version, re.M),
             'owned PX4 version does not match the selected SIH package')
        sockets = udp_sockets(self.px4.pid)
        need({14580, 18570}.issubset(row['local_port'] for row in sockets), 'exact PX4 PID lacks owned UDP sockets')
        self.report.update(px4_pid=self.px4.pid, px4_udp_sockets=sockets, px4_cli_version=version)
        if self.args.stable_pin:
            status = self.cli('simulator_sih', 'status')
            self.report['px4_sih_status'] = status
            need(re.search(r'PX4 version:\s*Release 1\.17\.0\b', version) and
                 re.search(self.stable_model_label, status),
                 'owned stable firmware did not report its release version and expected SIH model')
        for param, expected in self.fixture_params.items():
            self.cli('param', 'set', param, str(expected))
            value = re.search(r'\b' + param + r'\b[^\r\n]*?:\s*([-+0-9.]+)', self.cli('param', 'show', param))
            need(value and math.isclose(float(value[1]), expected, abs_tol=.001), f'fixture parameter readback {param}')
        self.report['verified_fixture_parameters'] = self.fixture_params
        origin = self.wait(8, lambda: self.latest('wire', 'GLOBAL_POSITION_INT') if
                           (self.latest('wire', 'GLOBAL_POSITION_INT') or {}).get('latitude_e7') else None,
                           'independent global origin absent')
        mission = self.generate(origin)
        self.antler = self.launch([self.bindir / 'pAntler', mission], 'pantler')
        self.helm_record = self.wait(10, lambda: self.stock_helm_process(mission),
                                     'pAntler did not start the exact stock helm')
        self.report['stock_helm_process'] = self.helm_record
        self.wait(8, lambda: self.runtime(configuration_valid='true', heartbeat_valid=1,
                                         control_profile=self.profile, nav_tx_accepted=0, armed=0),
                  'disarmed PX4 app configuration or heartbeat missing')
        self.event('booted_disarmed', origin=origin)

    def acquire_and_travel(self, nav_before, hold_before):
        acquire = self.poke('MAVMOOS_NAV_CONTROL', 'acquire')
        offboard = self.wait(8, lambda: self.air_mode(OFFBOARD, acquire), 'no observed PX4 Offboard after acquisition')
        sequence = [fields(row) for row in self.rows if row.get('key') == 'MAVMOOS_RUNTIME_STATUS' and
                    acquire <= row['mono'] <= offboard['mono']]
        prestream = max((int(row.get('px4_prestream_count', 0)) for row in sequence), default=0)
        holds = max((int(row.get('px4_hold_tx_accepted', 0)) for row in sequence), default=0)
        need(prestream >= 10 and holds - hold_before >= 10 and offboard['mono'] - acquire >= 1.25,
             'independent Offboard entry lacked prior accepted hold streaming')
        self.wait(4, lambda: self.runtime(acquire, permission='true', fc_mode=OFFBOARD,
                                         px4_offboard_confirmed=1, reason='px4_offboard_motion'),
                  'bridge did not confirm motion authority from the observed Offboard mode')
        start = self.wait(2, lambda: self.latest('wire', 'LOCAL_POSITION_NED', offboard['mono']),
                          'no independent start position after Offboard entry')

        def traveled():
            row = self.latest('wire', 'LOCAL_POSITION_NED')
            status = fields(self.runtime())
            return row if row and row['east'] - start['east'] >= 4 and abs(row['north'] - start['north']) <= 3 and \
                row['vy'] >= 1 and int(status.get('nav_tx_accepted', 0)) - nav_before >= 15 and \
                status.get('permission') == 'true' and self.air_mode(OFFBOARD, acquire) else None

        end = self.wait(10, traveled, 'stock helm did not produce observed eastbound Offboard travel')
        return {'independent_offboard': offboard, 'prestream_count': prestream,
                'hold_tx_accepted': holds - hold_before,
                'travel': {'start': start, 'end': end, 'east_m': end['east'] - start['east']}}

    def stable_mission_processes(self):
        records = owned_records(self.antler)
        mission = str(self.out / 'targ_px4.moos')
        identities = {}
        for name, executable in (('app', self.app), ('helm', self.helm)):
            matches = [row for row in records if (argv := shlex.split(row['command'])) and
                       argv[0] == str(executable) and mission in argv]
            need(len(matches) == 1, f'stable preflight requires exactly one owned {name} process')
            row = matches[0]
            proc = Path('/proc') / str(row['pid'])
            need((proc / 'exe').resolve() == executable, f'stable preflight {name} executable differs')
            identities[name] = {**row, 'start_ticks': int((proc / 'stat').read_text().rpartition(') ')[2].split()[19])}
        return identities

    def settle_stable_home(self, takeoff):
        started = self.event('stable_home_settle_started')
        correction_window = self.args.stable_pin['home_correction_window_us']
        settle_limit = self.args.stable_pin['home_settle_timeout_s']
        original_deadline = self.deadline
        self.deadline += settle_limit
        witness = {'correction_window_us': correction_window, 'started_at': started, 'samples': [],
                   'initial_hold': self.air_mode(HOLD, takeoff), 'initial_observer_pid': self.probe.pid,
                   'processes_before': self.stable_mission_processes()}
        self.report['stable_home_settle'] = witness
        # The frozen probe has its own 115 s lifetime. Keep the same app,
        # helm and simulator; reopen the same passive probe after the dwell.
        self.probe.stdin.close()
        witness['initial_observer_exit'] = self.probe.wait(timeout=5)
        need(witness['initial_observer_exit'] == 0 and free_ports(self.db)['14550'],
             'initial stable GCS probe did not release its owned port')
        self.processes.remove(self.probe)
        self.probe = None
        self.reader.close()
        self.reader = None
        next_poll = started
        stable_samples = witness['samples']

        def topic_integer(topic, field):
            match = re.search(r'^\s*' + field + r':\s*(\d+)\b', topic, re.M)
            need(match, f'owned PX4 topic lacks {field}')
            return int(match[1])

        def sample():
            nonlocal next_poll
            now = time.monotonic()
            if now < next_poll:
                return False
            next_poll = now + 2.2
            status = self.cli('listener', 'vehicle_status', '-n', '1')
            # v1.17.0 vehicle_status constants are ARMING_STATE_ARMED=2
            # and NAVIGATION_STATE_AUTO_LOITER=4.
            need(topic_integer(status, 'arming_state') == 2 and topic_integer(status, 'nav_state') == 4 and
                 topic_integer(status, 'nav_state_user_intention') == 4,
                 'stable PX4 left armed native Hold during home settle')
            takeoff_us = topic_integer(status, 'takeoff_time')
            flight_us = topic_integer(status, 'timestamp') - takeoff_us
            need(takeoff_us > 0 and flight_us >= 0, 'stable PX4 takeoff clock is missing or inconsistent')
            witness['last_flight_time_us'] = flight_us
            if flight_us < correction_window:
                return False
            home = self.cli('listener', 'home_position', '-n', '1')
            sample = {'mono': time.monotonic(), 'flight_time_us': flight_us, 'takeoff_time_us': takeoff_us,
                      'home_timestamp_us': topic_integer(home, 'timestamp'),
                      'home_update_count': topic_integer(home, 'update_count')}
            identity = ('takeoff_time_us', 'home_timestamp_us', 'home_update_count')
            if stable_samples and any(sample[key] != stable_samples[-1][key] for key in identity):
                stable_samples.clear()
            stable_samples.append(sample)
            return len(stable_samples) >= 3 and sample['mono'] - stable_samples[0]['mono'] >= 6

        self.wait(settle_limit, sample,
                  'stable PX4 did not show a settled home reference after its takeoff correction window')
        restarted = self.start_probe()
        witness['final_observer_pid'] = self.probe.pid
        witness['independent_hold_after_settle'] = self.wait(
            8, lambda: self.air_mode(HOLD, restarted) and
            self.runtime(restarted, permission='false', home_valid=1, nav_tx_accepted=0, px4_hold_tx_accepted=0,
                         px4_native_hold_ready=1) and self.current_mode(HOLD, HOLD, restarted),
            'fresh independent native Hold and zero authority were not observed after home settle')
        witness['processes_after'] = self.stable_mission_processes()
        need(witness['processes_after'] == witness['processes_before'],
             'owned bridge or stock helm changed across the stable preflight dwell')
        self.deadline = original_deadline + (time.monotonic() - started)

    def run(self):
        self.boot()
        takeoff = self.event('fixture_takeoff')
        self.cli('commander', 'takeoff')
        self.wait(24, lambda: self.air_mode(HOLD, takeoff) and
                  5 <= (self.latest('wire', 'GLOBAL_POSITION_INT') or {}).get('relative_m', -1) <= 12 and
                  abs((self.latest('wire', 'LOCAL_POSITION_NED') or {}).get('vz', 99)) < .45,
                  'fixture did not reach stable armed airborne Hold')
        if self.args.stable_pin:
            self.settle_stable_home(takeoff)
        deploy = self.poke('DEPLOY', 'true')
        def prepared_inputs():
            helm = self.latest('mail', 'MAVMOOS_HELM_STATUS')
            if helm and helm['source'] == 'iMAVLink' and fields(helm).get('acquire_preparation') == 'READY':
                return self.stock_inputs(deploy)
            return None

        inputs = self.wait(7, prepared_inputs, 'stock helm did not produce eligible course, speed and altitude')
        self.report['stock_inputs'] = inputs
        self.wait(5, lambda: self.runtime(permission='false', nav_tx_accepted=0,
                                         px4_hold_tx_accepted=0, px4_native_hold_ready=1),
                  'fresh native Hold or zero pre-acquire setpoints not observed')
        self.report.update(self.acquire_and_travel(0, 0))
        need(self.stock_helm_process(self.out / 'targ_px4.moos') == self.helm_record, 'stock helm process identity changed')
        paused_at = self.event('stock_helm_paused', pid=self.helm_record['pid'])
        os.kill(self.helm_record['pid'], signal.SIGSTOP)
        self.paused = True
        revoked = self.wait(3, lambda: self.runtime(paused_at, permission='false'),
                            'helm silence did not retire bridge authority')
        retired = fields(revoked)
        final_count = int(retired['nav_tx_accepted'])
        final_hold = int(retired['px4_hold_tx_accepted'])
        self.report.update(retired_nav_count=final_count, retired_hold_count=final_hold)
        self.report['retired_at'] = revoked['mono']
        fallback = self.wait(7, lambda: self.air_mode(HOLD, paused_at), 'PX4 did not independently enter Hold after stream loss')
        self.report['independent_loss_hold'] = fallback
        self.report['px4_after_loss'] = self.cli('commander', 'status')
        self.wait(5, lambda: (self.latest('wire', 'LOCAL_POSITION_NED') or {}).get('mono', 0) > fallback['mono'] and
                  math.hypot((self.latest('wire', 'LOCAL_POSITION_NED') or {}).get('vx', 99),
                             (self.latest('wire', 'LOCAL_POSITION_NED') or {}).get('vy', 99)) <= .5,
                  'PX4 Hold did not arrest horizontal travel')
        self.resume_helm()
        resumed = self.event('stock_helm_resumed')
        self.wait(5, lambda: self.stock_inputs(resumed), 'stock helm did not resume producing inputs')
        self.wait(5, lambda: time.monotonic() >= resumed + 2 and self.air_mode(HOLD, resumed) and
                  self.runtime(resumed, permission='false', nav_tx_accepted=final_count),
                  'resumed stock inputs did not stay latched without explicit reacquisition')
        after_revoke = [fields(row) for row in self.rows if row.get('key') == 'MAVMOOS_RUNTIME_STATUS' and
                        row['mono'] >= revoked['mono']]
        need(all(int(row['nav_tx_accepted']) == final_count and int(row['px4_hold_tx_accepted']) == final_hold
                 for row in after_revoke), 'new PX4 setpoint handoff occurred after authority retirement')
        self.event('observed_hold_without_automatic_reacquire')
        self.reacquire_after_native_hold(paused_at, final_count, final_hold)
        land = self.event('fixture_land')
        self.cli('commander', 'land')
        # SIH simulation ran slower than wall time on the CPU-limited runner.
        # Allow touchdown detection and PX4 auto-disarm to appear on the GCS link.
        landed = self.wait(30, lambda: (self.latest('wire', 'EXTENDED_SYS_STATE', land) or {}).get('landed_state') == 1 and
                           (self.latest('wire', 'HEARTBEAT', land) or {}).get('armed') is False,
                           'fixture did not land and disarm')
        self.report['landed_disarmed'] = landed

    def reacquire_after_native_hold(self, paused_at, nav_before, hold_before):
        def failsafe_mode():
            if self.runtime(paused_at, px4_native_hold_ready=0, px4_reported_mode=HOLD, px4_intended_mode=OFFBOARD):
                return self.current_mode(HOLD, OFFBOARD, paused_at)
            return None

        self.report['failsafe_current_mode'] = self.wait(
            6, failsafe_mode, 'independent and app failsafe Hold/Offboard evidence did not agree')
        denied_at = self.poke('MAVMOOS_NAV_CONTROL', 'acquire')

        def denied():
            row = self.latest('mail', 'MAVMOOS_HELM_STATUS', denied_at, age=.5)
            return row if row and row['source'] == 'iMAVLink' and fields(row).get('reason') == 'acquisition_rejected' else None

        self.report['denied_failsafe_acquire'] = self.wait(4, denied, 'failsafe Hold accepted an explicit acquire')
        self.wait(5, lambda: time.monotonic() >= denied_at + 2 and self.air_mode(HOLD, denied_at) and
                  self.runtime(denied_at, permission='false', nav_tx_accepted=nav_before, px4_hold_tx_accepted=hold_before),
                  'failsafe Hold emitted setpoints or switched mode after denied acquisition')

        handback = self.event('fixture_native_hold_handback')
        self.cli('commander', 'mode', 'auto:loiter')

        def native_hold():
            if self.runtime(handback, px4_native_hold_ready=1, permission='false',
                            nav_tx_accepted=nav_before, px4_hold_tx_accepted=hold_before):
                return self.current_mode(HOLD, HOLD, handback)
            return None

        self.report['handback_current_mode'] = self.wait(
            7, native_hold, 'independent and app intended Hold did not follow native handback')
        self.report['px4_after_native_handback'] = self.cli('commander', 'status')
        need('user intended navigation mode: Hold' in self.report['px4_after_native_handback'],
             'PX4 CLI and independent mode observation disagree')
        self.report['second_acquisition'] = self.acquire_and_travel(nav_before, hold_before)
        self.release_after_reacquisition()

    def release_after_reacquisition(self):
        release = self.poke('MAVMOOS_NAV_CONTROL', 'release')
        self.wait(4, lambda: self.runtime(release, permission='false'), 'explicit release did not revoke bridge permission')
        self.report['independent_release_hold'] = self.wait(
            7, lambda: self.air_mode(HOLD, release), 'PX4 did not enter Hold after operator release')

    def resume_helm(self):
        if self.paused:
            need(self.stock_helm_process(self.out / 'targ_px4.moos') == self.helm_record,
                 'paused helm process identity changed')
            os.kill(self.helm_record['pid'], signal.SIGCONT)
            self.paused = False

    def cleanup(self):
        resume_error = None
        try:
            self.resume_helm()
        except Exception as error:
            resume_error = str(error)
        if self.probe and self.probe.poll() is None:
            self.probe.stdin.close()
            try:
                self.probe.wait(timeout=2)
            except subprocess.TimeoutExpired:
                pass
        if self.reader is not None:
            self.reader.close()
        remaining = [row for proc in reversed(self.processes) for row in stop_process_tree(proc)]
        return {'owned_processes_remaining': remaining, 'ports_clear': free_ports(self.db),
                'resume_error': resume_error,
                'artifacts_before': self.protected,
                'artifact_drift': [path for path, digest in self.protected.items()
                                   if not Path(path).is_file() or sha(Path(path)) != digest]}


def main(args, workflow_type=Workflow):
    out = args.output
    out.mkdir(parents=True, exist_ok=False)
    report = {'result': 'FAIL', 'source_revision': args.source_revision,
              'firmware_revision': STABLE_PX4_REVISION if args.stable_pin else PX4_REVISION,
              'scope': workflow_type.scope}
    workflow = workflow_type(args, out, report)
    try:
        report['namespace'] = namespace()
        workflow.protect()
        workflow.run()
        report['result'] = 'PASS'
    except Exception as error:
        report['error'] = str(error)
        report['last_runtime'] = workflow.latest_rows.get(('mail', 'MAVMOOS_RUNTIME_STATUS'))
        report['last_helm'] = workflow.latest_rows.get(('mail', 'MAVMOOS_HELM_STATUS'))
    finally:
        try:
            report.update(workflow.cleanup())
            if (report['owned_processes_remaining'] or not all(report['ports_clear'].values()) or
                    report['artifact_drift'] or report['resume_error']):
                report.update(result='FAIL', cleanup_error='owned resources or inputs changed')
        except Exception as error:
            report.update(result='FAIL', cleanup_error=str(error))
        report['files'] = {str(path.relative_to(out)): sha(path) for path in sorted(out.rglob('*'))
                           if path.is_file() and not path.is_symlink()}
        (out / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({key: report[key] for key in ('result', 'error', 'cleanup_error', 'ports_clear')
                      if key in report}, indent=2))
    return int(report['result'] != 'PASS')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--source-revision', required=True)
    parser.add_argument('--probe', type=Path)
    parser.add_argument('--nsplug', type=Path)
    parser.add_argument('--build', type=Path, default=Path('/work/mavmoos-build'))
    parser.add_argument('--moos-root', type=Path, default=Path('/work/moos-ivp'))
    parser.add_argument('--behavior-dir', type=Path, default=Path('/work/altitude/build/lib'))
    parser.add_argument('--px4-root', type=Path, default=Path('/opt/px4'))
    parser.add_argument('--stable-v1.17.0', action='store_true', dest='stable_v1_17_0')
    parser.add_argument('--db-port', type=int, default=15300)
    args = parser.parse_args()
    args.stable_pin = json.loads(STABLE_PIN.read_text()) if args.stable_v1_17_0 else None
    if args.stable_pin and args.stable_pin['revision'] != STABLE_PX4_REVISION:
        parser.error('stable v1.17.0 pin must match the upstream tag revision')
    if not re.fullmatch(r'[0-9a-f]{40}', args.source_revision) or not 1024 <= args.db_port <= 65535:
        parser.error('exact source revision and unprivileged private MOOS port required')
    for key in ('output', 'build', 'moos_root', 'behavior_dir', 'px4_root'):
        setattr(args, key, getattr(args, key).resolve())
    for key in ('probe', 'nsplug'):
        if getattr(args, key):
            setattr(args, key, getattr(args, key).resolve())
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit(143)))
    raise SystemExit(main(args))
