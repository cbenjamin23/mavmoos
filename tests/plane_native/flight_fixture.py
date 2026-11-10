"""Owned native Plane fixture with independent serial1 telemetry and serial2 operator."""
import hashlib
import json
import math
import re
import select
import socket
import subprocess
import time
from pathlib import Path


HOME = (42.358456, -71.087589, 10, 0)
PIN = '3fc7011a7d3dc047cbb17d8bd98ee94577d144c6'
BINARY_SHA = 'b977ff6cd559f6a6c3d16cfb57c1e32840291210eb593e4f7254fea701137fa5'
ATTESTATION_SHA = '3ddd2117e19fe1238e1d05cb483946a5c9801aea4c467f2fbdcac5d9f6e34a75'
DEFAULTS_SHA = '93ba9a70c771609a90b81249d6a1d5a9df8d48bef7d149b42b2d9c7fbd06494a'
MOOS_PIN = '023fb54e00692611ecee2cd6f1138d10eec2127e'
HELM_SHA = '5b3d2e705609c37b5b5000384655b4491010d6d797b94c43830d9972e684a876'
OPERATOR_SYSID = 249
GUIDED, LOITER, RTL, TAKEOFF, FBWA = 15, 12, 11, 13, 5
FIXTURE = {
    'SYSID_THISMAV': 1, 'SYSID_MYGCS': 245, 'SERIAL0_PROTOCOL': 2,
    'SERIAL1_PROTOCOL': 2, 'SERIAL2_PROTOCOL': 2, 'Q_ENABLE': 0,
    'ARSPD_TYPE': 100, 'ARSPD_PRIMARY': 0, 'ARSPD_USE': 1, 'ARSPD2_TYPE': 0,
    'SIM_WIND_SPD': 0, 'SIM_WIND_TURB': 0, 'SIM_ARSPD_FAIL': 0, 'SIM_ARSPD_RND': .5,
    'FS_GCS_ENABL': 1, 'FS_LONG_ACTN': 1, 'FS_LONG_TIMEOUT': 5,
    'RTL_ALTITUDE': 65, 'RTL_AUTOLAND': 0, 'TKOFF_ALT': 65, 'TKOFF_DIST': 300,
    'LOG_DISARMED': 1, 'FLTMODE_CH': 8, 'FLTMODE3': 12, 'FLTMODE4': 5, 'MIS_RESTART': 0,
}
EXPECTED = FIXTURE | {
    'AIRSPEED_CRUISE': 22, 'AIRSPEED_MIN': 10, 'AIRSPEED_MAX': 30,
    'WP_LOITER_RAD': 80, 'RTL_RADIUS': 0, 'NAVL1_PERIOD': 15, 'ARMING_CHECK': 1,
}
INTERVALS = '0 1000\n33 100\n42 500\n87 200\n193 500\n245 500\n242 500\n295 200\n'
MAX_CAPTURE_BYTES = 8 * 1024 * 1024


def need(condition, reason):
    if not condition:
        raise RuntimeError(reason)


def now():
    # Match the bridge's libc++ steady_clock on the pinned Darwin host.
    return time.clock_gettime(time.CLOCK_MONOTONIC_RAW)


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def revision(path, clean=True):
    def git(*args):
        return subprocess.check_output(['git', '-C', str(path), *args], text=True, timeout=20).strip()

    if clean:
        need(not git('status', '--porcelain'), f'checkout has local changes: {path}')
    return git('rev-parse', 'HEAD')


def json_data(value):
    if isinstance(value, float) and not math.isfinite(value):
        return {'nonfinite': str(value)}
    if isinstance(value, bytes):
        return {'hex': value.hex()}
    if isinstance(value, (tuple, list)):
        return [json_data(item) for item in value]
    if isinstance(value, dict):
        return {key: json_data(item) for key, item in value.items()}
    return value


def log(stream, row):
    stream.write(json.dumps(json_data(row), allow_nan=False) + '\n')
    stream.flush()


def reserve(kind, host='127.0.0.1'):
    endpoint = socket.socket(socket.AF_INET, kind)
    try:
        endpoint.bind((host, 0))
        return endpoint
    except BaseException:
        endpoint.close()
        raise


def inventory(process):
    need(process.poll() is None, 'owned native process exited')
    answer = subprocess.run(['lsof', '-nP', '-a', '-p', str(process.pid), '-iUDP'],
                            capture_output=True, text=True, timeout=3)
    need(answer.returncode == 0, 'owned native UDP inventory unavailable')
    return answer.stdout


def exact_peer(proof, peer, destination):
    return (peer[0] == '127.0.0.1' and
            bool(re.search(rf'UDP\s+127\.0\.0\.1:{peer[1]}->127\.0\.0\.1:{destination}\b', proof)))


def exact_rc(proof, port):
    return bool(re.search(rf'UDP\s+(?:\*|0\.0\.0\.0|127\.0\.0\.1):{port}\b', proof))


def start(command, out, name, env=None):
    with (out / name).open('x') as stream:
        return subprocess.Popen([str(arg) for arg in command], cwd=out, env=env, stdout=stream,
                                stderr=subprocess.STDOUT, start_new_session=True)


class Channel:
    def __init__(self, name, endpoint, codec, stack, out):
        self.name = name
        self.endpoint = endpoint
        self.port = endpoint.getsockname()[1]
        self.endpoint.setblocking(False)
        self.peer = None
        self.decoder = codec.MAVLink(None)
        self.decoder.robust_parsing = True
        self.raw = stack.enter_context((out / f'{name}-rx.mavlink').open('xb'))
        self.rows = stack.enter_context((out / f'{name}-rx.jsonl').open('x'))
        self.latest = {}
        self.messages = []

    def read(self, simulator, socket_proofs):
        try:
            data, peer = self.endpoint.recvfrom(65535)
        except BlockingIOError:
            return
        at = now()
        if self.peer is None:
            proof = inventory(simulator)
            need(exact_peer(proof, peer, self.port), f'{self.name} source lacks exact owned native socket')
            self.peer = peer
            log(socket_proofs, {'mono': at, 'role': self.name, 'peer': peer, 'inventory': proof})
        need(self.peer == peer, f'{self.name} endpoint identity changed')
        offset = self.raw.tell()
        need(offset + len(data) <= MAX_CAPTURE_BYTES, 'native capture byte limit')
        self.raw.write(data)
        for msg in self.decoder.parse_buffer(data) or []:
            if (msg.get_srcSystem(), msg.get_srcComponent()) != (1, 1):
                continue
            row = {'mono': at, 'datagram_offset': offset, 'datagram_length': len(data), **msg.to_dict()}
            log(self.rows, row)
            self.latest[msg.get_type()] = row
            self.messages.append(row)


class Flight:
    def __init__(self, paths, out, stack, codec, report):
        self.report, self.out, self.codec = report, out, codec
        self.modern = paths.get('modern', False)
        self.simulator = None
        self.channels = {name: Channel(name, stack.enter_context(reserve(socket.SOCK_DGRAM)), codec, stack, out)
                         for name in ('bridge', 'monitor', 'operator')}
        self.bridge, self.monitor, self.operator = (self.channels[name] for name in ('bridge', 'monitor', 'operator'))
        self.db = stack.enter_context(reserve(socket.SOCK_STREAM))
        self.base = stack.enter_context(reserve(socket.SOCK_STREAM))
        self.rc = stack.enter_context(reserve(socket.SOCK_DGRAM, '0.0.0.0'))
        self.irlock = stack.enter_context(reserve(socket.SOCK_DGRAM))
        ports = {name: channel.port for name, channel in self.channels.items()}
        ports.update({name: getattr(self, name).getsockname()[1] for name in ('db', 'base', 'rc', 'irlock')})
        report['ports'] = self.ports = ports
        self.proofs = stack.enter_context((out / 'ownership.jsonl').open('x'))
        self.actions = stack.enter_context((out / 'fixture-operator.jsonl').open('x'))
        self.phase_events = stack.enter_context((out / 'phases.jsonl').open('x'))
        self.encoder = codec.MAVLink(None, srcSystem=OPERATOR_SYSID, srcComponent=190)
        defaults = out / 'fixture.parm'
        parameters = dict(FIXTURE)
        self.expected_parameters = dict(EXPECTED)
        if paths.get('modern'):
            del parameters['SYSID_THISMAV'], parameters['SYSID_MYGCS']
            self.expected_parameters = paths['verified_firmware']['fixture']['parameters']
            parameters.update({key: value for key, value in self.expected_parameters.items()
                               if key not in EXPECTED})
        defaults.write_text(''.join(f'{key} {value}\n' for key, value in parameters.items()))
        for channel in range(3):
            (out / f'message-intervals-chan{channel}.txt').write_text(INTERVALS)
        self.command = [paths['native'], '--model', 'plane', '--home', ','.join(map(str, HOME)),
                        '--speedup', '1', '--wipe', '--defaults', f"{paths['defaults']},{defaults}",
                        '--serial0', f"udpclient:127.0.0.1:{ports['bridge']}",
                        '--serial1', f"udpclient:127.0.0.1:{ports['monitor']}",
                        '--serial2', f"udpclient:127.0.0.1:{ports['operator']}",
                        '--serial5', 'none', '--serial6', 'none', '--serial7', 'none', '--serial8', 'none',
                        '--base-port', str(ports['base']), '--rc-in-port', str(ports['rc']),
                        '--irlock-port', str(ports['irlock'])]
        report['native_command'] = [str(arg) for arg in self.command]
        report['monitor_writes'] = 0

    def launch(self):
        self.base.close()
        self.rc.close()
        self.irlock.close()
        self.simulator = start(self.command, self.out, 'sitl.log')
        self.report['sitl_pid'] = self.simulator.pid

    def event(self, name, **values):
        at = now()
        log(self.phase_events, {'phase': name, 'mono': at, **values})
        return at

    def pump(self, duration=.04):
        need(self.simulator.poll() is None, 'owned simulator exited; inspect sitl.log')
        active = {c.endpoint: c for c in self.channels.values() if c.endpoint.fileno() != -1}
        for ready in select.select(active, (), (), duration)[0]:
            active[ready].read(self.simulator, self.proofs)

    def wait(self, timeout, check, why):
        end = now() + timeout
        while now() < end:
            self.pump(min(.04, max(0, end - now())))
            value = check()
            if value:
                return value
        raise RuntimeError(why)

    def latest(self, name, after=0, max_age=2):
        record = self.monitor.latest.get(name)
        return record if record and after <= record['mono'] and now() - record['mono'] <= max_age else None

    def heartbeat(self, mode=None, armed=None, after=0):
        row = self.latest('HEARTBEAT', after)
        return row if (row and row['type'] == 1 and row['autopilot'] == 3 and
                       (mode is None or row['custom_mode'] == mode) and
                       (armed is None or bool(row['base_mode'] & 128) == armed)) else None

    def transmit(self, message):
        need(self.operator.peer is not None, 'simulator operator peer unavailable')
        proof = inventory(self.simulator)
        need(exact_peer(proof, self.operator.peer, self.operator.port), 'simulator operator route lost ownership')
        at = now()
        payload = message.pack(self.encoder)
        self.encoder.seq = (self.encoder.seq + 1) % 256
        need(self.operator.endpoint.sendto(payload, self.operator.peer) == len(payload), 'incomplete simulator write')
        log(self.actions, {'mono': at, 'sysid': OPERATOR_SYSID, 'component': 190,
                           'peer': self.operator.peer, 'bytes': payload.hex(), **message.to_dict()})
        return at

    def mode(self, mode):
        need(mode in (LOITER, RTL, TAKEOFF, FBWA), 'simulator operator mode outside fixture policy')
        mark = self.transmit(self.encoder.command_long_encode(1, 1, 176, 0, 1, mode, 0, 0, 0, 0, 0))
        return self.wait(8, lambda: self.heartbeat(mode, after=mark), f'independent FC mode {mode} not observed')

    def arm(self):
        def prearm_ready():
            status = self.latest('SYS_STATUS')
            estimator = self.latest('EKF_STATUS_REPORT')
            return (status and estimator and estimator['flags'] & 63 == 63 and
                    status['onboard_control_sensors_health'] & self.codec.MAV_SYS_STATUS_PREARM_CHECK)

        self.wait(35, prearm_ready, 'fresh EKF and firmware pre-arm checks never became ready')
        self.event('prearm_ready', system_status=self.latest('SYS_STATUS'), estimator=self.latest('EKF_STATUS_REPORT'))
        mark = self.transmit(self.encoder.command_long_encode(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0))
        return self.wait(8, lambda: self.heartbeat(TAKEOFF, True, mark), 'ordinary simulator arming not confirmed')

    def verify_parameters(self):
        values = {}
        for name, expected in self.expected_parameters.items():
            def sample():
                row = self.operator.latest.get('PARAM_VALUE')
                return row if row and row['mono'] >= mark and row['param_id'] == name else None

            for attempt in range(3):
                mark = self.transmit(self.encoder.param_request_read_encode(1, 1, name.encode(), -1))
                try:
                    row = self.wait(1.25, sample, f'{name} readback unavailable')
                except RuntimeError:
                    if attempt == 2:
                        raise
                else:
                    break
            value = row['param_value']
            need(isinstance(value, (int, float)) and math.isclose(value, expected, rel_tol=0, abs_tol=.001),
                 f'{name} active {value!r} differs from fixture {expected}')
            values[name] = row
        self.report['active_parameters'] = values

    def verify_empty_missions(self):
        counts = {}
        for mission_type in (self.codec.MAV_MISSION_TYPE_MISSION, self.codec.MAV_MISSION_TYPE_RALLY):
            mark = self.transmit(self.encoder.mission_request_list_encode(1, 1, mission_type))

            def sample():
                row = self.operator.latest.get('MISSION_COUNT')
                return row if row and row['mono'] >= mark and row['mission_type'] == mission_type and \
                    (row['target_system'], row['target_component']) == (OPERATOR_SYSID, 190) else None

            row = self.wait(5, sample, f'mission/rally type {mission_type} readback unavailable')
            if self.modern and mission_type == self.codec.MAV_MISSION_TYPE_MISSION and row['count'] == 1:
                sent = self.transmit(self.encoder.mission_request_int_encode(1, 1, 0, mission_type))

                def home_row():
                    item = self.operator.latest.get('MISSION_ITEM_INT')
                    return item if item and item['mono'] > sent and item['seq'] == 0 and \
                        (item['target_system'], item['target_component']) == (OPERATOR_SYSID, 190) else None

                item = self.wait(5, home_row, 'native initial home row was not read back')
                home = self.latest('HOME_POSITION')
                need(home and item['mission_type'] == mission_type and
                     (item['command'], item['frame']) == (self.codec.MAV_CMD_NAV_WAYPOINT, self.codec.MAV_FRAME_GLOBAL) and
                     abs(item['x'] - home['latitude']) <= 300 and abs(item['y'] - home['longitude']) <= 300 and
                     abs(item['z'] - home['altitude'] / 1000) <= 4, 'initialized mission row is not selected HOME')
                self.report['initial_home_mission_row'] = item
            else:
                need(row['count'] == 0, 'native mission or rally plan was not empty')
            counts[str(mission_type)] = row
        self.report['empty_missions'] = counts

    def trimmed(self, mode):
        state, position, air = (self.latest(name) for name in ('EXTENDED_SYS_STATE', 'GLOBAL_POSITION_INT', 'AIRSPEED'))
        heartbeat = self.heartbeat(mode, True)
        if (heartbeat and state and state['landed_state'] == 2 and position and air and air['id'] == 0 and
                air['flags'] & 3 == 2 and 17 <= air['airspeed'] <= 28 and
                55 <= position['relative_alt'] / 1000 <= 82 and 15 <= ground_speed(position) <= 31):
            return {'heartbeat': heartbeat, 'landed': state, 'position': position, 'airspeed': air}
        return None

    def ground_prelude(self):
        self.wait(45, lambda: all(channel.peer for channel in self.channels.values()) and
                  self.heartbeat(armed=False) and self.latest('HOME_POSITION'), 'initial disarmed HOME and owned routes missing')
        self.report['native_peers'] = {name: channel.peer for name, channel in self.channels.items()}
        self.verify_parameters()
        self.verify_empty_missions()
        self.event('prelude_disarmed', heartbeat=self.heartbeat(armed=False), home=self.latest('HOME_POSITION'))

    def prelude(self):
        self.ground_prelude()
        self.mode(TAKEOFF)
        self.arm()
        lift = self.wait(45, lambda: self.trimmed(TAKEOFF), 'TAKEOFF never reached the predeclared airborne trim window')
        self.event('prelude_takeoff', evidence=lift)
        self.mode(LOITER)
        loiter = self.wait(12, lambda: self.trimmed(LOITER), 'native Loiter airborne trim not observed')
        self.event('prelude_loiter', evidence=loiter)
        self.report['prelude'] = {'takeoff': lift, 'loiter': loiter}
        return loiter


def ground_speed(row):
    return math.hypot(row['vx'], row['vy']) / 100
