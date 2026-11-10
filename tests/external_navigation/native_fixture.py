"""Owned disarmed Copter serial0 input and independent serial1 observation."""
import errno
import math
import select
import socket
from pathlib import Path

from pymavlink import DFReader
from pymavlink.dialects.v20 import all as mavlink
from owned_processes import stop_process_tree
from plane_native.flight_fixture import Channel, exact_peer, inventory, log, need, now, reserve

PROFILE = 'arducopter-4.6.3-quad'
APP = (245, 191)
FC = (1, 1)
SOURCE = 'external_navigation_fixture'
REQUEST = 'MAVMOOS_EXTERNAL_NAV_REQUEST'
INPUT = 'MAVMOOS_EXTERNAL_NAV_INPUT'
STATUS = 'MAVMOOS_EXTERNAL_NAV_STATUS'

# GPS supplies the ordinary local origin and an independent stationary witness.
# The EKF's measured horizontal position and velocity sources are ExternalNav.
PARAMETERS = {
    'SERIAL0_PROTOCOL': 2, 'SERIAL1_PROTOCOL': 2, 'LOG_DISARMED': 1,
    'VISO_TYPE': 1, 'VISO_DELAY_MS': 10, 'VISO_QUAL_MIN': 50,
    'EK3_SRC1_POSXY': 6, 'EK3_SRC1_VELXY': 6, 'EK3_SRC1_POSZ': 1,
    'EK3_SRC1_VELZ': 0, 'EK3_SRC1_YAW': 1,
}
INTERVALS = '0 1000\n24 250\n32 100\n193 200\n245 250\n'


def ground(heartbeat, landed):
    return (heartbeat and landed and not heartbeat['base_mode'] & mavlink.MAV_MODE_FLAG_SAFETY_ARMED and
            landed['landed_state'] == mavlink.MAV_LANDED_STATE_ON_GROUND and
            now() - min(heartbeat['mono'], landed['mono']) < 2)


def ports_clear(ports):
    results = {}
    for name, port in ports.items():
        tcp = name in ('db', 'base')
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM if tcp else socket.SOCK_DGRAM) as endpoint:
            if tcp:
                endpoint.settimeout(.2)
                results[name] = endpoint.connect_ex(('127.0.0.1', port)) == errno.ECONNREFUSED
                continue
            try:
                endpoint.bind(('0.0.0.0' if name == 'rc' else '127.0.0.1', port))
                results[name] = True
            except OSError:
                results[name] = False
    return results


class CopterBench:
    def __init__(self, out, paths, firmware, stack, report):
        self.out, self.paths, self.report = out, paths, report
        self.native = None
        self.bridge = Channel('bridge-prelude', stack.enter_context(reserve(socket.SOCK_DGRAM)), mavlink, stack, out)
        self.monitor = Channel('independent', stack.enter_context(reserve(socket.SOCK_DGRAM)), mavlink, stack, out)
        self.db = stack.enter_context(reserve(socket.SOCK_STREAM))
        self.base = stack.enter_context(reserve(socket.SOCK_STREAM))
        self.rc = stack.enter_context(reserve(socket.SOCK_DGRAM, '0.0.0.0'))
        self.irlock = stack.enter_context(reserve(socket.SOCK_DGRAM))
        self.ports = report['ports'] = {'bridge': self.bridge.port, 'monitor': self.monitor.port}
        self.ports.update({name: getattr(self, name).getsockname()[1] for name in ('db', 'base', 'rc', 'irlock')})
        self.proofs = stack.enter_context((out / 'ownership.jsonl').open('x'))
        params = firmware['fixture']['parameters'] | PARAMETERS
        defaults = out / 'fixture.parm'
        defaults.write_text(''.join(f'{key} {value}\n' for key, value in params.items()))
        for channel in (0, 1):
            (out / f'message-intervals-chan{channel}.txt').write_text(INTERVALS)
        report['fixture_parameters'] = params
        report['native_command'] = [str(paths['native']), '--model', 'quad',
            '--home', ','.join(map(str, firmware['fixture']['home'])), '--speedup', '1', '--wipe',
            '--defaults', f"{paths['defaults']},{defaults}",
            '--serial0', f"udpclient:127.0.0.1:{self.ports['bridge']}",
            '--serial1', f"udpclient:127.0.0.1:{self.ports['monitor']}",
            '--serial2', 'none', '--serial5', 'none', '--serial6', 'none', '--serial7', 'none', '--serial8', 'none',
            '--base-port', str(self.ports['base']), '--rc-in-port', str(self.ports['rc']),
            '--irlock-port', str(self.ports['irlock'])]
        report['monitor_tx_datagrams'] = 0

    def launch(self):
        from plane_native.flight_fixture import start
        for endpoint in (self.base, self.rc, self.irlock):
            endpoint.close()
        self.native = start(self.report['native_command'], self.out, 'sitl.log')
        self.report['sitl_pid'] = self.native.pid
        self.wait(18, lambda: self.bridge.peer and self.monitor.peer and self.is_ground(),
                  'owned Copter routes or disarmed/ground telemetry unavailable')
        self.ports['bridge_peer'] = self.bridge.peer[1]
        self.ports['monitor_peer'] = self.monitor.peer[1]
        self.verify_routes()

    def is_ground(self):
        return ground(self.monitor.latest.get('HEARTBEAT'), self.monitor.latest.get('EXTENDED_SYS_STATE'))

    def verify_routes(self):
        proof = inventory(self.native)
        need(exact_peer(proof, self.bridge.peer, self.bridge.port) and exact_peer(proof, self.monitor.peer, self.monitor.port),
             'exact native routes required')
        log(self.proofs, {'mono': now(), 'role': 'both', 'inventory': proof})

    def pump(self, duration=.02, extra=None):
        need(self.native.poll() is None, 'owned Copter exited')
        active = {self.monitor.endpoint: lambda: self.monitor.read(self.native, self.proofs)}
        if self.bridge.endpoint.fileno() != -1:
            active[self.bridge.endpoint] = lambda: self.bridge.read(self.native, self.proofs)
        if extra:
            active.update(extra)
        for ready in select.select(active, (), (), duration)[0]:
            active[ready]()
        heartbeat = self.monitor.latest.get('HEARTBEAT')
        if heartbeat:
            need(not heartbeat['base_mode'] & mavlink.MAV_MODE_FLAG_SAFETY_ARMED, 'native fixture armed unexpectedly')

    def wait(self, timeout, condition, reason, pump=None):
        deadline = now() + timeout
        while now() < deadline:
            (pump or self.pump)(min(.02, max(0, deadline-now())))
            value = condition()
            if value:
                return value
        raise RuntimeError(reason)

    def close(self):
        self.report['owned_processes_remaining'] = stop_process_tree(self.native)


def pose_trace(monitor, start):
    return [row for row in monitor.messages if row['mavpackettype'] == 'LOCAL_POSITION_NED' and row['mono'] >= start]


def ekf_trace(monitor, start):
    return [row for row in monitor.messages if row['mavpackettype'] == 'EKF_STATUS_REPORT' and row['mono'] >= start]


def aiding(row):
    return row['flags'] & (mavlink.EKF_POS_HORIZ_REL | mavlink.EKF_POS_HORIZ_ABS)


def last_gps(monitor):
    fix = monitor.latest.get('GPS_RAW_INT', {})
    if fix.get('fix_type', 0) >= 3:
        return fix


def gps_distance(a, b):
    return math.hypot((a['lat']-b['lat'])*.011132,
                      (a['lon']-b['lon'])*.011132*math.cos(math.radians(a['lat']/1e7)))


def decode_native_log(out):
    log_files = sorted((Path(out) / 'logs').glob('*.BIN'))
    need(bool(log_files), 'native DataFlash evidence absent')
    vision = []
    parameter_values = {}
    for path in log_files:
        reader = DFReader.DFReader_binary(str(path))
        while True:
            msg = reader.recv_match(type=['PARM', 'VISP', 'VISV'])
            if msg is None:
                break
            if msg.get_type() == 'PARM':
                parameter_values[msg.Name] = msg.Value
            else:
                vision.append(msg.to_dict())
    return log_files, parameter_values, vision


def corroborate_log(out, sent, report):
    log_files, parameter_values, vision = decode_native_log(out)
    required_params = {key: value for key, value in PARAMETERS.items() if key.startswith(('VISO_', 'EK3_'))}
    need(all(parameter_values.get(key) == value for key, value in required_params.items()),
         'native ExternalNav parameter readback differs')
    expected = {row['capture_us']: row for row in sent}
    pose = [row for row in vision if row['mavpackettype'] == 'VISP']
    vel = [row for row in vision if row['mavpackettype'] == 'VISV']
    # DataFlash records the message's original timestamp independently of MOOS.
    for group, coords in ((pose, ('PX', 'PY', 'PZ')), (vel, ('VX', 'VY', 'VZ'))):
        need(len(group)>=35 and len(group)==len({row['RTimeUS'] for row in group}) and
             all(row['RTimeUS'] in expected and row['Ign']==0 and row['Q']==90 and row['Rst']==0 for row in group),
             'native ExternalNav log did not uniquely accept only fresh fixture samples')
        values = 'position' if group is pose else 'velocity'
        need(all(abs(row[key]-expected[row['RTimeUS']][values][i])<.001
                 for row in group for i, key in enumerate(coords)),
             'native ExternalNav vectors differ from original producer data')
    report['native_log'] = {
        'paths': [str(path.relative_to(out)) for path in log_files],
        'pose_matches': len(pose), 'velocity_matches': len(vel), 'source_parameters': required_params,
        'first_capture_us': pose[0]['RTimeUS'], 'last_capture_us': pose[-1]['RTimeUS'],
    }
    with (Path(out) / 'native-external-nav.jsonl').open('x') as stream:
        for row in vision:
            log(stream, row)
