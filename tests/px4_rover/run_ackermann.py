#!/usr/bin/env python3
"""An offline protocol witness for the pinned PX4 v1.17.0 SIH Ackermann."""
import argparse
from contextlib import ExitStack
import importlib.util
import json
import math
import os
from pathlib import Path
import platform
import re
import select
import signal
import socket
import struct
import subprocess
import sys
import time
from uuid import uuid4

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path[:0] = [str(ROOT / 'scripts'), str(HERE.parent)]
from ci.isolation import LinuxDomainLease
from ci.run import CIInterrupted, accepted, bounded, dump, parse, sha
from owned_processes import stop_process_tree
from px4_offboard.run_px4_offboard import HOLD, OFFBOARD, PORTS, namespace, udp_sockets
from release.local_docker import CONTAINER_ID, OWNER_LABEL, LocalDocker, owned_container, remove_owned_container

PX4_REVISION = 'd6f12ad1c4f70ad3230afd7d86e971421e02fef4'
TOOLS_IMAGE = 'sha256:644154814d3e5c162392b229f26bd68e3ad55c44acf863e78ee83e7568cbed85'
PACKAGE = Path('/opt/px4')
NATIVE = PACKAGE / 'bin/px4'
DIALECT = Path('/codec.py')
MISSION = (4 << 16) | (4 << 24)
TARGET = (1, 1)
SENDER = (245, 191)
LOOPBACK = '127.0.0.1'
EARTH_RADIUS = 6371000
EXTRA_PINS = {
    'etc/init.d-posix/airframes/10045_sihsim_rover_ackermann':
        '1bf2823c3d13fafaa026a5d6fef18667469d2c27277e5420af669d5ad418cd62',
    'etc/init.d/rc.rover_ackermann_defaults':
        'eabb275785f46ed48118f92883735e4472132ed39b8f104f36080bd3b93c5ee2',
    'etc/init.d/rc.rover_ackermann_apps':
        'f77559bfae6a59cdbc4e0c5cee364b5830a85b391a0734d2079ba4aac94b3360',
    'etc/init.d/rcS': 'dcb98ee77f8c459a3046c16343dbaad77f4992ad11c48c5422d9532bcaed74f4',
}
SOURCE_FILES = [
    'tests/px4_rover/run_ackermann.py', 'tests/px4_offboard/firmware-v1.17.0.json',
    'tests/px4_offboard/run_px4_offboard.py', 'tests/owned_processes.py',
    'scripts/release/local_docker.py', 'scripts/ci/isolation.py', 'scripts/ci/run.py',
]


def need(condition, reason):
    if not condition:
        raise RuntimeError(reason)


def ports_free():
    free = {}
    for port in (*PORTS, 14280, 13030, 19450):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as endpoint:
            try:
                endpoint.bind(('0.0.0.0', port))
                free[str(port)] = True
            except OSError:
                free[str(port)] = False
    return free


class Receiver:
    def __init__(self, stack, mav, local_port, remote_port, trace):
        self.mav = mav
        self.trace = trace
        self.sock = stack.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
        self.sock.bind((LOOPBACK, local_port))
        self.sock.connect((LOOPBACK, remote_port))
        self.sock.setblocking(False)
        self.parser = mav.MAVLink(None)
        self.parser.robust_parsing = True

    def read(self):
        try:
            payload = self.sock.recv(65535)
        except BlockingIOError:
            return []
        self.trace.write(payload)
        frames = []
        for frame in self.parser.parse_buffer(payload) or []:
            if frame.get_type() == 'UNKNOWN_436':
                # The pinned common.py predates CURRENT_MODE. Verify message 436
                # against the pinned generated C header's CRC extra and layout.
                raw = bytes(frame.get_msgbuf())
                size = raw[1]
                if (raw[0] != 253 or raw[2] != 0 or tuple(raw[5:7]) != TARGET or
                        not 1 <= size <= 9 or len(raw) != size + 12 or
                        self.mav.x25crc(raw[1:-2] + bytes((193,))).crc != int.from_bytes(raw[-2:], 'little')):
                    continue
                frames.append(CurrentMode(*struct.unpack('<IIB', raw[10:-2].ljust(9, b'\0'))))
            elif (frame.get_srcSystem(), frame.get_srcComponent()) == TARGET:
                frames.append(frame)
        return frames


class CurrentMode:
    def __init__(self, custom_mode, intended_custom_mode, standard_mode):
        self.custom_mode = custom_mode
        self.intended_custom_mode = intended_custom_mode
        self.standard_mode = standard_mode

    def get_type(self):
        return 'CURRENT_MODE'

    def to_dict(self):
        return {'mavpackettype': self.get_type(), **vars(self)}


class Witness:
    def __init__(self, stack, mav, out, report):
        self.mav, self.out, self.report = mav, out, report
        self.command = Receiver(stack, mav, 14540, 14580, stack.enter_context((out / 'onboard.mavlink').open('xb')))
        self.observer = Receiver(stack, mav, 14550, 18570, stack.enter_context((out / 'gcs.mavlink').open('xb')))
        self.sent = stack.enter_context((out / 'sent.mavlink').open('xb'))
        self.trace = stack.enter_context((out / 'events.jsonl').open('x'))
        self.encoder = mav.MAVLink(self, srcSystem=SENDER[0], srcComponent=SENDER[1])
        self.last_heartbeat = 0
        self.latest = {}
        self.child = None
        self.deadline = time.monotonic() + 125
        self.env = {'PATH': '/opt/px4/bin:/usr/local/bin:/usr/bin:/bin', 'LC_ALL': 'C',
                    'PX4_SIM_MODEL': 'sihsim_rover_ackermann', 'PX4_SIMULATOR': 'sihsim'}

    def event(self, kind, **data):
        stamp = time.monotonic()
        self.trace.write(json.dumps({'event': kind, 'mono': stamp, **data}, default=list) + '\n')
        self.trace.flush()
        return stamp

    def owned(self):
        return self.child is not None and self.child.poll() is None and \
            Path(f'/proc/{self.child.pid}/exe').resolve() == NATIVE

    def write(self, payload):
        need(self.owned() and 14580 in {row['local_port'] for row in udp_sockets(self.child.pid)},
             'command lacks its exact owned simulator socket')
        need(self.command.sock.send(payload) == len(payload), 'partial command datagram')
        self.sent.write(payload)

    def send(self, name, *args):
        getattr(self.encoder, name + '_send')(*args)
        return self.event('tx', name=name, args=args)

    def receive(self):
        need(self.owned(), 'owned PX4 exited')
        now = time.monotonic()
        if self.last_heartbeat + 1 < now:
            self.encoder.heartbeat_send(self.mav.MAV_TYPE_GCS, self.mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.last_heartbeat = now
        select.select([self.command.sock, self.observer.sock], [], [], .025)
        for side, receiver in (('onboard', self.command), ('gcs', self.observer)):
            for frame in receiver.read():
                key = frame.get_type()
                stamp = time.monotonic()
                self.latest[side, key] = (stamp, frame)
                if key in {'HEARTBEAT', 'COMMAND_ACK', 'MISSION_COUNT', 'MISSION_REQUEST_INT',
                           'MISSION_ITEM_INT', 'MISSION_ACK', 'MISSION_CURRENT', 'MISSION_ITEM_REACHED',
                           'STATUSTEXT', 'SYS_STATUS', 'GLOBAL_POSITION_INT', 'GPS_RAW_INT', 'CURRENT_MODE'}:
                    self.event('rx', route=side, frame=frame.to_dict())

    def wait(self, condition, why, seconds=8, tick=None):
        until = min(self.deadline, time.monotonic() + seconds)
        while time.monotonic() < until:
            if tick is not None:
                tick()
            self.receive()
            result = condition()
            if result is not None and result is not False:
                return result
        raise RuntimeError(why)

    def frame(self, side, name, after=0, check=lambda _: True):
        at, frame = self.latest.get((side, name), (0, None))
        return frame if frame is not None and at > after and check(frame) else None

    def observe(self, name, after=0, check=lambda _: True, seconds=8, tick=None):
        return self.wait(lambda: self.frame('gcs', name, after, check), f'no independent {name} evidence',
                         seconds, tick)

    def reply(self, name, after, check, tick=None):
        return self.wait(lambda: self.frame('onboard', name, after, lambda row:
                         (row.target_system, row.target_component) == SENDER and check(row)),
                         f'no addressed {name} transaction reply', tick=tick)

    def cli(self, command, *args):
        need(self.owned(), 'CLI lacks its exact owned simulator process')
        result = subprocess.run([str(PACKAGE / 'bin' / ('px4-' + command)), *args], cwd=self.out,
                                env=self.env, capture_output=True, text=True, timeout=5)
        self.event('cli', command=command, args=args, code=result.returncode,
                   stdout=result.stdout, stderr=result.stderr)
        need(result.returncode == 0, f'owned PX4 CLI failed: {command}')
        return result.stdout + result.stderr

    def boot(self):
        with (self.out / 'startup.log').open('xb') as log:
            self.child = subprocess.Popen([str(NATIVE), '-d', str(PACKAGE / 'etc'), '-w', str(self.out),
                                           '-s', str(PACKAGE / 'etc/init.d-posix/rcS')],
                                          cwd=self.out, env=self.env, stdin=subprocess.DEVNULL,
                                          stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        until = time.monotonic() + 20
        while time.monotonic() < until:
            need(self.child.poll() is None, 'owned PX4 exited during startup')
            if self.owned() and {14580, 18570}.issubset(
                    row['local_port'] for row in udp_sockets(self.child.pid)):
                break
            time.sleep(.05)
        sockets = udp_sockets(self.child.pid)
        need({14580, 18570}.issubset(row['local_port'] for row in sockets), 'missing owned PX4 sockets')
        version = self.cli('ver', 'all')
        need(re.search(r'PX4 git-hash:\s*' + PX4_REVISION + r'\b', version) and
             re.search(r'PX4 version:\s*Release 1\.17\.0\b', version), 'owned firmware revision differs')
        simulator = self.cli('simulator_sih', 'status')
        need('ackermann' in simulator.lower() and 'running' in self.cli('rover_ackermann', 'status').lower(),
             'owned SIH/controller identity differs')
        self.report.update(simulator=simulator, px4_pid=self.child.pid, px4_udp_sockets=sockets,
                           version=version, forward_externalsp=self.cli('param', 'show', 'MAV_FWDEXTSP'))
        self.observe('HEARTBEAT', check=lambda row:
                     row.type == self.mav.MAV_TYPE_GROUND_ROVER and row.autopilot == self.mav.MAV_AUTOPILOT_PX4
                     and not self.armed(row))

    def armed(self, frame):
        return bool(frame.base_mode & self.mav.MAV_MODE_FLAG_SAFETY_ARMED)

    def command_long(self, command, *params, tick=None):
        need(len(params) <= 7, 'invalid command parameter count')
        at = self.send('command_long', *TARGET, command, 0, *params, *([0] * (7 - len(params))))
        ack = self.reply('COMMAND_ACK', at, lambda row: row.command == command, tick)
        need(ack.result == self.mav.MAV_RESULT_ACCEPTED, f'command {command} rejected with {ack.result}')
        return at, ack.to_dict()

    def mode(self, mode, tick=None):
        return self.command_long(self.mav.MAV_CMD_DO_SET_MODE, self.mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                 (mode >> 16) & 255, (mode >> 24) & 255, tick=tick)

    def require_mode(self, mode, after, armed=True, tick=None):
        return self.observe('HEARTBEAT', after,
                            lambda row: row.custom_mode == mode and self.armed(row) == armed, tick=tick)

    def ground_truth(self):
        return parse_ground_truth(self.cli('listener', 'vehicle_local_position_groundtruth', '-n', '1'))

    def stopped(self, after):
        # This is a simulator truth check. It cannot certify a field stop
        # when the estimator reports residual velocity while physically still.
        started = time.monotonic()
        next_query = 0
        previous = 0
        dwell = []
        while time.monotonic() < min(self.deadline, started + 12):
            self.receive()
            now = time.monotonic()
            if now < next_query:
                continue
            next_query = now + .2
            heartbeat = self.frame('gcs', 'HEARTBEAT', max(after, now - 2.5),
                                   lambda row: row.custom_mode == HOLD and self.armed(row))
            estimated = self.frame('gcs', 'GLOBAL_POSITION_INT', max(after, now - .5))
            if heartbeat is None or estimated is None:
                dwell.clear()
                continue
            truth = self.ground_truth()
            if truth['timestamp'] <= previous:
                dwell.clear()
                continue
            previous = truth['timestamp']
            speed = math.hypot(truth['vx'], truth['vy'])
            if speed > .25:
                dwell.clear()
                continue
            if dwell and math.hypot(truth['x'] - dwell[0]['x'], truth['y'] - dwell[0]['y']) > 1:
                dwell.clear()
            dwell.append({**truth, 'speed_m_s': speed, 'estimated_m_s': math.hypot(estimated.vx, estimated.vy) / 100})
            elapsed = (truth['timestamp'] - dwell[0]['timestamp']) / 1e6
            if elapsed >= 1.5 and len(dwell) >= 5:
                self.report.setdefault('stopped_dwells', []).append({
                    'mode': 'Hold', 'source': 'owned_sih_vehicle_local_position_groundtruth',
                    'elapsed_firmware_s': elapsed, 'elapsed_wall_s': time.monotonic() - started, 'samples': dwell,
                    'max_truth_speed_m_s': max(row['speed_m_s'] for row in dwell),
                    'max_estimated_speed_m_s': max(row['estimated_m_s'] for row in dwell),
                })
                return estimated
        raise RuntimeError('independent SIH stopped dwell failed')

    def mission(self):
        mav = self.mav
        home = self.observe('GLOBAL_POSITION_INT',
                            check=lambda row: row.lat and row.lon and 0 <= row.hdg < 36000, seconds=20)
        bearing = math.radians(home.hdg / 100)
        plan = [waypoint(home, bearing, distance) for distance in (15, 32)]
        mission_type = mav.MAV_MISSION_TYPE_MISSION
        frame_type = mav.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        command = mav.MAV_CMD_NAV_WAYPOINT
        at = self.send('mission_request_list', *TARGET, mission_type)
        before = self.reply('MISSION_COUNT', at, lambda row: row.mission_type == mission_type)
        need(before.count == 0, 'private simulated mission storage was not empty')
        # PX4 enters SENDLIST even for zero items. Close that transfer first.
        self.send('mission_ack', *TARGET, mav.MAV_MISSION_ACCEPTED, mission_type)
        at = self.send('mission_count', *TARGET, len(plan), mission_type)
        for seq, (lat, lon) in enumerate(plan):
            self.reply('MISSION_REQUEST_INT', at, lambda row: row.seq == seq and row.mission_type == mission_type)
            at = self.send('mission_item_int', *TARGET, seq, frame_type, command,
                           int(seq == 0), 1, 0, 0, 0, 0, lat, lon, 0, mission_type)
        upload_ack = self.reply('MISSION_ACK', at, lambda row: row.mission_type == mission_type)
        need(upload_ack.type == mav.MAV_MISSION_ACCEPTED, 'mission upload rejected')
        self.report['upload_ack'] = upload_ack.to_dict()

        at = self.send('mission_request_list', *TARGET, mission_type)
        count = self.reply('MISSION_COUNT', at, lambda row: row.mission_type == mission_type)
        need(count.count == len(plan), 'mission download count differs')
        readback = []
        for seq, coordinates in enumerate(plan):
            at = self.send('mission_request_int', *TARGET, seq, mission_type)
            row = self.reply('MISSION_ITEM_INT', at, lambda item: item.seq == seq and item.mission_type == mission_type)
            # PX4 may normalize GLOBAL_RELATIVE_ALT_INT to its equivalent MAV_FRAME.
            need(row.frame in {frame_type, mav.MAV_FRAME_GLOBAL_RELATIVE_ALT} and row.command == command and
                 row.autocontinue == 1 and (row.x, row.y, row.z) == (*coordinates, 0) and
                 all(getattr(row, f'param{index}') == 0 for index in (1, 2, 3, 4)), 'mission item readback differs')
            readback.append(row.to_dict())
        at = self.send('mission_ack', *TARGET, mav.MAV_MISSION_ACCEPTED, mission_type)
        self.observe('HEARTBEAT', at, lambda row: not self.armed(row))
        self.report['verified_readback'] = readback

        prearm = mav.MAV_SYS_STATUS_PREARM_CHECK
        self.observe('SYS_STATUS', at, lambda row:
                     row.onboard_control_sensors_health & prearm, seconds=20)

        at, ack = self.command_long(mav.MAV_CMD_MISSION_START, 0, len(plan) - 1)
        self.report['start_ack'] = ack
        self.require_mode(MISSION, at)
        moved = self.observe('GLOBAL_POSITION_INT', at, lambda row: separation(home, row) >= 4, seconds=18)
        at, _ = self.mode(HOLD)
        self.require_mode(HOLD, at)
        held = self.stopped(at)
        at, _ = self.mode(MISSION)
        self.require_mode(MISSION, at)
        lat, lon = plan[-1]
        goal = self.observe('GLOBAL_POSITION_INT', at, lambda row: offset_m(lat, lon, row.lat, row.lon) <= 2, seconds=30)
        reached = self.observe('MISSION_ITEM_REACHED', at, lambda row: row.seq == len(plan) - 1, seconds=8)
        at, _ = self.mode(HOLD)
        self.require_mode(HOLD, at)
        self.stopped(at)
        self.report['mission_execution'] = {
            'motion_before_hold_m': separation(home, moved), 'hold_from_origin_m': separation(home, held),
            'motion_after_resume_m': separation(held, goal), 'goal_error_m': offset_m(lat, lon, goal.lat, goal.lon),
            'last_item_reached': reached.seq,
        }

    def arm_for_offboard(self):
        mav = self.mav
        self.observe('SYS_STATUS', check=lambda row:
                     row.onboard_control_sensors_health & mav.MAV_SYS_STATUS_PREARM_CHECK, seconds=20)
        at, _ = self.mode(HOLD)
        self.require_mode(HOLD, at, armed=False)
        at, ack = self.command_long(mav.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        self.require_mode(HOLD, at)
        self.stopped(at)
        self.report['offboard_arm_ack'] = ack

    def gps_truth_sample(self, after, samples):
        at, gps = self.latest.get(('gcs', 'GPS_RAW_INT'), (0, None))
        now = time.monotonic()
        if gps is None or at <= after or now - at > .5 or gps.fix_type < 3 or gps.vel == 65535:
            return
        if samples and (gps.time_usec <= samples[-1]['gps']['time_usec'] or
                        now - samples[-1]['paired_mono'] < .15):
            return
        truth = self.ground_truth()
        estimate = self.frame('gcs', 'GLOBAL_POSITION_INT', now - .5)
        samples.append({
            'gps': gps.to_dict(), 'gps_mono': at, 'paired_mono': time.monotonic(),
            'truth': truth, 'truth_speed_m_s': math.hypot(truth['vx'], truth['vy']),
            'estimator_speed_m_s': math.hypot(estimate.vx, estimate.vy) / 100 if estimate else None,
        })

    def characterize_hold(self, after, phase):
        samples = []

        def dwell():
            self.gps_truth_sample(after, samples)
            if len(samples) >= 8 and samples[-1]['gps']['time_usec'] - samples[0]['gps']['time_usec'] >= 2_000_000:
                return samples
            return None

        self.wait(dwell, 'advancing raw GNSS samples absent in ' + phase, seconds=12)
        need(max(row['truth_speed_m_s'] for row in samples) <= .25,
             'raw GNSS parked witness did not remain physically still')
        self.report.setdefault('raw_gps_characterization', {})[phase] = samples

    def configure_stop_fallback(self):
        params = {'COM_OBL_RC_ACT': 5, 'COM_OF_LOSS_T': .7, 'MAV_FWDEXTSP': 1}
        for name, expected in params.items():
            self.cli('param', 'set', name, str(expected))
            result = self.cli('param', 'show', name)
            value = re.search(r'\b' + name + r'\b.*?:\s*([-+\d.eE]+)', result)
            need(value and math.isclose(float(value[1]), expected, abs_tol=.001),
                 'owned PX4 fallback parameter differs: ' + name)
        for message, rate in (('GPS_RAW_INT', 10), ('CURRENT_MODE', 5)):
            self.cli('mavlink', 'stream', '-u', '18570', '-s', message, '-r', str(rate))
        self.report['stop_fallback_parameters'] = params

    def probe_offboard(self, characterize=False):
        mav = self.mav
        start = self.observe('GLOBAL_POSITION_INT', check=lambda row: 0 <= row.hdg < 36000)
        bearing = math.radians(start.hdg / 100)
        mask = (mav.POSITION_TARGET_TYPEMASK_X_IGNORE | mav.POSITION_TARGET_TYPEMASK_Y_IGNORE |
                mav.POSITION_TARGET_TYPEMASK_Z_IGNORE | mav.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                mav.POSITION_TARGET_TYPEMASK_AX_IGNORE | mav.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                mav.POSITION_TARGET_TYPEMASK_AZ_IGNORE | mav.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                mav.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
        began = time.monotonic()
        last = 0
        count = 0
        drive_at = 0
        moving_samples = []

        def stream():
            nonlocal last, count
            now = time.monotonic()
            if now - last >= .10:
                speed = float(not characterize or drive_at > 0)
                self.send('set_position_target_local_ned', int((now - began) * 1000), *TARGET,
                          mav.MAV_FRAME_LOCAL_NED, mask, 0, 0, 0, speed * math.cos(bearing), speed * math.sin(bearing),
                          0, 0, 0, 0, 0, 0)
                last, count = now, count + 1
            if characterize and drive_at > 0:
                self.gps_truth_sample(drive_at, moving_samples)

        self.observe('GLOBAL_POSITION_INT', began, lambda row:
                     row.time_boot_ms - start.time_boot_ms >= 1250 and count >= 10, tick=stream)
        at, ack = self.mode(OFFBOARD, tick=stream)
        self.require_mode(OFFBOARD, at, tick=stream)
        drive_at = time.monotonic()
        moved = self.observe('GLOBAL_POSITION_INT', at, lambda row:
                             progress(start, row, bearing) >= 2, tick=stream)
        ceased = self.event('offboard_stream_ceased', count=count)
        lost = self.observe('HEARTBEAT', ceased, lambda row: row.custom_mode != OFFBOARD)
        self.report['offboard_probe'] = {
            'result': 'PASS', 'mode_ack': ack, 'motion_m': separation(start, moved),
            'forward_motion_m': progress(start, moved, bearing), 'setpoints': count,
            'loss_mode': lost.custom_mode,
        }
        if characterize:
            need(lost.custom_mode == HOLD and len(moving_samples) >= 3 and
                 any(row['truth_speed_m_s'] >= .5 for row in moving_samples),
                 'expected native Hold fallback and advancing motion witness')
            self.report.setdefault('raw_gps_characterization', {})['moving_offboard'] = moving_samples
            mode = self.observe('CURRENT_MODE', ceased, lambda row: row.custom_mode == HOLD)
            self.report['post_withdrawal_mode'] = mode.to_dict()
            self.stopped(ceased)
            self.characterize_hold(ceased, 'withdrawn_hold')

    def disarm(self):
        at, _ = self.mode(HOLD)
        self.require_mode(HOLD, at)
        self.stopped(at)
        at, ack = self.command_long(self.mav.MAV_CMD_COMPONENT_ARM_DISARM, 0)
        self.observe('HEARTBEAT', at, lambda row: not self.armed(row))
        self.report['disarm_ack'] = ack


def parse_ground_truth(text):
    need(all(re.search(r'^\s*' + flag + r':\s*(?:true|1)\b', text, re.M | re.I)
             for flag in ('xy_valid', 'v_xy_valid')), 'SIH ground truth is invalid')
    fields = {}
    for name in ('timestamp', 'x', 'y', 'vx', 'vy'):
        value = re.search(r'^\s*' + name + r':\s*([-+0-9.eE]+)', text, re.M)
        need(value is not None, f'SIH ground truth lacks {name}')
        fields[name] = int(value[1]) if name == 'timestamp' else float(value[1])
    need(fields['timestamp'] > 0 and all(math.isfinite(value) for value in fields.values()),
         'nonfinite SIH ground truth')
    return fields


def waypoint(home, bearing, distance):
    angular = math.degrees(distance / EARTH_RADIUS) * 1e7
    return (round(home.lat + angular * math.cos(bearing)),
            round(home.lon + angular * math.sin(bearing) / math.cos(math.radians(home.lat / 1e7))))


def offset_m(lat_a, lon_a, lat_b, lon_b):
    scale = EARTH_RADIUS * math.pi / (180 * 1e7)
    return scale * math.hypot(lat_b - lat_a, (lon_b - lon_a) * math.cos(math.radians(lat_a / 1e7)))


def separation(a, b):
    return offset_m(a.lat, a.lon, b.lat, b.lon)


def progress(a, b, bearing):
    scale = EARTH_RADIUS * math.pi / (180 * 1e7)
    north = (b.lat - a.lat) * scale
    east = (b.lon - a.lon) * scale * math.cos(math.radians(a.lat / 1e7))
    return north * math.cos(bearing) + east * math.sin(bearing)


def source_hashes():
    return {name: sha(ROOT / name) for name in SOURCE_FILES}


def worker(output, case):
    output.mkdir(mode=0o700)
    report = {'result': 'FAIL', 'firmware_revision': PX4_REVISION, 'model': 'sihsim_rover_ackermann', 'case': case,
              'scope': 'owned_loopback_protocol_witness', 'application_under_test': 'PX4'}
    witness = None
    try:
        report['namespace'] = namespace()
        need(os.statvfs(PACKAGE).f_flag & os.ST_RDONLY and os.statvfs(ROOT).f_flag & os.ST_RDONLY,
             'source and firmware mounts must be read-only')
        proof = parse((output.parent / 'inputs.json').read_text())
        need(source_hashes() == proof['source_files'], 'mounted runner sources differ')
        need(os.statvfs(DIALECT).f_flag & os.ST_RDONLY and sha(DIALECT) == proof['dialect_sha256'],
             'MAVLink codec mount or digest differs')
        spec = importlib.util.spec_from_file_location('pinned_mavlink_common_v20', DIALECT)
        mav = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mav)
        report['dialect_sha256'] = proof['dialect_sha256']
        pin = parse((HERE.parent / 'px4_offboard/firmware-v1.17.0.json').read_text())
        need(pin['revision'] == PX4_REVISION, 'stable revision pin differs')
        pins = {**pin['files'], **EXTRA_PINS}
        need(all(sha(PACKAGE / name) == expected for name, expected in pins.items()), 'firmware digest mismatch')
        report['firmware_files'] = pins
        need(all(ports_free().values()), 'private ports are in use before launch')
        with ExitStack() as stack:
            witness = Witness(stack, mav, output, report)
            witness.boot()
            if case in ('offboard', 'stop-fallback'):
                if case == 'stop-fallback':
                    witness.configure_stop_fallback()
                witness.arm_for_offboard()
            else:
                witness.mission()
            if case == 'stop-fallback':
                witness.characterize_hold(0, 'initial_hold')
                witness.probe_offboard(characterize=True)
            if case in ('offboard', 'both'):
                try:
                    witness.probe_offboard()
                except RuntimeError as error:
                    report['offboard_probe'] = {'result': 'FAIL', 'error': str(error)}
            witness.disarm()
        need(source_hashes() == proof['source_files'] and sha(DIALECT) == proof['dialect_sha256'] and
             all(sha(PACKAGE / name) == expected for name, expected in pins.items()), 'protected inputs changed')
        if case != 'offboard' or report['offboard_probe']['result'] == 'PASS':
            report['result'] = 'PASS'
    except (AttributeError, OSError, LookupError, RuntimeError, TypeError, ValueError,
            subprocess.SubprocessError) as error:
        report['error'] = f'{type(error).__name__}: {error}'
    finally:
        try:
            remaining = stop_process_tree(witness.child) if witness else []
            report['cleanup'] = {'remaining_owned': remaining, 'ports_free': ports_free()}
            if remaining or not all(report['cleanup']['ports_free'].values()):
                report.update(result='FAIL', cleanup_error='owned process or UDP port remains')
        finally:
            dump(output / 'result.json', report)
    return report['result'] != 'PASS'


def verify_container(info, volume, dialect, out):
    host = info['HostConfig']
    need(info['Image'] == TOOLS_IMAGE and info['Config']['Labels'].get('mavmoos.scope') == 'px4-rover-witness'
         and host['NetworkMode'] == 'none' and host['ReadonlyRootfs'] is True and host['Privileged'] is False
         and host['CapDrop'] == ['ALL'] and 'no-new-privileges' in host['SecurityOpt']
         and host['PidMode'] in ('', 'private') and host['IpcMode'] == host['CgroupnsMode'] == 'private'
         and host['UTSMode'] in ('', 'private') and all(not host.get(key) for key in
             ('CapAdd', 'Devices', 'DeviceRequests', 'PortBindings', 'VolumesFrom')),
         'container isolation policy differs')
    mounts = {}
    for row in info['Mounts']:
        source = row.get('Name') or row['Source']
        if row['Type'] == 'bind' and platform.system() == 'Darwin' and source.startswith('/host_mnt/'):
            source = source.removeprefix('/host_mnt')
        mounts[row['Destination']] = (row['Type'], source, row['RW'])
    need(len(info['Mounts']) == 4 and mounts == {
        '/opt/px4': ('volume', volume, False), '/source': ('bind', str(ROOT), False),
        str(DIALECT): ('bind', str(dialect), False),
        '/evidence': ('bind', str(out), True)}, 'unexpected source, firmware or evidence mount')


def run(output, volume, dialect, case):
    out = output.resolve()
    dialect = dialect.resolve(strict=True)
    need(re.fullmatch(r'[a-zA-Z0-9][a-zA-Z0-9_.-]{1,127}', volume) and ',' not in str(ROOT)
         and ',' not in str(out) and ',' not in str(dialect) and dialect.is_file(),
         'invalid local Docker mount argument')
    out.mkdir(mode=0o700, parents=True)
    fixture = subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True, timeout=10).strip()
    sources = source_hashes()
    codec_hash = sha(dialect)
    report = {'result': 'FAIL', 'firmware_revision': PX4_REVISION, 'fixture_head': fixture, 'source_files': sources,
              'case': case,
              'dialect_sha256': codec_hash}
    dump(out / 'inputs.json', report)
    name = 'mavmoos-px4-rover-' + uuid4().hex
    lease = LinuxDomainLease(out / 'domain-lease.json', fixture)
    container = endpoint = None
    try:
        endpoint = LocalDocker()
        docker = endpoint.capture
        report['docker_endpoint'] = endpoint.evidence()
        image, = parse(docker('image', 'inspect', TOOLS_IMAGE))
        need((image['Id'], image['Os'], image['Architecture']) == (TOOLS_IMAGE, 'linux', 'arm64'),
             'pinned offline Linux ARM64 tools image differs')
        local, = parse(docker('volume', 'inspect', volume))
        need(local['Name'] == volume and local['Driver'] == 'local' and not local.get('Options'),
             'firmware requires a local Docker volume')
        command = ['create', '--pull=never', '--name', name, '--label=mavmoos.scope=px4-rover-witness',
                   '--label=' + OWNER_LABEL + '=' + name, '--network=none', '--ipc=private', '--cgroupns=private',
                   '--read-only', '--cap-drop=ALL', '--security-opt=no-new-privileges', '--pids-limit=128',
                   '--cpus=2', '--memory=768m', '--tmpfs=/tmp:rw,nosuid,nodev,size=64m',
                   '--mount', f'type=volume,src={volume},dst=/opt/px4,readonly,volume-nocopy',
                   '--mount', f'type=bind,src={ROOT},dst=/source,readonly',
                   '--mount', f'type=bind,src={dialect},dst={DIALECT},readonly',
                   '--mount', f'type=bind,src={out},dst=/evidence',
                   '--entrypoint=/usr/bin/python3', TOOLS_IMAGE, '-B',
                   '/source/tests/px4_rover/run_ackermann.py', '--worker', '/evidence/native', '--case', case]
        dump(out / 'container-command.json', endpoint.argv(*command))
        lease.acquire()
        container = docker(*command)
        need(CONTAINER_ID.fullmatch(container), 'missing exact owned container ID')
        info, = parse(docker('container', 'inspect', container))
        owned_container(info, name, name, container)
        verify_container(info, volume, dialect, out)
        dump(out / 'container-before.json', info)
        execution = bounded(endpoint.argv('start', '-a', container), ROOT, out / 'launcher.log', 145,
                            env=endpoint.env, inventory_timeout=10)
        dump(out / 'launcher-result.json', execution)
        info, = parse(docker('container', 'inspect', container))
        owned_container(info, name, name, container)
        verify_container(info, volume, dialect, out)
        dump(out / 'container-after.json', info)
        native = parse((out / 'native/result.json').read_text())
        need(accepted(execution) and not info['State']['Running'] and info['State']['ExitCode'] == 0 and
             native['result'] == 'PASS' and sources == source_hashes() and sha(dialect) == codec_hash,
             'owned PX4 witness failed or protected inputs changed')
        report['native_result'] = native
        report['result'] = 'PASS'
    except (CIInterrupted, KeyboardInterrupt, OSError, RuntimeError, LookupError, ValueError,
            subprocess.SubprocessError) as error:
        report['error'] = f'{type(error).__name__}: {error}'
    finally:
        try:
            if endpoint and lease.handles:
                removed = remove_owned_container(endpoint.capture, name, name, container)
                report['container_cleanup'] = {'removed': True, 'container': removed or container}
        except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as error:
            report.update(result='FAIL', cleanup_error=str(error))
        finally:
            lease.release()
            if endpoint:
                endpoint.close()
            dump(out / 'result.json', report)
    print(f"{report['result']} PX4 Ackermann witness: {out / 'result.json'}")
    return report['result'] != 'PASS'


def interrupted(number, _):
    raise CIInterrupted(number, 'PX4 Rover native witness')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--output', type=Path)
    mode.add_argument('--worker', type=Path)
    parser.add_argument('--px4-package-volume')
    parser.add_argument('--dialect', type=Path, help='locally installed, generated pymavlink v20/common.py')
    parser.add_argument('--case', choices=('mission', 'offboard', 'both', 'stop-fallback'), default='mission')
    args = parser.parse_args()
    if bool(args.output) != bool(args.px4_package_volume) or bool(args.output) != bool(args.dialect):
        parser.error('host execution requires the retained PX4 package volume and local MAVLink dialect')
    os.umask(0o077)
    signal.signal(signal.SIGTERM, interrupted)
    if args.worker:
        return worker(args.worker, args.case)
    return run(args.output, args.px4_package_volume, args.dialect, args.case)


if __name__ == '__main__':
    raise SystemExit(main())
