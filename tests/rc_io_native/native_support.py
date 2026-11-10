"""Ownership and exact-source helpers. Importing this module performs no I/O."""
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import re
import shlex
import socket
import struct
import subprocess
import time


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def ref(path):
    return subprocess.check_output(['git', '-C', str(path), 'rev-parse', 'HEAD'], text=True).strip()


def parse(text):
    def finite(value):
        number = float(value)
        if not math.isfinite(number):
            raise ValueError('nonfinite/overflow JSON number')
        return number
    def reject(value):
        raise ValueError('nonfinite JSON ' + value)
    return json.loads(text, parse_float=finite, parse_constant=reject)


def source_helpers(source):
    """Freshly resolve helpers from the actual build source, without import caches."""
    source = Path(source).resolve()
    result = {}
    for role, relative in (('ownership', 'tests/owned_processes.py'), ('catalog', 'tests/control/catalog_evidence.py')):
        path = (source / relative).resolve()
        if not path.is_relative_to(source):
            raise RuntimeError('source helper escapes specified source root')
        spec = importlib.util.spec_from_file_location('_mavmoos_rc_native_' + role, path)
        if spec is None or spec.loader is None:
            raise RuntimeError('cannot load exact source helper ' + str(path))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        result[role] = module
    return result


def sockets(process):
    if process.poll() is not None:
        raise RuntimeError('owned native process exited')
    answer = subprocess.run(['lsof', '-nP', '-a', '-p', str(process.pid), '-iUDP'], capture_output=True, text=True, timeout=3)
    if answer.returncode:
        raise RuntimeError('owned native UDP inventory unavailable')
    return answer.stdout


def owns_udp(inventory, port, allow_wildcard=False):
    address = r'(?:\*|0\.0\.0\.0|127\.0\.0\.1)' if allow_wildcard else r'127\.0\.0\.1'
    return bool(re.search(r'UDP\s+' + address + ':' + str(port) + r'\b', inventory))


def exact_peer(inventory, source, destination):
    return bool(re.search(r'UDP\s+127\.0\.0\.1:' + str(source) + r'->127\.0\.0\.1:' + str(destination) + r'\b', inventory))


def live_app(ownership, launcher, mission, bridge, expected=None, helm_expected=None):
    records = ownership.owned_records(launcher)
    selected = {}
    for name in ('iMAVLink', 'pHelmIvP'):
        found = []
        for row in records:
            try:
                tokens = shlex.split(row['command'])
            except ValueError:
                continue
            if tokens and Path(tokens[0]).name == name and str(mission) in tokens:
                found.append(row)
        if name == 'iMAVLink' or helm_expected is not None:
            if len(found) != 1:
                raise RuntimeError('exact owned mission ' + name + ' is not unique/live')
        if len(found) == 1:
            selected[name] = found[0]
    app = selected['iMAVLink']
    if expected and (app['pid'] != expected['pid'] or app['command'] != expected['command']):
        raise RuntimeError('owned app identity changed')
    if helm_expected:
        helm = selected['pHelmIvP']
        if helm['pid'] != helm_expected['pid'] or helm['command'] != helm_expected['command']:
            raise RuntimeError('owned stock helm identity changed')
    answer = subprocess.run(['lsof', '-nP', '-a', '-p', str(app['pid']), '-iUDP'], capture_output=True, text=True, timeout=3)
    if answer.returncode or not owns_udp(answer.stdout, bridge):
        raise RuntimeError('owned app bound socket no longer live')
    return selected, answer.stdout


class NativePilot:
    """Only an exact owned native RC socket. Never MAVLink RC_OVERRIDE."""
    def __init__(self, process, rc, control, output, vehicle):
        self.process, self.rc, self.control, self.vehicle = process, rc, Path(control), vehicle
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('127.0.0.1', 0))
        self.trace = Path(output).open('w')
        self.last = self.start = 0.0
        self.sent = 0
        self.previous = None
        inventory = sockets(process)
        if not owns_udp(inventory, rc, allow_wildcard=True):
            self.socket.close();self.trace.close()
            raise RuntimeError('exact owned native RC socket not verified')
        self.write({'event': 'owned_rc_socket_verified', 'pid': process.pid, 'port': rc, 'inventory': inventory})

    def write(self, row):
        self.trace.write(json.dumps(row) + '\n');self.trace.flush()

    def update(self):
        at = time.monotonic()
        if at - self.last < .1:
            return
        if self.process.poll() is not None:
            raise RuntimeError('owned native exited before RC send')
        if not self.start:
            self.start = at
        if self.vehicle == 'copter':
            phase = ('A', 'B', 'C', 'D')[min(3, int((at - self.start) // 8))]
            pwm = [1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500]
            if phase == 'B':pwm[0], pwm[3] = 1100, 1700
            if phase == 'C':pwm[0], pwm[3] = 1900, 1300
        elif self.vehicle == 'rover':
            phase = 'MANUAL_SWITCH' if (self.control / 'takeover.request').is_file() else 'INITIAL_HOLD'
            pwm = [1500, 0, 1500, 0, 0, 0, 0, 1000 if phase == 'MANUAL_SWITCH' else 1500]
        else:
            raise RuntimeError('unsupported native RC fixture')
        if phase != self.previous:
            inventory = sockets(self.process)
            if not owns_udp(inventory, self.rc, allow_wildcard=True):
                raise RuntimeError('native RC ownership lost')
            pending = self.control / 'rc.phase.pending'
            pending.write_text(phase + '\n');pending.replace(self.control / 'rc.phase')
            self.write({'event': 'phase', 'sent_mono': at, 'phase': phase, 'pwm': pwm, 'inventory': inventory})
            self.previous = phase
        packet = struct.pack('<8H', *pwm)
        if self.socket.sendto(packet, ('127.0.0.1', self.rc)) != len(packet):
            raise RuntimeError('owned native RC local handoff failed')
        self.last = at;self.sent += 1
        self.write({'sent_mono': at, 'phase': phase, 'target': ['127.0.0.1', self.rc], 'pwm': pwm,
                    'zero_channels': 'retain', 'local_acceptance_only': True})

    def close(self):
        self.socket.close();self.trace.close()
