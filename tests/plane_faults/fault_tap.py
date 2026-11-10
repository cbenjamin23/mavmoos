"""Arm and replay the test-only direct IPv4 Plane fault tap."""

from dataclasses import dataclass
import os
from pathlib import Path
import struct
import sys
import tempfile
import time


MODES = frozenset(('pass', 'drop_rx', 'drop_tx', 'drop_loiter', 'hold_point', 'release_point', 'drop_mode_ack', 'exit_before_takeoff', 'hold_hover', 'drop_hover',
                   'drop_takeoff_ack', 'drop_rtl_ack'))
HEADER = struct.Struct('<4scHHHQ')
TRACE_LIMIT = 16 * 1024 * 1024


@dataclass(frozen=True)
class Datagram:
    disposition: str
    source_port: int
    destination_port: int
    at: float
    data: bytes


def steady_time():
    clock = time.CLOCK_MONOTONIC_RAW if sys.platform == 'darwin' else time.CLOCK_MONOTONIC
    return time.clock_gettime(clock)


def environment(tap_library, app_port, peer_port, trace_path, control_path):
    if sys.platform != 'darwin' and not sys.platform.startswith('linux'):
        raise ValueError('Plane fault tap requires Darwin or Linux')
    key = 'DYLD_INSERT_LIBRARIES' if sys.platform == 'darwin' else 'LD_PRELOAD'
    library = Path(tap_library).resolve(strict=True)
    if not library.is_file():
        raise ValueError('fault tap library must be a file')
    if app_port == peer_port or any(type(port) is not int or not 0 < port <= 65535
                                    for port in (app_port, peer_port)):
        raise ValueError('fault tap requires two distinct fixed IPv4 loopback ports')
    trace, control = Path(trace_path).absolute(), Path(control_path).absolute()
    if trace == control or trace.parent != control.parent or not trace.parent.is_dir():
        raise ValueError('fault tap files must share the existing private run directory')
    if os.path.lexists(trace) or control.is_symlink():
        raise ValueError('fault tap capture must be fresh and its control path must not be a symlink')
    return {key: str(library), 'MAVMOOS_PLANE_TAP_APP': str(app_port),
            'MAVMOOS_PLANE_TAP_PEER': str(peer_port), 'MAVMOOS_PLANE_TAP_PATH': str(trace),
            'MAVMOOS_PLANE_TAP_CONTROL': str(control)}


def clean_environment():
    return {key: value for key, value in os.environ.items()
            if key not in ('DYLD_INSERT_LIBRARIES', 'LD_PRELOAD') and
            not key.startswith('MAVMOOS_PLANE_TAP_')}


def set_mode(control_path, mode):
    """Publish one complete mode setting and return its pre/post local clock bracket."""
    if mode not in MODES:
        raise ValueError('unknown Plane fault mode: ' + str(mode))
    path = Path(control_path)
    if path.is_symlink():
        raise ValueError('fault tap control path must not be a symlink')
    descriptor, temporary = tempfile.mkstemp(prefix='.' + path.name + '.', dir=path.parent)
    try:
        with os.fdopen(descriptor, 'wb') as stream:
            os.fchmod(stream.fileno(), 0o600)
            stream.write((mode + '\n').encode('ascii'))
        before = steady_time()
        os.replace(temporary, path)
        return before, steady_time()
    finally:
        if os.path.lexists(temporary):
            os.unlink(temporary)


def read_trace(trace_path, app_port, peer_port):
    """Replay the capture after teardown and fail on wrong route or partial proof."""
    with Path(trace_path).open('rb') as stream:
        content = stream.read(TRACE_LIMIT + 1)
    if not 0 < len(content) <= TRACE_LIMIT:
        raise ValueError('empty or oversized Plane fault capture')
    records = []
    offset = 0
    last_stamp = 0
    pending = None
    while offset < len(content):
        if len(content) - offset < HEADER.size:
            raise ValueError('truncated or invalid Plane fault capture header')
        magic, disposition, size, source, destination, stamp = HEADER.unpack_from(content, offset)
        offset += HEADER.size
        if magic != b'PLF1' or disposition not in b'TRoilhqde' or not 0 < stamp or stamp < last_stamp:
            raise ValueError('invalid Plane fault capture header or local clock')
        if size > 4096 or offset + size > len(content):
            raise ValueError('truncated or oversized Plane fault capture datagram')
        route = (peer_port, app_port) if disposition in b'Ri' else (app_port, peer_port)
        if (source, destination) != route:
            raise ValueError('Plane fault capture tuple changed')
        payload = content[offset:offset + size]
        offset += size
        if disposition == b'h':
            if pending is not None:
                raise ValueError('second Plane target held while first still pending')
            pending = payload
        elif disposition == b'q' and pending is None:
            raise ValueError('suppressed Plane target without a held target')
        elif disposition == b'd':
            if pending is None or pending != payload:
                raise ValueError('released Plane target does not match held bytes')
            pending = None
        records.append(Datagram(disposition.decode('ascii'), source, destination, stamp / 1e9, payload))
        last_stamp = stamp
    return records
