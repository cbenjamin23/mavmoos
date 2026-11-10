#!/usr/bin/env python3
"""One exact owned disarmed Copter MAVLink FTP list, get and cancel witness."""
import argparse
from collections import Counter, defaultdict
from contextlib import ExitStack
from dataclasses import dataclass
import fcntl
import json
import os
from pathlib import Path
import re
import shlex
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time
import traceback


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tests'))
from flight_logs.run import cleared, fields, require, sha, start
from imu.run_native import dump, exact_peer, inventory, live_app, load_helpers
from observer_matrix.run_loopback import reserve


PROFILE = 'arducopter-4.6.3-quad'
FIRMWARE = '92b0cd788ec29406f26c6f9c31d5ceedbd1cc538'
MODULES = ('telemetry', 'mavlink_files')
REMOTE = 'mavmoos-ftp'
SEEDS = {'sample.bin': 965, 'empty.bin': 0, 'cancel.bin': 32768}
OPERATION_PLAN = (
    ('list1', 'list', REMOTE), ('get1', 'get', 'sample.bin'), ('list2', 'list', REMOTE),
    ('getempty', 'get', 'empty.bin'), ('list3', 'list', REMOTE), ('getstop', 'get', 'cancel.bin'),
    ('list4', 'list', REMOTE), ('get2', 'get', 'sample.bin'),
)
REQUESTS = tuple('schema=1,' + request for request in (
    'id=list1,action=list', 'id=get1,action=get,catalog=list1,name=sample.bin', 'id=list2,action=list',
    'id=getempty,action=get,catalog=list2,name=empty.bin', 'id=list3,action=list',
    'id=getstop,action=get,catalog=list3,name=cancel.bin', 'id=stop1,action=cancel,operation_id=getstop',
    'id=list4,action=list', 'id=get2,action=get,catalog=list4,name=sample.bin'))
REQUEST_KEY = 'MAVMOOS_FILE_REQUEST'
RESULT_KEY = 'MAVMOOS_FILE_RESULT'
CATALOG_KEY = 'MAVMOOS_FILE_CATALOG'
TX_KEY = 'MAVMOOS_FILE_TX_RESULT'
RX_KEY = 'MAVMOOS_FILE_RX_RESULT'
RUNTIME_KEY = 'MAVMOOS_RUNTIME_STATUS'
CAPS_KEY = 'MAVMOOS_CAPABILITIES'
APP_KEY = 'MAVMOOS_STATUS'
LIMIT = 4 * 1024 * 1024
TAP_HEADER = struct.Struct('<4scHHHQ')
FTP_HEADER = struct.Struct('<HBBBBBBI')
APP = (245, 191)
FC = (1, 1)
LIST, OPEN, READ, CLOSE = 3, 4, 5, 1
ACK, NAK = 128, 129
EOF = 6


def clean_revision(path):
    command = ['git', '-C', str(path)]
    status = subprocess.check_output([*command, 'status', '--porcelain', '--untracked-files=normal'], timeout=10)
    require(not status, f'selected checkout has changes: {path}')
    revision = subprocess.check_output([*command, 'rev-parse', '--verify', 'HEAD'], text=True, timeout=3).strip()
    require(re.fullmatch(r'[0-9a-f]{40}', revision), f'selected checkout revision unavailable: {path}')
    return revision


def firmware_identity(args, helpers):
    firmware = helpers['ownership'].native_firmware_profile(
        ROOT, args.ardupilot_root, args.firmware_manifest, args.firmware_attestation)
    manifest_path = ROOT / 'config/firmware' / (PROFILE + '.json')
    require(firmware and firmware['profile_id'] == PROFILE and firmware['source_revision'] == FIRMWARE
            and firmware['frame'] == 'quad' and firmware['fixture']['hardware'] is False
            and firmware['manifest_sha256'] == sha(manifest_path),
            'explicit pinned Copter 4.6.3 quad source manifest and native artifact attestation required')
    return firmware


def preflight(args):
    require(args.source_root == ROOT, 'native FTP runner must belong to the selected source')
    source_ref, moos_ref = clean_revision(ROOT), clean_revision(args.moos_root)
    helpers = load_helpers(ROOT)
    firmware = firmware_identity(args, helpers)
    manifest = helpers['manifest'].validate_manifest(Path(firmware['manifest_path']))
    require(moos_ref == manifest['dependencies']['moos_revision'], 'stock MOOS revision differs from firmware manifest')
    require(args.probe.is_relative_to(args.build) and args.probe.name == 'mavlink_files_moos_probe'
            and args.tap.is_relative_to(args.build)
            and args.tap.name in {'libmavlink_files_wire_tap.dylib', 'libmavlink_files_wire_tap.so'},
            'probe and tap must come from the selected fixture build')
    cache = args.build / 'CMakeCache.txt'
    text = cache.read_text()
    for key, selected in (('CMAKE_HOME_DIRECTORY', ROOT), ('MOOS_IVP_ROOT', args.moos_root)):
        rows = re.findall(rf'^{key}:[^=]+=([^\n]+)$', text, re.M)
        require(len(rows) == 1 and Path(rows[0]).resolve() == selected, f'configured build differs at {key}')
    paths = {
        'native': Path(firmware['native_path']), 'app': args.build / 'bin/iMAVLink',
        'probe': args.probe, 'tap': args.tap, 'antler': args.moos_root / 'bin/pAntler',
        'db': args.moos_root / 'bin/MOOSDB', 'helm': args.moos_root / 'bin/pHelmIvP',
        'defaults': args.ardupilot_root / firmware['fixture']['default_parameter_file'],
    }
    for module in MODULES:
        paths[module] = args.build / 'plugins' / f'mavmoos_{module}.so'
    protected = [cache, *paths.values()]
    protected.extend(ROOT / 'tests/mavlink_files' / name for name in (
        'run_native.py', 'FilesMoosProbe.cpp', 'OwnedUdpTap.cpp'))
    protected.extend(Path(module.__file__) for module in helpers.values())
    protected.extend(helpers['ownership'].firmware_source_inputs(ROOT, firmware))
    protected.extend(args.ardupilot_root / name for name in (
        'libraries/GCS_MAVLink/GCS_FTP.cpp', 'libraries/AP_Filesystem/AP_Filesystem.cpp',
        'libraries/AP_Filesystem/AP_Filesystem_posix.cpp'))
    require(all(path.is_file() and not path.is_symlink() for path in protected),
            'a required regular app, provider, source or fixture artifact is absent')
    for name in ('app', 'probe', 'tap'):
        expected = getattr(args, 'expected_' + name + '_sha256')
        require(expected is None or (re.fullmatch(r'[0-9a-f]{64}', expected) and sha(paths[name]) == expected),
                f'frozen {name} hash differs')
    return helpers, paths, protected, firmware, source_ref, moos_ref


def private_file(path, data):
    with os.fdopen(os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600), 'wb') as out:
        out.write(data)


def seed(out):
    sim = out / 'sim'
    sim.mkdir(mode=0o700)
    remote = sim / REMOTE
    remote.mkdir(mode=0o700)
    files = {}
    for name, size in SEEDS.items():
        path = remote / name
        data = bytes(((index * 37) ^ (index >> 3)) & 255 for index in range(size))
        private_file(path, data)
        files[name] = {'path': path.relative_to(out).as_posix(), 'bytes': size, 'sha256': sha(path)}
    return sim, files


def mission_text(ports, paths, downloads, pid):
    return f"""ServerHost = 127.0.0.1
ServerPort = {ports['moos']}
Community = mavmoos_files_{ports['moos']}
MOOSTimeWarp = 1
TERM_REPORTING = true
ProcessConfig = ANTLER
{{
  MSBetweenLaunches = 50
  Run = MOOSDB @ NewConsole = false
  Run = mavmoos-ftp-bridge @ NewConsole = false ~ iMAVLink
}}
ProcessConfig = iMAVLink
{{
  AppTick = 50
  CommsTick = 50
  bind_address = 127.0.0.1
  bind_port = {ports['bridge']}
  peer_address = 127.0.0.1
  peer_port = {ports['peer']}
  target_system = 1
  target_component = 1
  expected_type = 2
  expected_autopilot = 3
  simulation_only = true
  shadow_mode = false
  sitl_pid = {pid}
  firmware_requests = false
  plugins = {','.join(MODULES)}
  plugin_directory = {paths['telemetry'].parent}
  max_datagrams_per_iter = 128
  plugin.mavlink_files.directory = {downloads}
}}
"""


def app_launcher(out, paths, ports):
    platform = sys.platform
    require(platform == 'darwin' or platform.startswith('linux'), 'native UDP tap requires Darwin or Linux')
    key = 'DYLD_INSERT_LIBRARIES' if platform == 'darwin' else 'LD_PRELOAD'
    values = {key: paths['tap'], 'MAVMOOS_FTP_TAP_PATH': out / 'direct-wire.bin',
              'MAVMOOS_FTP_TAP_APP': ports['bridge'], 'MAVMOOS_FTP_TAP_PEER': ports['peer']}
    target = out / 'mavmoos-ftp-bridge'
    text = '#!/bin/sh\n' + ''.join(f'export {name}={shlex.quote(str(value))}\n' for name, value in values.items())
    target.write_text(text + f'exec {shlex.quote(str(paths["app"]))} "$@"\n')
    target.chmod(0o700)


def clean_environment():
    injected = {'DYLD_INSERT_LIBRARIES', 'LD_PRELOAD', 'MAVMOOS_FTP_TAP_PATH',
                'MAVMOOS_FTP_TAP_APP', 'MAVMOOS_FTP_TAP_PEER'}
    return {key: value for key, value in os.environ.items() if key not in injected}


def decode(codec, data):
    if data == b'0 ':
        return []
    parser = codec.MAVLink(None)
    messages = parser.parse_buffer(data) or []
    require(messages and not parser.buf_len()
            and all(not message.get_type().startswith(('BAD_DATA', 'UNKNOWN_')) for message in messages),
            'incomplete, corrupt or unknown MAVLink datagram')
    return messages


def peer_startup(receiver, simulator, codec, report):
    deadline = time.monotonic() + 23
    report['bridge_startup_non_mavlink'] = 0
    while time.monotonic() < deadline:
        require(simulator.poll() is None, 'owned Copter exited before startup heartbeat')
        try:
            data, sender = receiver.recvfrom(65535)
        except socket.timeout:
            continue
        require(sender[0] == '127.0.0.1', 'startup datagram came from outside the owned loopback route')
        # SITL UART0 may emit console text before it starts MAVLink. Never use
        # it to select the peer. Every framed candidate is still decoded strictly.
        if data[:1] not in (b'\xfd', b'\xfe'):
            report['bridge_startup_non_mavlink'] += 1
            require(report['bridge_startup_non_mavlink'] <= 256, 'excess non-MAVLink startup traffic')
            continue
        for message in decode(codec, data):
            if message.get_msgId() != 0:
                continue
            require((message.get_srcSystem(), message.get_srcComponent(), message.type, message.autopilot) == (1, 1, 2, 3)
                    and not message.base_mode & 128, 'startup heartbeat did not identify disarmed Copter')
            return sender[1]
    raise ValueError('owned disarmed heartbeat missing before startup deadline')


def exercise(out, downloads, sim, paths, helpers, codec, report, processes):
    ports = report['ports']
    env = clean_environment()
    with ExitStack() as stack:
        sockets = {name: stack.enter_context(reserve(kind)) for name, kind in (
            ('moos', socket.SOCK_STREAM), ('base', socket.SOCK_STREAM), ('bridge', socket.SOCK_DGRAM),
            ('monitor', socket.SOCK_DGRAM))}
        sockets['rc'] = stack.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
        sockets['rc'].bind(('0.0.0.0', 0))
        ports.update((name, sock.getsockname()[1]) for name, sock in sockets.items())
        sockets['bridge'].settimeout(.2)
        sockets['monitor'].settimeout(.1)
        fixture = out / 'disarmed.parm'
        fixture.write_text('LOG_DISARMED 0\nSERIAL0_PROTOCOL 2\nSERIAL1_PROTOCOL 2\n')
        (sim / 'message-intervals-chan1.txt').write_text('245 400\n')
        command = [paths['native'], '--model', 'quad', '--home', '42.358456,-71.087589,10,0',
                   '--speedup', '1', '--wipe', '--defaults', f'{paths["defaults"]},{fixture}',
                   '--serial0', f'udpclient:127.0.0.1:{ports["bridge"]}',
                   '--serial1', f'udpclient:127.0.0.1:{ports["monitor"]}',
                   '--serial2', 'none', '--serial5', 'none', '--serial6', 'none', '--serial7', 'none',
                   '--serial8', 'none', '--base-port', ports['base'], '--rc-in-port', ports['rc']]
        report['native_command'] = list(map(str, command))
        for name in ('base', 'rc'):
            sockets[name].close()
        simulator = start(command, sim, env, out / 'sitl.log')
        processes.append(simulator)
        report['sitl_pid'] = simulator.pid
        ports['peer'] = peer_startup(sockets['bridge'], simulator, codec, report)
        socket_proof = inventory(simulator)
        require(exact_peer(socket_proof, ports['peer'], ports['bridge']), 'native endpoint is not exact owned Copter')
        (out / 'native-sockets-before.txt').write_text(socket_proof)
        app_launcher(out, paths, ports)
        mission = out / 'mavlink-files.moos'
        mission.write_text(mission_text(ports, paths, downloads, simulator.pid))
        env['PATH'] = os.pathsep.join(map(str, (out, paths['app'].parent, paths['db'].parent, env.get('PATH', ''))))
        sockets['moos'].close()
        sockets['bridge'].close()
        antler = start([paths['antler'], mission], out, env, out / 'pantler.log')
        processes.append(antler)
        deadline = time.monotonic() + 7
        last_app_error = 'not examined'
        while time.monotonic() < deadline:
            try:
                owned, app_proof = live_app(helpers['ownership'], antler, mission, ports['bridge'])
                break
            except RuntimeError as error:
                last_app_error = str(error)
                require(antler.poll() is None, 'owned pAntler exited before app was bound')
                time.sleep(.1)
        else:
            report['app_startup_owned_processes'] = helpers['ownership'].owned_records(antler)
            raise ValueError('owned app socket is unavailable: ' + last_app_error)
        report['owned_app'] = owned
        (out / 'app-sockets-before.txt').write_text(app_proof)
        probe = start([paths['probe'], ports['moos']], out, env, out / 'probe.jsonl')
        processes.append(probe)
        deadline = time.monotonic() + 53
        drain_until = None
        monitor_packets = []
        report['monitor_startup_non_mavlink'] = 0
        monitor_bound = False
        raw_size = 0
        with (out / 'native-monitor.mavlink').open('xb') as raw, (out / 'native-monitor.jsonl').open('x') as trace:
            while time.monotonic() < deadline:
                require(simulator.poll() is None and antler.poll() is None, 'owned process exited during FTP workflow')
                if probe.poll() is not None:
                    require(probe.returncode == 0, 'FTP MOOS operator probe did not complete')
                    if drain_until is None:
                        drain_until = time.monotonic() + .45
                    if time.monotonic() >= drain_until:
                        break
                try:
                    data, sender = sockets['monitor'].recvfrom(65535)
                except socket.timeout:
                    continue
                stamp = time.monotonic()
                require(sender[0] == '127.0.0.1' and raw_size + len(data) <= LIMIT,
                        'independent monitor source or byte bound differs')
                if 'monitor_peer' not in ports:
                    ports['monitor_peer'] = sender[1]
                require(sender[1] == ports['monitor_peer'], 'independent monitor peer changed')
                raw.write(data)
                trace.write(json.dumps({'offset': raw_size, 'length': len(data), 'mono': stamp}) + '\n')
                raw_size += len(data)
                if not monitor_bound and data[:1] not in (b'\xfd', b'\xfe'):
                    report['monitor_startup_non_mavlink'] += 1
                    require(report['monitor_startup_non_mavlink'] <= 256, 'excess monitor startup traffic')
                    continue
                messages = decode(codec, data)
                monitor_bound |= any(message.get_msgId() == 0 and
                                     (message.get_srcSystem(), message.get_srcComponent()) == FC
                                     for message in messages)
                monitor_packets.extend((stamp, message) for message in messages)
            else:
                raise ValueError('disarmed FTP workflow exceeded its outer deadline')
        final = inventory(simulator)
        require(exact_peer(final, ports['peer'], ports['bridge']) and
                exact_peer(final, ports['monitor_peer'], ports['monitor']), 'owned native socket proof changed')
        _, app_proof = live_app(helpers['ownership'], antler, mission, ports['bridge'], owned)
        (out / 'native-sockets-after.txt').write_text(final)
        (out / 'app-sockets-after.txt').write_text(app_proof)
        report['probe_exit'] = probe.returncode
        report['monitor'] = ground_evidence(monitor_packets, time.monotonic())


def ground_evidence(rows, finished):
    heartbeat, landed = [], []
    forwarded_bridge = 0
    for stamp, message in rows:
        source = message.get_srcSystem(), message.get_srcComponent()
        if source == APP and message.get_msgId() == 0:
            # Pinned ArduPilot can forward heartbeat broadcasts to other active
            # MAVLink channels. These do not witness FC state.
            require(message.type == 18 and message.autopilot == 8 and message.base_mode == 0,
                    'monitor saw an unexpected bridge heartbeat')
            forwarded_bridge += 1
            continue
        require(source == FC, 'independent monitor changed FC identity')
        if message.get_msgId() == 0:
            require((message.type, message.autopilot) == (2, 3) and not message.base_mode & 128,
                    'independent monitor saw wrong identity or armed heartbeat')
            heartbeat.append(stamp)
        elif message.get_msgId() == 245:
            require(message.landed_state in (0, 1), 'independent monitor saw a non-ground state')
            if message.landed_state == 1:
                landed.append(stamp)
    require(len(heartbeat) >= 2 and len(landed) >= 2 and finished - heartbeat[-1] <= 2
            and finished - landed[-1] <= 2, 'independent native monitor lacked recent disarmed ground evidence')
    return {'heartbeats': len(heartbeat), 'ground': len(landed), 'forwarded_bridge_heartbeats': forwarded_bridge,
            'ever_armed': False, 'observer_writes_to_fc': 0}


@dataclass(frozen=True)
class FTP:
    sequence: int
    session: int
    opcode: int
    count: int
    request: int
    offset: int
    data: bytes

    def identity(self):
        return self.opcode, self.sequence, self.session, self.offset, self.count


def decode_ftp(message, direction):
    source = (message.get_srcSystem(), message.get_srcComponent())
    destination = (message.target_system, message.target_component)
    expected = (APP, FC) if direction == b'T' else (FC, APP)
    require((source, destination) == expected and message.target_network == 0, 'FTP packet target or source differs')
    raw = bytes(message.payload)
    sequence, session, opcode, count, request, burst, pad, offset = FTP_HEADER.unpack_from(raw)
    allowed = {CLOSE, LIST, OPEN, READ}
    require(session == 1 and burst == pad == 0 and count <= 239 and
            (opcode in allowed and request == 0 if direction == b'T' else opcode in (ACK, NAK) and request in allowed),
            'unexpected FTP session, opcode, burst, padding or size')
    data = raw[FTP_HEADER.size:FTP_HEADER.size + count]
    if direction == b'T':
        require((opcode in (LIST, OPEN) and data and b'\0' not in data) or
                (opcode == READ and count > 0 and not any(data)) or
                (opcode == CLOSE and count == offset == 0), 'invalid FTP request body')
    return FTP(sequence, session, opcode, count, request, offset, data)


def direct_wire(path, ports, codec):
    need_regular = path.is_file() and not path.is_symlink() and path.stat().st_mode & 0o077 == 0
    require(need_regular and 0 < path.stat().st_size <= LIMIT, 'exact app socket capture missing or out of bounds')
    data = path.read_bytes()
    offset = 0
    previous = 0
    protocol, rows = [], []
    outbound = Counter()
    startup_markers = 0
    known_console = {b'\n': 'newline', b'*': 'asterisk', b'0 ': 'udp_discovery'}
    telemetry = {33: 50000, 245: 250000, 193: 500000, 242: 500000}
    while offset < len(data):
        require(offset + TAP_HEADER.size <= len(data), 'incomplete app socket capture header')
        magic, direction, size, source, destination, stamp = TAP_HEADER.unpack_from(data, offset)
        end = offset + TAP_HEADER.size + size
        expected = (ports['bridge'], ports['peer']) if direction == b'T' else (ports['peer'], ports['bridge'])
        require(magic == b'FTP1' and direction in (b'T', b'R') and 0 < size <= 4096
                and end <= len(data) and previous <= stamp and (source, destination) == expected,
                'invalid socket trace framing, chronology or route')
        raw = data[offset + TAP_HEADER.size:end]
        if direction == b'R' and raw in known_console and not protocol:
            # Exact standalone UART0 boot markers can interleave with initial
            # telemetry. Retain the original tap bytes and never filter after FTP.
            startup_markers += 1
            require(startup_markers <= 64, 'excess exact console startup markers')
            rows.append({'direction': 'R', 'mono_ns': stamp, 'startup_console': known_console[raw]})
            offset, previous = end, stamp
            continue
        require(raw not in known_console, 'non-MAVLink marker outside native startup')
        messages = decode(codec, raw)
        require(direction == b'R' or messages, 'unframed app egress')
        for message in messages:
            source_id = message.get_srcSystem(), message.get_srcComponent()
            require(source_id == (APP if direction == b'T' else FC), 'direct MAVLink wire changed source')
            ident = message.get_msgId()
            if direction == b'T':
                outbound[ident] += 1
                if ident == 0:
                    require(message.type == 18 and message.autopilot == 8 and message.base_mode == 0,
                            'app heartbeat asserted control or arm state')
                elif ident == 76:
                    require((message.target_system, message.target_component, message.command) == (*FC, 511)
                            and message.param1 in telemetry and message.param2 == telemetry[message.param1]
                            and not any(getattr(message, f'param{i}') for i in range(3, 8)),
                            'app emitted an unexpected control or modifying command')
                else:
                    require(ident == 110, f'app emitted unexpected MAVLink ID {ident}')
            if ident == 110:
                item = decode_ftp(message, direction)
                protocol.append((direction, stamp, item))
                rows.append({'direction': direction.decode(), 'mono_ns': stamp, 'source': source_id,
                             'target': (message.target_system, message.target_component), 'sequence': item.sequence,
                             'session': item.session, 'opcode': item.opcode, 'request_opcode': item.request,
                             'offset': item.offset, 'size': item.count, 'data_hex': item.data.hex()})
        offset, previous = end, stamp
    require(outbound[0] and outbound[110] and any(direction == b'R' for direction, _, _ in protocol),
            'capture lacks app heartbeat, FTP request or native reply')
    return protocol, rows, outbound


def op_wire(protocol, tx):
    operations = defaultdict(list)
    sent = iter(tx)
    logical = {}
    order = []
    last_issued = None
    for direction, stamp, packet in protocol:
        if direction == b'T':
            receipt = next(sent, None)
            require(receipt is not None and receipt['clock_valid'] == 'true' and receipt['late'] == 'false'
                    and tuple(int(receipt[key]) for key in ('opcode', 'sequence', 'session', 'offset', 'count'))
                    == packet.identity(), 'app accepted TX receipt differs from direct wire')
            owner = receipt['operation_id']
            row = logical.get(packet.sequence)
            if row is not None:
                require(row is last_issued and row['owner'] == owner and row['request'] == packet,
                        'FTP sequence was reused or retried after newer work')
                row['attempts'] += 1
                continue
            if not order or order[-1] != owner:
                order.append(owner)
            row = {'owner': owner, 'request': packet, 'sent': stamp, 'attempts': 1, 'replies': []}
            logical[packet.sequence] = row
            operations[owner].append(row)
            last_issued = row
        else:
            row = logical.get((packet.sequence - 1) & 0xffff)
            require(row is not None and row['request'].session == packet.session
                    and row['request'].opcode == packet.request and stamp >= row['sent'],
                    'native FTP reply has no earlier matching direct request')
            replies = row['replies']
            require(not replies or replies[0][1] == packet, 'duplicate native FTP reply changed its contents')
            replies.append((stamp, packet))
    require(next(sent, None) is None and order == [ident for ident, _, _ in OPERATION_PLAN],
            'FTP wire omitted an accepted request or reordered operations')
    return operations


def response(row, required=True):
    replies = row['replies']
    require(replies or not required, 'required native FTP reply is absent')
    return replies[0][1] if replies else None


def audit_list(rows):
    offset = 0
    names = {}
    hidden = []
    for index, row in enumerate(rows):
        request, reply = row['request'], response(row)
        require(request.opcode == LIST and request.offset == reply.offset == offset and request.data == REMOTE.encode(),
                'ListDirectory request lost its fixed directory or entry offset')
        if index == len(rows) - 1:
            require(reply.opcode == NAK and reply.data == bytes([EOF]), 'directory terminal EOF is absent')
            continue
        require(reply.opcode == ACK and reply.data.endswith(b'\0'), 'invalid native directory page')
        entries = [item for item in reply.data.split(b'\0') if item]
        require(entries, 'empty directory page before EOF')
        for entry in entries:
            if entry in (b'D.', b'D..'):
                hidden.append(entry.decode())
                continue
            name, size = entry[1:].decode().rsplit('\t', 1)
            require(entry.startswith(b'F') and name not in names and name in SEEDS and int(size) == SEEDS[name],
                    'directory page included an unseeded path or wrong length')
            names[name] = int(size)
        offset += len(entries)
    require(names == SEEDS and len(hidden) == len(set(hidden)), 'native directory entries differ from fixed seeds')
    return {'catalog': names, 'native_entries': offset, 'request_count': len(rows), 'directory_eof': True}


def audit_get(rows, name, source, cancelled):
    opening, closing = rows[0], rows[-1]
    request, reply = opening['request'], response(opening)
    expected = f'{REMOTE}/{name}'.encode()
    require(len(rows) >= 2 and request.opcode == OPEN and request.offset == 0 and request.data == expected
            and reply.opcode == ACK and reply.offset == 0 and reply.count == 4
            and int.from_bytes(reply.data, 'little') == len(source),
            'native OpenFileRO lacked exact path or file-size proof')
    position = 0
    eof = False
    for row in rows[1:-1]:
        request = row['request']
        reply = response(row, not cancelled)
        require(request.opcode == READ and request.offset == position and not eof,
                'ReadFile did not consume the original file sequentially')
        if position == len(source):
            require(reply is not None and request.count == 1 and reply.opcode == NAK
                    and reply.offset == 0 and reply.data == bytes([EOF]),
                    'ReadFile did not verify EOF at the announced length')
            eof = True
            continue
        if reply is None:
            require(cancelled and row is rows[-2], 'missing ReadFile reply before a later read')
            continue
        expected_count = min(request.count, len(source) - position)
        require(reply.opcode == ACK and reply.offset == position and reply.count == expected_count
                and reply.data == source[position:position + expected_count],
                'ReadFile bytes or offset differ from the seeded source')
        position += reply.count
    final = response(closing)
    require(closing['request'].opcode == CLOSE and final.opcode == ACK and final.offset == final.count == 0,
            'TerminateSession did not receive exact matching native ACK')
    if cancelled:
        require(len(rows) > 2 and not eof and position < len(source),
                'cancel did not interrupt an in-progress native read')
    else:
        require(position == len(source) and eof, 'complete download omitted native bytes or verified EOF')
    return {'native_bytes': position, 'requests': len(rows), 'verified_eof': eof, 'session_closed_ack': True,
            'close_sequence': final.sequence, 'close_observed_mono_ns': closing['replies'][0][0]}


def load_probe(path):
    trace = [json.loads(line) for line in path.read_text().splitlines() if line.startswith('{')]
    require(trace and all(previous['mono'] <= current['mono'] for previous, current in zip(trace, trace[1:])),
            'MOOS operator trace is absent or out of order')
    requests = [row for row in trace if row['kind'] == 'request']
    require(all(row['key'] == REQUEST_KEY for row in requests) and tuple(row['value'] for row in requests) == REQUESTS,
            'operator request workflow differs from its bounded list/get/cancel plan')
    channels = defaultdict(list)
    for row in trace:
        if row['kind'] == 'app':
            channels[row['key']].append(row['value'])
    runtime = [fields(row) for row in channels[RUNTIME_KEY]]
    require(runtime and all(row['mode'] == 'SITL_CONTROL' and row['armed'] == row['in_air'] == '0'
                            and row['permission'] == 'false' and row['control_profile'] == 'none'
                            and row['nav_tx_accepted'] == '0' for row in runtime)
            and any(row['heartbeat_valid'] == row['landed_valid'] == '1' for row in runtime)
            and all(fields(row)['helm'] == 'disabled' for row in channels[APP_KEY]),
            'app runtime did not stay disarmed and without navigation authority')
    return trace, channels


def audit_moos(out, downloads, seeds, helpers, paths, channels):
    catalog = [json.loads(row) for row in channels[CAPS_KEY]]
    require(catalog and all(row == catalog[0] for row in catalog), 'app composition changed during FTP fixture')
    composition = catalog[0]['composition']
    require(catalog[0]['execution_mode'] == 'SITL_CONTROL' and composition['plugin_abi'] == 10 and
            {module['name'] for module in composition['modules']} == set(MODULES)
            and all(not module['control_provider'] for module in composition['modules']),
            'app did not load exactly the disarmed file-service composition')
    module_hashes = helpers['catalog'].loaded_module_hashes(paths['telemetry'].parent, catalog[0])
    outcomes = defaultdict(list)
    terminal = {}
    for text in channels[RESULT_KEY]:
        row = json.loads(text)
        require(row['schema'] == 1, 'unexpected FTP result schema')
        outcomes[row['id']].append(row)
    expected = {ident: (action, 'LISTED' if action == 'list' else 'COMPLETE')
                for ident, action, _ in OPERATION_PLAN}
    expected['getstop'] = ('get', 'CANCELLED')
    require(expected.keys() <= outcomes.keys() <= (expected.keys() | {'stop1'}), 'wrong FTP result operation roster')
    for ident, (action, state) in expected.items():
        rows = outcomes[ident]
        result = [row for row in rows if row['status'] == state]
        require(len(result) == 1 and rows[-1] == result[0] and all(row['action'] == action for row in rows),
                f'{ident}: expected {state} terminal operator result')
        terminal[ident] = result[0]
    if outcomes.get('stop1'):
        require(outcomes['stop1'][-1]['status'] == 'CANCEL_ACCEPTED', 'unexpected cancel request response')
    names = []
    for text in channels[CATALOG_KEY]:
        row = json.loads(text)
        entries = row['entries']
        if row['id'] is None:
            require(row['schema'] == 1 and not entries, 'invalid catalog clearance')
            names.append(None)
            continue
        actual = {entry['name']: entry['bytes'] for entry in entries if entry['kind'] == 'file'}
        require(row['schema'] == 1 and actual == SEEDS and len(entries) == len(SEEDS),
                'app catalog differs from exact seeded files')
        names.append(row['id'])
    require(names == ['list1', None, 'list2', None, 'list3', None, 'list4', None],
            'expected four distinct single-use catalogs and their invalidations')
    releases = {}
    for ident, action, name in OPERATION_PLAN:
        if action == 'list':
            continue
        result = terminal[ident]
        require(result['remote_session'] == 'confirmed_closed', f'{ident}: missing confirmed native closure')
        if ident == 'getstop':
            require(result['file'] is None, 'cancel released a partial file')
            continue
        filename = ident + '.bin'
        source, release = out / seeds[name]['path'], downloads / filename
        require(result['file'] == filename and result['bytes'] == seeds[name]['bytes'] and release.is_file()
                and not release.is_symlink() and release.stat().st_nlink == 1
                and release.stat().st_mode & 0o077 == 0 and sha(release) == seeds[name]['sha256']
                and release.read_bytes() == source.read_bytes(), f'{ident}: released local bytes or private mode differ')
        releases[filename] = {'source': name, 'bytes': result['bytes'], 'sha256': sha(release)}
    require({path.name for path in downloads.iterdir()} == releases.keys(), 'cancel staging or unexpected file remained')
    return terminal, releases, module_hashes


def audit_rx(rows, operations):
    actual = Counter()
    for text in rows:
        value = fields(text)
        row = (value['operation_id'], *(
            int(value[key]) for key in ('reply', 'opcode', 'sequence', 'session', 'offset', 'count')))
        actual[row] += 1
    available = Counter()
    required = Counter()
    for ident, requests in operations.items():
        for record in requests:
            for _, packet in record['replies']:
                row = (ident, packet.opcode, packet.request, packet.sequence, packet.session, packet.offset, packet.count)
                available[row] += 1
                if not (ident == 'getstop' and packet.request == READ):
                    required[row] = 1
    require(required <= actual <= available and all(count == 1 for count in actual.values()),
            'host FTP reply receipts differ from independently captured native packets')
    return len(actual)


def audit(out, downloads, seeds, helpers, paths, report, codec):
    trace, channels = load_probe(out / 'probe.jsonl')
    terminal, releases, modules = audit_moos(out, downloads, seeds, helpers, paths, channels)
    protocol, rows, ids = direct_wire(out / 'direct-wire.bin', report['ports'], codec)
    with (out / 'ftp-wire.jsonl').open('x') as output:
        for row in rows:
            output.write(json.dumps(row) + '\n')
    tx = [fields(row) for row in channels[TX_KEY]]
    accepted = [row for row in tx if row['status'] == 'LOCAL_ACCEPTED']
    operations = op_wire(protocol, accepted)
    observed = {}
    for ident, action, name in OPERATION_PLAN:
        work = operations[ident]
        if action == 'list':
            observed[ident] = audit_list(work)
        else:
            source = (out / seeds[name]['path']).read_bytes()
            observed[ident] = audit_get(work, name, source, ident == 'getstop')
    close = observed['getstop']['close_observed_mono_ns']
    require(close < operations['list4'][0]['sent'], 'reacquisition preceded the native cancel close ACK')
    stop_position = next(index for index, row in enumerate(trace) if row['value'] == REQUESTS[6])
    require(any(row['key'] == TX_KEY and fields(row['value']).get('operation_id') == 'getstop'
                and fields(row['value']).get('opcode') == '5'
                and fields(row['value']).get('status') == 'LOCAL_ACCEPTED' for row in trace[:stop_position]),
            'operator cancel was requested before a real ReadFile handoff')
    report.update(operator_results=terminal, released_files=releases, loaded_modules_sha256=modules,
                  native_ftp_operations=observed, app_accepted_tx_receipts=len(accepted),
                  app_accepted_rx_receipts=audit_rx(channels[RX_KEY], operations),
                  outbound_mavlink_ids={str(ident): count for ident, count in ids.items()},
                  wire_capture={'source': 'app_direct_udp_syscall_completion', 'tap': 'test_only',
                                'relay': False, 'privileged_capture': False,
                                'startup_console_markers': sum('startup_console' in row for row in rows),
                                'bytes': (out / 'direct-wire.bin').stat().st_size,
                                'sha256': sha(out / 'direct-wire.bin')})


def run(args, lock):
    from pymavlink.dialects.v20 import ardupilotmega

    out = args.output
    out.mkdir(parents=True, mode=0o700, exist_ok=False)
    report = {'schema': 1, 'result': 'FAIL', 'scenario': 'mavlink_files', 'profile': 'arducopter',
              'scope': 'owned disarmed Copter 4.6.3 MAVLink FTP list, get and cancel',
              'hardware': False, 'helm_started': False, 'arming_requested': False, 'rc_input_sent': False,
              'observer_writes_to_fc': 0, 'native_lock': str(lock), 'ports': {}}
    before = {}
    processes = []
    ownership = None
    try:
        helpers, paths, protected, firmware, source_ref, moos_ref = preflight(args)
        ownership = helpers['ownership']
        report.update(mavmoos_revision=source_ref, moos_revision=moos_ref, ardupilot_revision=FIRMWARE,
                      firmware_identity=firmware, native_sha256=firmware['native_sha256'])
        for key in ('app', 'probe', 'tap', 'helm'):
            report[key + '_sha256'] = sha(paths[key])
        report['copied_firmware'] = {}
        for key in ('manifest', 'attestation'):
            destination = out / f'firmware-{key}.json'
            private_file(destination, Path(firmware[key + '_path']).read_bytes())
            require(sha(destination) == firmware[key + '_sha256'], 'copied firmware input differs')
            protected.append(destination)
            report['copied_firmware'][key] = {'file': destination.name, 'sha256': sha(destination)}
        downloads = out / 'downloads'
        downloads.mkdir(mode=0o700)
        sim, seeds = seed(out)
        report['seeds'] = seeds
        protected.extend(out / row['path'] for row in seeds.values())
        before = {str(path): sha(path) for path in protected}
        report['artifacts_before'] = before
        exercise(out, downloads, sim, paths, helpers, ardupilotmega, report, processes)
    except BaseException as error:
        report['error'] = str(error) or type(error).__name__
        report['traceback'] = traceback.format_exc()
    finally:
        remaining, errors = [], []
        for process in reversed(processes):
            try:
                remaining.extend(ownership.stop_process_tree(process))
            except Exception as error:
                errors.append(f'{process.pid}: {error}')
        report['owned_processes_remaining'] = remaining
        report['cleanup_errors'] = errors
        report['ports_clear'] = {
            name: cleared(number, socket.SOCK_STREAM if name in ('moos', 'base') else socket.SOCK_DGRAM)
            for name, number in report['ports'].items()}
        report['cleanup_proven'] = (bool(report['ports_clear']) and all(report['ports_clear'].values())
                                    and not remaining and not errors)
    try:
        if before:
            report['artifacts_after'] = {key: sha(Path(key)) if Path(key).is_file() else None for key in before}
            report['mavmoos_revision_after'] = clean_revision(ROOT)
            report['moos_revision_after'] = clean_revision(args.moos_root)
            report['firmware_identity_after'] = firmware_identity(args, helpers)
            require(before == report['artifacts_after'] and report['mavmoos_revision_after'] == source_ref
                    and report['moos_revision_after'] == moos_ref and report['firmware_identity_after'] == firmware,
                    'protected file, source or firmware drift')
        if 'error' not in report:
            require(report['cleanup_proven'], 'scoped owned process or port cleanup was not proved')
            audit(out, downloads, seeds, helpers, paths, report, ardupilotmega)
            report['result'] = 'PASS'
    except Exception as error:
        report['error'] = str(error) or type(error).__name__
        report['traceback'] = traceback.format_exc()
    report['files'] = {path.relative_to(out).as_posix(): sha(path) for path in sorted(out.rglob('*'))
                       if path.is_file() and not path.is_symlink()}
    dump(out / 'result.json', report)
    print(json.dumps({'result': report['result'], 'error': report.get('error'), 'output': str(out)}))
    return int(report['result'] != 'PASS')


def interrupted(*_):
    raise InterruptedError('native FTP supervisor requested scoped teardown')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('source-root', 'ardupilot-root', 'moos-root', 'build', 'probe', 'tap', 'output'):
        parser.add_argument('--' + name, type=lambda value: Path(value).resolve(), required=True)
    for name in ('firmware-manifest', 'firmware-attestation'):
        parser.add_argument('--' + name, type=lambda value: Path(value).resolve())
    for name in ('app', 'probe', 'tap'):
        parser.add_argument('--expected-' + name + '-sha256')
    args = parser.parse_args()
    signal.signal(signal.SIGTERM, interrupted)
    lock_path = Path(tempfile.gettempdir()) / 'mavmoos-native-sitl.lock'
    with lock_path.open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return run(args, lock_path)


if __name__ == '__main__':
    raise SystemExit(main())
