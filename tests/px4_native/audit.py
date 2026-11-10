"""Check stock PX4 SIH capture files without starting a socket or a process."""

from collections import Counter
import hashlib
import json
from pathlib import Path
import re
from typing import NamedTuple

FIRMWARE_REVISION = 'fca3df865af36124a28c9d607e850f111dbaaea9'
FIRMWARE_BINARY = '23f0b5eb07430868b5525a5c59e28185fe5b0da35ade9a7e04bdea5894144ea8'
PORTS = {14540: 14580, 14550: 18570}
REQUIRED = {'HEARTBEAT', 'EXTENDED_SYS_STATE', 'GLOBAL_POSITION_INT', 'LOCAL_POSITION_NED', 'ODOMETRY'}
WIRES = (
    'firmware-binary.sha256', 'version.log', 'startup.log', 'px4-cmdline', 'px4-descriptors',
    'udp-sockets', 'interfaces', 'interface-flags', 'ipv4-routes', 'ipv6-routes',
    'ipv6-addresses', 'process-status', 'px4-wire.mavlink', 'px4-datagrams.tsv',
)


class Datagram(NamedTuple):
    nanoseconds: int
    port: int
    payload: bytes


def dump(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def need(condition, reason):
    if not condition:
        raise ValueError(reason)


def route_evidence(directory):
    flags = dict(line.split() for line in (directory / 'interface-flags').read_text().splitlines())
    need({interface for interface, value in flags.items() if int(value, 16) & 1} == {'lo'},
         'a non-loopback interface is active')
    need(len((directory / 'ipv4-routes').read_text().splitlines()) == 1 and
         all(line.split()[-1] == 'lo' for line in (directory / 'ipv6-routes').read_text().splitlines()),
         'a non-loopback route is active')
    ipv6 = [line.split() for line in (directory / 'ipv6-addresses').read_text().splitlines()]
    need(all(row[-1] == 'lo' and int(row[0], 16) == 1 for row in ipv6), 'unexpected IPv6 address')
    status = (directory / 'process-status').read_text()
    need(re.search(r'^CapEff:\s+0+$', status, re.M) and re.search(r'^NoNewPrivs:\s+1$', status, re.M),
         'capture container has unexpected process privileges')
    cmdline = (directory / 'px4-cmdline').read_text().split()
    need(cmdline == ['px4', '-d', '/opt/px4/etc', '-w', '/evidence', '-s', '/opt/px4/etc/init.d-posix/rcS'],
         'unexpected owned PX4 daemon command')
    descriptors = set(re.findall(r'^socket:\[(\d+)\]$', (directory / 'px4-descriptors').read_text(), re.M))
    socks = {}
    for raw in (directory / 'udp-sockets').read_text().splitlines()[1:]:
        row = raw.split()
        endpoint, port = row[1].split(':')
        port = int(port, 16)
        if port in PORTS.values():
            need(port not in socks and endpoint in ('00000000', '0100007F') and row[9] in descriptors,
                 'native source socket does not belong to the owned PX4 daemon')
            socks[port] = row[9]
    need(socks.keys() == set(PORTS.values()), 'missing exact PX4 source sockets')
    return {'up_interfaces': ['lo'], 'ipv4_routes': 0, 'px4_source_socket_inodes': socks,
            'cap_effective': 0, 'no_new_privileges': True}


def datagrams(directory):
    wire = directory / 'px4-wire.mavlink'
    index = directory / 'px4-datagrams.tsv'
    need(0 < wire.stat().st_size <= 4194304 and 0 < index.stat().st_size <= 786432,
         'capture exceeds evidence bounds')
    raw = wire.read_bytes()
    offset, previous = 0, -1
    records = index.read_text().splitlines()
    need(0 < len(records) <= 12000, 'invalid capture datagram count')
    for row in records:
        time, port, sender, start, length = map(int, row.split('\t'))
        need(previous <= time <= 16200000000 and time >= 0 and PORTS.get(port) == sender,
             'capture clock or UDP source differs')
        need(start == offset and 0 < length <= 65507 and start + length <= len(raw),
             'capture index has a gap, overlap, or truncated datagram')
        offset += length
        previous = time
        yield Datagram(time, port, raw[start:offset])
    need(offset == len(raw), 'capture contains unindexed bytes')


def source(packet):
    if packet[0] == 0xFD:
        return packet[5], packet[6]
    if packet[0] == 0xFE:
        return packet[3], packet[4]
    raise ValueError('unexpected wire frame prefix')


def decode_datagram(decoder, datagram):
    messages = decoder.parse_buffer(datagram.payload) or []
    need(b''.join(bytes(frame.get_msgbuf()) for frame in messages) == datagram.payload,
         'native decoding lost or joined UDP frame bytes')
    need(all(source(message.get_msgbuf()) == (1, 1) for message in messages), 'native MAVLink source differs')
    return messages


def audit_capture(directory):
    import pymavlink
    from pymavlink.dialects.v20 import ardupilotmega as wire

    proof = route_evidence(directory)
    version = (directory / 'version.log').read_text()
    need(f'PX4 git-hash: {FIRMWARE_REVISION}\n' in version and 'PX4 version: 1.18.0 c0 ' in version and
         'Build variant: sih\n' in version, 'unexpected PX4 revision, version or simulation variant')
    need((directory / 'firmware-binary.sha256').read_text().split() == [FIRMWARE_BINARY, '/opt/px4/bin/px4'],
         'unexpected firmware executable SHA-256')
    captured = list(datagrams(directory))
    decoders = {port: wire.MAVLink(None) for port in PORTS}
    families = {port: Counter() for port in PORTS}
    conditions = {port: {'heartbeat': set(), 'landed_state': set(), 'estimator_flags': set()} for port in PORTS}
    native = []
    for datagram in captured:
        for message in decode_datagram(decoders[datagram.port], datagram):
            family = message.get_type()
            families[datagram.port][family] += 1
            values = conditions[datagram.port]
            if family == 'HEARTBEAT':
                need(message.type == wire.MAV_TYPE_QUADROTOR and message.autopilot == wire.MAV_AUTOPILOT_PX4 and
                     not message.base_mode & wire.MAV_MODE_FLAG_SAFETY_ARMED,
                     'native PX4 heartbeat identity or disarmed state differs')
                values['heartbeat'].add((message.base_mode, message.custom_mode, message.system_status))
            if family == 'EXTENDED_SYS_STATE':
                values['landed_state'].add(message.landed_state)
            if family == 'ESTIMATOR_STATUS':
                values['estimator_flags'].add(message.flags)
            if datagram.port == 14540 and family in REQUIRED:
                native.append(message)
    for port in PORTS:
        expected = REQUIRED if port == 14540 else REQUIRED - {'ODOMETRY'}
        need(all(families[port][family] >= 3 for family in expected), 'missing native PX4 observations')
        need(conditions[port]['landed_state'] == {wire.MAV_LANDED_STATE_ON_GROUND},
             'native PX4 did not remain on ground')
    report = {
        'schema': 1, 'result': 'PASS', 'scope': 'owned receive-only PX4 SITL producer capture',
        'firmware_revision': FIRMWARE_REVISION, 'firmware_binary_sha256': FIRMWARE_BINARY,
        'fixture': 'sihsim_quadx', 'platform': 'Linux ARM64 OCI',
        'hardware': False, 'mavlink_writes_to_px4': 0, 'source': '1/1',
        'routes': proof, 'frames': families, 'datagrams': Counter(item.port for item in captured),
        'duration_s': (captured[-1].nanoseconds - captured[0].nanoseconds) / 1e9,
        'conditions': {port: {key: sorted(values) for key, values in checks.items()}
                       for port, checks in conditions.items()},
        'sha256': {file: sha(directory / file) for file in WIRES},
        'decoder': {'version': getattr(pymavlink, '__version__', 'unknown'),
                    'dialect_sha256': sha(wire.__file__),
                    'unknown_message_policy': 'bytes and source retained, payload and CRC not interpreted'},
    }
    return report, [item for item in captured if item.port == 14540], native
