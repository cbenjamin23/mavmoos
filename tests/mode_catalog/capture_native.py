#!/usr/bin/env python3
"""Capture the pinned PX4 mode reply and normal telemetry on owned Linux loopback."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import time

PIN = 'fca3df865af36124a28c9d607e850f111dbaaea9'
PX4_SHA = '23f0b5eb07430868b5525a5c59e28185fe5b0da35ade9a7e04bdea5894144ea8'
PACKAGE = 'mavmoos-px4-wave-package'
PX4 = Path('/opt/px4/bin/px4')


def require(condition, description):
    if not condition:
        raise RuntimeError(description)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def frames(data):
    offset = 0
    while offset < len(data):
        require(data[offset] in (253, 254) and len(data) - offset >= 8, 'invalid frame prefix')
        v2 = data[offset] == 253
        header = 10 if v2 else 6
        body = data[offset + 1]
        signed = v2 and bool(data[offset + 2] & 1)
        end = offset + header + body + 2 + (13 if signed else 0)
        require(end <= len(data), 'partial MAVLink datagram')
        frame = data[offset:end]
        msgid = int.from_bytes(frame[7:10], 'little') if v2 else frame[5]
        system, component = frame[5:7] if v2 else frame[3:5]
        yield msgid, system, component, frame[header:header + body], frame
        offset = end


def namespace():
    status = dict(line.split(':', 1) for line in Path('/proc/self/status').read_text().splitlines())
    flags = {item.parent.name: int(item.read_text(), 16)
             for item in Path('/sys/class/net').glob('*/flags')}
    routes = Path('/proc/self/net/route').read_text().splitlines()[1:]
    require([key for key, value in flags.items() if value & 1] == ['lo'] and not routes and
            status['CapEff'].strip() == '0000000000000000' and status['NoNewPrivs'].strip() == '1',
            'native capture requires loopback-only cap-dropped Linux')
    return {'interface_flags': flags, 'route_count': len(routes), 'capabilities': 0,
            'no_new_privileges': True}


def owns_peer(process):
    pid = Path('/proc') / str(process.pid)
    require((pid / 'exe').resolve() == PX4 and
            os.readlink(pid / 'ns/net') == os.readlink('/proc/self/ns/net'),
            'simulator executable or network namespace differs')
    sockets = set()
    for item in (pid / 'fd').iterdir():
        try:
            name = os.readlink(item)
        except FileNotFoundError:
            continue
        if name.startswith('socket:['):
            sockets.add(name[8:-1])
    matches = [line.split() for line in (pid / 'net/udp').read_text().splitlines()[1:]]
    routes = [row for row in matches if row[1] in ('00000000:38F4', '0100007F:38F4') and row[9] in sockets]
    require(len(routes) == 1, 'simulator does not own selected onboard UDP socket')
    return {'pid': process.pid, 'socket_inode': routes[0][9], 'address': routes[0][1]}


def read_request(output):
    data = (output / 'request-input.mavlink').read_bytes()
    (msgid, system, component, body, frame), = frames(data)
    require((msgid, system, component) == (76, 245, 190) and frame[0] == 253 and not frame[2] and
            32 <= len(body) <= 33 and struct.unpack('<7fHBBB', body.ljust(33, b'\0')) ==
            (435., 0., 0., 0., 0., 0., 0., 512, 1, 1, 0),
            'fixture can only request AVAILABLE_MODES from 1/1')
    return data


def capture(output):
    policy = namespace()
    request = read_request(output)
    require(sha(PX4) == PX4_SHA, 'wrong pinned PX4 executable')
    result = {'result': 'FAIL', 'namespace': policy, 'px4_binary_sha256': PX4_SHA, 'request_count': 0}
    process = None
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as peer, \
                (output / 'startup.log').open('wb') as log, \
                (output / 'wire.mavlink').open('xb') as raw, \
                (output / 'datagrams.tsv').open('x') as offsets:
            peer.bind(('127.0.0.1', 14540))
            peer.settimeout(.15)
            env = dict(os.environ, PX4_SIM_MODEL='sihsim_quadx', PX4_SIMULATOR='sihsim')
            env['PATH'] = '/opt/px4/bin:' + env['PATH']
            process = subprocess.Popen([PX4, '-d', '/opt/px4/etc', '-w', output, '-s',
                                        '/opt/px4/etc/init.d-posix/rcS'],
                                       cwd=output, stdout=log, stderr=subprocess.STDOUT, env=env)
            started = time.monotonic()
            request_at = None
            count = 0
            expected_modes = None
            mode_indices = set()
            while time.monotonic() - started < 12:
                require(process.poll() is None, 'PX4 exited during capture')
                try:
                    wire, sender = peer.recvfrom(65535)
                except TimeoutError:
                    continue
                require(sender == ('127.0.0.1', 14580) and wire, 'unexpected sender')
                now = time.monotonic() - started
                require(count < 14000 and raw.tell() + len(wire) <= 4 * 1024 * 1024,
                        'capture evidence limit')
                offsets.write(f'{now:.9f}\t{raw.tell()}\t{len(wire)}\n')
                raw.write(wire)
                count += 1
                for msgid, system, component, body, _ in frames(wire):
                    if request_at is not None and (msgid, system, component) == (435, 1, 1):
                        require(len(body) >= 10 and 0 < body[9] <= body[8] <= 64,
                                'malformed native mode row')
                        if expected_modes is None:
                            expected_modes = body[8]
                        require(body[8] == expected_modes, 'native available-mode count changed')
                        mode_indices.add(body[9])
                    if msgid != 0 or (system, component) != (1, 1):
                        continue
                    require(len(body) >= 9 and body[5] == 12 and not body[6] & 128,
                            'expected disarmed PX4 heartbeat')
                    if request_at is not None or now < 2:
                        continue
                    result['owned_peer'] = owns_peer(process)
                    require(peer.sendto(request, sender) == len(request), 'read request truncated')
                    (output / 'request.mavlink').write_bytes(request)
                    result['request_count'] = 1
                    request_at = now
                if request_at is not None and now - request_at >= 3.5:
                    break
            version = subprocess.check_output(['/opt/px4/bin/px4-ver', 'all'], cwd=output, env=env, timeout=2,
                                              text=True)
            (output / 'version.log').write_text(version)
            require(PIN in version and result['request_count'] == 1, 'native source or mode request missing')
            require(expected_modes and len(mode_indices) == expected_modes, 'native available-mode reply incomplete')
            result.update(result='PASS', datagrams=count, firmware_source=PIN, request_at_s=request_at,
                          available_modes=expected_modes)
    except Exception as error:
        result['error'] = str(error)
        raise
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)
        result['owned_px4_stopped'] = process is None or process.poll() is not None
        save(output / 'capture-process.json', result)


def container_capture(output):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tests/px4_native'))
    from run_capture import CONTAINER_POLICY, TOOLS_IMAGE, docker, image, mount
    output = output.resolve()
    output.mkdir(mode=0o700, parents=True, exist_ok=False)
    from pymavlink.dialects.v20 import all as mav
    codec = mav.MAVLink(None, srcSystem=245, srcComponent=190)
    request = codec.command_long_encode(1, 1, mav.MAV_CMD_REQUEST_MESSAGE, 0, 435, 0, 0, 0, 0, 0, 0)
    (output / 'request-input.mavlink').write_bytes(request.pack(codec))
    report = {'result': 'FAIL', 'tooling_image': image(TOOLS_IMAGE, TOOLS_IMAGE),
              'script_sha256': sha(Path(__file__))}
    container = docker('create', *CONTAINER_POLICY, '--label=mavmoos.scope=px4-mode-catalog',
                       '--mount', f'type=volume,src={PACKAGE},dst=/opt/px4,readonly',
                       '--mount', mount(Path(__file__).resolve().parent, '/fixture', True),
                       '--mount', mount(output, '/evidence', False),
                       '--entrypoint=/usr/bin/python3', TOOLS_IMAGE, '-B',
                       '/fixture/capture_native.py', '--inside', '--output', '/evidence').stdout.strip()
    try:
        info, = json.loads(docker('container', 'inspect', container).stdout)
        host = info['HostConfig']
        require(info['Image'] == TOOLS_IMAGE and host['NetworkMode'] == 'none' and
                host['ReadonlyRootfs'] and host['CapDrop'] == ['ALL'] and
                not any(host.get(key) for key in ('CapAdd', 'Devices', 'DeviceRequests', 'PortBindings')) and
                {(item['Destination'], item['RW']) for item in info['Mounts']} ==
                {('/opt/px4', False), ('/fixture', False), ('/evidence', True)},
                'container isolation or mounts differ')
        report['container_id'] = container
        report['network_mode'] = 'none'
        started = docker('start', '-a', container, timeout=25, check=False)
        (output / 'container.log').write_text(started.stdout + started.stderr)
        started.check_returncode()
        report['result'] = 'PASS'
        report['wire_sha256'] = sha(output / 'wire.mavlink')
        report['datagram_index_sha256'] = sha(output / 'datagrams.tsv')
        report['request_sha256'] = sha(output / 'request.mavlink')
    except Exception as error:
        report['error'] = str(error)
        raise
    finally:
        docker('container', 'rm', '-f', container)
        report['container_removed'] = docker('container', 'inspect', container, check=False).returncode != 0
        save(output / 'capture-identity.json', report)
    require(report['container_removed'], 'owned capture container remains')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--inside', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    (capture if args.inside else container_capture)(args.output)
