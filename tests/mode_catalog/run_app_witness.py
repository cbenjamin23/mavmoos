#!/usr/bin/env python3
"""Replay pinned PX4 mode and location wire and exercise the selected Remote ID route."""

import argparse
from contextlib import ExitStack
import hashlib
import json
import os
from pathlib import Path
import re
import socket
import struct
import subprocess
import sys
import time

from pymavlink.dialects.v20 import all as mav
from pymavlink.generator.mavcrc import x25crc

from capture_native import PIN, PX4_SHA, frames, read_request, require, save, sha

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tests'))
import owned_processes
from imu.run_native import live_app
from observer_matrix.run_loopback import audit, complete_capture, mission_text, reserve, start, take_egress

MODE_KEYS = ('MAVMOOS_MODE_CATALOG', 'MAVMOOS_CURRENT_MODE')
RID_KEYS = ('MAVMOOS_REMOTE_ID_IDENTITY', 'MAVMOOS_REMOTE_ID_LOCATION', 'MAVMOOS_REMOTE_ID_HEALTH')
STOCK = '023fb54e00692611ecee2cd6f1138d10eec2127e'


def replay_input(path):
    identity = json.loads((path / 'capture-identity.json').read_text())
    capture = json.loads((path / 'capture-process.json').read_text())
    require(identity['result'] == capture['result'] == 'PASS' and identity['container_removed'] and
            identity['network_mode'] == 'none' and capture['owned_px4_stopped'] and
            capture['firmware_source'] == PIN and capture['px4_binary_sha256'] == PX4_SHA and
            capture['request_count'] == 1, 'native capture identity differs')
    require((path / 'request.mavlink').read_bytes() == read_request(path), 'native read request differs')
    wire = (path / 'wire.mavlink').read_bytes()
    require(hashlib.sha256(wire).hexdigest() == identity['wire_sha256'] and
            sha(path / 'datagrams.tsv') == identity['datagram_index_sha256'] and
            sha(path / 'request.mavlink') == identity['request_sha256'], 'native capture digest differs')
    datagrams = []
    offset = 0
    for line in (path / 'datagrams.tsv').read_text().splitlines():
        stamp, start_offset, length = line.split()
        require(int(start_offset) == offset and int(length) > 0, 'invalid datagram index')
        stamp = float(stamp)
        require(not datagrams or datagrams[-1][0] <= stamp, 'native acquisition time regressed')
        datagrams.append((stamp, wire[offset:offset + int(length)]))
        offset += int(length)
    require(offset == len(wire) and len(datagrams) == capture['datagrams'], 'native capture bytes differ')
    available, current, locations = {}, set(), set()
    for _, packet in datagrams:
        for msgid, system, component, body, frame in frames(packet):
            if (system, component) != (1, 1):
                continue
            if msgid in (435, 436):
                extra = {435: 134, 436: 193}[msgid]
                checksum = x25crc(frame[1:-2] + bytes([extra])).crc
                require(frame[0] == 253 and frame[2] == 0 and checksum == int.from_bytes(frame[-2:], 'little'),
                        'invalid CRC for native mode frame')
            if msgid == 435:
                custom, properties, count, index, standard, label, seq = struct.unpack(
                    '<IIBBB35sB', body.ljust(47, b'\0'))
                require(count == capture['available_modes'] and index not in available and seq == 0,
                        'unexpected native table identity or coverage')
                available[index] = dict(index=index, custom_mode=custom, properties=properties, standard_mode=standard,
                                        name=label.partition(b'\0')[0].decode('ascii') or None)
            if msgid == 436:
                active, intended, standard = struct.unpack('<IIB', body.ljust(9, b'\0'))
                current.add((active, intended or None, standard or None))
            if msgid == 12901:
                location = mav.MAVLink(None).parse_char(frame)
                require(location is not None, 'native Remote ID wire decode failed')
                locations.add((location.latitude / 1e7, location.longitude / 1e7, location.altitude_geodetic))
    require(len(available) == capture['available_modes'] and current and locations,
            'native capture lacks mode or location evidence')
    return datagrams, [available[key] for key in sorted(available)], current, locations


def remote_fixture():
    fc = mav.MAVLink(None, srcSystem=1, srcComponent=1)
    rid = mav.MAVLink(None, srcSystem=1, srcComponent=236)
    other = mav.MAVLink(None, srcSystem=1, srcComponent=237)
    foreign = mav.MAVLink(None, srcSystem=2, srcComponent=236)

    def encoded(codec, message):
        data = message.pack(codec)
        codec.seq = (codec.seq + 1) % 256
        return data

    def hb():
        return encoded(fc, fc.heartbeat_encode(2, 12, 1, 50593792, 3))

    def ident(codec, text):
        return encoded(codec, codec.open_drone_id_basic_id_encode(1, 236, [0] * 20, 1, 2,
                         list(text.ljust(20, b'\0'))))

    def loc(codec, lat, lon):
        return encoded(codec, codec.open_drone_id_location_encode(
            1, 236, [0] * 20, 1, 36100, 25500, 6300, lat, lon, -1000, 32., 0, -1000, 0, 0, 0, 0, 65535., 0))

    def health():
        return encoded(rid, rid.open_drone_id_arm_status_encode(1, b'fixture confidential health'))

    timeline = [(0., ident(rid, b'PREHEARTBEAT')), (.23, hb()),
                (.38, ident(fc, b'PRIMARY-FOREIGN')), (.42, ident(other, b'OTHER-COMPONENT')),
                (.45, ident(foreign, b'FOREIGN-SYSTEM')), (.55, loc(other, 100, 500)),
                (.62, ident(rid, b'SITL-SECRET-ID')), (.64, health()),
                (.66, loc(rid, 473977436, 85455942)), (.89, hb()),
                (.95, ident(rid, b'SITL-SECRET-ID-2')), (1.05, loc(rid, 473977437, 85455942))]
    for i in range(1, 8):
        at = 1.15 + .18 * i
        if i % 2:
            timeline.append((at, hb()))
        timeline.append((at + .02, loc(rid, 473977437 + i, 85455942)))
    return timeline


def app_witness(out, build, moos, probe, datagrams, plugins, component, watched, modeled):
    out.mkdir(mode=0o700)
    processes = []
    report = {'result': 'FAIL', 'component': component, 'scope': 'modeled SITL source' if modeled else 'exact native wire replay',
              'app_egress_datagrams': 0}
    log = out / 'probe.log'
    try:
        with ExitStack() as stack:
            db = stack.enter_context(reserve(socket.SOCK_STREAM))
            receiver = stack.enter_context(reserve(socket.SOCK_DGRAM))
            peer = stack.enter_context(reserve(socket.SOCK_DGRAM))
            peer.setblocking(False)
            db_port, bridge_port, peer_port = [item.getsockname()[1] for item in (db, receiver, peer)]
            report['ports'] = {'moos': db_port, 'app': bridge_port, 'peer': peer_port}
            config = mission_text(db_port, bridge_port, peer_port, 12, plugins)
            config = re.sub(r'^  plugin\.odometry\..*\n', '', config, flags=re.M)
            config = config.replace('MSBetweenLaunches = 50', 'MSBetweenLaunches = 650')
            settings = f'  plugin_directory = {build / "plugins"}\n  remote_id_component = {component}\n'
            if modeled:
                settings += '  plugin.remote_id.sample_timeout = 0.8\n  plugin.remote_id.publish_interval = 0.08\n'
            config = config.replace('  simulation_only = false', settings + '  simulation_only = false')
            mission = out / 'observers.moos'
            mission.write_text(config)
            env = dict(os.environ)
            env['PATH'] = os.pathsep.join((str(build / 'bin'), str(moos / 'bin'), env.get('PATH', '')))
            db.close()
            receiver.close()
            observer = start([str(probe), str(db_port), '11' if modeled else '16',
                              *(f'--watch={name}' for name in watched)], out, env, log.name)
            processes.append(observer)
            antler = start([str(moos / 'bin/pAntler'), str(mission)], out, env, 'pantler.log')
            processes.append(antler)
            output = stack.enter_context((out / 'app-egress.mavlink').open('xb'))
            incoming = stack.enter_context((out / 'fixture-wire.mavlink').open('xb'))
            offsets = stack.enter_context((out / 'fixture-datagrams.tsv').open('x'))

            def drain():
                report['app_egress_datagrams'] += take_egress(peer, bridge_port, output)
                require(antler.poll() is None, 'pAntler exited early')

            until = time.monotonic() + 5
            while b'CAPTURE\tMAVMOOS_RUNTIME_STATUS\t' not in complete_capture(log):
                drain()
                require(time.monotonic() < until and observer.poll() is None, 'app readiness timeout')
                time.sleep(.025)
            app, socket_inventory = live_app(owned_processes, antler, mission, bridge_port)
            report['app_process'] = app
            (out / 'app-udp.txt').write_text(socket_inventory)
            started = time.monotonic()
            for stamp, payload in datagrams:
                at = stamp - datagrams[0][0]
                time.sleep(max(0, started + at - time.monotonic()))
                offsets.write(f'{at:.9f}\t{incoming.tell()}\t{len(payload)}\n')
                incoming.write(payload)
                require(peer.sendto(payload, ('127.0.0.1', bridge_port)) == len(payload), 'partial fixture datagram')
                drain()
            report['received_fixture_datagrams'] = len(datagrams)
            until = time.monotonic() + 13
            while observer.poll() is None:
                drain()
                require(time.monotonic() < until, 'observer shutdown timeout')
                time.sleep(.03)
            drain()
            live_app(owned_processes, antler, mission, bridge_port, app)
            require(observer.returncode == 0 and report['app_egress_datagrams'] == 0, 'probe failure or app egress')
        samples, modules = audit(log, plugins, build / 'plugins')
        report.update(result='PASS', loaded_modules=modules)
        return samples, report
    finally:
        report['remaining_owned_processes'] = [
            row for process in reversed(processes) for row in owned_processes.stop_process_tree(process)]
        save(out / 'app-process.json', report)
        require(not report['remaining_owned_processes'], 'owned app or probe remains')


def audit_native(samples, expected_table, current, locations):
    table_rows = samples[MODE_KEYS[0]]
    completed = [row for row in table_rows if row['complete'] and row['metadata']['fresh']]
    require(completed and all(row['entries'] == expected_table and not row['sequence_bound'] for row in completed),
            'complete app catalog differs from the 27 indexed native rows')
    mode_rows = [row for row in samples[MODE_KEYS[1]] if row['data'] and row['data']['source'] == 'CURRENT_MODE']
    require(mode_rows and all((row['data']['active_custom_mode'], row['data']['intended_custom_mode'],
                              row['data']['standard_mode']) in current for row in mode_rows),
            'current/intended app modes differ from native wire')
    location_rows = [row for row in samples[RID_KEYS[1]] if row['metadata']['fresh']]
    require(location_rows and all((row['data']['latitude_deg'], row['data']['longitude_deg'],
                                  row['data']['wgs84_altitude_m']) in locations for row in location_rows),
            'app Remote ID location differs from native wire')
    require(not table_rows[-1]['entries'] and all(not samples[key][-1]['metadata']['present'] for key in
            (MODE_KEYS[1], RID_KEYS[1])) and
            all(not row['metadata']['present'] for key in (RID_KEYS[0], RID_KEYS[2]) for row in samples[key]),
            'native silence did not clear evidence or unsupported data was inferred')
    return {'available_modes': len(expected_table), 'matched_mode_samples': len(mode_rows),
            'matched_location_samples': len(location_rows), 'cleared_after_silence': True}


def audit_modeled(samples, log):
    for secret in ('SITL-SECRET-ID', 'fixture confidential health', 'PRIMARY-FOREIGN',
                   'PREHEARTBEAT', 'FOREIGN-SYSTEM', 'OTHER-COMPONENT'):
        require(secret not in log.read_text(), 'sensitive Remote ID content leaked to MOOS')
    identity, location, health = (samples[key] for key in RID_KEYS)
    for rows in (identity, location, health):
        present = [row for row in rows if row['metadata']['present']]
        require(present and all((row['metadata']['system_id'], row['metadata']['component_id']) == (1, 236)
                                for row in present) and not rows[-1]['metadata']['present'],
                'selected Remote ID source or cleanup differs')
    generations = [row['data']['identity_generation'] for row in identity if row['data']]
    require(set(generations) == {1, 2} and generations == sorted(generations),
            'remote or reordered identity polluted the redacted generation')
    require(any(row['data'] and row['data']['reported_ready'] is False and row['data']['error_present']
                for row in health), 'transmitter health missing')
    last_health = next(row['metadata']['effective_at'] for row in reversed(health) if row['metadata']['present'])
    last_location = next(row['metadata']['effective_at'] for row in reversed(location) if row['metadata']['present'])
    require(last_location - last_health > 1 and any(row['metadata']['present'] and
            not row['metadata']['fresh'] and row['data'] is None for row in health), 'health age renewed with location')
    return {'identity_generations': [1, 2], 'health_expires_while_location_continues': True,
            'redaction_verified': True, 'cleared_after_silence': True}


def run(args):
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False, mode=0o700)
    build, moos = args.build.resolve(), args.moos_root.resolve()
    probe = args.probe.resolve() if args.probe else build / 'bin/observer_matrix_probe'
    app_source = Path(re.search(r'^CMAKE_HOME_DIRECTORY:INTERNAL=(.*)$',
                               (build / 'CMakeCache.txt').read_text(), re.M)[1]).resolve()
    revision = lambda path: subprocess.check_output(['git', '-C', str(path), 'rev-parse', 'HEAD'], text=True).strip()
    report = {'result': 'FAIL', 'app_checkout_at_start': revision(app_source), 'stock_moos_revision': revision(moos)}
    try:
        require(report['stock_moos_revision'] == STOCK, 'stock MOOS dependency differs')
        files = [probe, build / 'bin/iMAVLink', *(build / 'plugins' / f'mavmoos_{module}.so'
                  for module in ('telemetry', 'mode_catalog', 'remote_id')),
                 *(moos / 'bin' / name for name in ('MOOSDB', 'pAntler', 'pHelmIvP'))]
        hashes = {str(path): sha(path) for path in files}
        report['artifact_sha256'] = hashes
        datagrams, expected, current, locations = replay_input(args.capture.resolve())
        report['native_capture_sha256'] = sha(args.capture / 'capture-identity.json')
        samples, provenance = app_witness(out / 'native-replay', build, moos, probe, datagrams,
                                          ('telemetry', 'mode_catalog', 'remote_id'), 1, MODE_KEYS + RID_KEYS, False)
        report['native_replay'] = {**audit_native(samples, expected, current, locations), **provenance}
        samples, provenance = app_witness(out / 'remote-sidecar', build, moos, probe, remote_fixture(),
                                          ('telemetry', 'remote_id'), 236, RID_KEYS, True)
        report['remote_id_modeled'] = {**audit_modeled(samples, out / 'remote-sidecar/probe.log'), **provenance}
        require(hashes == {str(path): sha(path) for path in files}, 'app/stock artifacts changed during witness')
        report['result'] = 'PASS'
    except Exception as error:
        report['error'] = str(error)
        raise
    finally:
        save(out / 'result.json', report)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--moos-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--probe', type=Path)
    run(parser.parse_args())
