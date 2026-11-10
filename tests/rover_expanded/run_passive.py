#!/usr/bin/env python3
"""Run the real app with generated MAVLink on loopback and count every returned datagram."""
import argparse
import json
from pathlib import Path
import socket
import subprocess
import tempfile
import time

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from run_native import (
    HOME, MOOS, PROFILES, catalog_proof, endpoint_cleanup, env_for, evidence_files, generate, git,
    launcher_macros, mission_inventory, port, require, sha, start, stop_process_tree, tracked_pin,
)


def feed(peer, destination, codec, kind, tick, armed):
    base = mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    if armed:
        base |= mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    for message in (
        codec.heartbeat_encode(kind, mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, base, 15, mavlink.MAV_STATE_ACTIVE),
        codec.global_position_int_encode(tick * 100, int(HOME[0] * 1e7), int(HOME[1] * 1e7), 10000, 0, 0, 0, 0, 0),
        codec.ekf_status_report_encode(19, .1, .1, .1, .1, .1),
    ):
        packet = message.pack(codec)
        codec.seq = (codec.seq + 1) & 255
        require(peer.sendto(packet, destination) == len(packet), 'generated frame was not accepted locally')


def run_case(out, moos, build, kind, mode, moos_revision):
    out.mkdir()
    (out / 'logs').mkdir()
    control = out / 'control'
    control.mkdir()
    profile, expected, _, _ = PROFILES[kind]
    sibling = 11 if expected == 10 else 10
    peer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    peer.bind(('127.0.0.1', 0))
    peer.setblocking(False)
    ports = {'db': port(socket.SOCK_STREAM), 'bridge': port(socket.SOCK_DGRAM), 'peer': peer.getsockname()[1]}
    result = {'profile': profile, 'vehicle_profile': profile, 'mode': mode, 'expected_type': expected, 'sent_sibling_type': sibling,
              'ports': ports, 'received_datagrams': 0, 'result': 'FAIL', 'hardware': False,
              'app_sha256': sha(build / 'bin/iMAVLink'), 'stock_helm_sha256': sha(moos / 'bin/pHelmIvP'),
              'stock_moos_revision': moos_revision}
    launch = probe = None
    try:
        env = env_for(moos, build)
        macros = launcher_macros(ports['db'], build, moos, ports['bridge'], ports['peer'], kind)
        macros.update(SIMULATION_ONLY='false', NATIVE_ENABLED='false', SHADOW_MODE='true' if mode == 'SHADOW' else 'false',
                      HELM_ENABLED='true' if mode == 'SHADOW' else 'false')
        generate(moos, out, macros, env)
        mission = out / 'targ_rover.moos'
        launch = start([moos / 'bin/pAntler', mission], out, env, 'pantler')
        probe = start([build / 'bin/rover_expanded_probe', ports['db'], out / 'workflow.jsonl', control, kind, mode],
                      out, env, 'probe')
        codec = mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
        deadline = time.monotonic() + 28
        next_frame = time.monotonic()
        tick, drain = 0, None
        with (out / 'unexpected-egress.bin').open('wb') as capture:
            while time.monotonic() < deadline:
                require(launch.poll() is None, 'owned passive mission exited')
                now = time.monotonic()
                if probe.poll() is not None:
                    drain = drain or now + .8
                    if now >= drain:
                        break
                if now >= next_frame:
                    vehicle = expected if (control / 'expected.request').is_file() else sibling
                    feed(peer, ('127.0.0.1', ports['bridge']), codec, vehicle, tick, mode == 'SHADOW')
                    tick += 1
                    next_frame = now + .1
                try:
                    data, _ = peer.recvfrom(65535)
                except BlockingIOError:
                    time.sleep(.015)
                    continue
                result['received_datagrams'] += 1
                capture.write(data)
        result['generated_frames'] = tick * 3
        result['probe_exit'] = probe.poll()
        require(probe.returncode == 0 and not result['received_datagrams'],
                'passive probe did not pass or generated peer received outbound data; inspect probe.log')
        result['owned_mission_proof'] = mission_inventory(launch, mission, ports['bridge'], moos / 'bin/pHelmIvP')
        result['capabilities_catalog'], result['loaded_modules_sha256'] = catalog_proof(out, build, profile, mode)
        result['result'] = 'PASS'
    except BaseException as error:
        result['error'] = str(error) or type(error).__name__
    finally:
        remaining = []
        for process in (probe, launch):
            try:
                remaining.extend(stop_process_tree(process))
            except Exception as error:
                remaining.append({'pid': process.pid, 'error': str(error)})
        peer.close()
        result['ports_clear'] = endpoint_cleanup(ports)
        result['owned_processes_remaining'] = remaining
        if not all(result['ports_clear'].values()) or remaining:
            result['result'], result['cleanup_error'] = 'FAIL', 'owned endpoint or process remains'
        result['files'] = evidence_files(out)
        (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'result': result['result'], 'mode': mode, 'profile': profile,
                      'error': result.get('error'), 'output': str(out), 'ports_clear': result['ports_clear']}), flush=True)
    return result['result'] == 'PASS'


def main(args):
    moos, build, parent = (Path(getattr(args, key)).resolve() for key in ('moos_root', 'build', 'output_parent'))
    setup = Path(__file__).resolve().parents[2] / 'config/setup.json'
    public_pin = json.loads(setup.read_text())['moos']['revision']
    moos_revision = git(moos, 'rev-parse', 'HEAD')
    require(moos_revision in (MOOS, public_pin), 'generated-peer fixture needs a pinned stock MOOS checkout')
    tracked_pin(moos, moos_revision)
    parent.mkdir(parents=True, exist_ok=True)
    out = Path(tempfile.mkdtemp(prefix='passive-', dir=parent))
    outcomes = []
    for kind in PROFILES:
        for mode in ('RX_ONLY', 'SHADOW'):
            outcomes.append(run_case(out / f'{kind}-{mode.lower()}', moos, build, kind, mode, moos_revision))
    tracked_pin(moos, moos_revision)
    return 0 if all(outcomes) else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('moos-root', 'build', 'output-parent'):
        parser.add_argument('--' + name, required=True)
    raise SystemExit(main(parser.parse_args()))
