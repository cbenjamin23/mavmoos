#!/usr/bin/env python3
"""Capture native PX4 datagrams using two local, immutable, offline images."""

import argparse
import json
from pathlib import Path
import re
import shutil
import subprocess

from audit import audit_capture, dump, sha


FIXTURE = Path(__file__).resolve().parent
IMAGE = 'px4io/px4-sitl@sha256:b61cc6dd50b97da78fb27b4d399ff892d8fd51656ae848f72cba2d723926b962'
IMAGE_ID = 'sha256:2a4a4cc5e7e4f1ad80a1ae20750a30c07240cf63d4464f67588d4e3910d1ba38'
TOOLS_IMAGE = 'sha256:644154814d3e5c162392b229f26bd68e3ad55c44acf863e78ee83e7568cbed85'
CONTAINER_POLICY = [
    '--pull=never', '--network=none', '--read-only', '--cap-drop=ALL',
    '--security-opt=no-new-privileges', '--pids-limit=96', '--cpus=2', '--memory=512m',
    '--tmpfs=/tmp:rw,nosuid,nodev,size=64m',
]


def docker(*args, timeout=15, check=True):
    return subprocess.run(['docker', *args], text=True, capture_output=True, timeout=timeout, check=check)


def image(ref, expected_id):
    info, = json.loads(docker('image', 'inspect', ref).stdout)
    if (info['Id'], info['Os'], info['Architecture']) != (expected_id, 'linux', 'arm64'):
        raise ValueError('required local ARM64 image identity differs')
    return {'id': info['Id'], 'os': info['Os'], 'architecture': info['Architecture'],
            'repository_digests': info['RepoDigests']}


def mount(path, target, read_only):
    if ',' in str(path):
        raise ValueError('mount paths may not contain a comma')
    return f'type=bind,src={path},dst={target}' + (',readonly' if read_only else '')


def container_policy(info, native, fixture):
    host = info['HostConfig']
    mounts = {(item['Type'], item['Destination']): (Path(item['Source']).resolve(), item['RW'])
              for item in info['Mounts']}
    expected = {('bind', '/evidence'): (native, True), ('bind', '/fixture'): (fixture, False)}
    if (info['Image'] != IMAGE_ID or host['NetworkMode'] != 'none' or
            not host['ReadonlyRootfs'] or host['Privileged'] or host['CapDrop'] != ['ALL'] or
            'no-new-privileges' not in host['SecurityOpt'] or mounts != expected or
            any(host.get(key) for key in ('CapAdd', 'Devices', 'DeviceRequests', 'PortBindings', 'VolumesFrom'))):
        raise ValueError('PX4 container differs from the isolated capture policy')
    return {'image_id': info['Image'], 'network_mode': host['NetworkMode'],
            'read_only_root': host['ReadonlyRootfs'], 'drop_capabilities': host['CapDrop'],
            'security_options': host['SecurityOpt'], 'mounts': info['Mounts']}


def run(output):
    out = output.resolve()
    out.mkdir(mode=0o700, parents=True, exist_ok=False)
    fixture, native = out / 'inputs', out / 'native'
    fixture.mkdir(mode=0o700)
    native.mkdir(mode=0o700)
    for name in ('capture.sh', 'capture_udp.c'):
        shutil.copyfile(FIXTURE / name, fixture / name)
    identity = {'firmware_image': image(IMAGE, IMAGE_ID), 'tooling_image': image(TOOLS_IMAGE, TOOLS_IMAGE)}
    compile_command = ['run', '--rm', *CONTAINER_POLICY, '--mount', mount(fixture, '/fixture', False),
                       '--entrypoint=/usr/bin/gcc', TOOLS_IMAGE, '-Wall', '-Wextra', '-Werror',
                       '-std=c11', '-O2', '-static', '/fixture/capture_udp.c', '-o', '/fixture/capture_udp']
    compile_result = docker(*compile_command, timeout=90, check=False)
    (out / 'recorder-build.log').write_text(compile_result.stdout + compile_result.stderr)
    compile_result.check_returncode()
    files = sorted(fixture.iterdir())
    inputs = {item.name: sha(item) for item in files}
    identity['fixture_sha256'] = inputs
    command = [
        'create', *CONTAINER_POLICY, '--label=mavmoos.scope=px4-native-passive',
        '--mount', mount(fixture, '/fixture', True), '--mount', mount(native, '/evidence', False),
        '--env=PX4_SIM_MODEL=sihsim_quadx', '--env=PX4_SIMULATOR=sihsim',
        '--entrypoint=/bin/sh', IMAGE, '/fixture/capture.sh',
    ]
    identity['docker_create'] = command
    container = docker(*command).stdout.strip()
    if not re.fullmatch(r'[a-f0-9]{64}', container):
        raise ValueError('missing exact owned PX4 container ID')
    identity['container_id'] = container
    try:
        info, = json.loads(docker('container', 'inspect', container).stdout)
        identity['container_policy'] = container_policy(info, native, fixture)
        boot = docker('start', '-a', container, timeout=45, check=False)
        (out / 'container.log').write_text(boot.stdout + boot.stderr)
        boot.check_returncode()
        info, = json.loads(docker('container', 'inspect', container).stdout)
        if info['State']['Running'] or info['State']['ExitCode'] != 0:
            raise RuntimeError('PX4 capture container did not exit cleanly')
    finally:
        docker('container', 'rm', '-f', container)
        identity['container_removed'] = docker('container', 'inspect', container, check=False).returncode != 0
        dump(out / 'capture-identity.json', identity)
    if not identity['container_removed'] or inputs != {item.name: sha(item) for item in files}:
        raise RuntimeError('capture teardown or immutable inputs check failed')
    report, _, _ = audit_capture(native)
    dump(out / 'capture-result.json', report)
    print(f"PASS: {sum(report['datagrams'].values())} exact PX4 UDP datagrams in {out}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='new private evidence directory')
    run(parser.parse_args().output)
