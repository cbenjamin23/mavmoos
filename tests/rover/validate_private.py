#!/usr/bin/env python3
"""Build/test only fake-I/O Rover components. Never starts MOOS or a simulator."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mavlink', required=True, help='Pinned c_library_v2 header root')
    parser.add_argument('--output', required=True, help='Fresh build/evidence directory')
    parser.add_argument('--moos-root', help='Optionally compile the native workflow MOOS probe')
    parser.add_argument('--cxx', default='c++')
    parser.add_argument('--sanitizers', action='store_true')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    records = []

    def run(name, command, environment=None):
        with (out / (name + '.log')).open('w') as log:
            answer = subprocess.run(command, cwd=root, stdout=log, stderr=subprocess.STDOUT, env=environment)
        records.append({'name': name, 'command': list(map(str, command)), 'exit': answer.returncode,
                        'log_sha256': hashlib.sha256((out / (name + '.log')).read_bytes()).hexdigest()})
        (out / 'results.json').write_text(json.dumps({'result': 'PASS' if answer.returncode == 0 else 'FAIL', 'commands': records}, indent=2) + '\n')
        if answer.returncode:
            raise RuntimeError(name + ' failed; see ' + str(out / (name + '.log')))

    common = [args.cxx, '-std=c++17', '-Wall', '-Wextra', '-Wpedantic']
    if platform.system() == 'Darwin':
        common += ['-isysroot', '/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk']
    includes = ['-I', str(root / 'src/libmavmoos_rover'), '-I', str(root / 'src/libmavmoos_runtime'),
                '-I', str(root / 'src/libmavmoos_control'), '-isystem', str(Path(args.mavlink).resolve())]
    sources = [str(root / 'src/libmavmoos_rover' / name) for name in ['ArduRoverBackend.cpp', 'RoverCommands.cpp']]
    sources.append(str(root / 'src/libmavmoos_runtime/ControlReplayJournal.cpp'))
    variants = [('normal', [])]
    if args.sanitizers:
        variants.append(('sanitizers', ['-fsanitize=address,undefined', '-fno-omit-frame-pointer']))
    run('compiler-version', [args.cxx, '--version'])
    for name, flags in variants:
        compiler = common + flags + includes
        backend, control, module = (out / (name + suffix) for suffix in ['_backend_tests', '_control_tests', '_ardurover_control.so'])
        run(name + '-backend-build', compiler + sources + [str(root / 'tests/rover/RoverBackendTests.cpp'), '-o', str(backend)])
        run(name + '-module-build', compiler + ['-fPIC', '-shared'] + sources +
            [str(root / 'src/plugins/ArduRoverControlPlugin.cpp'), str(root / 'src/libmavmoos_control/HelmAdapter.cpp'), '-o', str(module)])
        run(name + '-control-build', compiler + [str(root / 'src/libmavmoos_runtime/PluginHost.cpp'),
            str(root / 'tests/rover/RoverControlTests.cpp'), '-o', str(control)] + ([] if platform.system() == 'Darwin' else ['-ldl']))
        environment = dict(os.environ)
        if flags:
            environment['ASAN_OPTIONS'] = 'detect_leaks=0'
        run(name + '-backend-tests', [str(backend)], environment)
        run(name + '-control-tests', [str(control), str(module)], environment)
    if args.moos_root:
        moos = Path(args.moos_root).resolve()
        moos_base = moos / 'MOOS/MOOSCore/Core/libMOOS'
        moos_includes = []
        for suffix in ['include', 'Comms/include', 'Utils/include', 'Thirdparty/PocoBits/include']:
            moos_includes += ['-I', str(moos_base / suffix)]
        moos_includes += ['-I', str(moos / 'build/MOOS/MOOSCore/Core/libMOOS/include')]
        run('workflow-probe-build', common + moos_includes + [str(root / 'tests/rover/RoverWorkflowProbe.cpp'),
            str(moos / 'build/MOOS/MOOSCore/lib/libMOOS.a'), '-pthread', '-o', str(out / 'rover_workflow_probe')])
    print(json.dumps({'result': 'PASS', 'output': str(out), 'variants': [name for name, _ in variants],
                      'evidence': 'fake I/O and local DSO loading; no simulator started'}, indent=2))


if __name__ == '__main__':
    main()
