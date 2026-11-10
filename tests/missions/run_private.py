#!/usr/bin/env python3
"""Compile/run only mission-provider fake-I/O tests; never starts native/MOOS I/O."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys


def run(args):
    repo = Path(__file__).resolve().parents[2]
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=True)
    headers = Path(args.mavlink_dir).resolve()
    compiler = args.cxx
    commands = []
    base = [compiler, '-std=c++17', '-Wall', '-Wextra', '-Wpedantic',
            '-I', str(repo / 'src/libmavmoos_runtime'), '-isystem', str(headers)]
    if platform.system() == 'Darwin':
        base += ['-isysroot', '/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk']
    if args.sanitize:
        base += ['-fsanitize=address,undefined', '-fno-omit-frame-pointer']
    source = repo / 'src/plugins/MissionPlugin.cpp'
    tests = repo / 'tests/missions/MissionPluginTests.cpp'
    executable = out / 'mission_provider_tests'
    module = out / 'mavmoos_missions.so'
    env = dict(os.environ)
    if args.sanitize:
        env['ASAN_OPTIONS'] = 'detect_leaks=0'
    def command(name, argv):
        commands.append({'name': name, 'argv': argv})
        with (out / f'{name}.log').open('w') as log:
            result = subprocess.run(argv, cwd=repo, env=env, stdout=log, stderr=subprocess.STDOUT)
        commands[-1]['exit'] = result.returncode
        if result.returncode:
            print((out / f'{name}.log').read_text()[-10000:])
            raise RuntimeError(f'{name} failed')
    try:
        command('build-direct', base + [str(tests), str(source), '-o', str(executable)])
        command('build-module', base + (['-bundle'] if platform.system() == 'Darwin' else ['-shared', '-fPIC']) + [str(source), '-o', str(module)])
        command('direct-tests', [str(executable)])
        command('module-tests', [str(executable), str(module)])
        result = {'result': 'PASS', 'scope': 'private actual-provider fake-I/O/direct+DSO tests only; no MOOS/native/hardware',
                  'sanitizers': 'address,undefined; detect_leaks=0' if args.sanitize else 'none',
                  'artifacts': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (executable, module)},
                  'sources': {str(p.relative_to(repo)): hashlib.sha256(p.read_bytes()).hexdigest()
                              for p in (source, tests, repo / 'src/libmavmoos_runtime/Plugin.h')}}
    except Exception as error:
        result = {'result': 'FAIL', 'error': str(error)}
    (out / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
    (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    return int(result['result'] != 'PASS')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--mavlink-dir', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--cxx', default='/Library/Developer/CommandLineTools/usr/bin/clang++' if platform.system() == 'Darwin' else 'c++')
    parser.add_argument('--sanitize', action='store_true')
    raise SystemExit(run(parser.parse_args()))
