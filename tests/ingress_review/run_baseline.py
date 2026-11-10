#!/usr/bin/env python3
"""Reproduce the archived read-time policy with owned loopback and fake egress."""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mavlink-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--compiler', default='/Library/Developer/CommandLineTools/usr/bin/clang++' if platform.system() == 'Darwin' else 'c++')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    stage = output / 'source'
    source_files = []
    directories = ['src/libmavmoos_rx', 'src/libmavmoos_runtime', 'src/libmavmoos_egress']
    names = subprocess.check_output(['git', '-C', str(root), 'ls-tree', '-r', '--name-only', '943ff39', *directories, 'tests/Fixtures.h'], text=True).splitlines()
    for name in names:
        path = stage / name
        path.parent.mkdir(parents=True, exist_ok=True)
        data = subprocess.check_output(['git', '-C', str(root), 'show', '943ff39:' + name])
        path.write_bytes(data)
        source_files.append({'path': name, 'sha256': hashlib.sha256(data).hexdigest()})
    command = [args.compiler, '-std=c++17', '-O0', '-pthread', '-isystem', str(args.mavlink_dir.resolve())]
    if platform.system() == 'Darwin':
        command += ['-isysroot', '/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk']
    for include in ['tests', *directories]:
        command += ['-I', str(stage / include)]
    diagnostic = root / 'tests/ingress_review/BacklogBaseline.cpp'
    sources = [str(p) for name in directories for p in (stage / name).glob('*.cpp')]
    executable = output / 'backlog-baseline'
    command += [str(diagnostic), *sources, '-o', str(executable)]
    if platform.system() == 'Linux':
        command += ['-ldl']
    compile_result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    (output / 'compile.log').write_text(compile_result.stdout)
    report = {'baseline_commit': '943ff39', 'platform': platform.platform(), 'hardware': False,
              'layer': 'owned loopback input + fake outbound I/O', 'compile_command': command,
              'compile_exit': compile_result.returncode, 'sources': source_files,
              'diagnostic_sha256': hashlib.sha256(diagnostic.read_bytes()).hexdigest()}
    if compile_result.returncode == 0:
        result = subprocess.run([str(executable)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=10)
        (output / 'diagnostic.log').write_text(result.stdout)
        report['diagnostic_exit'] = result.returncode
        report['executable_sha256'] = hashlib.sha256(executable.read_bytes()).hexdigest()
    (output / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k not in ['sources', 'compile_command']}, indent=2))
    return report.get('diagnostic_exit', report['compile_exit'])


if __name__ == '__main__':
    raise SystemExit(main())
