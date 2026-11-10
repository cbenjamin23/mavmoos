#!/usr/bin/env python3
"""Build one manifest-selected owned native executable; never launch it."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

from manifest import file_sha, make_attestation, require, source_identity, validate_manifest


def build_native(manifest_path, source, output, python, jobs=4):
    manifest_path, source, output = [Path(p).resolve() for p in (manifest_path, source, output)]
    manifest = validate_manifest(manifest_path)
    require(not output.exists(), 'refusing to replace build evidence')
    before = source_identity(manifest, source)
    require(not (source / manifest['artifact_path']).exists(), 'refusing to rebuild an existing native artifact')
    output.mkdir(parents=True)
    env = dict(os.environ)
    wrapper = None
    clt = Path('/Library/Developer/CommandLineTools/usr/bin')
    if sys.platform == 'darwin' and (clt / 'clang++').is_file():
        env['CC'], env['CXX'] = str(clt / 'clang'), str(clt / 'clang++')
        env['SDKROOT'] = '/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk'
        # Older source generators invoke gcc by name rather than honoring CC.
        env['PATH'] = str(clt) + os.pathsep + env.get('PATH', '')
        # gcc's system/CLT shim can request an unrelated full-Xcode license.
        # Use this owned compiler alias for generators which spell gcc literally.
        wrappers = output / 'toolchain';wrappers.mkdir()
        wrapper = wrappers / 'gcc'
        wrapper.write_text('#!/bin/sh\nexec /Library/Developer/CommandLineTools/usr/bin/clang "$@"\n')
        wrapper.chmod(0o755)
        env['PATH'] = str(wrappers) + os.pathsep + env['PATH']
    # Resolving a venv's interpreter symlink escapes its site-packages context.
    interpreter = Path(python).absolute()
    env['PATH'] = str(interpreter.parent) + os.pathsep + env.get('PATH', '')
    commands = [[str(interpreter), '-m', 'pip', 'freeze'],
                [str(interpreter), 'waf', 'configure', '--board', 'sitl', '--no-submodule-update'],
                [str(interpreter), 'waf', 'copter', '-j', str(jobs)]]
    report = {'schema': 1, 'result': 'FAIL', 'manifest_sha256': file_sha(manifest_path), 'source_identity': before,
              'commands': [], 'environment': {key: env[key] for key in ('CC', 'CXX', 'SDKROOT') if key in env},
              'builder_sha256': file_sha(__file__), 'scope': 'recorded native compilation only; executable never launched'}
    if wrapper is not None:
        report['environment'].update(gcc_alias=str(wrapper),gcc_alias_sha256=file_sha(wrapper))
    try:
        for index, command in enumerate(commands, 1):
            log = output / f'command-{index:02}.log'
            with log.open('w') as stream:
                result = subprocess.run(command, cwd=source, env=env, stdout=stream, stderr=subprocess.STDOUT)
            report['commands'].append({'argv': command, 'exit': result.returncode, 'log': log.name, 'log_sha256': file_sha(log)})
            require(result.returncode == 0, 'native build command failed: ' + log.name)
        require(source_identity(manifest, source) == before, 'original source changed during build')
        report['native_sha256'] = file_sha(source / manifest['artifact_path']);report['result'] = 'PASS'
    finally:
        (output / 'build-record.json').write_text(json.dumps(report, indent=2) + '\n')
    attestation = make_attestation(manifest_path, source, output / 'build-record.json')
    (output / 'native-attestation.json').write_text(json.dumps(attestation, indent=2) + '\n')
    return {'result': 'PASS', 'native_sha256': report['native_sha256'], 'attestation': str(output / 'native-attestation.json')}


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for key in ('manifest', 'source', 'output', 'python'):
        p.add_argument('--' + key, type=Path, required=True)
    p.add_argument('--jobs', type=int, default=4)
    a = p.parse_args()
    try:
        require(1 <= a.jobs <= 8, 'bounded jobs must be1..8')
        print(json.dumps(build_native(a.manifest, a.source, a.output, a.python, a.jobs), indent=2))
    except (OSError, ValueError, KeyError, TypeError, subprocess.CalledProcessError) as error:
        raise SystemExit(str(error))
