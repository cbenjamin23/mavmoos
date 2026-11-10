#!/usr/bin/env python3
"""Build/verify pure ObservationRequests with fake clocks; no network or FC."""
import argparse
import datetime
import gzip
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import zipfile

from verify_trace import negative_controls, parse, verify

REPO = Path(__file__).resolve().parents[2]
INPUTS = ['src/libmavmoos_runtime/ObservationRequests.h', 'src/libmavmoos_runtime/ObservationRequests.cpp',
          'src/libmavmoos_runtime/Plugin.h', 'tests/stream_requests/CMakeLists.txt',
          'tests/stream_requests/StreamRequestTests.cpp', 'tests/stream_requests/run_validation.py',
          'tests/stream_requests/verify_trace.py']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git(*args):
    return subprocess.check_output(['git', '-C', str(REPO), *args], text=True).strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mavlink-dir', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--build-dir', required=True, type=Path)
    parser.add_argument('--compiler', default=shutil.which('clang++'))
    parser.add_argument('--sdk')
    args = parser.parse_args()
    output, build, headers = args.output_dir.resolve(), args.build_dir.resolve(), args.mavlink_dir.resolve()
    if not args.compiler:
        parser.error('provide a C++ compiler')
    if output.exists() or build.exists():
        parser.error('output and build directories must be new; existing evidence/builds are preserved')
    output.mkdir(parents=True)
    build.mkdir(parents=True)
    header_files = sorted(p for p in headers.rglob('*.h') if '.git' not in p.parts)
    if not header_files or not (headers / 'ardupilotmega/mavlink.h').is_file():
        parser.error('MAVLINK_DIR must contain generated ardupilotmega headers')
    header_manifest = {str(p.relative_to(headers)): sha(p) for p in header_files}
    (output / 'generated-headers-sha256.json').write_text(json.dumps(header_manifest, indent=2) + '\n')
    manifest = {'schema': 1, 'created_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
                'repo_commit': git('rev-parse', 'HEAD'), 'worktree_status': git('status', '--short'),
                'scope': 'pure actual C++ helper + fake clocks; no socket/MOOS/FC/native/hardware',
                'platform': platform.platform(), 'python': sys.version, 'compiler': str(args.compiler),
                'mavlink_dir': str(headers), 'inputs': {name: sha(REPO / name) for name in INPUTS},
                'generated_header_count': len(header_files), 'generated_header_manifest_sha256': sha(output / 'generated-headers-sha256.json'),
                'commands': []}
    with zipfile.ZipFile(output / 'tested-sources.zip', 'w', zipfile.ZIP_DEFLATED) as archive:
        for name in INPUTS:
            info = zipfile.ZipInfo(name, (1980, 1, 1, 0, 0, 0))
            info.external_attr = 0o100644 << 16
            archive.writestr(info, (REPO / name).read_bytes(), compress_type=zipfile.ZIP_DEFLATED)
    manifest['tested_sources_sha256'] = sha(output / 'tested-sources.zip')
    failure = None

    def run(command, log, env):
        command = [str(v) for v in command]
        result = subprocess.run(command, cwd=REPO, env=env, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, timeout=55)
        log.write_bytes(result.stdout)
        manifest['commands'].append({'argv': command, 'returncode': result.returncode,
                                     'log': str(log.relative_to(output)), 'log_sha256': sha(log)})
        if result.returncode:
            raise RuntimeError(f'{log}: exit {result.returncode}')
        return result.stdout

    try:
        run([args.compiler, '--version'], output / 'compiler.log', os.environ.copy())
        for mode in ['normal', 'sanitized']:
            dest, target = output / mode, build / mode
            dest.mkdir()
            env = os.environ.copy()
            command = ['cmake', '-G', 'Ninja', '-S', REPO / 'tests/stream_requests', '-B', target,
                       '-DCMAKE_BUILD_TYPE=Debug', f'-DMAVLINK_DIR={headers}', f'-DCMAKE_CXX_COMPILER={args.compiler}']
            if args.sdk:
                command.append(f'-DCMAKE_OSX_SYSROOT={args.sdk}')
            if mode == 'sanitized':
                command.extend(['-DCMAKE_CXX_FLAGS=-fsanitize=address,undefined -fno-omit-frame-pointer',
                                '-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined'])
                env.update(ASAN_OPTIONS='detect_leaks=0:abort_on_error=1', UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
            run(command, dest / 'configure.log', env)
            run(['cmake', '--build', target, '--parallel', '4'], dest / 'build.log', env)
            trace = run([target / 'stream_request_tests'], dest / 'trace.jsonl', env)
            records = parse(trace.decode())
            result = verify(records)
            result['negative_controls_rejected'] = negative_controls(records)
            result['binary_sha256'] = sha(target / 'stream_request_tests')
            run(['ctest', '--test-dir', target, '--output-on-failure'], dest / 'ctest.log', env)
            with gzip.GzipFile(filename=str(dest / 'trace.jsonl.gz'), mode='wb', mtime=0) as compressed:
                compressed.write(trace)
            result['compressed_trace_sha256'] = sha(dest / 'trace.jsonl.gz')
            (dest / 'trace.jsonl').unlink()
            manifest[mode] = result
    except Exception as exc:
        failure = f'{type(exc).__name__}: {exc}'
    manifest['result'], manifest['failure'] = ('fail' if failure else 'pass'), failure
    # Detect source/header edits during build and verification instead of silently
    # attributing output to an earlier snapshot.
    changed = [name for name, expected in manifest['inputs'].items() if sha(REPO / name) != expected]
    changed_headers = [name for name, expected in header_manifest.items() if sha(headers / name) != expected]
    if changed or changed_headers:
        manifest.update(result='fail', changed_inputs=changed, changed_generated_headers=changed_headers)
    (output / 'validation.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps({'result': manifest['result'], 'manifest': str(output / 'validation.json'), 'failure': failure}))
    return 0 if manifest['result'] == 'pass' else 1


if __name__ == '__main__':
    raise SystemExit(main())
