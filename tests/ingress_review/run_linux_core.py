#!/usr/bin/env python3
"""Build final pure-core sources in a fresh, network-isolated Linux container."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile
import time


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mavlink-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--image', default='mavmoos-clean-linux:20261103')
    parser.add_argument('--docker', default='docker')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    commands = []
    container = 'mavmoos-ingress-core-' + str(time.time_ns())
    created = False
    result = {'result': 'FAIL', 'scope': 'fresh Linux pure-core build; no MOOS app/SITL/hardware execution',
              'network': 'none', 'container': container, 'image': args.image}

    def run(command, filename=None, required=True, timeout=240):
        commands.append(command)
        completed = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=timeout)
        if filename:
            (output / filename).write_text(completed.stdout)
        if required and completed.returncode:
            raise RuntimeError(f'{filename or command[0]} exited {completed.returncode}')
        return completed

    def execute(command, filename=None, **kwargs):
        return run([args.docker, 'exec', '--user', '65534:65534', container, *command], filename, **kwargs)

    try:
        identity = run([args.docker, 'image', 'inspect', '--format', '{{.Id}}', args.image], 'image-inspect.log')
        result['image_id'] = identity.stdout.strip()
        source_paths = []
        for directory in ['src', 'tests']:
            for path in sorted((root / directory).rglob('*')):
                if path.is_file() and '__pycache__' not in path.parts and (path.suffix in ['.h', '.cpp', '.py', '.cmake'] or path.name == 'CMakeLists.txt'):
                    source_paths.append(path)
        source_paths.append(root / 'packaging/linux/record_elf.py')
        manifest = [{'path': str(path.relative_to(root)), 'sha256': sha(path)} for path in source_paths]
        with tarfile.open(output / 'sources.tar', 'w') as archive:
            for path in source_paths:
                archive.add(path, arcname=str(path.relative_to(root)), recursive=False)
        headers = args.mavlink_dir.resolve()
        with tarfile.open(output / 'mavlink-headers.tar', 'w') as archive:
            for path in sorted(headers.rglob('*.h')):
                archive.add(path, arcname=str(path.relative_to(headers)), recursive=False)
        (output / 'source-manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        result['source_tar_sha256'] = sha(output / 'sources.tar')
        result['mavlink_headers_tar_sha256'] = sha(output / 'mavlink-headers.tar')
        run([args.docker, 'create', '--name', container, '--network', 'none', '--cap-drop', 'ALL',
             '--security-opt', 'no-new-privileges', '--init', args.image, 'sleep', '1800'], 'create.log')
        created = True
        run([args.docker, 'cp', str(output / 'sources.tar'), container + ':/tmp/mavmoos-sources.tar'])
        run([args.docker, 'cp', str(output / 'mavlink-headers.tar'), container + ':/tmp/mavlink-headers.tar'])
        run([args.docker, 'start', container], 'start.log')
        setup = ('mkdir -p /work/source /work/mavlink /work/recipe /work/build /work/evidence && '
                 'tar --no-same-owner -xf /tmp/mavmoos-sources.tar -C /work/source && '
                 'tar --no-same-owner -xf /tmp/mavlink-headers.tar -C /work/mavlink && '
                 'cp /work/source/tests/ingress_review/LinuxCore.cmake /work/recipe/CMakeLists.txt && '
                 'chmod -R a-w /work/source /work/mavlink /work/recipe && '
                 'chmod 777 /work/build /work/evidence')
        run([args.docker, 'exec', container, 'sh', '-c', setup], 'setup.log')
        execute(['id'], 'build-user.log')
        execute(['uname', '-a'], 'uname.log')
        execute(['c++', '--version'], 'compiler.log')
        execute(['cmake', '--version'], 'cmake.log')
        execute(['cmake', '-G', 'Ninja', '-S', '/work/recipe', '-B', '/work/build',
                 '-DREPO_ROOT=/work/source', '-DMAVLINK_ROOT=/work/mavlink', '-DCMAKE_BUILD_TYPE=Debug'], 'configure.log')
        execute(['cmake', '--build', '/work/build', '-j', '4'], 'build.log')
        tests = execute(['ctest', '--test-dir', '/work/build', '--output-on-failure', '-V'], 'ctest.log', required=False)
        result['ctest_exit'] = tests.returncode
        execute(['sh', '-c', 'python3 /work/source/packaging/linux/record_elf.py /work/build/bin/* /work/build/plugins/*.so'], 'elf-artifacts.json')
        execute(['ps', '-axww', '-o', 'pid=,ppid=,args='], 'remaining-processes.log')
        run([args.docker, 'cp', container + ':/work/build/Testing/Temporary/LastTest.log', str(output / 'ctest-detail.log')])
        result['result'] = 'PASS' if tests.returncode == 0 else 'FAIL'
    except Exception as error:
        result['error'] = str(error)
    finally:
        if created:
            removal = run([args.docker, 'rm', '-f', container], 'remove.log', required=False)
            remaining = run([args.docker, 'ps', '-aq', '--filter', 'name=^/' + container + '$'], 'cleanup-verify.log', required=False)
            result['container_removed'] = removal.returncode == 0 and remaining.returncode == 0 and not remaining.stdout.strip()
            if not result['container_removed']:
                result['result'] = 'FAIL'
        (output / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
        (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    return 0 if result['result'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
