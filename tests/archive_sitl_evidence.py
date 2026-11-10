#!/usr/bin/env python3
"""Archive exact local test results and source/binary provenance, without executables."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import zipfile


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--build', required=True)
    parser.add_argument('--sanitized', required=True)
    parser.add_argument('--moos-root', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    build, sanitized, moos, output = [Path(p).resolve() for p in [args.build, args.sanitized, args.moos_root, args.output]]
    output.mkdir(parents=True, exist_ok=True)
    final = json.loads((build / 'evidence/copter-sitl/result.json').read_text())
    if final['result'] != 'PASS':
        raise RuntimeError('final combined native SITL result is not PASS')
    if digest(build / 'bin/iMAVLink') != final['app_sha256']:
        raise RuntimeError('current application does not match the tested executable')
    if '100% tests passed' not in (build / 'evidence/ctest-final.log').read_text():
        raise RuntimeError('final CTest suite has not passed')
    shutil.copy2(build / 'evidence/copter-sitl/result.json', output / 'copter-sitl-result.json')
    selections = ['sitl-attempt-01', 'sitl-attempt-02', 'sitl-attempt-03', 'sitl-attempt-04',
                  'sitl-attempt-05', 'sitl-attempt-06', 'copter-sitl', 'stock-helm', 'helm-stock-source-check',
                  'shadow', 'shadow-live', 'shadow-live-02', 'shadow-live-03', 'normal', 'invalid_config']
    manifest = {'date': '2026-11-03', 'scope': 'owned native ArduCopter simulation only', 'files': [], 'sources': [], 'binaries': []}
    bundle = output / 'execution-evidence.zip'
    with zipfile.ZipFile(bundle, 'w', zipfile.ZIP_DEFLATED, compresslevel=9) as archive:
        for selection in selections:
            folder = build / 'evidence' / selection
            if not folder.exists():
                continue
            for path in sorted(folder.rglob('*')):
                if not path.is_file() or path.suffix.lower() in ['.pyc']:
                    continue
                name = str(path.relative_to(build / 'evidence'))
                archive.write(path, name)
                manifest['files'].append({'path': name, 'bytes': path.stat().st_size, 'sha256': digest(path)})
        for name, path in [('ctest-final.log', build / 'evidence/ctest-final.log'),
                           ('ctest-detail.log', build / 'Testing/Temporary/LastTest.log'),
                           ('asan-ubsan.log', sanitized / 'sanitizers-localhost.log'),
                           ('asan-ubsan-detail.log', sanitized / 'Testing/Temporary/LastTest.log'),
                           ('sanitizer-sandbox-failure.log', sanitized / 'sanitizers.log')]:
            archive.write(path, name)
            manifest['files'].append({'path': name, 'bytes': path.stat().st_size, 'sha256': digest(path)})
            shutil.copy2(path, output / name)
    manifest['bundle'] = {'file': bundle.name, 'bytes': bundle.stat().st_size, 'sha256': digest(bundle)}
    for folder in ['src', 'tests', 'missions/copter_sitl']:
        for path in sorted((root / folder).rglob('*')):
            if path.is_file() and (path.suffix in ['.cpp', '.h', '.py', '.sh'] or path.name in ['CMakeLists.txt', 'meta_vehicle.moos', 'meta_vehicle.bhv']):
                manifest['sources'].append({'path': str(path.relative_to(root)), 'sha256': digest(path)})
    for path in [root / 'CMakeLists.txt', root / 'AGENTS.md']:
        manifest['sources'].append({'path': str(path.relative_to(root)), 'sha256': digest(path)})
    for path in [build / 'bin/iMAVLink', moos / 'bin/pHelmIvP', Path.home() / 'moos-ivp-uav-base/lib/libBHV_ConstantAltitude.dylib'] + list((build / 'plugins').glob('mavmoos_*')):
        if path.is_file():
            manifest['binaries'].append({'path': str(path), 'sha256': digest(path), 'bytes': path.stat().st_size})
    manifest['moos_source_status'] = subprocess.check_output(['git', '-C', str(moos), 'status', '--porcelain'], text=True)
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    # Read back compressed artifacts and verify their hashes against the manifest.
    with zipfile.ZipFile(bundle) as archive:
        for entry in manifest['files']:
            if hashlib.sha256(archive.read(entry['path'])).hexdigest() != entry['sha256']:
                raise RuntimeError('archive verification failed: ' + entry['path'])
    print(json.dumps({'bundle': manifest['bundle'], 'archived_files': len(manifest['files']),
                      'source_files': len(manifest['sources']), 'moos_source_clean': not manifest['moos_source_status']}))


if __name__ == '__main__':
    main()
