#!/usr/bin/env python3
"""Create an owned source-only firmware tree from exact local Git objects.

The source checkout and its refs/submodules are never modified. The output is
fresh; Git archives bind every original file, including nested submodules.
"""
import argparse
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tarfile

from manifest import canonical, file_sha, require, source_identity, validate_manifest


def export_source(manifest_path, source_root, output):
    manifest_path = Path(manifest_path).resolve()
    profile = validate_manifest(manifest_path)
    source_root, output = Path(source_root).resolve(), Path(output).resolve()
    require(not output.exists(), 'refusing to replace native source export')
    pins = [(Path(), profile['source']['revision'])] + [(Path(item['path']), item['revision']) for item in profile['source']['submodules']]
    # A missing object fails before creating any output or modifying any refs.
    for relative, pin in pins:
        subprocess.run(['git', '-C', str(source_root / relative), 'cat-file', '-e', pin + '^{commit}'], check=True, stdout=subprocess.DEVNULL)
    output.parent.mkdir(parents=True, exist_ok=True)
    commands = [
        ['git', 'clone', '--quiet', '--no-local', '--no-checkout', '--depth=1', source_root.as_uri(), str(output)],
        ['git', '-C', str(output), 'fetch', '--quiet', '--depth=1', 'origin', profile['source']['revision']],
        ['git', '-C', str(output), 'checkout', '--quiet', '--detach', profile['source']['revision']],
    ]
    for command in commands:
        subprocess.run(command, check=True)
    original_files = {}
    for relative, pin in pins:
        data = subprocess.check_output(['git', '-C', str(source_root / relative), 'archive', '--format=tar', pin])
        with tarfile.open(fileobj=io.BytesIO(data)) as archive:
            for member in archive:
                path = Path(member.name)
                require(not path.is_absolute() and '..' not in path.parts, 'unsafe Git archive path')
                name = (relative / path).as_posix()
                if member.isfile():
                    value = archive.extractfile(member).read()
                    require(name not in original_files, 'duplicate source file across modules')
                    original_files[name] = {'type': 'file', 'sha256': hashlib.sha256(value).hexdigest()}
                elif member.issym():
                    require(name not in original_files, 'duplicate source symlink')
                    original_files[name] = {'type': 'symlink', 'sha256': hashlib.sha256(member.linkname.encode()).hexdigest()}
                elif not member.isdir():
                    raise ValueError('unsupported nonregular Git archive entry')
            # Root files came from checkout above. Only fill submodule contents.
            if relative != Path():
                archive.extractall(output / relative, filter='data')
    report = {'schema': 1, 'kind': 'mavmoos.firmware_export', 'revision': profile['source']['revision'],
              'manifest_sha256': canonical(profile), 'manifest_file_sha256': file_sha(manifest_path),
              'submodules': profile['source']['submodules'], 'files': dict(sorted(original_files.items())),
              'source_files_sha256': canonical(original_files), 'method': 'fresh shallow clone and exact recursive Git archives',
              'commands': commands, 'scope': 'original source bytes and refs only; no native execution or behavioral qualification'}
    (output / 'mavmoos-firmware-source.json').write_text(json.dumps(report, indent=2) + '\n')
    return {'result': 'PASS', 'source_identity': source_identity(profile, output), 'output': str(output)}


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest', type=Path, required=True)
    p.add_argument('--source-root', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    try:
        print(json.dumps(export_source(a.manifest, a.source_root, a.output), indent=2))
    except (OSError, ValueError, KeyError, TypeError, subprocess.CalledProcessError) as error:
        raise SystemExit(str(error))
