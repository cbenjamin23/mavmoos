#!/usr/bin/env python3
"""Export a pinned source-only ArduPilot tree with exact local submodule objects."""
import argparse
import io
import json
from pathlib import Path
import subprocess
import tarfile

ARDUPILOT_PIN = '04cf6486e8d42ce138de816e8466244346ed77cb'


def export(source, output):
    source, output = Path(source).resolve(), Path(output).resolve()
    if output.exists():
        raise RuntimeError('refusing to replace existing export: ' + str(output))
    subprocess.run(['git', 'clone', '--no-checkout', '--no-local', '--depth=1',
                    source.as_uri(), str(output)], check=True)
    subprocess.run(['git', '-C', str(output), 'fetch', '--depth=1', 'origin', ARDUPILOT_PIN], check=True)
    subprocess.run(['git', '-C', str(output), 'checkout', '--detach', ARDUPILOT_PIN], check=True)
    submodules = []

    def children(repository, revision, prefix):
        listing = subprocess.check_output(['git', '-C', str(repository), 'ls-tree', '-r', revision]).decode()
        for line in listing.splitlines():
            mode, kind, rest = line.split(' ', 2)
            if mode != '160000' or kind != 'commit':
                continue
            pin, rel = rest.split('\t', 1)
            relative = prefix / rel
            local = source / relative
            target = output / relative
            target.mkdir(parents=True, exist_ok=True)
            data = subprocess.check_output(['git', '-C', str(local), 'archive', '--format=tar', pin])
            with tarfile.open(fileobj=io.BytesIO(data)) as archive:
                for member in archive:
                    member_path = Path(member.name)
                    if member_path.is_absolute() or '..' in member_path.parts:
                        raise RuntimeError('unsafe source archive path')
                archive.extractall(target)
            submodules.append({'path': str(relative), 'revision': pin})
            children(local, pin, relative)

    children(source, ARDUPILOT_PIN, Path())
    provenance = {'revision': ARDUPILOT_PIN, 'submodules': submodules,
                  'method': 'fresh source clone; recursive git archives at gitlink revisions; no build products'}
    (output / 'mavmoos-source-provenance.json').write_text(json.dumps(provenance, indent=2) + '\n')
    return provenance


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    print(json.dumps(export(args.source, args.output), indent=2))
