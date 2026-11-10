#!/usr/bin/env python3
"""List and restore cold implementation archives from immutable local Git objects."""

import argparse
import hashlib
import os
from pathlib import Path, PurePosixPath
import re
import subprocess
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[2]
INDEX = ROOT / 'docs/development/evidence-index.tsv'
ARCHIVE_ROOT = PurePosixPath('docs/implementation')
CHUNK = 1024 * 1024


def relative(value):
    path = PurePosixPath(value)
    if (not value or path.is_absolute() or '..' in path.parts or '\\' in value or '\0' in value
            or str(path) != value or not path.is_relative_to(ARCHIVE_ROOT)):
        raise ValueError(f'expected a repository-relative archive path under {ARCHIVE_ROOT}: {value!r}')
    return path


def read_index():
    lines = INDEX.read_text(encoding='utf-8').splitlines()
    if len(lines) < 3:
        raise ValueError('empty cold evidence index')
    revision = lines[0].removeprefix('# git revision ')
    if not re.fullmatch('[0-9a-f]{40}', revision) or lines[1] != 'path\tbytes\tsha256\tgit_blob':
        raise ValueError('invalid cold evidence index header')
    entries = {}
    for line in lines[2:]:
        name, length, sha, blob = line.split('\t')
        path = relative(name)
        size = int(length)
        if (path in entries or path.suffix != '.zip' or size < 1 or
                not re.fullmatch('[0-9a-f]{64}', sha) or not re.fullmatch('[0-9a-f]{40}', blob)):
            raise ValueError(f'invalid cold evidence index row: {path}')
        entries[path] = size, sha, blob
    return revision, entries


def select(entries, names):
    if not names:
        return entries
    selected = {}
    for name in names:
        prefix = relative(name.rstrip('/'))
        matches = {path: metadata for path, metadata in entries.items() if path.is_relative_to(prefix)}
        if not matches:
            raise ValueError(f'no cold archives match {name!r}; use the list command to find archived paths')
        selected.update(matches)
    return selected


def local_git():
    return dict(os.environ, GIT_NO_LAZY_FETCH='1', GIT_NO_REPLACE_OBJECTS='1', GIT_ALLOW_PROTOCOL='')


def require_objects(revision, entries):
    command = ['git', '-C', str(ROOT), 'cat-file', '--batch-check']
    blobs = ''.join(f'{blob}\n' for _, _, blob in entries.values())
    result = subprocess.run(command, input=blobs, text=True, capture_output=True, env=local_git())
    expected = [f'{blob} blob {size}' for size, _, blob in entries.values()]
    if result.returncode or result.stdout.splitlines() != expected:
        raise ValueError(f'cold archive objects are missing locally. Use a full local checkout or fetch\n'
                         f'the exact archived revision from an origin that has it:\n'
                         f'  git fetch --depth=1 <archive-remote> {revision}\n'
                         'The restore command performs no fetch.')


def restore_file(output, path, metadata):
    size, expected, blob = metadata
    destination = output.joinpath(*path.parts)
    if destination.resolve() != destination or destination.exists():
        raise ValueError(f'output already exists or crosses a symlink: {destination}')
    destination.parent.mkdir(parents=True, exist_ok=True)
    command = ['git', '-C', str(ROOT), 'cat-file', 'blob', blob]
    digest = hashlib.sha256()
    count = 0
    with tempfile.NamedTemporaryFile(dir=destination.parent) as target:
        with subprocess.Popen(command, stdout=subprocess.PIPE, env=local_git()) as source:
            for block in iter(lambda: source.stdout.read(CHUNK), b''):
                digest.update(block)
                count += len(block)
                target.write(block)
            if source.wait() or count != size or digest.hexdigest() != expected:
                raise ValueError(f'archive failed byte and SHA-256 verification: {path}')
        target.flush()
        # Expose the file only after verification and refuse to replace an existing file.
        os.link(target.name, destination)
    return destination


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='action', required=True)
    inventory = commands.add_parser('list', help='show archive paths and size')
    inventory.add_argument('paths', nargs='*', metavar='PATH')
    restore = commands.add_parser('restore', help='copy selected verified ZIPs into a fresh output directory')
    restore.add_argument('--output', required=True, type=Path)
    restore.add_argument('paths', nargs='+', metavar='PATH')
    args = parser.parse_args()
    try:
        revision, entries = read_index()
        selected = select(entries, args.paths)
        if args.action == 'list':
            print(f'# {revision}\t{len(selected)} archives\t{sum(item[0] for item in selected.values())} bytes')
            for path, (size, _, _) in selected.items():
                print(f'{path}\t{size}')
            return
        output = args.output.resolve()
        if output.exists():
            raise ValueError(f'use a fresh output directory: {output}')
        require_objects(revision, selected)
        output.mkdir(parents=True, mode=0o700)
        for path, metadata in selected.items():
            print(restore_file(output, path, metadata))
    except (ValueError, OSError) as error:
        parser.exit(1, f'{parser.prog}: {error}\n')


if __name__ == '__main__':
    main()
