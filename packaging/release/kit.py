#!/usr/bin/env python3
"""Assemble and verify a local Linux MAVMOOS runtime kit."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import re
import shutil
import tarfile
import tempfile

HERE = Path(__file__).resolve().parent
MANIFEST = 'manifest.json'
CHECKSUMS = 'SHA256SUMS'
ARCHIVE = 'mavmoos-linux-arm64.tar.gz'
V1_VERSION = '1.0.0-rc.1'
V1_ARCHIVE = f'mavmoos-{V1_VERSION}-linux-arm64.tar.gz'
MODULE = re.compile(r'plugins/mavmoos_[a-z0-9_]+\.so')
LIBRARY = 'lib/libMOOSGeodesy.so'
LICENSES = ('MOOS-IvP-COPYING.md', 'PROJ-COPYING')
PAYLOAD = {'bin/iMAVLink', LIBRARY, 'share/mavmoos/README.md', 'share/mavmoos/receive-only.moos.in',
           *('share/mavmoos/licenses/' + name for name in LICENSES)}
STOCK_TOOLS = ('MOOSDB', 'pAntler', 'pHelmIvP', 'pLogger', 'nsplug', 'uProcessWatch', 'uPokeDB')
BEHAVIOR = 'lib/libBHV_ConstantAltitude.so'
STOCK_RUNTIME = {BEHAVIOR, *(f'bin/{name}' for name in STOCK_TOOLS)}
UAV_LICENSE = 'share/mavmoos/licenses/UAV-BASE-LICENSE'
V1_PAYLOAD = (PAYLOAD - {'share/mavmoos/README.md', 'share/mavmoos/receive-only.moos.in'}) | STOCK_RUNTIME | {
    'README.md', 'share/mavmoos/receive-only.moos', UAV_LICENSE}


def standard_modules(cmake):
    roster = re.findall(r'\bforeach\(feature\s+([^)]*)\)', cmake)
    features = roster[0].split() if len(roster) == 1 else []
    modules = {f'plugins/mavmoos_{feature}.so' for feature in features}
    if not features or len(modules) != len(features) or not all(MODULE.fullmatch(name) for name in modules):
        raise ValueError('empty or ambiguous built-in module roster')
    return modules


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def write_json(path, record):
    path.write_text(json.dumps(record, indent=2, sort_keys=True) + '\n')


def inventory(root):
    files = {}
    for path in root.rglob('*'):
        if path.is_symlink() or not (path.is_dir() or path.is_file()):
            raise ValueError(f'unexpected kit entry: {path.relative_to(root)}')
        if path.is_file():
            relative = path.relative_to(root).as_posix()
            if relative not in {MANIFEST, CHECKSUMS}:
                files[relative] = {'bytes': path.stat().st_size, 'sha256': sha(path)}
    return files


def validate_payload(root, files, expected_modules, *, v1=False):
    payload = V1_PAYLOAD if v1 else PAYLOAD
    modules = set(files) - payload
    if (not payload.issubset(files) or not expected_modules or modules != expected_modules or
            not all(MODULE.fullmatch(p) for p in modules)):
        raise ValueError(f'expected the app, {len(expected_modules)} standard modules, geodesy library and operator files')
    executables = modules | {'bin/iMAVLink', LIBRARY} | (STOCK_RUNTIME if v1 else set())
    for entry in sorted(executables):
        path = root / entry
        with path.open('rb') as stream:
            elf = stream.read(20)
        if (len(elf) != 20 or elf[:6] != b'\x7fELF\x02\x01' or
                int.from_bytes(elf[18:20], 'little') != 183 or not path.stat().st_mode & 0o111):
            raise ValueError(f'not an executable AArch64 ELF64 artifact: {entry}')


def checksums(files):
    return ''.join(f"{row['sha256']}  {name}\n" for name, row in sorted(files.items()))


def verify(root, expected_revision=None, expected_modules=None):
    files = inventory(root)
    record = json.loads((root / MANIFEST).read_text())
    v1 = record.get('schema') == 2
    payload = V1_PAYLOAD if v1 else PAYLOAD
    modules = set(record['files']) - payload if expected_modules is None else expected_modules
    validate_payload(root, files, modules, v1=v1)
    if (record.get('schema') not in (1, 2) or record.get('platform') != 'linux-arm64' or
            type(record.get('module_count')) is not int or record['module_count'] != len(modules) or
            not re.fullmatch(r'[0-9a-f]{40}', record.get('source_revision', ''))):
        raise ValueError('invalid kit manifest')
    if v1 and (record.get('bundle_version') != V1_VERSION or not all(
            re.fullmatch(r'[0-9a-f]{40}', record.get(key, '')) for key in ('moos_revision', 'uav_revision'))):
        raise ValueError('invalid v1 runtime provenance')
    if expected_revision and record['source_revision'] != expected_revision:
        raise ValueError('kit source revision differs')
    if record.get('files') != files or (root / CHECKSUMS).read_text() != checksums(files):
        raise ValueError('kit payload differs from its manifest')
    return record


def normalise(member):
    member.uid = member.gid = member.mtime = 0
    member.uname = member.gname = ''
    member.mode = 0o755 if member.isdir() or member.mode & 0o111 else 0o644
    return member


def pack(revision, build, output, *, stock=None):
    modules = standard_modules((HERE.parents[1] / 'src/plugins/CMakeLists.txt').read_text())
    with tempfile.TemporaryDirectory(prefix='mavmoos-kit-') as temporary:
        root = Path(temporary) / 'mavmoos'
        for name in ('bin', 'plugins', 'lib', 'share/mavmoos/licenses'):
            (root / name).mkdir(parents=True, exist_ok=True)
        shutil.copy2(build / 'bin/iMAVLink', root / 'bin')
        shutil.copy2(build / LIBRARY, root / LIBRARY)
        for name in LICENSES:
            shutil.copy2(build / 'licenses' / name, root / 'share/mavmoos/licenses' / name)
        for path in (build / 'plugins').glob('mavmoos_*.so'):
            shutil.copy2(path, root / 'plugins')
        if stock:
            for name in STOCK_TOOLS:
                shutil.copy2(build.parent / 'moos-ivp/bin' / name, root / 'bin')
            shutil.copy2(build.parent / 'altitude/build' / BEHAVIOR, root / BEHAVIOR)
            shutil.copy2(build / 'licenses/UAV-BASE-LICENSE', root / UAV_LICENSE)
            shutil.copy2(HERE / 'runtime-v1.md', root / 'README.md')
            template = (HERE / 'receive-only.moos.in').read_text()
            (root / 'share/mavmoos/receive-only.moos').write_text(
                template.replace('@PLUGIN_DIRECTORY@', 'plugins'))
        else:
            shutil.copy2(HERE / 'runtime.md', root / 'share/mavmoos/README.md')
            shutil.copy2(HERE / 'receive-only.moos.in', root / 'share/mavmoos')
        files = inventory(root)
        validate_payload(root, files, modules, v1=bool(stock))
        record = {'schema': 2 if stock else 1, 'source_revision': revision, 'platform': 'linux-arm64',
                  'module_count': len(modules), 'files': files}
        if stock:
            record.update(bundle_version=V1_VERSION, **stock)
        write_json(root / MANIFEST, record)
        (root / CHECKSUMS).write_text(checksums(files))
        verify(root, revision)
        archive = output / (V1_ARCHIVE if stock else ARCHIVE)
        with archive.open('xb') as stream:
            with gzip.GzipFile(filename='', mode='wb', fileobj=stream, mtime=0) as zipped:
                with tarfile.open(fileobj=zipped, mode='w', format=tarfile.PAX_FORMAT) as bundle:
                    bundle.add(root, arcname='mavmoos', filter=normalise)
        write_json(output / 'package-manifest.json', record)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest='mode', required=True)
    build = modes.add_parser('pack')
    build.add_argument('revision')
    build.add_argument('build', type=Path)
    build.add_argument('output', type=Path)
    build.add_argument('--stock-revisions', nargs=2, metavar=('MOOS', 'UAV'))
    check = modes.add_parser('verify')
    check.add_argument('root', type=Path)
    check.add_argument('revision')
    args = parser.parse_args()
    if args.mode == 'pack':
        stock = dict(zip(('moos_revision', 'uav_revision'), args.stock_revisions)) if args.stock_revisions else None
        pack(args.revision, args.build, args.output, stock=stock)
    else:
        print(json.dumps(verify(args.root, args.revision), indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
