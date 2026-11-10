#!/usr/bin/env python3
"""Strict native-SITL input identity. Validation never launches a vehicle/socket.

Source declarations, built-artifact identities and behavioral qualification are
separate records. An attestation is recorded local provenance, not a signature
or proof that a binary was causally produced by a particular compiler.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import platform
import re
import struct
import subprocess

SHA40 = re.compile(r'[0-9a-f]{40}\Z')
SHA64 = re.compile(r'[0-9a-f]{64}\Z')
ID = re.compile(r'[a-z0-9][a-z0-9_.-]{0,95}\Z')
PARAM = re.compile(r'[A-Z][A-Z0-9_]{0,15}\Z')


def require(value, message):
    if not value:
        raise ValueError(message)


def fields(value, keys, label):
    require(isinstance(value, dict) and set(value) == set(keys), label + ': missing/extra fields')


def pairs(rows):
    result = {}
    for key, value in rows:
        require(key not in result, 'duplicate JSON key: ' + key)
        result[key] = value
    return result


def load(path):
    def nonfinite(value):
        raise ValueError('nonfinite JSON: ' + value)
    def finite_float(value):
        result = float(value);require(math.isfinite(result), 'nonfinite JSON float');return result
    return json.loads(Path(path).read_text(), object_pairs_hook=pairs, parse_constant=nonfinite, parse_float=finite_float)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def file_sha(path):
    return digest(Path(path).read_bytes())


def canonical(value):
    return digest(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode())


def relative(value):
    require(isinstance(value, str) and value and '\\' not in value and '\0' not in value, 'bad relative path')
    path = PurePosixPath(value)
    require(path.parts and not path.is_absolute() and str(path) == value and '..' not in path.parts, 'unsafe relative path')
    return path


def git(root, *arguments):
    return subprocess.check_output(['git', '-C', str(root), *arguments], text=True).strip()


def submodules(value):
    require(isinstance(value, list) and len(value) <= 256, 'invalid submodule list')
    result = {}
    for item in value:
        fields(item, ['path', 'revision'], 'submodule')
        relative(item['path'])
        require(item['path'] not in result and SHA40.fullmatch(item['revision']), 'duplicate/bad submodule pin')
        result[item['path']] = item['revision']
    require(list(result) == sorted(result), 'submodule list not canonical')
    return result


def validate_manifest(path):
    value = load(path)
    fields(value, ['schema', 'kind', 'profile_id', 'role', 'qualification', 'vehicle_profile', 'target',
                   'source', 'artifact_path', 'dependencies', 'fixture'], 'manifest')
    require(type(value['schema']) is int and value['schema'] == 1 and value['kind'] == 'mavmoos.firmware_source', 'manifest version/kind')
    require(ID.fullmatch(value['profile_id']) and '..' not in value['profile_id'] and value['role'] in ('baseline', 'candidate', 'canary'), 'manifest ID/role')
    require(value['qualification'] == 'not_inferred', 'source declaration cannot assert qualification')
    require(value['vehicle_profile'] == 'arducopter' and value['artifact_path'] == 'build/sitl/bin/arducopter', 'unsupported native profile/path')
    require(value['target'] == {'system_id': 1, 'component_id': 1, 'mav_type': 2, 'autopilot': 3} and all(type(v) is int for v in value['target'].values()), 'target identity differs')
    src = value['source']
    fields(src, ['repository', 'revision', 'release_label', 'version', 'submodules', 'protected_files'], 'source')
    require(src['repository'] == 'https://github.com/ArduPilot/ardupilot.git' and SHA40.fullmatch(src['revision']), 'source repository/revision')
    require(isinstance(src['release_label'], str) and 1 <= len(src['release_label']) <= 128, 'release label')
    fields(src['version'], ['major', 'minor', 'patch', 'release_type'], 'reported version expectation')
    require(all(type(v) is int and 0 <= v <= 255 for v in src['version'].values()), 'version bytes')
    submodules(src['submodules'])
    require(isinstance(src['protected_files'], dict) and {'ArduCopter/version.h', 'Tools/autotest/default_params/copter.parm', 'libraries/GCS_MAVLink/GCS_Common.cpp'} <= set(src['protected_files']), 'protected source witnesses missing')
    for name, expected in src['protected_files'].items():
        relative(name);require(isinstance(expected, str) and SHA64.fullmatch(expected), 'protected source hash')
    dep = value['dependencies']
    fields(dep, ['mavlink_revision', 'mavlink_archive_sha256', 'moos_revision', 'uav_revision'], 'dependencies')
    require(all(SHA40.fullmatch(dep[k]) for k in ('mavlink_revision', 'moos_revision', 'uav_revision')) and SHA64.fullmatch(dep['mavlink_archive_sha256']), 'dependency pin')
    fixture = value['fixture']
    fields(fixture, ['model', 'timewarp', 'home', 'default_parameter_file', 'parameters', 'hardware'], 'fixture')
    require(fixture['model'] == 'quad' and type(fixture['timewarp']) is int and fixture['timewarp'] == 1 and fixture['hardware'] is False, 'fixture scope differs')
    require(fixture['default_parameter_file'] == 'Tools/autotest/default_params/copter.parm', 'default parameter path differs')
    home = fixture['home']
    require(isinstance(home, list) and len(home) == 4 and all(type(x) in (int, float) and math.isfinite(x) for x in home), 'fixture home')
    require(abs(home[0]) <= 90 and abs(home[1]) <= 180 and -1000 <= home[2] <= 10000 and 0 <= home[3] < 360, 'fixture home bounds')
    params = fixture['parameters']
    require(isinstance(params, dict) and len(params) <= 128 and params.get('LOG_DISARMED') == 1 and params.get('SERIAL0_PROTOCOL') == params.get('SERIAL1_PROTOCOL') == 2, 'fixture observation requirements')
    require(all(PARAM.fullmatch(k) and type(v) in (int, float) and math.isfinite(v) for k, v in params.items()), 'fixture parameter')
    return value


def source_identity(manifest, root):
    root = Path(root).resolve()
    require(root.is_dir() and git(root, 'rev-parse', 'HEAD') == manifest['source']['revision'], 'native source revision mismatch')
    for name, expected in manifest['source']['protected_files'].items():
        require(file_sha(root / name) == expected, 'protected native source changed: ' + name)
    version_source = (root / 'ArduCopter/version.h').read_text()
    label = re.search(r'^#define\s+THISFIRMWARE\s+"([^"]+)"', version_source, re.M)
    version = re.search(r'^#define\s+FIRMWARE_VERSION\s+(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*FIRMWARE_VERSION_TYPE_(\w+)', version_source, re.M)
    release_types = {'DEV': 0, 'ALPHA': 64, 'BETA': 128, 'RC': 192, 'OFFICIAL': 255}
    require(label and label.group(1) == manifest['source']['release_label'] and version and version.group(4) in release_types, 'source version declaration differs')
    expected_version = dict(zip(('major', 'minor', 'patch'), map(int, version.groups()[:3])))
    expected_version['release_type'] = release_types[version.group(4)]
    require(expected_version == manifest['source']['version'], 'source version bytes differ')
    expected_modules = submodules(manifest['source']['submodules'])
    provenance = root / 'mavmoos-firmware-source.json'
    if provenance.is_file():
        report = load(provenance)
        fields(report, ['schema', 'kind', 'revision', 'manifest_sha256', 'manifest_file_sha256', 'submodules', 'files', 'source_files_sha256', 'method', 'commands', 'scope'], 'source export')
        require(type(report['schema']) is int and report['schema'] == 1 and report['kind'] == 'mavmoos.firmware_export' and report['revision'] == manifest['source']['revision'], 'source export identity')
        require(report['manifest_sha256'] == canonical(manifest) and report['submodules'] == manifest['source']['submodules'], 'source export manifest/submodules mismatch')
        require(isinstance(report['files'], dict) and 1 <= len(report['files']) <= 100000 and canonical(report['files']) == report['source_files_sha256'], 'source export file map')
        for name, item in report['files'].items():
            fields(item, ['type', 'sha256'], 'source file');require(SHA64.fullmatch(item['sha256']), 'source file digest')
            relative(name);path = root / name
            if item['type'] == 'file':
                require(path.is_file() and not path.is_symlink() and path.resolve().is_relative_to(root) and file_sha(path) == item['sha256'], 'exported source changed: ' + name)
            elif item['type'] == 'symlink':
                require(path.is_symlink() and digest(str(path.readlink()).encode()) == item['sha256'] and path.resolve().is_relative_to(root), 'exported symlink changed/escaped')
            else:
                raise ValueError('unknown source file type')
        return {'mode': 'git_archive_export', 'revision': report['revision'], 'submodules': report['submodules'],
                'source_files_sha256': report['source_files_sha256'], 'source_files': len(report['files'])}
    require(not git(root, 'status', '--porcelain', '--untracked-files=no', '--ignore-submodules=none'), 'native source checkout dirty')
    actual = {}
    for line in git(root, 'submodule', 'status', '--recursive').splitlines():
        require(line and line[0] not in ('+', '-', 'U'), 'submodule is missing or differs')
        tokens = line.split();require(len(tokens) >= 2, 'submodule identity malformed')
        actual[tokens[1]] = tokens[0]
    require(actual == expected_modules, 'recursive native submodule identity differs')
    return {'mode': 'clean_git_checkout', 'revision': manifest['source']['revision'], 'submodules': manifest['source']['submodules'],
            'source_files_sha256': canonical(manifest['source']['protected_files']), 'source_files': len(manifest['source']['protected_files'])}


def native_format(path):
    data = Path(path).read_bytes()[:64]
    if len(data) >= 20 and data[:6] == b'\x7fELF\x02\x01':
        machine = struct.unpack_from('<H', data, 18)[0]
        require(machine in (62, 183), 'unsupported ELF machine')
        return {'format': 'ELF64', 'system': 'Linux', 'machine': 'x86_64' if machine == 62 else 'arm64'}
    if len(data) >= 8 and data[:4] == b'\xcf\xfa\xed\xfe':
        machine = struct.unpack_from('<I', data, 4)[0]
        require(machine in (0x1000007, 0x100000c), 'unsupported Mach-O machine')
        return {'format': 'Mach-O64', 'system': 'Darwin', 'machine': 'x86_64' if machine == 0x1000007 else 'arm64'}
    raise ValueError('native artifact is not a supported executable format')


def host_platform():
    machine = platform.machine().lower()
    return {'system': platform.system(), 'machine': 'arm64' if machine in ('arm64', 'aarch64') else machine}


def validate_build_record(record, source, manifest_sha256, native_sha256):
    fields(record, ['schema', 'result', 'manifest_sha256', 'source_identity', 'commands', 'environment', 'builder_sha256', 'scope', 'native_sha256'], 'build record')
    require(type(record['schema']) is int and record['schema'] == 1 and record['result'] == 'PASS' and record['source_identity'] == source and record['manifest_sha256'] == manifest_sha256 and record['native_sha256'] == native_sha256, 'recorded build binding differs')
    require(SHA64.fullmatch(record['builder_sha256']) and isinstance(record['scope'], str) and isinstance(record['environment'], dict), 'recorded builder/environment')
    commands = record['commands']
    require(isinstance(commands, list) and 1 <= len(commands) <= 32, 'recorded build commands missing/excessive')
    for row in commands:
        fields(row, ['argv', 'exit', 'log', 'log_sha256'], 'build command')
        require(type(row['exit']) is int and row['exit'] == 0 and isinstance(row['argv'], list) and 1 <= len(row['argv']) <= 64 and all(isinstance(x, str) and 0 < len(x) <= 4096 for x in row['argv']), 'recorded build command failed/malformed')
        relative(row['log']);require(SHA64.fullmatch(row['log_sha256']), 'build command log hash')


def make_attestation(manifest_path, root, build_record=None):
    manifest = validate_manifest(manifest_path);root = Path(root).resolve()
    source = source_identity(manifest, root)
    native = root / manifest['artifact_path']
    require(native.is_file() and not native.is_symlink() and native.resolve().is_relative_to(root) and native.stat().st_size >= 64 and os.access(native, os.X_OK), 'native artifact missing/escaped/not executable')
    fmt = native_format(native);host = host_platform()
    require({k: fmt[k] for k in host} == host, 'native platform does not match current host')
    build = {'kind': 'existing_artifact_identity', 'record': None, 'record_sha256': None}
    if build_record is not None:
        record = load(build_record)
        validate_build_record(record, source, file_sha(manifest_path), file_sha(native))
        build = {'kind': 'recorded_source_build', 'record': record, 'record_sha256': file_sha(build_record)}
    return {'schema': 1, 'kind': 'mavmoos.native_build_attestation', 'profile_id': manifest['profile_id'],
            'manifest_sha256': file_sha(manifest_path), 'manifest_identity_sha256': canonical(manifest),
            'source_identity': source, 'platform': host,
            'artifact': {'path': manifest['artifact_path'], 'sha256': file_sha(native), 'bytes': native.stat().st_size, 'format': fmt['format']},
            'build': build, 'qualification': 'not_inferred'}


def verify_native(manifest_path, attestation_path, ardupilot_root, expected_profile='arducopter'):
    manifest = validate_manifest(manifest_path);attestation = load(attestation_path)
    require(manifest['vehicle_profile'] == expected_profile, 'native profile differs')
    fields(attestation, ['schema', 'kind', 'profile_id', 'manifest_sha256', 'manifest_identity_sha256', 'source_identity', 'platform', 'artifact', 'build', 'qualification'], 'attestation')
    require(type(attestation['schema']) is int and attestation['schema'] == 1 and attestation['kind'] == 'mavmoos.native_build_attestation' and attestation['qualification'] == 'not_inferred', 'attestation kind/scope')
    require(attestation['profile_id'] == manifest['profile_id'] and attestation['manifest_sha256'] == file_sha(manifest_path) and attestation['manifest_identity_sha256'] == canonical(manifest), 'attestation profile/manifest differs')
    root = Path(ardupilot_root).resolve();source = source_identity(manifest, root)
    require(attestation['source_identity'] == source, 'attested source identity differs')
    native = root / manifest['artifact_path']
    require(native.is_file() and not native.is_symlink() and native.resolve().is_relative_to(root) and native.stat().st_size >= 64 and os.access(native, os.X_OK), 'native executable missing/escaped/not executable')
    artifact = attestation['artifact'];fields(artifact, ['path', 'sha256', 'bytes', 'format'], 'artifact')
    fmt = native_format(native);host = host_platform()
    require(attestation['platform'] == host and all(fmt[k] == host[k] for k in host), 'native platform differs')
    require(artifact == {'path': manifest['artifact_path'], 'sha256': file_sha(native), 'bytes': native.stat().st_size, 'format': fmt['format']}, 'native artifact identity differs')
    build = attestation['build'];fields(build, ['kind', 'record', 'record_sha256'], 'build provenance')
    if build['kind'] == 'existing_artifact_identity':
        require(build['record'] is None and build['record_sha256'] is None, 'identity-only attestation has invented build proof')
    elif build['kind'] == 'recorded_source_build':
        record = build['record'];validate_build_record(record, source, file_sha(manifest_path), artifact['sha256'])
        encoded = (json.dumps(record, indent=2) + '\n').encode()
        require(digest(encoded) == build['record_sha256'], 'build record hash differs')
    else:
        raise ValueError('unknown build provenance kind')
    return {'schema': 1, 'profile_id': manifest['profile_id'], 'manifest_sha256': file_sha(manifest_path),
            'attestation_sha256': file_sha(attestation_path), 'source_revision': source['revision'],
            'native_path': str(native), 'native_sha256': artifact['sha256'], 'platform': host,
            'frame': manifest['fixture']['model'], 'fixture': manifest['fixture'],
            'build_provenance': build['kind'], 'qualification': 'not_inferred'}


def main():
    p = argparse.ArgumentParser(description=__doc__);sub = p.add_subparsers(dest='action', required=True)
    validate = sub.add_parser('validate');validate.add_argument('--manifest', type=Path, required=True)
    attest = sub.add_parser('attest');attest.add_argument('--manifest', type=Path, required=True);attest.add_argument('--ardupilot-root', type=Path, required=True);attest.add_argument('--output', type=Path, required=True);attest.add_argument('--build-record', type=Path)
    verify = sub.add_parser('verify');verify.add_argument('--manifest', type=Path, required=True);verify.add_argument('--attestation', type=Path, required=True);verify.add_argument('--ardupilot-root', type=Path, required=True);verify.add_argument('--expected-profile', default='arducopter')
    a = p.parse_args()
    if a.action == 'verify':
        value = verify_native(a.manifest, a.attestation, a.ardupilot_root, a.expected_profile)
    elif a.action == 'validate':
        value = {'result': 'PASS', 'manifest': validate_manifest(a.manifest), 'manifest_sha256': file_sha(a.manifest)}
    else:
        require(not a.output.exists(), 'refusing to overwrite attestation')
        value = make_attestation(a.manifest, a.ardupilot_root, a.build_record)
        a.output.parent.mkdir(parents=True, exist_ok=True);a.output.write_text(json.dumps(value, indent=2) + '\n')
    print(json.dumps(value, indent=2, allow_nan=False))


if __name__ == '__main__':
    try:
        main()
    except (ValueError, OSError, KeyError, TypeError, subprocess.CalledProcessError) as error:
        raise SystemExit(str(error))
