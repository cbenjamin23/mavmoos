#!/usr/bin/env python3
"""Verify exact inputs for the separately attested conventional Plane SITL cell."""

import argparse
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import sys


SOURCE_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SOURCE_ROOT / 'scripts' / 'firmware'))
from manifest import PARAM, SHA40, SHA64, fields, file_sha, git, load, relative, require, submodules


MANIFEST_PATH = SOURCE_ROOT / 'config/firmware/arduplane-4.6.3-plane.json'
AUTOLAND_MANIFEST_PATH = SOURCE_ROOT / 'config/firmware/arduplane-4.7.2-plane.json'
REVISION = '3fc7011a7d3dc047cbb17d8bd98ee94577d144c6'
AUTOLAND_REVISION = 'fb773567896014da007bca58358704d4a0d8516f'
MODEL_PATH = 'Tools/autotest/models/plane.parm'
ARTIFACT_PATH = 'source/build/sitl/bin/arduplane'
PROTECTED_SOURCE = {'ArduPlane/version.h', 'ArduPlane/GCS_Mavlink.cpp', 'ArduPlane/system.cpp',
                    'ArduPlane/events.cpp', 'libraries/GCS_MAVLink/GCS_Common.cpp',
                    'libraries/GCS_MAVLink/GCS_Param.cpp'}
WATCHDOG_PARAMS = {'SYSID_MYGCS': 245, 'FS_GCS_ENABL': 1, 'FS_LONG_ACTN': 1, 'FS_LONG_TIMEOUT': 5,
                   'RTL_AUTOLAND': 0, 'RTL_ALTITUDE': 65, 'RTL_RADIUS': 0, 'WP_LOITER_RAD': 80, 'Q_ENABLE': 0,
                   'AIRSPEED_CRUISE': 22, 'AIRSPEED_MIN': 10, 'AIRSPEED_MAX': 30,
                   'ARSPD_USE': 1, 'ARSPD_TYPE': 100, 'ARSPD_PRIMARY': 0, 'ARSPD2_TYPE': 0}
AUTOLAND_PARAMS = {'MAV_SYSID': 1, 'MAV_GCS_SYSID': 245, 'MAV_GCS_SYSID_HI': 0, 'ARMING_SKIPCHK': 0, 'MIS_RESTART': 0,
                   'AUTOLAND_WP_ALT': 45, 'AUTOLAND_WP_DIST': 485, 'AUTOLAND_DIR_OFF': 0,
                   'AUTOLAND_OPTIONS': 1, 'AUTOLAND_CLIMB': 0}


def number(value):
    return type(value) is int or (type(value) is float and math.isfinite(value))


def matches(pattern, value):
    return isinstance(value, str) and pattern.fullmatch(value) is not None


def validate_manifest(path):
    value = load(path)
    fields(value, ('schema', 'kind', 'profile_id', 'vehicle_profile', 'qualification',
                   'target', 'source', 'native', 'dependencies', 'fixture'), 'Plane manifest')
    profiles = {'arduplane-4.6.3-plane': ('4.6.3', REVISION, 20),
                'arduplane-4.7.2-plane': ('4.7.2', AUTOLAND_REVISION, 21)}
    require(type(value['schema']) is int and value['schema'] == 1 and
            value['kind'] == 'mavmoos.plane_native_fixture' and
            isinstance(value['profile_id'], str) and value['profile_id'] in profiles and value['vehicle_profile'] == 'arduplane' and
            value['qualification'] == 'not_inferred', 'unsupported Plane manifest scope')
    release, revision, module_count = profiles[value['profile_id']]
    modern = revision == AUTOLAND_REVISION
    target = value['target']
    require(target == {'system_id': 1, 'component_id': 1, 'mav_type': 1, 'autopilot': 3} and
            all(type(item) is int for item in target.values()), 'Plane target identity differs')
    source = value['source']
    fields(source, ('repository', 'tag', 'revision', 'tree', 'release_label', 'version',
                    'submodules', 'protected_files'), 'Plane source')
    require(source['repository'] == 'https://github.com/ArduPilot/ardupilot.git' and
            source['revision'] == revision and source['tag'] == f'Plane-{release}' and
            source['release_label'] == f'ArduPlane V{release}' and matches(SHA40, source['tree']),
            'Plane source pin differs')
    version = source['version']
    major, minor, patch = map(int, release.split('.'))
    require(version == {'major': major, 'minor': minor, 'patch': patch, 'release_type': 255} and
            all(type(item) is int for item in version.values()), 'Plane firmware version differs')
    require(len(submodules(source['submodules'])) == module_count, 'recursive Plane source pins missing')
    protected = ((PROTECTED_SOURCE - {'ArduPlane/GCS_Mavlink.cpp'}) |
                 {'ArduPlane/GCS_MAVLink_Plane.cpp', 'ArduPlane/mode_autoland.cpp'}) if modern else PROTECTED_SOURCE
    fields(source['protected_files'], protected, 'Plane protected source files')
    require(all(relative(name) and matches(SHA64, sha) for name, sha in source['protected_files'].items()),
            'bad Plane protected source hash')

    native = value['native']
    fields(native, ('platform', 'architecture', 'format', 'artifact', 'build_attestation_sha256'), 'Plane native input')
    require((native['platform'], native['architecture'], native['format']) ==
            ('darwin', 'arm64', 'Mach-O 64-bit executable arm64'), 'Plane attestation platform differs')
    fields(native['artifact'], ('path', 'sha256', 'bytes'), 'Plane native artifact')
    require(native['artifact']['path'] == ARTIFACT_PATH and type(native['artifact']['bytes']) is int and
            native['artifact']['bytes'] > 0 and matches(SHA64, native['artifact']['sha256']) and
            matches(SHA64, native['build_attestation_sha256']), 'bad Plane artifact binding')
    dependencies = value['dependencies']
    fields(dependencies, ('mavlink_revision', 'moos_revision', 'stock_helm_sha256'), 'Plane dependency identity')
    require(all(matches(SHA40, dependencies[key]) for key in ('mavlink_revision', 'moos_revision')) and
            matches(SHA64, dependencies['stock_helm_sha256']), 'bad Plane dependency pin')

    fixture = value['fixture']
    fields(fixture, ('model', 'timewarp', 'hardware', 'home', 'model_defaults', 'gcs_source',
                     'fixture_operator', 'parameters'), 'Plane fixture')
    require(fixture['model'] == 'plane' and fixture['hardware'] is False and
            type(fixture['timewarp']) is int and fixture['timewarp'] == 1, 'Plane fixture scope differs')
    home = fixture['home']
    require(isinstance(home, list) and all(number(item) for item in home) and
            home == [42.358456, -71.087589, 10, 0], 'Plane fixture home differs')
    fields(fixture['model_defaults'], ('path', 'sha256'), 'Plane model defaults')
    require(fixture['model_defaults']['path'] == MODEL_PATH and
            matches(SHA64, fixture['model_defaults']['sha256']), 'bad Plane default parameter binding')
    require(fixture['gcs_source'] == {'system_id': 245, 'component_id': 191} and
            fixture['fixture_operator'] == {'system_id': 249, 'component_id': 190} and
            all(type(item) is int for key in ('gcs_source', 'fixture_operator') for item in fixture[key].values()),
            'Plane test sender identities differ')
    parameters = fixture['parameters']
    require(isinstance(parameters, dict) and len(parameters) <= 64 and
            all(PARAM.fullmatch(name) and number(parameter) for name, parameter in parameters.items()),
            'bad Plane fixture parameter')
    required = WATCHDOG_PARAMS | {'SYSID_THISMAV': 1, 'SERIAL0_PROTOCOL': 2, 'SERIAL1_PROTOCOL': 2,
                                 'SERIAL2_PROTOCOL': 2, 'SIM_WIND_SPD': 0, 'SIM_WIND_TURB': 0,
                                 'LOG_DISARMED': 1, 'ARMING_CHECK': 1, 'FLTMODE_CH': 8,
                                 'FLTMODE3': 12, 'FLTMODE4': 5}
    if modern:
        for name in ('SYSID_THISMAV', 'SYSID_MYGCS', 'ARMING_CHECK'):
            del required[name]
        required.update(AUTOLAND_PARAMS)
    require(all(parameters.get(key) == expected for key, expected in required.items()),
            'Plane fixture safety policy differs')
    return value


def verify_attestation(manifest, attestation_path):
    native, source = manifest['native'], manifest['source']
    require(file_sha(attestation_path) == native['build_attestation_sha256'], 'Plane build attestation bytes differ')
    attestation = load(attestation_path)
    require(attestation.get('schema') == 1 and attestation.get('status') == 'OFFLINE_BUILD_PASS' and
            attestation.get('qualification') == 'build_only' and attestation.get('source_snapshot_verified') is True,
            'Plane attestation scope differs')
    require(attestation.get('source_revision') == source['revision'] and attestation.get('source_tree') == source['tree'] and
            attestation.get('source_tag') == source['tag'] and attestation.get('release_banner_bytes') == source['release_label'] and
            attestation.get('embedded_short_source_revision') == source['revision'][:8] and
            attestation.get('submodules') == [list(item) for item in submodules(source['submodules']).items()],
            'Plane attested source identity differs')
    require(attestation.get('native_format') == native['format'] and attestation.get('artifact') == native['artifact'] and
            attestation.get('model_default') == manifest['fixture']['model_defaults'], 'Plane attested artifact/model differs')
    return attestation


def verify_inputs(manifest_path, executable, attestation_path, model_path, helm_root):
    """Check local bytes before binding sockets or starting any process."""
    manifest = validate_manifest(manifest_path)
    native, dependency = manifest['native'], manifest['dependencies']
    require((sys.platform, platform.machine()) == (native['platform'], native['architecture']),
            'Plane local attestation does not cover this host architecture')
    attestation_path = Path(attestation_path).resolve(strict=True)
    verify_attestation(manifest, attestation_path)
    executable = Path(executable).resolve(strict=True)
    model_path = Path(model_path).resolve(strict=True)
    source_root = attestation_path.parent / 'source'
    artifact = native['artifact']
    require(executable == attestation_path.parent / artifact['path'] and executable.is_file() and os.access(executable, os.X_OK) and
            executable.stat().st_size == artifact['bytes'] and file_sha(executable) == artifact['sha256'], 'Plane executable differs')
    model = manifest['fixture']['model_defaults']
    require(model_path == source_root / model['path'] and file_sha(model_path) == model['sha256'], 'Plane model defaults differ')
    require(git(source_root, 'rev-parse', 'HEAD') == manifest['source']['revision'] and
            git(source_root, 'rev-parse', 'HEAD^{tree}') == manifest['source']['tree'], 'Plane source checkout differs')
    for name, expected in manifest['source']['protected_files'].items():
        require(file_sha(source_root / name) == expected, 'protected Plane source changed: ' + name)
    helm_root = Path(helm_root).resolve(strict=True)
    helm = helm_root / 'bin/pHelmIvP'
    require(git(helm_root, 'rev-parse', 'HEAD') == dependency['moos_revision'] and
            git(helm_root, 'status', '--porcelain', '--untracked-files=normal') == '', 'stock MOOS source changed')
    require(helm.is_file() and os.access(helm, os.X_OK) and
            file_sha(helm) == dependency['stock_helm_sha256'], 'stock pHelmIvP binary differs')
    return {**manifest, 'manifest_sha256': file_sha(manifest_path),
            'checked_paths': {'executable': str(executable), 'attestation': str(attestation_path),
                              'model_defaults': str(model_path), 'helm': str(helm)}}


def verify_parameters(manifest, active):
    """Check all required values from the fixture's independent FC readback."""
    require(isinstance(active, dict), 'Plane active parameter map missing')
    result = {}
    for name, expected in manifest['fixture']['parameters'].items():
        observed = active.get(name)
        require(number(observed) and observed == expected,
                f'Plane active parameter {name} is {observed!r}, expected {expected!r}')
        result[name] = observed
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, default=MANIFEST_PATH)
    parser.add_argument('--plane-build-root', required=True, type=Path)
    parser.add_argument('--moos-root', required=True, type=Path)
    options = parser.parse_args()
    root = options.plane_build_root
    try:
        result = verify_inputs(options.manifest, root / ARTIFACT_PATH, root / 'build-attestation.json',
                               root / 'source' / MODEL_PATH, options.moos_root)
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        parser.exit(1, f'{error}\n')
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))


if __name__ == '__main__':
    main()
