#!/usr/bin/env python3
"""Qualify one release vehicle case with the existing owned SITL fixture."""
import argparse
import os
from pathlib import Path, PurePosixPath
import platform
import re
import signal
import subprocess
import sys
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
from ci.isolation import LinuxDomainLease
from ci.run import CIInterrupted, accepted, bounded, dump, parse, sha
from local_docker import CONTAINER_ID, OWNER_LABEL, LocalDocker, owned_container, remove_owned_container


MOOS_REVISION = '023fb54e00692611ecee2cd6f1138d10eec2127e'
HOST_HELM = '5b3d2e705609c37b5b5000384655b4491010d6d797b94c43830d9972e684a876'
ROVER_REVISION = '04cf6486e8d42ce138de816e8466244346ed77cb'
ROVER_SHA = 'a859ab703395bbb04da523dae5414cc5fbc23b736a1c71639f2ef3e255df5fc9'
PLANE_REVISION = '3fc7011a7d3dc047cbb17d8bd98ee94577d144c6'
PLANE_SHA = 'b977ff6cd559f6a6c3d16cfb57c1e32840291210eb593e4f7254fea701137fa5'
SUB_REVISION = '92b0cd788ec29406f26c6f9c31d5ceedbd1cc538'
SUB_SHA = '994970cc4d69145d537e4bae48c5ef8a5a254626c30c924151a1ab2b8cc3d5f8'
PX4_REVISION = 'fca3df865af36124a28c9d607e850f111dbaaea9'
PX4_SHA = '23f0b5eb07430868b5525a5c59e28185fe5b0da35ade9a7e04bdea5894144ea8'
TOOLS_IMAGE = 'sha256:644154814d3e5c162392b229f26bd68e3ad55c44acf863e78ee83e7568cbed85'
CASES = ('rover_ground', 'rover_boat', 'plane_normal', 'plane_takeover', 'sub', 'px4_offboard')
KIT_HELM = '/work/moos-ivp/bin/pHelmIvP'
KIT_PROBE = '/work/mavmoos-build/bin/px4_offboard_moos_probe'
KIT_STOCK = (KIT_HELM, '/work/moos-ivp/bin/MOOSDB', '/work/moos-ivp/bin/pAntler',
             '/work/moos-ivp/bin/pLogger', '/work/moos-ivp/bin/nsplug',
             '/work/altitude/build/lib/libBHV_ConstantAltitude.so')


def need(condition, message):
    if not condition:
        raise ValueError(message)


def git(path, *args):
    return subprocess.check_output(['git', '-C', str(path), *args], timeout=15, text=True).strip()


def clean_revision(path):
    need(git(path, 'status', '--porcelain', '--untracked-files=all') == '', f'dirty checkout: {path}')
    return git(path, 'rev-parse', 'HEAD')


def cache_path(cache, name):
    rows = re.findall(rf'^{name}:[^=]+=([^\n]+)$', cache.read_text(), re.M)
    need(len(rows) == 1, f'missing or ambiguous {name} in CMake cache')
    return Path(rows[0]).resolve()


def native_files(directory, index, required):
    """Verify the runner's original index without following path aliases."""
    need(isinstance(index, dict) and set(required) <= index.keys() and 0 < len(index) <= 4096,
         'native raw evidence index is incomplete')
    need(any(name.endswith(('.BIN', '.ulg')) for name in index), 'missing onboard log')
    for name, entry in index.items():
        relative = PurePosixPath(name)
        need(not relative.is_absolute() and relative.parts and '..' not in relative.parts and
             relative.as_posix() == name and name != 'result.json', f'unsafe native evidence member: {name}')
        path = directory / relative
        need(path.resolve() == path and path.is_file(), f'aliased or missing native evidence: {name}')
        if isinstance(entry, dict):
            need(type(entry['bytes']) is int and path.stat().st_size == entry['bytes'],
                 f'native evidence length differs: {name}')
        expected = entry['sha256'] if isinstance(entry, dict) else entry
        need(sha(path) == expected, f'native evidence digest differs: {name}')


def rover_proofs(receipt, case, revision, artifacts, *, live_source=None):
    boat = case == 'rover_boat'
    profile, frame = ('ardurover_boat', 'motorboat-skid') if boat else ('ardurover', 'rover')
    need(receipt['profile'] == profile and receipt['model'] == frame and
         receipt['mav_type'] == (11 if boat else 10) and receipt['probe_exit'] == 0,
         'Rover model, heartbeat type or workflow probe differs')
    source = receipt['mavmoos_cmake_source']
    need(receipt['mavmoos_revision'] == receipt['mavmoos_harness_revision'] == source['revision'] == revision and
         source['tracked_changes'] == [] and Path(source['path']).is_absolute() and
         (live_source is None or source['path'] == str(live_source)),
         'Rover app and runner source differ')
    need(receipt['ardupilot_revision'] == ROVER_REVISION and receipt['native_sha256'] == ROVER_SHA and
         receipt['moos_revision'] == MOOS_REVISION and receipt['app_sha256'] == artifacts['app'] and
         receipt['helm_sha256'] == artifacts['helm'] and receipt['digests_after'] == receipt['digests_before'] and
         receipt['loaded_modules_sha256'] == artifacts['modules'] and receipt['monitor_writes'] == 0,
         'Rover input artifact, selected modules or passive monitor differs')
    wire, dataflash = receipt['independent_monitor'], receipt['independent_dataflash']
    east, north = wire['stock_east_north_m']
    recovery = wire['reacquired_motion_m']
    loss = wire['guided_expiry_witness']['link_recovery']
    need(east >= 2.6 and north >= 3.6 and dataflash['pilot_manual']['Mode'] == 0 and
         dataflash['explicit_guided_recovery']['ModeNum'] == 15 and
         set(recovery) == {'allstop_reacquired', 'manual_reacquired', 'link_reacquired'} and
         min(recovery.values()) >= .8 and loss['firmware_elapsed_ms'] > 3000 and loss['final_speed_m_s'] <= .45,
         'Rover motion, pilot takeover or link loss proof is incomplete')
    proofs = {'stock_helm_motion_m': {'east': east, 'north': north}, 'observed_modes': wire['observed_modes'],
              'pilot_manual_dataflash': dataflash['pilot_manual'], 'link_loss_settlement': loss,
              'fresh_reacquisition_motion_m': recovery, 'returns': wire['returns']}
    if boat:
        loiter = wire['boat_native_loiter']
        need(loiter['loiter_seconds'] >= 25 and loiter['correction_servo_samples'] > 0 and
             loiter['loiter_max_displacement_m'] < loiter['unpowered_projection_m'] and
             bool(dataflash['native_loiter_modes']), 'boat disturbance Loiter evidence is incomplete')
        proofs['native_loiter'] = loiter
    return (ROVER_REVISION, ROVER_SHA, frame), proofs, ('fc-monitor.mavlink', 'workflow.jsonl')


def plane_proofs(receipt, case, revision, artifacts):
    selected = case.removeprefix('plane_')
    hashes = receipt['input_sha256']
    need(receipt['case'] == selected and receipt['profile'] == 'arduplane' and receipt['model'] == 'plane' and
         receipt['runner_revision'] == revision and receipt['firmware_revision'] == PLANE_REVISION and
         receipt['moos_revision'] == MOOS_REVISION and hashes['native'] == PLANE_SHA and
         hashes['app'] == artifacts['app'] and hashes['helm'] == artifacts['helm'] and
         hashes == receipt['input_sha256_after'] and receipt['monitor_writes'] == 0 and
         receipt['owned_mission_processes_remaining'] == [], 'Plane source, artifacts or teardown differ')
    for library, expected in artifacts['modules'].items():
        need(hashes['module_' + library.removeprefix('mavmoos_').removesuffix('.so')] == expected,
             f'Plane module differs: {library}')
    proofs = receipt['workflow']
    causes = receipt['mode_cause_evidence']
    if selected == 'normal':
        need(all(proofs[name]['displacement_m'] >= 68 for name in ('first_track', 'second_track')) and
             0 < proofs['normal_handoff_s'] < 1 and causes['explicit_rtl']['ModeNum'] == 11,
             'Plane motion, normal handoff or RTL proof is incomplete')
    else:
        mode = causes['pilot_takeover']
        need(mode['ModeNum'] == 5 and mode['Rsn'] == 1 and proofs['healthy_pilot_seconds'] >= 7 and
             proofs['watchdog_heartbeats_in_fbwa'] >= 6 and proofs['late_target_delivered_mono'] > 0,
             'Plane independent pilot takeover or late-target proof is incomplete')
    return (PLANE_REVISION, PLANE_SHA, 'plane'), {**proofs, 'dataflash_mode_causes': causes}, (
        'monitor-rx.mavlink', 'app-wire-0.bin', 'moos-probe.jsonl')


def sub_proofs(receipt, revision, artifacts, *, firmware=(SUB_REVISION, SUB_SHA, 'vectored')):
    inputs = {name: item['sha256'] for name, item in receipt['inputs'].items()}
    need(receipt['fixture_source'] == receipt['app_source'] == revision and
         (receipt['firmware_source'], inputs['simulator'], receipt.get('frame', 'vectored')) == firmware and
         receipt['moos_source'] == MOOS_REVISION and inputs['app'] == artifacts['app'] and
         inputs['helm'] == artifacts['helm'] and
         inputs['sub_plugin'] == artifacts['modules']['mavmoos_ardusub_control.so'] and
         inputs['telemetry_plugin'] == artifacts['modules']['mavmoos_telemetry.so'] and
         receipt['input_drift'] == [] and receipt['cleanup_proven'] is True,
         'Sub source, artifacts or teardown differ')
    transit, loss, takeover = (receipt[name] for name in ('transit', 'native_link_loss', 'pilot_takeover'))
    hold = receipt['captured_hold_readback']
    need(transit['east_m'] >= 3 and abs(transit['north_m']) <= 2 and 1.65 <= transit['depth_m'] <= 2.35 and
         hold['error_m'] <= .25 and receipt['helm_silence']['drift_m'] < .65 and loss['custom_mode'] == 2 and
         takeover['native']['custom_mode'] == 19 and takeover['nav_counter_frozen'] > 0,
         'Sub depth, captured hold, link loss or Manual takeover proof is incomplete')
    proofs = {'stock_helm_motion_m': {key: transit[key] for key in ('east_m', 'north_m', 'depth_m')},
              'captured_hold_error_m': hold['error_m'], 'link_loss_depth_hold': loss,
              'operator_handback': receipt['operator_handback'], 'pilot_takeover': takeover}
    return firmware, proofs, ('monitor-rx.mavlink', 'moos.jsonl')


def px4_proofs(receipt, revision, artifacts, firmware=(PX4_REVISION, PX4_SHA)):
    hashes = receipt['artifacts_before']
    hold, offboard = (4 << 16) | (3 << 24), 6 << 16
    firmware_revision, firmware_sha = firmware
    need(receipt['source_revision'] == revision and receipt['firmware_revision'] == firmware_revision and
         hashes['/opt/px4/bin/px4'] == firmware_sha and hashes['/work/mavmoos-build/bin/iMAVLink'] == artifacts['app'] and
         hashes['/work/moos-ivp/bin/pHelmIvP'] == artifacts['helm'] and
         receipt['artifact_drift'] == [] and receipt['resume_error'] is None, 'PX4 source, artifacts or teardown differ')
    namespace = receipt['namespace']
    need(namespace['ipv4_routes'] == [] and namespace['cap_eff'] == '0' * 16 and
         namespace['no_new_privileges'] is namespace['root_read_only'] is True and
         [name for name, flags in namespace['interface_flags'].items() if flags & 1] == ['lo'],
         'PX4 effective namespace proof is incomplete')
    first, second, fallback = receipt['travel'], receipt['second_acquisition'], receipt['independent_loss_hold']
    need(first['east_m'] >= 4 and second['travel']['east_m'] >= 4 and
         receipt['prestream_count'] >= 10 and second['prestream_count'] >= 10 and
         receipt['independent_offboard']['mode'] == second['independent_offboard']['mode'] == offboard and
         fallback['mode'] == receipt['independent_release_hold']['mode'] == hold and
         receipt['failsafe_current_mode']['intended_mode'] == offboard and
         receipt['handback_current_mode']['intended_mode'] == hold and
         'reason=acquisition_rejected' in receipt['denied_failsafe_acquire']['value'].split(',') and
         receipt['landed_disarmed'] is True, 'PX4 motion, loss, denial, reacquisition or landing proof is incomplete')
    proofs = {'stock_helm_legs_east_m': [first['east_m'], second['travel']['east_m']],
              'prestream_counts': [receipt['prestream_count'], second['prestream_count']],
              'independent_loss_hold': fallback, 'failsafe_mode': receipt['failsafe_current_mode'],
              'explicit_handback_mode': receipt['handback_current_mode'], 'landed_disarmed': True}
    return (firmware_revision, firmware_sha, 'sihsim_quadx'), proofs, ('gcs.mavlink', 'moos-wire.jsonl', 'startup.log')


def native_observations(receipt, directory, case, revision, artifacts, *, live_source=None,
                        px4_firmware=(PX4_REVISION, PX4_SHA)):
    """Read back the observed identity, workflow and original trace files."""
    need(receipt['result'] == 'PASS' and not receipt.get('postflight_error') and not receipt.get('cleanup_error') and
         receipt.get('hardware', False) is False and receipt['owned_processes_remaining'] == [] and
         isinstance(receipt['ports_clear'], dict) and receipt['ports_clear'] and
         all(value is True for value in receipt['ports_clear'].values()),
         'native runner failed or did not prove owned cleanup')
    if case.startswith('rover_'):
        identity, proofs, files = rover_proofs(receipt, case, revision, artifacts, live_source=live_source)
    elif case.startswith('plane_'):
        identity, proofs, files = plane_proofs(receipt, case, revision, artifacts)
    elif case == 'sub':
        identity, proofs, files = sub_proofs(receipt, revision, artifacts)
    else:
        need(case == 'px4_offboard', 'unknown native case')
        identity, proofs, files = px4_proofs(receipt, revision, artifacts, px4_firmware)
    native_files(directory, receipt['files'], files)
    firmware_revision, firmware_sha, frame = identity
    return {'revision': firmware_revision, 'native_sha256': firmware_sha, 'frame': frame}, proofs


def audit_native(directory, case, revision, artifacts, *, px4_firmware=(PX4_REVISION, PX4_SHA)):
    native = directory / 'result.json'
    need(native.is_file() and native.resolve() == native, 'missing or aliased native runner receipt')
    receipt = parse(native.read_text())
    firmware, proofs = native_observations(receipt, directory, case, revision, artifacts, live_source=ROOT,
                                           px4_firmware=px4_firmware)
    return {'firmware': firmware, 'proofs': proofs, 'cleanup_proven': True,
            'native_receipt': {'path': 'native/result.json', 'sha256': sha(native), 'file_count': len(receipt['files'])}}


def host_artifacts(build, moos, case):
    modules = ['telemetry']
    if case.startswith('rover_'):
        modules += ['parameters', 'ardurover_control']
    elif case.startswith('plane_'):
        modules += ['air_data', 'arduplane_control']
    else:
        modules.append('ardusub_control')
    names = [f'mavmoos_{module}.so' for module in modules]
    return {'app': sha(build / 'bin/iMAVLink'), 'helm': sha(moos / 'bin/pHelmIvP'),
            'modules': {name: sha(build / 'plugins' / name) for name in names}}


def host_command(args, native):
    need((platform.system(), platform.machine()) == ('Darwin', 'arm64'), 'host fixture pins require Darwin ARM64')
    need(args.build and args.moos_root and args.fixture_python, 'host case requires build, stock MOOS and fixture Python')
    build, moos = args.build.resolve(), args.moos_root.resolve()
    cache = build / 'CMakeCache.txt'
    need(cache_path(cache, 'CMAKE_HOME_DIRECTORY') == ROOT and cache_path(cache, 'MOOS_IVP_ROOT') == moos and
         clean_revision(moos) == MOOS_REVISION, 'host build or stock MOOS source differs')
    artifacts = host_artifacts(build, moos, args.case)
    need(artifacts['helm'] == HOST_HELM, 'stock Darwin helm artifact differs')
    common = ['--build', str(build), '--moos-root', str(moos), '--output', str(native)]
    if args.case.startswith('rover_'):
        need(args.rover_root, 'Rover case requires --rover-root')
        fixture = 'tests/rover_expanded/run_native.py'
        command = ['--ardupilot-root', str(args.rover_root.resolve()), '--profile', args.case.removeprefix('rover_'),
                   '--native-sha256', ROVER_SHA]
    elif args.case.startswith('plane_'):
        need(args.plane_build_root and args.behavior_dir, 'Plane case requires --plane-build-root and --behavior-dir')
        fixture = 'tests/plane_native/run_native.py'
        command = ['--plane-build-root', str(args.plane_build_root.resolve()), '--behavior-dir',
                   str(args.behavior_dir.resolve()), '--case', args.case.removeprefix('plane_'),
                   '--expected-app-revision', args.revision]
    else:
        need(args.sub_root, 'Sub case requires --sub-root')
        fixture = 'tests/ardusub/run_native.py'
        command = ['--ardupilot-root', str(args.sub_root.resolve()), '--app-revision', args.revision]
    return ['/usr/bin/caffeinate', '-i', str(args.fixture_python.absolute()), '-B', str(ROOT / fixture), *common, *command], artifacts


def volume(docker, name):
    need(name and re.fullmatch(r'[a-zA-Z0-9][a-zA-Z0-9_.-]{1,127}', name), 'invalid local Docker volume name')
    info, = parse(docker('volume', 'inspect', name))
    need(info['Name'] == name and info['Driver'] == 'local' and not info.get('Options'), 'expected a local Docker volume')
    return name


def isolated_px4_container(info, name, volumes, out):
    host = info['HostConfig']
    need(info['Image'] == TOOLS_IMAGE and info['Name'] == '/' + name and
         info['Config']['Labels']['mavmoos.scope'] == 'release-px4-offboard' and host['NetworkMode'] == 'none' and
         host['ReadonlyRootfs'] is True and host['Privileged'] is False and host['CapDrop'] == ['ALL'] and
         'no-new-privileges' in host['SecurityOpt'] and host['PidMode'] in ('', 'private') and
         host['IpcMode'] == host['CgroupnsMode'] == 'private' and host['UTSMode'] in ('', 'private') and
         all(not host.get(key) for key in ('CapAdd', 'Devices', 'DeviceRequests', 'PortBindings', 'VolumesFrom')),
         'PX4 container differs from the owned isolation policy')
    mounts = {}
    for entry in info['Mounts']:
        source = entry.get('Name') or entry['Source']
        if entry['Type'] == 'bind' and platform.system() == 'Darwin' and source.startswith('/host_mnt/'):
            source = source.removeprefix('/host_mnt')
        mounts[entry['Destination']] = (entry['Type'], source, entry['RW'])
    need(len(info['Mounts']) == 4 and mounts == {
        '/work': ('volume', volumes[0], False), '/opt/px4': ('volume', volumes[1], False),
        '/source': ('bind', str(ROOT), False), '/evidence': ('bind', str(out), True)},
         'PX4 container has unexpected source, firmware or evidence mounts')


def kit_witnesses(report, directory, revision):
    """Verify the archived builder witnesses used by the current release."""
    def read(name):
        path = directory / name
        need(path.is_file() and path.resolve() == path and path.stat().st_size <= 1024 * 1024,
             f'missing or aliased Linux kit witness: {name}')
        return parse(path.read_text())

    need(type(report['schema']) is int and report['schema'] == 1 and report['result'] == 'PASS' and
         report['source_revision'] == revision and
         report['moos_revision'] == MOOS_REVISION and report['platform'] == 'linux-arm64' and
         report['container_removed'] is True, 'Linux kit source or completion differs')
    rows = read('build-witness-artifacts.json')
    witnesses = {row['path']: row['sha256'] for row in rows}
    need(len(witnesses) == len(rows) and witnesses == report['build_witness_sha256'] and
         all(row['format'] == 'ELF64' and row['machine'] == 183 and
             re.fullmatch(r'[0-9a-f]{64}', row['sha256']) for row in rows) and
         set((*KIT_STOCK, KIT_PROBE)) <= witnesses.keys() and witnesses[KIT_HELM] == report['helm_sha256'],
         'Linux kit stock or probe witnesses are missing or differ from the build report')
    stock_archives = ('moos-ivp.tar', 'altitude.tar')
    for name, archives in (('stock-source-check.json', stock_archives),
                           ('final-source-check.json', (*stock_archives, 'mavmoos.tar'))):
        check = read(name)
        entries = check['checks']
        need(check['result'] == 'PASS' and [row['archive'] for row in entries] ==
             [f'/inputs/{archive}' for archive in archives] and
             all(re.fullmatch(r'[0-9a-f]{64}', row['sha256']) and
                 row['sha256'] == report['input_sha256'][archive] and row['changed_files'] == [] and
                 type(row['original_files']) is int and row['original_files'] > 0
                 for archive, row in zip(archives, entries)), f'Linux kit source check differs: {name}')
    return {'stock_witness_sha256': {name: witnesses[name] for name in KIT_STOCK},
            'probe_sha256': witnesses[KIT_PROBE]}


def px4_worker(revision, stable_fixture=None, *, aircraft=None):
    proof = parse(Path('/evidence/linux-inputs.json').read_text())
    build, source = Path('/work/mavmoos-build'), Path('/work/mavmoos')
    manifest = parse(Path('/work/evidence/package-manifest.json').read_text())
    need(manifest.get('schema') == 1 and manifest.get('source_revision') == revision,
         'Linux work volume was built for a different release revision')
    need(manifest['files']['bin/iMAVLink']['sha256'] == proof['app'],
         'Linux app digest differs from the retained package manifest')
    need(cache_path(build / 'CMakeCache.txt', 'CMAKE_HOME_DIRECTORY') == source and
         cache_path(build / 'CMakeCache.txt', 'MOOS_IVP_ROOT') == Path('/work/moos-ivp'),
         'Linux build volume does not bind the selected stock dependencies')
    need(all(sha(source / name) == expected for name, expected in proof['sources'].items()),
         'Linux build volume does not contain the selected release build sources')
    need(sha(build / 'bin/iMAVLink') == proof['app'] and
         sha('/work/moos-ivp/bin/pHelmIvP') == proof['helm'], 'Linux app or stock helm differs from build receipt')
    if aircraft:
        sys.path.insert(0, '/source/tests/px4_offboard')
        from run_aircraft import run_aircraft
        from run_fixed_wing import FRESH_BUILD_FILES
        from run_px4_offboard import namespace

        witnesses = kit_witnesses(proof['kit'], Path('/work/evidence'), revision)
        need(all(proof[key] == value for key, value in witnesses.items()),
             'retained Linux kit witnesses differ from the launch input')
        for path, digest in witnesses['stock_witness_sha256'].items():
            need(sha(path) == digest, f'retained Linux stock artifact differs: {path}')
        artifacts = {name: sha(build / name) for name in FRESH_BUILD_FILES}
        for name, digest in artifacts.items():
            expected = (witnesses['probe_sha256'] if str(build / name) == KIT_PROBE
                        else manifest['files'][name]['sha256'])
            need(digest == expected, f'Linux aircraft input differs from the release build: {name}')
        candidate = Path('/evidence/build-manifest.json')
        dump(candidate, {'schema': 1, 'source_revision': revision, 'tools_image': TOOLS_IMAGE,
                         'moos_revision': MOOS_REVISION, 'namespace': namespace(),
                         'stock_witness_sha256': witnesses['stock_witness_sha256'],
                         'mavlink_sha256': sha('/work/downloads/mavlink.tar.gz'), 'artifacts': artifacts})
        return run_aircraft(aircraft, revision, Path('/evidence/native'), candidate, build=build, release_stock=True)
    options = []
    if stable_fixture:
        need(proof['fixture_revision'] == stable_fixture and
             all(sha(Path('/source') / name) == expected for name, expected in proof['fixture_sources'].items()),
             'stable candidate fixture inputs differ')
        options.append('--stable-v1.17.0')
    os.execv(sys.executable, [sys.executable, '-B', '/source/tests/px4_offboard/run_px4_offboard.py',
                             '--source-revision', revision, '--output', '/evidence/native', *options])


def run_px4(args, out, endpoint, *, stable_fixture=None, aircraft=None):
    docker = endpoint.capture
    for value in (args.px4_app_sha256, args.px4_helm_sha256):
        need(value and re.fullmatch(r'[0-9a-f]{64}', value), 'PX4 requires Linux app and stock helm digests')
    artifacts = {'app': args.px4_app_sha256, 'helm': args.px4_helm_sha256}
    image, = parse(docker('image', 'inspect', TOOLS_IMAGE))
    need((image['Id'], image['Os'], image['Architecture']) == (TOOLS_IMAGE, 'linux', 'arm64'),
         'pinned offline Linux ARM64 tools image differs')
    volumes = (volume(docker, args.px4_build_volume), volume(docker, args.px4_package_volume))
    need(volumes[0] != volumes[1] and ',' not in str(out) and ',' not in str(ROOT), 'aliased or invalid PX4 mount')
    build_inputs = ['CMakeLists.txt', 'src', 'tests/px4_offboard']
    candidate_inputs = {}
    timeout = 150
    worker = ['--px4-worker', args.revision]
    if aircraft:
        kit_path = args.linux_kit_receipt.resolve(strict=True)
        kit = parse(kit_path.read_text())
        artifacts.update(kit_witnesses(kit, kit_path.parent, args.revision))
        need(kit['volume_retained'] is True and kit['work_volume'] == volumes[0] and
             kit['app_sha256'] == artifacts['app'] and kit['helm_sha256'] == artifacts['helm'],
             'PX4 release inputs differ from the verified Linux kit')
        candidate_inputs['kit'] = kit
        worker = ['--px4-aircraft-worker', args.revision, aircraft]
        timeout = 540
    if stable_fixture:
        build_inputs = ['CMakeLists.txt', 'src', *('tests/px4_offboard/' + name for name in
                        ('Px4MoosProbe.cpp', 'meta_vehicle.bhv', 'meta_vehicle.moos'))]
        need(not git(ROOT, 'diff', '--name-only', args.revision, stable_fixture, '--', *build_inputs),
             'stable candidate changed production or original mission/probe inputs')
        fixture_files = ('scripts/release/profiles.py', 'tests/px4_offboard/run_px4_offboard.py',
                         'tests/px4_offboard/firmware-v1.17.0.json')
        settle_timeout = parse((ROOT / fixture_files[2]).read_text())['home_settle_timeout_s']
        need(type(settle_timeout) is int and 0 < settle_timeout <= 240, 'invalid stable PX4 home settle limit')
        timeout += settle_timeout
        candidate_inputs = {'fixture_revision': stable_fixture,
                            'fixture_sources': {name: sha(ROOT / name) for name in fixture_files}}
        worker.append(stable_fixture)
    sources = git(ROOT, 'ls-files', '-z', '--', *build_inputs).split('\0')
    dump(out / 'linux-inputs.json', {**artifacts, **candidate_inputs,
                                    'sources': {name: sha(ROOT / name) for name in sources if name}})
    name = 'mavmoos-release-px4-' + uuid4().hex
    lease = LinuxDomainLease(out / 'domain-lease.json', args.revision)
    command = ['create', '--pull=never', '--name', name, '--label=mavmoos.scope=release-px4-offboard',
               '--label=' + OWNER_LABEL + '=' + name,
               '--network=none', '--ipc=private', '--cgroupns=private', '--read-only',
               '--cap-drop=ALL', '--security-opt=no-new-privileges',
               '--pids-limit=128', '--cpus=2', '--memory=768m', '--tmpfs=/tmp:rw,nosuid,nodev,size=64m',
               '--mount', f'type=volume,src={volumes[0]},dst=/work,readonly,volume-nocopy',
               '--mount', f'type=volume,src={volumes[1]},dst=/opt/px4,readonly,volume-nocopy',
               '--mount', f'type=bind,src={ROOT},dst=/source,readonly',
               '--mount', f'type=bind,src={out},dst=/evidence',
               '--entrypoint=/usr/bin/python3', TOOLS_IMAGE, '-B', '/source/scripts/release/profiles.py',
               *worker]
    dump(out / 'container-command.json', endpoint.argv(*command))
    lease.acquire()
    container = None
    try:
        created = docker(*command)
        need(CONTAINER_ID.fullmatch(created), 'missing exact owned container ID')
        container = created
        info, = parse(docker('container', 'inspect', container))
        dump(out / 'container-before.json', info)
        owned_container(info, name, name, container)
        isolated_px4_container(info, name, volumes, out)
        execution = bounded(endpoint.argv('start', '-a', container), ROOT, out / 'launcher.log', timeout,
                            env=endpoint.env, inventory_timeout=10)
        dump(out / 'launcher-result.json', execution)
        info, = parse(docker('container', 'inspect', container))
        dump(out / 'container-after.json', info)
        owned_container(info, name, name, container)
        need(accepted(execution) and info['State']['Running'] is False and info['State']['ExitCode'] == 0,
             'owned PX4 container did not exit successfully')
    finally:
        try:
            removed = remove_owned_container(docker, name, name, container)
            dump(out / 'container-cleanup.json', {'container': removed or container, 'removed': True,
                                                 'create_reply_received': container is not None})
        finally:
            lease.release()
    return artifacts


def audit_aircraft(out, revision, vehicle, artifacts):
    sys.path.insert(0, str(ROOT / 'tests/px4_offboard'))
    from qualify_aircraft import native_proof
    from run_aircraft import STABLE_PIN

    observed = native_proof(out, revision, vehicle, build_root='/work/mavmoos-build',
                            stock_witnesses=artifacts['stock_witness_sha256'])
    need(observed['app_sha256'] == artifacts['app'] and observed['helm_sha256'] == artifacts['helm'],
         'PX4 aircraft did not use the selected release app and stock helm')
    pin = parse(STABLE_PIN.read_text())
    raw = out / 'native/result.json'
    build = out / 'build-manifest.json'
    native = parse(raw.read_text())
    need(native['artifacts_before']['/evidence/build-manifest.json'] == sha(build) and
         native['artifacts_before'][KIT_PROBE] == artifacts['probe_sha256'],
         'PX4 raw artifact manifest or release probe differs')
    return {'firmware': {'revision': pin['revision'], 'native_sha256': pin['files']['bin/px4'],
                         'frame': observed['model']}, 'proofs': observed, 'cleanup_proven': True,
            'native_receipt': {'path': 'native/result.json', 'sha256': sha(raw), 'file_count': len(native['files'])}}


def run(args):
    out = args.output.resolve()
    out.mkdir(mode=0o700, parents=True, exist_ok=False)
    native = out / 'native'
    is_px4 = args.case == 'px4_offboard'
    report = {'schema': 1, 'result': 'FAIL', 'scope': 'owned_sitl', 'case': args.case, 'hardware': False,
              'source_revision': args.revision, 'moos_revision': MOOS_REVISION,
              'platform': 'linux-arm64' if is_px4 else f'{platform.system().lower()}-{platform.machine()}',
              'cleanup_proven': False}
    try:
        need(clean_revision(ROOT) == args.revision, 'clean runner checkout and release revision must agree')
        if is_px4:
            endpoint = LocalDocker()
            try:
                report['docker_endpoint'] = endpoint.evidence()
                artifacts = run_px4(args, out, endpoint, aircraft=args.aircraft)
            finally:
                endpoint.close()
        else:
            command, artifacts = host_command(args, native)
            execution = bounded(command, ROOT, out / 'launcher.log', 315, inventory_timeout=10)
            dump(out / 'launcher-result.json', execution)
            need(accepted(execution), 'owned host profile runner failed; inspect its original receipt and launcher log')
            need(host_artifacts(args.build.resolve(), args.moos_root.resolve(), args.case) == artifacts and
                 clean_revision(args.moos_root.resolve()) == MOOS_REVISION, 'protected host build or MOOS changed')
        need(clean_revision(ROOT) == args.revision, 'release source changed during qualification')
        if args.aircraft:
            report['aircraft'] = args.aircraft
            report.update(audit_aircraft(out, args.revision, args.aircraft, artifacts))
        else:
            report.update(audit_native(native, args.case, args.revision, artifacts))
        report.update(result='PASS', app_sha256=artifacts['app'], helm_sha256=artifacts['helm'])
    except (CIInterrupted, KeyboardInterrupt, OSError, ValueError, RuntimeError,
            LookupError, AttributeError, TypeError, subprocess.SubprocessError) as error:
        report['error'] = f'{type(error).__name__}: {error}'
    finally:
        dump(out / 'profile-result.json', report)
    print(f"{report['result']} {args.case}: {out / 'profile-result.json'}", flush=True)
    return report['result'] != 'PASS'


def interrupted(number, _):
    raise CIInterrupted(number, 'release vehicle qualification')


def main():
    if len(sys.argv) == 4 and sys.argv[1] == '--px4-aircraft-worker':
        return px4_worker(sys.argv[2], aircraft=sys.argv[3])
    if len(sys.argv) in (3, 4) and sys.argv[1] == '--px4-worker':
        return px4_worker(*sys.argv[2:])
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', required=True, choices=CASES)
    parser.add_argument('--revision', required=True, help='exact clean release commit')
    parser.add_argument('--output', required=True, type=Path, help='fresh private output directory')
    for name in ('build', 'moos-root', 'fixture-python', 'rover-root', 'plane-build-root', 'behavior-dir', 'sub-root'):
        parser.add_argument('--' + name, type=Path)
    for name in ('px4-build-volume', 'px4-package-volume', 'px4-app-sha256', 'px4-helm-sha256'):
        parser.add_argument('--' + name)
    parser.add_argument('--aircraft', choices=('quadrotor', 'fixed-wing', 'standard-vtol'))
    parser.add_argument('--linux-kit-receipt', type=Path)
    args = parser.parse_args()
    if args.aircraft and (args.case != 'px4_offboard' or args.linux_kit_receipt is None):
        parser.error('--aircraft requires --case px4_offboard and --linux-kit-receipt')
    if not re.fullmatch(r'[0-9a-f]{40}', args.revision):
        parser.error('--revision requires a full Git commit')
    signal.signal(signal.SIGTERM, interrupted)
    os.umask(0o077)
    return run(args)


if __name__ == '__main__':
    raise SystemExit(main())
