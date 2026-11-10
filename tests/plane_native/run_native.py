#!/usr/bin/env python3
"""Owned loopback Plane qualification with independent FC and MOOS evidence."""
import argparse
from contextlib import ExitStack
import errno
import fcntl
import json
import os
from pathlib import Path
import platform
import re
import signal
import socket
import sys
import tempfile
import traceback

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tests'))
from owned_processes import stop_process_tree
from flight_fixture import MOOS_PIN, PIN, Flight, json_data, need, now, revision, sha
sys.path.insert(0, str(ROOT / 'tests/plane_faults'))
from plane_firmware import AUTOLAND_MANIFEST_PATH, MANIFEST_PATH, verify_inputs, verify_parameters

CASES = ('prelude', 'rx-only', 'shadow', 'normal', 'silence', 'invalid', 'takeover',
         'loss-rx', 'loss-tx', 'loss-handback', 'loss-process', 'lifecycle', 'native-autoland',
         'operator-nudges', 'helm-tasks', 'approach-takeover', 'autonomy-survey', 'helm-mode-stress')
LIFECYCLE_CASES = ('lifecycle', 'native-autoland', 'approach-takeover')


def ports_clear(ports):
    cleared = {}
    for role, port in ports.items():
        kind = socket.SOCK_STREAM if role in ('db', 'base') else socket.SOCK_DGRAM
        with socket.socket(socket.AF_INET, kind) as check:
            if kind == socket.SOCK_STREAM:
                check.settimeout(.2)
                cleared[role] = check.connect_ex(('127.0.0.1', port)) == errno.ECONNREFUSED
            else:
                try:
                    check.bind(('0.0.0.0' if role == 'rc' else '127.0.0.1', port))
                    cleared[role] = True
                except OSError:
                    cleared[role] = False
    return cleared


def preflight(args):
    need(platform.system() == 'Darwin' and platform.machine() == 'arm64' and now() > 0,
         'pinned artifact and acquisition clock require Darwin ARM64')
    plane = args.plane_build_root.resolve()
    moos = args.moos_root.resolve()
    source = plane / 'source'
    paths = {
        'native': source / 'build/sitl/bin/arduplane',
        'defaults': source / 'Tools/autotest/models/plane.parm',
        'attestation': plane / 'build-attestation.json',
        'helm': moos / 'bin/pHelmIvP',
    }
    modern = args.case == 'native-autoland'
    manifest = AUTOLAND_MANIFEST_PATH if modern else MANIFEST_PATH
    verified = verify_inputs(manifest, paths['native'], paths['attestation'], paths['defaults'], moos)
    hashes = {'native': verified['native']['artifact']['sha256'], 'attestation': verified['native']['build_attestation_sha256'],
              'defaults': verified['fixture']['model_defaults']['sha256'],
              'helm': verified['dependencies']['stock_helm_sha256']}
    for role, path in paths.items():
        need(path.is_file() and not path.is_symlink() and sha(path) == hashes[role],
             f'exact {role} artifact is missing or differs')
    if args.case != 'prelude':
        from control_workflow import MODULES

        need(all((args.build, args.behavior_dir, args.expected_app_revision)),
             'integrated tests require --build, --behavior-dir, and --expected-app-revision')
        build = args.build.resolve()
        app_source = args.app_source_root.resolve()
        need(revision(app_source) == args.expected_app_revision and revision(ROOT) == args.expected_app_revision and
             app_source == ROOT, 'clean build and runner must select the same exact source revision')
        cache = build / 'CMakeCache.txt'
        for key, directory in (('CMAKE_HOME_DIRECTORY', app_source), ('MOOS_IVP_ROOT', moos)):
            line = re.search(rf'^{key}:[^=]+=([^\n]+)$', cache.read_text(), re.M)
            need(line and Path(line[1]).resolve() == directory, f'build provenance differs for {key}')
        behavior = args.behavior_dir.resolve()
        extension = '.dylib' if sys.platform == 'darwin' else '.so'
        paths.update({name: moos / 'bin' / file for name, file in (
            ('antler', 'pAntler'), ('db', 'MOOSDB'), ('logger', 'pLogger'), ('nsplug', 'nsplug'))})
        paths.update({'app': build / 'bin/iMAVLink', 'probe': build / 'bin/plane_native_probe',
                      'fault_tap': build / ('bin/libmavmoos_plane_fault_tap' + extension),
                      'altitude_behavior': behavior / ('libBHV_ConstantAltitude' + extension),
                      'manifest': manifest, 'cache': cache})
        for module in MODULES + (('missions',) if args.case in LIFECYCLE_CASES else ()):
            paths['module_' + module] = build / 'plugins' / ('mavmoos_' + module + '.so')
        if args.case in ('autonomy-survey', 'helm-mode-stress'):
            behavior_name = 'plane_stress.bhv' if args.case == 'helm-mode-stress' else 'plane_survey.bhv'
            paths.update(probe=build / 'bin/plane_autonomy_probe', evaluator=moos / 'bin/pMissionEval',
                         autonomy_behavior=ROOT / 'missions/autonomy_sitl' / behavior_name)
        for role, path in paths.items():
            need(path.is_file() and not path.is_symlink(), f'expected regular artifact missing: {role} {path}')
            hashes[role] = sha(path)
        paths['plugin_directory'] = build / 'plugins'
        paths['helm_tasks'] = args.case == 'helm-tasks'
    paths['modern'] = modern
    paths['verified_firmware'] = verified
    return paths, hashes


def dataflash(out):
    from pymavlink.DFReader import DFReader_binary

    bins = list((out / 'logs').glob('*.BIN'))
    need(len(bins) == 1, 'expected one native DataFlash BIN in fresh private state')
    digest = sha(bins[0])
    modes, positions = [], 0
    reader = DFReader_binary(str(bins[0]))
    while msg := reader.recv_match(type=['MODE', 'POS']):
        if msg.get_type() == 'MODE':
            modes.append(msg.to_dict())
        else:
            positions += 1
    need(modes and positions > 20 and sha(bins[0]) == digest, 'DataFlash mode/position evidence incomplete')
    return {'file': str(bins[0].relative_to(out)), 'sha256': digest, 'modes': modes, 'position_rows': positions}


def audit_reasons(reasons, dataflash_result):
    selected = {}
    used = set()
    modes = dataflash_result['modes']
    for check in reasons:
        previous_mode = check.get('previous_mode')
        rows = [row for index, row in enumerate(modes) if row['ModeNum'] == check['mode'] and
                row['Rsn'] == check['expected_reason'] and row['TimeUS'] not in used and
                (previous_mode is None or (index > 0 and modes[index - 1]['ModeNum'] == previous_mode)) and
                check['boot_ms_before'] - 350 <= row['TimeUS'] / 1000 <= check['boot_ms_after'] + 250]
        need(len(rows) == 1, f"DataFlash did not independently attribute {check['label']} to the expected cause")
        selected[check['label']] = rows[0]
        used.add(rows[0]['TimeUS'])
    return selected


def run(args):
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False, mode=0o700)
    report = {'result': 'FAIL', 'profile': 'arduplane', 'model': 'plane', 'hardware': False,
              'case': args.case, 'firmware_revision': PIN, 'moos_revision': MOOS_PIN,
              'speedup': 1, 'host': platform.platform(), 'start_mono': now()}
    paths = hashes = flight = mission = None
    try:
        from pymavlink.dialects.v20 import all as codec

        paths, hashes = preflight(args)
        report['firmware_revision'] = paths['verified_firmware']['source']['revision']
        airspeed = codec.mavlink_map.get(295)
        need(airspeed is not None and airspeed.crc_extra == 234 and airspeed.unpacker.format == '<ffhBB',
             'independent AIRSPEED decode does not match pinned wire definition')
        report['input_sha256'] = hashes
        report['runner_revision'] = revision(ROOT, clean=False)
        inputs = ('tests/plane_native/run_native.py', 'tests/plane_native/flight_fixture.py')
        if args.case != 'prelude':
            from control_workflow import Mission, exercise

            inputs += ('tests/plane_native/control_workflow.py', 'tests/plane_native/PlaneMoosProbe.cpp',
                       'tests/plane_faults/fault_tap.py', 'tests/plane_faults/OwnedPlaneFaultTap.cpp',
                       'tests/plane_faults/plane_firmware.py', 'tests/owned_processes.py',
                       'missions/plane_sitl/meta_vehicle.moos', 'missions/plane_sitl/meta_vehicle.bhv')
        if args.case in LIFECYCLE_CASES:
            inputs += ('tests/plane_native/lifecycle.py',)
        if args.case == 'operator-nudges':
            inputs += ('tests/plane_native/operator_nudges.py', 'tests/plane_native/lifecycle.py')
        if args.case == 'helm-tasks':
            inputs += ('tests/plane_native/operator_nudges.py', 'tests/plane_native/lifecycle.py',
                       'tests/plane_native/helm_tasks.py', 'tests/plane_native/task_behaviors.bhv')
        if args.case in ('autonomy-survey', 'helm-mode-stress'):
            inputs += ('tests/autonomy_missions/stock_grade.py', 'tests/plane_native/autonomy_survey.py')
        if args.case == 'helm-mode-stress':
            inputs += ('tests/plane_native/helm_mode_stress.py',)
        report['runner_sha256'] = {name: sha(ROOT / name) for name in inputs}
        with ExitStack() as stack:
            flight = Flight(paths, out, stack, codec, report)
            try:
                flight.launch()
                if args.case in LIFECYCLE_CASES:
                    flight.ground_prelude()
                else:
                    flight.prelude()
                for channel in flight.channels.values():
                    flight.ports[f'{channel.name}_native_peer'] = channel.peer[1]
                if args.case != 'prelude':
                    report['firmware_verification'] = paths['verified_firmware']
                    observed = {key: row['param_value'] for key, row in report['active_parameters'].items()}
                    report['verified_live_parameters'] = verify_parameters(paths['verified_firmware'], observed)
                    mission = Mission(flight, paths, out, stack, paths['fault_tap'],
                                      simulation=args.case not in ('rx-only', 'shadow'), shadow=args.case == 'shadow',
                                      lifecycle=args.case in LIFECYCLE_CASES)
                    report['workflow'] = exercise(mission, args.case)
            finally:
                if mission:
                    mission.close()
                if flight.simulator:
                    report['termination'] = {'action': 'private simulator stop', 'last_heartbeat': flight.heartbeat()}
                    report['owned_processes_remaining'] = stop_process_tree(flight.simulator)
        report['dataflash'] = dataflash(out)
        report['mode_cause_evidence'] = audit_reasons(report.get('required_mode_reasons', []), report['dataflash'])
        if mission:
            from control_workflow import packet_frames, fault_tap

            report['wire_trace_replay'] = {}
            for trace in mission.traces:
                records = fault_tap.read_trace(trace, flight.bridge.port, flight.bridge.peer[1])
                report['wire_trace_replay'][trace.name] = {
                    'records': len(records), 'app_frames': sum(1 for _ in packet_frames(codec, records)), 'sha256': sha(trace)}
            if args.case == 'helm-mode-stress':
                from helm_mode_stress import audit
                report['stress_readback'] = audit(out, report, records)
        report['result'] = 'PASS'
    except BaseException as error:
        report['error'] = str(error) or type(error).__name__
        (out / 'failure.log').write_text(traceback.format_exc())
    finally:
        try:
            report['ports_clear'] = ports_clear(report.get('ports', {}))
            need(all(report['ports_clear'].values()) and not report.get('owned_processes_remaining') and
                 not report.get('owned_mission_processes_remaining'),
                 'owned endpoint or process remains')
            if paths and hashes:
                report['input_sha256_after'] = {role: sha(paths[role]) for role in hashes}
                need(report['input_sha256_after'] == hashes and revision(args.moos_root.resolve()) == MOOS_PIN,
                     'protected firmware or stock MOOS changed')
                need(report.get('runner_sha256', {}) == {name: sha(ROOT / name) for name in report.get('runner_sha256', {})},
                     'runner or mission inputs changed during qualification')
        except BaseException as error:
            report['result'] = 'FAIL'
            report['postflight_error'] = str(error) or type(error).__name__
        report['end_mono'] = now()
        report['files'] = {str(file.relative_to(out)): {'bytes': file.stat().st_size, 'sha256': sha(file)}
                           for file in sorted(out.rglob('*')) if file.is_file() and not file.is_symlink()}
        (out / 'result.json').write_text(json.dumps(json_data(report), indent=2, allow_nan=False) + '\n')
    print(json.dumps({'result': report['result'], 'case': args.case, 'output': str(out),
                      'error': report.get('error'), 'postflight_error': report.get('postflight_error'),
                      'ports_clear': report['ports_clear']}, indent=2))
    return 0 if report['result'] == 'PASS' else 1


def interrupted(*_):
    raise InterruptedError('owned qualification interrupted')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plane-build-root', type=Path, required=True)
    parser.add_argument('--moos-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True, help='fresh private evidence directory')
    parser.add_argument('--case', choices=CASES, required=True)
    parser.add_argument('--build', type=Path, help='normal local build from the frozen runner checkout')
    parser.add_argument('--app-source-root', type=Path, default=ROOT)
    parser.add_argument('--expected-app-revision', help='exact clean revision configured into the build')
    parser.add_argument('--behavior-dir', type=Path, help='existing moos-ivp-uav-base behavior library directory')
    arguments = parser.parse_args()
    signal.signal(signal.SIGTERM, interrupted)
    with (Path(tempfile.gettempdir()) / 'mavmoos-native-sitl.lock').open('a') as lease:
        fcntl.flock(lease, fcntl.LOCK_EX)
        os.umask(0o077)
        raise SystemExit(run(arguments))
