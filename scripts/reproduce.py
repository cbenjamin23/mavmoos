#!/usr/bin/env python3
"""Discover and run the documented MAVMOOS workflows against a shared setup."""

import argparse
import fcntl
import json
from pathlib import Path
import re
import subprocess
import sys
import tempfile
from uuid import uuid4


ROOT = Path(__file__).resolve().parents[1]
TRACKER = 'docs/plugin-progress.md'
RELEASE = 'docs/development/sitl-release.md'
CONTROL = 'docs/development/control-qualification.md'
SETUP = 'docs/development/from-scratch.md'
COPTER_GUIDE = 'missions/copter_release/README.md'
COPTER_MANIFEST = 'config/firmware/arducopter-4.6.3-quad.json'
MATRIX = 'scripts/release/matrix.json'
REGISTRY = 'config/capabilities/copter.json'
HOST = ('build', 'moos-root', 'fixture-python')
COPTER = (*HOST, 'behavior-dir', 'copter-root', 'copter-attestation')
LINUX_KIT = ('moos-source', 'uav-source', 'cpm-file', 'proj-archive', 'mavlink-archive')

RELEASE_BASELINE = {
    'rover_ground': ('Ground Rover helm', 'Guided tracks, loss stop and Manual handback.',
                     (*HOST, 'rover-root')),
    'rover_boat': ('Boat helm and Loiter', 'Guided tracks, disturbance rejection in Loiter and handback.',
                   (*HOST, 'rover-root')),
    'plane_normal': ('Plane normal handoff', 'Two observed stock-helm tracks with bounded handoff and native RTL.',
                     (*HOST, 'plane-build-root', 'behavior-dir')),
    'plane_takeover': ('Plane pilot takeover', 'Pilot FBWA persists and late helm targets stay fenced.',
                       (*HOST, 'plane-build-root', 'behavior-dir')),
}
MATRIX_EXPECTED = {
    'copter_operations': 'Advanced operator actions and Precision Loiter with observed touchdown and disarm.',
    'copter_auto': 'Verified typed AUTO plan, pause/resume, restart fence and observed landing.',
    'rover_mission': 'Verified waypoints, stopped pause, retained cursor, abort and fresh helm after handback.',
    'plane': 'Runway takeoff, stock helm and the selected landing procedure through disarm and Manual.',
    'sub': 'Stock-helm depth control, native ascent and shallow Depth Hold.',
    'quadplane': 'Observed phase-specific helm or native AUTO, followed by native recovery and disarm.',
    'px4_aircraft': 'Native lifecycle and phase-specific stock helm with observed flight and handback.',
}
MATRIX_INPUTS = {
    'copter_operations': COPTER,
    'copter_auto': COPTER,
    'rover_mission': (*HOST, 'rover-root'),
    'plane': (*HOST, 'behavior-dir', 'plane-build-root'),
    'sub': (*HOST, 'stable-sub-root'),
    'quadplane': (*HOST, 'behavior-dir', 'plane-build-root'),
    'px4_aircraft': (*LINUX_KIT, 'linux-work-volume', 'stable-px4-package-volume'),
}
BENCHES = {
    'copter_external_navigation': ('Disarmed ExternalNav bench', 'external_navigation/run_native.py',
        'Observe fused position and velocity and independently verify loss of aiding.',
        'docs/plugins/external_navigation.md'),
    'copter_payload': ('Disarmed camera and gimbal bench', 'payload_operations/run_native.py',
        'Verify a new image, measured gimbal pose, cancellation and foreign takeover.',
        'tests/payload_operations/README.md'),
    'copter_fence_edit': ('Reversible Copter fence edit', 'fence_operations/run_fence.py',
        'Verify original fence, replacement, interrupted transfer and exact restoration.',
        'docs/plugins/missions.md'),
}


def read_json(path):
    def object_from(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f'duplicate {key} in {path}')
            result[key] = value
        return result

    return json.loads(path.read_text(), object_pairs_hook=object_from)


def workflow(identifier, title, kind, scope, expected, result, guide, keys, recipe, *, cases=(), logs=None):
    command = f'python3 -B scripts/reproduce.py run {identifier} --workspace "$WORKSPACE"'
    if recipe in ('campaign', 'ackermann') or identifier == 'copter_native_services':
        command += ' --inputs "$RELEASE_INPUTS"'
    return {
        'id': identifier, 'title': title, 'kind': kind, 'scope': scope, 'expected': expected,
        'result': result, 'logs': logs or 'Logs and traces beside the result, under the run directory.',
        'guide': guide, 'requires': list(keys), 'command': command, 'recipe': recipe, 'cases': list(cases),
    }


def provider_workflows(tests):
    """The provider tracker owns the descriptions and required software witnesses."""
    rows = {}
    for line in (ROOT / TRACKER).read_text().splitlines():
        columns = line.split('|')
        if len(columns) != 7:
            continue
        provider = re.fullmatch(r'\s*(?:\[`(\w+)`\]\(([^)]+)\)|`(\w+)`)\s*', columns[1])
        if provider is None:
            continue
        linked_name, guide, plain_name = provider.groups()
        name = linked_name or plain_name
        case_ids = re.findall(r'`(ctest\.[a-z0-9_.]+)`', columns[4])
        if (name in rows or not case_ids or any(
                not tests.get(case, {}).get('required') or 'software' not in tests[case]['tiers']
                for case in case_ids)):
            raise ValueError(f'{name} must name unique, required software witnesses in {TRACKER}')
        if guide:
            guide = (Path(TRACKER).parent / guide).as_posix()
            if not (ROOT / guide).is_file():
                raise ValueError(f'missing provider guide: {guide}')
        rows[name] = workflow(f'provider.{name}', name, 'software',
            'Provider workflow witnesses using local tests and generated peers.',
            columns[2].strip(), 'result.json', guide or TRACKER, ('build',), 'focused', cases=case_ids,
            logs='See build.log, ctest.log, junit.xml and the case evidence under the run directory.')
    if not rows:
        raise ValueError(f'no provider recipes in {TRACKER}')
    return rows


def catalog():
    tests = {row['id']: row for row in read_json(ROOT / REGISTRY)['tests']}
    rows = [
        workflow('receive_only', 'First run · receive only', 'software',
                 'Real iMAVLink and MOOSDB with a local MAVLink fixture.',
                 'Selected telemetry arrives in MOOS and the bridge sends zero frames.',
                 'result.json', SETUP, ('build',), 'focused', cases=('ctest.moos_normal',)),
        workflow('copter_operator', 'Copter starter mission', 'native',
                 'ArduCopter 4.6.3 quad and stock pHelmIvP on loopback.',
                 'Observe takeoff, a waypoint route, stationkeeping, helm handback, landing and disarm.',
                 'result.json', COPTER_GUIDE, COPTER, 'copter',
                 logs='Inspect workflow.log, fc-monitor.mavlink and the DataFlash file under logs/.'),
        workflow('software', 'Local software suite', 'software',
                 'All selected local software cases for the configured build.',
                 'All required software cases pass with per-case logs.',
                 'result.json', CONTROL, ('build',), 'software'),
        workflow('external_plugins', 'External provider examples', 'software',
                 'Load the separately built observer and disarmed command modules through the real host.',
                 'The observer publishes heartbeats and rejects incompatible modules. The command stays host-gated.',
                 'result.json', 'docs/development/external-plugins.md', ('build',), 'focused',
                 cases=('ctest.external_observer_moos_normal', 'ctest.external_observer_moos_wrong_abi',
                        'ctest.external_disarm_contract', 'ctest.external_disarm_runtime')),
    ]
    rows.extend(provider_workflows(tests).values())
    for name, (title, expected, keys) in RELEASE_BASELINE.items():
        rows.append(workflow(name, title, 'native', 'Exact release input artifacts on Darwin ARM64.',
                             expected, f'profiles/{name}/profile-result.json', RELEASE, keys, 'campaign'))
    for name, cell in read_json(ROOT / MATRIX)['cases'].items():
        fixture = cell['fixture']
        px4 = fixture == 'px4_aircraft'
        keys = MATRIX_INPUTS[fixture]
        if cell['scenario'] == 'native-autoland':
            keys = (*keys[:-1], 'plane-stable-build-root')
        result_name = 'profile-result.json' if px4 else 'result.json'
        rows.append(workflow(name, name.replace('_', ' ').capitalize(), 'native',
                             f'{cell["profile"]} with exact release input artifacts.',
                             MATRIX_EXPECTED[fixture], f'matrix/{name}/{result_name}', RELEASE, keys, 'campaign',
                             cases=('linux_kit', name) if px4 else (name,)))
    for name, (title, runner, expected, guide) in BENCHES.items():
        rows.append(workflow(name, title, 'native', 'Pinned, owned Copter 4.6.3 simulator bench.',
                             expected, 'result.json', guide, COPTER, 'bench', cases=(runner,)))
    native_cases = ('ctest.parameters_sitl', 'ctest.mission_native_transfer', 'ctest.mission_native_inspection',
                    'ctest.flight_logs_native_copter', 'ctest.mavlink_files_native_copter',
                    'native.distance_flight', 'native.imu.rx_only')
    rows.extend([
        workflow('copter_native_services', 'Copter services and sensors', 'native',
                 'Selected focused-native CTests and fixtures with historical stock MOOS inputs.',
                 'Confirm parameters, plans, log and file retrieval, range flight and IMU observation.',
                 'result.json', CONTROL, COPTER, 'focused-native', cases=native_cases),
        workflow('copter_native_recovery', 'Copter authority and fault scenarios', 'native',
                 'Selected owned native faults and handoff with the historical stock MOOS pin.',
                 'Verify inhibited output across pause, restart and link faults, then explicit recovery.',
                 'result.json', CONTROL, COPTER, 'focused-native',
                 cases=('ctest.fault_helm_pause', 'ctest.fault_moosdb_restart',
                        'ctest.fault_simulator_pause', 'ctest.fault_simulator_restart',
                        'ctest.fault_bridge_pause', 'ctest.fault_command_timeout',
                        'ctest.auto_restart_recovery_sitl', 'ctest.external_disarm_sitl')),
        workflow('px4_ackermann', 'PX4 Ackermann · experimental', 'native',
                 'Separate PX4 1.17.0 SIH Ackermann qualification with exact Linux volumes.',
                 'Run mission and live helm, restart with retained ownership, and observe stopped abort.',
                 'qualification.json in full/, restart/ and abort/', 'tests/px4_rover/README.md',
                 ('mavlink-archive', 'px4-stock-volume', 'stable-px4-package-volume'), 'ackermann'),
        workflow('linux_kit', 'Linux ARM64 kit', 'software',
                 'Build and relocate the package in the existing isolated Linux release domain.',
                 'Verify packaged binaries and start a fresh relocated receive-only consumer.',
                 'linux/result.json', 'packaging/release/README.md',
                 (*LINUX_KIT, 'linux-work-volume'), 'campaign'),
        workflow('release', 'Full pinned SITL release matrix', 'native',
                 'Exact release artifacts across Darwin ARM64 and an isolated Linux ARM64 domain.',
                 'Verify software, the relocated kit and every declared release vehicle cell.',
                 'release-result.json', RELEASE,
                 (*COPTER, *LINUX_KIT, 'rover-root', 'plane-build-root', 'plane-stable-build-root',
                  'stable-sub-root', 'stable-px4-package-volume'), 'campaign'),
    ])
    index = {row['id']: row for row in rows}
    if len(index) != len(rows):
        raise ValueError('duplicate workflow ID')
    for row in rows:
        if not (ROOT / row['guide']).is_file():
            raise ValueError(f'missing workflow guide for {row["id"]}: {row["guide"]}')
        if row['recipe'] in ('focused', 'focused-native'):
            tier = 'software' if row['recipe'] == 'focused' else 'full'
            for case in row['cases']:
                if case not in tests or tier not in tests[case]['tiers'] or not tests[case]['required']:
                    raise ValueError(f'{row["id"]} refers to an unavailable case: {case}')
    return index


def public(row):
    return {key: value for key, value in row.items() if key not in ('recipe', 'cases')}


def git(*args, root=ROOT):
    return subprocess.check_output(['git', '-C', str(root), *args], text=True).strip()


def absolute_input(config, key):
    value = config.get(key)
    if not isinstance(value, str) or not Path(value).is_absolute() or not Path(value).exists():
        raise ValueError(f'{key} needs an existing absolute path in setup.json or --inputs')
    return value


def options(config, *keys):
    return [item for key in keys for item in ('--' + key, absolute_input(config, key))]


def run(row, args):
    workspace = args.workspace.expanduser().resolve()
    state = workspace / 'setup.json'
    if not state.is_file() and args.inputs is None:
        raise ValueError(f'run scripts/setup.py --workspace "{workspace}" first')
    config = read_json(state) if state.is_file() else {}
    if args.inputs:
        config.update(read_json(args.inputs))
    output = (args.output or (workspace / 'runs' / f'{row["id"]}-{uuid4().hex[:12]}')).resolve()
    if output.exists() or output == ROOT or output in ROOT.parents:
        raise ValueError(f'choose a fresh output directory: {output}')
    if ROOT in output.parents and subprocess.run(
            ['git', '-C', str(ROOT), 'check-ignore', '-q', '--', str(output)]).returncode != 0:
        raise ValueError('an in-tree run must write beneath an ignored directory such as build-reproduce/')

    recipe = row['recipe']
    python = [sys.executable, '-B']
    target = ['--output', str(output)]
    if recipe in ('software', 'focused', 'focused-native'):
        command = [*python, 'scripts/ci/run.py', '--tier', recipe, *options(config, 'build'),
                   '--jobs', str(args.jobs), *target]
        for case in row['cases']:
            command.extend(('--case', case))
        if recipe == 'focused-native':
            verify_historical_moos(config)
            command.extend([*options(config, 'moos-root', 'behavior-dir'),
                            '--firmware-manifest', COPTER_MANIFEST,
                            '--ardupilot-root', absolute_input(config, 'copter-root'),
                            '--firmware-attestation', absolute_input(config, 'copter-attestation'),
                            '--sitl-python', absolute_input(config, 'fixture-python')])
    elif recipe in ('copter', 'bench'):
        fixture = [absolute_input(config, 'fixture-python'), '-B']
        copter = ['--ardupilot-root', absolute_input(config, 'copter-root')]
        attest = absolute_input(config, 'copter-attestation')
        common = [*options(config, 'build', 'moos-root'), *copter, *target]
        revision = git('rev-parse', 'HEAD')
        if recipe == 'copter':
            command = [*fixture, 'tests/run_sitl.py', '--release-demo', *common,
                       '--firmware-attestation', attest, *options(config, 'behavior-dir'),
                       '--expected-app-revision', revision]
            public_pin = read_json(ROOT / 'config/setup.json')['moos']['revision']
            if git('rev-parse', 'HEAD', root=Path(config['moos-root'])) == public_pin:
                command.append('--public-stock')
        else:
            verify_historical_moos(config)
            command = [*fixture, 'tests/' + row['cases'][0], *common]
            if row['id'] == 'copter_fence_edit':
                command.extend(('--manifest', COPTER_MANIFEST, '--attestation', attest, '--probe',
                                str(Path(config['build']) / 'bin/fence_moos_probe')))
            else:
                command.extend(('--firmware-attestation', attest, '--expected-app-revision', revision))
    elif recipe == 'campaign':
        requested = () if row['id'] == 'release' else row['cases'] or (row['id'],)
        for key in row['requires']:
            if not key.endswith('-volume'):
                absolute_input(config, key)
        # Retain a uniquely named work volume only for this Linux release run.
        if not config.get('linux-work-volume'):
            config['linux-work-volume'] = 'mavmoos-release-reproduce-' + uuid4().hex[:12]
        with tempfile.NamedTemporaryFile(mode='w', prefix='mavmoos-release-', suffix='.json') as handle:
            json.dump(config, handle, indent=2)
            handle.flush()
            command = [*python, 'scripts/release/campaign.py', '--inputs', handle.name,
                       '--jobs', str(args.jobs), *target]
            for case in requested:
                command.extend(('--case', case))
            return execute(command, output)
    else:
        archive = absolute_input(config, 'mavlink-archive')
        for key in ('px4-stock-volume', 'stable-px4-package-volume'):
            if not isinstance(config.get(key), str) or not re.fullmatch(r'[a-zA-Z0-9][a-zA-Z0-9_.-]{1,127}', config[key]):
                raise ValueError(f'provide a qualified local {key} in --inputs')
        base = [*python, 'tests/px4_rover/qualify_ackermann.py', '--stock-volume', config['px4-stock-volume'],
                '--px4-volume', config['stable-px4-package-volume']]
        for case in ('full', 'restart', 'abort'):
            build = ['--mavlink-archive', archive] if case == 'full' else ['--built', str(output / 'full')]
            status = execute([*base, '--case', case, '--output', str(output / case), *build], output)
            if status:
                return status
        return 0
    return execute(command, output)


def verify_historical_moos(config):
    moos = Path(absolute_input(config, 'moos-root'))
    pin = read_json(ROOT / 'config/setup.json')['moos']['qualification_revision']
    if git('rev-parse', 'HEAD', root=moos) != pin:
        raise ValueError(f'this fixture needs stock MOOS {pin}; see {SETUP}')


def execute(command, output):
    print(f'Run directory: {output}', flush=True)
    return subprocess.run(command, cwd=ROOT).returncode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='action', required=True)
    listing = subparsers.add_parser('list', help='list workflow IDs and what they exercise')
    listing.add_argument('--json', action='store_true')
    show = subparsers.add_parser('show', help='show a recipe, prerequisites, expected results and logs')
    show.add_argument('id')
    runner = subparsers.add_parser('run', help='run a workflow with fresh evidence')
    runner.add_argument('id')
    runner.add_argument('--workspace', type=Path, required=True)
    runner.add_argument('--inputs', type=Path, help='local JSON overrides for exact release artifacts')
    runner.add_argument('--output', type=Path, help='new run directory; defaults to WORKSPACE/runs/')
    runner.add_argument('--jobs', type=int, choices=range(1, 5), default=2)
    args = parser.parse_args()
    index = catalog()
    if args.action == 'list':
        if args.json:
            print(json.dumps({'workflows': [public(row) for row in index.values()]}, indent=2))
        else:
            for row in index.values():
                print(f'{row["id"]:<31} {row["kind"]:<9} {row["title"]}')
        return 0
    if args.id not in index:
        parser.error(f'unknown workflow {args.id}; see list')
    row = index[args.id]
    if args.action == 'show':
        print(row['title'], row['scope'], row['command'], 'Requires: ' + ', '.join(row['requires']),
              'Expected: ' + row['expected'], 'Result: ' + row['result'], row['logs'],
              'Guide: ' + row['guide'], sep='\n')
        return 0
    lock = args.workspace.expanduser() / '.setup.lock'
    if not lock.is_file():
        return run(row, args)
    with lock.open() as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise ValueError('this workspace already has an active setup or workflow') from None
        return run(row, args)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError, TypeError, subprocess.CalledProcessError) as error:
        raise SystemExit(f'MAVMOOS reproduction: {error}') from error
