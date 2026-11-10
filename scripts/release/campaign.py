#!/usr/bin/env python3
"""Run the local SITL release stages into one evidence directory."""
import argparse
import json
from pathlib import Path
import re
import signal
import subprocess
import sys

from matrix import STABLE_PX4_VOLUME, host_command, host_input_keys, host_observations, receipt_paths
from qualify import RECEIPTS, ROOT, bound, file_record, frozen_matrix, local, qualify_revision, receipt, save_report
from run import CIInterrupted, NATIVE_INVENTORY_TIMEOUT, accepted, bounded, dump, parse, require
import stress


def git(*args):
    return subprocess.check_output(['git', '-C', str(ROOT), *args], text=True).strip()


def clean_revision():
    require(not git('status', '--porcelain', '--untracked-files=all'), 'release checkout is dirty')
    return git('rev-parse', 'HEAD')


LINUX_INPUTS = ('moos-source', 'uav-source', 'cpm-file', 'proj-archive', 'mavlink-archive')
HOST_INPUTS = ('build', 'moos-root', 'fixture-python')
BASELINE_INPUTS = {
    'software': ('build',),
    'linux_kit': LINUX_INPUTS,
    'copter_operator': (*HOST_INPUTS, 'behavior-dir', 'copter-root', 'copter-attestation'),
    'rover_ground': (*HOST_INPUTS, 'rover-root'),
    'rover_boat': (*HOST_INPUTS, 'rover-root'),
    'plane_normal': (*HOST_INPUTS, 'plane-build-root', 'behavior-dir'),
    'plane_takeover': (*HOST_INPUTS, 'plane-build-root', 'behavior-dir'),
    'sub': (*HOST_INPUTS, 'sub-root'),
    'px4_offboard': (),
}


def input_path(config, key):
    value = config.get(key)
    require(isinstance(value, str) and value and '\0' not in value, f'missing local input {key}')
    path = Path(value).expanduser()
    require(path.is_absolute() and path.exists(), f'{key} must name an existing absolute path')
    # Keep the venv interpreter path, even when it is a symlink.
    return str(path)


def inputs(config, *keys):
    return [argument for key in keys for argument in ('--' + key, input_path(config, key))]


def case_inputs(case, cell):
    if case in stress.CASES:
        return stress.CASES[case].inputs
    if cell is None:
        return BASELINE_INPUTS[case]
    return () if cell['fixture'] == 'px4_aircraft' else host_input_keys(cell)


def volume_name(value, key, pattern=r'[a-zA-Z0-9][a-zA-Z0-9_.-]{1,127}'):
    require(isinstance(value, str) and re.fullmatch(pattern, value), f'invalid {key} volume name')
    return value


def px4_volume(config, aircraft):
    if aircraft:
        return volume_name(config.get('stable-px4-package-volume', STABLE_PX4_VOLUME), 'stable PX4 package')
    return volume_name(config.get('px4-package-volume', 'mavmoos-px4-wave-package'), 'PX4 package')


def preflight(config, cases, cells):
    keys = sorted({key for case in cases if case not in stress.CASES
                   for key in case_inputs(case, cells.get(case))})
    paths = {key: input_path(config, key) for key in keys}
    packages = {px4_volume(config, case != 'px4_offboard') for case in cases
                if case == 'px4_offboard' or cells.get(case, {}).get('fixture') == 'px4_aircraft'}
    if 'linux_kit' in cases:
        work = volume_name(config.get('linux-work-volume'), 'Linux work',
                           r'mavmoos-release-[a-zA-Z0-9][a-zA-Z0-9_.-]{1,80}')
        require(work not in packages, 'Linux build and PX4 firmware volumes alias')
    selected_stress = {name: stress.CASES[name] for name in cases if name in stress.CASES}
    return stress_inputs({**config, **paths}, selected_stress)


def command(case, config, output, revision, jobs, paths=RECEIPTS, cell=None):
    python = [sys.executable, '-B']
    stage = output / Path(paths[case]).parent
    target = ['--output', str(stage)]
    if case == 'software':
        return python + [str(ROOT / 'scripts/ci/run.py'), '--tier', 'software', '--jobs', str(jobs),
                         '--max-time', '900'] + inputs(config, 'build') + target, 1200
    if case == 'linux_kit':
        volume = config.get('linux-work-volume')
        require(isinstance(volume, str) and volume, 'missing Linux work volume name')
        return python + [str(ROOT / 'packaging/release/build_linux.py'), '--source-ref', revision,
                         '--work-volume', volume, '--jobs', str(jobs)] + inputs(
                             config, 'moos-source', 'uav-source', 'cpm-file', 'proj-archive',
                             'mavlink-archive') + target, 2400
    if case == 'copter_operator':
        copter = ['--ardupilot-root', inputs(config, 'copter-root')[1],
                  '--firmware-attestation', inputs(config, 'copter-attestation')[1]]
        return [str(ROOT / 'missions/copter_release/launch.sh'), '--python',
                inputs(config, 'fixture-python')[1], '--expected-app-revision', revision] + inputs(
                    config, 'build', 'moos-root', 'behavior-dir') + copter + target, 360
    aircraft = cell and cell['fixture'] == 'px4_aircraft'
    if cell and not aircraft:
        return host_command(cell, config, stage, revision)
    profile = python + [str(ROOT / 'scripts/release/profiles.py'), '--case', 'px4_offboard' if aircraft else case,
                        '--revision', revision] + target
    if case == 'px4_offboard' or aircraft:
        linux_path = output / RECEIPTS['linux_kit']
        linux = receipt(linux_path)
        bound(linux, revision)
        require(linux.get('volume_retained') is True and linux.get('container_removed') is True,
                'PX4 requires the verified retained Linux work volume')
        package_volume = px4_volume(config, aircraft)
        if aircraft:
            profile.extend(('--aircraft', cell['scenario'], '--linux-kit-receipt', str(linux_path)))
        return profile + ['--px4-build-volume', linux['work_volume'], '--px4-package-volume', package_volume,
                          '--px4-app-sha256', linux['app_sha256'], '--px4-helm-sha256', linux['helm_sha256']], 660
    extra = ('rover-root',) if case.startswith('rover_') else (
        ('plane-build-root', 'behavior-dir') if case.startswith('plane_') else ('sub-root',))
    return profile + inputs(config, 'build', 'moos-root', 'fixture-python', *extra), 600


def supervise(case, output, revision, argv, seconds):
    supervisor = output / 'supervisor'
    supervisor.mkdir(mode=0o700, exist_ok=True)
    with (supervisor / (case + '.started')).open('x') as started:
        started.write(revision + '\n')
    print(f'Starting {case}', flush=True)
    result = bounded(argv, ROOT, supervisor / (case + '.log'), seconds,
                     inventory_timeout=NATIVE_INVENTORY_TIMEOUT)
    dump(supervisor / (case + '.json'), result)
    return result


def launch(case, config, output, revision, jobs, paths, cell):
    result_path = output / paths[case]
    require(not result_path.parent.exists(), f'{case} already has a run directory')
    if case in stress.CASES:
        run_stress_case(case, stress.CASES[case], config, output, result_path.parent, revision, {})
        return
    argv, seconds = command(case, config, output, revision, jobs, paths, cell)
    result = supervise(case, output, revision, argv, seconds)
    require(accepted(result), f'{case} failed or did not clean up. See {result["log"]}')
    require(clean_revision() == revision, 'release source changed during the stage')
    result = receipt(result_path)
    if cell and cell['fixture'] != 'px4_aircraft':
        host_observations(result, result_path.parent, cell, revision)
    else:
        bound(result, revision)
    print(f'Passed {case}', flush=True)


def interrupted(number, _frame):
    raise CIInterrupted(number, 'release campaign')


def output_directory(path):
    output = path.resolve()
    if ROOT in output.parents:
        require(subprocess.run(['git', '-C', str(ROOT), 'check-ignore', '-q', str(output)]).returncode == 0,
                'in-tree release output must be ignored by Git')
    output.mkdir(mode=0o700, parents=True, exist_ok=True)
    return output


def stress_inputs(config, cases):
    keys = {key for case in cases.values() for key in case.inputs}
    config = {**config, **{key: input_path(config, key) for key in sorted(keys)}}
    if any(case.linux for case in cases.values()):
        config['stress-firmware-volume'] = px4_volume(config, True)
        if 'stress-stock-volume' in config:
            stock = volume_name(config['stress-stock-volume'], 'stress stock dependencies')
            require(stock != config['stress-firmware-volume'], 'stress stock and PX4 firmware volumes alias')
    return config


def run_stress_case(name, case, config, output, stage, revision, entry):
    entry.update(result='FAIL', supervisor=f'supervisor/{name}.json')
    run = supervise(name, output, revision, *stress.command(case, config, stage, revision))
    require(run['cleanup_proven'] is True and not run['timeout'] and run['unexpected_live_descendants'] == 0,
            f'{name} timed out or did not prove supervisor cleanup')
    require(clean_revision() == revision, 'stress campaign source changed during the stage')
    path = local(stage, case.receipt_name)
    raw = receipt(path)
    entry['receipt'] = file_record(output, path)
    native = stress.check_run(stage, raw, case, revision)
    if not case.blocking and raw.get('result') == native.get('result') == 'FAIL' and run['exit_code'] == 1:
        entry.update(result='ADVISORY_FAIL', error=raw.get('error'))
        print(f'Advisory failure in {name}', flush=True)
        return
    require(accepted(run) and raw.get('result') == native.get('result') == 'PASS',
            f'{name} failed its native stress mission')
    replay = supervise(name + '.replay', output, revision,
                       *stress.command(case, config, stage, revision, replay=True))
    require(accepted(replay) and clean_revision() == revision, f'{name} failed its independent replay')
    log = Path(replay['log'])
    receipt(log)
    entry.update(result='PASS', replay=file_record(output, log))
    print(f'Passed {name}', flush=True)


def stress_campaign(args, config, revision):
    cases = stress.select(args.case)
    report = {'schema': 1, 'source_revision': revision, 'tier': 'stress', 'cases': {
        name: {'profile': case.profile, 'domain': 'linux' if case.linux else 'darwin-arm64',
               'blocking': case.blocking, 'inputs': case.inputs, 'result': 'NOT_RUN'}
        for name, case in cases.items()}}
    if args.plan:
        if args.inputs:
            stress_inputs(config, cases)
        print(json.dumps(report, indent=2))
        return 0

    output = output_directory(args.output)
    require(not any(output.iterdir()), 'stress campaign requires a fresh output directory')
    report.update(result='PASS', advisory_failures=[])
    step = 'preflight'
    try:
        config = stress_inputs(config, cases)
        build = stress.host_build(config, cases, args.jobs)
        if build:
            step = 'stress_host_build'
            run = supervise(step, output, revision, build, 1200)
            require(accepted(run) and clean_revision() == revision, 'stress host build or its source check failed')
        for name, case in cases.items():
            step = name
            entry = report['cases'][name]
            run_stress_case(name, case, config, output, output / name, revision, entry)
            if entry['result'] == 'ADVISORY_FAIL':
                report['advisory_failures'].append(name)
    except (CIInterrupted, KeyboardInterrupt, OSError, ValueError, TypeError, KeyError, subprocess.SubprocessError) as error:
        report.update(result='FAIL', failed_stage=step, error=str(error) or type(error).__name__)
        if step in cases:
            report['cases'][step]['error'] = report['error']
        if isinstance(error, CIInterrupted):
            dump(output / 'stress-result.json', report)
            return 128 + error.number
    dump(output / 'stress-result.json', report)
    print(f"{report['result']} {output / 'stress-result.json'}", flush=True)
    return int(report['result'] != 'PASS')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inputs', type=Path, help='JSON with local pinned paths')
    parser.add_argument('--output', type=Path, help='evidence root for this campaign')
    parser.add_argument('--plan', action='store_true', help='show the exact revision and case inventory without running')
    parser.add_argument('--stress', action='store_true', help='run stress missions on their own')
    parser.add_argument('--case', action='append', metavar='CASE',
                        help='run specific cases in order. Default runs all cases in the selected tier')
    parser.add_argument('--jobs', type=int, choices=(1, 2, 3, 4), default=3)
    args = parser.parse_args()
    signal.signal(signal.SIGTERM, interrupted)
    if not args.plan and (args.inputs is None or args.output is None):
        parser.error('--inputs and --output are required for execution')
    config = parse(args.inputs.read_text()) if args.inputs else {}
    require(isinstance(config, dict), 'local inputs must be a JSON object')
    revision = clean_revision()
    if args.stress:
        return stress_campaign(args, config, revision)
    matrix = frozen_matrix(revision)
    paths = receipt_paths(matrix, RECEIPTS)
    cells = matrix['cases'] if matrix else {}
    requested = args.case or tuple(paths)
    require(len(requested) == len(set(requested)) and set(requested) <= paths.keys(),
            'invalid or duplicate campaign case; use --plan for available cases')
    cases = [case for case in paths if case in requested]
    if args.plan:
        if args.inputs:
            preflight(config, cases, cells)
        print(json.dumps({'source_revision': revision, 'cases': {
            case: {'receipt': paths[case], 'inputs': case_inputs(case, cells.get(case)),
                   **({'fixture': 'stress', 'profile': stress.CASES[case].profile} if case in stress.CASES
                      else cells.get(case, {}))}
            for case in cases},
            'outside_release_matrix': matrix['outside_release_matrix'] if matrix else {}}, indent=2))
        return 0
    output = output_directory(args.output)
    full = args.case is None
    final = output / 'release-result.json'
    if full:
        require(not any(output.iterdir()), 'a full campaign requires a fresh output directory')
    failure = None
    try:
        config = preflight(config, cases, cells)
        selected_stress = {name: stress.CASES[name] for name in cases if name in stress.CASES}
        for case in (name for name in cases if name not in selected_stress):
            launch(case, config, output, revision, args.jobs, paths, cells.get(case))
        build = stress.host_build(config, selected_stress, args.jobs)
        if build:
            step = next(iter(selected_stress)) + '.build'
            run = supervise(step, output, revision, build, 1200)
            require(accepted(run) and clean_revision() == revision, 'stress host build or its source check failed')
        for case in selected_stress:
            launch(case, config, output, revision, args.jobs, paths, None)
    except (CIInterrupted, KeyboardInterrupt, OSError, ValueError, TypeError, KeyError, subprocess.SubprocessError) as error:
        if not full:
            raise
        failure = error
    if not full:
        return 0
    report = qualify_revision(output, revision)
    if failure is not None:
        report['result'] = 'FAIL'
        report['errors']['campaign'] = str(failure) or type(failure).__name__
    status = save_report(report, final)
    return 128 + failure.number if isinstance(failure, CIInterrupted) else status


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except CIInterrupted as error:
        print(json.dumps({'result': 'FAIL', 'error': str(error), 'interrupted': True}), file=sys.stderr)
        raise SystemExit(128 + error.number)
    except (OSError, ValueError, TypeError, KeyError, subprocess.CalledProcessError) as error:
        print(json.dumps({'result': 'FAIL', 'error': str(error)}), file=sys.stderr)
        raise SystemExit(1)
