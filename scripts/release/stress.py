"""Selection and evidence binding for supplemental owned SITL missions."""
from dataclasses import dataclass
from pathlib import Path
import platform
import sys

from profiles import HOST_HELM, MOOS_REVISION, ROOT, cache_path, native_files
from receipts import local, receipt
from ci.run import require, sha


@dataclass(frozen=True)
class StressCase:
    profile: str
    runner: str
    arguments: tuple
    identity: tuple
    targets: tuple = ()
    replay: str = ''
    revision_field: str = 'source_revision'
    blocking: bool = True

    @property
    def linux(self):
        return not self.targets

    @property
    def receipt_name(self):
        return 'qualification.json' if self.linux else 'result.json'

    @property
    def inputs(self):
        common = ('mavlink-archive',) if self.linux else ('build', 'moos-root', 'fixture-python')
        return common + tuple(arg[1:] for arg in self.arguments if arg.startswith('@') and arg != '@tap')


CASES = {
    'stress_copter': StressCase(
        'ArduCopter 4.6.3 quad', 'autonomy_missions/run_native.py',
        ('copter-mode-stress', '--ardupilot-root', '@copter-root', '--firmware-attestation', '@copter-attestation',
         '--behavior-dir', '@behavior-dir', '--tap', '@tap'),
        ('mission', 'copter-mode-stress'), ('autonomy_mission_probe', 'mavmoos_arducopter_control', 'mavmoos_missions'),
        'autonomy_missions/audit.py', 'source'),
    'stress_boat': StressCase(
        'ArduRover 04cf6486 motorboat-skid', 'autonomy_missions/run_native.py',
        ('boat-mode-stress', '--ardupilot-root', '@rover-root', '--tap', '@tap'),
        ('mission', 'boat-mode-stress'), ('autonomy_mission_probe', 'mavmoos_ardurover_control'),
        'autonomy_missions/audit.py', 'source'),
    'stress_plane': StressCase(
        'ArduPlane 4.6.3 conventional', 'plane_native/run_native.py',
        ('--case', 'helm-mode-stress', '--plane-build-root', '@plane-build-root', '--behavior-dir', '@behavior-dir',
         '--expected-app-revision', '{revision}'),
        ('case', 'helm-mode-stress'), ('plane_autonomy_probe', 'mavmoos_arduplane_control', 'mavmoos_air_data'),
        'plane_native/replay.py', 'runner_revision'),
    'stress_quadplane': StressCase(
        'ArduPlane 4.6.3 fixed-motor QuadPlane', 'quadplane_native/run_native.py',
        ('--case', 'hover-mode-stress', '--plane-build-root', '@plane-build-root', '--behavior-dir', '@behavior-dir',
         '--app-revision', '{revision}'),
        ('case', 'hover-mode-stress'), ('depth_autonomy_probe', 'mavmoos_arduquadplane_control', 'mavmoos_missions'),
        'quadplane_native/hover_helm.py', 'app_revision'),
    'stress_sub': StressCase(
        'ArduSub 4.7.2 vectored', 'ardusub/run_native.py',
        ('--firmware-release', 'stable-4.7.2', '--surface', '--stress', '--ardupilot-root', '@stable-sub-root',
         '--app-revision', '{revision}'),
        ('stress_requested', True), ('depth_autonomy_probe', 'mavmoos_ardusub_control'),
        'ardusub/stress_transect.py', 'app_source'),
    'stress_px4_vtol': StressCase(
        'PX4 1.17.0 standard VTOL', 'px4_offboard/qualify_aircraft.py',
        ('--vehicle', 'standard-vtol', '--stress'), ('stress', True)),
    'stress_px4_ackermann': StressCase(
        'PX4 1.17.0 Ackermann', 'px4_rover/qualify_ackermann.py',
        ('--case', 'stress'), ('case', 'stress'), blocking=False),
}


def select(requested):
    names = tuple(CASES) if requested is None else requested
    require(len(names) == len(set(names)) and set(names) <= CASES.keys(),
            'invalid or duplicate stress case; use --stress --plan for available cases')
    return {name: case for name, case in CASES.items() if name in names}


def host_build(config, cases, jobs):
    targets = {target for case in cases.values() for target in case.targets}
    if not targets:
        return None
    require((platform.system(), platform.machine()) == ('Darwin', 'arm64'),
            'host stress fixtures require Darwin ARM64')
    build, moos = Path(config['build']).resolve(), Path(config['moos-root']).resolve()
    cache = build / 'CMakeCache.txt'
    require(cache_path(cache, 'CMAKE_HOME_DIRECTORY') == ROOT and cache_path(cache, 'MOOS_IVP_ROOT') == moos and
            sha(moos / 'bin/pHelmIvP') == HOST_HELM, 'host build or stock helm differs from the release input')
    targets.update(('iMAVLink', 'mavmoos_telemetry', 'mavmoos_plane_fault_tap'))
    return ['cmake', '--build', str(build), '--parallel', str(jobs), '--target', *sorted(targets)]


def command(case, config, stage, revision, *, replay=False):
    runner = ROOT / 'tests' / case.runner
    if case.linux:
        prefix = [sys.executable, '-B', str(runner)]
        if replay:
            return prefix + ['--audit', str(stage)], 90
        options = ['--mavlink-archive', config['mavlink-archive'], '--px4-volume', config['stress-firmware-volume']]
        if 'stress-stock-volume' in config:
            options.extend(('--stock-volume', config['stress-stock-volume']))
        return prefix + [*case.arguments, *options, '--output', str(stage)], 1200
    prefix = [config['fixture-python'], '-B']
    if replay:
        target = stage / 'result.json' if case.replay == 'quadplane_native/hover_helm.py' else stage
        return prefix + [str(ROOT / 'tests' / case.replay), str(target)], 90
    values = {**config, 'tap': str(Path(config['build']) / 'bin/libmavmoos_plane_fault_tap.dylib')}
    arguments = [values[arg[1:]] if arg.startswith('@') else arg.format(revision=revision) for arg in case.arguments]
    return ['/usr/bin/caffeinate', '-i', *prefix, str(runner), *arguments,
            '--build', config['build'], '--moos-root', config['moos-root'], '--output', str(stage)], 540


def check_identity(report, case, revision):
    key, expected = case.identity
    source = report.get(case.revision_field)
    if case.revision_field == 'source':
        require(isinstance(source, dict) and source.get('tracked_changes') == [], 'native source was dirty')
        source = source.get('revision')
    require(source == revision and report.get(key) == expected and report.get('hardware') is False,
            'stress fixture, scope or revision differs')


def check_ports(report):
    ports = report.get('ports_clear')
    require(isinstance(ports, dict) and ports and all(value is True for value in ports.values()) and
            report.get('owned_processes_remaining') == [] and not report.get('resume_error'),
            'stress fixture did not prove process and port cleanup')


def check_run(stage, report, case, revision):
    check_identity(report, case, revision)
    if case.linux:
        require(report.get('scope') == 'owned_sitl', 'PX4 stress evidence has the wrong scope')
        native = receipt(local(stage, 'native/result.json'))
        require(native.get('source_revision') == revision, 'PX4 native evidence has the wrong revision')
        check_ports(native)
        lease = receipt(local(stage, 'domain-lease.json'))
        require(lease.get('source_ref') == revision and lease.get('acquired') is lease.get('released') is True and
                lease.get('release_errors') == [], 'PX4 stress fixture did not release its lease')
        for name in ('build', 'native'):
            cleanup = receipt(local(stage, name + '-container-cleanup.json'))
            require(cleanup.get('removed') is True, 'PX4 stress fixture did not remove its container')
        require(native.get('artifact_drift') == [], 'PX4 stress inputs drifted')
        return native
    else:
        check_ports(report)
        require(not report.get('input_drift') and not report.get('cleanup_error') and not report.get('postflight_error'),
                'host stress inputs or cleanup differ')
        if report['result'] == 'PASS':
            native_files(stage, report.get('files'), ('results.txt',))
            stock = report.get('stock_moos_revision', report.get('moos_revision', report.get('moos_source')))
            require(stock == MOOS_REVISION, 'host stress stock MOOS revision differs')
    return report
