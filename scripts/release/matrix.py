"""Launch and check the additional native cells in a local release matrix."""
from pathlib import Path
import platform
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = 'scripts/release/matrix.json'
sys.path.insert(0, str(ROOT / 'scripts'))
from ci.run import parse, require, sha
from profiles import (HOST_HELM, MOOS_REVISION, PLANE_REVISION, PLANE_SHA, ROVER_REVISION, ROVER_SHA,
                      cache_path, clean_revision, native_files, sub_proofs)
import stress

COPTER_REVISION = '92b0cd788ec29406f26c6f9c31d5ceedbd1cc538'
COPTER_SHA = '2486cd1c8342872b7ac8e90d0a53ff8941c58e0dbb41775d223cf644c33df563'
STABLE_REVISION = 'fb773567896014da007bca58358704d4a0d8516f'
STABLE_PLANE_SHA = 'd66fca61a6f4b696a5d70178da482bb9e75a3bc3a13a30bd34b755fa6049ffd8'
STABLE_SUB_SHA = '2f169160e96e284e9db770efd3b1b8a8bf89836c84013995bc2c6e623be0126d'
COPTER_MANIFEST = 'config/firmware/arducopter-4.6.3-quad.json'
STABLE_PX4_VOLUME = 'mavmoos-stable-px4-4b2eac609afc45619159fc755701a8a3-package'
STANDARD_NAMES = re.compile(r'[a-z][a-z0-9_]*\Z')
DIGEST = re.compile(r'[0-9a-f]{64}\Z')
SCENARIOS = {
    'copter_operations': ('hexa', 'octa'),
    'copter_auto': ('quad', 'hexa', 'octa'),
    'rover_mission': ('ground', 'boat'),
    'plane': ('lifecycle', 'native-autoland'),
    'sub': ('vectored', 'vectored_6dof'),
    'quadplane': ('live', 'live-auto', 'hover'),
    'px4_aircraft': ('quadrotor', 'fixed-wing', 'standard-vtol'),
}
INPUTS = {
    'copter_operations': ('behavior-dir', 'copter-root', 'copter-attestation'),
    'copter_auto': ('behavior-dir', 'copter-root', 'copter-attestation'),
    'rover_mission': ('rover-root',),
    'plane': ('behavior-dir', 'plane-build-root'),
    'sub': ('stable-sub-root',),
    'quadplane': ('behavior-dir', 'plane-build-root'),
    'px4_aircraft': (),
}


def load_manifest(value, legacy_cases):
    plan = parse(value)
    require(isinstance(plan, dict) and plan.get('schema') == 1, 'unknown release matrix schema')
    baseline, cases = plan.get('baseline_cases'), plan.get('cases')
    require(isinstance(baseline, list) and len(set(baseline)) == len(baseline) and baseline and
            set(baseline) <= set(legacy_cases) - {'software', 'linux_kit'}, 'invalid release baseline cases')
    require(isinstance(cases, dict) and cases and not set(cases) & set(legacy_cases), 'invalid release matrix cells')
    for name, cell in cases.items():
        require(STANDARD_NAMES.fullmatch(name) and isinstance(cell, dict) and
                set(cell) == {'fixture', 'scenario', 'profile'} and
                cell['scenario'] in SCENARIOS.get(cell['fixture'], ()) and
                isinstance(cell['profile'], str) and cell['profile'], f'invalid release cell: {name}')
    stress_cases = plan.get('stress_cases', [])
    require(isinstance(stress_cases, list) and
            all(isinstance(name, str) and name in stress.CASES and stress.CASES[name].blocking
                for name in stress_cases) and len(stress_cases) == len(set(stress_cases)) and
            not set(stress_cases) & (set(cases) | set(legacy_cases)), 'invalid release stress cases')
    exclusions = plan.get('outside_release_matrix')
    require(isinstance(exclusions, dict) and not set(exclusions) & (set(cases) | set(baseline) | set(stress_cases)) and
            all(STANDARD_NAMES.fullmatch(name) and isinstance(reason, str) and reason for name, reason in
                exclusions.items()), 'invalid release matrix exclusions')
    return plan


def receipt_paths(plan, legacy):
    if plan is None:
        return legacy
    cases = ('software', 'linux_kit', *plan['baseline_cases'])
    paths = {case: legacy[case] for case in cases}
    for case, cell in plan['cases'].items():
        name = 'profile-result.json' if cell['fixture'] == 'px4_aircraft' else 'result.json'
        paths[case] = f'matrix/{case}/{name}'
    for name in plan.get('stress_cases', ()):
        paths[name] = f'stress/{name}/{stress.CASES[name].receipt_name}'
    return paths


def host_input_keys(cell):
    keys = INPUTS[cell['fixture']]
    if cell['scenario'] == 'native-autoland':
        keys = ('behavior-dir', 'plane-stable-build-root')
    return ('build', 'moos-root', 'fixture-python', *keys)


def host_command(cell, config, stage, revision):
    fixture, scenario = cell['fixture'], cell['scenario']
    require((platform.system(), platform.machine()) == ('Darwin', 'arm64'),
            'host fixture pins require Darwin ARM64')
    build = Path(config['build']).resolve()
    moos = Path(config['moos-root']).resolve()
    cache = build / 'CMakeCache.txt'
    require(cache_path(cache, 'CMAKE_HOME_DIRECTORY') == ROOT and cache_path(cache, 'MOOS_IVP_ROOT') == moos and
            clean_revision(moos) == MOOS_REVISION and sha(moos / 'bin/pHelmIvP') == HOST_HELM,
            'host build or stock helm differs from the release input')
    argv = ['/usr/bin/caffeinate', '-i', str(Path(config['fixture-python']).absolute()), '-B']
    common = ['--build', str(build), '--moos-root', str(moos), '--output', str(stage)]
    if fixture in ('copter_operations', 'copter_auto'):
        inputs = ['--ardupilot-root', config['copter-root'], '--behavior-dir', config['behavior-dir'],
                  '--firmware-attestation', config['copter-attestation'], '--copter-frame', scenario]
        if fixture == 'copter_operations':
            runner, options = 'operations/run_operations.py', ['--parity']
            argv[:0] = ['/usr/bin/env', 'MAVLINK20=1']
        else:
            runner = 'mission_execution/run_native.py'
            options = ['--scenario', 'auto_typed_lifecycle', '--source-root', str(ROOT),
                       '--firmware-manifest', str(ROOT / COPTER_MANIFEST),
                       '--expected-app-sha256', sha(build / 'bin/iMAVLink'),
                       '--expected-probe-sha256', sha(build / 'bin/mission_execution_probe')]
        flags = [*inputs, *options]
    elif fixture == 'rover_mission':
        runner, flags = 'rover_mission/run_native.py', ['--ardupilot-root', config['rover-root'], '--profile', scenario]
    elif fixture == 'plane':
        key = 'plane-stable-build-root' if scenario == 'native-autoland' else 'plane-build-root'
        runner = 'plane_native/run_native.py'
        flags = ['--plane-build-root', config[key], '--behavior-dir', config['behavior-dir'],
                 '--case', scenario, '--expected-app-revision', revision]
    elif fixture == 'sub':
        runner = 'ardusub/run_native.py'
        flags = ['--ardupilot-root', config['stable-sub-root'], '--firmware-release', 'stable-4.7.2',
                 '--frame', scenario, '--surface', '--app-revision', revision]
    else:
        require(fixture == 'quadplane', 'unknown host release fixture')
        runner = 'quadplane_native/run_native.py'
        flags = ['--plane-build-root', config['plane-build-root'], '--behavior-dir', config['behavior-dir'],
                 '--case', scenario, '--app-revision', revision]
    return argv + [str(ROOT / 'tests' / runner), *common, *flags], 540


def digests(inputs, names):
    selected = {}
    for alias, suffix in names.items():
        matches = [digest for path, digest in inputs.items() if Path(path).as_posix().endswith('/' + suffix)]
        require(len(matches) == 1, f'missing or ambiguous native input artifact: {alias}')
        selected[alias] = matches[0]
    return selected


def copter_proof(raw, fixture, frame, revision):
    operations = fixture == 'copter_operations'
    identity = raw['firmware_identity']
    protected = raw['protected_inputs_sha256'] if operations else raw['artifacts_before']
    hashes = digests(protected, {'app': 'bin/iMAVLink', 'helm': 'bin/pHelmIvP'})
    require((raw['revision'] if operations else raw['source_ref']) == revision and
            (raw['moos_revision'] if operations else raw['moos_ref']) == MOOS_REVISION and
            (raw['copter_sha256'] if operations else raw['native_sha256']) == COPTER_SHA and
            identity['source_revision'] == COPTER_REVISION and identity['native_sha256'] == COPTER_SHA and
            identity['profile_id'] == 'arducopter-4.6.3-quad' and
            raw.get('frame', 'quad') == frame and raw['command'][1:3] == ['--model', frame] and
            raw['probe_exit'] == 0 and raw['cleanup_proven'] is True and raw['artifact_drift'] == [] and
            hashes['app'] == raw['app_sha256'], 'Copter frame, firmware, source or inputs differ')
    proofs = raw['independent_fc']
    if operations:
        precision = raw['independent_precision_loiter']
        require(raw['parity'] is True and raw['independent_monitor_writes'] == 'none' and
                proofs['final_disarmed'] is proofs['final_on_ground'] is True and precision['travelled_m'] > 0 and
                raw['parity_helm'] and raw['parity_evidence'], 'Copter operator or precision flight proof is incomplete')
        proofs = {**proofs, 'precision': precision}
        required = ('fc-monitor.mavlink', 'workflow.log')
    else:
        require(raw['scenario'] == 'auto_typed_lifecycle' and raw['observer_writes'] == 0 and
                raw['artifacts_before'] == raw['artifacts_after'] and raw['firmware_identity_after'] == identity and
                proofs['result'] == 'PASS' and proofs['observed_landed_disarmed_guided'] is True and
                proofs['resume_travel_m'] > 0 and raw['dataflash_evidence'], 'Copter typed AUTO proof is incomplete')
        required = ('fc-observer.mavlink', 'probe.jsonl', 'firmware-source.json', 'firmware-attestation.json')
    return (COPTER_REVISION, COPTER_SHA, frame), hashes, raw['loaded_modules_sha256'], proofs, required


def rover_proof(raw, scenario, revision):
    model = 'motorboat-skid' if scenario == 'boat' else 'rover'
    provider = 'ardurover_boat' if scenario == 'boat' else 'ardurover'
    before = raw['digests_before']
    hashes = digests(before, {'app': 'bin/iMAVLink', 'helm': 'bin/pHelmIvP', 'native': 'bin/ardurover'})
    names = [f'mavmoos_{name}.so' for name in ('telemetry', 'ardurover_control', 'missions')]
    modules = digests(before, {name: 'plugins/' + name for name in names})
    require(raw['source']['revision'] == revision and raw['source']['tracked_changes'] == [] and
            raw['firmware'] == ROVER_REVISION and raw['stock_moos'] == MOOS_REVISION and
            raw['profile'] == provider and raw['model'] == model and hashes['native'] == ROVER_SHA and
            raw['digests_after'] == before and raw['monitor_writes'] == 0,
            'Rover mission source, input, or model differs')
    phases = raw['phases']
    require(all(phases.get(phase) for phase in ('pause_resume_completion', 'abort', 'takeover_handback',
                                               'restart_handback', 'stock_helm_after_handback')) and
            len(set(raw['bridge_pids'])) == 2 and raw['independent_dataflash']['MODE'],
            'Rover pause, takeover, restart or handback proof is incomplete')
    return (ROVER_REVISION, ROVER_SHA, model), hashes, modules, phases, ('fc.mavlink', 'moos.jsonl')


def plane_proof(raw, scenario, revision):
    firmware = (STABLE_REVISION, STABLE_PLANE_SHA) if scenario == 'native-autoland' else (PLANE_REVISION, PLANE_SHA)
    hashes = raw['input_sha256']
    modules = {'mavmoos_' + key.removeprefix('module_') + '.so': digest
               for key, digest in hashes.items() if key.startswith('module_')}
    proof = raw['workflow']
    require(raw['runner_revision'] == revision and raw['case'] == scenario and raw['model'] == 'plane' and
            raw['profile'] == 'arduplane' and raw['moos_revision'] == MOOS_REVISION and
            (raw['firmware_revision'], hashes['native']) == firmware and hashes == raw['input_sha256_after'] and
            raw['monitor_writes'] == 0 and raw['owned_mission_processes_remaining'] == [],
            'Plane lifecycle source, firmware, or input differs')
    require(all(proof.get(key) for key in ('stock_producer', 'stock_target', 'direct_waypoint', 'landing_mode',
                                          'touchdown', 'landed', 'explicit_disarm_after_landing')) and
            raw['dataflash'] and (scenario != 'native-autoland' or proof.get('independent_landing_mode')),
            'Plane lifecycle, approach, or ground release proof is incomplete')
    return (*firmware, 'plane'), hashes, modules, proof, ('monitor-rx.mavlink', 'app-wire-0.bin', 'moos-probe.jsonl')


def sub_surface_proof(raw, scenario, revision):
    hashes = {name: item['sha256'] for name, item in raw['inputs'].items()}
    modules = {f'mavmoos_{name}.so': hashes[key] for name, key in
               (('ardusub_control', 'sub_plugin'), ('telemetry', 'telemetry_plugin'))}
    proof = raw['surface']
    require(raw['frame'] == scenario and raw['surface_requested'] is True and
            proof['entered']['custom_mode'] == 9 and proof['native_depth_hold']['custom_mode'] == 2 and
            proof['ascended_m'] >= 1 and -.6 <= proof['final_depth_m'] <= .35,
            'Sub frame, measured Surface, or Depth Hold proof differs')
    firmware, transit, files = sub_proofs(raw, revision, {**hashes, 'modules': modules},
                                         firmware=(STABLE_REVISION, STABLE_SUB_SHA, scenario))
    return firmware, hashes, modules, {**transit, 'surface': proof}, files


def quadplane_proof(raw, scenario, revision):
    hashes = {name: item['sha256'] for name, item in raw['inputs'].items()}
    modules = {f'mavmoos_{name}.so': hashes[key] for name, key in
               (('arduquadplane_control', 'control'), ('telemetry', 'telemetry'), ('missions', 'missions'))}
    proof = raw['wire_audit']
    require(raw['case'] == scenario and raw['app_revision'] == revision and raw['moos_revision'] == MOOS_REVISION and
            raw['firmware_revision'] == PLANE_REVISION and hashes['native'] == PLANE_SHA and
            raw['cleanup_proven'] is True and raw['input_drift'] == [] and raw['dataflash'] and
            proof['monitor_crc_frames'] > 0 and
            (proof['live_targets'] == 0 if scenario == 'live-auto' else proof['live_targets'] > 0),
            'QuadPlane firmware, handoff, or wire proof differs')
    required = ('monitor-rx.mavlink', 'original.plf', 'phases.jsonl', 'moos.jsonl')
    if scenario == 'hover':
        adapter = 'scripts/mavmoos-hover.lua'
        initial, returned = raw['initial_motion'], raw['return_motion']
        require(proof['stable_phase_and_unique_offer'] is True and proof['target_readbacks'] >= 6 and
                initial['east_m'] >= 7 and initial['up_m'] >= 1.3 and returned['east_m'] >= 7 and
                abs(returned['end']['position']['relative_alt'] - 65000) <= 1500 and
                raw['producer_silence']['hold']['heartbeat']['custom_mode'] == 19 and
                raw['forward_transition']['phase']['state']['vtol_state'] == 1 and
                raw['forward_flight']['state']['vtol_state'] == 4 and
                raw['native_return']['hold']['state']['vtol_state'] == 3 and
                raw['pilot_takeover']['heartbeat']['custom_mode'] == 5 and
                raw['touchdown']['state']['landed_state'] == 1 and
                raw['files'][adapter] == hashes['adapter'], 'QuadPlane hover motion or handback proof differs')
        proof = {**proof, 'first_east_m': initial['east_m'], 'first_up_m': initial['up_m'],
                 'return_east_m': returned['east_m']}
        required += (adapter,)
    return (PLANE_REVISION, PLANE_SHA, 'quadplane'), hashes, modules, proof, required


def host_observations(raw, directory, cell, revision):
    require(raw['result'] == 'PASS' and raw['hardware'] is False and not raw.get('postflight_error') and
            not raw.get('cleanup_error') and not raw.get('verification_error') and raw['owned_processes_remaining'] == []
            and isinstance(raw['ports_clear'], dict) and raw['ports_clear'] and
            all(value is True for value in raw['ports_clear'].values()),
            'owned native fixture failed or did not prove teardown')
    fixture, scenario = cell['fixture'], cell['scenario']
    if fixture in ('copter_operations', 'copter_auto'):
        observed = copter_proof(raw, fixture, scenario, revision)
    else:
        check = {'rover_mission': rover_proof, 'plane': plane_proof, 'sub': sub_surface_proof,
                 'quadplane': quadplane_proof}[fixture]
        observed = check(raw, scenario, revision)
    (firmware, native, frame), hashes, modules, proofs, required = observed
    require(hashes['helm'] == HOST_HELM and DIGEST.fullmatch(hashes['app']) and modules and
            all(DIGEST.fullmatch(digest) for digest in modules.values()),
            'native app, modules or stock helm identity differs')
    native_files(directory, raw.get('files', raw.get('artifact_sha256')), required)
    return {'platform': 'darwin-arm64', 'moos_revision': MOOS_REVISION, 'app_sha256': hashes['app'],
            'helm_sha256': hashes['helm'], 'modules_sha256': modules, 'profile': cell['profile'],
            'firmware': {'revision': firmware, 'native_sha256': native, 'frame': frame}, 'proofs': proofs}
