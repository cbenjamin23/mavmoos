#!/usr/bin/env python3
"""Exercise release composition with a complete fixture and fail-closed cases."""
import hashlib
import io
import json
from pathlib import Path
import sys
import tarfile
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts/release'))
from profiles import (KIT_HELM, KIT_PROBE, KIT_STOCK, MOOS_REVISION, PLANE_REVISION, PLANE_SHA, PX4_REVISION, PX4_SHA,
                      ROVER_REVISION, ROVER_SHA, SUB_REVISION, SUB_SHA, native_observations)
from matrix import HOST_HELM as MATRIX_HELM
from qualify import (COPTER_FILES, KIT_CONSUMER_REJECTIONS, NATIVE_CASES, RAW_REVISION,
                     RECEIPTS, check_cohorts, copter_observations, observed_artifacts, qualify)

REVISION = 'a' * 40
MOOS = MOOS_REVISION
HOST_APP = 'c' * 64
LINUX_APP_BYTES = b'packaged app fixture'
LINUX_APP = hashlib.sha256(LINUX_APP_BYTES).hexdigest()
HOST_HELM = 'e' * 64
LINUX_HELM = 'f' * 64
CASE = 'ctest.release_evidence'
REGISTRY = json.dumps({'tests': [{'id': CASE, 'required': True,
                                  'tiers': ['software', 'smoke', 'full']}]}).encode()


def digest(data):
    return hashlib.sha256(data).hexdigest()


class QualificationTests(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory(prefix='mavmoos-release-evidence.')
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name).resolve()
        self.base = {'schema': 1, 'result': 'PASS', 'source_revision': REVISION}
        self.write_software()
        self.write_kit()
        for case in NATIVE_CASES:
            self.write_native(case)

    def write(self, name, data):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
        return {'path': path.name, 'sha256': digest(data)}

    def write_json(self, name, data):
        return self.write(name, json.dumps(data).encode())

    def write_supervision(self, directory, label):
        log = self.write(directory / (label + '.log'), b'complete')
        processes = self.write_json(directory / (label + '.log.processes.json'), {
            'schema': 1, 'cleanup_proven': True, 'remaining': [], 'unexpected_live_descendants': [],
            'cleanup_failure': None})
        return {'log': '/old/campaign/' + log['path'], 'log_sha256': log['sha256'],
                'process_evidence': '/old/campaign/' + processes['path'],
                'process_evidence_sha256': processes['sha256'], 'exit_code': 0,
                'timeout': False, 'cleanup_proven': True, 'unexpected_live_descendants': 0}

    def write_software(self):
        self.write('software/junit.xml',
                   f'<testsuite><testcase name="{CASE}" status="run" time="1"/></testsuite>'.encode())
        command = self.write_supervision(Path('software'), 'ctest')
        app = {'/owned/build/bin/iMAVLink': HOST_APP,
               '/owned/build/plugins/mavmoos_telemetry.so': '2' * 64,
               '/owned/build/plugins/mavmoos_air_data.so': '2' * 64,
               '/owned/build/plugins/mavmoos_ardusub_control.so': '3' * 64}
        self.write_json(RECEIPTS['software'], {
            **self.base, 'tier': 'software', 'hosted': False, 'native_evidence': [],
            'source_revision_after': REVISION, 'artifact_drift': [], 'input_drift': [],
            'registry_sha256': digest(REGISTRY), 'selected_ids': [CASE],
            'cases': [{'id': CASE, 'result': 'PASS', 'duration_s': 1.0}],
            'artifacts_before': app, 'artifacts_after': app,
            'commands': [command]})

    def context(self, linux):
        return {'moos_revision': MOOS, 'platform': 'linux-arm64' if linux else 'darwin-arm64',
                'app_sha256': LINUX_APP if linux else HOST_APP,
                'helm_sha256': LINUX_HELM if linux else HOST_HELM}

    def write_kit(self):
        payload = {'bin/iMAVLink': LINUX_APP_BYTES, 'lib/libMOOSGeodesy.so': b'geodesy',
                   **{f'plugins/mavmoos_module{i:02}.so': f'module{i}'.encode() for i in range(34)}}
        manifest = {'schema': 1, 'source_revision': REVISION, 'platform': 'linux-arm64', 'module_count': 34,
                    'files': {name: {'bytes': len(content), 'sha256': digest(content)}
                              for name, content in payload.items()}}
        self.write_json('linux/package-manifest.json', manifest)
        archive = io.BytesIO()
        with tarfile.open(fileobj=archive, mode='w:gz') as bundle:
            for name, content in {**payload, 'manifest.json': json.dumps(manifest).encode()}.items():
                member = tarfile.TarInfo('mavmoos/' + name)
                member.size = len(content)
                bundle.addfile(member, io.BytesIO(content))
        package = {**self.write('linux/runtime.tar.gz', archive.getvalue()), 'bytes': len(archive.getvalue())}
        loaded = {f'mavmoos_module{i:02}.so': manifest['files'][f'plugins/mavmoos_module{i:02}.so']['sha256']
                  for i in range(8)}
        consumer = self.write_json('linux/consumer/consumer-result.json', {
            **self.base, 'module_count': 34, 'prefix': '/work/installed-elsewhere/mavmoos',
            'interfaces': {'lo': 9, 'sit0': 128}, 'app_sha256': LINUX_APP,
            'package_sha256': package['sha256'], 'geodesy_sha256': digest(b'geodesy'),
            'loaded_module_hashes': loaded, 'rx_only_udp_egress': 0, 'missing_module_app_rejected': True,
            **{key: reason.format(count=34) for key, reason in KIT_CONSUMER_REJECTIONS.items()}})
        witness = {'owned_processes_remaining': [], 'cleanup_errors': [], 'cleanup_proven': True,
                   'ports_clear': {'moos': True, 'bridge': True, 'peer': True}, 'bridge_egress_datagrams': 0}
        self.write_json('linux/consumer/relocated/result.json',
                        {**witness, 'result': 'PASS', 'loaded_modules_sha256': loaded})
        self.write_json('linux/consumer/missing-module/result.json', {**witness, 'result': 'FAIL'})
        self.write('linux/consumer/missing-module/probe.log', b'state=CONFIG_ERROR,reason=plugin_configuration_failed')
        self.write_json(RECEIPTS['linux_kit'], {**self.base, **self.context(True), 'package': package,
                                              'module_count': 34, 'container_removed': True,
                                              'consumer_result': 'consumer/consumer-result.json',
                                              'consumer_result_sha256': consumer['sha256'],
                                              'build_witness_sha256': {'/work/moos-ivp/bin/pHelmIvP': LINUX_HELM}})

    def write_index(self, directory, names):
        content = b'original owned evidence'
        for name in names:
            self.write(directory / name, content)
        return {name: {'bytes': len(content), 'sha256': digest(content)} for name in names}

    def copter_raw(self, directory, raw, context):
        app, helm = context['app_sha256'], context['helm_sha256']
        files = self.write_index(directory, (*COPTER_FILES, 'logs/sample.BIN'))
        inputs = {'native': '2' * 64, 'app': app, 'pHelmIvP': helm,
                  'firmware_manifest': files['firmware-manifest.json']['sha256'],
                  'firmware_attestation': files['firmware-attestation.json']['sha256']}
        lifecycle = {'final_disarmed': True, 'final_on_ground': True}
        proofs = {'lifecycle': lifecycle, 'short_handoff': {'seconds': .25},
                  'expired_handoff': {'explicit_reacquire': True}, 'stationkeeping': {'drift_m': .05},
                  'pilot_takeover': {'stale_navigation_fenced': True, 'explicit_reacquire': True},
                  'in_flight_parameter_read': {'nav_before': 4, 'nav_after': 6}}
        return {**raw, 'files': files, 'release_case': 'copter_operator', 'timewarp': 1, 'probe_exit': 0,
                'cleanup_proven': True, 'moos_revision': MOOS, 'app_sha256': app, 'helm_sha256': helm,
                'ports_clear': dict.fromkeys(('bridge', 'monitor', 'rc', 'peer', 'operator', 'operator_peer',
                                             'monitor_peer', 'irlock', 'moos_tcp', 'base_tcp'), True),
                'ardupilot_revision': '1' * 40, 'copter_sha256': inputs['native'],
                'firmware_identity': {'profile_id': 'arducopter-4.6.3-quad', 'frame': 'quad',
                                      'fixture': {'hardware': False}, 'source_revision': '1' * 40,
                                      'native_sha256': inputs['native'],
                                      'platform': {'system': 'Darwin', 'machine': 'arm64'},
                                      'manifest_sha256': files['firmware-manifest.json']['sha256'],
                                      'attestation_sha256': files['firmware-attestation.json']['sha256']},
                'inputs_sha256': inputs, 'inputs_sha256_after': inputs, 'independent_monitor': lifecycle,
                'loaded_modules_sha256': {'mavmoos_telemetry.so': '2' * 64},
                'proofs': proofs, 'independent_pilot_rcin': {'input_path': '/old/campaign/logs/sample.BIN',
                                                           'input_sha256': files['logs/sample.BIN']['sha256']}}

    def profile_raw(self, case, directory, raw, context):
        app, helm = context['app_sha256'], context['helm_sha256']
        if case.startswith('rover_'):
            boat = case == 'rover_boat'
            wire = {'stock_east_north_m': [3, 4], 'reacquired_motion_m':
                    dict.fromkeys(('allstop_reacquired', 'manual_reacquired', 'link_reacquired'), 1),
                    'guided_expiry_witness': {'link_recovery': {'firmware_elapsed_ms': 3100, 'final_speed_m_s': .2}},
                    'observed_modes': [0, 15], 'returns': {}, 'boat_native_loiter':
                    {'loiter_seconds': 26, 'correction_servo_samples': 1, 'loiter_max_displacement_m': 2,
                     'unpowered_projection_m': 3}}
            return {**raw, 'profile': 'ardurover_boat' if boat else 'ardurover',
                    'model': 'motorboat-skid' if boat else 'rover', 'mav_type': 11 if boat else 10, 'probe_exit': 0,
                    'mavmoos_harness_revision': REVISION, 'mavmoos_cmake_source':
                    {'path': '/old/checkout', 'revision': REVISION, 'tracked_changes': []},
                    'ardupilot_revision': ROVER_REVISION, 'native_sha256': ROVER_SHA, 'moos_revision': MOOS,
                    'app_sha256': app, 'helm_sha256': helm, 'digests_before': {}, 'digests_after': {},
                    'loaded_modules_sha256': {'mavmoos_telemetry.so': '2' * 64}, 'monitor_writes': 0,
                    'independent_monitor': wire, 'independent_dataflash':
                    {'pilot_manual': {'Mode': 0}, 'explicit_guided_recovery': {'ModeNum': 15},
                     'native_loiter_modes': [5]},
                    'files': self.write_index(directory, ('sample.BIN', 'fc-monitor.mavlink', 'workflow.jsonl'))}
        if case.startswith('plane_'):
            hashes = {'native': PLANE_SHA, 'app': app, 'helm': helm, 'module_air_data': '2' * 64}
            return {**raw, 'case': case.removeprefix('plane_'), 'profile': 'arduplane', 'model': 'plane',
                    'firmware_revision': PLANE_REVISION, 'moos_revision': MOOS, 'input_sha256': hashes,
                    'input_sha256_after': hashes, 'monitor_writes': 0, 'owned_mission_processes_remaining': [],
                    'workflow': {'first_track': {'displacement_m': 70}, 'second_track': {'displacement_m': 70},
                                 'normal_handoff_s': .4, 'healthy_pilot_seconds': 8,
                                 'watchdog_heartbeats_in_fbwa': 7, 'late_target_delivered_mono': 1},
                    'mode_cause_evidence': {'explicit_rtl': {'ModeNum': 11}, 'pilot_takeover': {'ModeNum': 5, 'Rsn': 1}},
                    'files': self.write_index(directory,
                                              ('sample.BIN', 'monitor-rx.mavlink', 'app-wire-0.bin', 'moos-probe.jsonl'))}
        if case == 'sub':
            hashes = {'simulator': SUB_SHA, 'app': app, 'helm': helm, 'sub_plugin': '3' * 64,
                      'telemetry_plugin': '2' * 64}
            return {**raw, 'fixture_source': REVISION, 'firmware_source': SUB_REVISION, 'moos_source': MOOS,
                    'inputs': {name: {'sha256': sha} for name, sha in hashes.items()}, 'input_drift': [],
                    'cleanup_proven': True, 'transit': {'east_m': 3.1, 'north_m': .1, 'depth_m': 2},
                    'native_link_loss': {'custom_mode': 2}, 'pilot_takeover': {'native': {'custom_mode': 19},
                                                                              'nav_counter_frozen': 1},
                    'captured_hold_readback': {'error_m': .1}, 'helm_silence': {'drift_m': .1},
                    'operator_handback': {'mode': 16},
                    'files': self.write_index(directory, ('sample.BIN', 'monitor-rx.mavlink', 'moos.jsonl'))}
        hold, offboard = (4 << 16) | (3 << 24), 6 << 16
        outer = directory.parent
        container = '7' * 64
        self.write_json(outer / 'container-before.json', {'Id': container, 'HostConfig': {'NetworkMode': 'none'}})
        self.write_json(outer / 'container-after.json', {'Id': container, 'State': {'Running': False, 'ExitCode': 0}})
        self.write_json(outer / 'container-cleanup.json', {'container': container, 'removed': True})
        return {**raw, 'firmware_revision': PX4_REVISION, 'artifacts_before': {
            '/opt/px4/bin/px4': PX4_SHA, '/work/mavmoos-build/bin/iMAVLink': app,
            '/work/moos-ivp/bin/pHelmIvP': helm,
            '/work/mavmoos-build/plugins/mavmoos_module00.so': digest(b'module0')},
            'artifact_drift': [], 'resume_error': None,
            'namespace': {'ipv4_routes': [], 'cap_eff': '0' * 16, 'no_new_privileges': True,
                          'root_read_only': True, 'interface_flags': {'lo': 1}}, 'prestream_count': 11,
            'travel': {'east_m': 5}, 'second_acquisition': {'travel': {'east_m': 5},
                'prestream_count': 11, 'independent_offboard': {'mode': offboard}},
            'independent_offboard': {'mode': offboard}, 'independent_loss_hold': {'mode': hold},
            'independent_release_hold': {'mode': hold}, 'failsafe_current_mode': {'intended_mode': offboard},
            'handback_current_mode': {'intended_mode': hold},
            'denied_failsafe_acquire': {'value': 'reason=acquisition_rejected'}, 'landed_disarmed': True,
            'files': self.write_index(directory, ('sample.ulg', 'gcs.mavlink', 'moos-wire.jsonl', 'startup.log'))}

    def write_native(self, case, **overrides):
        name = RECEIPTS[case]
        stage = Path(name).parent
        directory = stage if case == 'copter_operator' else stage / 'native'
        context = {**self.context(case == 'px4_offboard'), **overrides}
        raw = {**self.base, RAW_REVISION[case]: REVISION, 'hardware': False,
               'ports_clear': {'db': True}, 'owned_processes_remaining': []}
        if case == 'copter_operator':
            raw = self.copter_raw(directory, raw, context)
            firmware, proofs = copter_observations(raw, self.root / directory, context)
        else:
            raw = self.profile_raw(case, directory, raw, context)
            firmware, proofs = native_observations(raw, self.root / directory, case, REVISION,
                                                   observed_artifacts(raw, case, context))
            self.write_json(stage / 'launcher-result.json', self.write_supervision(stage, 'launcher'))
        self.write_json(name, {**self.base, **context, 'case': case, 'cleanup_proven': True,
                               'firmware': firmware, 'proofs': proofs})
        self.write_raw(case, raw)

    def raw_path(self, case):
        stage = Path(RECEIPTS[case]).parent
        return stage / ('result.json' if case == 'copter_operator' else 'native/result.json')

    def write_raw(self, case, raw):
        path = self.raw_path(case)
        record = self.write_json(path, raw)
        self.update(case, native_receipt={**record, 'path': path.relative_to(Path(RECEIPTS[case]).parent).as_posix()})

    def alter_raw(self, case, **fields):
        raw = json.loads((self.root / self.raw_path(case)).read_text())
        self.write_raw(case, {**raw, **fields})

    def update(self, case, **fields):
        name = RECEIPTS[case]
        self.write_json(name, {**json.loads((self.root / name).read_text()), **fields})

    def audit(self):
        return qualify(self.root, REVISION, REGISTRY, MOOS, 34)

    def assert_rejected(self, case, message):
        result = self.audit()
        self.assertEqual(result['result'], 'FAIL', result)
        self.assertIn(message, result['errors'][case])

    def test_full_candidate(self):
        result = self.audit()
        self.assertEqual(result['result'], 'PASS', result)
        self.assertEqual(set(result['cells']), set(RECEIPTS))

    def test_incomplete(self):
        (self.root / RECEIPTS['px4_offboard']).unlink()
        result = self.audit()
        self.assertEqual(result['result'], 'INCOMPLETE')
        self.assertEqual(result['missing'], ['px4_offboard'])

    def test_mixed_source(self):
        self.update('sub', source_revision='3' * 40)
        self.assert_rejected('sub', 'different MAVMOOS commit')

    def test_raw_revision_mismatch(self):
        self.alter_raw('sub', app_source='3' * 40)
        self.assert_rejected('sub', 'underlying native receipt')

    def test_mixed_host_or_linux_artifact(self):
        for case, field in [('sub', 'app_sha256'), ('px4_offboard', 'helm_sha256')]:
            with self.subTest(case=case):
                self.write_native(case, **{field: '4' * 64})
                self.assert_rejected('cohort', 'differs')
                self.write_native(case)

    def test_normalized_firmware_does_not_replace_native_identity(self):
        self.update('plane_takeover', firmware={'revision': '7' * 40, 'frame': 'plane',
                                               'native_sha256': '2' * 64})
        self.assert_rejected('plane_takeover', 'normalized firmware differs')

    def test_same_commit_different_native_app_or_helm(self):
        for case, field in [('copter_operator', 'app_sha256'), ('rover_ground', 'helm_sha256')]:
            with self.subTest(case=case):
                self.alter_raw(case, **{field: '4' * 64})
                self.assert_rejected(case, 'differ')
                self.write_native(case)

    def test_raw_cleanup_overrides_normalized_claim(self):
        self.alter_raw('copter_operator', cleanup_proven=False)
        self.assert_rejected('copter_operator', 'teardown differs')

    def test_same_app_different_native_modules(self):
        self.alter_raw('copter_operator', loaded_modules_sha256={'mavmoos_telemetry.so': '8' * 64})
        self.assert_rejected('cohort', 'native host module differs')
        self.write_native('copter_operator')
        raw = json.loads((self.root / self.raw_path('px4_offboard')).read_text())
        raw['artifacts_before']['/work/mavmoos-build/plugins/mavmoos_module00.so'] = '8' * 64
        self.write_raw('px4_offboard', raw)
        self.assert_rejected('cohort', 'PX4 native module differs')

    def test_same_commit_different_native_case(self):
        ground = json.loads((self.root / self.raw_path('rover_ground')).read_text())
        self.write_raw('rover_boat', ground)
        self.assert_rejected('rover_boat', 'Rover model')

    def test_relabelled_proof_does_not_replace_native_observation(self):
        self.update('sub', proofs={'pilot_takeover': True})
        self.assert_rejected('sub', 'normalized workflow proofs differ')

    def test_mutated_package(self):
        self.write('linux/runtime.tar.gz', b'wrong bytes')
        self.assert_rejected('linux_kit', 'bytes differ')

    def test_wrong_package_length(self):
        report = json.loads((self.root / RECEIPTS['linux_kit']).read_text())
        self.update('linux_kit', package={**report['package'], 'bytes': -1})
        self.assert_rejected('linux_kit', 'package length')

    def test_missing_or_mutated_consumer_receipt(self):
        path = self.root / 'linux/consumer/consumer-result.json'
        path.unlink()
        self.assert_rejected('linux_kit', 'evidence file missing')
        self.write_kit()
        path.write_text('{}')
        self.assert_rejected('linux_kit', 'artifact bytes differ')

    def test_consumer_must_match_tested_kit_and_prove_fail_closed(self):
        for key, value in (('app_sha256', '5' * 64), ('rx_only_udp_egress', 1),
                           ('missing_module_app_rejected', False)):
            with self.subTest(key=key):
                path = Path('linux/consumer/consumer-result.json')
                consumer = json.loads((self.root / path).read_text())
                record = self.write_json(path, {**consumer, key: value})
                self.update('linux_kit', consumer_result_sha256=record['sha256'])
                self.assert_rejected('linux_kit', 'consumer receipt lacks tested kit identity')
                self.write_kit()
        self.write('linux/consumer/missing-module/probe.log',
                   b'state=CONFIG_ERROR,reason=plugin_configuration_failed\nCAPTURE\tMAVMOOS_CAPABILITIES\tbad')
        self.assert_rejected('linux_kit', 'did not fail closed')

    def test_mutated_onboard_trace(self):
        self.write('profiles/sub/native/sample.BIN', b'truncated')
        self.assert_rejected('sub', 'evidence length differs')

    def test_missing_required_raw_trace_in_consistent_index(self):
        raw = json.loads((self.root / self.raw_path('copter_operator')).read_text())
        raw['files'].pop('operator-rx.mavlink')
        self.write_raw('copter_operator', raw)
        self.assert_rejected('copter_operator', 'native raw evidence index is incomplete')

    def test_aliased_raw_evidence(self):
        path = self.root / self.raw_path('copter_operator')
        actual = path.rename(self.root / 'different-case.json')
        path.symlink_to(actual)
        self.assert_rejected('copter_operator', 'aliased')

    def test_missing_workflow_proof_and_cleanup(self):
        self.update('copter_operator', proofs={'lifecycle': True})
        self.assert_rejected('copter_operator', 'missing Copter operator proof')
        self.write_native('copter_operator')
        self.update('rover_boat', cleanup_proven=False)
        self.assert_rejected('rover_boat', 'cleanup failed')

    def test_profile_supervisor_and_px4_host_cleanup(self):
        self.write('profiles/sub/launcher.log.processes.json', b'altered')
        self.assert_rejected('sub', 'artifact bytes differ')
        self.write_native('sub')
        self.write_json('profiles/px4_offboard/container-cleanup.json', {'container': '7' * 64, 'removed': False})
        self.assert_rejected('px4_offboard', 'PX4 container identity or teardown differs')

    def test_supervisor_must_match_raw_process_cleanup(self):
        stage = Path('profiles/rover_ground')
        record = json.loads((self.root / stage / 'launcher-result.json').read_text())
        raw = json.loads((self.root / stage / 'launcher.log.processes.json').read_text())
        adverse = self.write_json(stage / 'launcher.log.processes.json', {**raw, 'remaining': [{'pid': 1}]})
        self.write_json(stage / 'launcher-result.json', {**record, 'process_evidence_sha256': adverse['sha256']})
        self.assert_rejected('rover_ground', 'raw process ownership or cleanup failed')

    def test_stale_software_or_skipped_case(self):
        self.update('software', registry_sha256='0' * 64)
        self.assert_rejected('software', 'different registry')
        self.write_software()
        self.write('software/junit.xml',
                   f'<testsuite><testcase name="{CASE}" time="0"><skipped/></testcase></testsuite>'.encode())
        self.assert_rejected('software', 'failed/skipped')
        self.write('software/junit.xml', b'<testsuite>')
        self.assert_rejected('software', 'no element found')

    def test_unknown_schema(self):
        self.update('px4_offboard', schema=True)
        self.assert_rejected('px4_offboard', 'unknown receipt schema')

    def matrix_fixture(self):
        case = 'quadplane_live'
        directory = Path(f'matrix/{case}')
        plan = {'baseline_cases': ['copter_operator'], 'outside_release_matrix': {'px4_ackermann': 'experimental'},
                'cases': {case: {'fixture': 'quadplane', 'scenario': 'live', 'profile': 'QuadPlane'}}}
        for name in ('software', 'linux_kit', 'copter_operator', case):
            self.write(Path('supervisor') / (name + '.started'), (REVISION + '\n').encode())
            self.write_json(Path('supervisor') / (name + '.json'), self.write_supervision(Path('supervisor'), name))
        modules = {'telemetry': '2' * 64, 'arduquadplane_control': '8' * 64, 'missions': '9' * 64}
        software = json.loads((self.root / RECEIPTS['software']).read_text())
        for module, sha256 in modules.items():
            for position in ('before', 'after'):
                software['artifacts_' + position]['/owned/build/plugins/mavmoos_' + module + '.so'] = sha256
        self.write_json(RECEIPTS['software'], software)
        kit = json.loads((self.root / RECEIPTS['linux_kit']).read_text())
        kit['build_witness_sha256'].update({name: 'c' * 64 for name in (*KIT_STOCK, KIT_PROBE) if name != KIT_HELM})
        rows = [{'path': name, 'sha256': digest, 'format': 'ELF64', 'machine': 183}
                for name, digest in kit['build_witness_sha256'].items()]
        self.write_json('linux/build-witness-artifacts.json', rows)
        kit['input_sha256'] = {name: digest(name.encode()) for name in ('moos-ivp.tar', 'altitude.tar', 'mavmoos.tar')}
        sources = [{'archive': f'/inputs/{name}', 'sha256': digest, 'original_files': 2, 'changed_files': []}
                   for name, digest in kit['input_sha256'].items()]
        self.write_json('linux/stock-source-check.json', {'result': 'PASS', 'checks': sources[:2]})
        self.write_json('linux/final-source-check.json', {'result': 'PASS', 'checks': sources})
        self.write_json(RECEIPTS['linux_kit'], kit)
        # The exact Darwin stock helm is shared with the original native stage.
        self.alter_raw('copter_operator', helm_sha256=MATRIX_HELM)
        raw = json.loads((self.root / self.raw_path('copter_operator')).read_text())
        raw['inputs_sha256']['pHelmIvP'] = raw['inputs_sha256_after']['pHelmIvP'] = MATRIX_HELM
        self.write_raw('copter_operator', raw)
        self.update('copter_operator', helm_sha256=MATRIX_HELM)
        files = {name: self.write(directory / name, b'independent sample')['sha256'] for name in (
            'flight.BIN', 'monitor-rx.mavlink', 'original.plf', 'phases.jsonl', 'moos.jsonl')}
        inputs = {'native': PLANE_SHA, 'app': HOST_APP, 'helm': MATRIX_HELM, 'control': modules['arduquadplane_control'],
                  'telemetry': modules['telemetry'], 'missions': modules['missions']}
        native = {'result': 'PASS', 'case': 'live', 'hardware': False, 'app_revision': REVISION,
                  'firmware_revision': PLANE_REVISION, 'moos_revision': MOOS, 'cleanup_proven': True,
                  'input_drift': [], 'owned_processes_remaining': [], 'ports_clear': {'monitor': True},
                  'inputs': {key: {'sha256': digest} for key, digest in inputs.items()}, 'dataflash': {'modes': [12]},
                  'wire_audit': {'monitor_crc_frames': 200, 'live_targets': 20}, 'files': files}
        self.write_json(directory / 'result.json', native)
        return plan, directory

    def test_matrix_full_and_started_failure_are_distinct(self):
        plan, directory = self.matrix_fixture()
        run = lambda: qualify(self.root, REVISION, REGISTRY, MOOS, 34, matrix=plan)
        actual = run()
        self.assertEqual(actual['result'], 'PASS', actual)
        self.assertEqual(actual['required_cases'], ['software', 'linux_kit', 'copter_operator', 'quadplane_live'])
        self.assertIn('px4_ackermann', actual['outside_release_matrix'])
        self.assertEqual(actual['cells']['quadplane_live']['firmware']['frame'], 'quadplane')
        (self.root / directory / 'result.json').unlink()
        actual = run()
        self.assertEqual(actual['result'], 'FAIL')
        self.assertIn('started stage lacks', actual['errors']['quadplane_live'])
        (self.root / 'supervisor/quadplane_live.started').unlink()
        actual = run()
        self.assertEqual((actual['result'], actual['missing']), ('INCOMPLETE', ['quadplane_live']))

    def test_matrix_rejects_corrupted_trace_or_stale_stage(self):
        plan, directory = self.matrix_fixture()
        self.write(directory / 'original.plf', b'changed')
        run = lambda: qualify(self.root, REVISION, REGISTRY, MOOS, 34, matrix=plan)
        self.assertIn('digest differs', run()['errors']['quadplane_live'])
        self.write('supervisor/quadplane_live.started', b'different source\n')
        self.assertIn('different commit', run()['errors']['quadplane_live'])

    def test_release_stress_requires_native_evidence_and_supervised_replay(self):
        plan, _ = self.matrix_fixture()
        case = 'stress_plane'
        plan['stress_cases'] = [case]
        run = lambda: qualify(self.root, REVISION, REGISTRY, MOOS, 34, matrix=plan)
        self.assertEqual(run()['missing'], [case])
        stage = Path('stress') / case
        supervisor = Path('supervisor')
        self.write(supervisor / f'{case}.started', (REVISION + '\n').encode())
        self.write_json(supervisor / f'{case}.json', self.write_supervision(supervisor, case))
        files = {name: self.write(stage / name, b'raw observation')['sha256']
                 for name in ('flight.BIN', 'results.txt')}
        self.write_json(stage / 'result.json', {
            'result': 'PASS', 'case': 'helm-mode-stress', 'runner_revision': REVISION,
            'moos_revision': MOOS, 'hardware': False, 'ports_clear': {'probe': True},
            'owned_processes_remaining': [], 'files': files,
        })
        self.assertIn('stress_plane.replay.started', run()['errors'][case])
        replay = f'{case}.replay'
        self.write(supervisor / f'{replay}.started', (REVISION + '\n').encode())
        record = self.write_supervision(supervisor, replay)
        log = self.write_json(supervisor / f'{replay}.log', {'result': 'PASS'})
        self.write_json(supervisor / f'{replay}.json', {**record, 'log_sha256': log['sha256']})
        qualified = run()
        self.assertEqual(qualified['result'], 'PASS', qualified)
        self.assertIn('replay', qualified['cells'][case])
        self.write(supervisor / f'{replay}.log', b'changed')
        self.assertIn('artifact bytes differ', run()['errors'][case])

    def test_matrix_rejects_linux_probe_drift(self):
        context = {**self.context(True), 'modules_sha256': {'module': 'b' * 64}}
        report = {'linux_kit': {**context, 'probe_sha256': 'a' * 64},
                  'px4_fixed_wing': {**context, 'probe_sha256': 'b' * 64}}
        with self.assertRaisesRegex(ValueError, 'native probe differs'):
            check_cohorts(report)

    def test_matrix_rejects_missing_kit_stock_witness_and_edited_source(self):
        plan, _ = self.matrix_fixture()
        run = lambda: qualify(self.root, REVISION, REGISTRY, MOOS, 34, matrix=plan)
        witnesses = json.loads((self.root / 'linux/build-witness-artifacts.json').read_text())
        self.write_json('linux/build-witness-artifacts.json', [row for row in witnesses if '/pLogger' not in row['path']])
        self.assertIn('stock or probe witnesses', run()['errors']['linux_kit'])
        self.write_json('linux/build-witness-artifacts.json', witnesses)
        stock = json.loads((self.root / 'linux/stock-source-check.json').read_text())
        stock['checks'][0]['changed_files'] = ['MOOSDB.cpp']
        self.write_json('linux/stock-source-check.json', stock)
        self.assertIn('source check differs', run()['errors']['linux_kit'])

    def test_matrix_rejects_linux_non_helm_stock_drift(self):
        stock = {name: 'a' * 64 for name in KIT_STOCK}
        context = {**self.context(True), 'modules_sha256': {'module': 'b' * 64}, 'probe_sha256': 'a' * 64}
        report = {'linux_kit': {**context, 'stock_witness_sha256': stock},
                  'px4_fixed_wing': {**context, 'stock_witness_sha256': {**stock, '/work/moos-ivp/bin/nsplug': 'd' * 64}}}
        with self.assertRaisesRegex(ValueError, 'native stock files differ'):
            check_cohorts(report)


if __name__ == '__main__':
    unittest.main()
