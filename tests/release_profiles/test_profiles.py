"""Fail-closed tests for release profile evidence and Linux isolation gates."""
import copy
from contextlib import redirect_stdout
import io
import json
import os
from pathlib import Path
import stat
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts/release'))
sys.path.insert(0, str(ROOT / 'packaging/release'))
import build_linux
import campaign
import kit
import local_docker
import matrix
import profiles
sys.path.insert(0, str(ROOT / 'tests/px4_offboard'))
from run_fixed_wing import stock_work_pins


class ReceiptTests(unittest.TestCase):
    def setUp(self):
        scratch = tempfile.TemporaryDirectory()
        self.addCleanup(scratch.cleanup)
        self.out = Path(scratch.name).resolve()
        self.wire = self.out / 'gcs.mavlink'
        self.wire.write_bytes(b'independent trace')
        log = self.out / 'aircraft.ulg'
        log.write_bytes(b'flight log')
        self.files = {'gcs.mavlink': profiles.sha(self.wire),
                      'aircraft.ulg': {'sha256': profiles.sha(log), 'bytes': log.stat().st_size}}
        self.receipt = {'result': 'PASS', 'owned_processes_remaining': [], 'ports_clear': {'db': True},
                        'files': self.files, 'hardware': False}

    def audit(self):
        (self.out / 'result.json').write_text(json.dumps(self.receipt))
        shape = (('a' * 40, 'b' * 64, 'quad'), {'independent_flight_m': 4.1}, ('gcs.mavlink',))
        with patch.object(profiles, 'px4_proofs', return_value=shape):
            return profiles.audit_native(self.out, 'px4_offboard', 'f' * 40, {})

    def test_audits_raw_identity(self):
        result = self.audit()
        self.assertEqual(result['native_receipt']['sha256'], profiles.sha(self.out / 'result.json'))
        self.assertEqual(result['firmware']['frame'], 'quad')
        self.assertTrue(result['cleanup_proven'])

    def test_rejects_protected_trace_tamper(self):
        self.wire.write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError, 'digest differs'):
            self.audit()

    def test_rejects_leaked_resources(self):
        for patch_values in ({'ports_clear': {}}, {'ports_clear': {'db': False}},
                             {'ports_clear': {'db': 1}}, {'owned_processes_remaining': [7]}):
            with self.subTest(changes=patch_values), patch.dict(self.receipt, patch_values):
                with self.assertRaisesRegex(ValueError, 'cleanup'):
                    self.audit()

    def test_rejects_unsafe_receipt_paths(self):
        (self.out / 'alias').symlink_to(self.wire)
        for path in ('../gcs.mavlink', '/gcs.mavlink', './gcs.mavlink', 'alias'):
            with self.subTest(path=path):
                with patch.dict(self.files, {path: self.files['gcs.mavlink']}):
                    with self.assertRaisesRegex(ValueError, 'unsafe|aliased'):
                        self.audit()


class ReleasePlanTests(unittest.TestCase):
    def setUp(self):
        self.plan = matrix.load_manifest((ROOT / matrix.MANIFEST).read_bytes(), campaign.RECEIPTS)
        self.paths = matrix.receipt_paths(self.plan, campaign.RECEIPTS)

    def test_plan_requires_each_stable_aircraft_and_excludes_experimental_rover(self):
        aircraft = {cell['scenario'] for cell in self.plan['cases'].values() if cell['fixture'] == 'px4_aircraft'}
        self.assertEqual(aircraft, {'quadrotor', 'fixed-wing', 'standard-vtol'})
        self.assertIn('px4_ackermann', self.plan['outside_release_matrix'])
        self.assertNotIn('px4_ackermann', self.paths)
        self.assertIn('software', self.paths)
        self.assertIn('linux_kit', self.paths)

    def test_release_stage_uses_supervisor_log_on_failure(self):
        revision = 'b' * 40
        case = 'software'
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            log = output / 'supervisor' / 'software.log'
            run = {'exit_code': 0, 'timeout': False, 'cleanup_proven': True,
                   'unexpected_live_descendants': 0, 'log': str(log)}
            result = {'schema': 1, 'source_revision': revision, 'result': 'PASS'}
            with patch.object(campaign, 'command', return_value=([], 1)), \
                    patch.object(campaign, 'supervise', return_value=run), \
                    patch.object(campaign, 'clean_revision', return_value=revision), \
                    patch.object(campaign, 'receipt', return_value=result) as read, redirect_stdout(io.StringIO()):
                campaign.launch(case, {}, output, revision, 1, self.paths, None)
                read.assert_called_once_with(output / self.paths[case])
                read.reset_mock()
                run['exit_code'] = 1
                with self.assertRaises(ValueError) as failed:
                    campaign.launch(case, {}, output, revision, 1, self.paths, None)
                self.assertIn(str(log), str(failed.exception))
                read.assert_not_called()

    def test_preflight_rejects_missing_inputs_and_aliased_volumes(self):
        with tempfile.TemporaryDirectory() as temporary:
            self.assertEqual(campaign.preflight({'build': temporary}, ('software',), {})['build'], temporary)
            with self.assertRaisesRegex(ValueError, 'stable-sub-root'):
                campaign.preflight({name: temporary for name in ('build', 'moos-root', 'fixture-python')},
                                   ('sub_vectored',), self.plan['cases'])
            config = {name: temporary for name in campaign.LINUX_INPUTS}
            config['linux-work-volume'] = config['stable-px4-package-volume'] = 'mavmoos-release-alias'
            with self.assertRaisesRegex(ValueError, 'volumes alias'):
                campaign.preflight(config, ('linux_kit', 'px4_quadrotor'), self.plan['cases'])

    def test_release_gate_requires_the_stable_stress_cases(self):
        cases = campaign.stress.select(None)
        self.assertEqual(len(cases), 7)
        stable = {name for name, case in cases.items() if case.blocking}
        self.assertEqual(set(cases) & self.paths.keys(), stable)
        self.assertEqual(len(self.paths) - 2, 28)
        self.assertEqual([name for name, case in cases.items() if not case.blocking], ['stress_px4_ackermann'])
        self.assertEqual(list(campaign.stress.select(['stress_copter'])), ['stress_copter'])
        with self.assertRaisesRegex(ValueError, 'duplicate stress case'):
            campaign.stress.select(['stress_copter', 'stress_copter'])
        experimental = {**self.plan, 'stress_cases': ['stress_px4_ackermann']}
        with self.assertRaisesRegex(ValueError, 'invalid release stress cases'):
            matrix.load_manifest(json.dumps(experimental), campaign.RECEIPTS)

    def test_ackermann_failure_is_advisory_only_after_proven_cleanup(self):
        revision = 'c' * 40
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for released in (True, False):
                with self.subTest(released=released):
                    output = root / str(released)
                    args = SimpleNamespace(case=['stress_px4_ackermann'], output=output, plan=False, jobs=1)

                    def simulator(name, folder, _revision, argv, _seconds):
                        self.assertEqual(name, 'stress_px4_ackermann')
                        self.assertIn('--mavlink-archive', argv)
                        stage = folder / name
                        (stage / 'native').mkdir(parents=True)
                        for filename, data in {
                            'qualification.json': {'source_revision': revision, 'case': 'stress', 'hardware': False,
                                                   'result': 'FAIL', 'scope': 'owned_sitl'},
                            'native/result.json': {'source_revision': revision, 'result': 'FAIL',
                                                   'ports_clear': {'db': True}, 'owned_processes_remaining': [],
                                                   'artifact_drift': []},
                            'domain-lease.json': {'source_ref': revision, 'acquired': True, 'released': released,
                                                  'release_errors': []},
                            'build-container-cleanup.json': {'removed': True},
                            'native-container-cleanup.json': {'removed': True},
                        }.items():
                            (stage / filename).write_text(json.dumps(data))
                        return {'cleanup_proven': True, 'timeout': False, 'unexpected_live_descendants': 0,
                                'exit_code': 1}

                    with patch.object(campaign, 'supervise', side_effect=simulator), \
                            patch.object(campaign, 'clean_revision', return_value=revision), redirect_stdout(io.StringIO()):
                        status = campaign.stress_campaign(args, {'mavlink-archive': str(root)}, revision)
                    report = json.loads((output / 'stress-result.json').read_text())
                    self.assertEqual(status, 0 if released else 1)
                    self.assertEqual(report['cases']['stress_px4_ackermann']['result'],
                                     'ADVISORY_FAIL' if released else 'FAIL')

    def test_pack_uses_exact_selected_roster_instead_of_fixed_count(self):
        modules = kit.standard_modules((ROOT / 'src/plugins/CMakeLists.txt').read_text())
        self.assertIn('plugins/mavmoos_arduquadplane_control.so', modules)
        self.assertIn('plugins/mavmoos_px4_rover_control.so', modules)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            elf = b'\x7fELF\x02\x01' + bytes(12) + (183).to_bytes(2, 'little')
            for name in modules | kit.PAYLOAD:
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(elf)
                path.chmod(0o755)
            kit.validate_payload(root, kit.inventory(root), modules)
            current = root / 'plugins/mavmoos_arduquadplane_control.so'
            current.rename(current.with_name('mavmoos_unexpected.so'))
            with self.assertRaisesRegex(ValueError, 'standard modules'):
                kit.validate_payload(root, kit.inventory(root), modules)

    def test_release_aircraft_uses_every_fresh_stock_pin(self):
        stable = json.loads((ROOT / 'tests/px4_offboard/firmware-v1.17.0.json').read_text())
        historical = stock_work_pins(stable)
        self.assertEqual(set(historical), set(profiles.KIT_STOCK))
        fresh = {name: 'f' * 64 for name in historical}
        self.assertEqual(stock_work_pins(stable, {'stock_witness_sha256': fresh}), fresh)
        self.assertNotEqual(historical, fresh)
        fresh.pop('/work/moos-ivp/bin/pLogger')
        with self.assertRaisesRegex(RuntimeError, 'stock dependency witnesses'):
            stock_work_pins(stable, {'stock_witness_sha256': fresh})


class Px4IsolationTests(unittest.TestCase):
    def setUp(self):
        self.name = 'mavmoos-release-px4-example'
        self.out = Path('/temporary-evidence')
        self.volumes = ('source-work', 'pinned-package')
        self.info = {'Image': profiles.TOOLS_IMAGE, 'Name': '/' + self.name,
                     'Config': {'Labels': {'mavmoos.scope': 'release-px4-offboard'}},
                     'HostConfig': {'NetworkMode': 'none', 'ReadonlyRootfs': True, 'Privileged': False,
                                    'CapDrop': ['ALL'], 'SecurityOpt': ['no-new-privileges'],
                                    'PidMode': '', 'IpcMode': 'private', 'UTSMode': '',
                                    'CgroupnsMode': 'private'},
                     'Mounts': [{'Destination': '/work', 'Type': 'volume', 'Name': self.volumes[0], 'RW': False},
                                {'Destination': '/opt/px4', 'Type': 'volume', 'Name': self.volumes[1], 'RW': False},
                                {'Destination': '/source', 'Type': 'bind', 'Source': str(profiles.ROOT), 'RW': False},
                                {'Destination': '/evidence', 'Type': 'bind', 'Source': str(self.out), 'RW': True}]}

    def check(self, info):
        profiles.isolated_px4_container(info, self.name, self.volumes, self.out)

    def test_accepts_exact_isolation(self):
        self.check(self.info)

    def test_accepts_only_exact_desktop_bind_aliases(self):
        for entry in self.info['Mounts'][2:]:
            entry['Source'] = '/host_mnt' + entry['Source']
        with patch.object(profiles.platform, 'system', return_value='Darwin'):
            self.check(self.info)
            self.info['Mounts'][2]['Source'] += '/other-source'
            with self.assertRaisesRegex(ValueError, 'mounts'):
                self.check(self.info)

    def test_rejects_altered_namespace_and_mounts(self):
        for key, value in (('NetworkMode', 'bridge'), ('Devices', ['/dev/ttyUSB0']),
                           ('Privileged', True), ('CapDrop', []), ('CgroupnsMode', 'host')):
            with self.subTest(key=key):
                candidate = copy.deepcopy(self.info)
                candidate['HostConfig'][key] = value
                with self.assertRaisesRegex(ValueError, 'isolation'):
                    self.check(candidate)
        for index in (0, 1, 2):
            with self.subTest(mount=index):
                candidate = copy.deepcopy(self.info)
                candidate['Mounts'][index]['RW'] = True
                with self.assertRaisesRegex(ValueError, 'mounts'):
                    self.check(candidate)


class DockerEndpointTests(unittest.TestCase):
    def test_rejects_remote_and_non_socket_contexts(self):
        with tempfile.TemporaryDirectory() as directory:
            regular = Path(directory) / 'regular'
            regular.touch()
            for host in ('ssh://user@example.invalid', 'tcp://127.0.0.1:2375',
                         'unix://relative', 'unix://' + str(regular)):
                with self.subTest(host=host), patch('local_docker.subprocess.check_output',
                                                     return_value=json.dumps(host)) as read:
                    with self.assertRaisesRegex(ValueError, 'Unix socket'):
                        local_docker.LocalDocker(sys.executable)
                    self.assertEqual(read.call_args.args[0][1:3], ['context', 'inspect'])
                    self.assertEqual(read.call_count, 1)

    def test_pins_resolved_socket_and_scrubs_docker_environment(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory).resolve() / 'socket'
            path.touch()
            alias = path.with_name('alias')
            alias.symlink_to(path)
            socket_info = SimpleNamespace(st_mode=stat.S_IFSOCK, st_dev=3, st_ino=4)
            with patch.dict(os.environ, {'DOCKER_HOST': 'ssh://elsewhere', 'DOCKER_CONTEXT': 'other',
                                         'DOCKER_TLS_VERIFY': '1', 'DOCKER_CONFIG': '/ambient'}), \
                    patch.object(Path, 'lstat', return_value=socket_info), \
                    patch('local_docker.subprocess.check_output',
                          side_effect=[json.dumps('unix://' + str(alias)), 'server\n']) as read:
                endpoint = local_docker.LocalDocker(sys.executable)
                self.addCleanup(endpoint.close)
                self.assertEqual(endpoint.capture('version'), 'server')
                command = read.call_args.args[0]
                self.assertEqual(command[1:5], ['--config', endpoint.config.name, '--host', 'unix://' + str(path)])
                self.assertFalse(any(name.startswith('DOCKER_') for name in read.call_args.kwargs['env']))
                socket_info.st_ino = 5
                with self.assertRaisesRegex(ValueError, 'socket changed'):
                    endpoint.capture('version')
                self.assertEqual(read.call_count, 2)


class OwnedDockerTests(unittest.TestCase):
    def test_recovery_uses_owned_identity_even_after_execution_policy_failure(self):
        owner, identity = 'owner', '1' * 64
        for role in ('builder', 'consumer', 'px4'):
            with self.subTest(role=role):
                name = owner + '-' + role
                row = {'Id': identity, 'Name': '/' + name, 'Config': {'Labels': {local_docker.OWNER_LABEL: owner}},
                       'HostConfig': {'NetworkMode': 'bridge', 'Privileged': True}}
                docker = Mock(side_effect=[identity, json.dumps([row]), '', '', ''])
                self.assertEqual(local_docker.remove_owned_container(docker, name, owner, None), identity)
                self.assertIn('name=^/' + name + '$', docker.call_args_list[0].args)
                self.assertIn('label=' + local_docker.OWNER_LABEL + '=' + owner, docker.call_args_list[0].args)
                self.assertEqual(docker.call_args_list[2], call('container', 'rm', '--force', identity))

    def test_refuses_container_with_replaced_owner_or_id(self):
        identity = '1' * 64
        row = {'Id': identity, 'Name': '/expected', 'Config': {'Labels': {local_docker.OWNER_LABEL: 'other'}}}
        for expected in (None, '2' * 64):
            with self.subTest(expected=expected):
                docker = Mock(side_effect=[identity, json.dumps([row])])
                with self.assertRaisesRegex(ValueError, 'identity differs'):
                    local_docker.remove_owned_container(docker, 'expected', 'owner', expected)
                self.assertEqual(docker.call_count, 2)
        docker = Mock(side_effect=['', identity])
        with self.assertRaisesRegex(ValueError, 'lost its name or owner label'):
            local_docker.remove_owned_container(docker, 'expected', 'owner', identity)

    def test_recovers_only_the_exact_owned_volume(self):
        row = {'Name': 'work', 'Driver': 'remote', 'Labels': {local_docker.OWNER_LABEL: 'owner'}}
        docker = Mock(side_effect=['other\nwork', json.dumps([row]), '', 'other'])
        self.assertTrue(local_docker.remove_owned_volume(docker, 'work', 'owner'))
        self.assertEqual(docker.call_args_list[2], call('volume', 'rm', 'work'))
        unrelated = Mock(return_value='unrelated')
        self.assertFalse(local_docker.remove_owned_volume(unrelated, 'work', 'owner'))
        self.assertEqual(unrelated.call_count, 1)
        row['Labels'] = None
        replaced = Mock(side_effect=['work', json.dumps([row])])
        with self.assertRaisesRegex(ValueError, 'identity differs'):
            local_docker.remove_owned_volume(replaced, 'work', 'owner')
        self.assertEqual(replaced.call_count, 2)


class LinuxCreateReplyTests(unittest.TestCase):
    def check_lost_reply(self, resource):
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory) / 'evidence'
            volume_name = 'mavmoos-release-unit' if resource == 'volume' else None
            args = SimpleNamespace(output=out, source_ref='a' * 40, image=build_linux.IMAGE,
                                   work_volume=volume_name, docker='docker', jobs=2)
            endpoint = SimpleNamespace(argv=lambda *parts: list(parts), env={}, evidence=lambda: {}, close=Mock())
            identity = '2' * 64
            state = {}
            calls = []

            def run(command, **_):
                calls.append(command)
                if command[:2] == ['image', 'inspect']:
                    output = json.dumps([{'Id': args.image, 'Architecture': 'arm64', 'Os': 'linux'}])
                elif command[:2] == ['volume', 'ls']:
                    output = state.get('volume', '')
                elif command[:2] == ['volume', 'create'] or command[0] == 'create':
                    assert 'mavmoos.release.owner=' in command[command.index('--label') + 1]
                    state['owner'] = command[command.index('--label') + 1].partition('=')[2]
                    intent = json.loads((out / 'owned-containers.json').read_text())
                    if resource == 'volume':
                        self.assertEqual(intent['volume_attempt'], volume_name)
                        state['volume'] = volume_name
                    else:
                        self.assertEqual(intent['containers'], {'builder': None})
                        state['container'] = identity
                        state['name'] = '/' + command[command.index('--name') + 1]
                    raise subprocess.TimeoutExpired(command, 25)
                elif command[:2] == ['container', 'ls']:
                    output = state.get('container', '')
                elif command[:2] == ['container', 'inspect']:
                    output = json.dumps([{'Id': identity, 'Name': state['name'],
                                          'Config': {'Labels': {local_docker.OWNER_LABEL: state['owner']}}}])
                elif command[:2] == ['volume', 'inspect']:
                    output = json.dumps([{'Name': volume_name,
                                          'Labels': {local_docker.OWNER_LABEL: state['owner']}}])
                elif command[:2] == ['container', 'rm'] or command[:2] == ['volume', 'rm']:
                    state.pop(resource)
                    output = ''
                else:
                    self.fail('unexpected Docker operation: ' + repr(command))
                return subprocess.CompletedProcess(command, 0, output, '')

            with patch.object(build_linux, 'LocalDocker', return_value=endpoint), \
                    patch.object(build_linux, 'git_file', return_value=Path(build_linux.__file__).read_bytes()), \
                    patch.object(build_linux, 'prepare', return_value={}), \
                    patch.object(build_linux.subprocess, 'run', side_effect=run), redirect_stdout(io.StringIO()):
                self.assertEqual(build_linux.build(args), 1)
            report = json.loads((out / 'result.json').read_text())
            self.assertEqual(report['result'], 'FAIL')
            self.assertNotIn(resource, state)
            self.assertFalse(any(command[0] in ('cp', 'start') for command in calls))
            if resource == 'volume':
                self.assertTrue(report['work_volume_removed'])
            else:
                self.assertTrue(report['container_removed'])
                self.assertEqual(report['removed_container_ids'], {'builder': identity})

    def test_builder_create_reply_loss(self):
        self.check_lost_reply('container')

    def test_volume_create_reply_loss(self):
        self.check_lost_reply('volume')


if __name__ == '__main__':
    unittest.main()
