#!/usr/bin/env python3
"""Read-only source-snapshot and inactive-workflow contract tests.

Local temporary Git archives are exercised; no Docker image/container, hosted
workflow, socket or native vehicle is launched by these tests.
"""
import argparse
import copy
import importlib.util
import io
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'scripts/ci'))
spec = importlib.util.spec_from_file_location('_source_linux_recipe', ROOT / 'scripts/linux_reproduce.py')
recipe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recipe)
spec = importlib.util.spec_from_file_location('_source_linux_ci', ROOT / 'scripts/ci/linux.py')
linux = importlib.util.module_from_spec(spec)
spec.loader.exec_module(linux)
from ci import isolation


class SnapshotTests(unittest.TestCase):
    def make_tar(self, path, entries):
        with tarfile.open(path, 'w') as bundle:
            for name, content, kind in entries:
                info = tarfile.TarInfo(name)
                if kind == 'file':
                    info.size = len(content)
                    bundle.addfile(info, io.BytesIO(content))
                else:
                    info.type = tarfile.SYMTYPE
                    info.linkname = content.decode()
                    bundle.addfile(info)

    def test_reads_selected_snapshot_bytes(self):
        with tempfile.TemporaryDirectory(prefix='mavmoos-ci-snapshot-') as name:
            path = Path(name) / 'source.tar'
            self.make_tar(path, [('config/firmware/profile.json', b'exact archived bytes', 'file')])
            self.assertEqual(recipe.snapshot_file(path, 'config/firmware/profile.json'), b'exact archived bytes')

    def test_missing_duplicate_nonregular_and_escaping_snapshot_inputs_fail(self):
        with tempfile.TemporaryDirectory(prefix='mavmoos-ci-snapshot-') as name:
            path = Path(name) / 'source.tar'
            cases = [([], 'missing'), ([('x', b'a', 'file'), ('x', b'b', 'file')], 'x'),
                     ([('x', b'/unrelated', 'symlink')], 'x'),
                     ([('../x', b'a', 'file')], '../x'),
                     ([('/x', b'a', 'file')], '/x')]
            for entries, selected in cases:
                self.make_tar(path, entries)
                with self.subTest(selected=selected, entries=entries), self.assertRaises(ValueError):
                    recipe.snapshot_file(path, selected)

    def test_exact_source_commit_metadata_is_preserved(self):
        with tempfile.TemporaryDirectory(prefix='mavmoos-ci-git-') as name:
            root = Path(name)
            source = root / 'source'
            source.mkdir()
            subprocess.run(['git', 'init', '--quiet', str(source)], check=True)
            (source / 'original.txt').write_text('original source\n')
            subprocess.run(['git', '-C', str(source), 'add', 'original.txt'], check=True)
            subprocess.run(['git', '-C', str(source), '-c', 'user.name=Inert CI fixture',
                '-c', 'user.email=ci-fixture@example.invalid', 'commit', '--quiet', '-m', 'source'], check=True)
            selected = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
            recipe.metadata_archive(source, selected, root / 'metadata.git', root / 'metadata.tar')
            copy = root / 'copy'
            copy.mkdir()
            (copy / '.git').mkdir()
            (copy / 'original.txt').write_text('original source\n')
            with tarfile.open(root / 'metadata.tar') as bundle:
                bundle.extractall(copy / '.git', filter='data')
            subprocess.run(['git', '--git-dir', str(copy / '.git'), 'config', 'core.bare', 'false'], check=True)
            subprocess.run(['git', '--git-dir', str(copy / '.git'), 'config', 'core.worktree', str(copy)], check=True)
            subprocess.run(['git', '-C', str(copy), 'read-tree', 'HEAD'], check=True)
            self.assertEqual(subprocess.check_output(['git', '-C', str(copy), 'rev-parse', 'HEAD'], text=True).strip(), selected)
            self.assertEqual(subprocess.check_output(['git', '-C', str(copy), 'status', '--porcelain'], text=True), '')


class LinuxCommandTests(unittest.TestCase):
    def arguments(self, **changes):
        value = argparse.Namespace(source_root=Path('/trusted/source'),
            output=Path('/fresh/output'), moos_source=Path('/trusted/moos'),
            uav_source=Path('/trusted/uav'), tier='software', docker='docker',
            ardupilot_source=None, source_ref=None, firmware_manifest=None,
            image=None, mavlink_archive=None)
        for key, field in changes.items():
            setattr(value, key, field)
        return value

    def test_software_linux_hook_is_explicit(self):
        command = linux.recipe_command(self.arguments())
        self.assertEqual(command[command.index('--ci-tier') + 1], 'software')
        self.assertNotIn('--ardupilot-source', command)
        self.assertNotIn('--keep-container', command)

    def test_native_requires_exact_source_selection_inputs(self):
        for changes in ({'tier': 'smoke'}, {'tier': 'full', 'ardupilot_source': Path('/ap')},
                        {'tier': 'smoke', 'ardupilot_source': Path('/ap'), 'source_ref': 'a' * 40}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                linux.recipe_command(self.arguments(**changes))

    def test_native_profile_selection_is_structured(self):
        command = linux.recipe_command(self.arguments(tier='smoke', ardupilot_source=Path('/ap'),
            source_ref='a' * 40, firmware_manifest='config/firmware/stable.json'))
        self.assertEqual(command[command.index('--source-ref') + 1], 'a' * 40)
        self.assertEqual(command[command.index('--ci-firmware-manifest') + 1], 'config/firmware/stable.json')

    def test_isolated_domain_requires_original_source_image_and_offline_archive(self):
        args = self.arguments(tier='full', ardupilot_source=Path('/ap'), source_ref='a' * 40,
            firmware_manifest='config/firmware/baseline.json', image='sha256:' + 'b' * 64,
            mavlink_archive=Path('/pinned/archive'), isolated_native_domain=True)
        self.assertIn('--isolated-native-domain', linux.recipe_command(args))
        for field, value in (('source_ref', 'moving-tag'), ('image', 'image:latest'),
                             ('mavlink_archive', None), ('tier', 'software')):
            changed = copy.copy(args); setattr(changed, field, value)
            with self.subTest(field=field), self.assertRaises(ValueError): linux.recipe_command(changed)

    def test_native_recipe_missing_prerequisites_fails_before_output_or_docker(self):
        with tempfile.TemporaryDirectory(prefix='mavmoos-ci-prerequisite-') as name:
            output = Path(name) / 'not-created'
            result = subprocess.run([sys.executable, '-B', str(ROOT / 'scripts/linux_reproduce.py'),
                '--moos-source', '/not-used', '--uav-source', '/not-used', '--output', str(output),
                '--ci-tier', 'smoke'], capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('native CI requires', result.stderr)
            self.assertFalse(output.exists())

    def test_container_recovery_requires_exact_label_name_and_id(self):
        owner = {'schema': 1, 'name': 'mavmoos-linux-12345', 'label_key': 'mavmoos.ci.owner',
                 'label_value': 'mavmoos-linux-12345', 'container_id': 'a' * 64}
        value = [{'Id': 'a' * 64, 'Name': '/mavmoos-linux-12345', 'Config': {'Labels': {
            'mavmoos.ci.owner': 'mavmoos-linux-12345', 'mavmoos.scope': 'clean-linux-20261103'}}}]
        self.assertEqual(linux.owned_container(owner, value), 'a' * 64)
        import copy
        mutations = [lambda v: v[0].update(Id='b' * 64), lambda v: v[0].update(Name='/unrelated'),
                     lambda v: v[0]['Config']['Labels'].update({'mavmoos.ci.owner': 'other-run'}),
                     lambda v: v[0]['Config']['Labels'].pop('mavmoos.scope'), lambda v: v.clear()]
        for mutate in mutations:
            changed = copy.deepcopy(value)
            mutate(changed)
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                linux.owned_container(owner, changed)


class IsolationTests(unittest.TestCase):
    """Inert metadata/lease controls; no Docker, app, FC or packet launch."""
    def fixture(self):
        owner = {'schema': 1, 'name': 'mavmoos-linux-12345', 'label_key': 'mavmoos.ci.owner',
                 'label_value': 'mavmoos-linux-12345', 'container_id': 'a' * 64}
        image = 'sha256:' + 'b' * 64
        config = {'Id': 'a' * 64, 'Name': '/' + owner['name'], 'Image': image,
            'Config': {'User': '65534:65534', 'Labels': {'mavmoos.ci.owner': owner['name'],
                                                      'mavmoos.scope': isolation.SCOPE}},
            'HostConfig': {'NetworkMode': 'bridge', 'PidMode': '', 'IpcMode': 'private', 'UTSMode': '',
                'CgroupnsMode': 'private', 'Privileged': False,
                'CapAdd': None, 'CapDrop': ['ALL'], 'SecurityOpt': ['no-new-privileges'],
                'NanoCpus': 4000000000, 'Memory': 6442450944, 'MemorySwap': 6442450944,
                'PidsLimit': 512}, 'Mounts': [], 'NetworkSettings': {'Ports': {}, 'Networks': {}}}
        namespace = {'uid': 65534, 'gid': 65534, 'pid1_cmdline': ['sleep', 'infinity'],
            'cap_eff': '0000000000000000', 'no_new_privileges': '1',
            'cgroup': {'cpu.max': '400000 100000', 'memory.max': '6442450944',
                       'memory.swap.max': '0', 'pids.max': '512'},
            'interface_details': {'lo': {'flags': '0x9', 'ipv4': '127.0.0.1', 'address_error': None},
                                  'tunl0': {'flags': '0x80', 'ipv4': None, 'address_error': 'EADDRNOTAVAIL'}},
            'ipv6_addresses': ['00000000000000000000000000000001 01 80 10 80 lo'],
            'routes4': [], 'routes6': ['00000000000000000000000000000001 lo'],
            'socket_rows': {name: [] for name in ('udp', 'udp6', 'tcp', 'tcp6')},
            'namespaces': {name: name + ':[12345]' for name in ('pid', 'net', 'mnt', 'ipc', 'uts', 'cgroup')},
            'devices': ['null', 'urandom', 'tty']}
        return owner, image, config, namespace

    def test_effective_disconnection_accepts_down_unaddressed_kernel_tunnel(self):
        owner, image, config, namespace = self.fixture()
        self.assertEqual(isolation.validate_container(owner, [config], image, True), owner['container_id'])
        proof = isolation.validate_namespace(namespace)
        self.assertEqual(proof['result'], 'PASS')
        self.assertIn('not literal network-none', proof['network_claim'])
        self.assertEqual(config['HostConfig']['NetworkMode'], 'bridge')

    def test_shared_exposure_privilege_budget_or_network_rejects(self):
        owner, image, original, _ = self.fixture()
        mutations = [lambda v: v.update(Id='c' * 64), lambda v: v.update(Image='sha256:' + 'c' * 64),
            lambda v: v['Config']['Labels'].update({'mavmoos.scope': 'other'}),
            lambda v: v['HostConfig'].update(PidMode='host'), lambda v: v['HostConfig'].update(IpcMode='host'),
            lambda v: v['HostConfig'].update(UTSMode='host'), lambda v: v['HostConfig'].update(CgroupnsMode='host'),
            lambda v: v['HostConfig'].update(NetworkMode='host'), lambda v: v.update(Mounts=[{'Source': '/host'}]),
            lambda v: v['HostConfig'].update(Binds=['/host:/work']),
            lambda v: v['HostConfig'].update(Devices=[{'PathOnHost': '/dev/ttyUSB0'}]),
            lambda v: v['HostConfig'].update(DeviceRequests=[{'Capabilities': [['gpu']]}]),
            lambda v: v['HostConfig'].update(Privileged=True), lambda v: v['HostConfig'].update(CapAdd=['NET_ADMIN']),
            lambda v: v['Config'].update(User='0'), lambda v: v['HostConfig'].update(SecurityOpt=[]),
            lambda v: v['HostConfig'].update(NanoCpus=8000000000),
            lambda v: v['HostConfig'].update(Memory=12884901888),
            lambda v: v['HostConfig'].update(MemorySwap=-1), lambda v: v['HostConfig'].update(PidsLimit=0),
            lambda v: v['NetworkSettings'].update(Networks={'bridge': {}}),
            lambda v: v['HostConfig'].update(PortBindings={'14550/udp': [{'HostPort': '14550'}]})]
        for mutate in mutations:
            value = copy.deepcopy(original); mutate(value)
            with self.subTest(value=value), self.assertRaises(ValueError):
                isolation.validate_container(owner, [value], image, True)

    def test_nonloopback_address_route_listener_or_unknown_state_rejects(self):
        _, _, _, original = self.fixture()
        mutations = [lambda v: v['interface_details']['tunl0'].update(flags='0x81'),
            lambda v: v['interface_details']['tunl0'].update(ipv4='192.168.1.2', address_error=None),
            lambda v: v['interface_details']['lo'].update(ipv4='192.168.1.2'),
            lambda v: v.update(ipv6_addresses=['fe800000000000000000000000000001 02 40 20 80 eth0']),
            lambda v: v.update(routes4=['eth0 00000000 0100007F']),
            lambda v: v.update(routes6=['00000000000000000000000000000000 eth0']),
            lambda v: v['socket_rows']['udp6'].append('listener'), lambda v: v['socket_rows'].pop('tcp6'),
            lambda v: v['cgroup'].update(**{'memory.swap.max': '100'}),
            lambda v: v.update(cap_eff='0000000000000001'), lambda v: v.update(no_new_privileges='0'),
            lambda v: v.update(pid1_cmdline=['host-process']), lambda v: v['devices'].append('ttyUSB0'),
            lambda v: v['namespaces'].pop('net')]
        for mutate in mutations:
            value = copy.deepcopy(original); mutate(value)
            with self.subTest(value=value), self.assertRaises(ValueError): isolation.validate_namespace(value)
        changed = copy.deepcopy(original); changed['namespaces']['net'] = 'net:[54321]'
        with self.assertRaises(ValueError): isolation.validate_namespace(changed, original)

    def test_closed_loopback_tcp_time_wait_is_reported_not_false_listener_failure(self):
        _, _, _, value = self.fixture()
        ipv4 = '0: 0100007F:C001 0100007F:C002 06 00000000:00000000 03:0000001A 00000000 0 0 0 3 0000000000000000'
        ipv6 = '1: 00000000000000000000000001000000:C003 00000000000000000000000001000000:C004 06 00000000:00000000 03:0000001A 00000000 0 0 0 3 0000000000000000'
        value['socket_rows'].update(tcp=[ipv4], tcp6=[ipv6])
        self.assertEqual(isolation.validate_namespace(value)['closed_tcp_time_wait_counts'], {'tcp': 1, 'tcp6': 1})
        self.assertEqual(value['socket_rows']['tcp'][0], ipv4)
        for rows, version in (([ipv4.replace(' 06 ', ' 0A ')], False),
                              ([ipv4.replace(' 06 ', ' 01 ')], False),
                              ([ipv4.replace('0100007F:C002', '0101A8C0:C002')], False),
                              ([ipv6.replace('00000000000000000000000001000000:C004',
                                             '00000000000000000000000002000000:C004')], True),
                              (['malformed row'], False), ([ipv4 + ' unexpected'], False),
                              ([' '.join(ipv4.split()[:10])], False),
                              ([ipv4.replace(' 0 0 0 3 ', ' 0 0 99 3 ')], False)):
            with self.subTest(rows=rows, ipv6=version), self.assertRaises(ValueError):
                isolation.closed_tcp_rows(rows, version)

    def test_failed_fresh_identity_or_disconnection_proof_blocks_following_stage(self):
        owner, _, _, _ = self.fixture()
        for failure in ('identity', 'disconnect', 'namespace', None):
            events = []
            def inspect(label, disconnected=False, namespace=False):
                events.append(label)
                if failure == ('namespace' if namespace else 'identity'):
                    raise ValueError('deliberate proof failure')
                return {'result': 'PASS'}, {'namespaces': {}}
            def run(argv):
                events.append(argv)
                if failure == 'disconnect': raise ValueError('disconnect failed')
            def stage():
                recipe.disconnect_owned_bridge('docker', owner, run, inspect)
                events.append('app-stage')
            if failure is None:
                stage(); self.assertEqual(events[-1], 'app-stage')
                self.assertEqual(events[1], ['docker', 'network', 'disconnect', 'bridge', 'a' * 64])
            else:
                with self.assertRaises(ValueError): stage()
                self.assertNotIn('app-stage', events)
                if failure == 'identity': self.assertEqual(events, ['isolation-before-disconnect'])

    def test_fixed_linux_domain_lease_is_exclusive_and_released(self):
        with tempfile.TemporaryDirectory(prefix='mavmoos-linux-lease-') as name:
            root = Path(name); paths = [root / 'first.lock', root / 'second.lock']
            first = isolation.LinuxDomainLease(root / 'first.json', 'a' * 40, paths)
            second = isolation.LinuxDomainLease(root / 'second.json', 'a' * 40, paths)
            first.acquire()
            try:
                self.assertTrue(first.record['acquired'])
                self.assertFalse(first.record['released'])
                self.assertEqual(first.record['pid'], __import__('os').getpid())
                with self.assertRaises(BlockingIOError): second.acquire()
                self.assertFalse(second.record['acquired'])
            finally: first.release()
            second.acquire(); second.release()
            self.assertTrue(first.record['released']); self.assertTrue(second.record['released'])

    def test_partial_lease_failure_releases_only_owned_handles(self):
        with tempfile.TemporaryDirectory(prefix='mavmoos-linux-lease-') as name:
            root = Path(name); first, second = root / 'first.lock', root / 'second.lock'
            blocker = isolation.LinuxDomainLease(root / 'blocker.json', 'a' * 40, [second]); blocker.acquire()
            failed = isolation.LinuxDomainLease(root / 'failed.json', 'a' * 40, [first, second])
            try:
                with self.assertRaises(BlockingIOError): failed.acquire()
                probe = isolation.LinuxDomainLease(root / 'probe.json', 'a' * 40, [first])
                probe.acquire(); probe.release()
                self.assertFalse(failed.record['acquired']); self.assertTrue(failed.record['released'])
            finally: blocker.release()

    def test_lease_does_not_follow_lock_symlink(self):
        with tempfile.TemporaryDirectory(prefix='mavmoos-linux-lease-') as name:
            root = Path(name); target = root / 'target'; target.write_text('preserve')
            link = root / 'link'; link.symlink_to(target)
            lease = isolation.LinuxDomainLease(root / 'receipt.json', 'a' * 40, [link])
            with self.assertRaises(OSError): lease.acquire()
            self.assertEqual(target.read_text(), 'preserve')


class InactiveWorkflowTests(unittest.TestCase):
    def test_manual_only_activation_and_no_shell_input_interpolation(self):
        value = (ROOT / '.github/workflows/ci.yml').read_text()
        self.assertIn('workflow_dispatch:', value)
        self.assertNotIn('\n  push:', value)
        self.assertNotIn('\n  pull_request:', value)
        self.assertNotIn('\n  schedule:', value)
        self.assertIn('contents: read', value)
        self.assertIn('persist-credentials: false', value)
        block = value.split('        run: |', 1)[1].split('      - name:', 1)[0]
        self.assertNotIn('${{', block)
        self.assertIn('subprocess.run(command, check=True)', block)

    def test_action_refs_are_immutable_and_failure_artifacts_retained(self):
        value = (ROOT / '.github/workflows/ci.yml').read_text()
        import re
        references = re.findall(r'uses:\s+([^\s]+)', value)
        self.assertEqual(len(references), 2)
        self.assertTrue(all(re.fullmatch(r'actions/[a-z-]+@[0-9a-f]{40}', item) for item in references))
        self.assertIn('if: always()', value)
        self.assertIn('if-no-files-found: error', value)


if __name__ == '__main__':
    unittest.main(verbosity=2)
