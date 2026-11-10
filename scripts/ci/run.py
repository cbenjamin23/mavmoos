#!/usr/bin/env python3
"""Run a local qualification tier or an exact software case list.

The CTest graph is copied before inventory so historical LastTest logs are not
modified. All selected tests run in a fresh private CTest plan. No remote,
container, physical route or firmware is created by this entry point.
"""
import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import shutil
import shlex
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
ID = re.compile(r'[a-z][a-z0-9_.-]{0,127}\Z')
TIERS = {'software', 'smoke', 'full'}
FOCUSED_TIERS = {'focused', 'focused-native'}
SMOKE_WORKFLOW = 'copter_sitl_workflow'
COPTER_PARITY_PROFILE = 'arducopter-4.6.3-quad'
COPTER_PARITY_REVISION = '92b0cd788ec29406f26c6f9c31d5ceedbd1cc538'
NATIVE_INVENTORY_TIMEOUT = 15
MANUAL = {'distance_flight', 'protocol_faults', 'copter_scenarios', 'mission_execution', 'imu_observations'}
NATIVE_REQUIRED = {'native.distance_flight', 'native.protocol.wind_readback_loss',
                   'native.protocol.mission_ack_loss',
                   'native.copter.route_hold_return', 'native.copter.wind_repeated_cycle',
                   'native.copter.auto_complete_handback', 'native.copter.auto_pause_abort', 'native.imu.rx_only'}
ROVER_WORKFLOWS = {'native_rover_ground': ('ground', 'ardurover', 10, 'rover'),
                   'native_rover_boat': ('boat', 'ardurover_boat', 11, 'motorboat-skid')}
ROVER_NATIVE_PROFILES = {'native_rover': 'ardurover',
                         **{key: value[1] for key, value in ROVER_WORKFLOWS.items()}}
ROVER_PROFILES = set(ROVER_NATIVE_PROFILES.values())
BASELINE_NATIVE = set('''observation_load_status observation_load_attitude mission_native_cancel
mission_native_reset status_native_observations mission_native_transfer copter_native_operations
copter_sitl_workflow parameters_sitl concurrent_bulk concurrent_mission_default concurrent_bridge_pause
concurrent_mission_read fault_helm_pause fault_moosdb_restart fault_simulator_pause
fault_simulator_restart fault_command_timeout fault_bridge_pause
external_disarm_sitl auto_restart_recovery_sitl observer_extensions_native mavlink_files_native_copter'''.split()) | set(ROVER_NATIVE_PROFILES)
MANUAL_CONTRACT = {'native.distance_flight': ('distance_flight', 'distance'),
    'native.protocol.wind_readback_loss': ('protocol_faults', 'wind_readback_loss'),
    'native.protocol.mission_ack_loss': ('protocol_faults', 'mission_ack_loss'),
    'native.copter.route_hold_return': ('copter_scenarios', 'route_hold_return'),
    'native.copter.wind_repeated_cycle': ('copter_scenarios', 'wind_repeated_cycle'),
    'native.copter.auto_complete_handback': ('mission_execution', 'auto_complete_handback'),
    'native.copter.auto_pause_abort': ('mission_execution', 'auto_pause_abort'),
    'native.imu.rx_only': ('imu_observations', 'rx_only')}
MISSION_EXECUTION_SCENARIOS = {'auto_complete_handback', 'auto_pause_abort'}
MISSION_EXECUTION_MODULES = {'mavmoos_telemetry.so', 'mavmoos_parameters.so',
                             'mavmoos_missions.so', 'mavmoos_arducopter_control.so'}
MISSION_EXECUTION_CONTRACT = 'mission_execution/v1'
IMU_CONTRACT = 'imu_observations/v1'
IMU_MODULES = {'mavmoos_telemetry.so', 'mavmoos_imu.so'}
STRICT_NATIVE_CONTRACTS = {
    MISSION_EXECUTION_CONTRACT: (MISSION_EXECUTION_SCENARIOS, MISSION_EXECUTION_MODULES, 'firmware_inputs', 'path'),
    IMU_CONTRACT: ({'rx_only'}, IMU_MODULES, 'copied_firmware', 'file')}
RUNNER_CONTRACTS = {'mission_execution': MISSION_EXECUTION_CONTRACT, 'imu_observations': IMU_CONTRACT}
FIRMWARE_IDENTITY_FIELDS = ('schema', 'profile_id', 'manifest_sha256', 'attestation_sha256',
    'source_revision', 'native_path', 'native_sha256', 'platform', 'frame', 'fixture',
    'build_provenance', 'qualification')


class Failure(ValueError):
    pass


class CIInterrupted(BaseException):
    """Cancellation must not be consumed as an EINTR/OSError by subprocess."""
    def __init__(self, number, scope='CI'):
        self.number = number
        super().__init__(scope + ' interrupted by signal ' + str(number))


_owner_spec = importlib.util.spec_from_file_location('_ci_owned_processes', ROOT / 'tests/owned_processes.py')
_owner_module = importlib.util.module_from_spec(_owner_spec)
_owner_spec.loader.exec_module(_owner_module)


class ProcessOwner:
    """Retain observed descendant identities across separate groups/reparenting.

    Uses the repository's ps snapshot helper. PID/argv/group checks are local
    best-effort identity checks, not atomic kernel handles or a code sandbox.
    """
    def __init__(self, process):
        self.process, self.records = process, {}
        self.inventory_errors, self.direct_actions = [], []
        self.cleanup_errors = []
        self.snapshot_calls, self.observer_ps_omitted = 0, set()
        self.cleanup_proven = False

    @staticmethod
    def matches(old, current):
        return current and all(old[key] == current[key] for key in ('pid', 'group', 'command'))

    def read(self, phase, pids=None, timeout=3):
        if pids is not None and not pids:
            return {}
        self.snapshot_calls += 1
        try:
            return _owner_module.snapshot(pids=pids, timeout=timeout)
        except Exception as error:
            self.inventory_errors.append({'phase': phase, 'type': type(error).__name__, 'error': str(error)})
            raise

    @staticmethod
    def observer_ps(row):
        """Only the exact read-only inventory commands; unknown '(ps)' stays."""
        try:
            tokens = shlex.split(row['command'])
        except ValueError:
            return False
        if not tokens or Path(tokens[0]).name != 'ps':
            return False
        if tokens[1:] == ['-axww', '-o', 'pid=,ppid=,pgid=,command=']:
            return True
        return (len(tokens) == 6 and tokens[1:3] == ['-ww', '-p'] and
                re.fullmatch(r'[1-9][0-9]*(?:,[1-9][0-9]*)*', tokens[3]) is not None and
                tokens[4:] == ['-o', 'pid=,ppid=,pgid=,command='])

    def capture(self, timeout=3, phase='capture'):
        live = self.process.poll() is None
        rows = self.read(phase, timeout=timeout)
        owned = {self.process.pid} if live else set()
        owned.update(pid for pid, old in self.records.items() if self.matches(old, rows.get(pid)))
        for _ in range(64):
            added = {pid for pid, row in rows.items() if row['parent'] in owned}
            if added <= owned:
                break
            owned |= added
        require(len(owned) <= 8192, 'owned process-tree bound exceeded')
        for pid in owned:
            if pid in rows:
                row = rows[pid]
                if self.observer_ps(row):
                    self.observer_ps_omitted.add((pid, row['group'], row['command']))
                    self.records.pop(pid, None)
                else:
                    self.records[pid] = row
        return rows

    def living(self, timeout=3):
        rows = self.capture(timeout=timeout)
        return [row for pid, row in self.records.items() if self.matches(row, rows.get(pid))]

    def stop(self):
        """Stop scheduling even if inventory fails; never turn unknown into []."""
        self.cleanup_proven = False
        deadline = time.monotonic() + 8
        cohort = set(self.records)

        def direct(sig):
            # Popen's unreaped direct child cannot have its PID reused. Group
            # signalling additionally requires that live child's own group ID.
            if self.process.poll() is not None:
                return
            try:
                group = os.getpgid(self.process.pid)
                if group == self.process.pid:
                    os.killpg(group, sig)
                    scope = 'owned_new_session_group'
                else:
                    self.process.send_signal(sig)
                    scope = 'unreaped_direct_child'
                self.direct_actions.append({'signal': int(sig), 'pid': self.process.pid, 'scope': scope})
            except ProcessLookupError:
                pass

        def attempt_direct(sig):
            try:
                direct(sig)
            except OSError as error:
                # Keep permission/signal failures explicit, while still trying
                # TERM/CONT/KILL and preserving the caller's original failure.
                self.cleanup_errors.append({'phase': 'direct_signal_' + str(int(sig)),
                    'type': type(error).__name__, 'error': str(error)})

        def query(phase, global_table=False):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError('owned cleanup time budget exhausted')
            timeout = min(1 if global_table else .5, remaining)
            if global_table:
                return self.capture(timeout=timeout, phase=phase)
            rows = self.read(phase, pids=list(cohort), timeout=timeout)
            cohort.intersection_update(pid for pid in cohort
                                       if self.matches(self.records[pid], rows.get(pid)))
            return rows

        def matching(rows):
            return [row for pid, row in self.records.items() if self.matches(row, rows.get(pid))]

        def signal_rows(rows, sig):
            # One fresh batched inventory, then only matching recorded
            # descendants. Historical exited IDs cause no per-record ps calls.
            for row in matching(rows):
                if row['pid'] == self.process.pid:
                    continue
                try:
                    os.kill(row['pid'], sig)
                except ProcessLookupError:
                    pass
                except OSError as error:
                    self.cleanup_errors.append({'phase': 'descendant_signal_' + str(int(sig)),
                        'pid': row['pid'], 'type': type(error).__name__, 'error': str(error)})

        attempt_direct(signal.SIGSTOP)  # Freeze the scheduler before ending its case.
        inventory_complete, rows = False, {}
        try:
            rows = query('cleanup.initial', global_table=True)
            inventory_complete = True
            cohort = {row['pid'] for row in matching(rows)}
        except Exception:
            try:
                rows = query('cleanup.targeted_fallback')
            except Exception:
                rows = {}
        finally:
            # Never depend on a global process-table read to stop our scheduler.
            attempt_direct(signal.SIGTERM)
            attempt_direct(signal.SIGCONT)

        signal_rows(rows, signal.SIGTERM)
        try:
            rows = query('cleanup.before_continue')
        except Exception:
            rows = {}
        signal_rows(rows, signal.SIGCONT)
        term_deadline = min(deadline, time.monotonic() + 3)
        while matching(rows) and time.monotonic() < term_deadline:
            self.process.poll()
            time.sleep(.1)
            try:
                rows = query('cleanup.term_check')
            except Exception:
                rows = {}
                break
        try:
            rows = query('cleanup.before_kill')
            signal_rows(rows, signal.SIGKILL)
        except Exception:
            pass
        attempt_direct(signal.SIGKILL)
        if self.process.poll() is None:
            try:
                self.process.wait(timeout=max(.001, min(1, deadline - time.monotonic())))
            except subprocess.TimeoutExpired:
                pass
        final_deadline = min(deadline, time.monotonic() + 1)
        while True:
            try:
                rows = query('cleanup.final', global_table=True)
            except Exception:
                return None
            remaining = matching(rows)
            if not remaining or time.monotonic() >= final_deadline:
                break
            time.sleep(.1)
        if not inventory_complete or self.process.poll() is None or self.cleanup_errors:
            return None
        self.cleanup_proven = not remaining
        return remaining


def require(value, why):
    if not value:
        raise Failure(why)


def parse(data):
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, 'duplicate JSON key: ' + key)
            result[key] = value
        return result
    def bad(value):
        raise Failure('nonfinite JSON number: ' + value)
    def real(value):
        result = float(value)
        require(math.isfinite(result), 'nonfinite JSON numeric value')
        return result
    return json.loads(data, object_pairs_hook=pairs, parse_constant=bad, parse_float=real)


def sha(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def bounded(argv, cwd, output, timeout, env=None, *, inventory_timeout=3):
    """Supervise the recorded owned tree, including separately grouped children."""
    started = time.monotonic()
    with Path(output).open('wb') as log:
        process = subprocess.Popen(argv, cwd=cwd, env=env, stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        owner = ProcessOwner(process)
        timed_out, unexpected, remaining = False, [], None
        cleanup_proven, original_failure, cleanup_failure = False, None, None
        try:
            deadline = time.monotonic() + timeout
            while process.poll() is None:
                if time.monotonic() >= deadline:
                    timed_out = True
                    remaining = owner.stop()
                    cleanup_proven = owner.cleanup_proven
                    break
                try:
                    owner.capture(timeout=min(inventory_timeout, max(.001, deadline - time.monotonic())))
                except subprocess.TimeoutExpired:
                    # Inventory shares the original command deadline. Reaching
                    # that deadline is an ordinary failed timeout, while an
                    # earlier inventory failure remains the original exception.
                    if time.monotonic() < deadline:
                        raise
                    timed_out = True
                    remaining = owner.stop()
                    cleanup_proven = owner.cleanup_proven
                    break
                try:
                    process.wait(timeout=min(.2, max(.001, deadline - time.monotonic())))
                except subprocess.TimeoutExpired:
                    pass
            if not timed_out:
                unexpected = owner.living(timeout=inventory_timeout)
                if unexpected:
                    remaining = owner.stop()
                    cleanup_proven = owner.cleanup_proven
                else:
                    remaining, cleanup_proven = [], True
            code = process.wait(timeout=3)
        except BaseException as error:
            original_failure = {'type': type(error).__name__, 'error': str(error)}
            try:
                remaining = owner.stop()
                cleanup_proven = owner.cleanup_proven
            except BaseException as cleanup_error:
                remaining, cleanup_proven = None, False
                cleanup_failure = {'type': type(cleanup_error).__name__, 'error': str(cleanup_error)}
            raise
        finally:
            if remaining is None and not cleanup_proven and cleanup_failure is None:
                cleanup_failure = {'type': 'CleanupUnproven',
                    'error': 'complete fresh ownership/absence could not be established'}
            process_report = {'schema': 1, 'observed': list(owner.records.values()),
                              'unexpected_live_descendants': unexpected, 'remaining': remaining,
                              'cleanup_proven': cleanup_proven,
                              'original_failure': original_failure, 'cleanup_failure': cleanup_failure,
                              'inventory_errors': owner.inventory_errors,
                              'cleanup_errors': owner.cleanup_errors,
                              'snapshot_calls': owner.snapshot_calls,
                              'observer_ps_omitted': len(owner.observer_ps_omitted),
                              'direct_actions': owner.direct_actions,
                              'scope': 'recorded descendant PID/argv/group identities; no global cleanup'}
            dump(str(output) + '.processes.json', process_report)
    return {'argv': argv, 'cwd': str(cwd), 'exit_code': code,
            'timeout': timed_out, 'duration_s': time.monotonic() - started,
            'log': str(output), 'log_sha256': sha(output),
            'process_evidence': str(output) + '.processes.json',
            'process_evidence_sha256': sha(str(output) + '.processes.json'),
            'cleanup_proven': cleanup_proven, 'unexpected_live_descendants': len(unexpected)}


def accepted(record):
    return record['exit_code'] == 0 and not record['timeout'] and record['cleanup_proven'] and \
        record['unexpected_live_descendants'] == 0


def read_registry(path):
    registry = parse(Path(path).read_text())
    require(registry.get('schema') == 1, 'unsupported capability/test registry schema')
    tests = registry.get('tests')
    require(isinstance(tests, list) and 0 < len(tests) <= 512, 'empty/invalid test registry')
    names, ctests = set(), set()
    for test in tests:
        require(isinstance(test, dict), 'test entry is not an object')
        name = test.get('id')
        require(isinstance(name, str) and ID.fullmatch(name) and name not in names,
                'invalid/duplicate test ID')
        names.add(name)
        require(test.get('kind') in {'ctest', 'native'}, name + ': invalid kind')
        require(test.get('vehicle_profile') in (ROVER_PROFILES | {None, 'arducopter'}),
                name + ': unsupported vehicle profile')
        require(type(test.get('required')) is bool, name + ': required must be boolean')
        tiers = test.get('tiers')
        require(isinstance(tiers, list) and tiers and len(tiers) == len(set(tiers)) and
                set(tiers) <= TIERS, name + ': invalid tiers')
        scope = test.get('scope')
        require(scope in ('software', 'optional_software_extra', 'native_sitl'), name + ': invalid scope')
        if scope == 'native_sitl':
            require(test['required'] and 'full' in tiers and 'software' not in tiers and
                    test.get('vehicle_profile') is not None,
                    name + ': native scope must be profile-bound, required in full, and excluded from software')
        else:
            require(test['kind'] == 'ctest' and set(tiers) == TIERS and
                    test['required'] == (scope == 'software') and test.get('vehicle_profile') is None,
                    name + ': invalid software scope or tier membership')
        timeout = test.get('timeout_s')
        require(type(timeout) in (int, float) and math.isfinite(timeout) and
                0 < timeout <= 3600, name + ': invalid timeout')
        if test['kind'] == 'ctest':
            value = test.get('ctest_name')
            require(isinstance(value, str) and value and value not in ctests,
                    name + ': invalid/duplicate CTest mapping')
            require(value not in BASELINE_NATIVE or scope == 'native_sitl',
                    name + ': baseline native CTest requires native scope')
            if value in ROVER_NATIVE_PROFILES:
                require(test['vehicle_profile'] == ROVER_NATIVE_PROFILES[value],
                        name + ': native Rover profile differs from the fixture')
            ctests.add(value)
        else:
            require(test.get('runner') in MANUAL, name + ': unknown native runner')
    return registry


def snapshot_source(root, extra=()):
    files = [root / 'CMakeLists.txt']
    for name in ('src', 'tests', 'scripts', 'config', 'missions', 'packaging', '.github', 'examples'):
        parent = root / name
        if parent.is_dir():
            files.extend(p for p in parent.rglob('*') if p.is_file() and
                         '__pycache__' not in p.parts and p.suffix != '.pyc')
    files.extend(Path(p) for p in extra)
    return {str(p.resolve()): sha(p) for p in sorted(set(files)) if p.is_file()}


def copy_inventory(build, destination):
    """Copy only declarative CTest files; inventory writes solely below destination."""
    destination.mkdir()
    require((build / 'CTestTestfile.cmake').is_file(), 'configured CTest graph is absent')
    pending, seen = [Path('.')], set()
    while pending:
        relative = pending.pop()
        if relative in seen:
            continue
        seen.add(relative)
        require(len(seen) <= 512, 'CTest metadata graph exceeds512 directories')
        path = build / relative / 'CTestTestfile.cmake'
        if not path.exists():
            continue  # CMake lists some source-only subdirectories without tests.
        require(not path.is_symlink(), 'symlinked CTest metadata is unsupported')
        text = path.read_text()
        require(not re.search(r'^\s*include\s*\(', text, re.M),
                'external/dynamic CTest includes need an explicit inventory adapter')
        target = destination / relative / path.name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, target)
        for subdir in re.findall(r'^\s*subdirs\(\s*"([^"\n]+)"\s*\)', text, re.M):
            child = (build / relative / subdir).resolve()
            require(child == build or build in child.parents, 'CTest subdirectory escapes build root')
            pending.append(child.relative_to(build))


def inventory(build, output, ctest):
    directory = output / 'inventory'
    copy_inventory(build, directory)
    record = bounded([ctest, '--test-dir', str(directory), '--show-only=json-v1'],
                     output, output / 'inventory.log', 30)
    require(accepted(record), 'CTest inventory failed or leaked owned descendants')
    graph = parse((output / 'inventory.log').read_text())
    tests = graph.get('tests')
    require(isinstance(tests, list) and tests, 'configured CTest graph contains zero tests')
    names = [test.get('name') for test in tests]
    require(all(isinstance(n, str) and n for n in names) and len(names) == len(set(names)),
            'duplicate/invalid configured CTest names')
    for test in tests:
        command = test.get('command')
        require(isinstance(command, list) and command and
                all(isinstance(arg, str) and '\0' not in arg for arg in command),
                test['name'] + ': missing/invalid configured command')
        for prop in test.get('properties', []):
            if prop['name'] == 'WORKING_DIRECTORY':
                path = Path(prop['value']).resolve()
                if path == directory or directory in path.parents:
                    prop['value'] = str(build / path.relative_to(directory))
    dump(output / 'configured-tests.json', graph)
    return {test['name']: test for test in tests}, record


def properties(test):
    result = {}
    for item in test.get('properties', []):
        require(item['name'] not in result, 'duplicate CTest property')
        result[item['name']] = item['value']
    return result


def require_manual_binding(test):
    name = test['id']
    require(name in MANUAL_CONTRACT, name + ': unknown native qualification case')
    runner, scenario = MANUAL_CONTRACT[name]
    require(test['kind'] == 'native' and test['required'] and test['runner'] == runner and
            test.get('scenario') == scenario, name + ': wrong required manual qualifier mapping')
    if runner in RUNNER_CONTRACTS:
        limit = 360 if runner == 'mission_execution' else 120
        require(test.get('vehicle_profile') == 'arducopter' and test['timeout_s'] == limit,
                name + ': strict qualifier requires its Copter profile and original supervisor')


def select_tests(registry, graph, tier, case_ids=()):
    known = {test['ctest_name']: test for test in registry['tests'] if test['kind'] == 'ctest'}
    unknown = set(graph) - known.keys()
    require(not unknown, 'unregistered configured CTests: ' + ', '.join(sorted(unknown)))
    for name in graph:
        test = known[name]
        require(native(test, graph) == (test['scope'] == 'native_sitl'),
                test['id'] + ': registry scope disagrees with CTest sitl label')
    if tier in FOCUSED_TIERS:
        require(case_ids and len(case_ids) == len(set(case_ids)), tier + ' tier requires unique --case IDs')
        by_id = {test['id']: test for test in registry['tests']}
        selected = []
        for case_id in case_ids:
            test = by_id.get(case_id)
            require(test is not None, 'unknown case ID: ' + case_id)
            if tier == 'focused':
                require('software' in test['tiers'], 'focused tier accepts only software cases: ' + case_id)
            else:
                require(test['scope'] == 'native_sitl' and test['required'] and 'full' in test['tiers'],
                        'focused-native tier accepts only required native cases: ' + case_id)
                if test['kind'] == 'native' or case_id in MANUAL_CONTRACT:
                    require_manual_binding(test)
                else:
                    require(case_id == 'ctest.' + test['ctest_name'],
                            'focused-native requires the exact CTest ID: ' + case_id)
            if test['kind'] == 'ctest':
                require(test['ctest_name'] in graph, 'requested CTest is not configured: ' + case_id)
            selected.append(test)
        return selected, []
    selected, absent_optional = [], []
    for test in registry['tests']:
        if tier not in test['tiers']:
            continue
        if test['kind'] == 'ctest' and test['ctest_name'] not in graph:
            require(not test['required'], 'required CTest is not configured: ' + test['id'])
            absent_optional.append(test['id'])
            continue
        selected.append(test)
    require(selected and any(test['required'] for test in selected), 'selected tier has zero required tests')
    if tier == 'smoke':
        smoke = [test for test in selected if test['scope'] == 'native_sitl']
        require(len(smoke) == 1 and smoke[0]['id'] == 'ctest.' + SMOKE_WORKFLOW and
                smoke[0].get('ctest_name') == SMOKE_WORKFLOW and smoke[0]['vehicle_profile'] == 'arducopter',
                'smoke requires exactly the native Copter lifecycle case')
    if tier == 'full':
        by_id = {test['id']: test for test in selected}
        require(NATIVE_REQUIRED <= set(by_id),
                'full registry omits required explicit native qualifiers')
        for name in MANUAL_CONTRACT:
            require_manual_binding(by_id[name])
        for name in BASELINE_NATIVE:
            test = by_id.get('ctest.' + name)
            require(test and test['kind'] == 'ctest' and test['required'] and test['ctest_name'] == name and
                    'sitl' in properties(graph[name]).get('LABELS', []),
                    'full tier omits exact baseline native case: ' + name)
            if name in ROVER_NATIVE_PROFILES:
                require(test['vehicle_profile'] == ROVER_NATIVE_PROFILES[name],
                        name + ': native Rover profile differs from the fixture')
    return selected, absent_optional


def native(test, graph):
    return test['kind'] == 'native' or 'sitl' in properties(graph[test['ctest_name']]).get('LABELS', [])


def verify_firmware(args, output, label='firmware-verification'):
    for key in ('firmware_manifest', 'firmware_attestation', 'ardupilot_root',
                'moos_root', 'behavior_dir', 'sitl_python'):
        require(getattr(args, key) is not None, 'native tier requires --' + key.replace('_', '-'))
    for key in ('firmware_manifest', 'firmware_attestation', 'sitl_python'):
        require(Path(getattr(args, key)).is_file(), 'native input missing: ' + key)
    helper = args.source_root / 'scripts/firmware/manifest.py'
    require(helper.is_file(), 'firmware verification helper is absent')
    command = [sys.executable, '-B', str(helper), 'verify', '--manifest', str(args.firmware_manifest),
               '--attestation', str(args.firmware_attestation), '--ardupilot-root', str(args.ardupilot_root),
               '--expected-profile', 'arducopter']
    record = bounded(command, args.source_root, output / (label + '.log'), 60)
    require(accepted(record), 'firmware attestation verification failed')
    identity = parse((output / (label + '.log')).read_text())
    require(isinstance(identity, dict) and identity.get('native_sha256'), 'firmware verifier returned no identity')
    return identity, record


def bracket(value):
    value = str(value)
    marks = '='
    while ']' + marks + ']' in value:
        marks += '='
    return '[' + marks + '[' + value + ']' + marks + ']'


def rebase_outputs(command, build, case):
    result = list(command)
    original = (build / 'evidence').resolve()
    outputs = []
    for index, value in enumerate(result):
        if index and result[index - 1] == '--output':
            result[index] = str(case / 'evidence')
            outputs.append(result[index])
            continue
        path = Path(value)
        if path.is_absolute():
            path = path.resolve()
        if path.is_absolute() and path != original and original in path.parents:
            target = case / 'artifacts' / path.relative_to(original)
            target.parent.mkdir(parents=True, exist_ok=True)
            result[index] = str(target)
    return result, outputs


def native_profile(test):
    return test.get('vehicle_profile', ROVER_NATIVE_PROFILES.get(test.get('ctest_name'), 'arducopter'))


def selected_native_command(test, command, ardupilot_root, firmware=None):
    """Select Copter source without relabelling the separately configured Rover."""
    result = list(command)
    indexes = [index for index, value in enumerate(result) if value == '--ardupilot-root']
    require(len(indexes) == 1 and indexes[0] + 1 < len(result),
            test['id'] + ': native runner must declare exactly one ArduPilot root')
    profile = native_profile(test)
    require(profile in (ROVER_PROFILES | {'arducopter'}), 'unknown selected native profile')
    rover = ROVER_WORKFLOWS.get(test.get('ctest_name'))
    if rover:
        indexes_profile = [index for index, value in enumerate(result) if value == '--profile']
        require(len(indexes_profile) == 1 and indexes_profile[0] + 1 < len(result) and
                result[indexes_profile[0] + 1] == rover[0] and profile == rover[1],
                test['id'] + ': native Rover runner and selected profile disagree')
    if profile == 'arducopter':
        result[indexes[0] + 1] = str(ardupilot_root)
    if test.get('ctest_name') == 'copter_native_operations':
        require('--parity' not in result, 'Copter parity must be selected by the verified firmware')
        if firmware and firmware.get('profile_id') == COPTER_PARITY_PROFILE:
            require(firmware.get('source_revision') == COPTER_PARITY_REVISION,
                    'Copter parity requires the exact 4.6.3 source')
            result.append('--parity')
    return result, profile


def manual_command(test, args, case):
    source, build = args.source_root, args.build
    app = build / 'bin/iMAVLink'
    require(app.is_file(), 'built iMAVLink is absent')
    command = [str(args.sitl_python), '-B']
    common = ['--source-root', str(source), '--ardupilot-root', str(args.ardupilot_root),
              '--build', str(build), '--output', str(case / 'evidence'),
              '--firmware-manifest', str(args.firmware_manifest),
              '--firmware-attestation', str(args.firmware_attestation)]
    if test['runner'] == 'distance_flight':
        command += [str(source / 'tests/distance_native/run_distance.py')] + common
        command += ['--moos-root', str(args.moos_root), '--behavior-dir', str(args.behavior_dir),
                    '--probe', str(build / 'bin/distance_moos_probe'), '--expected-app-sha256', sha(app),
                    '--expected-distance-sha256', sha(build / 'plugins/mavmoos_distance_sensor.so')]
    elif test['runner'] == 'protocol_faults':
        require(test.get('scenario') in {'wind_readback_loss', 'mission_ack_loss'}, 'invalid protocol scenario')
        command += [str(source / 'tests/protocol_faults/run_native.py')] + common
        command += ['--scenario', test['scenario'],
                    '--expected-driver-sha256', sha(build / 'bin/protocol_native_driver'),
                    '--expected-parameters-sha256', sha(build / 'plugins/mavmoos_parameters.so'),
                    '--expected-missions-sha256', sha(build / 'plugins/mavmoos_missions.so')]
    elif test['runner'] == 'copter_scenarios':
        require(test.get('scenario') in {'route_hold_return', 'wind_repeated_cycle'}, 'invalid broader Copter scenario')
        command += [str(source / 'tests/copter_scenarios/run_scenarios.py')] + common
        command += ['--scenario', test['scenario'], '--moos-root', str(args.moos_root),
                    '--behavior-dir', str(args.behavior_dir), '--expected-app-sha256', sha(app),
                    '--expected-probe-sha256', sha(build / 'bin/copter_scenario_probe')]
    elif test['runner'] == 'mission_execution':
        require(test.get('scenario') in MISSION_EXECUTION_SCENARIOS, 'invalid mission execution scenario')
        command += [str(source / 'tests/mission_execution/run_native.py')] + common
        command += ['--scenario', test['scenario'], '--moos-root', str(args.moos_root),
                    '--behavior-dir', str(args.behavior_dir), '--probe', str(build / 'bin/mission_execution_probe'),
                    '--expected-app-sha256', sha(app),
                    '--expected-probe-sha256', sha(build / 'bin/mission_execution_probe')]
    elif test['runner'] == 'imu_observations':
        require(test.get('scenario') == 'rx_only', 'invalid IMU observation scenario')
        command += [str(source / 'tests/imu/run_native.py')] + common
        command += ['--scenario', 'rx_only', '--moos-root', str(args.moos_root),
                    '--probe', str(build / 'bin/imu_moos_probe'), '--expected-app-sha256', sha(app),
                    '--expected-probe-sha256', sha(build / 'bin/imu_moos_probe')]
    else:
        require(False, 'unknown native runner')
    require(Path(command[2]).is_file(), 'native runner is absent: ' + command[2])
    return command, [str(case / 'evidence')]


def plan(selected, graph, args, output, firmware=None, revision=None):
    directory = output / 'plan'
    directory.mkdir()
    lines, planned = [], []
    for test in selected:
        case = output / 'cases' / test['id']
        case.mkdir(parents=True)
        is_native = native(test, graph)
        props = properties(graph[test['ctest_name']]) if test['kind'] == 'ctest' else {}
        if test['kind'] == 'ctest':
            command, outputs = rebase_outputs(graph[test['ctest_name']]['command'], args.build, case)
            if is_native:
                command, _ = selected_native_command(test, command, args.ardupilot_root, firmware)
        else:
            command, outputs = manual_command(test, args, case)
        expected = None
        if is_native:
            require(len(outputs) == 1, test['id'] + ': native case must have exactly one fresh --output')
            profile = native_profile(test)
            if profile == 'arducopter':
                require(firmware is not None, 'no verified selected Copter identity')
                expected = dict(firmware)
            else:
                root = Path(command[command.index('--ardupilot-root') + 1])
                expected = {'native_sha256': sha(root / 'build/sitl/bin/ardurover'),
                    'source_revision': source_revision(root), 'profile_id': 'separate_legacy_rover'}
            ctest_name = test.get('ctest_name', '')
            if ctest_name == 'copter_native_operations' and '--parity' in command:
                expected['copter_operations_parity'] = True
            if ctest_name.startswith('mission_native_'):
                single_module = 'mavmoos_missions.so'
            elif ctest_name == 'mavlink_files_native_copter':
                single_module = 'mavmoos_mavlink_files.so'
            elif ctest_name == 'flight_logs_native_copter':
                single_module = 'mavmoos_flight_logs.so'
            elif ctest_name in ROVER_NATIVE_PROFILES:
                single_module = 'mavmoos_ardurover_control.so'
            else:
                single_module = 'mavmoos_parameters.so'
            expected.update({'vehicle_profile': profile, 'mavmoos_revision': revision,
                'app_sha256': sha(args.build / 'bin/iMAVLink'),
                'driver_sha256': sha(args.build / 'bin/protocol_native_driver')
                    if test.get('runner') == 'protocol_faults' else None,
                'module_sha256': {path.name: sha(path) for path in sorted((args.build / 'plugins').glob('mavmoos_*.so'))},
                'single_module': single_module,
                'probe_sha256': sha(command[command.index('--probe') + 1]) if '--probe' in command else None})
            if ctest_name == 'mavlink_files_native_copter':
                expected.update({'scenario': 'mavlink_files',
                    'moos_revision': source_revision(args.moos_root),
                    'helm_sha256': sha(args.moos_root / 'bin/pHelmIvP'),
                    'tap_sha256': sha(command[command.index('--tap') + 1])})
            if ctest_name in ROVER_WORKFLOWS:
                _, _, mav_type, model = ROVER_WORKFLOWS[ctest_name]
                expected.update({'profile_id': 'separate_rover', 'mav_type': mav_type, 'model': model,
                    'moos_revision': source_revision(args.moos_root),
                    'helm_sha256': sha(args.moos_root / 'bin/pHelmIvP'),
                    'probe_sha256': sha(args.build / 'bin/rover_expanded_probe')})
            if test.get('runner') == 'copter_scenarios':
                expected['probe_sha256'] = sha(args.build / 'bin/copter_scenario_probe')
            if test.get('runner') == 'mission_execution':
                expected.update({'probe_sha256': sha(args.build / 'bin/mission_execution_probe'),
                                 'raw_contract': MISSION_EXECUTION_CONTRACT,
                                 'scenario': test['scenario']})
            if test.get('runner') == 'imu_observations':
                expected.update({'probe_sha256': sha(args.build / 'bin/imu_moos_probe'),
                                 'raw_contract': IMU_CONTRACT, 'scenario': test['scenario']})
            if test.get('runner') in RUNNER_CONTRACTS:
                expected['moos_revision'] = source_revision(args.moos_root)
        env = props.get('ENVIRONMENT', [])
        if isinstance(env, str):
            env = [env]
        require(isinstance(env, list), test['id'] + ': invalid environment metadata')
        props['ENVIRONMENT'] = list(env)
        if is_native:
            props['ENVIRONMENT'] += ['MAVMOOS_FIRMWARE_MANIFEST=' + str(args.firmware_manifest),
                                     'MAVMOOS_FIRMWARE_ATTESTATION=' + str(args.firmware_attestation)]
            config = {'schema': 1, 'id': test['id'], 'command': command,
                      'working_directory': props.get('WORKING_DIRECTORY', str(args.build)),
                      'source_root': str(args.source_root), 'manifest': str(args.firmware_manifest),
                      'attestation': str(args.firmware_attestation), 'ardupilot_root': str(args.ardupilot_root),
                      'vehicle_profile': native_profile(test),
                      'output': str(case), 'timeout_s': test['timeout_s']}
            dump(case / 'launch.json', config)
            command = [sys.executable, '-B', str(args.source_root / 'scripts/ci/native_case.py'),
                       '--config', str(case / 'launch.json')]
        # Native wrappers reserve a bounded manifest check plus scoped teardown
        # outside the documented child fixture's own lifecycle ceiling.
        props['TIMEOUT'] = test['timeout_s'] + (75 if is_native else 0)
        if is_native:
            props['RUN_SERIAL'] = True
        props.setdefault('WORKING_DIRECTORY', str(args.build))
        # Dependency names belong to original CTests and cannot silently point
        # outside this explicit plan. Current project has no such dependencies.
        require(not any(key in props for key in ('DEPENDS', 'DEPENDENCIES', 'FIXTURES_REQUIRED',
                    'FIXTURES_SETUP', 'FIXTURES_CLEANUP')), test['id'] + ': unsupported dependency/fixture graph')
        lines.append('add_test(' + ' '.join(bracket(v) for v in [test['id'], *command]) + ')')
        pairs = []
        for key, value in sorted(props.items()):
            if key.startswith('_'):
                continue
            if isinstance(value, list):
                value = ';'.join(str(v) for v in value)
            elif type(value) is bool:
                value = 'TRUE' if value else 'FALSE'
            pairs.extend((bracket(key), bracket(value)))
        lines.append('set_tests_properties(' + bracket(test['id']) + ' PROPERTIES ' + ' '.join(pairs) + ')')
        planned.append({'id': test['id'], 'kind': test['kind'], 'native': is_native,
                        'required': test['required'], 'command': command, 'outputs': outputs,
                        'case_directory': str(case), 'timeout_s': test['timeout_s'],
                        'expected_identity': expected,
                        'vehicle_profile': native_profile(test) if is_native else None})
    (directory / 'CTestTestfile.cmake').write_text('\n'.join(lines) + '\n')
    dump(output / 'selected-tests.json', planned)
    return directory, planned


def audit_junit(path, selected):
    require(Path(path).is_file(), 'CTest did not produce JUnit evidence')
    suite = ET.parse(path).getroot()
    rows = list(suite.iter('testcase'))
    expected = {test['id'] for test in selected}
    names = [row.get('name') for row in rows]
    require(len(names) == len(set(names)), 'duplicate JUnit case')
    require(set(names) == expected and names, 'missing/extra/zero JUnit cases')
    report = []
    for row in rows:
        elapsed = float(row.get('time', 'nan'))
        require(math.isfinite(elapsed) and elapsed >= 0, 'invalid JUnit duration')
        bad = [element.tag for element in row if element.tag in {'failure', 'error', 'skipped'}]
        require(not bad and row.get('status', 'run') not in {'notrun', 'disabled'},
                row.get('name', '?') + ': failed/skipped/disabled JUnit case')
        report.append({'id': row.get('name'), 'result': 'PASS', 'duration_s': elapsed})
    return report


def source_revision(root):
    result = subprocess.run(['git', '-C', str(root), 'rev-parse', 'HEAD'], capture_output=True, text=True)
    value = result.stdout.strip()
    return value if result.returncode == 0 and re.fullmatch(r'[0-9a-f]{40}', value) else None


def moos_dependency(args, cache):
    """Bind native CI to the selected stock source and observed runtime bytes."""
    require(args.firmware_manifest is not None,
            'native CI requires --firmware-manifest before MOOS dependency verification')
    require(args.moos_root is not None and args.behavior_dir is not None,
            'native CI requires --moos-root and --behavior-dir')
    manifest = parse(args.firmware_manifest.read_text())
    pin = manifest.get('dependencies', {}).get('moos_revision')
    require(isinstance(pin, str) and re.fullmatch(r'[0-9a-f]{40}', pin),
            'selected manifest has no exact MOOS dependency revision')
    revision = source_revision(args.moos_root)
    require(revision == pin, 'MOOS source revision differs from selected manifest dependency: ' +
            str(revision) + ' != ' + pin)
    for key, path in (('MOOS_IVP_ROOT', args.moos_root),
                      ('MAVMOOS_BEHAVIOR_DIR', args.behavior_dir)):
        rows = re.findall(r'^' + key + r':(?:PATH|STRING)=(.*)$', cache.decode(), re.M)
        require(len(rows) == 1 and rows[0] and Path(rows[0]).resolve() == path.resolve(),
                'configured ' + key + ' differs from selected native CI path')
    def record(path):
        require(path.is_file(), 'required MOOS/behavior runtime file is absent: ' + str(path))
        return {'resolved_path': str(path.resolve()), 'sha256': sha(path)}
    executables = {str(args.moos_root / 'bin' / name): record(args.moos_root / 'bin' / name)
                   for name in ('pAntler', 'pHelmIvP', 'MOOSDB', 'nsplug')}
    require(args.behavior_dir.is_dir(), 'selected behavior directory is absent')
    files = sorted(path for path in args.behavior_dir.rglob('*') if path.is_file())
    require(0 < len(files) <= 4096, 'selected behavior directory is empty or exceeds file bound')
    return {'source_root': str(args.moos_root), 'source_revision': revision,
            'expected_revision': pin, 'behavior_dir': str(args.behavior_dir),
            'executables': executables,
            'behavior_files': {str(path): record(path) for path in files},
            'scope': 'Git HEAD pin and observed file bytes; binary build provenance not inferred'}


def verify_selected_moos_paths(selected, graph, args):
    expected = {'--moos-root': args.moos_root, '--moos-bin': args.moos_root / 'bin',
                '--behavior-dir': args.behavior_dir}
    for test in selected:
        if test['kind'] != 'ctest':
            continue
        command = graph[test['ctest_name']]['command']
        for option, path in expected.items():
            indexes = [index for index, value in enumerate(command) if value == option]
            if indexes:
                require(len(indexes) == 1 and indexes[0] + 1 < len(command) and
                        Path(command[indexes[0] + 1]).resolve() == path.resolve(),
                        test['id'] + ': configured ' + option + ' differs from selected native CI path')


def audit_raw_identity(report, expected):
    profile = expected['vehicle_profile']
    for key in ('vehicle_profile', 'control_profile'):
        if key in report:
            require(report[key] == profile, 'raw native vehicle profile mismatch')
    if report.get('profile') in (ROVER_PROFILES | {'arducopter'}):
        require(report['profile'] == profile, 'raw native vehicle profile mismatch')
    pins = [report[key] for key in ('native_sha256', 'copter_sha256') if key in report]
    require(pins and all(value == expected['native_sha256'] for value in pins), 'raw native executable pin mismatch')
    if expected['driver_sha256']:
        require(report.get('driver_sha256') == expected['driver_sha256'], 'raw protocol driver pin mismatch')
    else:
        require(report.get('app_sha256') == expected['app_sha256'], 'raw native app pin mismatch')
    if 'app_sha256' in report:
        require(report['app_sha256'] == expected['app_sha256'], 'raw native app pin mismatch')
    if 'driver_sha256' in report:
        require(report['driver_sha256'] == expected['driver_sha256'], 'raw protocol driver pin mismatch')
    for key in ('ardupilot_ref', 'ardupilot_revision'):
        if key in report:
            require(report[key] == expected['source_revision'], 'raw ArduPilot source pin mismatch')
    for key in ('source_ref', 'revision', 'mavmoos_revision', 'mavmoos_ref'):
        if key in report:
            require(expected['mavmoos_revision'] and report[key] == expected['mavmoos_revision'],
                    'raw MAVMOOS source pin mismatch')
    modules = expected['module_sha256']
    witnessed = False
    for key in ('loaded_modules_sha256', 'modules_sha256'):
        if key in report:
            values = report[key]
            require(isinstance(values, dict) and values, 'raw native module map absent/empty')
            require(all(name in modules and modules[name] == value for name, value in values.items()),
                    'raw native module pin mismatch')
            witnessed = True
    for key, module in (('module_sha256', expected['single_module']),
                        ('status_module_sha256', 'mavmoos_system_status.so')):
        if key in report:
            require(report[key] == modules[module], 'raw native single module pin mismatch')
            witnessed = True
    require(witnessed, 'raw native module identity missing')
    if expected['probe_sha256'] and 'probe_sha256' in report:
        require(report['probe_sha256'] == expected['probe_sha256'], 'raw native probe pin mismatch')
    if expected.get('profile_id') == 'separate_rover':
        require(all(report.get(key) == expected[key] for key in
            ('vehicle_profile', 'mav_type', 'model', 'mavmoos_revision', 'moos_revision', 'helm_sha256', 'probe_sha256')) and
            report.get('profile') == profile and report.get('ardupilot_revision') == expected['source_revision'],
            'expanded Rover identity missing or differs from the selected fixture')
        require(report.get('mavmoos_cmake_source', {}).get('tracked_changes') == [],
                'expanded Rover fixture was built from a dirty source')
        require(set(report.get('loaded_modules_sha256', {})) ==
                {'mavmoos_telemetry.so', 'mavmoos_parameters.so', 'mavmoos_ardurover_control.so'},
                'expanded Rover exact loaded composition missing')
    for key in ('firmware_identity', 'firmware_identity_after'):
        if key in report:
            value = report[key]
            require(profile == 'arducopter' and isinstance(value, dict), 'raw firmware identity wrong scope')
            for field in ('native_sha256', 'source_revision', 'profile_id'):
                require(value.get(field) == expected[field], 'raw firmware identity mismatch: ' + field)
            if 'manifest_sha256' in value:
                require(value['manifest_sha256'] == expected['manifest_sha256'], 'raw firmware manifest differs')
    if expected.get('scenario') == 'mavlink_files':
        require(all(report.get(key) == expected[key] for key in
                    ('scenario','mavmoos_revision','moos_revision','helm_sha256','tap_sha256','probe_sha256')),
                'file bench exact evidence identity differs')
        require(set(report.get('loaded_modules_sha256',{})) ==
                {'mavmoos_mavlink_files.so','mavmoos_telemetry.so'} and
                report.get('cleanup_proven') is True and report.get('hardware') is False and
                report.get('arming_requested') is False and report.get('observer_writes_to_fc') == 0 and
                report.get('artifacts_before') == report.get('artifacts_after') and
                report.get('mavmoos_revision_after') == expected['mavmoos_revision'] and
                report.get('moos_revision_after') == expected['moos_revision'] and
                set(report.get('native_ftp_operations',{})) ==
                {'list1','get1','list2','getempty','list3','getstop','list4','get2'},
                'file bench native operations, effects or teardown are unproven')
    contract = expected.get('raw_contract')
    if contract in STRICT_NATIVE_CONTRACTS:
        scenarios, required_modules, copy_field, path_field = STRICT_NATIVE_CONTRACTS[contract]
        require(isinstance(expected.get('mavmoos_revision'), str) and
                re.fullmatch(r'[0-9a-f]{40}', expected['mavmoos_revision']) and
                isinstance(expected.get('probe_sha256'), str) and
                re.fullmatch(r'[0-9a-f]{64}', expected['probe_sha256']) and
                isinstance(expected.get('platform'), dict) and expected['platform'],
                'strict native planned source/probe/platform identity missing')
        require(isinstance(expected.get('moos_revision'), str) and
                re.fullmatch(r'[0-9a-f]{40}', expected['moos_revision']) and
                report.get('moos_ref') == expected['moos_revision'], 'strict native MOOS source identity differs')
        require(profile == 'arducopter' and report.get('profile') == profile and
                report.get('scenario') == expected['scenario'] and
                expected['scenario'] in scenarios, 'strict native raw scenario/profile mismatch')
        require(report.get('hardware') is False and type(report.get('probe_exit')) is int and
                report['probe_exit'] == 0, 'strict native raw probe/hardware scope differs')
        require(report.get('source_ref') == expected['mavmoos_revision'] and
                report.get('ardupilot_ref') == expected['source_revision'] and
                report.get('probe_sha256') == expected['probe_sha256'] and expected['probe_sha256'],
                'strict native raw source/probe identity is missing or mismatched')
        loaded = report.get('loaded_modules_sha256')
        require(isinstance(loaded, dict) and set(loaded) == required_modules and
                all(name in modules and loaded[name] == modules[name] for name in loaded),
                'strict native exact loaded module identity differs')
        for key in ('firmware_identity', 'firmware_identity_after'):
            value = report.get(key)
            require(isinstance(value, dict) and all(value.get(field) == expected[field]
                    for field in FIRMWARE_IDENTITY_FIELDS),
                    'strict native mandatory firmware identity differs: ' + key)
        copies, files = report.get(copy_field), report.get('files')
        require(isinstance(copies, dict) and set(copies) == {'manifest', 'attestation'} and
                isinstance(files, dict) and files, 'strict native copied firmware/file evidence missing')
        for role in ('manifest', 'attestation'):
            value = copies[role]
            require(isinstance(value, dict) and isinstance(value.get(path_field), str) and
                    value.get('sha256') == expected[role + '_sha256'] and
                    files.get(value[path_field]) == value['sha256'], 'strict native copied firmware input differs: ' + role)
        require(report.get('artifact_drift') == [] and report.get('owned_processes_remaining') == [],
                'strict native explicit preservation/process absence proof missing')
        if contract == IMU_CONTRACT:
            provider, capture = report.get('provider_evidence'), report.get('capture')
            require(isinstance(provider, dict) and provider.get('zero_tx_counters_observed') is True and
                    provider.get('authority_granted') is False and
                    isinstance(provider.get('loaded_modules'), list) and
                    len(provider['loaded_modules']) == 2 and set(provider['loaded_modules']) == {'imu', 'telemetry'} and
                    all(type(provider.get(key)) is int and provider[key] > 0
                        for key in ('io_status_records', 'runtime_status_records')),
                    'IMU observed receive-only counters/authority/catalog proof missing')
            require(type(report.get('observer_writes')) is int and report['observer_writes'] == 0 and
                    all(report.get(key) is False for key in ('arming_requested', 'rc_input_sent', 'relay_used')) and
                    report.get('cleanup_proven') is True and isinstance(capture, dict) and
                    capture.get('thread_retired') is True and capture.get('socket_closed') is True and
                    'error' in capture and capture['error'] is None,
                    'IMU fixture effect scope/capture cleanup unproven')
            require(report.get('imu_module_sha256') == modules['mavmoos_imu.so'] and
                    report.get('source_ref_after') == expected['mavmoos_revision'] and
                    report.get('moos_ref_after') == expected['moos_revision'], 'IMU postflight source/module identity differs')


def audit_native(item):
    folder = Path(item['outputs'][0])
    reports = sorted(folder.rglob('result.json'))
    require(reports, item['id'] + ': missing native result')
    expected = item['expected_identity']
    expanded_rover = item['id'] in {'ctest.' + name for name in ROVER_WORKFLOWS}
    copter_parity = expected.get('copter_operations_parity', False)
    strict_case = item['id'] in {name for name, (runner, _) in MANUAL_CONTRACT.items()
                                if runner in RUNNER_CONTRACTS}
    if strict_case:
        runner, scenario = MANUAL_CONTRACT[item['id']]
        require(item['vehicle_profile'] == expected['vehicle_profile'] == 'arducopter' and
                expected.get('raw_contract') == RUNNER_CONTRACTS[runner] and expected.get('scenario') == scenario,
                'strict native planned identity contract missing or mismatched')
    if strict_case or expanded_rover or copter_parity:
        require(reports == [folder / 'result.json'] and not reports[0].is_symlink(),
                'strict native case requires its one actual top-level result/cleanup report')
    launch = parse((Path(item['case_directory']) / 'native-launch-result.json').read_text())
    require(launch.get('result') == 'PASS' and launch['id'] == item['id'], 'per-case native launch verification failed')
    if item['vehicle_profile'] == 'arducopter':
        verified = launch['firmware']
        require(all(verified.get(key) == expected[key] for key in
            ('source_revision', 'native_sha256', 'profile_id', 'manifest_sha256')),
            'per-case verified firmware differs from planned identity')
    cleanup = []
    for path in reports:
        report = parse(path.read_text())
        require(report.get('result') == 'PASS', item['id'] + ': raw native report failed')
        if strict_case or expanded_rover:
            require('ports_clear' in report, 'strict native raw endpoint cleanup proof missing')
            require(isinstance(report.get('files'), dict) and report['files'],
                    'strict native raw file inventory missing or empty')
        if expanded_rover:
            _, profile, mav_type, model = ROVER_WORKFLOWS[item['id'].removeprefix('ctest.')]
            require(item['vehicle_profile'] == expected['vehicle_profile'] == profile and
                    expected.get('mav_type') == mav_type and expected.get('model') == model,
                    'expanded Rover planned fixture identity missing or mismatched')
        for relative, expected_hash in report.get('files', {}).items():
            name = Path(relative)
            require(not name.is_absolute() and '..' not in name.parts and
                    str(name) == relative and relative != 'result.json', 'unsafe raw evidence member')
            member = path.parent / name
            require(member.is_file() and not member.is_symlink() and
                    sha(member) == expected_hash, 'raw native evidence member hash mismatch')
        if 'ports_clear' not in report:
            continue
        audit_raw_identity(report, expected)
        if copter_parity:
            commands, helm = report.get('parity_evidence'), report.get('parity_helm')
            require(report.get('parity') is True and isinstance(commands, dict) and commands.get('commands') and
                    isinstance(helm, dict) and helm, 'selected stable Copter parity evidence is missing')
        require(report.get('hardware', False) is False, item['id'] + ': wrong hardware scope')
        ports = report['ports_clear']
        require(isinstance(ports, dict) and ports and all(value is True for value in ports.values()),
                item['id'] + ': native endpoint cleanup not proven')
        require('owned_processes_remaining' in report or 'mission_processes_remaining' in report,
                item['id'] + ': no scoped process cleanup evidence')
        for key in ('owned_processes_remaining', 'mission_processes_remaining', 'artifact_drift'):
            require(not report.get(key), item['id'] + ': native process/artifact cleanup failed')
        cleanup.append({'path': str(path), 'sha256': sha(path)})
    require(cleanup, item['id'] + ': missing raw cleanup report')
    return {'id': item['id'], 'result': 'PASS', 'cleanup_reports': cleanup,
            'firmware_binding': expected['profile_id'] if item['vehicle_profile'] in ROVER_PROFILES
                                else 'selected_copter_manifest'}


def run(args):
    output = args.output
    output.mkdir(parents=True, exist_ok=False)
    report = {'schema': 1, 'result': 'FAIL', 'tier': args.tier, 'hosted': False,
              'scope': 'local selected registry tier; no physical vehicle or undeclared firmware qualification',
              'source_root': str(args.source_root), 'build': str(args.build),
              'registry': str(args.registry), 'registry_sha256': None,
              'commands': [], 'cases': [], 'native_evidence': []}
    if args.tier in FOCUSED_TIERS:
        report['scope'] = ('focused software feedback; no qualification tier completed' if args.tier == 'focused'
                           else 'exact selected owned native cases; no full-tier qualification')
        report['requested_ids'] = args.case
    inputs, artifacts = {}, {}
    try:
        require((args.build / 'CMakeCache.txt').is_file(), 'configured build directory is absent')
        cache = (args.build / 'CMakeCache.txt').read_bytes()
        (output / 'CMakeCache.txt').write_bytes(cache)
        source = re.search(r'^CMAKE_HOME_DIRECTORY:INTERNAL=(.*)$', cache.decode(), re.M)
        require(source and Path(source.group(1)).resolve() == args.source_root,
                'build source directory differs from --source-root')
        report['cache_sha256'] = hashlib.sha256(cache).hexdigest()
        registry = read_registry(args.registry)
        report['registry_sha256'] = sha(args.registry)
        extra = [args.registry] + [p for p in (args.firmware_manifest, args.firmware_attestation) if p]
        inputs = snapshot_source(args.source_root, extra)
        report['inputs_before'] = inputs
        report['source_revision'] = source_revision(args.source_root)
        if args.tier in ('smoke', 'full', 'focused-native'):
            report['moos_dependency_before'] = moos_dependency(args, cache)
            for values in ('executables', 'behavior_files'):
                for record in report['moos_dependency_before'][values].values():
                    artifacts[record['resolved_path']] = record['sha256']
        command = bounded([args.cmake, '--build', str(args.build), '--parallel', str(args.jobs)],
                          args.source_root, output / 'build.log', args.build_timeout)
        report['commands'].append(command)
        require(accepted(command), 'build failed/timed out or leaked owned descendants')
        # Ninja/Make may regenerate the configured graph during an incremental
        # build. Inventory the resulting graph, not a stale pre-build copy.
        graph, command = inventory(args.build, output, args.ctest)
        report['commands'].append(command)
        selected, missing_optional = select_tests(registry, graph, args.tier, getattr(args, 'case', ()))
        report['absent_optional'] = missing_optional
        if any(native(test, graph) for test in selected):
            require('moos_dependency_before' in report, 'native cases require a native CI tier')
            require(moos_dependency(args, (args.build / 'CMakeCache.txt').read_bytes()) ==
                    report['moos_dependency_before'], 'MOOS dependency changed during build')
            verify_selected_moos_paths(selected, graph, args)
            report['firmware'], command = verify_firmware(args, output)
            report['commands'].append(command)
            path = Path(report['firmware']['native_path'])
            artifacts[str(path.resolve())] = sha(path)
            for test in selected:
                if native(test, graph) and test['kind'] == 'ctest' and native_profile(test) in ROVER_PROFILES:
                    argv = graph[test['ctest_name']]['command']
                    command, _ = selected_native_command(test, argv, args.ardupilot_root)
                    root = Path(command[command.index('--ardupilot-root') + 1])
                    path = root / 'build/sitl/bin/ardurover'
                    require(path.is_file(), 'separately configured Rover executable is absent')
                    artifacts[str(path.resolve())] = sha(path)
        for parent in (args.build / 'bin', args.build / 'plugins'):
            if parent.is_dir():
                for path in parent.iterdir():
                    if path.is_file():
                        artifacts[str(path.resolve())] = sha(path)
        report['artifacts_before'] = artifacts
        directory, planned = plan(selected, graph, args, output, report.get('firmware'), report['source_revision'])
        report['selected_ids'] = [test['id'] for test in selected]
        report['scope_excluded_ids'] = [test['id'] for test in registry['tests']
                                       if test['id'] not in report['selected_ids']]
        # Long native plans share the host process table with active simulators
        # and nested owners. The inventory cap cannot extend the CTest deadline.
        inventory_timeout = NATIVE_INVENTORY_TIMEOUT if any(test['native'] for test in planned) else 3
        command = bounded([args.ctest, '--test-dir', str(directory), '--output-on-failure',
                           '--parallel', str(args.jobs), '--output-junit', str(output / 'junit.xml')],
                          args.source_root, output / 'ctest.log', args.max_time,
                          inventory_timeout=inventory_timeout)
        report['commands'].append(command)
        require(accepted(command), 'selected CTest plan failed/timed out or leaked owned descendants')
        report['cases'] = audit_junit(output / 'junit.xml', selected)
        report['native_evidence'] = [audit_native(item) for item in planned if item['native']]
        report['result'] = 'PASS'
    except CIInterrupted as error:
        report['interrupted'] = True
        report['error'] = str(error)
        raise
    except (OSError, ValueError, KeyError, TypeError, ET.ParseError, subprocess.SubprocessError) as error:
        report['error'] = str(error)
    finally:
        report['source_revision_after'] = source_revision(args.source_root)
        if report.get('source_revision') and report['source_revision_after'] != report['source_revision']:
            report['result'] = 'FAIL'
            report.setdefault('error', 'MAVMOOS revision changed during qualification')
        if 'moos_dependency_before' in report:
            try:
                report['moos_dependency_after'] = moos_dependency(args,
                    (args.build / 'CMakeCache.txt').read_bytes())
                report['moos_dependency_drift'] = report['moos_dependency_after'] != report['moos_dependency_before']
                require(not report['moos_dependency_drift'], 'MOOS/behavior dependency changed during CI')
            except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as error:
                report['result'] = 'FAIL'
                report['moos_dependency_drift'] = True
                report['moos_dependency_after_error'] = str(error)
                report.setdefault('error', str(error))
        if 'firmware' in report and not report.get('interrupted'):
            try:
                report['firmware_after'], command = verify_firmware(args, output, 'firmware-after')
                report['commands'].append(command)
                require(report['firmware_after'] == report['firmware'], 'selected firmware identity changed during CI')
            except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as error:
                report['result'] = 'FAIL'
                report['firmware_after_error'] = str(error)
                report.setdefault('error', 'firmware identity failed after CI: ' + str(error))
        try:
            report['inputs_after'] = snapshot_source(args.source_root,
                [args.registry] + [p for p in (args.firmware_manifest, args.firmware_attestation) if p])
        except OSError as error:
            report['inputs_after'] = {}
            report['result'] = 'FAIL'
            report['inputs_after_error'] = 'protected input readback failed: ' + str(error)
            report.setdefault('error', report['inputs_after_error'])
        report['input_drift'] = sorted(key for key in set(inputs) | set(report['inputs_after'])
            if inputs.get(key) != report['inputs_after'].get(key)) if inputs else []
        report['artifacts_after'] = {key: sha(key) if Path(key).is_file() else None for key in artifacts}
        report['artifact_drift'] = sorted(key for key in artifacts
            if artifacts[key] != report['artifacts_after'][key])
        if report['input_drift'] or report['artifact_drift']:
            report['result'] = 'FAIL'
            report['postflight_error'] = 'protected source/input/artifact changed during qualification'
            report.setdefault('error', report['postflight_error'])
        dump(output / 'result.json', report)
    print(json.dumps({'result': report['result'], 'tier': args.tier,
                      'executed_cases': len(report['cases']), 'error': report.get('error'),
                      'output': str(output)}, indent=2))
    return 0 if report['result'] == 'PASS' else 1


def main():
    def interrupted(number, _frame):
        raise CIInterrupted(number)
    signal.signal(signal.SIGTERM, interrupted)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path, default=ROOT)
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--tier', choices=sorted(TIERS | FOCUSED_TIERS), required=True)
    parser.add_argument('--case', action='append', default=[], help='exact registry ID (repeat in a focused tier)')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--registry', type=Path)
    for name in ('firmware-manifest', 'firmware-attestation', 'ardupilot-root',
                 'moos-root', 'behavior-dir', 'sitl-python'):
        parser.add_argument('--' + name, type=Path)
    parser.add_argument('--jobs', type=int, default=1)
    parser.add_argument('--max-time', type=int, default=7200)
    parser.add_argument('--build-timeout', type=int, default=1800)
    parser.add_argument('--ctest', default='ctest')
    parser.add_argument('--cmake', default='cmake')
    args = parser.parse_args()
    if bool(args.case) != (args.tier in FOCUSED_TIERS):
        parser.error('--case is required in focused tiers and allowed only in those tiers')
    require(1 <= args.jobs <= 32 and 1 <= args.max_time <= 10800 and
            1 <= args.build_timeout <= 7200, 'invalid jobs/time budget')
    for name in ('source_root', 'build', 'output', 'registry', 'firmware_manifest',
                 'firmware_attestation', 'ardupilot_root', 'moos_root', 'behavior_dir', 'sitl_python'):
        value = getattr(args, name)
        if value is not None:
            # A venv interpreter is commonly a symlink to the base executable;
            # resolving it would discard that venv's installed pymavlink.
            setattr(args, name, input_path(name, value))
    args.registry = args.registry or args.source_root / 'config/capabilities/copter.json'
    return run(args)


def input_path(name, value):
    return value.absolute() if name == 'sitl_python' else value.resolve()


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except CIInterrupted as error:
        print(json.dumps({'result': 'FAIL', 'error': str(error), 'interrupted': True}), file=sys.stderr)
        raise SystemExit(128 + error.number)
    except (OSError, ValueError) as error:
        print(json.dumps({'result': 'FAIL', 'error': str(error)}), file=sys.stderr)
        raise SystemExit(1)
