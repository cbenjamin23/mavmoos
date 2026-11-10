#!/usr/bin/env python3
"""Create/read back additive capability cells from an archived scenario result.

Definitions are read-only. This checks identity and replayable evidence, not a
cryptographic or physical attestation. No socket, simulator or module is run.
"""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import re
import zipfile
from registry import parse, relative, require, validate_registry

REV = re.compile(r'[0-9a-f]{40}\Z')
SHA = re.compile(r'[0-9a-f]{64}\Z')
OPERATIONS = {
    'copter.guided': ('c{n}-guided', 'COMPLETED'),
    'copter.arm': ('c{n}-arm', 'COMPLETED'),
    'copter.disarm': ('c{n}-disarm', 'COMPLETED'),
    'copter.takeoff': ('c{n}-takeoff', 'COMPLETED'),
    'copter.land': ('c{n}-land', 'COMPLETED'),
    'copter.goto.helm': ('c{n}-route1', 'COMPLETED'),
    'copter.hold.helm': ('cycle{n}-hold-helm', 'CANCELLED'),
    'copter.hold.guided': ('cycle{n}-hold-guided', 'CANCELLED'),
    'copter.return.helm': ('c{n}-return', 'COMPLETED'),
    'copter.cancel': ('cycle{n}-cancel-helm', 'COMPLETED'),
}


def sha(data):
    return hashlib.sha256(data).hexdigest()


def checker(source_root):
    path = Path(source_root) / 'tests/copter_scenarios/scenario_evidence.py'
    spec = importlib.util.spec_from_file_location('_capability_scenario_checker', path)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module, path


def protected_source(source_root, result, relative_path):
    path = Path(source_root) / relative_path
    witnesses = [value for name, value in result['artifacts_before'].items()
                 if name.endswith('/' + relative_path)]
    require(witnesses == [sha(path.read_bytes())], 'helper differs from executed protected source: ' + relative_path)
    return path


def scenario_times(result, proof, read, source_root, api):
    """Replay the coordinator contract selected by the supplied source tree.

    ACK receipts have their own content/causality check. They are not extra
    flight phases, and cannot be ignored merely because a result says PASS.
    """
    times = {row['kind']: row['python_mono'] for row in result['events']}
    require(len(times) == len(result['events']), 'repeated independent phase marker')
    markers = set(api.markers(result['scenario']))
    helper_name = 'tests/copter_scenarios/hold_boundary.py'
    helper_path = Path(source_root) / helper_name
    if not helper_path.exists():
        require(set(times) == markers and 'hold_boundaries' not in result and not proof.get('boundary_tokens'),
                'legacy independent phase contract differs')
        return times
    path = protected_source(source_root, result, helper_name)
    protected_source(source_root, result, 'tests/observation_load/passive_receiver.py')
    spec = importlib.util.spec_from_file_location('_capability_hold_boundary', path)
    helper = importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)
    acknowledgements = {label + suffix + '.ack' for label in proof['windows']
                        for suffix in ('.started', '.finished')}
    require(set(times) == markers | acknowledgements, 'independent exact phase/ACK inventory differs')
    names = {'control/' + key for key in proof['boundary_tokens']}
    names |= {name + '.ack' for name in names}
    require(names <= set(result['files']), 'hold token/ACK files missing from protected raw inventory')
    boundaries = helper.audit_boundary_bytes(result['events'], proof, lambda key: read('control/' + str(relative(key))))
    require(boundaries == result['hold_boundaries'], 'recorded causal hold boundaries differ')
    capture = result.get('passive_receiver')
    require(result.get('cleanup_proven') is True and isinstance(capture, dict) and
            capture.get('read_only') is True and capture.get('thread_retired') is True and
            capture.get('socket_closed') is True and 'error' in capture and capture['error'] is None,
            'passive capture/cleanup unproven')
    require(all(type(capture.get(k)) is int for k in ('maximum_datagrams', 'datagrams', 'accepted_datagrams')) and
            capture['maximum_datagrams'] == 150000 and 0 < capture['accepted_datagrams'] <= capture['datagrams'] <= 150000,
            'original passive receive bound differs')
    first, last, finished = capture.get('first_read_at'), capture.get('last_read_at'), result['monitor_finished_python_mono']
    require(all(type(v) in (int, float) and math.isfinite(v) for v in (first, last, finished)) and
            0 <= first <= last <= finished, 'actual receive clock scope differs')
    peer = result['ports']['monitor_peer']
    require(type(peer) is int and 0 < peer < 65536 and capture.get('peer') == ['127.0.0.1', peer] and
            isinstance(result.get('native_observer_socket_inventory'), str) and result['native_observer_socket_inventory'],
            'owned passive endpoint witness absent/different')
    return {key: times[key] for key in markers}


def evidence_cells(registry, result, read, source_root):
    """Read/validate the exact recorded source/profile/scenario and raw members."""
    index = validate_registry(registry)
    require(result['result'] == 'PASS' and result['hardware'] is False and result['probe_exit'] == 0,
            'scenario did not pass in native simulation scope')
    require(result['profile'] == 'arducopter' and result['observer_writes'] == 0 and result['rc_input_sent'] is False and
            result['mission_execution_requested'] is False and result['parameter_write_requested'] is False,
            'scenario widened its effect scope')
    require(result['ports_clear'] and all(v is True for v in result['ports_clear'].values()) and
            result['owned_processes_remaining'] == [] and result['artifact_drift'] == [], 'scenario cleanup/artifact drift')
    require(REV.fullmatch(result['source_ref']) and REV.fullmatch(result['ardupilot_ref']) and
            SHA.fullmatch(result['app_sha256']) and SHA.fullmatch(result['native_sha256']), 'source/artifact identity missing')
    firmware = result['firmware_identity']
    require(firmware['source_revision'] == result['ardupilot_ref'] and firmware['native_sha256'] == result['native_sha256'] and
            firmware['frame'] == 'quad' and firmware['qualification'] == 'not_inferred' and
            result['firmware_identity_after'] == firmware, 'firmware identity/promotion mismatch')
    api, path = checker(source_root)
    witnesses = [value for name, value in result['artifacts_before'].items()
                 if name.endswith('/tests/copter_scenarios/scenario_evidence.py')]
    require(witnesses == [sha(path.read_bytes())], 'checker differs from executed protected source')
    require({'probe.jsonl', 'fc-observer.jsonl', 'dataflash-observations.json', *api.FIRMWARE_FILES.values()} <= set(result['files']), 'required raw scenario files missing')
    for name, expected in result['files'].items():
        relative(name);require(SHA.fullmatch(expected) and sha(read(name)) == expected, 'raw file hash differs: ' + name)
    events = [api.parse(line) for line in read('probe.jsonl').decode().splitlines() if line.startswith('{')]
    packets = [api.parse(line) for line in read('fc-observer.jsonl').decode().splitlines() if line]
    closed = api.parse(read('dataflash-observations.json'))
    require(set(result['firmware_inputs']) == set(api.FIRMWARE_FILES), 'copied firmware input inventory differs')
    for role, name in api.FIRMWARE_FILES.items():
        require(result['firmware_inputs'][role] == {'path': name, 'sha256': result['files'][name]}, 'copied firmware input/result binding differs')
    inputs = api.firmware_inputs(read(api.FIRMWARE_FILES['manifest']), read(api.FIRMWARE_FILES['attestation']), firmware)
    proof = api.audit_probe(events, result['scenario'], inputs)
    times = scenario_times(result, proof, read, source_root, api)
    fc = api.audit_fc(packets, times, result['monitor_finished_python_mono'], proof)
    df = api.audit_dataflash(closed['arms'], closed['modes'], closed['fixture_parameters'], result['scenario'])
    require(proof == result['probe_evidence'] and proof['firmware_discovery'] == result['firmware_discovery'] and
            fc == result['independent_fc'] and df == result['dataflash_evidence'],
            'recorded semantic report differs from raw replay')
    test_id = 'native.copter.' + result['scenario']
    require(test_id in index['tests'] and index['tests'][test_id]['runner'] == 'copter_scenarios', 'scenario/test registry binding missing')
    cells = []
    for capid, (pattern, terminal) in OPERATIONS.items():
        cap = index['capabilities'][capid]
        require(cap['implementation_status'] == 'implemented' and test_id in cap['test_ids'], 'capability/test mapping missing: ' + capid)
        observed = [pattern.format(n=n) for n in range(1, proof['cycles'] + 1) if pattern.format(n=n) in proof['operations']]
        if not observed:
            continue  # Auto-disarm is not proof that an explicit DISARM request ran.
        require(all(proof['operations'][op]['terminal'] == terminal for op in observed), 'operation outcome differs: ' + capid)
        cells.append({'capability_id': capid, 'level': 'native_sitl', 'test_id': test_id,
                      'operation_ids': observed, 'observed_terminal': terminal,
                      'original_expires': {op: proof['operations'][op]['expires'] for op in observed},
                      'scope': 'exact recorded operation(s) and fixture only; no stable-release range or field claim'})
    return cells, {'source_revision': result['source_ref'], 'firmware_revision': result['ardupilot_ref'],
                   'firmware_profile_id': firmware['profile_id'], 'platform': firmware['platform'],
                   'firmware_manifest_sha256': inputs['manifest_sha256'], 'firmware_attestation_sha256': inputs['attestation_sha256'],
                   'observed_version': inputs['expected_version'], 'early_version_observation': proof['firmware_discovery']['early_observation'],
                   'frame': firmware['frame'], 'app_sha256': result['app_sha256'], 'native_sha256': result['native_sha256'],
                   'scenario': result['scenario'], 'cycles': proof['cycles']}


def read_archive(path, prefix, result_bytes):
    prefix = str(relative(prefix)).rstrip('/') + '/'
    archive = zipfile.ZipFile(path)
    names = [entry.filename for entry in archive.infolist()]
    require(len(names) == len(set(names)), 'duplicate raw archive member')
    for info in archive.infolist():
        relative(info.filename.rstrip('/') if info.is_dir() else info.filename)
    require(archive.read(prefix + 'result.json') == result_bytes, 'archived whole result differs')
    return archive, lambda name: archive.read(prefix + str(relative(name)))


def verify_report(report_path, registry_path, source_root):
    source_root = Path(source_root).resolve()
    report = parse(Path(report_path).read_bytes());registry_bytes = Path(registry_path).read_bytes()
    require(report['schema'] == 1 and report['kind'] == 'mavmoos.capability_qualification' and report['result'] == 'PASS', 'qualification schema/result')
    require(sha(registry_bytes) == report['registry_sha256'], 'immutable registry bytes differ')
    result_ref, archive_ref = report['evidence']['result'], report['evidence']['raw_archive']
    result_bytes = (source_root / relative(result_ref['path'])).read_bytes()
    raw_path = source_root / relative(archive_ref['path'])
    require(sha(result_bytes) == result_ref['sha256'] and sha(raw_path.read_bytes()) == archive_ref['sha256'], 'result/raw archive identity differs')
    archive, read = read_archive(raw_path, archive_ref['prefix'], result_bytes)
    try:
        cells, identity = evidence_cells(parse(registry_bytes), parse(result_bytes), read, source_root)
    finally:
        archive.close()
    require(report['cells'] == cells and report['identity'] == identity, 'qualification cells contradict evidence')
    return {'result': 'PASS', 'cells': len(cells), 'identity': identity,
            'scope': 'offline source-bound semantic/file readback, no simulator; closed DataFlash JSON replay, not independent BIN extraction'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path, required=True)
    parser.add_argument('--registry', type=Path, required=True)
    parser.add_argument('--result', type=Path)
    parser.add_argument('--raw-root', type=Path)
    parser.add_argument('--raw-archive', type=Path)
    parser.add_argument('--archive-prefix')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--verify', type=Path)
    args = parser.parse_args();root = args.source_root.resolve()
    if args.verify:
        print(json.dumps(verify_report(args.verify, args.registry, root), indent=2));return
    require(all(v is not None for v in (args.result, args.raw_root, args.raw_archive, args.archive_prefix, args.output)), 'generation inputs missing')
    require(not args.output.exists(), 'refusing to replace an existing qualification output')
    result_path, raw_path = args.result.resolve(), args.raw_archive.resolve()
    require(result_path.is_relative_to(root) and raw_path.is_relative_to(root), 'durable evidence paths must be repository-relative')
    require(str(result_path.relative_to(root)).startswith('docs/') and str(raw_path.relative_to(root)).startswith('docs/'), 'qualification evidence must be archived under docs')
    registry_bytes = args.registry.read_bytes();result_bytes = result_path.read_bytes()
    cells, identity = evidence_cells(parse(registry_bytes), parse(result_bytes), lambda name: (args.raw_root / relative(name)).read_bytes(), root)
    archive, read = read_archive(raw_path, args.archive_prefix, result_bytes)
    try:
        archived_cells, archived_identity = evidence_cells(parse(registry_bytes), parse(result_bytes), read, root)
        require(cells == archived_cells and identity == archived_identity, 'raw directory/archive evidence differ')
    finally:
        archive.close()
    report = {'schema': 1, 'kind': 'mavmoos.capability_qualification', 'result': 'PASS', 'registry_sha256': sha(registry_bytes),
              'evidence': {'result': {'path': str(result_path.relative_to(root)), 'sha256': sha(result_bytes)},
                           'raw_archive': {'path': str(raw_path.relative_to(root)), 'sha256': sha(raw_path.read_bytes()), 'prefix': args.archive_prefix}},
              'identity': identity, 'cells': cells,
              'scope': 'additive exact-profile native evidence; frozen capability/test definitions unchanged'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as handle:
        handle.write(json.dumps(report, indent=2) + '\n')
    print(json.dumps(verify_report(args.output, args.registry, root), indent=2))


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, KeyError, TypeError, zipfile.BadZipFile) as error:
        print(json.dumps({'result': 'FAIL', 'error': str(error)}));raise SystemExit(1)
