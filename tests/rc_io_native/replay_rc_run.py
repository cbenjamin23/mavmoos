#!/usr/bin/env python3
"""Replay a closed native RC evidence ZIP entirely offline; no extraction/I/O effects."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import zipfile
from RcNativeAudit import audit, corroborate, parse, summary


def canonical(value):return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def load_closed_archive(path, expected_sha):
    raw = Path(path).read_bytes();digest = hashlib.sha256(raw).hexdigest()
    if digest != expected_sha:raise ValueError('closed archive hash differs from expected identity')
    with zipfile.ZipFile(path) as source:
        members = source.infolist()
        if len(members) > 128 or sum(item.file_size for item in members) > 128 * 1024 * 1024:
            raise ValueError('native evidence archive exceeds bounds')
        names = [item.filename for item in members]
        if len(names) != len(set(names)) or any(PurePosixPath(name).is_absolute() or '..' in PurePosixPath(name).parts for name in names):
            raise ValueError('duplicate/unsafe archive member')
        result_names = [name for name in names if name.endswith('/result.json')]
        if len(result_names) != 1:raise ValueError('expected one native result record')
        result_name = result_names[0];prefix = result_name[:-len('result.json')]
        if any(not name.startswith(prefix) for name in names):raise ValueError('mixed evidence roots')
        result = parse(source.read(result_name).decode())
        files = result.get('files', {})
        if set(names) != {prefix + name for name in files} | {result_name}:raise ValueError('native file manifest/member mismatch')
        for name, expected in files.items():
            if hashlib.sha256(source.read(prefix + name)).hexdigest() != expected:raise ValueError('native member hash mismatch: ' + name)
        return result, {name[len(prefix):]: source.read(name) for name in names}, digest


def replay(path, expected_sha):
        result, content, digest = load_closed_archive(path, expected_sha)
        if result.get('result') != 'PASS' or result.get('artifact_drift') != [] or result.get('owned_processes_remaining') != []:
            raise ValueError('native outcome/drift/cleanup not qualified')
        if not result.get('ports_clear') or any(value is not True for value in result['ports_clear'].values()):raise ValueError('native ports not cleared')
        rover = 'rc_provider_evidence' in result
        trace_name = 'rc-probe.jsonl' if rover else 'probe.jsonl'
        rows = [parse(line) for line in content[trace_name].decode().splitlines() if line.startswith('{')]
        proof = audit(rows, mode='SITL_CONTROL' if rover else 'RX_ONLY', duration=210 if rover else 36)
        stored = result['rc_provider_evidence' if rover else 'provider_evidence']
        if canonical(summary(proof)) != canonical(stored):raise ValueError('recomputed MOOS audit differs from retained report')
        packets = [parse(line) for line in content['fc-monitor.jsonl' if rover else 'observer.jsonl'].decode().splitlines() if line]
        records = parse(content['dataflash-observations.json'].decode())
        paired = corroborate(proof, packets, records)
        if canonical(paired) != canonical(result['independent_rc_values' if rover else 'independent_values']):
            raise ValueError('recomputed FC/DataFlash value comparison differs')
        if not rover:
            bridge = result['ports']['bridge']
            wire = [parse(line) for line in content['wire-datagrams.jsonl'].decode().splitlines() if line]
            if any(row.get('source') == ['127.0.0.1', bridge] for row in wire):raise ValueError('raw route evidence contains app-origin UDP')
            if result.get('app_origin_datagrams') != 0 or result.get('relay_forwards_to_native') != 0 or result.get('observer_writes') != 0:
                raise ValueError('RX_ONLY route effect assertion failed')
            if any(row.get('base_mode', 0) & 128 for row in packets if row.get('mavpackettype') == 'HEARTBEAT'):
                raise ValueError('Copter raw heartbeat shows arming')
            if any(row.get('ArmState') != 0 for row in records.get('arms', [])):raise ValueError('Copter DataFlash shows arming')
        return {'schema': 1, 'result': 'pass', 'archive_sha256': digest, 'members_verified': len(content),
                'mode': proof['mode'], 'late_windows': [row['phase'] for row in proof['windows']],
                'scope': 'Offline integrity, original metadata, advancing late source values and retained cleanup evidence; no new simulator or hardware observation'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, required=True);parser.add_argument('--expected-sha256', required=True)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    try:result = replay(args.archive, args.expected_sha256)
    except Exception as error:result = {'schema': 1, 'result': 'fail', 'reason': str(error)}
    text = json.dumps(result, indent=2) + '\n'
    if args.output:
        if args.output.exists():raise RuntimeError('refuse to replace replay output')
        args.output.write_text(text)
    print(text, end='');return int(result['result'] != 'pass')


if __name__ == '__main__':raise SystemExit(main())
