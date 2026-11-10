#!/usr/bin/env python3
"""Read back scheduler evidence, verifying exact sources, logs, traces and controls."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import zipfile

from verify_trace import negative_controls, parse, require, verify


def digest(data):
    return hashlib.sha256(data).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    parser.add_argument('--current-source-root', type=Path)
    args = parser.parse_args()
    root = args.archive
    manifest = json.loads((root / 'validation.json').read_text())
    require(manifest['result'] == 'pass' and manifest['failure'] is None, 'qualification did not pass')
    require(digest((root / 'tested-sources.zip').read_bytes()) == manifest['tested_sources_sha256'], 'source ZIP hash')
    with zipfile.ZipFile(root / 'tested-sources.zip') as source:
        require(set(source.namelist()) == set(manifest['inputs']), 'source ZIP file set')
        for name, expected in manifest['inputs'].items():
            require(digest(source.read(name)) == expected, f'snapshot hash: {name}')
            if args.current_source_root:
                require(digest((args.current_source_root / name).read_bytes()) == expected, f'current source hash: {name}')
    header_path = root / 'generated-headers-sha256.json'
    require(digest(header_path.read_bytes()) == manifest['generated_header_manifest_sha256'], 'generated header manifest hash')
    require(len(json.loads(header_path.read_text())) == manifest['generated_header_count'], 'generated header count')
    for mode in ['normal', 'sanitized']:
        compressed = (root / mode / 'trace.jsonl.gz').read_bytes()
        trace = gzip.decompress(compressed)
        require(digest(compressed) == manifest[mode]['compressed_trace_sha256'], f'{mode}: compressed trace hash')
        records = parse(trace.decode())
        result = verify(records)
        require(result['groups'] == manifest[mode]['groups'], f'{mode}: group count')
        require(negative_controls(records) == manifest[mode]['negative_controls_rejected'], f'{mode}: negative controls')
    for command in manifest['commands']:
        log = root / command['log']
        if command['log'].endswith('/trace.jsonl'):
            data = gzip.decompress(Path(str(log) + '.gz').read_bytes())
        else:
            data = log.read_bytes()
        require(command['returncode'] == 0, f'failed command: {command["argv"]}')
        require(digest(data) == command['log_sha256'], f'log hash: {log}')
    print(json.dumps({'result': 'pass', 'modes': ['normal', 'sanitized'], 'groups_each': manifest['normal']['groups'],
                      'negative_controls_each': len(manifest['normal']['negative_controls_rejected']),
                      'source_files': len(manifest['inputs']), 'current_sources_match': bool(args.current_source_root)}))


if __name__ == '__main__':
    main()
