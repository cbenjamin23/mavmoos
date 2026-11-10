#!/usr/bin/env python3
"""Qualify the real metadata module against private fixture files and wire reports."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import zlib


def crc(data):
    return zlib.crc32(data, 0xffffffff) ^ 0xffffffff


def main(driver, module):
    assert crc(b'123456789') == 0x2dfd2d88
    a = {'version': 1, 'metadataTypes': [], 'vendorName': 'MOOS "Lab"',
         'modelName': 'Ocean Bench', 'firmwareVersion': '1.2.3'}
    b = {**a, 'firmwareVersion': '1.2.4'}
    documents = {
        'a': json.dumps(a).encode(),
        'b': json.dumps(b).encode(),
        'version': json.dumps({**b, 'version': 2}).encode(),
        'duplicate': b'{"version":1,"version":1,"metadataTypes":[]}',
        'large': b' ' * 4097,
    }
    with tempfile.TemporaryDirectory(prefix='mavmoos-meta.') as private:
        root = Path(private).resolve()
        for name, data in documents.items():
            path = root / name
            path.write_bytes(data)
            path.chmod(0o600)
        trace = subprocess.check_output([driver, module, root, *[
            str(crc(documents[name])) for name in ('a', 'b', 'version', 'duplicate')]],
            text=True, timeout=8)
    rows = {row['phase']: row['posts']['MAVMOOS_COMPONENT_METADATA']
            for row in map(json.loads, trace.splitlines())}
    expected = {
        'prebind': 'waiting_declaration',
        'wrong_component': 'waiting_declaration',
        'verified': 'verified',
        'repeat_stale': 'stale_declaration',
        'checksum': 'checksum_mismatch',
        'updated': 'verified',
        'conflict': 'declaration_conflict_or_boot_regression',
        'regressed': 'declaration_conflict_or_boot_regression',
        'version': 'unsupported_document_schema',
        'size': 'document_size_invalid',
        'symlink': 'mirror_file_unavailable',
        'duplicate_key': 'unsupported_document_schema',
        'unsupported_uri': 'unsupported_uri',
        'cleared': 'waiting_declaration',
        'rebound': 'verified',
    }
    assert {key: row['verification'] for key, row in rows.items()} == expected
    assert rows['verified']['document'] == a and rows['updated']['document'] == b
    assert rows['rebound']['document'] == a
    assert rows['repeat_stale']['metadata']['effective_at'] == rows['verified']['metadata']['effective_at']
    for phase in ('conflict', 'regressed'):
        assert rows[phase]['metadata']['effective_at'] == rows['updated']['metadata']['effective_at']
        assert rows[phase]['declaration'] == rows['updated']['declaration']
    assert rows['rebound']['metadata']['session_epoch'] > rows['verified']['metadata']['session_epoch']
    for name, status in expected.items():
        assert (rows[name]['document'] is not None) == (status == 'verified')
        assert rows[name]['metadata']['authoritative_for_control'] is False
    print(trace, end='')


if __name__ == '__main__':
    main(*sys.argv[1:])
