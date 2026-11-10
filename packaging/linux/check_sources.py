#!/usr/bin/env python3
"""Prove the Linux build did not edit any original upstream source file."""
import hashlib
import argparse
import json
from pathlib import Path
import sys
import tarfile


parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--mavmoos',action='store_true',help='Also verify original MAVMOOS files after app tests')
args=parser.parse_args()
checks = []
inputs=[('/inputs/moos-ivp.tar', '/work/moos-ivp'),('/inputs/altitude.tar', '/work/altitude')]
if args.mavmoos:
    inputs.append(('/inputs/mavmoos.tar','/work/mavmoos'))
for archive_name, root_name in inputs:
    archive_path = Path(archive_name)
    changed = []
    count = 0
    with tarfile.open(archive_path) as archive:
        for member in archive:
            if not member.isfile():
                continue
            count += 1
            path = Path(root_name) / member.name
            if not path.is_file() or path.read_bytes() != archive.extractfile(member).read():
                changed.append(member.name)
    checks.append({'archive': archive_name,
                   'sha256': hashlib.sha256(archive_path.read_bytes()).hexdigest(),
                   'original_files': count, 'changed_files': changed})
passed = all(not check['changed_files'] for check in checks)
print(json.dumps({'result': 'PASS' if passed else 'FAIL', 'checks': checks}, indent=2))
sys.exit(0 if passed else 1)
