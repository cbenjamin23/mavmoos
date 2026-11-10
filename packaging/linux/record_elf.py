#!/usr/bin/env python3
"""Verify Linux artifacts directly without a host or external file utility."""
import hashlib
import json
from pathlib import Path
import sys

records = []
for argument in sys.argv[1:]:
    path = Path(argument)
    data = path.read_bytes()
    if data[:4] != b'\x7fELF' or data[4] != 2:
        raise SystemExit('expected native ELF64 artifact: ' + str(path))
    order = 'little' if data[5] == 1 else 'big'
    records.append({'path': str(path), 'format': 'ELF64',
                    'machine': int.from_bytes(data[18:20], order),
                    'sha256': hashlib.sha256(data).hexdigest()})
print(json.dumps(records, indent=2))
