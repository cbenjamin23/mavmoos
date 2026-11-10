#!/usr/bin/env python3
"""Offline CI-controller component archive integrity; never start any process."""
import argparse
import copy
import hashlib
import io
import json
from pathlib import Path, PurePosixPath
import re
import zipfile


def require(value, why):
    if not value:
        raise ValueError(why)


def digest(value):
    return hashlib.sha256(value).hexdigest()


def verify(folder, value, source=None):
    require(value['schema'] == 1 and value['result'] == 'PASS', 'component qualification failed')
    require(value['native_or_docker_executed_by_stream'] is False and
            value['hosted_workflow_executed'] is False, 'component evidence scope changed')
    archive = value['tested_source']
    payload = (folder / archive['path']).read_bytes()
    require(digest(payload) == archive['sha256'], 'source ZIP hash differs')
    with zipfile.ZipFile(io.BytesIO(payload)) as bundle:
        names = bundle.namelist()
        require(len(names) == len(set(names)) and set(names) == set(archive['members']),
                'source member set missing/duplicate/extra')
        for name in names:
            path = PurePosixPath(name)
            require(not path.is_absolute() and '..' not in path.parts and str(path) == name,
                    'unsafe source member')
            data = bundle.read(name)
            require(digest(data) == archive['members'][name], 'source member hash differs')
            if source:
                require((source / name).read_bytes() == data, 'current component source differs: ' + name)
    log = (folder / value['test_log']['path']).read_bytes()
    require(digest(log) == value['test_log']['sha256'], 'test log hash differs')
    text = log.decode()
    require(re.search(r'^Ran ' + str(value['tests_run']) + r' tests in [\d.]+s$', text, re.M),
            'actual unittest completion count missing')
    require(re.search(r'^OK$', text, re.M) and not re.search(r'^FAILED|^ERROR:|^FAIL:', text, re.M),
            'actual unittest execution failed')
    require(len(re.findall(r'^test_\w+ \(', text, re.M)) == value['tests_run'], 'test identity row count differs')
    for name, expected in value['failure_files'].items():
        path = PurePosixPath(name)
        require(not path.is_absolute() and '..' not in path.parts, 'unsafe failure member')
        require(digest((folder / name).read_bytes()) == expected, 'retained failure evidence changed')
    return {'result': 'PASS', 'source_members': len(names), 'controller_tests': value['tests_run'],
            'retained_failure_files': len(value['failure_files']),
            'scope': 'offline component source/log integrity; no native, container or hosted outcome inferred'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    parser.add_argument('--current-source-root', type=Path)
    parser.add_argument('--negative-controls', action='store_true')
    args = parser.parse_args()
    value = json.loads((args.archive / 'qualification.json').read_text())
    result = verify(args.archive, value, args.current_source_root)
    if args.negative_controls:
        rejected = []
        for name, change in [('false_result', lambda v: v.update(result='FAIL')),
                ('invented_native_scope', lambda v: v.update(native_or_docker_executed_by_stream=True)),
                ('fabricated_test_count', lambda v: v.update(tests_run=v['tests_run'] + 1)),
                ('wrong_source_zip', lambda v: v['tested_source'].update(sha256='0' * 64)),
                ('missing_source_member', lambda v: v['tested_source']['members'].pop(next(iter(v['tested_source']['members'])))),
                ('wrong_source_member', lambda v: v['tested_source']['members'].update({next(iter(v['tested_source']['members'])): '0' * 64})),
                ('wrong_test_log', lambda v: v['test_log'].update(sha256='0' * 64))]:
            mutated = copy.deepcopy(value)
            change(mutated)
            try:
                verify(args.archive, mutated)
            except (ValueError, KeyError):
                rejected.append(name)
            else:
                raise ValueError('malformed component metadata accepted: ' + name)
        result['negative_controls_rejected'] = rejected
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
