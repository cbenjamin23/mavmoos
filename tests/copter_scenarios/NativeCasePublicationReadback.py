#!/usr/bin/env python3
"""Read actual archived metadata through the additive publication validator.

This command never creates qualification records, imports archived code, decodes
MAVLink or launches native processes. Its output is a component-check receipt.
"""
import argparse
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import zipfile


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def readback(source_root, full_directory, readback_name, targeted_directory):
    path = source_root / 'tests/copter_scenarios/NativeCasePublicationTests.py'
    spec = importlib.util.spec_from_file_location('_publication_controls', path)
    controls = importlib.util.module_from_spec(spec); spec.loader.exec_module(controls); controls.initialize(source_root)
    api = controls.API
    matrix_path, review_path = full_directory / 'matrix.json', full_directory / readback_name
    matrix, review = api.parse(matrix_path.read_bytes()), api.parse(review_path.read_bytes())
    matrix_ref = {'path': 'matrix.json', 'sha256': sha(matrix_path)}
    review_ref = {'path': readback_name, 'sha256': sha(review_path), 'reviewer_sha256': review['reviewer_sha256']}
    context = api.NativeCaseEvidence(full_directory, matrix_ref, review_ref)
    jobs = [job for job in matrix['jobs'] if job.get('tier') == 'full' and job.get('qualification') == 'current']
    api.require(len(jobs) == 1, 'component check expects one exact positive full job')
    job = jobs[0]
    archive_paths = {ref['file']: full_directory / ref['file'] for ref in matrix['archives']}
    before = {name: sha(path) for name, path in archive_paths.items()}
    accepted = []
    augmented = copy.deepcopy(controls.REGISTRY)
    for ident, supported in api.NATIVE_CASE_CAPS.items():
        member = job['prefix'] + '/cases/' + ident + '/evidence/result.json'
        with zipfile.ZipFile(archive_paths[job['archive']]) as zipped:
            raw_data = zipped.read(member)
        raw = api.parse(raw_data)
        copied = raw.get('copied_firmware') if ident == 'native.imu.rx_only' else raw['firmware_inputs']
        key = 'file' if ident == 'native.imu.rx_only' else 'path'
        manifest = context.json(job['archive'], str(Path(member).parent / copied['manifest'][key]))
        fw = raw['firmware_identity']
        cell = {'id': 'component-only.' + ident, 'level': 'native_sitl', 'test_id': ident,
                'capability_ids': sorted(supported), 'source_revision': matrix['source_revision'],
                'firmware_revision': raw['ardupilot_ref'], 'firmware_label': manifest['source']['release_label'],
                'vehicle_profile': 'arducopter', 'frame': 'quad',
                'platform': fw['platform']['system'].lower() + '-' + fw['platform']['machine'].lower(),
                'app_sha256': raw['app_sha256'], 'native_sha256': raw['native_sha256'],
                'scope': 'component-check only: exact named scenario and selected capability; no records published',
                'evidence': {'kind': 'native_case', 'matrix': matrix_ref, 'readback': review_ref, 'job': job['id'],
                             'case': ident, 'raw': {'archive': job['archive'], 'member': member,
                                                   'sha256': hashlib.sha256(raw_data).hexdigest()}}}
        api.native_case_shape(cell)
        context.verify(cell, controls.INDEX['capabilities'], controls.INDEX['tests'])
        augmented['qualifications'].append(cell)
        for cap in augmented['capabilities']:
            if cap['id'] in supported:
                cap['qualification_ids'].append(cell['id'])
        accepted.append({key: cell[key] for key in ('test_id', 'capability_ids', 'source_revision', 'firmware_revision',
                                                   'vehicle_profile', 'platform', 'app_sha256', 'native_sha256')})
    api.validate_registry(augmented)
    target_path = targeted_directory / 'campaign-result.json'
    targeted_data = target_path.read_bytes(); targeted = api.parse(targeted_data)
    api.require(targeted.get('result') == 'PASS' and len(targeted.get('cases', [])) == 2 and
                all(row.get('result') == 'PASS' for row in targeted['cases']), 'expected actual two-case target evidence absent')
    target_cases = []
    for row in targeted['cases']:
        raw_path = targeted_directory / 'cases' / row['id'] / 'evidence/result.json'
        raw = api.parse(raw_path.read_bytes())
        api.require(raw.get('result') == 'PASS' and raw.get('hardware') is False, 'targeted raw evidence not closed PASS')
        target_cases.append({'id': row['id'], 'sha256': sha(raw_path), 'source_revision': raw['source_ref']})
    # Counterfactual outer full labels are confined to an inert temporary graph.
    # The candidate full-job result is the ACTUAL unmodified targeted JSON bytes.
    with tempfile.TemporaryDirectory() as directory:
        fixture = controls.Fixture(directory)
        fixture.extra[fixture.prefix + '/result.json'] = targeted_data
        fixture.publish()
        try:
            fixture.check()
        except (ValueError, KeyError, TypeError) as error:
            rejection = str(error)
        else:
            raise ValueError('actual targeted campaign was accepted as a full qualification')
    after = {name: sha(path) for name, path in archive_paths.items()}
    api.require(before == after, 'immutable archive bytes changed during component readback')
    api.require(controls.REGISTRY['qualifications'] == augmented['qualifications'][:len(controls.REGISTRY['qualifications'])],
                'original qualification cells changed')
    api.require(len([cell for cell in controls.REGISTRY['qualifications'] if 'kind' not in cell['evidence']]) == 11,
                'expected historical eleven operation cells absent')
    return {'schema': 1, 'result': 'PASS', 'validator_sha256': sha(source_root / 'scripts/capabilities/registry.py'),
            'matrix_sha256': matrix_ref['sha256'], 'readback_sha256': review_ref['sha256'],
            'deep_reviewer_sha256': review_ref['reviewer_sha256'], 'archive_hashes_before': before,
            'archive_hashes_after': after, 'accepted_native_cases': accepted,
            'legacy_operation_cells_unchanged': 11, 'records_published': 0,
            'targeted_full_rejection': {'campaign_sha256': hashlib.sha256(targeted_data).hexdigest(),
                                        'actual_cases': target_cases, 'error': rejection,
                                        'counterfactual_full_labels': 'inert temporary metadata graph only'},
            'scope': 'stdlib publication metadata consistency against an existing deep-readback receipt; no semantic replay/native launch/new qualification'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path, required=True)
    parser.add_argument('--full-evidence', type=Path, required=True)
    parser.add_argument('--readback', default='readback-02.json')
    parser.add_argument('--targeted-campaign', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = readback(args.source_root.resolve(), args.full_evidence.resolve(), args.readback, args.targeted_campaign.resolve())
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2); stream.write('\n')
    print(json.dumps({'result': report['result'], 'native_cases': len(report['accepted_native_cases']),
                      'targeted_full_rejected': True, 'output': str(args.output)}, indent=2))


if __name__ == '__main__':
    main()
