#!/usr/bin/env python3
"""Offline evidence-validator regression tests; no simulator, sockets or MOOS."""
import copy
import json
import unittest

from mission_evidence import INSPECTION_CAPABILITIES, observed_catalog, validate_snapshots


def row(seq, x, y, z, frame):
    return {'seq': seq, 'command': 16, 'frame': frame, 'current': 0, 'autocontinue': 1,
            'params': [0, 0, 0, 0], 'x': x, 'y': y, 'z': z}


def evidence():
    events = []
    for operation, count in [('missions-initial', 0), ('missions-upload', 3), ('missions-download', 3),
                             ('missions-clear', 0), ('missions-empty', 0)]:
        items = [row(0, 423584560, -710875890, 10, 0), row(1, 423584660, -710875890, 5, 3),
                 row(2, 423584760, -710875790, 5, 3)] if count else []
        snapshot = {'schema': 1, 'id': operation, 'status': 'complete', 'count': count, 'items': items,
                    'verification': 'semantic_profile_match' if operation in ('missions-upload', 'missions-clear') else 'observation'}
        events.append({'type': 'snapshot', 'value': json.dumps(snapshot)})
    names = ['missions.download', 'missions.upload.simulator', 'missions.clear.simulator', 'missions.cancel', 'missions.progress']
    catalog = {'schema': 1, 'execution_mode': 'SITL_CONTROL', 'composition': {'plugin_abi': 9,
               'modules': [{'name': 'missions', 'control_provider': False, 'capabilities': [{'name': n} for n in names]}]}}
    events.append({'type': 'capabilities', 'value': json.dumps(catalog)})
    return events


def mutate(events, index, fn):
    value = json.loads(events[index]['value']); fn(value); events[index]['value'] = json.dumps(value)


class EvidenceTests(unittest.TestCase):
    def setUp(self):
        self.events = evidence()

    def test_complete_expected_observations_and_catalog(self):
        self.assertEqual([s['count'] for s in validate_snapshots(self.events)], [0, 3, 3, 0, 0])
        self.assertEqual(observed_catalog(self.events)['composition']['plugin_abi'], 9)

    def test_missing_and_duplicate_observations_reject(self):
        with self.assertRaises(RuntimeError): validate_snapshots(self.events[1:])
        self.events.append(self.events[0])
        with self.assertRaises(RuntimeError): validate_snapshots(self.events)

    def test_incomplete_or_wrong_count_reject(self):
        for key, value in [('status', 'incomplete'), ('schema', 2), ('count', 2)]:
            events = copy.deepcopy(self.events); mutate(events, 1, lambda s: s.update({key: value}))
            with self.subTest(key=key), self.assertRaises(RuntimeError): validate_snapshots(events)

    def test_items_must_be_distinct_and_in_sequence(self):
        for sequence in [0, 2]:
            events = copy.deepcopy(self.events); mutate(events, 1, lambda s: s['items'][1].update(seq=sequence))
            with self.assertRaises(RuntimeError): validate_snapshots(events)

    def test_mutations_require_explicit_semantic_verification(self):
        for index in (1, 3):
            events = copy.deepcopy(self.events); mutate(events, index, lambda s: s.update(verification='observation'))
            with self.assertRaises(RuntimeError): validate_snapshots(events)

    def test_waypoint_coordinates_frames_altitude_and_flags_reject(self):
        for key, value in [('x', 0), ('y', 0), ('z', 5.02), ('frame', 0), ('command', 21), ('autocontinue', 0)]:
            events = copy.deepcopy(self.events); mutate(events, 1, lambda s: s['items'][1].update({key: value}))
            with self.subTest(key=key), self.assertRaises(RuntimeError): validate_snapshots(events)

    def test_nonfinite_and_malformed_parameters_reject(self):
        for key, value in [('z', float('nan')), ('z', float('inf')), ('params', [0, 0, 0, float('nan')]), ('params', [0, 0, 0])]:
            events = copy.deepcopy(self.events); mutate(events, 1, lambda s: s['items'][1].update({key: value}))
            with self.subTest(key=key, value=value), self.assertRaises(RuntimeError): validate_snapshots(events)

    def test_home_convention_is_explicit_and_dynamic_current_accepted(self):
        mutate(self.events, 1, lambda s: s['items'][0].update(x=423584550, z=9.81, current=1))
        validate_snapshots(self.events)
        mutate(self.events, 1, lambda s: s['items'][0].update(command=22))
        with self.assertRaises(RuntimeError): validate_snapshots(self.events)

    def test_wrong_or_changing_catalog_reject(self):
        for key, value in [('execution_mode', 'RECEIVE_ONLY'), ('schema', 2)]:
            events = copy.deepcopy(self.events); mutate(events, -1, lambda c: c.update({key: value}))
            with self.assertRaises(RuntimeError): observed_catalog(events)
        self.events.append(copy.deepcopy(self.events[-1])); mutate(self.events, -1, lambda c: c.update(schema=2))
        with self.assertRaises(RuntimeError): observed_catalog(self.events)

    def test_control_provider_other_modules_and_extra_capability_reject(self):
        for fn in [lambda c: c['composition']['modules'][0].update(control_provider=True),
                   lambda c: c['composition']['modules'].append(c['composition']['modules'][0]),
                   lambda c: c['composition']['modules'][0]['capabilities'].append({'name': 'missions.start'}),
                   lambda c: c['composition'].update(plugin_abi=5)]:
            events = copy.deepcopy(self.events); mutate(events, -1, fn)
            with self.assertRaises(RuntimeError): observed_catalog(events)

    def test_expanded_catalog_requires_complete_surface_and_matching_abi(self):
        def expand(value):
            value['composition']['plugin_abi'] = 10
            value['composition']['modules'][0]['capabilities'].extend(
                {'name': name} for name in sorted(INSPECTION_CAPABILITIES))
        mutate(self.events, -1, expand)
        self.assertEqual(observed_catalog(self.events)['composition']['plugin_abi'], 10)
        mutate(self.events, -1, lambda c: c['composition']['modules'][0]['capabilities'].pop())
        with self.assertRaises(RuntimeError): observed_catalog(self.events)


if __name__ == '__main__':
    unittest.main(verbosity=2)
