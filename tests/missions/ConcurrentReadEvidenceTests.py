#!/usr/bin/env python3
"""Offline negative controls for HOME-only mission/parameter read flight proof."""
import importlib.util
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
def source_module(name, relative):
    path = ROOT / relative
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError('selected-source checker unavailable: ' + str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

checker = source_module('_home_only_read_checker', 'tests/concurrent/mission_read_evidence.py')
selector = source_module('_home_only_fixture_selector', 'scripts/firmware/scenario_fixtures.py')
PROFILE = 'arducopter-dev-04cf6486-quad'
pin = selector.PROFILES[PROFILE]
# Synthetic identities select the frozen baseline declaration for pure tests;
# they do not verify a binary or constitute native qualification.
BASELINE = selector.select(ROOT, {'schema': 1, 'profile_id': PROFILE,
    'manifest_sha256': pin['firmware_manifest_sha256'], 'source_revision': pin['source_revision'],
    'frame': 'quad', 'native_sha256': '1' * 64, 'attestation_sha256': '2' * 64,
    'qualification': 'not_inferred'}, 'armed_wiped_mission_read')

def validate(rows):
    return checker.mission_read_evidence(rows, BASELINE)


def event(kind, key, data):
    value = json.dumps(data) if kind == 'mission_snapshot' else ','.join(f'{k}={v}' for k, v in data.items())
    return {'type': kind, 'key': key, 'value': value}


def evidence():
    rows = [event('home_observation', 'MAVMOOS_HOME_POSITION', {'present': 'true', 'identity': 'EXPECTED', 'encoding_valid': 'true', 'freshness': 'REFERENCE_KNOWN', 'lat_deg': '42.3584560', 'lon_deg': '-71.0875890', 'alt_msl_m': '10.083'}),
            event('request', 'MAVMOOS_MISSION_REQUEST', {'id': 'concurrent-mission-read', 'action': 'download', 'timeout': '15'}),
            event('request', 'MAVMOOS_PARAM_REQUEST', {'id': 'concurrent-parameter-read', 'action': 'read', 'name': 'SIM_WIND_SPD'}),
            event('mission_snapshot', 'MAVMOOS_MISSION_SNAPSHOT', {'schema': 1, 'id': 'concurrent-mission-read', 'status': 'complete', 'count': 1, 'mission_type': 0, 'verification': 'observation', 'atomic': False, 'items': [{'seq': 0, 'command': 16, 'frame': 0, 'current': 0, 'autocontinue': 1, 'params': [0, 0, 0, 0], 'x': 423584560, 'y': -710875890, 'z': 10.08}]}),
            event('mission_result', 'MAVMOOS_MISSION_RESULT', {'id': 'concurrent-mission-read', 'action': 'download', 'status': 'ADMITTED', 'at': '1000', 'expires': '1015', 'received': '0', 'expected': '0', 'mutation_may_have_applied': 'false'}),
            event('mission_result', 'MAVMOOS_MISSION_RESULT', {'id': 'concurrent-mission-read', 'action': 'download', 'status': 'COMPLETED', 'at': '1000.2', 'expires': '1015', 'received': '1', 'expected': '1', 'mutation_may_have_applied': 'false'}),
            event('parameter_result', 'MAVMOOS_PARAM_RESULT', {'id': 'concurrent-parameter-read', 'status': 'COMPLETED'}),
            event('assertion', 'read_services_complete', {'mission_status': 'COMPLETED', 'parameter_status': 'COMPLETED', 'service_window_s': '.125', 'nav_at_completion': '102', 'mission_mutations': '0'}),
            event('assertion', 'mission_read_continuity', {'post_read_observation_s': '2.001', 'mission_request_list_sends': '1', 'mission_request_item_sends': '1', 'mission_ack_sends': '1', 'parameter_read_sends': '1', 'no_mutation': 'true'}),
            event('assertion', 'transfer_outcome', {'status': 'COMPLETED', 'nav_before': '100', 'nav_after': '150', 'loss_seen': 'false', 'max_observed_nav_counter_update_gap_s': '.22'})]
    rows += [event('mission_tx', 'MAVMOOS_MISSION_TX_RESULT', {'operation_id': 'concurrent-mission-read', 'msgid': ident, 'status': 'LOCAL_ACCEPTED', 'late': 'false', 'clock_valid': 'true'}) for ident in (43, 51, 47)]
    return [{'type': 'fixture', 'key': 'mission_shape', 'value': BASELINE['expectation']['shape']}] + rows


def patch(rows, kind, key, fn):
    item = next(e for e in rows if e['type'] == kind and e['key'] == key)
    value = json.loads(item['value']) if kind == 'mission_snapshot' else dict(p.split('=', 1) for p in item['value'].split(','))
    fn(value); item.update(event(kind, key, value))


class ReadEvidenceTests(unittest.TestCase):
    def test_complete_home_only_read_and_reference(self):
        result = validate(evidence())
        self.assertEqual(result['snapshot_count'], 1)
        self.assertEqual(result['mission_message_ids'], [43, 47, 51])
        self.assertFalse(result['mutation_requested'])

    def test_zero_and_extra_items_do_not_replace_home_only_proof(self):
        for count, items in [(0, []), (2, [{}, {}])]:
            rows = evidence(); patch(rows, 'mission_snapshot', 'MAVMOOS_MISSION_SNAPSHOT', lambda s: s.update(count=count, items=items))
            with self.subTest(count=count), self.assertRaises(RuntimeError): validate(rows)

    def test_missing_or_unbound_home_reference_rejects(self):
        rows = [e for e in evidence() if e['type'] != 'home_observation']
        with self.assertRaises(RuntimeError): validate(rows)
        for key, value in [('present', 'false'), ('encoding_valid', 'false'), ('identity', 'UNEXPECTED'), ('freshness', 'ABSENT'), ('lat_deg', 'nan'), ('lon_deg', '181'), ('alt_msl_m', 'inf')]:
            rows = evidence(); patch(rows, 'home_observation', 'MAVMOOS_HOME_POSITION', lambda s: s.update({key: value}))
            with self.subTest(key=key, value=value), self.assertRaises(RuntimeError): validate(rows)

    def test_home_row_identity_coordinates_altitude_and_flags_reject(self):
        for key, value in [('seq', 1), ('command', 22), ('frame', 3), ('x', 423584561), ('y', -710875891), ('z', 10.2), ('z', float('nan')), ('params', [0, 0, 0, 1]), ('current', 2), ('autocontinue', 0)]:
            rows = evidence(); patch(rows, 'mission_snapshot', 'MAVMOOS_MISSION_SNAPSHOT', lambda s: s['items'][0].update({key: value}))
            with self.subTest(key=key, value=value), self.assertRaises(RuntimeError): validate(rows)

    def test_mission_and_parameter_mutation_requests_reject(self):
        for kind, key, action in [('request', 'MAVMOOS_MISSION_REQUEST', 'clear'), ('request', 'MAVMOOS_MISSION_REQUEST', 'start'), ('request', 'MAVMOOS_PARAM_REQUEST', 'write')]:
            rows = evidence(); patch(rows, kind, key, lambda s: s.update(action=action))
            with self.subTest(key=key, action=action), self.assertRaises(RuntimeError): validate(rows)

    def test_outgoing_mutation_ids_or_missing_item_request_reject(self):
        for ident in (44, 45, 73):
            rows = evidence(); patch(rows, 'mission_tx', 'MAVMOOS_MISSION_TX_RESULT', lambda s: s.update(msgid=ident))
            with self.subTest(ident=ident), self.assertRaises(RuntimeError): validate(rows)
        rows = [e for e in evidence() if e['type'] != 'mission_tx' or 'msgid=51,' not in e['value']]
        with self.assertRaises(RuntimeError): validate(rows)

    def test_failed_late_or_uncorrelated_local_send_rejects(self):
        for key, value in [('status', 'NOT_ACCEPTED'), ('late', 'true'), ('clock_valid', 'false'), ('operation_id', 'old')]:
            rows = evidence(); patch(rows, 'mission_tx', 'MAVMOOS_MISSION_TX_RESULT', lambda s: s.update({key: value}))
            with self.subTest(key=key), self.assertRaises(RuntimeError): validate(rows)

    def test_missing_outcome_and_mutation_uncertainty_reject(self):
        for key, value in [('status', 'FAILED'), ('mutation_may_have_applied', 'true')]:
            rows = evidence(); patch(rows, 'mission_result', 'MAVMOOS_MISSION_RESULT', lambda s: s.update({key: value}))
            with self.assertRaises(RuntimeError): validate(rows)
        rows = [e for e in evidence() if e['type'] != 'parameter_result']
        with self.assertRaises(RuntimeError): validate(rows)

    def test_lost_stalled_or_unbounded_navigation_evidence_reject(self):
        for key, value in [('loss_seen', 'true'), ('nav_after', '100'), ('max_observed_nav_counter_update_gap_s', 'nan'), ('max_observed_nav_counter_update_gap_s', '1')]:
            rows = evidence(); patch(rows, 'assertion', 'transfer_outcome', lambda s: s.update({key: value}))
            with self.subTest(key=key), self.assertRaises(RuntimeError): validate(rows)
        rows = evidence(); patch(rows, 'assertion', 'read_services_complete', lambda s: s.update(mission_mutations='1'))
        with self.assertRaises(RuntimeError): validate(rows)

    def test_nonfinite_service_or_short_continuation_evidence_reject(self):
        rows = evidence(); patch(rows, 'assertion', 'read_services_complete', lambda s: s.update(service_window_s='nan'))
        with self.assertRaises(RuntimeError): validate(rows)
        rows = evidence(); patch(rows, 'assertion', 'mission_read_continuity', lambda s: s.update(post_read_observation_s='1'))
        with self.assertRaises(RuntimeError): validate(rows)


if __name__ == '__main__':
    unittest.main(verbosity=2)
