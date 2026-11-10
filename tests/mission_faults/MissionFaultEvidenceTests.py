#!/usr/bin/env python3
"""Negative controls for actual fixture evidence; no MOOS/socket/simulator I/O."""
import copy
import json
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mission_fault_evidence import validate


def kv(values):
    return ','.join(f'{key}={value}' for key, value in values.items())


def evidence(scenario='cancel'):
    events = []
    def event(kind, key, value):
        events.append({'elapsed': len(events) * .1, 'type': kind, 'key': key,
                       'value': json.dumps(value) if kind in ('capabilities', 'snapshot') else kv(value) if isinstance(value, dict) else value})
    names = ['missions.download', 'missions.upload.simulator', 'missions.clear.simulator', 'missions.cancel', 'missions.progress']
    event('capabilities', 'MAVMOOS_CAPABILITIES', {'schema': 1, 'execution_mode': 'SITL_CONTROL',
        'composition': {'plugin_abi': 9, 'modules': [{'name': 'missions', 'control_provider': False,
            'request_owners': ['MAVMOOS_MISSION_REQUEST'], 'capabilities': [{'name': name} for name in names]}]}})
    event('runtime', 'MAVMOOS_RUNTIME_STATUS', {'armed': 0, 'permission': 'false', 'nav_tx_accepted': 0, 'control_profile': 'none'})
    def result(ident, status, **extra):
        event('result', 'MAVMOOS_MISSION_RESULT', dict(id=ident, action='upload' if ident == 'mf-upload' else 'clear' if ident in ('mf-baseline', 'mf-final-clear') else 'download', status=status,
            reason='test', expires=600, at=500 + len(events) * .1, mutation_may_have_applied='false', remote_accepted='false', readback_confirmed='false', **extra))
    def snapshot(ident, rows, verification):
        event('snapshot', 'MAVMOOS_MISSION_SNAPSHOT', {'schema': 1, 'id': ident, 'status': 'complete',
            'mission_type': 0, 'count': len(rows), 'items': rows, 'verification': verification, 'atomic': False})
    event('assertion', 'stable_home_ground_fixture', 'home_initialized=true,armed=false,permission=false')
    event('request', 'MAVMOOS_MISSION_REQUEST', 'id=mf-baseline,action=clear,timeout=20')
    result('mf-baseline', 'COMPLETED');snapshot('mf-baseline', [], 'semantic_profile_match')
    upload = 'id=mf-upload,action=upload,timeout=30,items=' + '|'.join(
        f'{seq}:16:{0 if seq == 0 else 3}:0:1:0:0:0:0:{423584560 + seq * 10}:{-710875890 + seq * 5}:{10 if seq == 0 else 5}' for seq in range(64))
    event('request', 'MAVMOOS_MISSION_REQUEST', upload)
    result('mf-upload', 'ADMITTED')
    for seq, message in [(0, 44), (0, 73), (1, 73), (2, 73), (3, 73), (4, 73)]:
        event('tx', 'MAVMOOS_MISSION_TX_RESULT', {'operation_id': 'mf-upload', 'msgid': message, 'status': 'LOCAL_ACCEPTED', 'late': 'false', 'clock_valid': 'true', 'at': 500 + len(events) * .1})
        result('mf-upload', 'SENT', seq=seq)
    event('assertion', 'partial_trigger', 'sent_seq=4,requested_count=64')
    if scenario == 'cancel':event('request', 'MAVMOOS_MISSION_REQUEST', 'id=mf-upload,action=cancel')
    terminal = {'id': 'mf-upload', 'action': 'upload', 'status': 'CANCELLED', 'reason': 'operator_cancelled' if scenario == 'cancel' else 'connection_reset',
                'expires': 600, 'at': 503, 'mutation_may_have_applied': 'true', 'remote_accepted': 'false', 'readback_confirmed': 'false'}
    event('result', 'MAVMOOS_MISSION_RESULT', terminal);event('assertion', 'retirement', terminal)
    event('request', 'MAVMOOS_MISSION_REQUEST', upload);result('mf-upload', 'DUPLICATE')
    event('assertion', 'old_id_not_replayed', 'status=DUPLICATE')
    event('request', 'MAVMOOS_MISSION_REQUEST', 'id=mf-busy-read,action=download,timeout=5')
    event('tx', 'MAVMOOS_MISSION_TX_RESULT', {'operation_id': 'mf-busy-read', 'msgid': 43, 'status': 'LOCAL_ACCEPTED', 'late': 'false', 'clock_valid': 'true', 'at': 504})
    event('result', 'MAVMOOS_MISSION_RESULT', {'id': 'mf-busy-read', 'action': 'download', 'status': 'FAILED', 'reason': 'remote_mission_error_14', 'expires': 600, 'at': 505})
    event('assertion', 'remote_busy_after_local_retirement', 'status=FAILED,reason=remote_mission_error_14')
    event('assertion', 'settlement_wait', 'elapsed_since_retirement_s=10.501,native_source_timeout_s=8,remote_timeout_observed=false')
    event('request', 'MAVMOOS_MISSION_REQUEST', 'id=mf-readback,action=download,timeout=20')
    rows = [{'seq': seq, 'command': 16, 'frame': 0 if seq == 0 else 3, 'current': 0, 'autocontinue': 1,
             'params': [0, 0, 0, 0], 'x': 0 if seq == 0 else 423584560 + seq * 10,
             'y': 0 if seq == 0 else -710875890 + seq * 5, 'z': 0 if seq == 0 else 5} for seq in range(5)]
    snapshot('mf-readback', rows, 'observation');result('mf-readback', 'COMPLETED')
    event('assertion', 'partial_remote_storage', 'count=5,requested_count=64,rollback=false')
    event('request', 'MAVMOOS_MISSION_REQUEST', 'id=mf-final-clear,action=clear,timeout=20')
    snapshot('mf-final-clear', [], 'semantic_profile_match');result('mf-final-clear', 'COMPLETED')
    event('assertion', 'final_idle', 'mission_queue=0,mission_operations=0,mission_mutations=0,armed=false,permission=false')
    event('workflow', 'result', 'PASS')
    wait_index = next(i for i, e in enumerate(events) if e['type'] == 'assertion' and e['key'] == 'settlement_wait')
    for entry in events[wait_index:]:
        entry['elapsed'] += 10.5
    return events


def change_record(rows, predicate, values):
    record = next(e for e in rows if predicate(e));data = dict(p.split('=', 1) for p in record['value'].split(','));data.update(values);record['value'] = kv(data)


class EvidenceTests(unittest.TestCase):
    def test_both_real_case_shapes(self):
        for case in ('cancel', 'reset'):
            with self.subTest(case=case):
                outcome = validate(evidence(case), case)
                self.assertEqual(outcome['partial_count'], 5)
                self.assertFalse(outcome['remote_timeout_observed'])

    def test_retirement_must_keep_uncertainty_and_reason(self):
        for key, value in [('mutation_may_have_applied', 'false'), ('remote_accepted', 'true'), ('readback_confirmed', 'true'), ('reason', 'unknown')]:
            rows = evidence();change_record(rows, lambda e: e['type'] == 'result' and 'status=CANCELLED' in e['value'], {key: value})
            with self.subTest(key=key), self.assertRaises(RuntimeError):validate(rows, 'cancel')

    def test_original_expiry_finite_and_unchanged(self):
        for value in ('nan', 'inf', '0', '601'):
            rows = evidence();change_record(rows, lambda e: e['type'] == 'result' and 'status=CANCELLED' in e['value'], {'expires': value})
            with self.subTest(value=value), self.assertRaises(RuntimeError):validate(rows, 'cancel')

    def test_no_successful_send_or_replay_after_retirement(self):
        for kind, value in [('tx', 'operation_id=mf-upload,msgid=73,status=LOCAL_ACCEPTED,late=false,clock_valid=true,at=504'),
                            ('result', 'id=mf-upload,action=upload,status=SENT,reason=test,expires=600,at=504,seq=5')]:
            rows = evidence();at = next(i for i, e in enumerate(rows) if e['type'] == 'result' and 'status=CANCELLED' in e['value'])
            rows.insert(at+1, {'elapsed': rows[at]['elapsed'], 'type': kind, 'key': 'test', 'value': value})
            with self.subTest(kind=kind), self.assertRaises(RuntimeError):validate(rows, 'cancel')

    def test_delayed_receipt_is_ordered_by_producer_clock(self):
        rows = evidence();index = next(i for i, e in enumerate(rows) if e['type'] == 'tx' and 'msgid=44,' in e['value']);delayed = rows.pop(index)
        terminal = next(i for i, e in enumerate(rows) if e['type'] == 'result' and 'status=CANCELLED' in e['value']);delayed['elapsed'] = rows[terminal]['elapsed'];rows.insert(terminal + 1, delayed)
        self.assertEqual(validate(rows, 'cancel')['retired_local_accepted_at_or_after_terminal'], 0)
        # Conversely, an early-delivered receipt with a later producer stamp fails.
        rows = evidence();change_record(rows, lambda e: e['type'] == 'tx', {'at': '504'})
        with self.assertRaises(RuntimeError):validate(rows, 'cancel')
        for value in ('nan', 'inf', '-1', '503'):
            rows = evidence();change_record(rows, lambda e: e['type'] == 'tx', {'at': value})
            with self.subTest(at=value), self.assertRaises(RuntimeError):validate(rows, 'cancel')

    def test_recovery_uses_its_own_fixed_finite_unexpired_deadline(self):
        for ident in ('mf-baseline', 'mf-busy-read', 'mf-readback', 'mf-final-clear'):
            for value in ('nan', '0', '500'):
                rows = evidence();change_record(rows, lambda e: e['type'] == 'result' and f'id={ident},' in e['value'], {'expires': value})
                with self.subTest(ident=ident, value=value), self.assertRaises(RuntimeError):validate(rows, 'cancel')

    def test_missing_actual_local_mutation_acceptance_rejects(self):
        rows = [e for e in evidence() if e['type'] != 'tx']
        with self.assertRaises(RuntimeError):validate(rows, 'cancel')

    def test_no_motion_execution_request_or_provider(self):
        rows = evidence();rows.insert(-1, {'elapsed': rows[-1]['elapsed'], 'type': 'request', 'key': 'MAVMOOS_COMMAND', 'value': 'id=bad,action=arm'})
        with self.assertRaises(RuntimeError):validate(rows, 'cancel')
        rows = evidence();change_record(rows, lambda e: e['type'] == 'runtime', {'permission': 'true'})
        with self.assertRaises(RuntimeError):validate(rows, 'cancel')
        rows = evidence();change_record(rows, lambda e: e['type'] == 'runtime', {'armed': '1'})
        with self.assertRaises(RuntimeError):validate(rows, 'cancel')

    def test_partial_row_mutation_sequence_and_nonfinite_reject(self):
        for key, value in [('x', 423584999), ('seq', 99), ('z', float('nan')), ('z', True), ('current', 2), ('params', [0, 0, 0, 1])]:
            rows = evidence();record = next(e for e in rows if e['type'] == 'snapshot' and 'mf-readback' in e['value']);snapshot = json.loads(record['value']);snapshot['items'][1][key] = value;record['value'] = json.dumps(snapshot)
            with self.subTest(key=key), self.assertRaises(RuntimeError):validate(rows, 'cancel')

    def test_home_observation_is_not_compared_to_placeholder_but_is_valid(self):
        rows = evidence();record = next(e for e in rows if e['type'] == 'snapshot' and 'mf-readback' in e['value']);snapshot = json.loads(record['value']);snapshot['items'][0].update(x=423584558, y=-710875890, z=10.09);record['value'] = json.dumps(snapshot)
        self.assertEqual(validate(rows, 'cancel')['partial_count'], 5)
        for value in (float('nan'), 900000001, True):
            bad = copy.deepcopy(rows);record = next(e for e in bad if e['type'] == 'snapshot' and 'mf-readback' in e['value']);s = json.loads(record['value']);s['items'][0]['x'] = value;record['value'] = json.dumps(s)
            with self.subTest(value=value), self.assertRaises(RuntimeError):validate(bad, 'cancel')

    def test_complete_or_empty_plan_does_not_qualify_partial_storage(self):
        for size in (0, 1, 64):
            rows = evidence();record = next(e for e in rows if e['type'] == 'snapshot' and 'mf-readback' in e['value']);s = json.loads(record['value']);s['count'] = size;record['value'] = json.dumps(s)
            with self.subTest(size=size), self.assertRaises(RuntimeError):validate(rows, 'cancel')

    def test_rows_not_locally_sent_cannot_be_claimed_as_fixture_prefix(self):
        rows = evidence();record = next(e for e in rows if e['type'] == 'snapshot' and 'mf-readback' in e['value']);s = json.loads(record['value'])
        s['items'].append(dict(s['items'][-1], seq=5, x=423584610, y=-710875865));s['count'] = 6;record['value'] = json.dumps(s)
        with self.assertRaises(RuntimeError):validate(rows, 'cancel')

    def test_settlement_is_bounded_wait_not_fabricated_timeout_observation(self):
        for values in ({'elapsed_since_retirement_s': '7.9'}, {'elapsed_since_retirement_s': 'nan'}, {'remote_timeout_observed': 'true'}):
            rows = evidence();change_record(rows, lambda e: e['type'] == 'assertion' and e['key'] == 'settlement_wait', values)
            with self.subTest(values=values), self.assertRaises(RuntimeError):validate(rows, 'cancel')

    def test_raw_probe_chronology_must_support_reported_settlement(self):
        rows = evidence();index = next(i for i, e in enumerate(rows) if e['type'] == 'assertion' and e['key'] == 'settlement_wait')
        for entry in rows[index:]:entry['elapsed'] -= 10.5
        with self.assertRaises(RuntimeError):validate(rows, 'cancel')
        rows = evidence();requests = [e for e in rows if e['type'] == 'request'];requests[-2]['value'],requests[-1]['value'] = requests[-1]['value'],requests[-2]['value']
        with self.assertRaises(RuntimeError):validate(rows, 'cancel')

    def test_remote_denial_is_distinct_from_local_retirement(self):
        rows = evidence();change_record(rows, lambda e: e['type'] == 'result' and 'mf-busy-read' in e['value'], {'reason': 'local_timeout'})
        with self.assertRaises(RuntimeError):validate(rows, 'cancel')

    def test_clear_must_really_be_verified_empty(self):
        rows = evidence();record = next(e for e in rows if e['type'] == 'snapshot' and 'mf-final-clear' in e['value']);s = json.loads(record['value']);s['verification'] = 'observation';record['value'] = json.dumps(s)
        with self.assertRaises(RuntimeError):validate(rows, 'cancel')

    def test_bad_clock_and_catalog_reject(self):
        for value in (float('nan'), -1, True):
            rows = evidence();rows[1]['elapsed'] = value
            with self.subTest(value=value), self.assertRaises(RuntimeError):validate(rows, 'cancel')
        rows = evidence();record = next(e for e in rows if e['type'] == 'capabilities');s = json.loads(record['value']);s['composition']['plugin_abi'] = 6;record['value'] = json.dumps(s)
        with self.assertRaises(RuntimeError):validate(rows, 'cancel')


if __name__ == '__main__':
    unittest.main()
