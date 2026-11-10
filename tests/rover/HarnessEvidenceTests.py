#!/usr/bin/env python3
"""Negative controls for the native wrapper's evidence gates; no simulator/I/O."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import run_rover


class Message:
    def __init__(self, kind, **values):
        self.kind, self.values = kind, values

    def get_type(self):
        return self.kind

    def to_dict(self):
        return self.values


class Reader:
    def __init__(self, values):
        self.values = iter(values)

    def recv_match(self, **_):
        return next(self.values, None)


class EvidenceTests(unittest.TestCase):
    def test_wildcard_is_allowed_only_for_owned_native_rc(self):
        snapshot = 'ardurover 123 user 8u IPv4 0 UDP *:63104\n'
        self.assertFalse(run_rover.owns_udp(snapshot, 63104))
        self.assertTrue(run_rover.owns_udp(snapshot, 63104, allow_wildcard=True))
        snapshot = 'ardurover 123 user 8u IPv4 0 UDP 127.0.0.1:53104->127.0.0.1:53105\n'
        self.assertTrue(run_rover.owns_udp(snapshot, 53104))
        self.assertFalse(run_rover.owns_udp(snapshot, 53105))

    def mode_records(self):
        return [Message('MODE', TimeUS=1, ModeNum=4, Rsn=1),
                Message('MODE', TimeUS=2, ModeNum=15, Rsn=9),
                Message('ARM', TimeUS=3, ArmState=1, Forced=0),
                Message('MODE', TimeUS=4, ModeNum=0, Rsn=1),
                Message('RCIN', TimeUS=5, C1=1500, C3=1500, C8=1000),
                Message('MODE', TimeUS=6, ModeNum=4, Rsn=1),
                Message('MODE', TimeUS=7, ModeNum=15, Rsn=9),
                Message('MODE', TimeUS=8, ModeNum=11, Rsn=9),
                Message('ARM', TimeUS=9, ArmState=0, Forced=0)]

    def proof(self, records):
        with patch.object(run_rover.mavutil, 'mavlink_connection', return_value=Reader(records)):
            return run_rover.dataflash(Path('fake.BIN'))

    def test_qualified_rc_takeover(self):
        self.assertEqual(self.proof(self.mode_records())['manual_neutral_samples'], 1)

    def test_mavlink_mode_change_is_not_pilot_takeover(self):
        records = self.mode_records(); records[3].values['Rsn'] = 9
        with self.assertRaises(RuntimeError):
            self.proof(records)

    def test_non_neutral_drive_input_rejects(self):
        records = self.mode_records(); records[4].values['C3'] = 1700
        with self.assertRaises(RuntimeError):
            self.proof(records)

    def test_forced_arm_rejects(self):
        records = self.mode_records(); records[2].values['Forced'] = 1
        with self.assertRaises(RuntimeError):
            self.proof(records)

    def test_missing_recovery_cannot_pass(self):
        records = self.mode_records(); del records[6]
        with self.assertRaises(StopIteration):
            self.proof(records)

    def test_missing_pilot_hold_cannot_pass(self):
        records = self.mode_records(); del records[5]
        with self.assertRaisesRegex(RuntimeError, 'pilot selection of Hold'):
            self.proof(records)

    def sample_catalog(self):
        names = ['control.arm', 'control.disarm', 'control.guided', 'control.hold.native_stop',
                 'control.return.native_rtl', 'control.navigation.helm', 'control.hold.guided_zero_velocity',
                 'control.operations.native', 'control.operations.guided', 'control.operations.cancel']
        def variable(name):
            if name.startswith('control.operations.'):
                return 'MAVMOOS_CONTROL_REQUEST'
            if name.endswith('zero_velocity'):
                return ''
            return 'MAVMOOS_NAV_CONTROL' if name == 'control.navigation.helm' else 'MAVMOOS_COMMAND'
        controls = {'name': 'ardurover_control', 'profile': 'ardurover', 'control_provider': True,
                    'request_owners': ['MAVMOOS_COMMAND', 'MAVMOOS_CONTROL_REQUEST', 'MAVMOOS_NAV_CONTROL'],
                    'capabilities': [{'name': name, 'access': 'internal' if name.endswith('zero_velocity') else 'request',
                                      'request_variable': variable(name)} for name in names]}
        return {'schema': 1, 'execution_mode': 'SITL_CONTROL', 'composition': {'plugin_abi': 10,
                'modules': [{'name': name, 'control_provider': False, 'capabilities': [], 'request_owners': []}
                            for name in ['telemetry', 'parameters']] + [controls]}}

    def check_catalog(self, catalogs):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'workflow.jsonl'
            path.write_text(''.join(json.dumps({'key': 'MAVMOOS_CAPABILITIES', 'value': json.dumps(item)})+'\n' for item in catalogs))
            return run_rover.catalog(path)

    def test_actual_rover_catalog_accepts(self):
        self.assertEqual(self.check_catalog([self.sample_catalog()])['composition']['plugin_abi'], 10)

    def test_old_abi_wrong_profile_or_flight_capability_reject(self):
        for mutation in ['abi', 'profile', 'flight', 'owner', 'operation', 'guided_stop']:
            item = self.sample_catalog()
            if mutation == 'abi':
                item['composition']['plugin_abi'] = 5
            elif mutation == 'profile':
                item['composition']['modules'][-1]['profile'] = 'arducopter'
            elif mutation == 'flight':
                item['composition']['modules'][-1]['capabilities'][0]['name'] = 'control.takeoff'
            elif mutation == 'owner':
                item['composition']['modules'][-1]['request_owners'].remove('MAVMOOS_CONTROL_REQUEST')
            elif mutation == 'operation':
                item['composition']['modules'][-1]['capabilities'][-1]['request_variable'] = 'MAVMOOS_COMMAND'
            else:
                item['composition']['modules'][-1]['capabilities'][6]['access'] = 'request'
            with self.assertRaises(RuntimeError, msg=mutation):
                self.check_catalog([item])

    def test_changing_observed_composition_rejects(self):
        item = self.sample_catalog(); changed = copy.deepcopy(item)
        changed['composition']['modules'][-1]['profile'] = 'boat'
        with self.assertRaises(RuntimeError):
            self.check_catalog([item, changed])


if __name__ == '__main__':
    unittest.main()
