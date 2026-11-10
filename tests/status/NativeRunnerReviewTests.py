#!/usr/bin/env python3
"""Offline ownership/fixture evidence checks; no process/socket/simulator startup."""
from pathlib import Path
from types import SimpleNamespace
import unittest
import tempfile
from unittest.mock import patch
import run_status_observations as runner

class RunnerReviewTests(unittest.TestCase):
    def test_helpers_resolve_explicit_source_even_with_cached_other_checkout(self):
        with tempfile.TemporaryDirectory() as temporary:
            source=Path(temporary);(source/'tests/control').mkdir(parents=True)
            (source/'tests/owned_processes.py').write_text('marker="requested ownership"\n')
            (source/'tests/control/catalog_evidence.py').write_text('marker="requested ABI7 catalog"\n')
            # An old module with the normal import name must not be consulted.
            with patch.dict('sys.modules',{'control.catalog_evidence':SimpleNamespace(marker='stale ABI6')}):
                modules=runner.source_helpers(source)
            self.assertEqual(modules['ownership'].marker,'requested ownership')
            self.assertEqual(modules['catalog'].marker,'requested ABI7 catalog')
            self.assertEqual(Path(modules['catalog'].__file__),(source/'tests/control/catalog_evidence.py').resolve())

    def test_exact_live_app_and_bound_socket_are_required(self):
        expected={'pid':101,'command':'/owned/iMAVLink /owned/mission.moos'}
        inventory='iMAVLink 101 user UDP 127.0.0.1:4242\n'
        with patch.object(runner,'exact_app',return_value=expected),patch.object(runner.subprocess,'run',return_value=SimpleNamespace(returncode=0,stdout=inventory)):
            self.assertEqual(runner.live_app(None,Path('/owned/mission.moos'),4242,expected),(expected,inventory))

    def test_dead_or_replaced_app_cannot_use_retained_moos_evidence(self):
        expected={'pid':101,'command':'/owned/iMAVLink /owned/mission.moos'}
        with patch.object(runner,'exact_app',side_effect=RuntimeError('no owned app')),self.assertRaises(RuntimeError):runner.live_app(None,None,4242,expected)
        for replacement in ({'pid':102,'command':expected['command']},{'pid':101,'command':'/different/app'}):
            with patch.object(runner,'exact_app',return_value=replacement),self.assertRaises(RuntimeError):runner.live_app(None,None,4242,expected)

    def test_missing_or_wrong_bound_socket_fails(self):
        record={'pid':101,'command':'/owned/iMAVLink /owned/mission.moos'}
        for reply in (SimpleNamespace(returncode=1,stdout=''),SimpleNamespace(returncode=0,stdout='UDP 127.0.0.1:42420\n')):
            with patch.object(runner,'exact_app',return_value=record),patch.object(runner.subprocess,'run',return_value=reply),self.assertRaises(RuntimeError):runner.live_app(None,None,4242,record)

    def test_late_identified_battery_zero_wire_voltage_matches_fixture(self):
        packets=[{'mavpackettype':'BATTERY_STATUS','id':0,'received_mono':at,'voltages':[12600]+[65535]*9,'voltages_ext':[0]*4} for at in (23,24)]
        samples=runner.observed_battery_zero(packets,25)
        self.assertEqual([s['reported_voltage_sum_v'] for s in samples],[12.6,12.6])

    def test_early_unknown_wrong_id_or_zero_voltage_wire_cannot_qualify(self):
        base={'mavpackettype':'BATTERY_STATUS','id':0,'received_mono':24,'voltages':[12600]+[65535]*9,'voltages_ext':[0]*4}
        for mutation in ({'id':4},{'received_mono':1},{'voltages':[0]+[65535]*9},{'voltages':[65535]*10},{'voltages':[12600]+[65535]*8+[1000]}):
            packets=[{**base,**mutation},{**base,'received_mono':23,**mutation}]
            with self.subTest(mutation=mutation),self.assertRaises(RuntimeError):runner.observed_battery_zero(packets,25)

    def test_duplicate_wire_timestamp_or_invalid_slot_cannot_qualify(self):
        packet={'mavpackettype':'BATTERY_STATUS','id':0,'received_mono':24,'voltages':[12600]+[65535]*9,'voltages_ext':[0]*4}
        with self.assertRaises(RuntimeError):runner.observed_battery_zero([packet,packet],25)
        packet['voltages'][0]=True
        with self.assertRaises(RuntimeError):runner.observed_battery_zero([packet],25)

if __name__=='__main__':unittest.main()
