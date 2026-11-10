#!/usr/bin/env python3
"""Mutation controls on a verified closed native capture; never start native I/O.

Needs the ArduPilot virtualenv's pymavlink to read the retained closed BIN.
This is separate from the stdlib-only synthetic audit test used by base CTest.
"""
import argparse
import copy
import json
from pathlib import Path
import unittest
from adjudicate_rover import adjudicate, extract_dataflash
from RcNativeAudit import audit, parse
from replay_rc_run import load_closed_archive

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--archive', type=Path, required=True);parser.add_argument('--expected-sha256', required=True)
args, remaining = parser.parse_known_args()
ORIGINAL, CONTENT, DIGEST = load_closed_archive(args.archive, args.expected_sha256)
RECORDS, BIN_SHA = extract_dataflash(CONTENT, ORIGINAL)
EVENTS = [parse(line) for line in CONTENT['rc-probe.jsonl'].decode().splitlines() if line.startswith('{')]
PACKETS = [parse(line) for line in CONTENT['fc-monitor.jsonl'].decode().splitlines() if line]
WORKFLOW = [parse(line) for line in CONTENT['workflow.jsonl'].decode().splitlines() if line.startswith('{')]
FINAL = audit(EVENTS, mode='SITL_CONTROL', duration=210)['windows'][-1]


class AdjudicationControls(unittest.TestCase):
    def setUp(self):
        self.original = copy.deepcopy(ORIGINAL);self.events = copy.deepcopy(EVENTS)
        self.packets = copy.deepcopy(PACKETS);self.records = copy.deepcopy(RECORDS)

    def run_case(self):return adjudicate(self.original, self.events, self.packets, self.records, WORKFLOW)

    def test_closed_capture_qualifies_only_stated_scope(self):
        result = self.run_case()
        self.assertEqual(result['result'], 'PASS_SCOPED_OBSERVATIONS')
        self.assertEqual(result['original_runner_result'], 'FAIL');self.assertFalse(result['new_native_execution'])
        self.assertGreaterEqual(result['final_stationary_disarmed']['source_span_s'], 2)
        self.assertEqual(ORIGINAL['result'], 'FAIL')

    def test_one_falsified_final_servo_value_is_rejected(self):
        target = FINAL['servo_samples'][0]['observation_id']
        for event in self.events:
            if event.get('key') != 'MAVMOOS_SERVO_OUTPUT':continue
            record = parse(event['value'])
            for port in record['ports']:
                if port['observation_id'] == target:
                    channel = port['data']['channels'][0];channel['raw_us'] += 1;channel['pwm_us']['value'] = channel['raw_us']
            event['value'] = json.dumps(record)
        with self.assertRaisesRegex(ValueError, 'servo command values'):self.run_case()

    def test_missing_final_position_window_is_rejected(self):
        stamps = [row['data']['time_usec'] for row in FINAL['servo_samples']]
        begin, end = min(stamps) / 1000 - 100, max(stamps) / 1000 + 100
        self.packets = [row for row in self.packets if row.get('mavpackettype') != 'GLOBAL_POSITION_INT' or not begin <= row['time_boot_ms'] <= end]
        with self.assertRaisesRegex(ValueError, 'final stationary window'):self.run_case()

    def test_RC_input_mismatch_is_rejected(self):
        for row in self.packets:
            if row.get('mavpackettype') == 'RC_CHANNELS':row['chan1_raw'] += 7
        with self.assertRaisesRegex(ValueError, 'RC input phase values'):self.run_case()

    def test_final_RCOU_mismatch_is_rejected(self):
        for row in self.records['RCOU']:row['C1'] += 1
        with self.assertRaisesRegex(ValueError, 'RCOU values'):self.run_case()

    def test_claimed_stationary_window_must_be_stationary(self):
        for row in self.packets:
            if row.get('mavpackettype') == 'GLOBAL_POSITION_INT':row['vx'] = 101
        with self.assertRaisesRegex(ValueError, 'not stationary'):self.run_case()

    def test_stationary_window_must_have_fresh_disarmed_FC_evidence(self):
        for row in self.packets:
            if row.get('mavpackettype') == 'HEARTBEAT':row['base_mode'] |= 128
        with self.assertRaisesRegex(ValueError, 'not disarmed'):self.run_case()

    def test_moving_range_plausibility_cannot_replace_two_exact_matches(self):
        initial = audit(EVENTS, mode='SITL_CONTROL', duration=210)['windows'][0]
        stamps = [row['data']['time_usec'] for row in initial['servo_samples']]
        self.packets = [row for row in self.packets if row.get('mavpackettype') != 'SERVO_OUTPUT_RAW' or not min(stamps) - 350000 <= row['time_usec'] <= max(stamps) + 350000]
        with self.assertRaisesRegex(ValueError, 'two independently exact'):self.run_case()

    def test_other_native_failure_is_not_adjudicated_away(self):
        self.original['error'] = 'native process exited'
        with self.assertRaisesRegex(ValueError, 'not the preserved'):self.run_case()

    def test_source_or_cleanup_failure_is_not_adjudicated_away(self):
        self.original['artifact_drift'] = ['Runtime.cpp']
        with self.assertRaisesRegex(ValueError, 'source/resource'):self.run_case()


if __name__ == '__main__':unittest.main(argv=[__file__] + remaining)
