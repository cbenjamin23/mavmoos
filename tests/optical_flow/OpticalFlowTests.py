#!/usr/bin/env python3
"""Validate strict JSON from the real optical-flow module and fake host."""

import argparse
from copy import deepcopy
import json
import subprocess
import unittest

from wire_fixture import EXPECTED, FLOW, KEYS, RAD, audit_loopback, entries


def reject_constant(value):
    raise ValueError(f"nonfinite JSON number {value}")


def unique_object(pairs):
    value = dict(pairs)
    if len(value) != len(pairs):
        raise ValueError("duplicate JSON key")
    return value


def sample(cases, phase, key, sensor=7):
    return entries(cases[phase][key], key)[sensor]


class OpticalFlowTests(unittest.TestCase):
    def test_scope_and_cadence(self):
        for phase, posts in CASES.items():
            if phase == "interval_suppressed":
                self.assertEqual(posts, {})
                continue
            self.assertEqual(posts.keys(), set(KEYS))
            for key, record in posts.items():
                entries(record, key)

    def test_raw_values_are_kept_in_their_family_and_sensor(self):
        for key, sensors in EXPECTED.items():
            rows = entries(CASES["fresh"][key], key)
            self.assertEqual(rows.keys(), sensors.keys())
            for sensor, (length, data) in sensors.items():
                row = rows[sensor]
                self.assertEqual((row["wire_version"], row["payload_length"], row["data"]), (2, length, data))
                self.assertTrue(row["metadata"]["fresh"])
        self.assertNotEqual(sample(CASES, "fresh", FLOW)["metadata"]["observation_id"],
                            sample(CASES, "fresh", RAD)["metadata"]["observation_id"])

    def test_independent_original_ages_and_order(self):
        initial = sample(CASES, "fresh", FLOW)["metadata"]
        self.assertEqual((initial["effective_at"], initial["read_at"], initial["age_basis"]),
                         (100.21, 100.25, "QUEUE_EMPTY_FLOOR"))
        self.assertIs(initial["age_is_upper_bound"], True)
        self.assertAlmostEqual(initial["age_s"], .13)
        after = sample(CASES, "identity_and_order", FLOW)["metadata"]
        self.assertEqual(after["source_rejected"], initial["source_rejected"] + 2)
        self.assertEqual(after["order_rejected"], initial["order_rejected"] + 1)
        for key in ("effective_at", "read_at", "observation_id"):
            self.assertEqual(after[key], initial[key])
        stale = sample(CASES, "independent_staleness", FLOW)["metadata"]
        self.assertFalse(stale["fresh"])
        self.assertEqual(stale["effective_at"], initial["effective_at"])
        self.assertAlmostEqual(stale["age_s"], 1.19)
        self.assertTrue(sample(CASES, "independent_staleness", FLOW, 201)["metadata"]["fresh"])
        rad = sample(CASES, "independent_staleness", RAD)
        self.assertTrue(rad["metadata"]["fresh"])
        self.assertEqual(rad["data"]["distance_m_raw"], -2)
        self.assertIsNone(rad["data"]["distance_m"])

    def test_malformed_packets_preserve_samples_and_increment_counters(self):
        self.assertEqual(CASES["malformed"][FLOW]["invalid_samples"], 5)
        self.assertEqual(CASES["malformed"][RAD]["invalid_samples"], 2)
        for key in KEYS:
            before = sample(CASES, "identity_and_order", key)
            after = sample(CASES, "malformed", key)
            self.assertEqual(after["metadata"]["wire_rejected"], before["metadata"]["wire_rejected"] + 7)
            self.assertEqual(after["metadata"]["observation_id"], before["metadata"]["observation_id"])
            self.assertEqual(after["data"], before["data"])

    def test_loss_rebind_clock_and_source_gates(self):
        for phase in ("absent", "prebind", "heartbeat_loss", "rebind_required", "clock_fault",
                      "reset_floor", "unknown_autopilot", "wrong_heartbeat_component"):
            self.assertTrue(all(not record["samples"] for record in CASES[phase].values()), phase)
        rebound = sample(CASES, "px4_rebind", RAD)["metadata"]
        self.assertEqual((rebound["system_id"], rebound["component_id"], rebound["autopilot"]), (1, 1, 12))
        self.assertEqual(rebound["source_age"], "unknown")
        self.assertGreater(rebound["session_epoch"], sample(CASES, "fresh", RAD)["metadata"]["session_epoch"])
        self.assertFalse(CASES["px4_rebind"][FLOW]["present"])

    def test_v1_and_valid_v2_trimming(self):
        legacy = sample(CASES, "v1", FLOW)
        self.assertEqual((legacy["wire_version"], legacy["payload_length"]), (1, 26))
        self.assertIsNone(legacy["data"]["flow_rate_x_rad_s"])
        self.assertIsNone(legacy["data"]["flow_rate_y_rad_s"])
        self.assertEqual(sample(CASES, "v1", RAD)["payload_length"], 44)
        partial = sample(CASES, "partial_v2", FLOW, 8)
        self.assertEqual((partial["wire_version"], partial["payload_length"]), (2, 27))
        self.assertGreater(partial["data"]["flow_rate_x_rad_s"], 0)
        self.assertIsNone(partial["data"]["flow_rate_y_rad_s"])
        minimal = sample(CASES, "partial_v2", RAD, 0)
        self.assertEqual((minimal["wire_version"], minimal["payload_length"]), (2, 1))
        self.assertEqual(minimal["data"]["time_usec_raw"], "1")
        self.assertEqual(minimal["data"]["integration_time_us"], 0)
        self.assertEqual(minimal["data"]["quality_raw"], 0)
        self.assertEqual(minimal["data"]["distance_m"], 0)

    def test_sensor_capacity_is_per_family_and_releases_on_reset(self):
        flow = CASES["capacity"][FLOW]
        self.assertEqual(list(entries(flow, FLOW)), [1, 8, 9, 22, 63, 109, 200, 255])
        self.assertEqual(flow["capacity_drops"], 1)
        self.assertEqual(sample(CASES, "capacity", FLOW, 255)["data"]["quality_raw"], 255)
        self.assertEqual(list(entries(CASES["capacity"][RAD], RAD)), [7])
        reset = CASES["capacity_reset"][FLOW]
        self.assertEqual(list(entries(reset, FLOW)), [99])
        self.assertEqual(reset["capacity_drops"], 1)
        self.assertFalse(CASES["capacity_reset"][RAD]["present"])

    def test_app_capture_audit_rejects_relabelled_or_renewed_samples(self):
        prebind, prefix, full = {}, {}, {}
        for key in KEYS:
            empty = CASES["absent"][key]
            fresh = deepcopy(CASES["fresh"][key])
            for row in fresh["samples"]:
                row["metadata"].update(age_basis="QUEUE_EMPTY_FLOOR", age_is_upper_bound=True)
            stale = deepcopy(fresh)
            for row in stale["samples"]:
                row["metadata"].update(age_s=1.2, fresh=False)
            rebound = deepcopy(fresh)
            for row in rebound["samples"]:
                row["metadata"]["session_epoch"] += 1
                row["metadata"]["observation_id"] += 10
            prebind[key] = [empty]
            prefix[key] = [empty, fresh, stale, empty, empty]
            full[key] = [*prefix[key], rebound]
        self.assertEqual(audit_loopback(prebind, prefix, full, 3)[FLOW]["recovered"], [7, 201])

        changed_prefix = deepcopy(prefix)
        changed_full = deepcopy(full)
        for capture in (changed_prefix, changed_full):
            capture[FLOW][2]["samples"][0]["metadata"]["effective_at"] += .01
        with self.assertRaisesRegex(ValueError, "renewed sensor"):
            audit_loopback(prebind, changed_prefix, changed_full, 3)

        changed_full = deepcopy(full)
        changed_full[FLOW][-1]["samples"][1]["data"]["flow_rate_y_rad_s"] = 0
        with self.assertRaisesRegex(ValueError, "wire values changed"):
            audit_loopback(prebind, prefix, changed_full, 3)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--driver", required=True)
    parser.add_argument("--module", required=True)
    args = parser.parse_args()
    output = subprocess.check_output([args.driver, args.module], text=True, timeout=10)
    decoded = (json.loads(line, parse_constant=reject_constant, object_pairs_hook=unique_object)
               for line in output.splitlines())
    CASES = {row["phase"]: row["posts"] for row in decoded}
    unittest.main(argv=[__file__], verbosity=2)
