#!/usr/bin/env python3
"""Exercise the actual ESC module with generated wire payloads and fake Services."""
import argparse
import json
import math
import subprocess
import unittest


def finite_float(text):
    value = float(text)
    if not math.isfinite(value):
        raise ValueError("nonfinite float in ESC publication")
    return value


def reject_constant(text):
    raise ValueError("nonfinite JSON constant in ESC publication: " + text)


class EscStatusTests(unittest.TestCase):
    def report(self, case, family):
        return self.reports[case, "MAVMOOS_ESC_" + family]

    def group(self, case, family):
        groups = self.report(case, family)["groups"]
        self.assertEqual(len(groups), 1)
        return groups[0]

    def test_families_remain_separate_and_observation_only(self):
        expected_keys = {"MAVMOOS_ESC_INFO", "MAVMOOS_ESC_STATUS", "MAVMOOS_ESC_TELEMETRY"}
        self.assertEqual({key for _, key in self.reports}, expected_keys)
        for report in self.reports.values():
            self.assertEqual(report["schema"], 1)
            self.assertTrue(report["read_only"])
            self.assertFalse(report["authoritative_for_control"])
            self.assertFalse(report["atomic"])
            self.assertEqual(report["device_presence"], "unknown")
            self.assertEqual(report["health"], "unassessed")
            self.assertEqual(report["present"], bool(report["groups"]))
        info = self.group("first_group", "INFO")
        status = self.group("first_group", "STATUS")
        telemetry = self.group("first_group", "TELEMETRY")
        self.assertEqual([row["message_id"] for row in (info, status, telemetry)], [290, 291, 11030])
        self.assertEqual([row["first_index"] for row in (info, status, telemetry)], [4, 0, 0])
        self.assertEqual(len({row["metadata"]["observation_id"] for row in (info, status, telemetry)}), 3)

    def test_info_limits_partial_groups_and_sentinel(self):
        group = self.group("first_group", "INFO")
        self.assertEqual(group["count"], 6)
        self.assertEqual(group["time_usec_raw"], "123456789")
        self.assertEqual(group["counter"], 21)
        self.assertEqual(group["connection_type"], 1)
        self.assertEqual(group["info_raw"], 0x9b)
        self.assertEqual(group["entries"], [
            {"index": 4, "temperature_cdeg": 3125, "error_count": 11, "failure_flags": 0},
            {"index": 5, "temperature_cdeg": None, "error_count": 12, "failure_flags": 0x4000},
        ])
        for family in ("INFO", "STATUS"):
            full = self.report("full_common_groups", family)["groups"]
            invalid = self.report("invalid_group_shapes", family)["groups"]
            self.assertEqual([group["first_index"] for group in full], list(range(0, 64, 4)))
            self.assertEqual([entry["index"] for group in full for entry in group["entries"]], list(range(64)))
            self.assertEqual([row["entries"] for row in invalid], [row["entries"] for row in full])
            self.assertEqual([row["metadata"]["observation_id"] for row in invalid],
                             [row["metadata"]["observation_id"] for row in full])
            self.assertTrue(all(row["metadata"]["wire_rejected"] == 4 for row in invalid))
        empty = self.report("count_zero", "INFO")["groups"][0]
        self.assertEqual((empty["first_index"], empty["count"], empty["entries"]), (0, 0, []))

    def test_status_preserves_signed_values_and_marks_nonfinite_unknown(self):
        group = self.group("first_group", "STATUS")
        self.assertEqual(group["time_usec_raw"], "123456790")
        self.assertEqual(group["entries"], [
            {"index": 0, "rpm": 1700, "voltage_v": 16.75, "current_a": .75},
            {"index": 1, "rpm": -1200, "voltage_v": 15.5, "current_a": -.25},
            {"index": 2, "rpm": 0, "voltage_v": None, "current_a": None},
            {"index": 3, "rpm": 450, "voltage_v": 0, "current_a": 0},
        ])

    def test_ardupilot_group_retains_raw_units_and_wrapping_counter(self):
        group = self.group("first_group", "TELEMETRY")
        self.assertEqual(len(group["entries"]), 4)
        self.assertEqual(group["entries"][0], {
            "index": 0, "voltage_cv": 1680, "current_ca": 80, "total_current_mah": 1,
            "rpm_raw": 350, "packet_count": 65535, "temperature_c": 32,
        })
        self.assertEqual(group["entries"][3], {
            "index": 3, "voltage_cv": 0, "current_ca": 0, "total_current_mah": 0,
            "rpm_raw": 0, "packet_count": 0, "temperature_c": 0,
        })
        wrapped = self.group("counter_wrap", "TELEMETRY")
        self.assertEqual(wrapped["entries"][0]["packet_count"], 0)
        self.assertEqual(wrapped["metadata"]["session_epoch"], group["metadata"]["session_epoch"])
        self.assertTrue(wrapped["metadata"]["fresh"])

    def test_group_age_is_original_and_updates_independently(self):
        first = self.group("first_group", "INFO")["metadata"]
        cached = self.group("independent_ages", "INFO")["metadata"]
        self.assertEqual(first["age_basis"], "QUEUE_EMPTY_FLOOR")
        self.assertTrue(first["age_is_upper_bound"])
        self.assertEqual(first["source_age"], "unknown")
        self.assertEqual((first["system_id"], first["component_id"], first["autopilot"]), (1, 1, 3))
        self.assertEqual((first["effective_at"], first["read_at"]), (100.4, 100.5))
        for key in ("observation_id", "effective_at", "read_at", "session_epoch"):
            self.assertEqual(first[key], cached[key])
        self.assertAlmostEqual(cached["age_s"], 1.35)
        self.assertTrue(first["fresh"])
        self.assertFalse(cached["fresh"])
        status = self.report("independent_ages", "STATUS")["groups"]
        self.assertEqual([group["first_index"] for group in status], [0, 60])
        self.assertEqual([group["metadata"]["fresh"] for group in status], [False, True])
        self.assertFalse(self.group("independent_ages", "TELEMETRY")["metadata"]["fresh"])

    def test_source_and_time_rejections_do_not_renew_data(self):
        first = self.group("before_rejections", "STATUS")["metadata"]
        rejected = self.group("rejections", "STATUS")["metadata"]
        accepted = self.group("after_rejections", "STATUS")["metadata"]
        for key in ("observation_id", "effective_at", "read_at"):
            self.assertEqual(rejected[key], first[key])
        self.assertEqual(rejected["source_rejected"], 2)
        self.assertEqual(rejected["order_rejected"], 1)
        self.assertEqual(rejected["metadata_rejected"], 1)
        self.assertEqual(rejected["wire_rejected"], 4)
        self.assertEqual(accepted["observation_id"], first["observation_id"] + 1)
        self.assertEqual(accepted["effective_at"], 101.5)

    def test_cache_clears_at_heartbeat_loss_reset_and_clock_fault(self):
        for case in ("startup", "before_heartbeat", "unsupported_autopilot", "heartbeat_expired",
                     "new_heartbeat", "pre_reset_sample", "clock_fault"):
            for family in ("INFO", "STATUS", "TELEMETRY"):
                with self.subTest(case=case, family=family):
                    self.assertEqual(self.report(case, family)["groups"], [])
        first = self.group("first_group", "TELEMETRY")["metadata"]
        recovered = self.group("new_sample", "TELEMETRY")["metadata"]
        reset = self.group("after_reset", "INFO")["metadata"]
        clock = self.group("clock_recovered", "INFO")["metadata"]
        self.assertLess(first["session_epoch"], recovered["session_epoch"])
        self.assertLess(recovered["session_epoch"], reset["session_epoch"])
        self.assertLess(reset["session_epoch"], clock["session_epoch"])
        self.assertGreater(clock["effective_at"], 103.9)

    def test_invalid_config_batches_preserve_the_prior_settings(self):
        fresh = self.group("configured_fresh", "TELEMETRY")["metadata"]
        stale = self.group("configured_stale", "TELEMETRY")["metadata"]
        self.assertEqual((fresh["sample_timeout_s"], fresh["publish_interval_s"]), (.4, .05))
        self.assertTrue(fresh["fresh"])
        self.assertFalse(stale["fresh"])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--driver", required=True)
    parser.add_argument("--module", required=True)
    args = parser.parse_args()
    run = subprocess.run([args.driver, args.module], capture_output=True, text=True, timeout=10)
    if run.returncode:
        raise RuntimeError(f"ESC test driver exited {run.returncode}: {run.stderr}")
    EscStatusTests.reports = {}
    for line in run.stdout.splitlines():
        row = json.loads(line, parse_float=finite_float, parse_constant=reject_constant)
        key = (row["case"], row["key"])
        if key in EscStatusTests.reports:
            raise ValueError(f"duplicate ESC test publication {key}")
        EscStatusTests.reports[key] = row["value"]
    unittest.main(argv=[__file__], verbosity=2)
