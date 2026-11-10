#!/usr/bin/env python3
"""Exercise the real clock_status DSO using fake host acquisition times."""

import argparse
from copy import deepcopy
import json
import subprocess
import unittest

from wire_fixture import EXPECTED, KEYS, REPLY, REQUEST, SYSTEM, audit_loopback, validate_envelope


def reject_constant(value):
    raise ValueError(f"nonfinite JSON number {value}")


def unique_object(pairs):
    value = dict(pairs)
    if len(value) != len(pairs):
        raise ValueError("duplicate JSON key")
    return value


class ClockStatusTests(unittest.TestCase):
    def test_scope_and_cadence(self):
        for phase, posts in CASES.items():
            if phase == "interval_suppressed":
                self.assertEqual(posts, {})
                continue
            self.assertEqual(posts.keys(), set(KEYS))
            for key, row in posts.items():
                validate_envelope(row, key)

    def test_unsigned_utc_and_signed_timesync_keep_exact_wire_values(self):
        observations = set()
        self.assertTrue(CASES["reply_first"][REPLY]["present"])
        self.assertFalse(CASES["reply_first"][REQUEST]["present"])
        for key, (length, expected) in EXPECTED.items():
            row = CASES["fresh"][key]
            self.assertEqual((row["wire_version"], row["payload_length"], row["data"]), (2, length, expected))
            self.assertTrue(row["fresh"])
            observations.add(row["observation_id"])
        self.assertEqual(len(observations), 3)

    def test_source_order_and_original_acquisition_are_independent(self):
        original = CASES["fresh"][SYSTEM]
        after = CASES["identity_and_order"][SYSTEM]
        self.assertEqual((original["effective_at"], original["read_at"], original["age_basis"]),
                         (100.22, 100.26, "QUEUE_EMPTY_FLOOR"))
        self.assertAlmostEqual(original["age_s"], .19)
        self.assertTrue(original["age_is_upper_bound"])
        self.assertEqual(after["source_rejected"], original["source_rejected"] + 2)
        self.assertEqual(after["order_rejected"], original["order_rejected"] + 2)
        for key in KEYS:
            for field in ("effective_at", "read_at", "observation_id", "data"):
                self.assertEqual(CASES["identity_and_order"][key][field], CASES["fresh"][key][field])

    def test_malformed_wire_frames_keep_previous_values(self):
        invalid = CASES["malformed"]
        self.assertEqual(invalid[SYSTEM]["invalid_family_frames"], 2)
        self.assertEqual(invalid[REQUEST]["invalid_family_frames"], 3)
        self.assertEqual(invalid[REPLY]["invalid_family_frames"], 3)
        for key in KEYS:
            previous = CASES["identity_and_order"][key]
            self.assertEqual(invalid[key]["wire_rejected"], previous["wire_rejected"] + 5)
            for field in ("effective_at", "read_at", "observation_id", "data"):
                self.assertEqual(invalid[key][field], previous[field])

    def test_stale_ages_zero_utc_and_session_rebinding(self):
        stale = CASES["independent_staleness"]
        self.assertFalse(stale[SYSTEM]["fresh"])
        self.assertFalse(stale[REPLY]["fresh"])
        self.assertTrue(stale[REQUEST]["fresh"])
        self.assertEqual(CASES["boot_without_utc"][SYSTEM]["data"], {
            "time_boot_ms": 7, "time_unix_usec_raw": "0", "utc_reported": False, "time_unix_usec": None,
        })
        for phase in ("absent", "prebind", "heartbeat_loss", "rebind_required", "clock_fault",
                      "reset_floor", "unknown_autopilot"):
            self.assertTrue(all(not CASES[phase][key]["present"] for key in KEYS), phase)
        rebound = CASES["px4_rebind"][REPLY]
        self.assertEqual((rebound["system_id"], rebound["component_id"], rebound["autopilot"]), (1, 1, 12))
        self.assertGreater(rebound["session_epoch"], CASES["fresh"][REPLY]["session_epoch"])
        self.assertTrue(CASES["clock_fault"][SYSTEM]["clock_fault"])

    def test_v1_v2_trimming_and_target_address_shape(self):
        old = CASES["v1"]
        self.assertEqual((old[SYSTEM]["wire_version"], old[SYSTEM]["payload_length"]), (1, 12))
        self.assertEqual((old[REQUEST]["wire_version"], old[REQUEST]["payload_length"]), (1, 16))
        for key in (REQUEST, REPLY):
            self.assertEqual(old[key]["data"]["target_scope"], "broadcast_or_unspecified")
            self.assertIsNone(old[key]["data"]["target_system_raw"])
            self.assertIsNone(old[key]["data"]["target_component_raw"])
        self.assertEqual(old[REPLY]["data"]["tc1_ns_raw"], "-1")
        self.assertEqual(old[REPLY]["data"]["packet_kind"], "reply")
        partial = CASES["partial_v2"]
        self.assertEqual(partial[SYSTEM]["payload_length"], 1)
        self.assertEqual(partial[SYSTEM]["data"]["time_boot_ms"], 0)
        self.assertEqual(partial[SYSTEM]["data"]["time_unix_usec"], "1")
        self.assertEqual(partial[REQUEST]["payload_length"], 1)
        self.assertEqual(partial[REQUEST]["data"]["ts1_ns_raw"], "0")
        self.assertEqual(partial[REPLY]["data"]["target_scope"], "component_without_system")
        self.assertEqual(partial[REPLY]["data"]["target_system_raw"], 0)
        self.assertEqual(partial[REPLY]["data"]["target_component_raw"], 191)

    def test_shared_audit_rejects_clock_renewal_or_numeric_coercion(self):
        prebind, prefix, full = {}, {}, {}
        for key in KEYS:
            empty = CASES["absent"][key]
            fresh = dict(CASES["fresh"][key], age_basis="QUEUE_EMPTY_FLOOR", age_is_upper_bound=True)
            stale = dict(fresh, age_s=1.2, fresh=False)
            rebound = deepcopy(fresh)
            rebound.update(session_epoch=fresh["session_epoch"] + 1,
                           observation_id=fresh["observation_id"] + 10)
            prebind[key] = [empty]
            prefix[key] = [empty, fresh, stale, empty, empty]
            full[key] = [*prefix[key], rebound]
        self.assertEqual(audit_loopback(prebind, prefix, full, 3)[REPLY]["recovered_observations"], 1)

        changed_prefix = deepcopy(prefix)
        changed_full = deepcopy(full)
        for capture in (changed_prefix, changed_full):
            capture[SYSTEM][2]["effective_at"] += .01
        with self.assertRaisesRegex(ValueError, "renewed the clock"):
            audit_loopback(prebind, changed_prefix, changed_full, 3)
        changed_full = deepcopy(full)
        changed_full[REQUEST][-1]["data"]["ts1_ns_raw"] = -(2**63)
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
