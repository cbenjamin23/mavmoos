#!/usr/bin/env python3
"""Check exact JSON and effect isolation from the loadable debug-values provider."""

import argparse
from copy import deepcopy
import json
import struct
import subprocess
import unittest

from wire_fixture import (
    ARRAY, DEBUG, EXPECTED, FLOAT, FULL, INT, KEYS, MESSAGES, SHARED, VECTOR, audit_loopback, entries, matches,
)


def no_constant(value):
    raise ValueError(f"nonfinite JSON number: {value}")


def unique_keys(pairs):
    result = dict(pairs)
    if len(result) != len(pairs):
        raise ValueError("duplicate JSON key")
    return result


def entry(phase, key, identity):
    return entries(CASES[phase][key], key)[identity]


class DebugValuesTests(unittest.TestCase):
    def test_provider_scope_and_cadence(self):
        for phase, records in CASES.items():
            if phase == "cadence":
                self.assertEqual(records, {})
                continue
            self.assertEqual(records.keys(), set(KEYS))
            for key, record in records.items():
                entries(record, key)

    def test_typed_values_and_bounded_names(self):
        for key, expected in EXPECTED.items():
            first = entries(CASES["fresh"][key], key)
            self.assertEqual(first.keys(), expected.keys())
            for identity, target in expected.items():
                self.assertTrue(matches(first[identity], target), (key, identity, first[identity]))
                self.assertTrue(first[identity]["metadata"]["fresh"])
        self.assertNotEqual(entry("fresh", INT, SHARED)["metadata"]["observation_id"],
                            entry("fresh", FLOAT, SHARED)["metadata"]["observation_id"])
        self.assertEqual(entry("fresh", VECTOR, FULL)["name"], "VECTOR0123")
        self.assertIsNone(entry("fresh", FLOAT, "78ff")["name"])
        self.assertEqual(entry("fresh", INT, "")["name_state"], "empty")

    def test_rejected_inputs_do_not_change_the_original_sample(self):
        initial = entry("fresh", DEBUG, 7)["metadata"]
        self.assertEqual((initial["effective_at"], initial["read_at"], initial["age_basis"]),
                         (100.22, 100.26, "QUEUE_EMPTY_FLOOR"))
        self.assertTrue(initial["age_is_upper_bound"])
        self.assertAlmostEqual(initial["age_s"], .1)
        after = entry("rejected_source_and_order", DEBUG, 7)["metadata"]
        self.assertEqual(after["source_rejected"], initial["source_rejected"] + 2)
        self.assertEqual(after["order_rejected"], initial["order_rejected"] + 1)
        self.assertEqual([after[field] for field in ("effective_at", "read_at", "observation_id")],
                         [initial[field] for field in ("effective_at", "read_at", "observation_id")])
        for key, count in ((DEBUG, 2), (VECTOR, 1), (ARRAY, 1), (FLOAT, 0), (INT, 1)):
            self.assertEqual(CASES["malformed"][key]["invalid_samples"], count)
            before = entries(CASES["rejected_source_and_order"][key], key)
            for identity, sample in entries(CASES["malformed"][key], key).items():
                old = before[identity]
                self.assertEqual(sample["data"], old["data"])
                self.assertEqual(sample["metadata"]["observation_id"], old["metadata"]["observation_id"])
                self.assertEqual(sample["metadata"]["wire_rejected"], old["metadata"]["wire_rejected"] + 5)

    def test_stale_samples_clear_and_require_rebinding(self):
        old = entry("independent_ages", DEBUG, 7)["metadata"]
        self.assertFalse(old["fresh"])
        self.assertEqual(old["effective_at"], 100.22)
        self.assertAlmostEqual(old["age_s"], 1.18)
        replacement = entry("independent_ages", DEBUG, 0)
        self.assertTrue(replacement["metadata"]["fresh"])
        self.assertEqual(replacement["data"]["value_state"], "wire_nan")
        for phase in ("initial", "prebind", "heartbeat_loss", "unbound_after_loss", "clock_fault",
                      "reset_floor", "unsupported_autopilot", "wrong_heartbeat_component"):
            self.assertTrue(all(not row["present"] for row in CASES[phase].values()), phase)
        rebound = entry("px4_rebind", VECTOR, FULL)["metadata"]
        self.assertEqual((rebound["system_id"], rebound["component_id"], rebound["autopilot"]), (1, 1, 12))
        self.assertGreater(rebound["session_epoch"], old["session_epoch"])

    def test_mavlink1_and_name_padding(self):
        for key, full_length in ((DEBUG, 9), (VECTOR, 30), (FLOAT, 18), (INT, 18)):
            for identity, row in entries(CASES["v1"][key], key).items():
                self.assertTrue(matches(row, (full_length, EXPECTED[key][identity][1]), version=1))
        self.assertFalse(CASES["v1"][ARRAY]["present"])
        updated = entries(CASES["name_padding_and_boot_wrap"][FLOAT], FLOAT)
        self.assertEqual(updated.keys(), EXPECTED[FLOAT].keys())
        self.assertEqual((updated[SHARED]["wire_version"], updated[SHARED]["payload_length"]), (2, 17))
        self.assertEqual(updated[SHARED]["data"]["time_boot_ms_raw"], 0)
        self.assertEqual(updated[SHARED]["data"]["value"], 1.5)

    def test_array_identity_independent_age_and_exact_wire_padding(self):
        first = entry("fresh", ARRAY, 17)
        second = entry("fresh", ARRAY, 65535)
        self.assertEqual(first["name"], second["name"])
        self.assertEqual(first["data"]["element_wire_bytes"], [4] * 58)
        self.assertEqual(second["data"]["element_wire_bytes"], [1] + [0] * 57)
        self.assertEqual(struct.pack("<f", second["data"]["values"][0]), bytes((1, 0, 0, 0)))
        self.assertEqual(second["data"]["value_states"][1:], ["zero_padded"] * 57)
        renamed = entry("independent_ages", ARRAY, 17)
        old = entry("independent_ages", ARRAY, 65535)
        self.assertEqual(renamed["name"], "renamed")
        self.assertEqual(renamed["payload_length"], 17)
        self.assertEqual(renamed["data"]["extension_payload_bytes"], 0)
        self.assertEqual(renamed["data"]["values"], [0] * 58)
        self.assertEqual(renamed["data"]["value_states"], ["zero_padded"] * 58)
        self.assertTrue(renamed["metadata"]["fresh"])
        self.assertFalse(old["metadata"]["fresh"])
        self.assertEqual(old["metadata"]["effective_at"], second["metadata"]["effective_at"])
        self.assertEqual(old["metadata"]["observation_id"], second["metadata"]["observation_id"])

    def test_capacity_per_family_and_reset(self):
        for key in KEYS:
            self.assertEqual(CASES["capacity"][key]["capacity_drops"], 1)
            actual = entries(CASES["capacity"][key], key)
            expected = list(range(1, 9)) if key in (DEBUG, ARRAY) else [f"key{i}".encode().hex() for i in range(1, 9)]
            self.assertEqual(list(actual), expected)
            self.assertEqual(CASES["capacity_reset"][key]["capacity_drops"], 1)
        self.assertEqual(entry("capacity", DEBUG, 8)["data"]["value"], 16)
        self.assertEqual(entry("capacity", FLOAT, b"key8".hex())["data"]["value"], 16)
        self.assertEqual(list(entries(CASES["capacity_reset"][DEBUG], DEBUG)), [9])
        self.assertEqual(list(entries(CASES["capacity_reset"][ARRAY], ARRAY)), [9])
        self.assertEqual(list(entries(CASES["capacity_reset"][INT], INT)), [b"key9".hex()])
        self.assertFalse(CASES["capacity_reset"][FLOAT]["present"])

    def test_generated_fixture_header_identity(self):
        raw = subprocess.check_output([DRIVER, "--frames"], text=True, timeout=4)
        decoded = [bytes.fromhex(line) for line in raw.splitlines()]
        expected = [(MESSAGES[key][1], length) for key in KEYS for length, _ in EXPECTED[key].values()]
        self.assertEqual(sorted((int.from_bytes(frame[7:10], "little"), frame[1]) for frame in decoded),
                         sorted(expected))
        self.assertTrue(all(frame[0] == 0xfd and frame[5:7] == b"\x01\x01" for frame in decoded))

    def test_loopback_audit_rejects_changed_original_age_and_invalid_projection(self):
        prebind, prefix, full = {}, {}, {}
        for key in KEYS:
            empty = CASES["prebind"][key]
            fresh = deepcopy(CASES["fresh"][key])
            stale = deepcopy(fresh)
            rebound = deepcopy(fresh)
            for row in stale["samples"]:
                row["metadata"].update(age_s=1.2, fresh=False)
            for row in rebound["samples"]:
                row["metadata"]["session_epoch"] += 1
                row["metadata"]["observation_id"] += 10
            prebind[key] = [empty]
            prefix[key] = [empty, fresh, stale, empty, empty]
            full[key] = [*prefix[key], rebound]
        self.assertEqual(audit_loopback(prebind, prefix, full, 3)[DEBUG]["recovered"], [0, 7])
        changed_prefix = deepcopy(prefix)
        changed_full = deepcopy(full)
        for report in (changed_prefix, changed_full):
            report[DEBUG][2]["samples"][0]["metadata"]["effective_at"] += .01
        with self.assertRaisesRegex(ValueError, "renewed"):
            audit_loopback(prebind, changed_prefix, changed_full, 3)
        wrong_value = deepcopy(full)
        wrong_value[FLOAT][-1]["samples"][0]["data"]["value"] = 0.0
        with self.assertRaisesRegex(ValueError, "wire values differ"):
            audit_loopback(prebind, prefix, wrong_value, 3)
        wrong_array = deepcopy(full)
        wrong_array[ARRAY][-1]["samples"][1]["data"]["element_wire_bytes"][0] = 4
        with self.assertRaisesRegex(ValueError, "wire values differ"):
            audit_loopback(prebind, prefix, wrong_array, 3)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--driver", required=True)
    parser.add_argument("--module", required=True)
    args = parser.parse_args()
    DRIVER = args.driver
    output = subprocess.check_output([args.driver, args.module], text=True, timeout=10)
    rows = [json.loads(line, parse_constant=no_constant, object_pairs_hook=unique_keys)
            for line in output.splitlines()]
    CASES = {row["phase"]: row["posts"] for row in rows}
    if len(CASES) != len(rows):
        raise ValueError("duplicate fake fixture phase")
    unittest.main(argv=[__file__], verbosity=2)
