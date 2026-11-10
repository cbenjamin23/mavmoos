#!/usr/bin/env python3
"""Exercise the real link_status DSO through bounded fake host callbacks."""

import argparse
from copy import deepcopy
import json
import subprocess
import unittest

from wire_fixture import CELLULAR, EXPECTED, KEYS, RADIO, audit_loopback, entries


def reject_constant(value):
    raise ValueError(f"nonfinite JSON number {value}")


def unique_object(pairs):
    value = dict(pairs)
    if len(value) != len(pairs):
        raise ValueError("duplicate JSON key")
    return value


def sample(phase, key, source=-1):
    return entries(CASES[phase][key], key)[source]


class LinkStatusTests(unittest.TestCase):
    def test_scope_and_cadence(self):
        for phase, posts in CASES.items():
            if phase == "interval_suppressed":
                self.assertEqual(posts, {})
                continue
            self.assertEqual(posts.keys(), set(KEYS))
            for key, record in posts.items():
                entries(record, key)

    def test_radio_bytes_and_cellular_extensions_are_independent(self):
        observations = set()
        for key, sources in EXPECTED.items():
            rows = entries(CASES["fresh"][key], key)
            self.assertEqual(rows.keys(), sources.keys())
            for source, (length, data) in sources.items():
                row, meta = rows[source]
                self.assertEqual((row["wire_version"], row["payload_length"], row["data"]), (2, length, data))
                self.assertTrue(meta["fresh"])
                observations.add(meta["observation_id"])
        self.assertEqual(len(observations), 3)

    def test_identity_order_and_legacy_radio_do_not_renew(self):
        original = sample("fresh", RADIO)[1]
        after = sample("identity_and_order", RADIO)[1]
        self.assertEqual((original["effective_at"], original["read_at"], original["age_basis"]),
                         (100.22, 100.26, "QUEUE_EMPTY_FLOOR"))
        self.assertAlmostEqual(original["age_s"], .12)
        self.assertTrue(original["age_is_upper_bound"])
        self.assertEqual(after["source_rejected"], original["source_rejected"] + 2)
        self.assertEqual(after["order_rejected"], original["order_rejected"] + 2)
        for key, source in ((RADIO, -1), (CELLULAR, 3)):
            original, original_meta = sample("fresh", key, source)
            retained, retained_meta = sample("identity_and_order", key, source)
            self.assertEqual(original["data"], retained["data"])
            for field in ("effective_at", "read_at", "observation_id"):
                self.assertEqual(original_meta[field], retained_meta[field])

    def test_malformed_frames_keep_previous_data_and_original_ages(self):
        self.assertEqual(CASES["malformed"][RADIO]["invalid_samples"], 3)
        self.assertEqual(CASES["malformed"][CELLULAR]["invalid_samples"], 2)
        for key, source in ((RADIO, -1), (CELLULAR, 3)):
            before, before_meta = sample("identity_and_order", key, source)
            after, after_meta = sample("malformed", key, source)
            self.assertEqual(after_meta["wire_rejected"], before_meta["wire_rejected"] + 5)
            self.assertEqual(after_meta["effective_at"], before_meta["effective_at"])
            self.assertEqual(after["data"], before["data"])

    def test_stale_clear_and_explicit_rebind(self):
        self.assertFalse(sample("independent_staleness", RADIO)[1]["fresh"])
        self.assertFalse(sample("independent_staleness", CELLULAR, 0)[1]["fresh"])
        cellular, metadata = sample("independent_staleness", CELLULAR, 3)
        self.assertTrue(metadata["fresh"])
        self.assertEqual(cellular["data"]["quality_pct_raw"], 191)
        self.assertIsNone(cellular["data"]["quality_pct"])
        self.assertEqual(cellular["data"]["failure_reason_raw"], 254)
        self.assertFalse(cellular["data"]["failure_reason_applicable"])
        for phase in ("absent", "prebind", "heartbeat_loss", "rebind_required", "clock_fault",
                      "reset_floor", "unknown_autopilot"):
            self.assertTrue(all(not CASES[phase][key]["present"] for key in KEYS), phase)
        radio, rebound = sample("px4_rebind", RADIO)
        self.assertEqual((rebound["system_id"], rebound["component_id"], rebound["autopilot"]), (1, 1, 12))
        self.assertGreater(rebound["session_epoch"], sample("fresh", RADIO)[1]["session_epoch"])
        self.assertTrue(all(radio["data"][f"{field}_device_units"] is None
                            for field in ("rssi", "remrssi", "noise", "remnoise")))
        self.assertEqual(radio["data"]["tx_buffer_free_pct"], 100)
        self.assertTrue(CASES["clock_fault"][RADIO]["clock_fault"])
        self.assertFalse(CASES["px4_rebind"][CELLULAR]["present"])

    def test_v1_v2_trimming_and_raw_field_availability(self):
        self.assertEqual(sample("v1", RADIO)[0]["wire_version"], 1)
        radio, _ = sample("partial_v2", RADIO)
        self.assertEqual((radio["wire_version"], radio["payload_length"]), (2, 1))
        self.assertEqual(radio["data"]["rssi_device_units"], 0)
        self.assertEqual(radio["data"]["tx_buffer_free_pct"], 0)
        partial, _ = sample("partial_v2", CELLULAR, 5)
        self.assertEqual((partial["wire_version"], partial["payload_length"]), (2, 12))
        self.assertEqual(partial["data"]["link_tx_rate_kib_s"], 7)
        self.assertIsNone(partial["data"]["link_rx_rate_kib_s"])
        self.assertIsNone(partial["data"]["band_frequency_mhz"])
        self.assertEqual(sample("field_availability", RADIO)[0]["data"]["tx_buffer_free_pct_raw"], 200)
        self.assertIsNone(sample("field_availability", RADIO)[0]["data"]["tx_buffer_free_pct"])
        binary_tower = sample("field_availability", CELLULAR, 3)[0]["data"]
        self.assertIsNone(binary_tower["cell_tower_id"])
        self.assertEqual(binary_tower["cell_tower_id_raw"][0], 1)

    def test_modem_capacity_preserves_existing_samples(self):
        cell = CASES["capacity"][CELLULAR]
        self.assertEqual(list(entries(cell, CELLULAR)), [0, 1, 2, 3, 4, 5, 6, 255])
        self.assertEqual(cell["capacity_drops"], 1)
        self.assertEqual(sample("capacity", CELLULAR, 255)[0]["data"]["quality_pct"], 100)
        self.assertTrue(CASES["capacity"][RADIO]["present"])
        self.assertFalse(CASES["capacity_reset"][RADIO]["present"])
        self.assertEqual(list(entries(CASES["capacity_reset"][CELLULAR], CELLULAR)), [254])

    def test_shared_audit_detects_renewed_or_fabricated_data(self):
        prebind, prefix, full = {}, {}, {}
        for key in KEYS:
            empty = CASES["absent"][key]
            fresh = deepcopy(CASES["fresh"][key])
            for _, meta in entries(fresh, key).values():
                meta.update(age_basis="QUEUE_EMPTY_FLOOR", age_is_upper_bound=True)
            stale = deepcopy(fresh)
            for _, meta in entries(stale, key).values():
                meta.update(age_s=1.2, fresh=False)
            rebound = deepcopy(fresh)
            for _, meta in entries(rebound, key).values():
                meta["session_epoch"] += 1
                meta["observation_id"] += 10
            prebind[key] = [empty]
            prefix[key] = [empty, fresh, stale, empty, empty]
            full[key] = [*prefix[key], rebound]
        self.assertEqual(audit_loopback(prebind, prefix, full, 3)[CELLULAR]["recovered_sources"], 2)

        changed_prefix = deepcopy(prefix)
        changed_full = deepcopy(full)
        for capture in (changed_prefix, changed_full):
            capture[RADIO][2]["effective_at"] += .01
        with self.assertRaisesRegex(ValueError, "renewed source"):
            audit_loopback(prebind, changed_prefix, changed_full, 3)
        changed_full = deepcopy(full)
        changed_full[CELLULAR][-1]["modems"][0]["data"]["link_rx_rate_kib_s"] = 0
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
