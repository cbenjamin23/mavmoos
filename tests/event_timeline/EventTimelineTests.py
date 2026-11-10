#!/usr/bin/env python3
"""Check the loadable passive event provider against a fake bound host."""

import argparse
from copy import deepcopy
import json
import subprocess
import unittest

from wire_fixture import ERROR, KEYS, SEQUENCE, TIMELINE, audit_loopback, fixture_matches, samples


def no_constant(value):
    raise ValueError(f"nonfinite JSON number: {value}")


def unique_keys(pairs):
    result = dict(pairs)
    if len(result) != len(pairs):
        raise ValueError("duplicate JSON key")
    return result


def timeline(phase):
    return CASES[phase][TIMELINE]


def sequences(phase):
    return [item["data"]["sequence"] for item in timeline(phase)["samples"]]


class EventTimelineTests(unittest.TestCase):
    def test_publication_scope_and_cadence(self):
        for phase, posts in CASES.items():
            if phase == "cadence":
                self.assertEqual(posts, {})
                continue
            self.assertEqual(posts.keys(), set(KEYS))
            for key, record in posts.items():
                samples(record, key)

    def test_wrap_gap_recipient_and_exact_raw_values(self):
        first, wrapped = timeline("before_wrap"), timeline("after_wrap")
        self.assertEqual(sequences("before_wrap"), [65531, 65534])
        self.assertEqual(sequences("after_wrap"), [65531, 65534, 0, 3])
        self.assertEqual((first["sequence_anchor"], first["latest_observed_sequence"], first["advertised_unseen"]),
                         (65530, 65534, 1))
        self.assertEqual((wrapped["holes_observed"], wrapped["sequence_wraps"], wrapped["other_destinations"]),
                         (3, 1, 2))
        self.assertEqual([row["data"]["missing_before"] for row in wrapped["samples"]], [0, 1, 1, 1])
        self.assertEqual([row["data"]["sequence_wrap"] for row in wrapped["samples"]], [False, False, True, False])
        for key in KEYS:
            self.assertTrue(all(fixture_matches(key, row, data) for _, row, data in samples(CASES["after_wrap"][key], key)))
        self.assertEqual(wrapped["samples"][-1]["data"]["internal_log_level"], {"raw": 12, "name": None})

    def test_replay_unknown_error_and_invalid_inputs_preserve_event_records(self):
        old, after = timeline("after_wrap"), timeline("rejected_and_status")
        self.assertEqual(sequences("after_wrap"), sequences("rejected_and_status"))
        self.assertEqual((after["duplicates"], after["older_or_replayed"], after["ambiguous_sequence"]), (1, 1, 1))
        for initial, later in zip(old["samples"], after["samples"]):
            self.assertEqual(initial["data"], later["data"])
            a, b = initial["metadata"], later["metadata"]
            self.assertEqual((a["observation_id"], a["effective_at"], a["read_at"]),
                             (b["observation_id"], b["effective_at"], b["read_at"]))
            self.assertEqual((b["source_rejected"], b["order_rejected"], b["wire_rejected"]),
                             (a["source_rejected"] + 2, a["order_rejected"] + 1, a["wire_rejected"] + 2))
        error = CASES["rejected_and_status"][ERROR]["data"]
        self.assertEqual((error["reason_raw"], error["reason"], error["sequence_oldest_available"]), (42, None, 30000))
        self.assertEqual(error["request_correlation"], "none")
        self.assertEqual((after["latest_observed_sequence"], after["advertised_sequence"], after["advertised_unseen"]),
                         (3, 7, 4))
        self.assertEqual(after["holes_observed"], old["holes_observed"])

    def test_explicit_reset_clears_old_epoch_and_fences_old_mail(self):
        reset, later = timeline("reset"), timeline("same_reset_and_old_mail")
        self.assertEqual((reset["sequence_epoch"], reset["sequence_resets"], reset["sequence_anchor"]), (2, 1, 9))
        self.assertEqual(reset["samples"], [])
        self.assertFalse(CASES["reset"][ERROR]["present"])
        self.assertEqual(CASES["reset"][SEQUENCE]["data"], {"sequence": 9, "flags_raw": 129, "unknown_flags": 128,
                                                          "reset_announced": True, "reset_applied": True})
        self.assertEqual((later["sequence_epoch"], later["sequence_resets"], later["sequence_order_rejected"]), (2, 1, 2))
        self.assertEqual(sequences("same_reset_and_old_mail"), [10])
        self.assertFalse(CASES["same_reset_and_old_mail"][ERROR]["present"])
        self.assertFalse(CASES["same_reset_and_old_mail"][SEQUENCE]["data"]["reset_applied"])
        self.assertEqual((later["advertised_relation"], later["advertised_unseen"]), ("behind", None))

    def test_independent_ages_and_link_rebinding(self):
        rows = timeline("independent_ages")["samples"]
        self.assertEqual(sequences("independent_ages"), [10, 11])
        self.assertEqual([row["metadata"]["fresh"] for row in rows], [False, True])
        self.assertEqual(rows[0]["metadata"]["effective_at"], 100.74)
        self.assertTrue(CASES["independent_ages"][SEQUENCE]["fresh"])
        for phase in ("initial", "prebind", "heartbeat_loss", "unbound_after_loss", "clock_fault", "explicit_source_reset"):
            self.assertTrue(all(not record["present"] for record in CASES[phase].values()), phase)
        rebound = timeline("px4_rebind")["samples"][0]["metadata"]
        self.assertEqual((rebound["system_id"], rebound["component_id"], rebound["autopilot"]), (1, 1, 12))
        self.assertGreater(rebound["session_epoch"], rows[0]["metadata"]["session_epoch"])
        self.assertEqual(timeline("px4_rebind")["sequence_epoch"], 1)

    def test_capacity_and_mavlink2_trailing_zero_padding(self):
        sample, = timeline("trimmed")["samples"]
        self.assertEqual((sample["payload_length"], sample["data"]["sequence"]), (1, 0))
        self.assertEqual(sample["data"]["argument_wire_bytes"], 0)
        self.assertEqual(sample["data"]["arguments_raw_hex"], "00" * 40)
        self.assertEqual(CASES["trimmed"][SEQUENCE]["payload_length"], 1)
        self.assertEqual(sequences("capacity"), list(range(7, 23)))
        self.assertEqual(timeline("capacity")["evicted"], 7)

    def test_advertisement_does_not_preempt_a_later_actual_event(self):
        ahead, actual = timeline("advertised_ahead"), timeline("advertised_caught_up")
        self.assertEqual((ahead["latest_observed_sequence"], ahead["advertised_unseen"]), (0, 1))
        self.assertEqual((actual["latest_observed_sequence"], actual["advertised_unseen"]), (1, 0))
        self.assertEqual(sequences("advertised_caught_up"), [0, 1])
        self.assertEqual(actual["holes_observed"], 0)
        self.assertEqual(CASES["advertised_ahead"][SEQUENCE]["effective_at"],
                         CASES["advertised_caught_up"][SEQUENCE]["effective_at"])

    def test_generated_wire_headers(self):
        raw = subprocess.check_output([DRIVER, "--frames"], text=True, timeout=4)
        frames = [bytes.fromhex(line) for line in raw.splitlines()]
        self.assertEqual([int.from_bytes(row[7:10], "little") for row in frames], [410, 410, 410, 411, 413] * 2)
        self.assertEqual([row[1] for row in frames], [53, 16, 16, 3, 6] * 2)
        self.assertTrue(all(row[0] == 0xfd and row[5:7] == b"\x01\x01" for row in frames))

    def test_app_audit_rejects_changed_original_time_or_arguments(self):
        initial, prefix, full = {}, {}, {}
        for key in KEYS:
            empty = CASES["prebind"][key]
            fresh = deepcopy(CASES["after_wrap"][key])
            stale = deepcopy(fresh)
            rebound = deepcopy(fresh)
            for meta, _, _ in samples(stale, key):
                meta.update(age_s=1, fresh=False)
            for meta, _, _ in samples(rebound, key):
                meta["session_epoch"] += 1
                meta["observation_id"] += 20
            initial[key] = [empty]
            prefix[key] = [empty, fresh, stale, empty]
            full[key] = [*prefix[key], rebound]
        result = audit_loopback(initial, prefix, full, 3)
        self.assertEqual(result[TIMELINE]["rebound"], 4)
        bad_prefix, bad_full = deepcopy(prefix), deepcopy(full)
        for rows in (bad_prefix, bad_full):
            rows[ERROR][2]["effective_at"] += .001
        with self.assertRaisesRegex(ValueError, "renewed"):
            audit_loopback(initial, bad_prefix, bad_full, 3)
        changed = deepcopy(full)
        changed[TIMELINE][-1]["samples"][-1]["data"]["arguments_raw_hex"] = "00" * 40
        with self.assertRaisesRegex(ValueError, "differ"):
            audit_loopback(initial, prefix, changed, 3)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--driver", required=True)
    parser.add_argument("--module", required=True)
    args = parser.parse_args()
    DRIVER = args.driver
    output = subprocess.check_output([DRIVER, args.module], text=True, timeout=10)
    rows = [json.loads(line, parse_constant=no_constant, object_pairs_hook=unique_keys)
            for line in output.splitlines()]
    CASES = {row["phase"]: row["posts"] for row in rows}
    if len(CASES) != len(rows):
        raise ValueError("duplicate event fixture phase")
    unittest.main(argv=[__file__], verbosity=2)
