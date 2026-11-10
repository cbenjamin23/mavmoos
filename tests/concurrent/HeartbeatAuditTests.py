#!/usr/bin/env python3
"""Synthetic ALOG acceptance/rejection tests; no MOOS, simulator or MAVLink SDK."""

import copy
from pathlib import Path
import tempfile
import unittest

from heartbeat_audit import audit_primary_heartbeat_reset


class HeartbeatAuditTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.out = Path(self.directory.name)
        (self.out / "logs").mkdir()
        self.terminal = {"id": "concurrent-list", "action": "list",
                         "status": "CANCELLED", "reason": "connection_reset", "at": "102.1"}
        self.before = {"session": "owned-session", "present": "true", "identity": "EXPECTED",
                       "encoding_valid": "true", "rx_mono_s": "100", "read_mono_s": "100.02",
                       "age_basis": "QUEUE_EMPTY_FLOOR", "receipt_id": "1", "wire_seq": "7"}
        self.after = dict(self.before, rx_mono_s="103", read_mono_s="103.02", receipt_id="2", wire_seq="8")
        self.cancel = dict(self.terminal)
        self.reset = {"reason": "reset:vehicle_connection_lost", "permission": "false", "heartbeat_valid": "0"}
        self.helm = {"state": "LATCHED", "reason": "reset:vehicle_connection_lost"}

    @staticmethod
    def line(at, variable, fields, source="iMAVLink"):
        body = ",".join(f"{key}={value}" for key, value in fields.items())
        return f"{at} {variable} {source} {body}\n"

    def lines(self):
        return [self.line(1, "MAVMOOS_HEARTBEAT", self.before),
                self.line(3.1, "MAVMOOS_PARAM_RESULT", self.cancel),
                self.line(3.10002, "MAVMOOS_RUNTIME_STATUS", self.reset),
                self.line(3.10001, "MAVMOOS_HELM_STATUS", self.helm),
                self.line(4, "MAVMOOS_HEARTBEAT", self.after)]

    def audit(self, lines=None, name="fixture.alog"):
        (self.out / "logs" / name).write_text("".join(self.lines() if lines is None else lines))
        return audit_primary_heartbeat_reset(self.out, self.terminal)

    def reject(self, lines=None, message=None):
        with self.assertRaisesRegex(RuntimeError, message or "."):
            self.audit(lines)

    def test_prior_long_read_and_floor_gap(self):
        result = self.audit()
        self.assertEqual(result["classification"], "PRE_INGRESS_CONSERVATIVE_HEARTBEAT_CACHE_EXPIRED")
        self.assertGreater(result["effective_heartbeat_gap_s"], 2)
        self.assertGreater(result["read_heartbeat_gap_s"], 2)
        self.assertFalse(result["complete_table_claimed"])

    def test_actual_same_poll_refresh_with_shorter_floor_and_read_gaps(self):
        self.terminal["at"] = "3392297.556088375"
        self.cancel["at"] = self.terminal["at"]
        self.before.update(rx_mono_s="3392295.5400226", read_mono_s="3392295.5652372", receipt_id="1464")
        self.after.update(rx_mono_s="3392297.5320889", read_mono_s="3392297.5562252", receipt_id="1498")
        self.reset["heartbeat_valid"] = "1"
        result = self.audit()
        self.assertGreater(result["age_since_last_effective_at_reset_s"], 2)
        self.assertLess(result["effective_heartbeat_gap_s"], 2)
        self.assertLess(result["read_heartbeat_gap_s"], 2)
        self.assertLess(result["next_effective_floor_minus_reset_s"], 0)
        self.assertGreater(result["next_actual_read_minus_reset_s"], 0)
        self.assertEqual(result["read_boundary_ordering"], "STRICT_READ_BRACKET")

    def test_equal_next_read_uses_runtime_input_boundary_ordering(self):
        self.after.update(rx_mono_s="102.08", read_mono_s=self.terminal["at"])
        self.reset["heartbeat_valid"] = "1"
        result = self.audit()
        self.assertEqual(result["next_actual_read_minus_reset_s"], 0)
        self.assertEqual(result["read_boundary_ordering"], "RUNTIME_INPUT_BOUNDARY_EQUALITY")

    def test_no_epsilon_turns_earlier_next_read_into_equal_boundary(self):
        self.after.update(rx_mono_s="102.08", read_mono_s="102.0999999")
        self.reject(message="missing primary heartbeat actual-read bracket")

    def test_unexpired_old_cache_and_exact_ttl_rejected(self):
        for effective in ("100.1000001", "100.1", "101"):
            with self.subTest(effective=effective):
                self.before.update(rx_mono_s=effective, read_mono_s="101.02")
                self.reject(message="not expired")

    def test_next_actual_read_strictly_before_terminal_rejected(self):
        self.after.update(rx_mono_s="102", read_mono_s="102.05")
        self.reject(message="missing primary heartbeat actual-read bracket")

    def test_old_actual_read_equal_terminal_is_not_an_old_bracket(self):
        self.before["read_mono_s"] = self.terminal["at"]
        self.reject(message="missing primary heartbeat actual-read bracket")

    def test_intervening_read_is_used_instead_of_an_older_expired_sample(self):
        middle = dict(self.before, rx_mono_s="101", read_mono_s="101.02", receipt_id="3")
        lines = self.lines() + [self.line(2, "MAVMOOS_HEARTBEAT", middle)]
        self.reject(lines, "not expired")

    def test_duplicate_cached_publications_do_not_create_ambiguity(self):
        lines = self.lines() + [self.line(2, "MAVMOOS_HEARTBEAT", self.before),
                                self.line(4.2, "MAVMOOS_HEARTBEAT", self.after),
                                self.line(3.11, "MAVMOOS_RUNTIME_STATUS", self.reset),
                                self.line(3.12, "MAVMOOS_HELM_STATUS", self.helm)]
        self.assertFalse(self.audit(lines)["complete_table_claimed"])

    def test_conflicting_receipts_at_same_bracket_read_rejected(self):
        for which in ("before", "after"):
            with self.subTest(which=which):
                extra = dict(getattr(self, which), receipt_id="different")
                self.reject(self.lines() + [self.line(2, "MAVMOOS_HEARTBEAT", extra)], "ambiguous primary")

    def test_multiple_matching_log_contexts_rejected(self):
        (self.out / "logs" / "other.alog").write_text("".join(self.lines()))
        self.reject(message="ambiguous matching primary log contexts")

    def test_missing_primary_observations_rejected(self):
        lines = [line for line in self.lines() if "MAVMOOS_HEARTBEAT" not in line]
        self.reject(lines, "missing primary heartbeat")

    def test_observer_or_other_app_cannot_supply_primary_observations(self):
        self.reject([line.replace(" iMAVLink ", " observer ") if "MAVMOOS_HEARTBEAT" in line
                     else line for line in self.lines()], "missing primary heartbeat")

    def test_primary_aux_source_suffix_supported(self):
        lines = [line.replace(" iMAVLink ", " iMAVLink:aux ") for line in self.lines()]
        self.assertEqual(self.audit(lines)["last_accepted_heartbeat"]["receipt_id"], "1")

    def test_missing_or_mismatched_session_rejected(self):
        for session in ("", "another-process"):
            with self.subTest(session=session):
                self.after["session"] = session
                self.reject(message="nonempty session")

    def test_unexpected_identity_and_invalid_encoding_rejected(self):
        for target in ("before", "after"):
            for field, value in (("identity", "OTHER"), ("identity", ""),
                                 ("encoding_valid", "false"), ("encoding_valid", "")):
                with self.subTest(target=target, field=field, value=value):
                    original = copy.deepcopy(getattr(self, target))
                    getattr(self, target)[field] = value
                    self.reject(message="identity/encoding")
                    setattr(self, target, original)

    def test_wrong_missing_or_nonterminal_cancellation_rejected(self):
        for key, value in (("id", "other"), ("action", "read"), ("status", "COMPLETE"),
                           ("reason", "timeout"), ("at", "102.100001")):
            with self.subTest(key=key):
                original = self.cancel.copy()
                self.cancel[key] = value
                self.reject(message="missing exact cancellation")
                self.cancel = original
        self.reject([line for line in self.lines() if "MAVMOOS_PARAM_RESULT" not in line])

    def test_wrong_terminal_argument_rejected(self):
        for key, value in (("id", "other"), ("action", "read"),
                           ("status", "COMPLETE"), ("reason", "timeout")):
            with self.subTest(key=key):
                original = self.terminal.copy()
                self.terminal[key] = value
                self.reject(message="terminal is not")
                self.terminal = original

    def test_wrong_or_missing_reset_cause_permission_and_helm_latch_rejected(self):
        for target, key, value in (("reset", "reason", "reset:other"), ("reset", "permission", "true"),
                                   ("helm", "state", "PAUSED"), ("helm", "reason", "reset:other")):
            with self.subTest(target=target, key=key):
                original = getattr(self, target).copy()
                getattr(self, target)[key] = value
                self.reject(message="missing exact cancellation")
                setattr(self, target, original)
        for variable in ("MAVMOOS_RUNTIME_STATUS", "MAVMOOS_HELM_STATUS"):
            with self.subTest(missing=variable):
                self.reject([line for line in self.lines() if variable not in line])

    def test_three_witnesses_must_share_one_quarter_second_window(self):
        lines = self.lines()
        lines[2] = self.line(3.34, "MAVMOOS_RUNTIME_STATUS", self.reset)
        lines[3] = self.line(2.86, "MAVMOOS_HELM_STATUS", self.helm)
        self.reject(lines)
        lines[3] = self.line(3.35, "MAVMOOS_HELM_STATUS", self.helm)
        self.assertFalse(self.audit(lines)["complete_table_claimed"])
        lines[3] = self.line(3.350001, "MAVMOOS_HELM_STATUS", self.helm)
        self.reject(lines)

    def test_publication_witnesses_cannot_be_joined_across_logs(self):
        lines = self.lines()
        (self.out / "logs" / "other.alog").write_text(lines[3])
        self.reject([line for line in lines if "MAVMOOS_HELM_STATUS" not in line])

    def test_invalid_heartbeat_stamps_rejected(self):
        for field in ("rx_mono_s", "read_mono_s"):
            for value in ("nan", "inf", "-1", "garbage", None):
                with self.subTest(field=field, value=value):
                    original = self.before.copy()
                    if value is None:
                        self.before.pop(field)
                    else:
                        self.before[field] = value
                    self.reject(message="invalid primary heartbeat")
                    self.before = original

    def test_effective_stamp_after_actual_read_rejected(self):
        self.before["rx_mono_s"] = "100.03"
        self.reject(message="effective stamp exceeds actual read")

    def test_invalid_terminal_and_cancellation_stamps_rejected(self):
        for target in ("terminal", "cancel"):
            for value in ("nan", "inf", "-1", "garbage", None):
                with self.subTest(target=target, value=value):
                    original = getattr(self, target).copy()
                    if value is None:
                        getattr(self, target).pop("at")
                    else:
                        getattr(self, target)["at"] = value
                    self.reject(message="invalid .*stamp")
                    setattr(self, target, original)

    def test_invalid_alog_stamp_and_duplicate_fields_rejected(self):
        lines = self.lines()
        self.reject([lines[0].replace("1 ", "nan ", 1)] + lines[1:], "invalid primary diagnostic")
        self.reject([lines[0].rstrip() + ",rx_mono_s=99\n"] + lines[1:], "duplicate diagnostic")

    def test_unrelated_nonprimary_and_absent_observations_ignored(self):
        extra = ["% header\n", "not-an-alog-line\n", "MAVMOOS_HEARTBEAT 00:01 iMAVLink status summary\n",
                 self.line(0.1, "MAVMOOS_HEARTBEAT", {"present": "false"}),
                 self.line("nan", "MAVMOOS_HEARTBEAT", {"present": "true"}, "other")]
        self.assertFalse(self.audit(extra + self.lines())["complete_table_claimed"])

    def test_missing_log_directory_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "missing exact cancellation"):
            audit_primary_heartbeat_reset(self.out / "missing", self.terminal)


if __name__ == "__main__":
    unittest.main(verbosity=2)
