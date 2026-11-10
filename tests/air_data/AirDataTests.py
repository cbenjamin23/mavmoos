#!/usr/bin/env python3
"""Check air_data's actual loaded-module publications using fake Services."""

import argparse
import json
import subprocess
import unittest
from wire_fixture import AIRSPEED, AIRSPEED_DATA, HIGH_LATENCY, HIGH_LATENCY_DATA


HUD = "MAVMOOS_VFR_HUD"
WIND = "MAVMOOS_WIND"
COV = "MAVMOOS_WIND_COV"
PRESSURE = "MAVMOOS_SCALED_PRESSURE"
PRESSURE2 = "MAVMOOS_SCALED_PRESSURE2"
PRESSURE3 = "MAVMOOS_SCALED_PRESSURE3"
FAMILIES = {
    HUD: ("VFR_HUD", 74), WIND: ("WIND", 168), COV: ("WIND_COV", 231),
    PRESSURE: ("SCALED_PRESSURE", 29), PRESSURE2: ("SCALED_PRESSURE2", 137),
    PRESSURE3: ("SCALED_PRESSURE3", 143),
    HIGH_LATENCY: ("HIGH_LATENCY2", 235),
}


def reject_constant(value):
    raise ValueError(f"non-JSON number {value}")


def unique_object(pairs):
    values = dict(pairs)
    if len(values) != len(pairs):
        raise ValueError("duplicate JSON key")
    return values


def sensors(case):
    return {sample["sensor_id"]: sample for sample in case[AIRSPEED]["samples"]}


class AirDataTests(unittest.TestCase):
    def test_every_record_keeps_family_and_observation_scope(self):
        for name, posts in CASES.items():
            if name == "interval_suppressed":
                self.assertEqual(posts, {})
                continue
            self.assertEqual(posts.keys(), FAMILIES.keys() | {AIRSPEED}, name)
            for key in FAMILIES:
                record = posts[key]
                self.assertEqual((record["family"], record["message_id"]), FAMILIES[key])
                self.assertEqual(record["schema"], 1)
                self.assertIs(record["read_only"], True)
                self.assertIs(record["authoritative_for_control"], False)
                self.assertEqual(record["source_age"], "unknown")
                self.assertEqual(record["source_quality"], "unassessed")
                if not record["present"]:
                    self.assertIsNone(record["data"])
                    self.assertIsNone(record["effective_at"])
                    self.assertIsNone(record["read_at"])
                    self.assertIs(record["fresh"], False)
            summary = posts[AIRSPEED]
            self.assertEqual((summary["family"], summary["message_id"], summary["schema"]), ("AIRSPEED", 295, 1))
            self.assertIs(summary["read_only"], True)
            self.assertIs(summary["authoritative_for_control"], False)
            self.assertIs(summary["atomic"], False)
            self.assertEqual(summary["sensor_limit"], 2)
            self.assertEqual(summary["present"], bool(summary["samples"]))
            self.assertEqual(len(summary["samples"]), len(sensors(posts)))

    def test_exact_wire_semantics_and_unknowns(self):
        fresh = CASES["fresh"]
        self.assertTrue(all(fresh[key]["fresh"] for key in (HUD, WIND, COV)))
        self.assertEqual(fresh[HUD]["data"], {
            "airspeed_reported_m_s": 5.5,
            "airspeed_basis": "fc_reported_unspecified",
            "groundspeed_m_s": 7.25,
            "heading_deg": 278,
            "throttle_pct": 42,
            "alt_msl_m": 132.5,
            "climb_m_s": -1.25,
        })
        self.assertEqual(fresh[WIND]["data"], {
            "direction_from_deg": -135.5,
            "speed_reported_m_s": 8.25,
            "speed_z_reported_m_s": -2.5,
        })
        self.assertEqual(fresh[COV]["data"], {
            "frame": "NED",
            "time_usec_raw": str(2**64 - 1),
            "north_m_s": 3.25,
            "east_m_s": None,
            "down_m_s": -1.5,
            "horizontal_variability_std_m_s": 1.5,
            "vertical_variability_std_m_s": None,
            "alt_msl_m": 140.5,
            "horizontal_accuracy_std_m_s": None,
            "vertical_accuracy_std_m_s": .75,
        })
        self.assertEqual(
            (fresh[HUD]["system_id"], fresh[HUD]["component_id"], fresh[HUD]["autopilot"]),
            (1, 1, 3),
        )

    def test_age_source_order_and_session_boundaries(self):
        for phase in ("absent", "no_heartbeat", "heartbeat_expired", "reset", "unsupported_autopilot"):
            self.assertTrue(all(not record["present"] for record in CASES[phase].values()), phase)
        fresh = CASES["fresh"]
        self.assertEqual(fresh[HUD]["effective_at"], 100.4)
        self.assertEqual(fresh[HUD]["read_at"], 100.5)
        self.assertEqual(fresh[HUD]["age_basis"], "QUEUE_EMPTY_FLOOR")
        self.assertTrue(fresh[HUD]["age_is_upper_bound"])
        self.assertAlmostEqual(fresh[HUD]["age_s"], .7)
        self.assertEqual(fresh[WIND]["age_basis"], "READ_ACQUISITION")
        self.assertFalse(fresh[WIND]["age_is_upper_bound"])

        wrong_source = CASES["wrong_source"][HUD]
        self.assertEqual(wrong_source["source_rejected"], fresh[HUD]["source_rejected"] + 1)
        old = CASES["independent"]
        self.assertEqual(old[HUD]["order_rejected"], fresh[HUD]["order_rejected"] + 1)
        self.assertEqual(old[HUD]["observation_id"], fresh[HUD]["observation_id"])
        self.assertEqual(old[HUD]["effective_at"], fresh[HUD]["effective_at"])
        self.assertEqual(old[HUD]["read_at"], fresh[HUD]["read_at"])
        self.assertAlmostEqual(old[HUD]["age_s"], 1.2)
        self.assertTrue(old[HUD]["present"])
        self.assertFalse(old[HUD]["fresh"])
        self.assertTrue(old[WIND]["fresh"])
        self.assertTrue(old[COV]["fresh"])
        rebound = CASES["reacquired"]
        self.assertTrue(rebound[WIND]["fresh"])
        self.assertFalse(rebound[HUD]["present"])
        self.assertGreater(rebound[WIND]["session_epoch"], fresh[WIND]["session_epoch"])
        self.assertGreater(CASES["reset"][WIND]["session_epoch"], rebound[WIND]["session_epoch"])
        self.assertGreater(CASES["reset"][WIND]["metadata_rejected"], rebound[WIND]["metadata_rejected"])

    def test_malformed_inputs_cannot_replace_cache(self):
        before = CASES["before_invalid"]
        after = CASES["invalid_preserves"]
        counts = {HUD: 6, WIND: 2, COV: 3}
        for key, count in counts.items():
            with self.subTest(key=key):
                self.assertEqual(after[key]["invalid_samples"], count)
                self.assertEqual(after[key]["wire_rejected"], sum(counts.values()))
                for field in ("data", "observation_id", "effective_at", "read_at"):
                    self.assertEqual(after[key][field], before[key][field])

    def test_mavlink_1_and_trimmed_mavlink_2(self):
        codec = CASES["codec"]
        self.assertEqual(codec[HUD]["data"], CASES["fresh"][HUD]["data"])
        self.assertFalse(codec[WIND]["present"])
        self.assertEqual(codec[WIND]["invalid_samples"], 1)
        self.assertEqual(codec[COV]["data"], {
            "frame": "NED",
            "time_usec_raw": "1",
            "north_m_s": 0,
            "east_m_s": 0,
            "down_m_s": 0,
            "horizontal_variability_std_m_s": 0,
            "vertical_variability_std_m_s": 0,
            "alt_msl_m": 0,
            "horizontal_accuracy_std_m_s": None,
            "vertical_accuracy_std_m_s": None,
        })

    def test_pressure_instances_and_temperature_encodings(self):
        fresh = CASES["pressure_fresh"]
        self.assertTrue(all(fresh[key]["fresh"] for key in (PRESSURE, PRESSURE2, PRESSURE3)))
        self.assertEqual(fresh[PRESSURE]["data"], {
            "time_boot_ms": 424242,
            "press_abs_hpa": 1007.25,
            "press_diff_hpa": -1.75,
            "press_diff_availability": "nonzero_reported",
            "temperature_cdeg": 2345,
            "temperature_press_diff_cdeg_raw": -125,
            "temperature_press_diff_encoding": "centidegrees",
        })
        self.assertEqual(fresh[PRESSURE2]["data"], {
            "time_boot_ms": 2**32 - 1,
            "press_abs_hpa": None,
            "press_diff_hpa": -4.5,
            "press_diff_availability": "nonzero_reported",
            "temperature_cdeg": 2134,
            "temperature_press_diff_cdeg_raw": 1,
            "temperature_press_diff_encoding": "zero_or_one_cdeg",
        })
        self.assertEqual(fresh[PRESSURE3]["data"], {
            "time_boot_ms": 123,
            "press_abs_hpa": 912.5,
            "press_diff_hpa": 0,
            "press_diff_availability": "zero_ambiguous",
            "temperature_cdeg": 0,
            "temperature_press_diff_cdeg_raw": None,
            "temperature_press_diff_encoding": "unknown",
        })
        self.assertEqual(CASES["pressure_zero_tail"][PRESSURE3]["data"], {
            "time_boot_ms": 1,
            "press_abs_hpa": None,
            "press_diff_hpa": 0,
            "press_diff_availability": "zero_ambiguous",
            "temperature_cdeg": None,
            "temperature_press_diff_cdeg_raw": None,
            "temperature_press_diff_encoding": "unknown",
        })

    def test_pressure_instances_keep_independent_age_and_reject_bad_updates(self):
        first = CASES["pressure_fresh"]
        later = CASES["pressure_independent"]
        self.assertEqual(first[PRESSURE]["effective_at"], 100.2)
        self.assertEqual(first[PRESSURE]["read_at"], 100.25)
        self.assertEqual(first[PRESSURE]["age_basis"], "QUEUE_EMPTY_FLOOR")
        self.assertFalse(later[PRESSURE]["fresh"])
        self.assertTrue(later[PRESSURE2]["fresh"])
        self.assertTrue(later[PRESSURE3]["fresh"])
        self.assertEqual(later[PRESSURE]["source_rejected"], 1)
        self.assertEqual(later[PRESSURE2]["order_rejected"], 1)
        for key, count in ((PRESSURE, 4), (PRESSURE2, 0), (PRESSURE3, 1)):
            self.assertEqual(later[key]["invalid_samples"], count)
            for field in ("data", "observation_id", "effective_at", "read_at"):
                self.assertEqual(later[key][field], first[key][field])
            self.assertFalse(CASES["pressure_heartbeat_expired"][key]["present"])

    def test_checked_configuration_and_publish_pacing(self):
        before = CASES["configuration"][HUD]
        self.assertEqual(before["sample_timeout_s"], .25)
        self.assertEqual(before["publish_interval_s"], .2)
        self.assertTrue(before["fresh"])
        self.assertEqual(CASES["interval_suppressed"], {})
        after = CASES["configured_stale"][HUD]
        self.assertFalse(after["fresh"])
        self.assertEqual(after["observation_id"], before["observation_id"])

    def test_fixed_wing_air_data_semantics_and_trimming(self):
        first = CASES["fixed_wing_fresh"]
        self.assertEqual(sensors(first).keys(), {0, 1})
        for sensor, data in AIRSPEED_DATA.items():
            sample = sensors(first)[sensor]
            meta = sample["metadata"]
            self.assertEqual(sample["data"], data)
            self.assertEqual((sample["wire_version"], sample["payload_length"]), (2, 12 - sensor))
            self.assertTrue(meta["fresh"])
            self.assertEqual((meta["system_id"], meta["component_id"], meta["autopilot"]), (1, 1, 3))
        self.assertEqual(sensors(first)[0]["metadata"]["age_basis"], "QUEUE_EMPTY_FLOOR")
        self.assertTrue(sensors(first)[0]["metadata"]["age_is_upper_bound"])
        self.assertEqual(first[HIGH_LATENCY]["data"], HIGH_LATENCY_DATA)
        self.assertEqual(first[HUD]["data"], CASES["fresh"][HUD]["data"])

        rebound = CASES["fixed_wing_rebound"]
        zero = sensors(rebound)[0]
        self.assertEqual((zero["wire_version"], zero["payload_length"]), (2, 1))
        self.assertEqual(zero["data"], {
            **AIRSPEED_DATA[1],
            "airspeed_reported_cas_m_s": 0,
            "temperature_cdeg": 0,
            "raw_pressure_wire": 0,
            "differential_pressure_hpa_if_pinned_ardupilot_pa": 0,
        })
        self.assertEqual(rebound[HIGH_LATENCY]["data"], {
            **HIGH_LATENCY_DATA,
            "timestamp_ms_raw": 0,
            "airspeed_raw": 255,
            "airspeed_setpoint_raw": 0,
            "airspeed_reported_m_s": 51,
            "airspeed_setpoint_reported_m_s": 0,
        })

    def test_fixed_wing_source_malformed_and_independent_lifetimes(self):
        first, rejected = CASES["fixed_wing_fresh"], CASES["fixed_wing_rejected"]
        self.assertEqual(rejected[AIRSPEED]["invalid_samples"], 9)
        self.assertEqual(rejected[HIGH_LATENCY]["invalid_samples"], 2)
        self.assertEqual(rejected[HIGH_LATENCY]["source_rejected"], first[HIGH_LATENCY]["source_rejected"] + 4)
        self.assertEqual(rejected[HIGH_LATENCY]["order_rejected"], first[HIGH_LATENCY]["order_rejected"] + 2)
        pairs = [
            (first[HIGH_LATENCY], rejected[HIGH_LATENCY]),
            (first[HUD], rejected[HUD]),
        ]
        pairs.extend((sensors(first)[i]["metadata"], sensors(rejected)[i]["metadata"]) for i in (0, 1))
        for before, after in pairs:
            for field in ("session_epoch", "observation_id", "effective_at", "read_at"):
                self.assertEqual(before[field], after[field])
        for i in (0, 1):
            self.assertEqual(sensors(first)[i]["data"], sensors(rejected)[i]["data"])
        self.assertEqual(rejected[HIGH_LATENCY]["data"], HIGH_LATENCY_DATA)
        self.assertEqual(rejected[HUD]["data"], first[HUD]["data"])

        later = CASES["fixed_wing_independent"]
        self.assertFalse(sensors(later)[0]["metadata"]["fresh"])
        self.assertEqual(sensors(later)[0]["metadata"]["observation_id"], sensors(first)[0]["metadata"]["observation_id"])
        self.assertTrue(sensors(later)[1]["metadata"]["fresh"])
        self.assertGreater(sensors(later)[1]["metadata"]["observation_id"], sensors(first)[1]["metadata"]["observation_id"])
        self.assertFalse(later[HIGH_LATENCY]["fresh"])
        for name in ("fixed_wing_before_bind", "fixed_wing_lost", "fixed_wing_payload_without_heartbeat",
                     "fixed_wing_reset", "wrong_vehicle_type", "unsupported_autopilot"):
            self.assertEqual(CASES[name][AIRSPEED]["samples"], [], name)
            self.assertFalse(CASES[name][HIGH_LATENCY]["present"], name)
        self.assertTrue(CASES["wrong_vehicle_type"][HUD]["present"])
        self.assertGreater(CASES["fixed_wing_rebound"][HIGH_LATENCY]["session_epoch"], first[HIGH_LATENCY]["session_epoch"])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--driver", required=True)
    parser.add_argument("--module", required=True)
    args = parser.parse_args()
    result = subprocess.run([args.driver, args.module], capture_output=True, text=True, timeout=10)
    if result.returncode:
        parser.exit(1, f"scenario driver failed ({result.returncode})\n{result.stderr}")
    CASES = {}
    for line in result.stdout.splitlines():
        event = json.loads(line, parse_constant=reject_constant, object_pairs_hook=unique_object)
        if event["phase"] in CASES:
            raise ValueError(f'duplicate phase {event["phase"]}')
        CASES[event["phase"]] = event["posts"]
    unittest.main(argv=[__file__])
