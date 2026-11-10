"""Audit the bounded debug reports produced by the generated app fixture."""

import math
import re

DEBUG = "MAVMOOS_DEBUG"
VECTOR = "MAVMOOS_DEBUG_VECT"
ARRAY = "MAVMOOS_DEBUG_FLOAT_ARRAY"
FLOAT = "MAVMOOS_NAMED_VALUE_FLOAT"
INT = "MAVMOOS_NAMED_VALUE_INT"
KEYS = (DEBUG, VECTOR, ARRAY, FLOAT, INT)
MESSAGES = {DEBUG: ("DEBUG", 254), VECTOR: ("DEBUG_VECT", 250), ARRAY: ("DEBUG_FLOAT_ARRAY", 350),
            FLOAT: ("NAMED_VALUE_FLOAT", 251), INT: ("NAMED_VALUE_INT", 252)}


def scalar(time, value, state="reported"):
    return {"time_boot_ms_raw": time, "remote_clock": "unmapped_boot_ms", "value": value, "value_state": state}


def vector(time, values, states):
    return {"time_usec_raw": time, "remote_clock": "unmapped_boot_or_unix_us",
            "xyz": values, "xyz_states": states}


def array(time, extension_bytes, values, states, element_bytes):
    return {"time_usec_raw": time, "remote_clock": "unmapped_boot_or_unix_us",
            "extension_payload_bytes": extension_bytes, "values": values,
            "value_states": states, "element_wire_bytes": element_bytes}


SHARED = "shared".encode().hex()
ESCAPED = bytes((120, 34, 92, 118)).hex()
FULL = "VECTOR0123".encode().hex()
ARRAY_LABEL = "ARRAY01234"
EXPECTED = {
    DEBUG: {0: (1, scalar(1, 0)), 7: (9, scalar(4294967295, -2.5))},
    VECTOR: {
        ESCAPED: (24, vector("9007199254741001", [.125, None, None],
                             ["reported", "wire_nan", "wire_positive_infinity"])),
        FULL: (30, vector("18446744073709551000", [-0.0, None, 2.25],
                          ["reported", "wire_negative_infinity", "reported"])),
    },
    ARRAY: {
        17: (252, array("9007199254741009", 232, [-0.0, None, None, None, 1.5] + [0] * 52 + [4.5],
                        ["reported", "wire_nan", "wire_positive_infinity", "wire_negative_infinity"]
                        + ["reported"] * 54, [4] * 58)),
        65535: (21, array("18446744073709551000", 1, [1.40129846e-45] + [0] * 57,
                          ["reported"] + ["zero_padded"] * 57, [1] + [0] * 57)),
    },
    FLOAT: {SHARED: (14, scalar(4294967295, -0.0)), "78ff": (10, scalar(4294967295, None, "wire_nan"))},
    INT: {SHARED: (14, scalar(0, -(2**31))), "": (8, scalar(2**31, 2**31 - 1))},
}


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def entries(record, key):
    require((record["family"], record["message_id"]) == MESSAGES[key] and record["schema"] == 1
            and record["read_only"] is True and record["authoritative_for_control"] is False
            and record["atomic"] is False and record["sample_limit"] == 8
            and all(type(record[field]) is int and record[field] >= 0
                    for field in ("capacity_drops", "invalid_samples")),
            f"{key}: debug family or observation scope differs")
    result = {}
    for row in record["samples"]:
        if key in (DEBUG, ARRAY):
            field, maximum = ("index", 255) if key == DEBUG else ("array_id", 65535)
            identity = row[field]
            require(type(identity) is int and 0 <= identity <= maximum, f"{key}: invalid debug index or array ID")
        if key != DEBUG:
            raw_hex = row["name_raw_hex"]
            if key != ARRAY:
                identity = raw_hex
            require(isinstance(raw_hex, str) and re.fullmatch(r"(?:[0-9a-f]{2}){0,10}", raw_hex),
                    f"{key}: unbounded or malformed raw name")
            raw = bytes.fromhex(raw_hex)
            require(0 not in raw, f"{key}: a name retained padding beyond its terminator")
            printable = bool(raw) and all(32 <= value <= 126 for value in raw)
            name = raw.decode("ascii") if printable else None
            state = "text" if printable else "non_ascii" if raw else "empty"
            require((row["name"], row["name_state"]) == (name, state), f"{key}: unsafe or changed display name")
        require(identity not in result, f"{key}: duplicate debug identity")
        result[identity] = row
    require(list(result) == sorted(result) and len(result) <= record["sample_limit"]
            and record["present"] is bool(result), f"{key}: invalid bounded cache")
    return result


def matches(row, expected, version=2):
    length, data = expected
    if (row["wire_version"], row["payload_length"], row["data"]) != (version, length, data):
        return False
    if "time_boot_ms_raw" in data and type(row["data"]["time_boot_ms_raw"]) is not int:
        return False
    if "values" in data:
        if row.get("name") != ARRAY_LABEL:
            return False
        expected_numbers, numbers = data["values"], row["data"]["values"]
    else:
        expected_numbers = data.get("xyz", [data.get("value")])
        numbers = row["data"].get("xyz", [row["data"].get("value")])
    return all(value is None or (type(actual) in (int, float) and
               (value != 0 or math.copysign(1, actual) == math.copysign(1, value)))
               for actual, value in zip(numbers, expected_numbers))


def audit_loopback(before_bind, before_recovery, samples, autopilot):
    result = {}
    for key in KEYS:
        initial = before_bind.get(key, ())
        prefix = before_recovery.get(key, ())
        records = samples.get(key, ())
        require(initial and prefix[:len(initial)] == initial and records[:len(prefix)] == prefix
                and all(not entries(row, key) for row in initial)
                and not entries(prefix[-1], key), f"{key}: absent prebind or clear phase differs")
        expected = EXPECTED[key]
        fresh, previous, stale, recovered = {}, {}, set(), set()
        cleared = False
        for index, record in enumerate(records):
            rows = entries(record, key)
            if not rows and stale == expected.keys() and index < len(prefix):
                cleared = True
            require(not cleared or not rows or index >= len(prefix), f"{key}: data restored an unbound session")
            for member, row in rows.items():
                require(member in expected and matches(row, expected[member]), f"{key}: fixture wire values differ")
                meta = row["metadata"]
                require((meta["system_id"], meta["component_id"], meta["autopilot"]) == (1, 1, autopilot)
                        and meta["schema"] == 1 and meta["present"] is True and meta["read_only"] is True
                        and meta["authoritative_for_control"] is False and meta["clock_fault"] is False
                        and meta["source_age"] == "unknown" and meta["source_quality"] == "unassessed"
                        and meta["age_basis"] == "QUEUE_EMPTY_FLOOR" and meta["age_is_upper_bound"] is True
                        and 0 < meta["effective_at"] <= meta["read_at"]
                        and meta["age_s"] + 1e-6 >= meta["read_at"] - meta["effective_at"]
                        and meta["fresh"] is (meta["age_s"] <= meta["sample_timeout_s"])
                        and all(type(meta[field]) is int and meta[field] > 0
                                for field in ("observation_id", "session_epoch")),
                        f"{key}: source or original age differs")
                identity = member, meta["session_epoch"], meta["observation_id"]
                times = meta["effective_at"], meta["read_at"]
                if identity in previous:
                    old_times, old_age = previous[identity]
                    require(times == old_times and meta["age_s"] >= old_age, f"{key}: republish renewed a sample")
                previous[identity] = times, meta["age_s"]
                if index < len(prefix):
                    if meta["fresh"]:
                        fresh.setdefault(member, set()).add(identity)
                    elif identity in fresh.get(member, ()):
                        stale.add(member)
                elif cleared and meta["fresh"] and all(identity[1] > old[1] for old in fresh[member]):
                    recovered.add(member)
        require(cleared and fresh.keys() == stale == recovered == expected.keys(),
                f"{key}: incomplete fresh, stale, cleared and rebound witness")
        result[key] = {"first_session_fresh": {str(member): len(ids) for member, ids in fresh.items()},
                       "stale": sorted(stale), "recovered": sorted(recovered)}
    return result
