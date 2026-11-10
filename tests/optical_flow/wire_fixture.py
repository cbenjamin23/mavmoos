"""Independently check the generated optical-flow vectors published by iMAVLink."""

FLOW = "MAVMOOS_OPTICAL_FLOW"
RAD = "MAVMOOS_OPTICAL_FLOW_RAD"
KEYS = (FLOW, RAD)
MESSAGES = {FLOW: ("OPTICAL_FLOW", 100), RAD: ("OPTICAL_FLOW_RAD", 106)}

LEGACY = {
    "time_usec_raw": "9007199254741001",
    "flow_x_raw": 21,
    "flow_y_raw": -17,
    "flow_comp_m_x_raw": .75,
    "flow_comp_m_y_raw": -1.25,
    "ground_distance_m_raw": -1,
    "ground_distance_m": None,
    "flow_rate_x_rad_s": .125,
    "flow_rate_y_rad_s": -.25,
    "quality_raw": 127,
}
INTEGRATED = {
    "time_usec_raw": "9007199254741003",
    "integration_time_us": 30000,
    "integrated_x_rad": .25,
    "integrated_y_rad": -.125,
    "integrated_xgyro_rad": .03125,
    "integrated_ygyro_rad": None,
    "integrated_zgyro_rad": -.0625,
    "distance_m_raw": 0,
    "distance_m": 0,
    "time_delta_distance_us_raw": 2000,
    "temperature_cdeg_raw": -180,
    "quality_raw": 240,
}
EXPECTED = {
    FLOW: {
        7: (34, LEGACY),
        201: (30, dict(LEGACY, ground_distance_m_raw=0, flow_rate_y_rad_s=None, quality_raw=0)),
    },
    RAD: {7: (44, INTEGRATED)},
}


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def entries(record, key):
    require((record["family"], record["message_id"]) == MESSAGES[key] and record["schema"] == 1,
            f"{key}: wrong observation family")
    require(record["read_only"] is True and record["authoritative_for_control"] is False
            and record["atomic"] is False and record["axis_basis"] == "sensor_wire",
            f"{key}: wrong observer scope or frame basis")
    rows = record["samples"]
    ids = [row["sensor_id"] for row in rows]
    require(record["present"] is bool(rows) and record["sensor_limit"] == 8 and len(rows) <= 8
            and ids == sorted(set(ids)) and all(type(value) is int and 0 <= value <= 255 for value in ids),
            f"{key}: inconsistent sensor envelope")
    return {row["sensor_id"]: row for row in rows}


def audit_loopback(before_bind, before_recovery, samples, autopilot):
    """Prove decoded records, independent sensor ages, loss and fresh rebound."""
    result = {}
    for key in KEYS:
        initial = before_bind.get(key, ())
        prefix = before_recovery.get(key, ())
        records = samples.get(key, ())
        require(initial and prefix[:len(initial)] == initial and records[:len(prefix)] == prefix
                and all(not entries(record, key) for record in initial),
                f"{key}: missing or unexpected publication before binding")
        require(prefix and not entries(prefix[-1], key), f"{key}: missing cleared pre-recovery sample")
        accepted = {}
        stale = set()
        recovered = set()
        previous = {}
        cleared = False
        expected = EXPECTED[key]
        for index, record in enumerate(records):
            rows = entries(record, key)
            if not rows:
                if stale == expected.keys() and index < len(prefix):
                    cleared = True
                continue
            require(not cleared or index >= len(prefix), f"{key}: data reappeared without a new heartbeat")
            for sensor, row in rows.items():
                require(sensor in expected, f"{key}: unselected sensor {sensor}")
                length, data = expected[sensor]
                require(row["wire_version"] == 2 and row["payload_length"] == length and row["data"] == data,
                        f"{key}: wire values changed for sensor {sensor}")
                meta = row["metadata"]
                require(meta["read_only"] is True and meta["authoritative_for_control"] is False
                        and meta["present"] is True and meta["clock_fault"] is False
                        and (meta["system_id"], meta["component_id"], meta["autopilot"]) == (1, 1, autopilot)
                        and meta["source_age"] == "unknown" and meta["source_quality"] == "unassessed"
                        and meta["age_basis"] == "QUEUE_EMPTY_FLOOR" and meta["age_is_upper_bound"] is True
                        and 0 < meta["effective_at"] <= meta["read_at"]
                        and meta["age_s"] >= meta["read_at"] - meta["effective_at"]
                        and meta["fresh"] is (meta["age_s"] <= meta["sample_timeout_s"]),
                        f"{key}: invalid source or original age for sensor {sensor}")
                identity = sensor, meta["session_epoch"], meta["observation_id"]
                times = meta["effective_at"], meta["read_at"]
                if identity in previous:
                    earlier_times, age = previous[identity]
                    require(times == earlier_times and meta["age_s"] >= age,
                            f"{key}: repeated publication renewed sensor {sensor}")
                previous[identity] = times, meta["age_s"]
                if index < len(prefix):
                    if meta["fresh"]:
                        accepted.setdefault(sensor, set()).add(identity)
                    elif identity in accepted.get(sensor, ()):
                        stale.add(sensor)
                elif (cleared and meta["fresh"] and sensor in accepted
                      and all(identity[1] > old[1] for old in accepted[sensor])):
                    recovered.add(sensor)
        require(accepted.keys() == stale == recovered == expected.keys() and cleared,
                f"{key}: missing fresh, stale, cleared or recovered evidence")
        result[key] = {"fresh": {sensor: len(ids) for sensor, ids in accepted.items()},
                       "stale": sorted(stale), "recovered": sorted(recovered)}
    return result
