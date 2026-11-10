"""Audit fixed-wing air data from the real receive-only application."""

AIRSPEED = "MAVMOOS_AIRSPEED"
HIGH_LATENCY = "MAVMOOS_HIGH_LATENCY_AIR_DATA"
KEYS = (AIRSPEED, HIGH_LATENCY)

AIRSPEED_DATA = {
    0: {
        "airspeed_reported_cas_m_s": 19.75,
        "temperature_cdeg": -125,
        "raw_pressure_wire": -175,
        "raw_pressure_mavlink_unit": "hPa",
        "differential_pressure_hpa_if_pinned_ardupilot_pa": -1.75,
        "flags_raw": 131,
        "sensor_unhealthy_flag": True,
        "sensor_used_flag": True,
        "unknown_flags_raw": 128,
    },
    1: {
        "airspeed_reported_cas_m_s": 22.25,
        "temperature_cdeg": None,
        "raw_pressure_wire": None,
        "raw_pressure_mavlink_unit": "hPa",
        "differential_pressure_hpa_if_pinned_ardupilot_pa": None,
        "flags_raw": 0,
        "sensor_unhealthy_flag": False,
        "sensor_used_flag": False,
        "unknown_flags_raw": 0,
    },
}

HIGH_LATENCY_DATA = {
    "timestamp_ms_raw": 2**32 - 1,
    "type_raw": 1,
    "autopilot_raw": 3,
    "airspeed_raw": 101,
    "airspeed_setpoint_raw": 112,
    "airspeed_reported_m_s": 20.2,
    "airspeed_setpoint_reported_m_s": 22.4,
    "airspeed_basis": "fc_reported_unspecified",
    "airspeed_availability": "unknown",
    "airspeed_setpoint_availability": "unknown",
}


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def check_lifecycle(records, pre_recovery_count, name):
    before = set()
    stale = cleared = False
    recovered = set()
    prior = {}
    for index, meta in enumerate(records):
        if not meta or not meta["present"]:
            if stale and index < pre_recovery_count:
                cleared = True
            continue
        require(not cleared or index >= pre_recovery_count, f"{name}: report rebound before heartbeat")
        require(meta["schema"] == 1 and meta["read_only"] is True
                and meta["authoritative_for_control"] is False
                and (meta["system_id"], meta["component_id"], meta["autopilot"]) == (1, 1, 3)
                and meta["source_age"] == "unknown" and meta["source_quality"] == "unassessed"
                and meta["age_basis"] == "QUEUE_EMPTY_FLOOR" and meta["age_is_upper_bound"] is True
                and 0 < meta["effective_at"] <= meta["read_at"]
                and meta["age_s"] >= meta["read_at"] - meta["effective_at"]
                and meta["fresh"] is (meta["age_s"] <= meta["sample_timeout_s"]),
                f"{name}: invalid observation age or source")
        identity = meta["session_epoch"], meta["observation_id"]
        times = meta["effective_at"], meta["read_at"]
        if identity in prior:
            previous_times, age = prior[identity]
            require(times == previous_times and meta["age_s"] >= age,
                    f"{name}: old sample acquired a new clock")
        prior[identity] = times, meta["age_s"]
        if index < pre_recovery_count:
            if meta["fresh"]:
                before.add(identity)
            elif identity in before:
                stale = True
        elif cleared and meta["fresh"] and all(identity[0] > old[0] for old in before):
            recovered.add(identity)
    require(before and stale and cleared and recovered, f"{name}: missing fresh/stale/clear/rebind sequence")
    return {"fresh_observations": len(before), "stale": stale, "recovered_observations": len(recovered)}


def audit_loopback(before_bind, before_recovery, samples, invalid_high_latency):
    for key in KEYS:
        initial, prefix, records = before_bind[key], before_recovery[key], samples[key]
        require(initial and records[:len(prefix)] == prefix and prefix[:len(initial)] == initial
                and all(not record["present"] for record in initial) and not prefix[-1]["present"],
                f"{key}: prebind/prefix/cleared evidence failed")

    sensor_records = {sensor: [] for sensor in AIRSPEED_DATA}
    independent = False
    for row in samples[AIRSPEED]:
        require((row["schema"], row["family"], row["message_id"]) == (1, "AIRSPEED", 295)
                and row["read_only"] is True and row["authoritative_for_control"] is False
                and row["atomic"] is False and row["sensor_limit"] == 2 and row["invalid_samples"] == 0
                and row["present"] == bool(row["samples"]),
                "AIRSPEED envelope changed")
        by_sensor = {item["sensor_id"]: item for item in row["samples"]}
        require(len(by_sensor) == len(row["samples"]) and by_sensor.keys() <= AIRSPEED_DATA.keys(),
                "AIRSPEED admitted an unexpected sensor")
        for sensor, data in AIRSPEED_DATA.items():
            item = by_sensor.get(sensor)
            if item:
                require(item["data"] == data and item["wire_version"] == 2
                        and item["payload_length"] == 12 - sensor,
                        f"AIRSPEED {sensor}: wire field changed")
            sensor_records[sensor].append(item["metadata"] if item else None)
        if len(by_sensor) == 2 and not by_sensor[0]["metadata"]["fresh"] and by_sensor[1]["metadata"]["fresh"]:
            independent = True
    require(independent, "AIRSPEED sensor 1 did not remain fresh independently of sensor 0")

    reports = samples[HIGH_LATENCY]
    for row in reports:
        require((row["family"], row["message_id"]) == ("HIGH_LATENCY2", 235)
                and row["read_only"] is True and row["authoritative_for_control"] is False
                and row["data"] == (HIGH_LATENCY_DATA if row["present"] else None),
                "HIGH_LATENCY2 fields or authority changed")
    require(invalid_high_latency > 0 and before_recovery[HIGH_LATENCY][-1]["invalid_samples"] == invalid_high_latency
            and reports[-1]["invalid_samples"] == invalid_high_latency,
            "in-route HIGH_LATENCY2 type/autopilot mismatches did not reject")
    return {
        "sensors": {str(sensor): check_lifecycle(records, len(before_recovery[AIRSPEED]), f"AIRSPEED {sensor}")
                    for sensor, records in sensor_records.items()},
        "independent_aging": independent,
        "high_latency2": check_lifecycle(reports, len(before_recovery[HIGH_LATENCY]), "HIGH_LATENCY2"),
        "embedded_high_latency2_rejected": invalid_high_latency,
    }
