"""Check link-status wire vectors and lifecycle from the shared iMAVLink fixture."""

RADIO = "MAVMOOS_RADIO_STATUS"
CELLULAR = "MAVMOOS_CELLULAR_STATUS"
KEYS = (RADIO, CELLULAR)
MESSAGES = {RADIO: ("RADIO_STATUS", 109), CELLULAR: ("CELLULAR_STATUS", 334)}

RADIO_DATA = {
    "signal_scale": "device_dependent",
    "rssi_raw": 201, "rssi_device_units": 201,
    "remrssi_raw": 255, "remrssi_device_units": None,
    "noise_raw": 0, "noise_device_units": 0,
    "remnoise_raw": 47, "remnoise_device_units": 47,
    "tx_buffer_free_pct_raw": 37, "tx_buffer_free_pct": 37,
    "rx_errors_raw": 65535, "corrected_packets_raw": 4000,
}
CELLULAR_EXTENSIONS = {
    "link_tx_rate_kib_s": 123, "link_rx_rate_kib_s": 456,
    "cell_tower_id": 'C"42\\AB99', "cell_tower_id_raw": list(b'C"42\\AB99'),
    "band_number": 3, "band_frequency_mhz": 1800.25, "channel_number": 1822,
    "rx_level_dbm": -85.5, "tx_level_dbm": 13.25, "rx_quality_raw": -12.5, "sinr_db": 8.75,
}
CELLULAR_DATA = {
    "status_raw": 1, "failure_reason_raw": 3, "failure_reason_applicable": True, "network_type_raw": 4,
    "quality_pct_raw": 72, "quality_pct": 72,
    "mcc_raw": 302, "mcc": 302, "mnc_raw": 65535, "mnc": None, "lac_raw": 0, "lac": None,
    **CELLULAR_EXTENSIONS,
}
BASE_DATA = {
    "status_raw": 200, "failure_reason_raw": 254, "failure_reason_applicable": False, "network_type_raw": 253,
    "quality_pct_raw": 255, "quality_pct": None,
    "mcc_raw": 65535, "mcc": None, "mnc_raw": 0, "mnc": 0, "lac_raw": 0, "lac": None,
    **dict.fromkeys(CELLULAR_EXTENSIONS),
}
EXPECTED = {RADIO: {-1: (9, RADIO_DATA)}, CELLULAR: {0: (10, BASE_DATA), 3: (53, CELLULAR_DATA)}}


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def entries(record, key):
    require((record["family"], record["message_id"]) == MESSAGES[key] and record["schema"] == 1
            and record["read_only"] is True and record["authoritative_for_control"] is False,
            f"{key}: wrong observation envelope")
    if key == RADIO:
        if record["present"]:
            return {-1: (record, record)}
        require(record["data"] is None and record["effective_at"] is None and record["read_at"] is None
                and record["wire_version"] is None and record["payload_length"] is None and not record["fresh"],
                f"{key}: absent radio retained a sample")
        return {}
    rows = record["modems"]
    ids = [row["modem_id"] for row in rows]
    require(record["present"] is bool(rows) and record["modem_limit"] == 8 and len(rows) <= 8
            and record["atomic"] is False and ids == sorted(set(ids))
            and all(type(value) is int and 0 <= value <= 255 for value in ids),
            f"{key}: inconsistent modem envelope")
    return {row["modem_id"]: (row, row["metadata"]) for row in rows}


def audit_loopback(before_bind, before_recovery, samples, autopilot):
    result = {}
    for key in KEYS:
        initial = before_bind.get(key, ())
        prefix = before_recovery.get(key, ())
        records = samples.get(key, ())
        require(initial and prefix[:len(initial)] == initial and records[:len(prefix)] == prefix
                and all(not entries(record, key) for record in initial),
                f"{key}: missing or unexpected observations before binding")
        require(prefix and not entries(prefix[-1], key), f"{key}: missing cleared pre-recovery record")
        expected = EXPECTED[key]
        fresh = {}
        stale = set()
        recovered = set()
        prior = {}
        cleared = False
        for index, record in enumerate(records):
            rows = entries(record, key)
            if not rows:
                if stale == expected.keys() and index < len(prefix):
                    cleared = True
                continue
            require(not cleared or index >= len(prefix), f"{key}: report restored without a heartbeat")
            for source, (row, meta) in rows.items():
                require(source in expected, f"{key}: unselected source {source}")
                length, data = expected[source]
                require(row["wire_version"] == 2 and row["payload_length"] == length and row["data"] == data,
                        f"{key}: wire values changed for source {source}")
                require(meta["read_only"] is True and meta["authoritative_for_control"] is False
                        and meta["present"] is True and meta["clock_fault"] is False
                        and (meta["system_id"], meta["component_id"], meta["autopilot"]) == (1, 1, autopilot)
                        and meta["source_age"] == "unknown" and meta["source_quality"] == "unassessed"
                        and meta["age_basis"] == "QUEUE_EMPTY_FLOOR" and meta["age_is_upper_bound"] is True
                        and 0 < meta["effective_at"] <= meta["read_at"]
                        and meta["age_s"] >= meta["read_at"] - meta["effective_at"]
                        and meta["fresh"] is (meta["age_s"] <= meta["sample_timeout_s"]),
                        f"{key}: invalid source or original age")
                identity = source, meta["session_epoch"], meta["observation_id"]
                times = meta["effective_at"], meta["read_at"]
                if identity in prior:
                    old_times, age = prior[identity]
                    require(times == old_times and meta["age_s"] >= age,
                            f"{key}: repeated publication renewed source {source}")
                prior[identity] = times, meta["age_s"]
                if index < len(prefix):
                    if meta["fresh"]:
                        fresh.setdefault(source, set()).add(identity)
                    elif identity in fresh.get(source, ()):
                        stale.add(source)
                elif (cleared and meta["fresh"] and source in fresh
                      and all(identity[1] > old[1] for old in fresh[source])):
                    recovered.add(source)
        require(fresh.keys() == stale == recovered == expected.keys() and cleared,
                f"{key}: missing fresh, stale, cleared or recovered evidence")
        result[key] = {"fresh_observations": sum(len(ids) for ids in fresh.values()),
                       "stale_sources": len(stale), "recovered_sources": len(recovered)}
    return result
