"""Check passive clock-status payloads and lifetimes in the generated app fixture."""

SYSTEM = "MAVMOOS_SYSTEM_TIME"
REQUEST = "MAVMOOS_TIMESYNC_REQUEST"
REPLY = "MAVMOOS_TIMESYNC_REPLY"
KEYS = (SYSTEM, REQUEST, REPLY)
FAMILIES = {SYSTEM: ("SYSTEM_TIME", 2), REQUEST: ("TIMESYNC", 111), REPLY: ("TIMESYNC", 111)}

EXPECTED = {
    SYSTEM: (12, {
        "time_boot_ms": 2**32 - 1,
        "time_unix_usec_raw": str(2**64 - 1),
        "utc_reported": True,
        "time_unix_usec": str(2**64 - 1),
    }),
    REQUEST: (17, {
        "packet_kind": "request",
        "tc1_ns_raw": "0",
        "ts1_ns_raw": str(-(2**63)),
        "timestamp_domains": "unqualified",
        "target_scope": "system_all_components",
        "target_system_raw": 42,
        "target_component_raw": None,
    }),
    REPLY: (18, {
        "packet_kind": "reply",
        "tc1_ns_raw": str(2**63 - 1),
        "ts1_ns_raw": "-9007199254741011",
        "timestamp_domains": "unqualified",
        "target_scope": "system_component",
        "target_system_raw": 245,
        "target_component_raw": 191,
    }),
}


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def validate_envelope(row, key):
    require((row["family"], row["message_id"]) == FAMILIES[key] and row["schema"] == 1
            and row["read_only"] is True and row["authoritative_for_control"] is False,
            f"{key}: wrong observation envelope")
    if not row["present"]:
        require(not row["fresh"] and row["data"] is None and row["wire_version"] is None
                and row["payload_length"] is None and row["effective_at"] is None and row["read_at"] is None,
                f"{key}: absent clock record retained a sample")


def audit_loopback(before_bind, before_recovery, samples, autopilot):
    result = {}
    for key in KEYS:
        initial = before_bind.get(key, ())
        prefix = before_recovery.get(key, ())
        records = samples.get(key, ())
        require(initial and prefix[:len(initial)] == initial and records[:len(prefix)] == prefix
                and all(not row["present"] for row in initial),
                f"{key}: missing or unexpected observations before binding")
        require(prefix and not prefix[-1]["present"], f"{key}: missing cleared pre-recovery record")
        length, expected = EXPECTED[key]
        fresh = set()
        stale = cleared = False
        recovered = set()
        prior = {}
        for index, row in enumerate(records):
            validate_envelope(row, key)
            if not row["present"]:
                if stale and index < len(prefix):
                    cleared = True
                continue
            require(not cleared or index >= len(prefix), f"{key}: report restored without a heartbeat")
            require(row["wire_version"] == 2 and row["payload_length"] == length and row["data"] == expected,
                    f"{key}: wire values changed")
            require((row["system_id"], row["component_id"], row["autopilot"]) == (1, 1, autopilot)
                    and row["clock_fault"] is False
                    and row["source_age"] == "unknown" and row["source_quality"] == "unassessed"
                    and row["age_basis"] == "QUEUE_EMPTY_FLOOR" and row["age_is_upper_bound"] is True
                    and 0 < row["effective_at"] <= row["read_at"]
                    and row["age_s"] >= row["read_at"] - row["effective_at"]
                    and row["fresh"] is (row["age_s"] <= row["sample_timeout_s"]),
                    f"{key}: invalid source or original age")
            identity = row["session_epoch"], row["observation_id"]
            times = row["effective_at"], row["read_at"]
            if identity in prior:
                old_times, age = prior[identity]
                require(times == old_times and row["age_s"] >= age, f"{key}: repeated publication renewed the clock")
            prior[identity] = times, row["age_s"]
            if index < len(prefix):
                if row["fresh"]:
                    fresh.add(identity)
                elif identity in fresh:
                    stale = True
            elif cleared and row["fresh"] and all(identity[0] > old[0] for old in fresh):
                recovered.add(identity)
        require(fresh and stale and cleared and recovered, f"{key}: missing fresh, stale, cleared or recovered evidence")
        result[key] = {"fresh_observations": len(fresh), "stale": stale, "recovered_observations": len(recovered)}
    return result
