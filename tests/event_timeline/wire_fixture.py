"""Recheck the generated Events app witness and its original acquisition ages."""

TIMELINE = "MAVMOOS_EVENT_TIMELINE"
SEQUENCE = "MAVMOOS_EVENT_SEQUENCE_STATUS"
ERROR = "MAVMOOS_EVENT_REPLAY_ERROR"
KEYS = (TIMELINE, SEQUENCE, ERROR)
MESSAGES = {TIMELINE: ("EVENT", 410), SEQUENCE: ("CURRENT_EVENT_SEQUENCE", 411),
            ERROR: ("RESPONSE_EVENT_ERROR", 413)}
BROADCAST_ID, LOCAL_ID = 0xab010203, 0x01020304


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def forward_distance(start, end):
    return (end - start) & 65535


def valid_sequence(value):
    return type(value) is int and 0 <= value <= 65535


def samples(record, key):
    require((record["family"], record["message_id"]) == MESSAGES[key] and record["schema"] == 1
            and record["read_only"] is True and record["authoritative_for_control"] is False
            and type(record["sequence_epoch"]) is int and record["sequence_epoch"] > 0,
            f"{key}: invalid event family or observation scope")
    if key != TIMELINE:
        if record["present"]:
            return [(record, record, record["data"])]
        require(record["data"] is None and record["payload_length"] is None
                and record["wire_version"] is None and record["effective_at"] is None
                and record["read_at"] is None, f"{key}: absent event report retained a sample")
        return []
    rows = record["samples"]
    require(record["atomic"] is False and record["timeline_limit"] == 16
            and len(rows) <= 16 and record["present"] is bool(rows), f"{key}: invalid timeline bound")
    advertised = record["advertised_sequence"]
    cursor = record["latest_observed_sequence"]
    reference = cursor if cursor is not None else record["sequence_anchor"]
    require(all(sequence is None or valid_sequence(sequence) for sequence in (advertised, cursor, reference)),
            f"{key}: invalid sequence position")
    unseen, relation = None, "unseen"
    if advertised is not None and reference is not None:
        distance = forward_distance(reference, advertised)
        if distance >= 32768:
            relation = "ambiguous" if distance == 32768 else "behind"
        else:
            unseen = distance
            relation = "same" if distance == 0 else "ahead_across_wrap" if advertised < reference else "ahead"
    require((record["advertised_unseen"], record["advertised_relation"]) == (unseen, relation),
            f"{key}: advertised distance changed sequence history")
    sequences = [row["data"]["sequence"] for row in rows]
    require(all(valid_sequence(sequence) for sequence in sequences)
            and all(0 < forward_distance(a, b) < 32768 for a, b in zip(sequences, sequences[1:]))
            and (not rows or cursor is not None), f"{key}: invalid observed sequence order")
    return [(row["metadata"], row, row["data"]) for row in rows]


def batch_sequence(sequence, offset):
    distance = forward_distance(65456, (sequence - offset) & 65535)
    return valid_sequence(sequence) and distance % 5 == 0 and distance <= 5 * 1024


def fixture_matches(key, row, data):
    if row["wire_version"] != 2:
        return False
    if key == SEQUENCE:
        return (row["payload_length"] == 3 and batch_sequence(data["sequence"], 4)
                and data == {"sequence": data["sequence"], "flags_raw": 128, "unknown_flags": 128,
                             "reset_announced": False, "reset_applied": False})
    if key == ERROR:
        sequence = data["sequence"]
        return (row["payload_length"] == 6 and batch_sequence(sequence, 2)
                and data == {"sequence": sequence, "sequence_oldest_available": (sequence + 1) & 65535,
                             "target_system": 247, "target_component": 190, "reason_raw": 0,
                             "reason": "unavailable", "request_correlation": "none"})
    event_id, sequence = data["event_id_raw"], data["sequence"]
    if event_id not in (BROADCAST_ID, LOCAL_ID):
        return False
    local = event_id == LOCAL_ID
    base_args = bytes((0x12, 0, 0xff)) + bytes(36)
    return (row["payload_length"] == (16 if local else 53) and batch_sequence(sequence, 3 if local else 0)
            and data["event_namespace_raw"] == (event_id >> 24)
            and data["event_time_boot_ms_raw"] == (12 if local else 4294967294)
            and (data["destination_system"], data["destination_component"]) == ((245, 191) if local else (0, 0))
            and data["log_levels_raw"] == (0xc6 if local else 0x84)
            and data["internal_log_level"] == ({"raw": 12, "name": None} if local else {"raw": 8, "name": "protocol"})
            and data["external_log_level"] == ({"raw": 6, "name": "info"} if local else {"raw": 4, "name": "warning"})
            and data["argument_wire_bytes"] == (3 if local else 40)
            and data["arguments_raw_hex"] == (base_args + bytes([0 if local else 0x5a])).hex()
            and data["missing_before"] in ((1,) if local else (0, 1)) and type(data["sequence_wrap"]) is bool)


def audit_loopback(before_bind, before_recovery, records, autopilot):
    result = {}
    for key in KEYS:
        initial, prefix, full = before_bind[key], before_recovery[key], records[key]
        require(initial and prefix[:len(initial)] == initial and full[:len(prefix)] == prefix
                and all(not samples(row, key) for row in initial) and not samples(prefix[-1], key),
                f"{key}: missing or unexpected unbound probe evidence")
        seen, first_fresh, rebound = {}, set(), set()
        stale = cleared = timeline_evidence = wrap_seen = False
        for index, record in enumerate(full):
            group = samples(record, key)
            if not group and stale and index < len(prefix):
                cleared = True
            require(not cleared or not group or index >= len(prefix), f"{key}: data appeared before rebind")
            if key == TIMELINE and group:
                require(record["sequence_resets"] == 0, f"{key}: unexpected fixture sequence reset")
                timeline_evidence |= record["other_destinations"] > 0 and record["holes_observed"] > 0
            all_stale = bool(group)
            for meta, row, data in group:
                require(fixture_matches(key, row, data), f"{key}: published values differ from generated Events")
                if key == TIMELINE and index < len(prefix):
                    wrap_seen |= data["sequence_wrap"]
                require(meta["schema"] == 1 and meta["present"] is True and meta["read_only"] is True
                        and meta["authoritative_for_control"] is False and meta["clock_fault"] is False
                        and (meta["system_id"], meta["component_id"], meta["autopilot"]) == (1, 1, autopilot)
                        and meta["source_age"] == "unknown" and meta["source_quality"] == "unassessed"
                        and meta["age_basis"] == "QUEUE_EMPTY_FLOOR" and meta["age_is_upper_bound"] is True
                        and 0 < meta["effective_at"] <= meta["read_at"]
                        and meta["age_s"] + 1e-6 >= meta["read_at"] - meta["effective_at"]
                        and meta["fresh"] is (meta["age_s"] <= meta["sample_timeout_s"]),
                        f"{key}: invalid source or original age")
                identity = meta["session_epoch"], meta["observation_id"]
                original = meta["effective_at"], meta["read_at"], data
                if identity in seen:
                    prior, age = seen[identity]
                    require(original == prior and meta["age_s"] >= age, f"{key}: republish renewed an event sample")
                seen[identity] = original, meta["age_s"]
                if meta["fresh"]:
                    all_stale = False
                    if index < len(prefix):
                        first_fresh.add(identity)
                    elif cleared and all(identity[0] > old[0] for old in first_fresh):
                        rebound.add(identity)
            if all_stale and first_fresh and index < len(prefix):
                stale = True
        require(stale and cleared and first_fresh and rebound, f"{key}: incomplete age, clear and rebound evidence")
        require(key != TIMELINE or (timeline_evidence and wrap_seen),
                f"{key}: missing addressed-event, gap or wrap coverage")
        result[key] = {"first_session_fresh": len(first_fresh), "stale": stale, "rebound": len(rebound)}
        if key == TIMELINE:
            result[key]["sequence_wrap_witnessed"] = wrap_seen
    return result
