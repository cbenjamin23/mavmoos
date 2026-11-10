"""Check the actual battery metadata/generator output through the shared app."""
KEYS = ("MAVMOOS_BATTERY_INFO", "MAVMOOS_GENERATOR_STATUS")


def need(ok, why):
    if not ok:
        raise ValueError(why)


def _samples(key, rows):
    if key == KEYS[0]:
        return [sample for row in rows for sample in row["instances"]]
    return [row for row in rows if row["present"]]


def audit_loopback(before_bind, before_recovery, samples, autopilot):
    need(autopilot == 3, "power fixture is pinned to the ArduPilot status route")
    found = {}
    for key in KEYS:
        need(before_bind.get(key) and not any(row["present"] for row in before_bind[key]),
             key + ": prebind sample admitted")
        data = _samples(key, samples[key])
        earlier = _samples(key, before_recovery[key])
        after = _samples(key, samples[key][len(before_recovery[key]):])
        need(earlier and after, key + ": missing selected FC data before or after recovery")
        retained = {}
        for sample in data:
            need(sample["read_only"] and not sample["authoritative_for_control"] and
                 (sample["system_id"], sample["component_id"], sample["autopilot"]) == (1, 1, 3) and
                 sample["age_basis"] == "QUEUE_EMPTY_FLOOR" and sample["age_is_upper_bound"],
                 key + ": producer or age scope changed")
            need(sample["fresh"] == (sample["age_s"] <= .65), key + ": freshness bound changed")
            row = sample["data"]
            if key == KEYS[0]:
                need(row["battery_id"] == 4 and row["design_capacity_ah"]["value"] == 10.25 and
                     row["full_charge_capacity_ah"]["value"] == 8.5 and
                     row["serial_number"]["value"] == "unit-4" and row["state_of_health_pct"]["value"] == 83 and
                     row["payload_length"] < 140, "battery manufacturer fields changed")
            else:
                need(row["flags"]["unknown_decimal"] == str(1 << 60) and
                     row["flags"]["reported"]["maintenance_required"] and row["battery_current_a"]["value"] == -3.25 and
                     row["maintenance_in_s"]["value"] == -90 and row["runtime_s"]["value"] is None,
                     "generator wire fields changed")
            token = sample["session_epoch"], sample["observation_id"]
            witness = sample["effective_at"], sample["read_at"], row
            need(token not in retained or retained[token] == witness, key + ": original sample restamped")
            retained[token] = witness
        stale = [row for row in earlier if not row["fresh"]]
        need(any(row["fresh"] for row in earlier) and stale and any(row["fresh"] for row in after),
             key + ": missing fresh/stale/recovery evidence")
        need(max(row["session_epoch"] for row in after) > max(row["session_epoch"] for row in earlier) and
             any(not row["present"] for row in before_recovery[key][len(before_bind[key]):]),
             key + ": session loss did not clear observation")
        found[key] = {"distinct": len(retained), "stale": len(stale), "source": "generated_ardupilot"}
    return found
