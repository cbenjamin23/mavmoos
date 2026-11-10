import argparse
import json
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--driver", required=True)
    parser.add_argument("--module", required=True)
    args = parser.parse_args()
    trace = subprocess.check_output([args.driver, args.module], text=True)
    reports = {row["phase"]: row["report"] for row in map(json.loads, trace.splitlines())}
    for record in reports.values():
        assert record["read_only"] and not record["authoritative_for_control"]

    for phase in ("startup", "sensor_cannot_bind", "plane_cannot_bind",
                  "after_heartbeat_loss", "after_reset"):
        assert not reports[phase]["present"] and not reports[phase]["fresh"], phase
        assert reports[phase]["data"] is None, phase

    angular = reports["angular_v1"]
    assert angular["source_rejected"] >= 4
    assert angular["system_id"] == 1 and angular["component_id"] == 158
    assert angular["fresh"] and angular["age_is_upper_bound"]
    assert angular["effective_at"] == 100.27 and angular["read_at"] == 100.30
    assert angular["data"] == {
        "message_id": 149, "wire_version": 1, "payload_length": 30,
        "time_usec_raw": str(2**64 - 1), "target_num": 7, "frame": 12,
        "distance_m": 2.5, "position_valid": False, "target_type_raw": None,
        "angle_x_rad": .125, "angle_y_rad": -.25, "position_m": None,
    }

    pose = reports["pose_v2"]
    assert pose["data"] == {
        **angular["data"], "wire_version": 2, "payload_length": 60, "target_num": 8,
        "position_valid": True, "target_type_raw": 0, "angle_x_rad": None,
        "position_m": [.5, -.125, 2],
    }
    assert pose["fresh"] and pose["observation_id"] == angular["observation_id"] + 1
    rejected = reports["preserved_after_bad_data"]
    assert rejected["wire_rejected"] == 3 and rejected["order_rejected"] == 1
    for phase in ("preserved_after_bad_data", "stale"):
        for key in ("observation_id", "data", "effective_at", "read_at"):
            assert reports[phase][key] == pose[key], (phase, key)
    assert reports["stale"]["present"] and not reports["stale"]["fresh"]
    assert reports["stale"]["age_s"] > .5

    trimmed = reports["trimmed_v2"]
    assert trimmed["fresh"] and trimmed["session_epoch"] > pose["session_epoch"]
    assert trimmed["data"]["payload_length"] == 1 and trimmed["data"]["position_m"] is None
    assert trimmed["data"]["target_type_raw"] is None
    assert reports["after_reset"]["session_epoch"] > trimmed["session_epoch"]
    fc = reports["fc_component"]
    assert fc["fresh"] and fc["component_id"] == 1
    assert fc["data"]["distance_m"] is None and fc["data"]["angle_x_rad"] == .125
    print("landing_target loaded module scenarios passed")


if __name__ == "__main__":
    main()
