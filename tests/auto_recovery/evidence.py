"""Replay the restart fixture's original MOOS, boundary and selected FC wire evidence."""
import hashlib
import json
import math
import re
import shlex
import struct
from pathlib import PurePath

BOUNDARIES = ("restart.ready", "takeover.request", "handback.ready")
EVENTS = ("restart.ready", "bridge_killed", "bridge_restarted", "takeover.request",
          "operator_sent", "handback.ready", "capture_finished")
MARKERS = ("before_kill", "after_kill", "after_restart", "before_handback")
MODULES = {"telemetry", "parameters", "missions", "arducopter_control"}
TERMINALS = {"COMPLETED", "REJECTED", "RELEASED", "FAILED", "TIMED_OUT", "INCOMPLETE",
             "LOST", "CANCELLED"}
RUNTIME = "MAVMOOS_RUNTIME_STATUS"
RECOVERY = "MAVMOOS_AUTO_RECOVERY_STATUS"
PROOF = "MAVMOOS_STORED_MISSION_PROOF"


def need(condition, why):
    if not condition:
        raise ValueError(why)


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def records(data, clock, *, allow_plain=False):
    need(len(data) <= 32 * 1024 * 1024, "trace byte bound exceeded")
    rows = []
    previous = -1
    for line in data.splitlines():
        if allow_plain and not line.lstrip().startswith(b"{"):
            continue
        row = json.loads(line)
        now = row[clock]
        need(finite(now) and now >= max(0, previous), "trace clock is invalid or backward")
        previous = now
        rows.append(row)
    need(0 < len(rows) <= 120000, "trace record bound differs")
    return rows


def fields(text):
    items = [pair.split("=", 1) for pair in text.split(",")]
    need(all(len(item) == 2 for item in items), "malformed MOOS key/value observation")
    value = dict(items)
    need(len(value) == len(items), "duplicate MOOS key/value observation")
    return value


def crc(frame, extra):
    state = 0xffff
    for byte in frame + bytes([extra]):
        tmp = byte ^ (state & 255)
        tmp ^= (tmp << 4) & 255
        state = (state >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
    return state


def frames(data):
    need(0 < len(data) <= 32 * 1024 * 1024, "MAVLink byte bound differs")
    offset = 6 if data.startswith(b"0 " * 3) else 0
    while offset < len(data):
        need(data[offset] in (0xfe, 0xfd) and offset + 2 <= len(data), "MAVLink frame boundary differs")
        v2 = data[offset] == 0xfd
        header = 10 if v2 else 6
        length = data[offset + 1]
        end = offset + header + length
        # These owned simulator endpoints do not use signatures.
        need(end + 2 <= len(data) and (not v2 or data[offset + 2] == 0),
             "truncated or incompatible MAVLink frame")
        packet = data[offset:end + 2]
        source = tuple(packet[5:7] if v2 else packet[3:5])
        message = int.from_bytes(packet[7:10] if v2 else packet[5:6], "little")
        yield message, source, packet[header:-2], packet
        offset = end + 2


def selected_crc(packet, extra):
    need(crc(packet[1:-2], extra) == int.from_bytes(packet[-2:], "little"),
         "selected MAVLink frame checksum differs")


def eastward_metres(start, end):
    latitude = math.radians(start["lat"] * 1e-7)
    return math.radians((end["lon"] - start["lon"]) * 1e-7) * 6371000 * math.cos(latitude)


def audit_wire(raw, read, events, expected_type, extended):
    observed = records(read("fc-heartbeats.jsonl"), "mono")
    heartbeats, positions = [], []
    for message, source, payload, packet in frames(read("fc-observer.mavlink")):
        need(source == (1, 1), "passive wire source differs")
        if extended and message == 33:
            selected_crc(packet, 104)
            _, lat, lon, _, relative_alt, _, vy, _, _ = struct.unpack("<IiiiihhhH", payload.ljust(28, b"\0"))
            positions.append((lat, lon, relative_alt, vy, packet.hex()))
        if message != 0:
            continue
        selected_crc(packet, 50)
        need(len(payload) == 9, "HEARTBEAT payload size differs")
        mode, vehicle, autopilot, base, _status, version = struct.unpack("<IBBBBB", payload)
        need((vehicle, autopilot, version) == (expected_type, 3, 3), "HEARTBEAT target profile differs")
        heartbeats.append((mode, bool(base & 128), packet.hex()))
    need(heartbeats == [(p["mode"], p["armed"], p["wire_hex"]) for p in observed]
         and all(type(p["mode"]) is int and type(p["armed"]) is bool for p in observed),
         "raw HEARTBEAT content/order differs from recorded events")
    command = list(frames(read("operator-guided.mavlink")))
    need(len(command) == 1, "operator packet count differs")
    message, source, payload, packet = command[0]
    selected_crc(packet, 89)
    need(message == 11 and source == (255, 190) and payload == struct.pack("<IBB", 4, 1, 1),
         "operator wire does not contain the single explicit Guided request")
    actor = raw["operator_effect"]
    need(actor == {"datagrams_sent": 1, "source": [255, 190], "target": 1, "mode": 4},
         "operator send witness differs")

    def witness(mode, armed, after, before):
        return any(row["mode"] == mode and row["armed"] is armed and after < row["mono"] <= before
                   for row in observed)

    before = events["restart.ready"]
    takeover = events["takeover.request"]
    guided = events["handback.ready"]
    need(witness(3, True, before - 2, before)
         and witness(3, True, max(takeover - 2, events["bridge_restarted"]), takeover)
         and witness(4, True, max(guided - 2, events["operator_sent"]), guided)
         and any(not row["armed"] and guided < row["mono"] <= events["capture_finished"] for row in observed),
         "independent FC AUTO, Guided or final disarmed boundary missing")
    if extended:
        motion = raw["fresh_helm_motion"]
        start, end = motion["start"], motion["end"]
        keys = ("lat", "lon", "relative_alt", "vy", "wire_hex")
        samples = [tuple(row[key] for key in keys) for row in (start, end)]
        need(samples[0] in positions and samples[1] in positions and
             positions.index(samples[0]) < positions.index(samples[1]) and
             witness(5, True, events["handback.ready"], events["guidance.active"]) and
             events["guidance.active"] < start["mono"] < end["mono"] <= events["guidance.moved"] and
             witness(4, True, events["guidance.active"], events["guidance.moved"]) and
             motion["east_m"] == eastward_metres(start, end) >= 3 and
             60 <= end["vy"] <= 220 and 3000 <= end["relative_alt"] <= 7000,
             "post-restart stock helm motion differs from independent wire")
    return len(observed)


def audit_processes(raw, events, expected_frame):
    actors = raw["processes"]
    expected = {"sitl", "antler", "bridge_one", "bridge_two", "starter", "probe"}
    need(set(actors) == expected and len({p["pid"] for p in actors.values()}) == len(expected)
         and all(type(p["pid"]) is int and p["pid"] > 0 and finite(p["started_mono"])
                 and p["argv"] for p in actors.values()), "owned process actor identity differs")
    native, first, second = (actors[name] for name in ("sitl", "bridge_one", "bridge_two"))
    need(raw["sitl_pid"] == native["pid"] and PurePath(native["argv"][0]).name == "arducopter"
         and native["argv"][1:3] == ["--model", expected_frame]
         and native["started_mono"] < first["started_mono"] < events["restart.ready"]
         and events["bridge_killed"] < second["started_mono"] < events["bridge_restarted"]
         and first["argv"] == second["argv"] and len(first["argv"]) == 3
         and PurePath(first["argv"][0]).name == "iMAVLink"
         and PurePath(first["argv"][1]).name == "targ_copter.moos" and first["argv"][2] == "--alias=iMAVLink"
         and raw["bridge_one_exit"] == -9 and raw["starter_exit"] == -15,
         "bridge replacement or same-native execution proof differs")
    old, new = raw["owned_helm_before"], raw["owned_helm_after"]
    argv = shlex.split(old["command"])
    need(old["pid"] == new["pid"] and old["command"] == new["command"] and argv
         and PurePath(argv[0]).name == "pHelmIvP" and first["argv"][1] in argv,
         "stock pHelmIvP process changed across bridge replacement")
    for name, process, port, peer in (
        ("bridge_socket_before", first, "bridge", None),
        ("bridge_socket_after", second, "bridge", None),
        ("native_primary_socket_inventory", native, "bridge", "peer"),
        ("native_primary_socket_inventory_after", native, "bridge", "peer"),
        ("operator_socket_inventory_at_send", native, "operator", "operator_peer"),
    ):
        destination = f"127.0.0.1:{raw['ports'][port]}"
        endpoint = destination if peer is None else f"127.0.0.1:{raw['ports'][peer]}->{destination}"
        need(any(len(columns := line.split()) > 2 and columns[1] == str(process["pid"])
                 and "UDP" in columns and endpoint in columns for line in raw[name].splitlines()),
             "owned socket inventory differs: " + name)


def audit_marker(raw, read, events):
    snapshots = raw["marker_observations"]
    need(set(snapshots) == set(MARKERS) and raw["marker_absent_after_handback"] is True,
         "marker persistence/release inventory differs")
    identity = {(row["device"], row["inode"]) for row in snapshots.values()}
    need(len(identity) == 1 and all(type(x) is int and x >= 0 for x in next(iter(identity))),
         "marker inode changed across restart")
    for role, row in snapshots.items():
        name = role.replace("_", "-") + ".marker.bin"
        original = read(name)
        need(row["file"] == name and original == b"" and row["size"] == 0
             and row["sha256"] == hashlib.sha256(original).hexdigest() and finite(row["mono"]),
             "presence-only original marker bytes differ")
    times = [snapshots[role]["mono"] for role in MARKERS]
    need(events["restart.ready"] <= times[0] < events["bridge_killed"] <= times[1]
         < events["bridge_restarted"] <= times[2] < events["handback.ready"] <= times[3]
         < events["capture_finished"], "marker lifecycle event order differs")


def audit_moos(raw, read, expected_abi, expected_class):
    rows = records(read("probe.jsonl"), "elapsed", allow_plain=True)
    starter = records(read("starter.jsonl"), "elapsed", allow_plain=True)
    need(not any(r["kind"] == "fail" for r in rows) and not any(r["type"] == "fail" for r in starter),
         "probe failure contradicts completion")
    events = [(i, row) for i, row in enumerate(rows) if row["kind"] != "mail"]
    mail = [(i, row, fields(row["value"])) for i, row in enumerate(rows)
            if row["kind"] == "mail" and row["key"] != "MAVMOOS_CAPABILITIES"]
    mail_fields = {i: value for i, _, value in mail}
    need(all(r["source"] == "iMAVLink" and finite(r["published_at"]) for r in rows if r["kind"] == "mail"),
         "observed bridge source/publication time differs")

    def event(kind, key):
        hits = [(i, row) for i, row in events if row["kind"] == kind and row["key"] == key]
        need(len(hits) == 1, "missing or duplicate probe event: " + key)
        return hits[0]

    def observed(key, after, before, expected):
        return any(after < i < before and r["key"] == key and all(f.get(k) == v for k, v in expected.items())
                   for i, r, f in mail)

    def response(id_, request_key, result_key, status, after, before, request_fields, reason=None):
        sent = [(i, fields(r["value"])) for i, r in events if r["kind"] == "request"
                and r["key"] == request_key and r["value"] != "acquire" and r["value"] != "release"
                and fields(r["value"]).get("id") == id_]
        need(len(sent) == 1 and after < sent[0][0] < before
             and all(sent[0][1].get(k) == v for k, v in request_fields.items()),
             "explicit request differs: " + id_)
        received = [(i, f) for i, r, f in mail if r["key"] == result_key and f.get("id") == id_
                    and f.get("status") in TERMINALS]
        need(received and all(sent[0][0] < i < before and f["status"] == status
                             and (reason is None or f.get("reason") == reason) for i, f in received),
             "terminal outcome differs: " + id_)
        return received[-1][0]

    boundaries = (*BOUNDARIES, "guidance.active") if expected_class is not None else BOUNDARIES
    for key in boundaries:
        sent, request = event("boundary", key)
        got, reply = event("boundary_ack", key)
        data = read("control/" + key)
        need(sent < got and request["value"] == reply["value"] and data == request["value"].encode()
             and read("control/" + key + ".ack") == data,
             "original boundary token/ack differs: " + key)

    restart, restart_token = event("boundary", "restart.ready")
    observed_restart, restarted = event("phase", "restart.observed")
    takeover, takeover_token = event("boundary", "takeover.request")
    handback, handback_token = event("boundary", "handback.ready")
    completed, complete = event("phase", "handback.completed")
    acquired, grant = event("phase", "new_acquisition")
    final, finished = event("pass", "auto_restart_recovery")
    session = restarted["value"]
    old = restart_token["value"].removeprefix("restart.ready:").rstrip("\n")
    need(old != session and all(re.fullmatch(r"[A-Za-z0-9_.-]{1,40}", value) for value in (old, session))
         and restart_token["value"] == "restart.ready:" + old + "\n"
         and takeover_token["value"] == "takeover.request:" + session + "\n"
         and handback_token["value"] == "handback.ready:" + session + "\n"
         and complete["value"] == grant["value"] == finished["value"] == session
         and restart < observed_restart < takeover < handback < completed < acquired < final,
         "process session/phase sequence differs")
    for boundary, next_phase in zip(BOUNDARIES, (observed_restart, handback, completed)):
        need(event("boundary_ack", boundary)[0] < next_phase, "phase preceded matching acknowledgement")
    airborne = {"armed": "1", "in_air": "1", "heartbeat_valid": "1", "landed_valid": "1", "permission": "false"}
    unresolved = {"session": session, "state": "UNRESOLVED", "marker": "present", "effects_blocked": "true"}
    need(observed(RECOVERY, -1, restart, {"session": old, "state": "OWNED", "marker": "present"})
         and observed(RUNTIME, -1, restart, {**airborne, "fc_mode": "3", "mission_execution_held": "true"})
         and observed(PROOF, -1, restart, {"valid": "true", "verification_id": "upload"})
         and observed(RECOVERY, restart, observed_restart, unresolved)
         and observed(RUNTIME, restart, observed_restart, {**airborne, "fc_mode": "3", "mission_execution_held": "false"})
         and observed(PROOF, restart, observed_restart, {"valid": "false"})
         and observed(RECOVERY, takeover, handback, {**unresolved, "ready_for_handback": "true"})
         and observed(RUNTIME, takeover, handback, {**airborne, "fc_mode": "4", "mission_execution_held": "false"}),
         "owned, unresolved or fresh non-AUTO MOOS observation missing")
    need(any(r["type"] == "phase" and r["key"] == "auto.started" for r in starter)
         and any(r["type"] == "execution_result" and r.get("source") == "iMAVLink"
                 and fields(r["value"]).get("status") == "ACTIVE" and fields(r["value"]).get("id") == "run" for r in starter)
         and read("starter-control/auto.started").startswith(b"phase:auto.started:"),
         "original explicit starter did not observe ACTIVE")

    def round_trip(id_, family, status, action, after=observed_restart, before=takeover, reason=None, **extra):
        request_key = "MAVMOOS_" + family + "_REQUEST"
        result_key = "MAVMOOS_" + family + "_RESULT"
        return response(id_, request_key, result_key, status, after, before, {"action": action, **extra}, reason)

    round_trip("wrong_session", "AUTO_RECOVERY", "REJECTED", "handback", reason="wrong_process_session", session=old)
    round_trip("in_auto", "AUTO_RECOVERY", "REJECTED", "handback", reason="fresh_prior_non_auto_observation_required", session=session)
    round_trip("blocked_land", "CONTROL", "REJECTED", "land", reason="busy")
    clear = round_trip("blocked_clear", "MISSION", "FAILED", "clear", reason="mutation_state_no_longer_permitted")
    write = round_trip("blocked_write", "PARAM", "REJECTED", "write",
                       reason="simulation_wind_write_requires_disarmed_ground", name="SIM_WIND_SPD")
    need(mail_fields[clear].get("mutation_may_have_applied") == "false"
         and mail_fields[write].get("write_may_have_applied") == "false",
         "denied mutation unexpectedly reports possible handoff")
    round_trip("read_frame", "PARAM", "COMPLETED", "read", name="FRAME_CLASS")
    if expected_class is not None:
        need(observed("MAVMOOS_PARAM_VALUE", observed_restart, takeover,
                      {"id": "read_frame", "name": "FRAME_CLASS", "value": str(expected_class)}),
             "independent frame class readback differs")
    round_trip("read_mission", "MISSION", "COMPLETED", "download")
    round_trip("before_handback", "CONTROL", "REJECTED", "guided", takeover, handback, "busy")
    old_proofs = [f for i, r, f in mail if i < restart and r["key"] == PROOF
                  and f.get("valid") == "true" and f.get("verification_id") == "upload"]
    rejected_proof = response(
        "old_proof", "MAVMOOS_MISSION_CONTROL", "MAVMOOS_MISSION_EXECUTION_RESULT", "REJECTED",
        takeover, handback, {"action": "start", "verification_id": "upload",
                             "content_id": old_proofs[-1]["content_id"],
                             "proof_revision": old_proofs[-1]["revision"]})
    need(mail_fields[rejected_proof].get("reason") in {
        "current_verified_guided_entry_required", "current_verified_airborne_guided_profile_required"
    }, "old plan rejection was outside the verified entry gate")
    releases = [(i, f) for i, r, f in mail if r["key"] == "MAVMOOS_AUTO_RECOVERY_RESULT"
                and f.get("status") == "RELEASED"]
    need(len(releases) == 1 and releases[0][1]["id"] in {"handback0", "handback1", "handback2"},
         "explicit release count/identity differs")
    released = round_trip(releases[0][1]["id"], "AUTO_RECOVERY", "RELEASED", "handback",
                          handback, completed, "explicit_handback_no_grant", session=session)
    nav_requests = [(i, r["value"]) for i, r in events if r["kind"] == "request" and r["key"] == "MAVMOOS_NAV_CONTROL"]
    need(len(nav_requests) == 3 and [value for _, value in nav_requests] == ["acquire", "acquire", "release"],
         "navigation request inventory differs")
    denied, requested, release = [i for i, _ in nav_requests]
    quiet_until = completed if expected_class is not None else requested
    tx_keys = {"MAVMOOS_TX_RESULT", "MAVMOOS_MISSION_TX_RESULT", "MAVMOOS_PARAM_TX_RESULT"}
    accepted = {(r["key"], f["operation_id" if r["key"] == "MAVMOOS_MISSION_TX_RESULT" else "id"])
                for i, r, f in mail if observed_restart < i < quiet_until
                and r["key"] in tx_keys and f.get("status") == "LOCAL_ACCEPTED"}
    need(accepted == {("MAVMOOS_MISSION_TX_RESULT", "read_mission"),
                      ("MAVMOOS_PARAM_TX_RESULT", "read_frame/initial")},
         "accepted host handoffs differed from the two bounded reads")
    hold = [f for i, r, f in mail if observed_restart < i < requested and r["key"] == RUNTIME]
    need(observed_restart < denied < takeover and completed < requested < acquired < release < final
         and observed("MAVMOOS_HELM_STATUS", denied, takeover, {"reason": "acquisition_rejected"})
         and observed(RECOVERY, handback, completed, {"session": session, "state": "READY", "marker": "absent"})
         and observed(RUNTIME, released, completed, {**airborne, "fc_mode": "4"})
         and hold and all(f.get("permission") == "false" and f.get("nav_tx_accepted") == "0" for f in hold)
         and observed(RUNTIME, requested, acquired, {"permission": "true"})
         and observed(RUNTIME, release, final, {"permission": "false", "armed": "0", "in_air": "0", "landed_valid": "1"}),
         "recovery, quiet old authority, explicit acquisition or final ground evidence differs")
    round_trip("land_after", "CONTROL", "COMPLETED", "land", release, final)
    if expected_class is not None:
        round_trip("post_recovery_loiter", "CONTROL", "COMPLETED", "hold", completed, requested)
        ready, _ = event("boundary", "guidance.active")
        ack, _ = event("boundary_ack", "guidance.active")
        need(acquired < ready < ack < release and
             observed(RUNTIME, completed, requested, {"fc_mode": "5", "permission": "false"}) and
             observed(RUNTIME, requested, ack, {"fc_mode": "4", "permission": "true"}) and
             observed("MAVMOOS_HELM_STATUS", acquired, ack, {"state": "ACTIVE", "source": "pHelmIvP"}) and
             {r["key"] for i, r in events if acquired < i < ack and r["kind"] == "helm" and
              r.get("source") == "pHelmIvP" and finite(r.get("published_at"))} ==
             {"DESIRED_HEADING", "DESIRED_SPEED", "DESIRED_ALTITUDE"},
             "fresh post-acquisition stock helm evidence missing")

    catalogs = [(i, r["published_at"], json.loads(r["value"])) for i, r in enumerate(rows)
                if r["kind"] == "mail" and r["key"] == "MAVMOOS_CAPABILITIES"]
    previous = [at for i, at, _ in catalogs if i < restart]
    following = [at for i, at, _ in catalogs if restart < i < takeover]
    catalog = raw["catalog"]
    modules = catalog["composition"]["modules"]
    need(previous and following and max(previous) < min(following)
         and all(value == catalog for _, _, value in catalogs)
         and catalog["composition"]["plugin_abi"] == expected_abi and catalog["execution_mode"] == "SITL_CONTROL"
         and len(modules) == len(MODULES) and {m["name"] for m in modules} == MODULES,
         "two actual bridge compositions/publication epochs differ")
    return old, session, len(rows)


def replay_archived(raw, read, *, expected_abi=9, expected_frame="quad", expected_type=2, expected_class=1):
    """Recheck original artifact bytes through a safe relative member reader."""
    need(raw["profile"] == "arducopter" and raw["scenario"] == "auto_restart_recovery"
         and raw["hardware"] is False and raw["rc_input_sent"] is False and raw["observer_writes"] == 0
         and raw["probe_exit"] == 0, "restart case scope or probe exit differs")
    extended = expected_frame != "quad"
    need(raw.get("frame", "quad") == expected_frame, "selected Copter frame differs")
    if extended:
        identity = raw["frame_identity"]
        need((identity["expected_mav_type"], identity["expected_frame_class"]) == (expected_type, expected_class),
             "selected Copter frame evidence differs")
    events = raw["events"]
    order = (*EVENTS[:-1], "guidance.active", "guidance.moved", EVENTS[-1]) if extended else EVENTS
    need(set(events) == set(order) and all(finite(events[name]) for name in order)
         and all(events[a] < events[b] for a, b in zip(order, order[1:])),
         "restart boundary chronology differs")
    audit_processes(raw, events, expected_frame)
    audit_marker(raw, read, events)
    heartbeats = audit_wire(raw, read, events, expected_type, extended)
    old, session, mail_rows = audit_moos(raw, read, expected_abi, expected_class if extended else None)
    result = {"result": "PASS", "original_session": old, "recovered_session": session,
            "same_sitl_pid": raw["sitl_pid"], "same_stock_helm_pid": raw["owned_helm_before"]["pid"],
            "bridge_replaced": True, "marker_preserved_and_cleared": True,
            "observed_airborne_auto_across_restart": True, "observed_guided_before_explicit_handback": True,
            "bounded_reads_completed": ["read_frame", "read_mission"],
            "only_bounded_read_handoffs_observed_during_recovery": True,
            "old_authority_and_proof_rejected": True, "new_explicit_acquisition": True,
            "observed_disarmed": True, "selected_heartbeat_frames": heartbeats, "probe_rows": mail_rows,
            "independent_primary_egress_capture": False}
    if extended:
        result["fresh_stock_helm_east_m"] = raw["fresh_helm_motion"]["east_m"]
    return result
