"""Offline primary-app heartbeat-cache reset audit; Python standard library only.

This classifies an interrupted concurrent-list operation. It does not prove a
packet, flight-controller or link outage, nor replace the native probe's control,
reacquisition, landing and cleanup assertions.
"""

import math
from pathlib import Path


_RESET = "reset:vehicle_connection_lost"
_HEARTBEAT_TTL_S = 2.0
_PUBLICATION_TOLERANCE_S = 0.25
_VARIABLES = {
    "MAVMOOS_HEARTBEAT", "MAVMOOS_PARAM_RESULT",
    "MAVMOOS_RUNTIME_STATUS", "MAVMOOS_HELM_STATUS",
}


def _number(value, description):
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise RuntimeError(f"invalid {description}") from error
    if not math.isfinite(result) or result < 0:
        raise RuntimeError(f"invalid {description}")
    return result


def _fields(text):
    result = {}
    for part in text.strip().split(","):
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        if key in result:
            raise RuntimeError(f"ambiguous duplicate diagnostic field: {key}")
        result[key] = value
    return result


def _heartbeat(fields, alog_at, path):
    effective = _number(fields.get("rx_mono_s"), "primary heartbeat effective stamp")
    read = _number(fields.get("read_mono_s"), "primary heartbeat actual-read stamp")
    if effective > read:
        raise RuntimeError("primary heartbeat effective stamp exceeds actual read")
    return {
        "effective_cpp_mono_s": effective,
        "read_cpp_mono_s": read,
        "alog_s": alog_at,
        "alog_path": str(path),
        "session": fields.get("session", ""),
        "identity": fields.get("identity"),
        "encoding_valid": fields.get("encoding_valid"),
        "age_basis": fields.get("age_basis"),
        "receipt_id": fields.get("receipt_id"),
        "wire_seq": fields.get("wire_seq"),
    }


def _sample_key(heartbeat):
    # Repeated periodic publications of the same accepted observation are one
    # sample. Distinct receipts/sessions at an identical read stamp are ambiguous.
    return tuple(heartbeat[key] for key in (
        "effective_cpp_mono_s", "read_cpp_mono_s", "session", "identity",
        "encoding_valid", "age_basis", "receipt_id", "wire_seq"))


def _bracket(heartbeats, terminal_at):
    before = [h for h in heartbeats if h["read_cpp_mono_s"] < terminal_at]
    after = [h for h in heartbeats if terminal_at <= h["read_cpp_mono_s"]]
    if not before or not after:
        raise RuntimeError("missing primary heartbeat actual-read bracket")
    lower = max(h["read_cpp_mono_s"] for h in before)
    upper = min(h["read_cpp_mono_s"] for h in after)
    before = [h for h in before if h["read_cpp_mono_s"] == lower]
    after = [h for h in after if h["read_cpp_mono_s"] == upper]
    if len(before) != 1 or len(after) != 1:
        raise RuntimeError("ambiguous primary heartbeat actual-read bracket")
    before, after = before[0], after[0]
    if not before["session"] or before["session"] != after["session"]:
        raise RuntimeError("primary heartbeat bracket lacks one nonempty session")
    if any(h["identity"] != "EXPECTED" or h["encoding_valid"] != "true"
           for h in (before, after)):
        raise RuntimeError("primary heartbeat bracket lacks accepted identity/encoding")
    if terminal_at - before["effective_cpp_mono_s"] <= _HEARTBEAT_TTL_S:
        raise RuntimeError("old conservative heartbeat cache was not expired at reset")
    return before, after


def audit_primary_heartbeat_reset(out, terminal):
    """Audit an exact concurrent-list cancellation from ``out/logs/*.alog``.

    The heartbeat effective/read stamps and terminal ``at`` are C++ steady-clock
    values from the same primary app session. Alog-relative stamps are used only
    to associate CANCELLED, Runtime reset and LATCHED Helm publications within
    0.25 s in one log. Python clocks are never read or compared with C++ clocks.

    Require old_actual_read < terminal <= next_actual_read and old-cache age >2s.
    Equality is supported because frozen Runtime::message calls checkpoint(read)
    before applying the incoming frame; it is input-boundary ordering, not proof
    the reset physically preceded the read. No epsilon is applied to serialized
    stamps (heartbeat precision 7 decimals, terminal precision 9 in this fixture).
    A new queue-empty floor can precede reset, and the same poll can refresh HB
    before periodic status. Neither a >2s floor gap nor heartbeat_valid=0 is
    required. The host's existing 2s cache policy is unchanged.
    """
    if any(terminal.get(key) != expected for key, expected in (
        ("id", "concurrent-list"), ("action", "list"),
        ("status", "CANCELLED"), ("reason", "connection_reset"))):
        raise RuntimeError("terminal is not the concurrent-list connection-reset cancellation")
    terminal_at = _number(terminal.get("at"), "terminal C++ steady-clock stamp")
    matches = []
    errors = []
    for path in sorted((Path(out) / "logs").rglob("*.alog")):
        heartbeats = {}
        cancellations, resets, latched = [], [], []
        for line in path.read_text().splitlines():
            parts = line.split(None, 3)
            if (len(parts) != 4 or parts[1] not in _VARIABLES
                    or parts[2].split(":", 1)[0] != "iMAVLink"):
                continue
            alog_at = _number(parts[0], "primary diagnostic alog stamp")
            fields = _fields(parts[3])
            if parts[1] == "MAVMOOS_HEARTBEAT" and fields.get("present") == "true":
                heartbeat = _heartbeat(fields, alog_at, path)
                heartbeats.setdefault(_sample_key(heartbeat), heartbeat)
            elif (parts[1] == "MAVMOOS_PARAM_RESULT"
                  and fields.get("id") == "concurrent-list"
                  and fields.get("action") == "list"
                  and fields.get("status") == "CANCELLED"
                  and fields.get("reason") == "connection_reset"):
                if _number(fields.get("at"), "cancellation C++ steady-clock stamp") == terminal_at:
                    cancellations.append(alog_at)
            elif (parts[1] == "MAVMOOS_RUNTIME_STATUS"
                  and fields.get("reason") == _RESET
                  and fields.get("permission") == "false"):
                resets.append(alog_at)
            elif (parts[1] == "MAVMOOS_HELM_STATUS"
                  and fields.get("state") == "LATCHED"
                  and fields.get("reason") == _RESET):
                latched.append(alog_at)
        # Require all three publications to fit the existing tolerance; many
        # recurring status publications are allowed. Never join separate logs.
        witnesses = [(cancel, reset, helm) for cancel in cancellations
                     for reset in resets for helm in latched
                     if max(cancel, reset, helm) - min(cancel, reset, helm)
                     <= _PUBLICATION_TOLERANCE_S]
        if not witnesses:
            continue
        try:
            before, after = _bracket(list(heartbeats.values()), terminal_at)
        except RuntimeError as error:
            errors.append(str(error))
            continue
        witnesses.sort(key=lambda w: (max(w) - min(w), w))
        matches.append((before, after, witnesses[0], str(path)))
    if len(matches) != 1:
        detail = "; ".join(sorted(set(errors))) or "missing exact cancellation/reset/latched publications"
        if len(matches) > 1:
            detail = "ambiguous matching primary log contexts"
        raise RuntimeError(f"heartbeat cache reset audit failed: {detail}")
    before, after, witness, path = matches[0]
    equality = terminal_at == after["read_cpp_mono_s"]
    return {
        "result": "AUDITED_SESSION_RESET_CANCELLATION",
        "classification": "PRE_INGRESS_CONSERVATIVE_HEARTBEAT_CACHE_EXPIRED",
        "terminal_cpp_mono_s": terminal_at,
        "last_accepted_heartbeat": before,
        "next_accepted_heartbeat": after,
        "effective_heartbeat_gap_s": after["effective_cpp_mono_s"] - before["effective_cpp_mono_s"],
        "read_heartbeat_gap_s": after["read_cpp_mono_s"] - before["read_cpp_mono_s"],
        "age_since_last_effective_at_reset_s": terminal_at - before["effective_cpp_mono_s"],
        "next_actual_read_minus_reset_s": after["read_cpp_mono_s"] - terminal_at,
        "next_effective_floor_minus_reset_s": after["effective_cpp_mono_s"] - terminal_at,
        "read_boundary_ordering": "RUNTIME_INPUT_BOUNDARY_EQUALITY" if equality else "STRICT_READ_BRACKET",
        "cancel_reset_publication_alog_s": list(witness[:2]),
        "latched_helm_publication_alog_s": witness[2],
        "publication_alog_path": path,
        "publication_tolerance_s": _PUBLICATION_TOLERANCE_S,
        "complete_table_claimed": False,
        "note": "Pre-ingress conservative heartbeat cache expired; this is not a measured >2s packet/FC/link outage. The independent FC observer may continue receiving. No Python/C++ clock subtraction.",
    }
