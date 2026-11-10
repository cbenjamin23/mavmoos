"""Scoped cleanup, including pAntler children which create separate process groups."""
import os
from pathlib import Path
import shlex
import signal
import subprocess
import time
import importlib.util


def native_firmware_profile(source_root, ardupilot_root, manifest=None,
                            attestation=None, expected_profile='arducopter'):
    """Resolve test-only firmware identity from the explicitly selected source.

    This has no process/socket authority and provides no executable override.
    Fixtures retain their legacy checks when no manifest pair was selected.
    """
    path = Path(source_root).resolve() / 'tests/firmware_identity.py'
    spec = importlib.util.spec_from_file_location('_mavmoos_firmware_selection', path)
    if spec is None or spec.loader is None:
        raise ValueError('selected-source firmware adapter unavailable')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.load_selected_firmware(source_root, ardupilot_root, manifest,
                                         attestation, expected_profile)


def firmware_source_inputs(source_root, identity=None):
    """Inputs which a standalone fixture must retain alongside frozen binaries."""
    source = Path(source_root).resolve()
    paths = [source / name for name in (
        'tests/firmware_identity.py', 'tests/owned_processes.py',
        'src/libmavmoos_runtime/FirmwareDiscovery.h',
        'src/libmavmoos_runtime/FirmwareDiscovery.cpp',
        'src/libmavmoos_runtime/RuntimeFirmware.cpp',
        'src/libmavmoos_runtime/Runtime.h')]
    if identity:
        paths.extend(Path(identity[key]) for key in ('manifest_path', 'attestation_path', 'verifier_path'))
    return paths


def arducopter_gcs_source(ardupilot_root):
    """Find the one known Copter GCS filename in an already verified tree.

    This is a provenance input, never a substitute for firmware verification.
    ArduPilot renamed this source between the two selected profiles.
    """
    root = Path(ardupilot_root).resolve()
    choices = [root / 'ArduCopter' / name for name in
               ('GCS_MAVLink_Copter.cpp', 'GCS_Mavlink.cpp')]
    present = [path for path in choices if path.is_file() and not path.is_symlink()]
    if len(present) != 1:
        raise ValueError('expected one unambiguous regular Copter GCS source')
    return present[0]


def snapshot(pids=None, timeout=3):
    """Read one bounded inventory, optionally limited to known process IDs.

    A targeted query that completes with no matching IDs is an empty inventory;
    query errors/timeouts still raise and must never establish cleanup.
    """
    if not isinstance(timeout, (int, float)) or isinstance(timeout, bool) or not 0 < timeout <= 15:
        raise ValueError('invalid process snapshot timeout')
    command = ['ps', '-axww', '-o', 'pid=,ppid=,pgid=,command=']
    if pids is not None:
        selected = list(pids)
        if any(type(pid) is not int or pid <= 0 for pid in selected) or len(selected) > 8192:
            raise ValueError('invalid targeted process IDs')
        selected = sorted(set(selected))
        if not selected:
            return {}
        command = ['ps', '-ww', '-p', ','.join(map(str, selected)), '-o', 'pid=,ppid=,pgid=,command=']
    result = subprocess.run(command, capture_output=True, text=True, timeout=timeout)
    if result.returncode and not (pids is not None and result.returncode == 1 and
                                  not result.stdout.strip() and not result.stderr.strip()):
        raise subprocess.CalledProcessError(result.returncode, command, result.stdout, result.stderr)
    rows = {}
    for line in result.stdout.splitlines():
        parts = line.strip().split(None, 3)
        if len(parts) == 4:
            try:
                pid, parent, group = map(int, parts[:3])
            except ValueError:
                continue
            rows[pid] = {'pid': pid, 'parent': parent, 'group': group, 'command': parts[3]}
    if not rows and (pids is None or result.stdout.strip()):
        raise ValueError('process snapshot contains no parsed rows')
    return rows


def owned_records(process):
    # poll() may reap the original child. A reaped PID is no longer an identity:
    # never seed traversal from a replacement process which reused its number.
    live = process.poll() is None
    rows = snapshot()
    owned = {process.pid} if live else set()
    for _ in range(32):
        added = {pid for pid, row in rows.items() if row['parent'] in owned}
        if added.issubset(owned):
            break
        owned |= added
    # If pAntler exited unexpectedly, its children may already be reparented.
    # Only the exact owned mission argument can reconcile these known MOOS apps.
    args = process.args
    if isinstance(args, (list, tuple)) and len(args) > 1 and Path(args[0]).name == 'pAntler':
        mission = str(args[1])
        allowed = {'MOOSDB', 'iMAVLink', 'pHelmIvP', 'pLogger', 'uProcessWatch'}
        for pid, row in rows.items():
            try:
                tokens = shlex.split(row['command'])
            except ValueError:
                continue
            if tokens and Path(tokens[0]).name in allowed and mission in tokens:
                owned.add(pid)
    return [rows[pid] for pid in owned if pid in rows]


def stop_process_tree(process):
    if process is None:
        return []
    records = owned_records(process)
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGCONT)
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2)
    current = snapshot()
    for row in records:
        if row['pid'] in current and current[row['pid']]['command'] == row['command']:
            try:
                # Exact PID + unchanged argv keeps unrelated process groups out of scope.
                os.kill(row['pid'], signal.SIGKILL)
            except ProcessLookupError:
                pass
    remaining = snapshot()
    deadline = time.monotonic() + 0.5
    while any(row['pid'] in remaining and remaining[row['pid']]['command'] == row['command'] for row in records) and time.monotonic() < deadline:
        time.sleep(0.05)
        remaining = snapshot()
    return [row for row in records if row['pid'] in remaining and remaining[row['pid']]['command'] == row['command']]
